# ESPID-Coffee — Architecture

This document is the **single source of truth** for the firmware's
structure. It captures the system in its current shape and the
near-term planned subsystems so any contributor (or AI agent) can
locate themselves quickly. Keep it updated alongside every commit
that introduces, removes, or moves a subsystem.

> Companion: [`AGENTS.md`](AGENTS.md) — process / conventions / build &
> commit etiquette. Read both.

---

## 1. One-paragraph summary

ESPID-Coffee is an ESP-IDF v6.0.x firmware for the Rancilio Silvia
espresso machine, replacing the Arduino-framework
[CleverCoffee](https://github.com/rancilio-pid/clevercoffee). It runs
on an ESP32-S3 (N16R8). Modern C++17, no exceptions/RTTI,
`Result<T>`/`Status` for error propagation, **strict bisectability**,
TDD via GoogleTest on the ESP-IDF Linux target for all pure-logic
components, and atomic Conventional-Commits commits.

---

## 2. Target hardware

| Item              | Detail                                                    |
| ----------------- | --------------------------------------------------------- |
| MCU               | ESP32-S3 (Sparkle XH-S3E N16R8 — 16 MB flash, 8 MB octal PSRAM) |
| Toolchain         | ESP-IDF v6.0.1 (`/Users/facosta/git/espressif/esp-idf-v6.0.1`) |
| Serial port       | `/dev/cu.usbmodem5ABA0676261`                             |
| Temperature       | TSIC 306, ZACwire 1-wire on **GPIO 1**                    |
| Display           | SSD1306 0.96″ 128×64 OLED, I²C addr 0x3C @ 400 kHz, **SDA=GPIO 8 / SCL=GPIO 9**, rotated 180° |
| Brew button       | Active-low, HW-debounced (RC + Schmitt) on **GPIO 5**, internal pull-up enabled as fallback |
| Heater SSR        | _planned_ — GPIO 15 (slow-PWM, 1 s period)                |
| Pump / valve SSR  | _planned_ — TBD                                           |
| Pressure sensor   | _planned_ — TBD                                           |
| Load cell (scale) | _planned_ — HX711, TBD                                    |

Pin allocation rationale: GPIO 0/3/45/46 are strapping; 19/20 = USB-JTAG;
26-32 = octal PSRAM/flash on N16R8; 33-37 = JTAG. We avoid all of them.

---

## 3. Source tree

```
ESPID-Coffee/
├── CMakeLists.txt                # top-level; patches managed nixy4/u8g2
├── partitions.csv                # OTA layout (factory + ota_0 + ota_1 + nvs + spiffs)
├── sdkconfig{,.defaults}         # ESP-IDF config
├── main/
│   ├── CMakeLists.txt
│   ├── app_main.cpp              # entry: spawns sensor / display / button tasks
│   └── config.hpp                # compile-time defaults (namespace espid::config)
├── components/
│   ├── hal_cpp/                  # header-only HAL contracts + Result<T>/Status
│   │   └── include/hal_cpp/
│   │       ├── result.hpp
│   │       ├── i_system_clock.hpp
│   │       ├── i_gpio.hpp
│   │       ├── i_i2c_bus.hpp
│   │       ├── i_zacwire_capture.hpp
│   │       ├── i_display.hpp
│   │       └── i_network.hpp     # Wi-Fi HAL contract
│   ├── tsic306/                  # RMT-based ZACwire driver + pure decoder
│   ├── ssd1306/                  # U8g2-backed IDisplay implementation
│   ├── wifi/                     # esp_wifi INetwork impl (STA + AP fallback)
│   ├── pid/                      # header-only discrete PID controller
│   └── brew/                     # debounced brew button + shot timer (header-only)
├── host_tests/                   # GoogleTest harness, ESP-IDF Linux target
│   ├── CMakeLists.txt
│   ├── test_tsic306.cpp
│   ├── test_pid.cpp
│   └── test_brew.cpp
└── scripts/
    └── bisect_verify.sh          # walks N commits, runs build+host-tests on each
```

---

## 4. Layering

```mermaid
flowchart TB
    subgraph App["main/ — application"]
        APP_MAIN["app_main.cpp"]
        CFG["config.hpp"]
    end

    subgraph Components["components/ — domain logic"]
        TSIC["tsic306"]
        SSD["ssd1306"]
        BREW["brew"]
        PID["pid"]
      WIFI["wifi (STA/AP fallback impl)"]
        HEAT["heater (planned)"]
        HTTPSVC["webui (planned)"]
        NVS["settings/NVS (planned)"]
    end

    subgraph HAL["components/hal_cpp — interfaces"]
        ICLK["IClock"]
        IGPIO["IGpio"]
        II2C["II2cBus"]
        IZAC["IZacwireCapture"]
        IDSP["IDisplay"]
        INET["INetwork"]
        IHEAT["IHeater (planned)"]
    end

    subgraph IDF["ESP-IDF v6.0.1"]
        FRTOS["FreeRTOS"]
        DRV["driver/* (gpio, i2c, rmt)"]
        ESPWIFI["esp_wifi + esp_netif"]
        HTTPD["esp_http_server"]
        ESPNVS["nvs_flash"]
        TIMER["esp_timer"]
        LOG["esp_log"]
    end

    APP_MAIN --> CFG
    APP_MAIN --> TSIC
    APP_MAIN --> SSD
    APP_MAIN --> BREW
    APP_MAIN -. planned .-> PID
    APP_MAIN -. planned .-> WIFI
    APP_MAIN -. planned .-> HEAT

    TSIC --> IZAC
    TSIC --> ICLK
    SSD --> IDSP
    SSD --> II2C
    BREW --> ICLK
    WIFI -. impl .-> INET
    HEAT -. impl .-> IHEAT

    IZAC --> DRV
    IDSP --> DRV
    II2C --> DRV
    INET --> ESPWIFI
    INET --> HTTPD
    APP_MAIN --> FRTOS
    APP_MAIN --> TIMER
    APP_MAIN --> LOG
    NVS --> ESPNVS
```

Rules:

- `components/hal_cpp/` is **header-only** and contains only interfaces.
  Never put ESP-IDF symbols in HAL headers (they must compile against
  the Linux target for unit tests).
- Concrete drivers depend on HAL interfaces, never on each other.
- Pure-logic components (`pid`, `brew`, `tsic306` decoder half) have **zero**
  ESP-IDF includes — they are exercised by `host_tests/` on Linux.
- `main/` owns wiring (tasks, snapshots, lifetime), not domain logic.

---

## 5. Runtime — tasks, cores, priorities

| Task          | Core | Priority | Stack  | Period   | Owner                              |
| ------------- | ---- | -------- | ------ | -------- | ---------------------------------- |
| `sensor`      |  1   |    5     | 4096 B | event    | TSIC capture + decode → snapshot   |
| `display`     |  1   |    4     | 4096 B | 200 ms   | OLED render from snapshot          |
| `button`      |  1   |    3     | 3072 B |  10 ms   | Brew button debounce → shot timer  |
| `heartbeat`   | idle | (main)   |  —     | 5000 ms  | `app_main`, proof-of-life log      |
| `control`     |   ?  |    ?     |   ?    |  200 ms  | _planned_ — PID loop driving heater|
| `wifi-evt`    |  0   | esp-wifi | (IDF)  | event    | _planned_ — esp_event task         |
| `httpd`       |  0   |    5     | 4096 B | event    | _planned_ — esp_http_server worker |

All values come from `main/config.hpp`. Treat that file as the only
place where stack/prio/core/period literals live in firmware code.

---

## 6. Shared state — snapshot pattern

Producers publish into struct-of-`std::atomic<>` snapshots; consumers
read with `memory_order_relaxed` for the payload after observing a
release-published `valid` flag. **No queues, no mutexes** for the
hot path — values are 32-bit naturally aligned on ESP32-S3.

Current snapshots (defined in `main/app_main.cpp`):

```cpp
struct Snapshot {        // produced by sensor_task
    std::atomic<float>         temp_c;
    std::atomic<std::uint16_t> raw;
    std::atomic<bool>          valid;   // release/acquire barrier
};

struct ShotSnapshot {    // produced by button_task
    std::atomic<std::uint32_t> elapsed_ms;
    std::atomic<bool>          running;
};
```

Planned additions: `HeaterSnapshot { duty_pct, output_on }`,
`NetSnapshot { state, ip[16] }`.

---

## 7. Data flow

```mermaid
flowchart LR
    SENSOR["TSIC 306<br/>(GPIO 1, ZACwire)"]
    BUTTON["Brew button<br/>(GPIO 5)"]
    HEATER["Heater SSR<br/>(GPIO 15, planned)"]

    SENSOR -->|RMT frames| TSIC_DRV[tsic306::Driver]
    TSIC_DRV --> TSIC_DEC[tsic306::Decoder]
    TSIC_DEC -->|temp_c| SNAP[Snapshot]

    BUTTON -->|level| BBTN[brew::BrewButton]
    BBTN -->|Pressed event| TIMER[brew::ShotTimer]
    TIMER -->|elapsed_ms, running| SHOTSNAP[ShotSnapshot]

    SNAP --> DISP[display_task → OLED]
    SHOTSNAP --> DISP

    SNAP -. planned .-> PIDCTRL["pid::Controller<br/>(control_task)"]
    PIDCTRL -. planned .-> HEATER_DRV["heater (slow-PWM)"]
    HEATER_DRV -. planned .-> HEATER

    SNAP -. planned .-> HTTPD["webui /api/status"]
    SHOTSNAP -. planned .-> HTTPD
```

---

## 8. Display states

```mermaid
stateDiagram-v2
    [*] --> Splash
    Splash --> Idle: 2 s elapsed
    Idle --> Brewing: brew button pressed (timer.start)
    Brewing --> Idle: brew button pressed (timer.stop)

    note right of Splash : "ESPID-Coffee"<br/>"v0.0.4"
    note right of Idle : Big temperature<br/>Small shot footer (0.0 s)<br/>Wi-Fi badge in header
    note right of Brewing : Big shot timer<br/>Small temperature footer<br/>Wi-Fi badge in header
```

---

## 9. Error model

All driver / init APIs return `espid::hal_cpp::Result<T>` (value-or-error)
or `Status` (void-or-error). Both are `[[nodiscard]]`. Errors are
finite — see `Error` enum in
[`components/hal_cpp/include/hal_cpp/result.hpp`](components/hal_cpp/include/hal_cpp/result.hpp).
Add new variants there before introducing them at call sites.

Convention: non-critical subsystems log a warning and continue
headless on init failure (e.g. OLED). Critical subsystems halt with
an `ESP_LOGE` + busy-loop (e.g. TSIC driver).

---

## 10. Configuration

[`main/config.hpp`](main/config.hpp) is the **single boot-time
configuration namespace** (`espid::config::*`). Everything is
`constexpr` — no macros, no globals. Sections in current use: `tsic`,
`oled`, `brew`. Stubs ready to be wired: `pid`, `heater`, `net`,
`mqtt`.

When the embedded web UI lands, runtime-mutable values will be
shadowed from NVS in a separate `settings` component; `config::`
remains the compile-time fallback.

---

## 11. Build, flash, test

Day-to-day commands (source ESP-IDF once per shell, then use `idf.py`
normally):

```sh
# Prepare shell (once per terminal session)
get_idf

# Build firmware
idf.py build

# Flash + monitor
idf.py -p /dev/cu.usbmodem5ABA0676261 flash monitor

# Host unit tests (Linux target)
cmake -S host_tests -B host_tests/build && cmake --build host_tests/build \
  && ctest --test-dir host_tests/build --output-on-failure

# Bisect-green sanity check across the last N commits
scripts/bisect_verify.sh -n 5
```

If `get_idf` alias is unavailable in your shell:

```sh
. ~/git/espressif/esp-idf-v6.0.1/export.sh
```

---

## 12. Roadmap

| Phase | Subsystem                            | Status   | Notes |
| ----- | ------------------------------------ | -------- | ----- |
| 0     | ESP-IDF scaffold + partition table   | ✅ done  | `44fc917` |
| 1     | HAL contracts + `Result<T>`/`Status` | ✅ done  | `100b673`, `5df06cc` |
| 2a    | TSIC 306 decoder (host tests)        | ✅ done  | `aa3a9fc` |
| 2b    | TSIC 306 RMT driver + sensor_task    | ✅ done  | `968e86f`, `42480fb`, `8d8834e` |
| 2c    | SSD1306 OLED + display_task          | ✅ done  | `b019316`, `fc88319`, `df4f6ce` |
| 2c'   | Rotation option (180°)               | ✅ done  | `6fe45f1`, `524b1e2` |
| 2d-1  | PID controller (host tests)          | ✅ done  | `27f1e91` |
| 2d-2  | Brew button + shot timer (host tests)| ✅ done  | `b398854` |
| 2d-3  | Brew button wiring + OLED shot view  | ✅ done  | `653fe47` |
| 2e    | `main/config.hpp` (single SoT)       | ✅ done  | `d30bb14` |
| 2f    | Boot splash                          | ✅ done  | `074797b` |
| 3a    | **`INetwork` HAL contract**          | ✅ done  | `ede75a0` |
| 3b    | Wi-Fi STA + AP-fallback impl         | ✅ done  | `d8a3191` |
| 3c    | Captive-portal credential capture    | ⏳ planned |     |
| 3d    | `app_main` wiring + OLED Wi-Fi glyph | 🚧 active | this commit |
| 4a    | `IHeater` HAL + slow-PWM SSR driver  | ⏳ planned |     |
| 4b    | `control_task` running `pid::Controller` | ⏳ planned |  |
| 5a    | Embedded web UI scaffold (`esp_http_server` + static assets) | ⏳ planned | |
| 5b    | `/api/status`, `/api/config` JSON    | ⏳ planned |     |
| 6     | `settings` component (NVS-backed)    | ⏳ planned |     |
| 7     | MQTT bridge (optional, Home Assistant Auto Discovery) | ⏳ planned | |
| 8     | Pump PSM / valve control             | ⏳ planned |     |
| 9     | HX711 scale integration              | ⏳ planned |     |
| 10    | Safety watchdog + A/B OTA            | ⏳ planned |     |

Explicitly deferred: TLS, pressure sensor, Bluetooth.

---

## 13. Glossary

- **Bisect-green** — every commit on `main` builds AND passes host
  tests in isolation. Enforced by `scripts/bisect_verify.sh`.
- **Snapshot** — struct-of-atomics published by a producer task and
  read lock-free by consumers; `valid` flag uses release/acquire as
  the publication barrier.
- **Header-only component** — `idf_component_register(INCLUDE_DIRS …)`
  with no SRCS. Consumed via `REQUIRES`.
- **Slow-PWM** — software-toggled output with a period in the
  hundreds-of-ms to seconds range, suitable for solid-state relays
  driving resistive loads.
