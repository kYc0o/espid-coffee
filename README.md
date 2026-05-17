# ESPID-Coffee

ESP-IDF (v6.0.x) firmware for the Rancilio Silvia espresso machine, rewriting
[clevercoffee](https://github.com/rancilio-pid/clevercoffee) on native FreeRTOS
with modern C++17, TDD on the ESP-IDF Linux target, and A/B OTA.

## Target hardware

- **MCU**: ESP32-S3 (Sparkle XH-S3E N16R8 — 16 MB flash, 8 MB octal PSRAM)
- **Temp sensor**: TSIC 306 on GPIO 1 (ZACwire 1-wire)
- **Display**: SSD1306 0.96" OLED on I²C (SCL=GPIO 0, SDA=GPIO 2, addr 0x3C)
- **Brew button**: GPIO 9 (active-low, internal pull-up)

## Build & flash

```sh
. ~/git/espressif/esp-idf-v6.0.1/export.sh
idf.py set-target esp32s3
idf.py build flash monitor
```

## Roadmap

| Phase | Scope |
|-------|-------|
| 0 | Toolchain + partition table + boot OK |
| 1 | HAL interfaces + gtest harness on Linux target |
| 2 | TSIC 306 (RMT) → OLED °C display |
| 3 | Brew button + timer state machine |
| 4 | LittleFS + Wi-Fi SoftAP provisioning |
| 5 | Minimal web UI (brew-time config) + FS upload |
| 6 | Safety task scaffold + A/B OTA |

Deferred: PID/SSR, pump PSM, HX711 scale, MQTT, TLS.

## License

MIT (pending — see LICENSE in a future commit).
