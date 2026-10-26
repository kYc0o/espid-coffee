# PCB_REQUIREMENTS.md — controller board for the Rancilio Silvia

Requirements for a dedicated KiCad PCB to replace the experimental build
(ESP32-S3-DevKitC-1 on an MRD076A screw-terminal breakout, with flying
relay/SSR/triac modules). Every requirement carries its reason, so it can be
weighed rather than copied. Companion documents: [`WIRING.md`](WIRING.md)
(current pin map, measured behaviour) and [`main/config.hpp`](main/config.hpp)
(the pin assignments the firmware actually uses).

Status words: **must** (the firmware or safety depends on it), **should**
(clear benefit, deviate only with a reason), **option** (fit the footprint,
populate later or never).

---

## 1. Scope and what this board replaces

| Today | Problem it causes |
|---|---|
| DevKitC-1 + MRD076A breakout, wires to modules | Bulky, loose connections (an I²C cable fault already cost a debugging session), no strain relief |
| On-module PCB antenna inside the steel case | Wi-Fi averaged −77 dBm, worst −97 dBm (Oct 2026); slower wakes |
| S3 internal pull-downs (~45 kΩ) on the switch inputs | Switch wires next to mains pick up pulses: up to 74 glitch wakes per night (`spurious_wakes`, 2026-10-24) |
| DevKit power LED, RGB LED, LDO | Draw that firmware cannot switch off in deep sleep |

The board must keep the **current firmware pin map** (§4) unless a change is
recorded in `config.hpp` and `WIRING.md` in the same commit.

---

## 2. Safety and mains (decide first)

- **must** — Galvanic isolation between mains and every logic net. The heater,
  pump and valve are switched mains loads; the ESP32 is touched through USB.
- **Decision for the next session:** mains switching **on this board** or on
  **external modules** (as now, and as CleverCoffee does).
  - External modules (recommended for a first revision): the PCB stays
    logic-only (5 V/3V3), mains never enters it, creepage is a non-issue,
    modules can be swapped. Cost: more wiring and connectors.
  - On-board: fewer parts in the case, but then **must** meet mains creepage
    and clearance (≥ 6 mm reinforced between mains and logic, slots under
    optocouplers), fuse the mains input, rate traces for the heater current
    (Silvia element ~1 kW at 230 V ≈ 4.5 A; size for ≥ 10 A), and place the
    triac/SSR heat on a heatsink outside the boiler's hot zone.
- **must** — Outputs default to OFF in every state where the MCU is not
  driving them: power-up, reset, brown-out, deep sleep (§6). The firmware
  already latches them low in sleep, but a pull-down is what protects the
  boot and brown-out windows.
- **must** — Keep the machine's original thermostats/thermal fuses in series
  with the element. The firmware's over-temperature guard is a second line,
  not the only one.
- **should** — Water-tank level input reserved (§5): a dry pump and an empty
  boiler are the remaining unguarded failure modes.

## 3. MCU, antenna, programming

- **must** — ESP32-S3 with **16 MB flash + 8 MB octal PSRAM** (N16R8). The
  partition table (A/B OTA + LittleFS) and the PSRAM-backed heap assume it.
  Octal PSRAM makes **GPIO26–37 unusable**.
- **must** — External antenna: use **ESP32-S3-WROOM-1U-N16R8** (U.FL) or a
  module/carrier whose U.FL path is enabled, with the antenna mounted
  **outside the steel case** (or in a gap). An external antenna already
  measured well in this location; the PCB antenna inside the case did not.
  If a PCB-antenna module is ever used, respect its copper keep-out.
- **must** — Native USB (GPIO19/20) on a USB-C connector for flashing and
  console (`/dev/cu.usbmodem…` via USB-Serial-JTAG). No USB-UART bridge is
  needed. Add ESD protection on D+/D−.
- **must** — BOOT (GPIO0) and EN/RESET buttons; EN RC (10 kΩ / 1 µF) per
  Espressif's reference.
- **must** — Never put a load-driving output on a strapping pin
  (GPIO0, 3, 45, 46): they toggle at boot.
- **should** — Header for the JTAG pins (GPIO39–42) unpopulated, for debug.

## 4. Pin map (keep — the firmware depends on it)

RTC GPIOs on the S3 are **GPIO0–21**; only those can wake the chip from
deep sleep or keep an RTC pull in sleep. Every wake-source input must be on one.

| Signal | GPIO | Dir | Board requirement | Reason |
|---|---|---|---|---|
| TSIC306 VDD | 2 | out | Direct to sensor VDD, 100 nF at the sensor | GPIO-powered so the firmware switches the sensor off in sleep |
| TSIC306 data (ZACwire) | 1 | in | Short run or twisted with GND; optional 100 Ω series | Push-pull sensor output, timing-critical |
| I²C SDA / SCL (OLED 0x3C, ABP2 0x28) | 8 / 9 | bidir | **4.7 kΩ to 3V3** each, once on the board | Internal pulls too weak for 400 kHz; idle-high bus costs nothing in sleep |
| Heater SSR | 47 | out | **10 kΩ pull-down**; driver able to source the SSR input current | OFF while the pin floats (boot, brown-out, sleep) |
| Valve relay | 16 | out | **10 kΩ pull-down** (new); transistor/driver if the relay coil is on board | Today it has none: a floating input could click the valve at boot |
| Pump triac gate (PSM) | 17 | out | **10 kΩ pull-down**; opto-triac LED resistor for ~10 mA | Gate must never fire while undriven |
| Pump zero-cross | 18 | in | Opto, **10 kΩ pull-up + 2.2 kΩ series + 100 nF to GND** (τ ≈ 220 µs) | Measured: without the RC the ZC ISR mis-triggers on opto noise |
| Brew switch | 5 | in | See §5 (switch inputs) | Deep-sleep wake source |
| Steam switch | 4 | in | See §5 | Deep-sleep wake source |
| HX711 DOUT 1 / 2 | 7 / 10 | in | Option (§7) | Wired scale |
| HX711 SCK (shared) | 21 | out | Option (§7); RTC pin so it can be held high in sleep | HX711 powers down with SCK high |

Do not use GPIO38/48 if the RGB LED of a DevKit is ever kept; on a bare
module they are free.

## 5. Machine switches (brew, steam, **hot water**)

All three are latching switches read by the firmware; the **hot-water switch
is new and handled only in software** (pump via PSM, valve closed, water out
of the wand — the firmware does not support it yet).

- **must** — Each input on an **RTC GPIO**. Brew = GPIO5, steam = GPIO4, and
  **hot water = GPIO6** (free, RTC-capable, not a strapping pin). Any of the
  three must be able to wake the machine.
- **must** — Active-high to 3V3 (as now) with an **external 10 kΩ pull-down**,
  **1 kΩ series resistor** to the pin and **100 nF to GND** at the pin
  (τ ≈ 1 ms), plus an ESD/TVS diode where the wire enters the board.
  Reason: the 45 kΩ internal pull-down plus metres of wire next to mains gave
  up to 74 glitch wakes in one night; a stiff line also lets the firmware stop
  keeping the RTC peripheral domain on in sleep.
- **must** — Confirm which switch poles carry the 3V3 signal and that **no
  mains** is on those poles. The Silvia's rockers are mains switches with
  lamps; if a pole cannot be dedicated to logic, read it through an
  **optocoupler** with a mains-rated input instead.
- **should** — Locking connectors for the switch harness.

## 6. Power and deep-sleep consumption

The machine stays mains-powered 24/7 and sleeps most of the day; the board's
standby draw is now the dominant constant load.

- **must** — 5 V from an isolated mains module with **low no-load loss**
  (≤ 0.1 W class, e.g. HLK-PM01/-5M05 family), input fuse and varistor if it
  sits on this board.
- **must** — 3V3 regulator with **low quiescent current** (tens of µA, not
  the mA of an AMS1117) able to supply **≥ 500 mA peaks** (Wi-Fi TX), with
  bulk capacitance close to the module (≥ 22 µF + 100 nF).
- **must** — **No always-on LEDs.** A power LED, if wanted, behind a solder
  jumper or a GPIO.
- **should** — The zero-cross pull-up draws while the opto pulses (tens of µA
  at 10 kΩ). Either feed it from a GPIO that is low in sleep, or accept it.
- **should** — Peripheral rail switch (P-MOSFET from 3V3, GPIO-controlled,
  default off by pull-up) for OLED + ABP2 + HX711, so sleep can cut them
  entirely. Today the OLED is put into sleep by command and the ABP2 idles on
  its own; a switch removes the dependency. Keep I²C pull-ups on the switched
  side or add series resistors so a powered-down device is not back-fed.
- **option** — Current-sense shunt + test points on the 3V3 rail to measure
  sleep current.

Firmware facts the hardware should not break: in deep sleep the heater,
valve, pump-gate and TSIC-power pins are latched low; switch pins use RTC
pull-downs; I²C and ZC are released (see WIRING.md, "Pin states in deep
sleep").

## 7. Scale: wired option, Bluetooth alternative

- **option** — Keep the **two-HX711 wired scale** footprint (shared SCK on
  GPIO21, DOUT on GPIO7/GPIO10, 3V3 supply, connector per load cell). The
  3D-printed holders are not trusted yet, so it stays unpopulated; the
  firmware's scale driver exists and is disabled.
- **must if fitted** — HX711 power-down needs **SCK held high** (RTC pin,
  done) or the rail switch above; otherwise ~1.5 mA each, even with the scale
  disabled.
- A **Bluetooth scale** needs no hardware beyond a good antenna (§3).

## 8. Reserved for future features

Route to a header or test pad so a later revision does not need a respin:

| Feature | Suggested GPIO | Note |
|---|---|---|
| Water-tank level (dry-run guard) | 12 | Digital or capacitive sensor; highest-value safety addition |
| Flow meter (pulse) | 13 | PCNT input, pull-up + RC |
| Rotary encoder A/B + push | 14 / 15 / 11 | Local UI without the web page |
| Status LED | 38 | Off in sleep; GPIO39–42 stay free for JTAG |

Bring every remaining free GPIO (not strapping, not 19/20, not 26–37) to a
spare header.

## 9. Mechanical and environment

- **must** — Rated for the inside of the case: ≥ 85 °C parts, keep away from
  the boiler and the steam path; vibration from the vibratory pump → locking
  connectors (JST-XH/PH or screw terminals with ferrules) and strain relief.
- **should** — Conformal coating on the logic area (steam, condensation).
- **should** — Mounting holes matching an existing fixing point or a printed
  bracket; OLED and ABP2 on cables, not on the board.
- **should** — Silkscreen every connector with signal names that match
  `WIRING.md`.

## 10. Licensing note

[CleverCoffee hardware](../clevercoffee-hardware) is **CC BY-NC-SA 4.0**. A
board derived from its files inherits non-commercial share-alike terms; this
repository is MIT (pending). Decide whether the new board starts from scratch
(recommended if the licences should stay compatible) or is explicitly a
CleverCoffee derivative.

## 11. Open questions for the design session

1. Mains on board or external modules (§2)?
2. Which switch poles carry logic, or optocoupled inputs (§5)?
3. Module: WROOM-1U-N16R8 directly, or a carrier with U.FL (§3)?
4. Peripheral rail switch: yes/no (§6)?
5. Connector family and board outline/mounting in the case (§9).
6. Wired scale footprint kept, or Bluetooth only (§7)?

---

## Prompt for the next session

> We're designing a KiCad PCB for my ESPID-Coffee firmware (Rancilio Silvia,
> ESP32-S3 N16R8). Read `AGENTS.md`, `WIRING.md` and `PCB_REQUIREMENTS.md` in
> the ESPID-Coffee repo first; `PCB_REQUIREMENTS.md` lists every requirement
> with its reason, and `main/config.hpp` is the source of truth for pins. The
> CleverCoffee KiCad projects in `../clevercoffee-hardware` are a reference
> only (CC BY-NC-SA). Start with the open questions in §11 of
> `PCB_REQUIREMENTS.md`, then propose a block diagram and a schematic plan
> before drawing anything. Any pin change must also update `config.hpp` and
> `WIRING.md`.
