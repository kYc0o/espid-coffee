# WIRING.md — ESPID-Coffee hardware wiring reference

Physical wiring for the **Rancilio Silvia** build. Single source of
truth for GPIO assignments is [`main/config.hpp`](main/config.hpp);
this file is the human-facing companion for bench and in-machine work.

> Keep this in sync with `config.hpp`. If a pin changes there, change
> it here in the same commit.

---

## 1. Platform

- **MCU:** ESP32-S3-WROOM-1 **N16R8** (16 MB flash, 8 MB **octal PSRAM**).
- **Carrier:** ESP32-S3-DevKitC-1 on an **MRD076A screw-terminal
  breakout** (every S3 pin fanned out to a labelled terminal).
- **Not used:** the CleverCoffee PCB's ESP module socket. That socket
  is a WROOM-32 (DevKitC-v4) footprint — its power rails collide with
  the S3 pinout (PCB `5V` would land on S3 `GPIO13`, and signals fall
  on the N16R8 PSRAM pins). Wire peripherals directly to the S3 via the
  breakout instead. You may still reuse the PCB's relay/SSR modules and
  connectors, driven from the S3 GPIOs below.

### Off-limits GPIOs (do not wire anything here)

| Pins | Reason |
|------|--------|
| `GPIO26–37` | SPI flash + octal PSRAM (N16R8). Not usable. |
| `GPIO0`, `GPIO3`, `GPIO45`, `GPIO46` | Strapping — pulse at boot; never use for relays/SSR. |
| `GPIO19`, `GPIO20` | USB D-/D+ (native USB-JTAG/CDC — used by `/dev/cu.usbmodem`). |
| `GPIO43`, `GPIO44` | UART0 console (TX/RX). |
| `GPIO38`/`GPIO48` | On-board RGB LED on some DevKitC-1 revisions. |

---

## 2. Pin map

| Function | S3 GPIO | Dir | Pull resistor | Notes |
|---|---|---|---|---|
| Temp sensor TSIC306 VDD | `GPIO2` | out | **none** | GPIO-sourced power; IO2 is physically adjacent to IO1 on MRD076A. |
| Temp sensor TSIC306 (ZACwire) | `GPIO1` | in | **none** | Sensor drives push-pull; 3.3 V signal. |
| OLED + pressure **I²C SDA** | `GPIO8` | bidir | **4.7 kΩ → 3V3** | Shared bus. See §3. |
| OLED + pressure **I²C SCL** | `GPIO9` | bidir | **4.7 kΩ → 3V3** | Shared bus. See §3. |
| Heater **SSR** | `GPIO47` | out | **10 kΩ → GND** | Pull-down keeps heater OFF at boot. Isolated SSR. |
| Brew switch | `GPIO5` | in | internal pull-**down** | Switch sources 3V3 when closed → active-high. |
| Steam switch | `GPIO4` | in | internal pull-**down** | Switch to 3V3; `kPulldownEnable = true`. |
| Valve / 3-way solenoid | `GPIO16` | out | none | Open during brew only. Closed during startup fill and post-steam refill, so the pump can only push water into the boiler (through the check valve). |
| Pump **neutral SSR** | `GPIO15` | — | — | **Removed** — pump neutral now wired directly to mains. `kSsrEnabled = false`. |
| Pump **PSM** triac gate | `GPIO17` | out | none | To dimmer/triac gate. Pulse-skip (integral-cycle) firing at the zero-cross. |
| Pump **zero-cross** input | `GPIO18` | in | **10 kΩ → 3V3** pull-up + **2.2 kΩ series + 100 nF → GND** RC low-pass | Opto ZC, ~100 Hz at 50 Hz mains. RC τ ≈ 220 µs (f_c ≈ 720 Hz). S3 pin glitch filter also on. |
| Scale HX711 #1 **DOUT** | `GPIO7` | in | none | HX711 drives push-pull. |
| Scale HX711 #2 **DOUT** | `GPIO10` | in | none | Second load cell. |
| Scale HX711 **SCK** (shared) | `GPIO21` | out | none | Common clock to both HX711. |

Free for future use: `GPIO2`, `GPIO6`, `GPIO11`, `GPIO12`, `GPIO13`,
`GPIO14`, `GPIO15` (freed by SSR removal), `GPIO39`, `GPIO40`, `GPIO41`,
`GPIO42`.

Deferred (not wired yet): hot-water switch, water-tank level sensor,
rotary encoder, status/brew LED, flow meter, power switch, E-trigger.

---

## 3. I²C bus (OLED SSD1306 + Honeywell ABP2 pressure)

- One bus, two devices: **SDA `GPIO8`**, **SCL `GPIO9`**, 400 kHz.
- Addresses: OLED `0x3C`, ABP2 pressure `0x28`.
- **Pull-ups: 4.7 kΩ from each of SDA and SCL to 3V3.** Only add these
  if your OLED/ABP2 breakout boards don't already carry them (most
  do — check with a multimeter for ~4.7 kΩ SDA/SCL→3V3 before adding).
  The S3 internal pull-ups (~45 kΩ) are **too weak** for 400 kHz.
- Power the I²C devices from **3.3 V**, not 5 V.

---

## 4. Pull-up / pull-down summary

| Signal | What to fit | Why |
|---|---|---|
| I²C SDA, SCL | 4.7 kΩ to **3V3** each | Bus idle-high; internal pulls too weak. |
| Heater SSR (`GPIO47`) | 10 kΩ to **GND** | Guarantee heater OFF while the S3 boots (pin is Hi-Z). |
| Steam switch (`GPIO4`) | internal pull-**down** | Switch sources 3V3 when closed → hold low when open. |
| Pump zero-cross (`GPIO18`) | 10 kΩ pull-**up** to 3V3 + **2.2 kΩ series + 100 nF to GND** RC | Opto is open-collector; RC (τ ≈ 220 µs) + S3 pin glitch filter tame opto noise. |
| Brew switch (`GPIO5`) | internal pull-**up** | Config uses active-low to GND. |
| Valve / Pump gate outputs | none | Push-pull outputs. |
| TSIC temp, HX711 DOUT/SCK | none | Push-pull sensor/ADC outputs. |

Rule of thumb: **inputs to 3V3 → pull-down; inputs to GND → pull-up.**
The S3 has usable internal pull-up/down on all the GPIOs above; only
I²C (speed) and the heater SSR (boot safety) need external resistors.

---

## 5. External hardware needed (the breakout is logic-level only)

- 5 V → 3.3 V is on-board (DevKitC regulator). Provide **5 V** from a
  small isolated PSU (or the S3's USB-C on the bench).
- **Isolated relay module** for the 3-way valve (`GPIO16`).
- **Solid-state relay (SSR)** for the heater (`GPIO47`), with the
  10 kΩ pull-down on its control line.
- **Triac dimmer + zero-cross detector** for the pump PSM
  (`GPIO17` gate, `GPIO18` ZC). Fit the **2.2 kΩ + 100 nF** RC
  low-pass on the ZC line (see §2). Pump neutral is wired directly to
  mains (the `GPIO15` neutral SSR was removed).
- **2× HX711** amplifier boards, one per drip-tray load cell.

---

## 6. Safety notes

- **Mains isolation:** all mains switching stays on the isolated
  relay/SSR/triac modules. The breakout carries logic-level only.
- **Boot-strap state:** at power-on the S3 GPIOs are high-impedance.
  Never place the heater SSR or the pump gate on a strapping pin, and
  keep the heater SSR pull-down fitted so it cannot self-trigger.
- **Dry-run:** a water-tank sensor is not yet wired — until it is,
  don't leave the machine heating/pumping unattended.

---

## 7. Reference: MRD076A terminal labels

The adapter fans the DevKitC-1 out to two screw-terminal rows. Use the
`IOxx` labels directly — they match the S3 GPIO numbers in the table
above (e.g. terminal `IO8` = `GPIO8`). Power terminals: `3V3`, `5V`,
`GND`. Avoid the off-limits terminals listed in §1.
