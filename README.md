# ESPID-Coffee

ESP-IDF (v6.0.x) firmware for the Rancilio Silvia espresso machine, rewriting
[clevercoffee](https://github.com/rancilio-pid/clevercoffee) on native FreeRTOS
with modern C++17, TDD on the ESP-IDF Linux target, and A/B OTA.

## Target hardware

- **MCU**: ESP32-S3 (Sparkle XH-S3E N16R8 — 16 MB flash, 8 MB octal PSRAM)
- **Temp sensor**: TSIC 306 on GPIO 1 (ZACwire 1-wire)
- **Display**: SSD1306 0.96" OLED on I²C (SDA=GPIO 8, SCL=GPIO 9, addr 0x3C, rotated 180°)
- **Brew button**: GPIO 5 (active-low, hardware-debounced, internal pull-up fallback)

## Documentation

- [`ARCHITECTURE.md`](ARCHITECTURE.md) — full system schematic, layering, tasks, data flow, roadmap.
- [`AGENTS.md`](AGENTS.md) — handoff guide for new contributors / AI agents.

## Build & flash

```sh
. ~/git/espressif/esp-idf-v6.0.1/export.sh
idf.py set-target esp32s3
idf.py build flash monitor
```

## Roadmap

See [`ARCHITECTURE.md`](ARCHITECTURE.md#12-roadmap) for the live phase table.

## License

MIT (pending — see LICENSE in a future commit).
