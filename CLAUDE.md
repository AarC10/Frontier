# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Layout

All commands below must be run from the **repo root** (`Frontier/`), not this app directory.

```
apps/marshal/       — Marshal flight computer firmware (this app)
apps/outlaw/        — Outlaw LoRa GPS tracker firmware
apps/hunter/        — Hunter LoRa receiver firmware
boards/marshal/     — Board definition for STM32G474CC
boards/outlaw_gen3/ — Board definition for Outlaw Gen 3 (STM32L151Xb)
boards/hunter_gen2/ — Board definition for Hunter Gen 2 (STM32L151Xb)
include/core/       — Public C++ headers for all shared libraries
lib/                — Shared library implementations (core modules)
drivers/            — Out-of-tree Zephyr drivers (MS5611 baro, RDA5807M radio)
```

## Build & Flash

```bash
just marshal                        # Build (from repo root)
just sflash marshal                 # Flash via ST-Link
just jflash marshal                 # Flash via J-Link
just clean marshal                  # Delete builds/marshal/

# Equivalent west command
west build -b marshal apps/marshal -p auto --build-dir builds/marshal
```

Alternative configuration:
- `debug.conf` — debug overlay (extra logging)

## Hardware: Marshal Board (STM32G474CC)

- **CPU:** ARM Cortex-M4 (clock TBD — currently 80 MHz, will be tuned down after full system is fleshed out)
- **SPI1 (sensor bus):** MS5611 barometer, LSM6DSV16X IMU (240 Hz ODR)
- **SPI2 (storage bus):** W25Q128 128 Mb SPI-NOR flash
  - `nvs_partition` (16 KB) — NVS-backed settings
  - `raw_partition` (12 MB) — binary flight log
  - `fat_partition` (4 MB) — FAT export for USB mass storage
- **ADC1:** VBAT + VCC monitoring; **ADC5:** pyro ILM channels
- **GPIO:** Status LED (PA15), Buzzer (PB2), pyro fire/sense lines
- **USART2:** Debug UART at 115200 baud (PA2/PA3)

## Architecture

### `src/main.cpp`
Entry point. Initialises all peripherals, checks battery lockout via `FlightComputerSettings::minBatteryMv()`, spawns two background Zephyr threads (baro @ 25 Hz, voltage @ 1 Hz), and runs the status LED blink loop. `FlightStateMachine`, `FlightLogger`, and `PyroController` integration are present but commented out (in-progress).

### Core library (`include/core/` + `lib/`)

| Module | Header | Purpose |
|---|---|---|
| `FlightStateMachine` | `flight/FlightStateMachine.h` | State machine: PAD→BOOST→COAST→APOGEE→DESCENT→LANDED, driven by accel magnitude and barometric altitude |
| `FlightLogger` | `flight_logger/FlightLogger.h` | ISR-safe queue → writer thread → 256-byte page writes to `raw_partition` |
| `FlightExporter` | `flight_logger/FlightExporter.h` | Converts raw partition to FAT for USB mass storage export |
| `Barometer` / `Imu` / `VoltageMonitor` | `sensors/` | Thin wrappers around Zephyr sensor API |
| `PyroController` | `pyro/PyroController.h` | Dual-deploy / drogue-only / main-only ejection logic |
| `FlightComputerSettings` | `settings/FlightComputerSettings.h` | NVS-backed persistent config (arm altitude, deploy mode, main alt, min battery mV, flight counter) |

### Device Tree aliases (used in `main.cpp`)
`barometer`, `imu`, `vbat_sensor`, `vcc_sensor`, `led`, `buzzer` — all defined in `boards/marshal/marshal.dts`.

## In-Progress Features (see `docs/CHECKLIST.md`)

- ADC VBAT/VCC readings not yet validated
- `FlightLogger` / `FlightExporter` integration (code present, commented out)
- USB mass storage (MSC class disabled in `prj.conf`)
- PyroController hookup (continuity check + fire logic)
- Buzzer error/success tones

## Other Apps

### Outlaw (LoRa GPS Tracker — `apps/outlaw/`, board: `outlaw_gen3`)
Dual-mode LoRa tracker running on STM32L151Xb. Switches between transmitter and receiver roles via DIP switches. In TX mode, acquires GNSS fixes and broadcasts location over LoRa (SX1276) at 5-second intervals. Supports TDMA clock synchronization (PPS from GNSS), licensed (433 MHz) and unlicensed (903 MHz) bands, and NVS-backed settings (node ID, callsign, frequency). Has a shell for runtime config.

### Hunter (LoRa Receiver — `apps/hunter/`, board: `hunter_gen2`)
Receive-only counterpart to Outlaw, also on STM32L151Xb. Uses an SX1262 modem. Boots straight into an infinite receive loop — no state machine, shell disabled. Logs received packets with RSSI/SNR over UART.

Both Outlaw and Hunter share the same core `LoraTransceiver`, `GnssReceiver`, `TdmaClock`, and `Settings` abstractions from `include/core/`.

## Code Style

`.clang-format` at the repo root: C++23, 4-space indent, 120-column limit, left pointer alignment. Run `clang-format -i <file>` before committing.