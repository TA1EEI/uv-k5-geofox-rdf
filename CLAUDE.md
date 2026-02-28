# CLAUDE.md

## Project Overview

Custom firmware for **Quansheng UV-K5/K6/5R v2.1.27** handheld radios. Written in C targeting the DP32G030 ARM Cortex-M0 MCU. Merges OneOfEleven's custom firmware, Fagci's spectrum analyzer, and additional enhancements on top of DualTachyon's open firmware.

## Build

### Prerequisites

- `arm-none-eabi-gcc` v10.3.1 (Ubuntu 22.04 LTS toolchain recommended)
- GNU `make`
- Python 3 with `crcmod` module (for firmware packing)

### Build Commands

```bash
make              # Build firmware (outputs firmware.bin and firmware.packed.bin)
make clean        # Clean build artifacts
```

### Docker Build (alternative)

```bash
./compile-with-docker.sh   # Linux/Mac
compile-with-docker.bat    # Windows
```

### Build Output

- `firmware` — ELF binary
- `firmware.bin` — Raw binary
- `firmware.packed.bin` — Obfuscated binary with CRC (used by radio flasher)

## Architecture

### Memory Constraints

- **Flash:** 60KB (0x00000000–0x0000FFFF)
- **RAM:** 16KB (0x20000000–0x20003FFF)

Size is extremely constrained. Every byte matters.

### Key Directories

| Directory | Purpose |
|-----------|---------|
| `app/` | Application layer (menu, spectrum analyzer, scanner, DTMF, FM, UART) |
| `driver/` | Hardware drivers (BK4819 radio IC, BK1080 FM IC, LCD, keyboard, EEPROM, GPIO, SPI, I2C) |
| `ui/` | Display/UI rendering (main screen, menu, status bar, battery) |
| `helper/` | Utility functions (battery calculations, boot sequence) |
| `bsp/dp32g030/` | Board support package headers (auto-generated from `.def` files) |
| `hardware/dp32g030/` | MCU register definitions (`.def` files) |
| `external/` | Dependencies: printf library, ARM CMSIS |

### Key Files

| File | Purpose |
|------|---------|
| `main.c` | Entry point |
| `radio.c` | Radio control logic |
| `settings.c` | Settings management |
| `board.c` | Board initialization |
| `am_fix.c` | AM reception fix (dynamic front-end gain) |
| `start.S` | ARM assembly startup code |
| `firmware.ld` | Linker script (memory layout) |
| `fw-pack.py` | Firmware packing/obfuscation with XOR and CRC |
| `Makefile` | Build system with 50+ compile-time feature flags |

### Feature Flags

Feature toggles are defined at the top of `Makefile` (e.g., `ENABLE_UART`, `ENABLE_AM_FIX`, `ENABLE_SPECTRUM`). Edit the Makefile to enable/disable features, then run `make`.

## Code Conventions

- **Function names:** `PascalCase` (e.g., `RADIO_CheckValidChannel`)
- **Type names:** `camelCase` with `_t` suffix (e.g., `VFO_Info_t`)
- **Macros/constants:** `UPPER_SNAKE_CASE` (e.g., `ENABLE_UART`)
- **Style:** K&R-like bracing
- **Standard:** C2x (`-std=c2x`)
- **Warnings:** `-Wall -Werror -Wextra` — all warnings are errors

## Testing

No unit test suite. Compiler strict warnings (`-Werror`) serve as the primary automated quality check. CI runs `make` on every push.

## CI/CD

GitHub Actions (`.github/workflows/main.yml`):
- Triggers on every push
- Runs in Arch Linux container
- Builds firmware and uploads artifacts
- On version tags (`v*`): publishes to GitHub Releases as `egzumer_<tag>.packed.bin`

## Important Notes

- This is embedded firmware — no OS, no stdlib beyond nano.specs newlib
- Hardware registers are accessed directly via memory-mapped I/O
- BSP headers in `bsp/dp32g030/` are auto-generated from `.def` files in `hardware/dp32g030/` — do not edit BSP headers directly
- The `external/` directory contains vendored dependencies — do not modify
- `firmware.packed.bin` uses XOR obfuscation matching the Quansheng flasher protocol
