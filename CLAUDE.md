# MeshCom Firmware - Development Guidelines

## Project Context
This is an open-source project (MeshCom Firmware). We contribute via PRs against the **upstream DEV branch** of the icssw-org repository.

## PR Workflow

### 1. Sync Upstream First
Before any coding, always sync/rebase against the latest upstream DEV branch to incorporate all upstream changes.

### 2. Minimal Changes Only
We **cherry-pick the absolute minimum** of code changes. We do NOT rewrite or refactor large parts of the project. Every change must be targeted and justified.

### 3. PR Description (German, Detailed)
Every PR **must** include a detailed description written in **German**:
- Describe exactly which code was changed (files, functions, logic)
- Explain **why** each change was made (motivation, bug fix rationale, improvement reason)
- This description must be prepared **before** submitting the PR

### 4. PR Target
All PRs target the **DEV branch** of the upstream repository (not main).

## Hardware & Flashing

### RAK4631 (nRF52840)
- **Serial port:** `/dev/cu.usbmodem2101`
- **Bootloader:** WisBlock RAK4631 UF2 Bootloader v0.4.2, SoftDevice S140 6.1.1
- **Build:** `pio run -e wiscore_rak4631`
- **Flash method (UF2):**
  1. Double-tap the reset button to enter UF2 bootloader mode (volume `RAK4631` appears under `/Volumes/`)
  2. Convert hex to UF2: `python3 ~/.platformio/packages/framework-arduinoadafruitnrf52/tools/uf2conv/uf2conv.py .pio/build/wiscore_rak4631/firmware.hex -c -f 0xADA52840 -o .pio/build/wiscore_rak4631/firmware.uf2`
  3. Copy UF2 to volume: `cp .pio/build/wiscore_rak4631/firmware.uf2 /Volumes/RAK4631/`
  4. Device reboots automatically after flashing (macOS may show an I/O error — this is cosmetic)
- **Flash method (PlatformIO):** `pio run -e wiscore_rak4631 --target upload` — uses `adafruit-nrfutil` DFU serial, requires the device to be running (not in UF2 mode)
- **Note:** If the serial port is busy (e.g. Chrome Web Serial), close the connection first. Check with `lsof /dev/cu.usbmodem2101`.

### ESP32 boards (Heltec V3, T-Beam, T-Deck, etc.)
- Flash via `esptool` using custom `upload_command` defined in each variant's `platformio.ini`
- Use `pio run -e <env> --target upload`
