# RoverSAR Wokwi Simulation Setup

**Wokwi** is the active ESP32 hardware/firmware simulator for RoverSAR, replacing BenchTwin as the primary active simulation target while preserving `benchtwin/` for future reactivation.

## Prerequisites
1. **Build PlatformIO Firmware**:
   ```bash
   cd firmware
   pio run
   ```
   This generates `firmware/.pio/build/esp32dev/firmware.bin` and `firmware.elf`, which `wokwi/wokwi.toml` references directly.

2. **Wokwi Extension or Wokwi CLI**:
   - **VS Code**: Install the *Wokwi Simulator* extension and open `wokwi/diagram.json` (`F1` -> `Wokwi: Start Simulator`).
   - **CLI (CI / Headless)**: Install `wokwi-cli`, export your API token, and run from `wokwi/`:
     ```bash
     export WOKWI_CLI_TOKEN="<your-wokwi-token>"   # PowerShell: $env:WOKWI_CLI_TOKEN="..."
     cd wokwi
     wokwi-cli --timeout 10000
     ```
   > **Note**: `WOKWI_CLI_TOKEN` is required only for headless `wokwi-cli` cloud runs; never commit tokens into the repository.

## Component Coverage & Custom Chips
See [`../simulation/documentation/COMPONENT-SUPPORT-MATRIX.md`](../simulation/documentation/COMPONENT-SUPPORT-MATRIX.md) for the full native vs. stubbed component table:
- **Native Wokwi parts**: `board-esp32-devkit-c-v4`, `wokwi-mpu6050` (I2C `GPIO21/22`), `wokwi-hc-sr04` (`GPIO15/12`), `wokwi-pushbutton` NC E-STOP (`GPIO32`), `wokwi-potentiometer` 2S battery divider (`GPIO33`), `wokwi-led` PWM & fault indicators (`GPIO25/19/2`).
- **Custom Wokwi chip stubs**:
  - `chip-quad-encoder.chip.json` / `.c` — TB6612FNG + 1440 CPR quadrature encoder pulse stub (`GPIO25/26/27/19/18/23/4` & `GPIO34/35/36/39`).
  - `chip-vl53l0x-array.chip.json` / `.c` — 5-channel VL53L0X I2C + `XSHUT` stub (`GPIO13/14/16/17/5`).
- **50 Hz Physics & FPV Harness**: Paired with `simulation/adapters/wokwi/wokwi_rover_adapter.py` and `simulator/rover_sim_server.py`.
