# RoverSAR BenchTwin Hardware-in-the-Loop & Bring-Up Notes

## 1. Staging Package Contents
- `roversar.btw`: BenchTwin virtual workbench project configuration (DC supply 7.4V @ 3.5A limit, 8-channel logic analyzer, fault injection triggers).
- `roversar-benchtwin.json`: Full 3D placement, power rail topology, I2C address map, and pin-to-pin wiring manifest.
- `roversar.net` & `BOM.csv`: Synchronized KiCad 8 netlist and complete component Bill of Materials.
- `roversar_base_plate.stl`: Primary `240 × 180 mm` chassis deck CAD mesh for 3D mechanical clearance inspection.

## 2. Safe Bench Bring-Up Sequence
1. **Current-Limited Supply**: Remove `BT1` and connect bench PSU to XT60 input at **`7.40 V`** with current limit **`0.80 A`** (motors mechanically unloaded on jack stands).
2. **Rail Verification**:
   - Confirm `V5_REG = 5.00 V ± 0.10 V` at `LM2596` output before inserting `U1` (ESP32-WROOM-32D) or `U9` (ESP32-CAM).
   - Confirm `VBAT_ADC` on `GPIO33` reads `7.40 * (10 / 32) = 2.31 V` (safe below ESP32 3.3V ADC ceiling).
   - Confirm `STBY` (`GPIO4`) remains `0.0 V` during boot due to `R3 = 10 kΩ` pull-down.
3. **I2C Address Enumeration**:
   - Probe `I2C_SDA (GPIO21)` and `I2C_SCL (GPIO22)` at `400 kHz`.
   - Verify `MPU6050` responds at `0x68` and all five `VL53L0X` sensors sequentially migrate from `0x29` to `0x30..0x34` via `XSHUT` (`GPIO13, 14, 16, 17, 5`).
4. **Safety Interlock Bench Tests**:
   - **500ms Watchdog Check**: Command manual forward drive (`0.30 m/s`) via `/ws/rover`, then halt WebSocket heartbeats. Measure `STBY (GPIO4)` dropping from `3.3V -> 0V` and `PWMA`/`PWMB` duty dropping to `0%` within `<= 500 ms`.
   - **Hardware E-STOP Check**: Open `SW2` (NC switch on `GPIO32`). Verify immediate hardware latch `ESTOP_HARDWARE` and zero motor output regardless of WebSocket traffic.
   - **Rollover Tilt Cutoff**: Tilt chassis `> 35°` on roll/pitch axis and confirm `TILT_ROLLOVER` disables `STBY`.
