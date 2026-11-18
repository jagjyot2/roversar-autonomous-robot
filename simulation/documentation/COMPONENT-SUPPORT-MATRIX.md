# RoverSAR Component Support Matrix

This document defines the exact simulation fidelity boundary across **Wokwi**, the **Python 50 Hz Physics Simulator**, and the preserved **BenchTwin** staging package.

> **Strict Policy — No Silent Substitutions**: Unsupported hardware components are **never** silently replaced with incompatible parts. Where Wokwi lacks native models, RoverSAR uses explicit **custom Wokwi chip stubs (`.chip.json` + `.chip.c`)** paired with the documented **Python 50 Hz physics & raycasting harness (`simulator/engine.py`)**.

## Hardware-to-Simulator Matrix

| Component | Hardware Ref / Bus / Pins | Wokwi Native Support | Wokwi Implementation | Python 50 Hz Harness (`simulator/`) | BenchTwin (`benchtwin/`) |
|---|---|---|---|---|---|
| **ESP32-WROOM-32D** | `U1` (240 MHz, 4 MB Flash) | **Native** | `board-esp32-devkit-c-v4` running `firmware.elf` | Firmware kinematics & safety mirrored 1:1 | Preserved (`U1`) |
| **MPU6050 6-DOF IMU** | `U3` I2C `0x68` (`GPIO21` SDA, `GPIO22` SCL) | **Native** | `wokwi-mpu6050` | `ImuState` yaw rate & rollover tilt injection | Preserved (`U3`) |
| **HC-SR04 Ultrasonic (Aux)** | Auxiliary forward ranging (`GPIO15` TRIG, `GPIO12` ECHO) | **Native** | `wokwi-hc-sr04` | Mirrors center `0°` raycast distance | Optional aux header |
| **Hardware NC E-STOP** | `SW2` (`GPIO32`, HIGH = tripped) | **Native** | `wokwi-pushbutton` | `SafetySupervisor` `ESTOP_HW` latch | Preserved (`SW2`) |
| **2S Li-Ion Battery Divider** | `R1/R2` 22k/10k on `GPIO33` (`ADC1_CH5`) | **Native** | `wokwi-potentiometer` | `BatterySim` (`8.4V` full, `6.0V` cutoff) | Preserved (`BT1`) |
| **Motor PWM & Fault LEDs** | `GPIO25` (L PWM), `GPIO19` (R PWM), `GPIO2` (Status/Fault) | **Native** | `wokwi-led` | `WheelActuatorPlant` effort & `SafetyFault` | Preserved (`STATUS_LED`) |
| **5-ch VL53L0X ToF Array** | `U4-U8` I2C `0x30..0x34`, `XSHUT` `{13,14,16,17,5}` | **Unsupported Natively** | Custom Wokwi chip `chip-vl53l0x-array` (`wokwi/chip-vl53l0x-array.chip.json/.c`) | `Vl53l0xArraySim` 5-beam raycaster (`-60°..+60°`) | Preserved (`U4-U8`) |
| **TB6612FNG / L298N + DC Motors + 1440 CPR Encoders** | `U2`, `M1/M2`, `GPIO25/26/27/19/18/23/4`, `GPIO34/35/36/39` | **Unsupported Natively** | Custom Wokwi chip `chip-quad-encoder` (`wokwi/chip-quad-encoder.chip.json/.c`) + PWM LEDs | `WheelActuatorPlant` + `DiffDriveKinematics` + `EncoderOdometry` | Preserved (`U2`, `M1`, `M2`) |
| **ESP32-CAM OV2640** | `U9` MJPEG `:81/stream` | **Unsupported Natively** | External stream URL (`http://192.168.4.2:81/stream`) | `simulator/camera_feed.py` synthetic FPV raycaster | Preserved (`U9`) |

## Verification Rule
All pin assignments in `wokwi/diagram.json` MUST match `firmware/include/roversar_config.h` and are automatically verified by `tests/test_simulation_architecture_and_wokwi.py`.
