# Wokwi RoverSAR Adapter

`wokwi_rover_adapter.py` implements `RoverHalProtocol` (`simulation/adapters/common/rover_hal_interface.py`) to bridge the **Wokwi ESP32 simulation** (`wokwi/diagram.json` + `wokwi/wokwi.toml`) with the **50 Hz RoverSAR physics & raycasting harness** (`simulator/engine.py`).

## Component Boundary

| Subsystem | Wokwi Representation | Python 50 Hz Harness Role |
|---|---|---|
| ESP32-WROOM-32D | `board-esp32-devkit-c-v4` (native) | Executes compiled `firmware.elf` |
| MPU6050 IMU (`0x68`) | `wokwi-mpu6050` (native I2C GPIO21/22) | Injects tilt/rollover scenarios |
| Auxiliary Ultrasonic | `wokwi-hc-sr04` (native GPIO15/12) | Mirrors center raycast distance |
| Hardware E-STOP (NC) | `wokwi-pushbutton` (native GPIO32) | Latches `ESTOP_HW` safety fault |
| 2S Li-Ion Divider | `wokwi-potentiometer` (GPIO33) | Mirrors `BatterySim` ADC voltage |
| Motor PWM / Fault LEDs | `wokwi-led` (GPIO25, GPIO19, GPIO2) | Visualizes PWM & fault status |
| 5-ch VL53L0X ToF Array | `chip-vl53l0x-array` custom chip stub | Supplies 5-beam raycasted distances |
| TB6612FNG + 1440 CPR Encoders | `chip-quad-encoder` custom chip stub | Integrates diff-drive ticks at 50 Hz |
| ESP32-CAM MJPEG | Not simulated in Wokwi | Served by `simulator/camera_feed.py` |
