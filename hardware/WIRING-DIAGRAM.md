# RoverSAR Electrical Wiring & GPIO Architecture

## 1. Power Distribution & Safety Interlock Topology
```
[2S Li-Ion 7.4V 2200mAh]
   │
   ├──(+)──[5A Automotive Blade Fuse F1]──[Main Toggle SW1]──┬──> TB6612FNG VM (Motor Power, 7.4V)
   │                                                         ├──> LM2596 5V Buck IN+ ──> 5.0V Rail
   │                                                         │       ├── ESP32 DevKit VIN (AMS1117 -> 3.3V)
   │                                                         │       ├── ESP32-CAM 5V Pin
   │                                                         │       └── Quadrature Encoder VCC (5V/3.3V)
   │                                                         └──> VBAT Divider (R1=22kΩ / R2=10kΩ) -> GPIO33
   └──(─)── Common Star Ground (GND)
```

> **Hardware E-STOP Loop**: A Normally-Closed (NC) panel push-button switch `SW2` connects `GPIO32` to `GND` (with `10 kΩ` pull-up to `3.3V` + internal pull-up). Pressing E-STOP or severing the switch harness drives `GPIO32 = HIGH`, immediately latching `SafetyFault::ESTOP_HARDWARE` and pulling `TB6612FNG STBY (GPIO4)` to `LOW` (reinforced by external `10 kΩ` pull-down resistor `R3` so motors stay disabled during ESP32 boot/reset!).

## 2. Complete ESP32-WROOM-32D GPIO Pin Mapping
| Peripheral / Function | Signal Name | ESP32 GPIO | Direction / Type | Notes / Conditioning |
|---|---|---|---|---|
| **I2C Shared Bus** | `I2C_SDA` | `GPIO21` | Bidir (400 kHz) | 4.7 kΩ pull-up to 3.3V (`R4`) |
| **I2C Shared Bus** | `I2C_SCL` | `GPIO22` | Output (400 kHz) | 4.7 kΩ pull-up to 3.3V (`R5`) |
| **Left Motor PWM** | `TB6612_PWMA` | `GPIO25` | PWM Output (20 kHz) | 10-bit LEDC channel |
| **Left Motor Dir 1/2** | `AIN1` / `AIN2` | `GPIO26` / `GPIO27` | Output | Direction H-bridge logic |
| **Right Motor PWM** | `TB6612_PWMB` | `GPIO19` | PWM Output (20 kHz) | 10-bit LEDC channel |
| **Right Motor Dir 1/2** | `BIN1` / `BIN2` | `GPIO18` / `GPIO23` | Output | Direction H-bridge logic |
| **Motor Driver Standby** | `TB6612_STBY` | `GPIO4` | Output | External 10 kΩ pull-down (`R3`) to GND |
| **Left Quadrature Enc** | `ENC_L_A` / `ENC_L_B` | `GPIO34` / `GPIO35` | Input-Only ISR | 10 kΩ pull-ups to 3.3V (`R6, R7`) |
| **Right Quadrature Enc** | `ENC_R_A` / `ENC_R_B` | `GPIO36` / `GPIO39` | Input-Only ISR | 10 kΩ pull-ups to 3.3V (`R8, R9`) |
| **VL53L0X #0 (-60°)** | `TOF_XSHUT_0` | `GPIO13` | Output | Reassigned to I2C `0x30` |
| **VL53L0X #1 (-30°)** | `TOF_XSHUT_1` | `GPIO14` | Output | Reassigned to I2C `0x31` |
| **VL53L0X #2 (0°)** | `TOF_XSHUT_2` | `GPIO16` | Output | Reassigned to I2C `0x32` |
| **VL53L0X #3 (+30°)** | `TOF_XSHUT_3` | `GPIO17` | Output | Reassigned to I2C `0x33` |
| **VL53L0X #4 (+60°)** | `TOF_XSHUT_4` | `GPIO5` | Output | Reassigned to I2C `0x34` (boot pull-up safe) |
| **Battery ADC Monitor** | `VBAT_ADC` | `GPIO33` | Analog ADC1_CH5 | `R1=22kΩ` top, `R2=10kΩ` bottom + `100nF` filter cap |
| **Hardware E-STOP** | `ESTOP_HW` | `GPIO32` | Input (Pull-Up) | NC E-STOP switch to GND (open = trip) |
| **Status LED** | `STATUS_LED` | `GPIO2` | Output | Onboard LED (solid ON during fault) |

## 3. I2C Address Allocation Map
| Device | Native Addr | Runtime Addr | Notes |
|---|---|---|---|
| **MPU6050 6-DoF IMU** | `0x68` | `0x68` | `AD0` tied to GND |
| **VL53L0X #0 (-60°)** | `0x29` | `0x30` | Enabled 1st via `GPIO13` |
| **VL53L0X #1 (-30°)** | `0x29` | `0x31` | Enabled 2nd via `GPIO14` |
| **VL53L0X #2 (0°)** | `0x29` | `0x32` | Enabled 3rd via `GPIO16` |
| **VL53L0X #3 (+30°)** | `0x29` | `0x33` | Enabled 4th via `GPIO17` |
| **VL53L0X #4 (+60°)** | `0x29` | `0x34` | Enabled 5th via `GPIO5` |

## 4. ESP32-CAM Companion Link
- The AI-Thinker **ESP32-CAM** receives regulated `5.0V` (`500 mA` peak during Wi-Fi TX) from the LM2596 buck converter (`C2 = 470 µF` bulk electrolytic capacitor near module header to prevent brownout resets).
- Connects to the RoverSAR Access Point (`RoverSAR-AP`) at static IP `192.168.4.2` streaming MJPEG on `:81/stream`.
