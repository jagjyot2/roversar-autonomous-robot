# RoverSAR — Autonomous Search & Rescue Ground Robot

[![License: MIT](https://img.shields.io/badge/License-MIT-cyan.svg)](./LICENSE)
[![PlatformIO ESP32](https://img.shields.io/badge/Firmware-PlatformIO%20ESP32-blue.svg)](./firmware/platformio.ini)
[![FastAPI + React](https://img.shields.io/badge/Simulator-FastAPI%20%2B%20React%20TS-0ea5e9.svg)](./simulator/rover_sim_server.py)

**RoverSAR** is an autonomous differential-drive search-and-rescue (SAR) ground robot combining modular **PlatformIO ESP32 C++ firmware**, a **FastAPI + WebSocket 2D physics/SLAM simulator**, a **responsive React + TypeScript dark-mode control HUD**, **3D-printable OpenSCAD/STL chassis CAD**, **KiCad 8 schematics & BOM**, and a **BenchTwin** hardware staging package.

---

## Key Capabilities

- **Modular ESP32 Firmware (`firmware/`)**:
  - `motor_control/`: Differential-drive inverse/forward kinematics, `50 Hz` anti-windup wheel encoder PID, `20 kHz` TB6612FNG / L298N PWM driver.
  - `sensors/`: 5-channel `VL53L0X` ToF array (`XSHUT` address reassignment `0x30..0x34`), `MPU6050` IMU yaw/tilt fusion, `IRAM_ATTR` quadrature encoder odometry (`1440 CPR`), ADC1 2S Li-Ion battery monitor.
  - `navigation/`: 2D Bayesian log-odds occupancy grid mapper + VFH+ local reactive obstacle avoidance + waypoint pursuit.
  - `safety/`: Deterministic **500 ms comm-loss heartbeat watchdog** (immediate motor halt), IMU `> 35°` rollover cutoff, critical battery cutoff (`<= 6.0 V`), and latched software/hardware NC E-STOP.
  - `communication/` & `camera/`: Built-in RFC-6455 WebSocket + HTTP JSON server and ESP32-CAM MJPEG stream link.
- **2D Physics Simulator & Responsive Control HUD (`simulator/` + `dashboard/`)**:
  - Real-time `50 Hz` differential-drive dynamics, ToF laser raycasting against indoor SAR rooms/debris, live occupancy grid SLAM, and synthetic raycasted FPV ESP32-CAM JPEG/MJPEG stream.
  - Dark-mode React + TypeScript HUD served directly at `/` featuring a **touch virtual joystick + WASD**, **live 2D occupancy map & laser rays canvas**, **click-to-set-waypoint**, **FPV camera HUD**, **RPM/battery/watchdog gauges**, **E-STOP**, and **"Simulate Comm Loss (500ms Test)"** fault injection.
- **Mechanical CAD, KiCad & BenchTwin (`cad/`, `hardware/`, `benchtwin/`)**:
  - `240 × 180 × 95 mm` PETG chassis deck (`roversar_base_plate.stl`), 5-beam ToF sensor arc (`vl53l0x_sensor_mount.stl`), and 15° ESP32-CAM tilt bracket (`esp32_cam_tilt_bracket.stl`).
  - Complete KiCad 8 schematic, netlist, `BOM.csv`, mechanical/CG analysis, GPIO wiring map, and BenchTwin staging package.

---

## Quick Start

### 1. Run Simulator & Web Control HUD
```bash
pip install -r requirements.txt
uvicorn simulator.rover_sim_server:app --host 0.0.0.0 --port 8050
```
Open **`http://localhost:8050`** on desktop or mobile:
- Use **WASD / Arrow keys** or the **virtual touch joystick** to drive manually.
- **Click/tap anywhere** inside the 2D arena map to dispatch an autonomous VFH+ waypoint.
- Click **"Simulate Comm Loss (500ms Test)"** to verify the watchdog trips `COMM_LOSS_500MS` and halts both motors within 500 ms.
- Click **EMERGENCY E-STOP** to immediately lock drive actuation.

### 2. Run Automated Verification Tests
```bash
pytest -v
```

### 3. Build ESP32 Firmware (`firmware/`) & Run Wokwi Simulation (`wokwi/`)
```bash
cd firmware
pio run
```
Once built, `wokwi/wokwi.toml` points directly to `firmware/.pio/build/esp32dev/firmware.bin` and `firmware.elf`.
- **VS Code Wokwi Extension**: Open `wokwi/diagram.json` and run **Wokwi: Start Simulator**.
- **Wokwi CLI**: Set `WOKWI_CLI_TOKEN` and run `wokwi-cli` inside `wokwi/` (see [`wokwi/README.md`](./wokwi/README.md)).
- **Supported vs. Stubbed Components**: `ESP32-WROOM-32D`, `MPU6050`, `HC-SR04`, `NC E-STOP`, `2S VBAT divider`, and PWM/fault LEDs are natively modeled in Wokwi; the 5-ch `VL53L0X` array, `TB6612FNG` + 1440 CPR quadrature encoders, and `ESP32-CAM` use custom Wokwi chip stubs paired with the 50 Hz Python physics harness (see [`simulation/documentation/COMPONENT-SUPPORT-MATRIX.md`](./simulation/documentation/COMPONENT-SUPPORT-MATRIX.md)).

### 4. Rebuild React + TypeScript Dashboard (`dashboard/`)
```bash
cd dashboard
npm install
npm run build
```

---

## Repository Structure
- `firmware/` — Modular PlatformIO ESP32 C++ project (`include/`, `src/`, `platformio.ini`)
- `simulator/` — FastAPI + WebSocket `/ws/rover` physics, raycasting, SLAM, VFH+, and safety engine
- `dashboard/` — React + TypeScript + Vite responsive dark-mode ground control UI (`dist/` served at `/`)
- `cad/` — OpenSCAD `.scad` sources, reproducible STL builder, and binary `.stl` meshes
- `hardware/` — `CHASSIS-DESIGN.md`, `WIRING-DIAGRAM.md`, `roversar.kicad_sch`, `roversar.net`, `BOM.csv`
- `simulation/` — Simulator-independent HAL (`adapters/common/`), active `wokwi/` adapter, preserved `benchtwin/` manifest, fault/navigation `scenarios/`, and `COMPONENT-SUPPORT-MATRIX.md`
- `wokwi/` — Active Wokwi ESP32 diagram (`diagram.json`), `wokwi.toml`, and custom chip stubs (`chip-quad-encoder`, `chip-vl53l0x-array`)
- `benchtwin/` — Preserved BenchTwin hardware-in-the-loop staging package (`roversar.btw`, manifest, netlist, BOM, STL) for future reactivation
- `tests/` — Automated pytest suite covering kinematics, PID, odometry/mapping, wall avoidance, 500 ms failsafe, FastAPI/WS, STL, KiCad & BenchTwin validation
- `docs/ARCHITECTURE.md` & `ROADMAP.md` — System architecture & future engineering roadmap

---

## License
Released under the [MIT License](./LICENSE).
