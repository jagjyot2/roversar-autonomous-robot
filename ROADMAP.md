# RoverSAR Development Roadmap

## Phase 1: Embedded Firmware & Safety Architecture (Completed)
- [x] Modular PlatformIO C++ ESP32 firmware (`motor_control`, `sensors`, `navigation`, `safety`, `communication`, `camera`)
- [x] Hardware-safe GPIO allocation (ADC1-only battery sense, input-only quadrature ISRs, external pulldown `STBY`)
- [x] Verified ESP32 compilation (`pio run` ELF/BIN build passing cleanly)

## Phase 2: Physics/SLAM Simulator & Responsive Control HUD (Completed)
- [x] Differential-drive physics, wheel PID, VL53L0X raycaster, and 2D log-odds occupancy SLAM (`simulator/`)
- [x] VFH+ reactive obstacle avoidance & waypoint pursuit around indoor rubble/walls
- [x] FastAPI + `/ws/rover` WebSocket server & FPV raycasted JPEG/MJPEG stream
- [x] React + TypeScript dark-mode responsive HUD (virtual touch joystick, WASD, click-to-waypoint, E-STOP, comm-loss injection)

## Phase 3: Mechanical CAD, KiCad Electrical Design & BenchTwin (Completed)
- [x] Parametric OpenSCAD `.scad` & verified binary `.stl` files (`roversar_base_plate`, `vl53l0x_sensor_mount`, `esp32_cam_tilt_bracket`)
- [x] `CHASSIS-DESIGN.md`, `WIRING-DIAGRAM.md`, KiCad 8 schematic (`roversar.kicad_sch`), netlist (`roversar.net`), and `BOM.csv`
- [x] BenchTwin hardware-in-the-loop staging package (`benchtwin/`)

## Phase 4: Field Hardware Expansion (Planned)
- [ ] A* global path planning over the live 2D occupancy grid for multi-room frontier exploration
- [ ] Optional RPLIDAR A1 / Slamtec 360° UART integration alongside the 5-beam VL53L0X low-obstacle bumper ring
- [ ] AprilTag / thermal ML victim detection overlay inside ESP32-CAM stream
- [ ] Custom 2-layer carrier PCB Gerber fabrication from `roversar.kicad_sch`
