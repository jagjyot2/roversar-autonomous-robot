# RoverSAR Physical Chassis Mechanical Design

## 1. Overall Envelope & Mass Budget
- **Bounding Dimensions**: `240 mm (L) × 180 mm (W) × 95 mm (H)`
- **Ground Clearance**: `16.5 mm` beneath main 4 mm structural deck (with 65 mm diameter rubber-tread wheels)
- **Total Ready-to-Run Mass**: `~860 g` (including 2S 2200 mAh Li-Ion pack, dual geared motors, electronics stack, and ESP32-CAM)
- **Primary Chassis Material**: PETG or Carbon-Fiber PLA (`35%` gyroid infill, `4` perimeters, `0.20 mm` layer height)

## 2. Coordinate Frame & Component Layout
Origin `(0, 0, 0)` lies at the geometric center of the bottom surface of [`roversar_base_plate.stl`](../cad/roversar_base_plate.stl), with `+X` forward, `+Y` port (left), `+Z` up:

| Subsystem | X Center (mm) | Y Center (mm) | Z Elevation (mm) | Notes |
|---|---|---|---|---|
| **Rear Swivel Caster** | `-92.0` | `0.0` | `-16.0` (ground) | 4x M3×10mm mount boss; supports rear stability |
| **2S 18650 Battery Pack** | `-22.0` | `0.0` | `+4.0` to `+26.0` | Seated between retention ribs; rear-biased CG improves traction & eliminates forward pitch tip |
| **Drive Axle (L & R JGA25-370)** | `+18.0` | `±66.0` | `+11.0` | Track width `160 mm` center-to-center across 65mm rubber wheels |
| **ESP32 + TB6612FNG Carrier PCB** | `+10.0` | `0.0` | `+34.0` | Mounted above battery bay on 4× M3×28mm brass hex standoffs |
| **MPU6050 6-DoF IMU** | `+15.0` | `0.0` | `+36.0` | Rigidly bolted near axle centerline to minimize centrifugal yaw coupling |
| **ESP32-CAM Tilt Bracket** | `+65.0` | `0.0` | `+40.0` to `+84.0` | `15°` downward pitch via [`esp32_cam_tilt_bracket.stl`](../cad/esp32_cam_tilt_bracket.stl) |
| **5× VL53L0X ToF Sensor Arc** | `+105.0` | `0.0` | `+12.0` to `+36.0` | Curved sensor array [`vl53l0x_sensor_mount.stl`](../cad/vl53l0x_sensor_mount.stl) |

## 3. Drive & Motor Mounting
- **Motors**: Dual 12V/7.4V JGA25-370 DC geared motors (`180 RPM` nominal at 7.4V, `34:1` planetary/spur reduction) with integrated rear magnetic quadrature encoders (`11 PPR × 34 × 4 ≈ 1440 counts/rev`).
- **Mounting**: Dual semi-enclosed PETG motor cradles (`Ø24.6 mm` bore) molded onto the top deck with two M3×8mm clamping screws per bracket face, keeping the wheels recessed cleanly inside `74 mm × 24 mm` side wheel arches.
- **Wheel Track & Kinematics**: Track width $L = 160\text{ mm}$, wheel radius $r = 32.5\text{ mm}$.

## 4. Sensor Field-of-View (FOV) & Placement
- **5× VL53L0X Time-of-Flight Array**: Mounted at `R = 68 mm` arc centered at `X = +37 mm` (front tip at `X = +105 mm`), oriented at `[-60°, -30°, 0°, +30°, +60°]`. Since each VL53L0X emitter has a `25°` cone FOV, the 5 sensors produce an overlapping **`145°` forward collision-avoidance fan** (`-72.5°` to `+72.5°`) with zero blind spots at `> 0.20 m`.
- **ESP32-CAM (OV2640)**: Elevated to `Z = 74 mm` with a `15°` downward tilt, providing unobstructed `64°` horizontal FOV ahead of the front bumper starting `~0.25 m` in front of the chassis up to `> 6 m`.

## 5. Cable Routing & Center of Gravity (CG)
- **Cable Routing**: Dual `12 mm × 40 mm` rounded pass-through slots at `X = -55 mm` (battery leads to fuse & main switch) and `X = +58 mm` (I2C ribbon cable & ToF XSHUT harness) prevent pinch hazards near spinning wheels.
- **Center of Gravity (CG)**: Battery mass (`~110 g`) placed at `X = -22 mm` and drive motors (`~180 g` combined) at `X = +18 mm` yield a composite CG at **`X ≈ +2 mm, Y = 0 mm, Z = 27 mm`** — placing `~56%` weight on the drive wheels and `~44%` on the rear caster, ensuring crisp turn-in without wheel slip or nose-diving under hard braking.

## 6. Fasteners & M3 Hardware Schedule
- **M3×8mm SHCS (×12)**: Motor brackets, VL53L0X sensor arc mount, ESP32-CAM tilt base.
- **M3×28mm Brass Hex Standoffs + M3×6mm Screws (×4 pairs)**: Upper electronics deck over battery bay.
- **M3×10mm Nylon Locknuts + Screws (×4)**: Rear swivel caster attachment.
- **M2.5×6mm Screws (×10)**: Securing VL53L0X breakout boards to front optical bays.
- **M3×14mm Pivot Bolt + Locknut (×2)**: ESP32-CAM tilt angle friction clamp.
