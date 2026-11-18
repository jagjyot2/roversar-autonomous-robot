# Manufacturing Readiness Report — `05-ROVERSAR` (`roversar`)

- **Board Reviewed**: [`roversar.kicad_pcb`](roversar.kicad_pcb) / [`roversar.kicad_sch`](roversar.kicad_sch)
- **Audit Date**: `2026-11-18`
- **Release Decision**: **`HOLD — PROVISIONAL HARDWARE REVISION NEEDED`** (`F1` fuse holder, `SW1` rocker switch, and `U9` ESP32-CAM interface need mechanical/schematic updates before fabrication)

---

## 1. Board Specifications & DRC/ERC Summary (Observed Live)

| Parameter | Measured Value | Status / Observation |
|---|---|---|
| Dimensions (`Edge.Cuts`) | `100.0 x 80.0 mm`, thickness `1.6 mm`, `ENIG` | `VERIFIED` |
| Copper Stackup | `2` Layers (`F.Cu`, `B.Cu`), `1 oz` Cu | `VERIFIED` |
| Placed Symbols / Footprints | `41` schematic symbols / `41` PCB footprints (`37` components + `H1..H4`) | `VERIFIED` |
| KiCad `10.0.8` ERC (`--severity-all`) | `0` Errors, `0` Warnings (exit `0`) | `VERIFIED WITH LIMITATIONS` |
| KiCad `10.0.8` DRC (`--schematic-parity`) | `0` Violations, `0` Unconnected Pads, `0` Parity Issues (exit `0`) | `VERIFIED WITH LIMITATIONS` |
| Mounting Holes (`H1..H4`) | `(4,4), (96,4), (96,76), (4,76)` (`3.2 mm` drill, `5.2 mm` pad, **`1.40 mm`** pad-to-edge clearance) | `VERIFIED` |
| Vias & Drills (`roversar-PTH.drl`) | `56` vias (`0.70 / 0.35 mm`); component drills: `0.90`, `1.00`, `3.20`, `3.80 mm` | `VERIFIED` |
| `B.Cu` Ground Pour (`GND`) | `1` contiguous `filled_polygon` (`0.25 mm` clearance, `0.35 mm` thermal bridge) | **`VERIFIED`** (best ground-plane integrity among breakout boards alongside `CellGuard`) |
| BOM Integrity (`BOM.csv` vs `BOM-kicad.csv`) | `BOM.csv`: `20` grouped procurement rows (incl. off-board motors/battery/casters); `BOM-kicad.csv`: `41` per-refdes rows | `VERIFIED` |
| Gerber / Drill Files (`hardware/gerbers/`) | Complete 2-layer Gerber set (`9` Gerber + `.gbrjob` + `2` Excellon `.drl` + drill maps) + current `roversar_{erc,drc}.{txt,json}` | `VERIFIED` |

---

## 2. Trace Width vs Current Capacity (`1 oz` Copper, `20 °C` Rise Estimate)

| Net Name | Measured Widths (mm) | Length (mm) | Expected Current | Engineering Assessment | Status |
|---|---|---|---|---|---|
| `VBAT_RAW` (`J1 XT60` -> `F1`) | `[1.50]` | `13.0` | Up to `5 A` fuse limit (`~2.4 A` typical 2x `TB6612` + logic) | `1.5 mm` `1 oz` Cu carries `~3.5 A` @ `20 °C` rise (`~4.8 A` @ `35 °C` rise); widen to `>= 2.0–2.5 mm` for full `5 A` continuous headroom. | **`VERIFIED WITH LIMITATIONS`** |
| `VBAT_FUSED` (`F1` -> `SW1`) | `[1.50]` | `7.9` | Up to `5 A` | Same as `VBAT_RAW` (`~3.5 A` @ `20 °C` rise). | **`VERIFIED WITH LIMITATIONS`** |
| `VBAT_SW` (`SW1` -> `D1` / `U10` / `U2`) | `[1.00]` | `110.5` | `~1.5–3.2 A` peak stall | `1.0 mm` `1 oz` carries `~2.6 A` @ `20 °C` rise; adequate for `1.2 A` continuous dual-motor + buck load, borderline on simultaneous dual-motor `3.2 A` stall. | **`VERIFIED WITH LIMITATIONS`** |
| `M_L_A/B`, `M_R_A/B` | `[0.80]` | `17.0–34.1` | `1.2 A` cont / `3.2 A` peak (`TB6612FNG`) | `0.80 mm` `1 oz` Cu carries `~2.3 A` @ `20 °C` rise (adequate for `JGA25-370` `1.2 A` continuous). | `VERIFIED` |
| `V5_REG`, `V3_3` | `[0.60]`, `[0.50]` | `53.6`, `260.7` | `~0.8 A` (`ESP32-CAM` + `ESP32` + `5x VL53L0X`) | `470 uF` bulk capacitor `C2` protects `ESP32-CAM` TX spikes; `0.6 mm` carries `~1.9 A` @ `20 °C` rise. | `VERIFIED` |

---

## 3. Component Footprint, Connector & Architecture Audit

| RefDes | BOM Part / Role | Placed Footprint | Verdict | Finding |
|---|---|---|---|---|
| `F1` | `5A_ATO_FUSE` (Inline blade) | `PinHeader_1x02_P2.54mm` | **`PROVISIONAL` / `FAILED` (direct mount)** | Standard ATO/ATC blade fuse prongs are `10.8 mm` apart (`~6.3 x 0.8 mm` blades) and cannot insert into a `2.54 mm` header; requires PCB ATO clip footprint (`Littelfuse 01000057`) or off-board wire pigtails. |
| `SW1` | `MAIN_PWR_SW` (`Carling R13-112` `10A` rocker) | `PinHeader_1x02_P2.54mm` | **`PROVISIONAL`** | Must be wired via off-board panel leads (`1x02` header pins rated `~3 A` max, below `10 A` switch). |
| `SW2` | `ESTOP_NC_SW` (`GPIO32` emergency stop) | `PinHeader_1x02_P2.54mm` | **`VERIFIED WITH LIMITATIONS`** | Logic-level `3.3 V` NC E-stop switch with `10k` pull-up; safe for panel lead wiring (`STBY` hardware cutoff also depends on firmware polling unless E-stop directly gates `TB6612` `STBY` / `VMOT`). |
| `U9` | `ESP32-CAM AI-Thinker` (`16-pin DIP` in BOM) | `PinHeader_1x02_P2.54mm` | **`VERIFIED WITH LIMITATIONS`** | Power-only header (`5V`, `GND` only) — lacks UART TX/RX connection to `U1` (`ESP32`) and full 16-pin mechanical support. |
| `U2` | `TB6612FNG` Breakout | `Breakout_16Pin_DIP` | **`PROVISIONAL`** | Matches generic Pololu/SparkFun `16-pin` DIP breakout width; verify exact `15.24 mm` vs `20.32 mm` row spacing before soldering. |
| `U4..U8` | `5x VL53L0X` (`X = 34,42,50,58,66 mm`) | `PinHeader_1x05_P2.54mm` | **`PROVISIONAL`** | `8.0 mm` center-to-center spacing (`10.5 mm` typical GY-530 module width) prevents adjacent modules plugging vertically into the PCB without angled headers or ribbon cables. |
| `M1`, `M2` | `JGA25-370` `6-pin JST` | `PinHeader_1x06_P2.54mm` | **`PROVISIONAL`** | Uses `2.54 mm` headers rather than keyed `JST-XH` (`2.50 mm`) / `JST-PH` (`2.00 mm`). |
| `J1` | `XT60_2S_BAT` (`7.4–12.6 V`) | `AMASS_XT60_Vertical` | **`PROVISIONAL`** | Lacks reverse-polarity diode/PFET protection ahead of `U10 LM2596` and `U2 TB6612` if battery leads are miswired (relies strictly on keyed `XT60`). |

## 4. Required Actions Before Fabrication
1. Replace `F1` with a high-current PCB ATO fuse holder footprint (`10.8 mm` blade pitch, `>= 5 A`) and widen `VBAT_RAW` / `VBAT_FUSED` / `VBAT_SW` to `>= 2.0–2.5 mm`.
2. Add hardware E-stop tie or `VMOT` relay/PFET gate if safety rules require hardware-level motor cutoff on `SW2` open (in addition to `GPIO32` software detection).
3. Upgrade `M1/M2` to keyed `2.00 mm` / `2.50 mm` shrouded headers matching the `JGA25-370` harness and add silkscreen `+ / -` markers on `J1`.
