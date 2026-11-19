"""Generate and verify 05-ROVERSAR KiCad 10.0.8 schematic, PCB, Gerbers & 3D outputs."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "00-PORTFOLIO-MANAGEMENT" / "kicad-automation"))

import kicad_toolkit as kt  # noqa: E402

HW_DIR = Path(__file__).resolve().parent
LIB = "RoverSAR"
PROJ = "roversar"


def build() -> None:
    kt.write_utf8(HW_DIR / f"{PROJ}.kicad_pro", kt.build_kicad_pro(PROJ))
    kt.write_project_lib_tables(HW_DIR, LIB)

    sb = kt.SchematicBuilder(
        project_name=PROJ,
        lib_name=LIB,
        title="RoverSAR Autonomous Search & Rescue Robot Carrier",
        company="Jagjyot Engineering Portfolio",
        rev="1.1",
        paper="A1",
    )
    sb.add_note(
        25.4,
        15.24,
        "ROVERSAR CARRIER v1.1 (KICAD 10.0.8)\n"
        "- ESP32-WROOM-32D + TB6612FNG + 5x VL53L0X + MPU6050 + ESP32-CAM + LM2596 5V + 470uF Bulk Cap\n"
        "- Safety: 5A Fuse F1 + Hardware NC E-STOP (GPIO32) + R3 10k STBY Pull-Down + 500ms Watchdog",
    )

    # U1: ESP32-WROOM-32D DevKit (30P carrier mapping matching WIRING-DIAGRAM.md)
    sb.add_component(
        kt.SchComponent(
            "U1",
            "ESP32-WROOM-32D",
            f"{LIB}:ESP32_DevKit_30P",
            [
                kt.SchPin("1", "EN", "NC_EN", "left", "no_connect"),
                kt.SchPin("2", "GPIO36_ENC_RA", "ENC_R_A", "left"),
                kt.SchPin("3", "GPIO39_ENC_RB", "ENC_R_B", "left"),
                kt.SchPin("4", "GPIO34_ENC_LA", "ENC_L_A", "left"),
                kt.SchPin("5", "GPIO35_ENC_LB", "ENC_L_B", "left"),
                kt.SchPin("6", "GPIO32_ESTOP", "ESTOP_HW", "left"),
                kt.SchPin("7", "GPIO33_VBAT", "VBAT_ADC", "left"),
                kt.SchPin("8", "GPIO25_PWMA", "TB6612_PWMA", "left"),
                kt.SchPin("9", "GPIO26_AIN1", "AIN1", "left"),
                kt.SchPin("10", "GPIO27_AIN2", "AIN2", "left"),
                kt.SchPin("11", "GPIO14_XSHUT1", "TOF_XSHUT_1", "left"),
                kt.SchPin("12", "GPIO12_NC", "NC_D12", "left", "no_connect"),
                kt.SchPin("13", "GPIO13_XSHUT0", "TOF_XSHUT_0", "left"),
                kt.SchPin("14", "GND1", "GND", "left"),
                kt.SchPin("15", "VIN_5V", "V5_REG", "left"),
                kt.SchPin("16", "3V3", "V3_3", "right"),
                kt.SchPin("17", "GND2", "GND", "right"),
                kt.SchPin("18", "GPIO15_NC", "NC_D15", "right", "no_connect"),
                kt.SchPin("19", "GPIO2_STAT", "STATUS_LED", "right"),
                kt.SchPin("20", "GPIO4_STBY", "TB6612_STBY", "right"),
                kt.SchPin("21", "GPIO16_XSHUT2", "TOF_XSHUT_2", "right"),
                kt.SchPin("22", "GPIO17_XSHUT3", "TOF_XSHUT_3", "right"),
                kt.SchPin("23", "GPIO5_XSHUT4", "TOF_XSHUT_4", "right"),
                kt.SchPin("24", "GPIO18_BIN1", "BIN1", "right"),
                kt.SchPin("25", "GPIO19_PWMB", "TB6612_PWMB", "right"),
                kt.SchPin("26", "GPIO21_SDA", "I2C_SDA", "right"),
                kt.SchPin("27", "RX0", "CAM_UART_TX", "right"),
                kt.SchPin("28", "TX0", "CAM_UART_RX", "right"),
                kt.SchPin("29", "GPIO22_SCL", "I2C_SCL", "right"),
                kt.SchPin("30", "GPIO23_BIN2", "BIN2", "right"),
            ],
            mpn="ESP32-DevKitC-32D",
            manufacturer="Espressif",
            rating="3.3V IO / 5V VIN",
        )
    )

    # U2: TB6612FNG Dual Motor Driver
    sb.add_component(
        kt.SchComponent(
            "U2",
            "TB6612FNG",
            f"{LIB}:Breakout_16Pin_DIP",
            [
                kt.SchPin("1", "VM", "VBAT_SW", "left"),
                kt.SchPin("2", "VCC", "V3_3", "left"),
                kt.SchPin("3", "GND1", "GND", "left"),
                kt.SchPin("4", "PWMA", "TB6612_PWMA", "left"),
                kt.SchPin("5", "AIN1", "AIN1", "left"),
                kt.SchPin("6", "AIN2", "AIN2", "left"),
                kt.SchPin("7", "STBY", "TB6612_STBY", "left"),
                kt.SchPin("8", "PWMB", "TB6612_PWMB", "left"),
                kt.SchPin("9", "BIN1", "BIN1", "right"),
                kt.SchPin("10", "BIN2", "BIN2", "right"),
                kt.SchPin("11", "AO1", "M_L_A", "right"),
                kt.SchPin("12", "AO2", "M_L_B", "right"),
                kt.SchPin("13", "BO1", "M_R_A", "right"),
                kt.SchPin("14", "BO2", "M_R_B", "right"),
                kt.SchPin("15", "GND2", "GND", "right"),
                kt.SchPin("16", "GND3", "GND", "right"),
            ],
            mpn="TB6612FNG",
            manufacturer="Toshiba / Pololu",
            rating="15V / 1.2A cont (3.2A peak)",
        )
    )

    # U3: MPU6050
    sb.add_component(
        kt.SchComponent(
            "U3",
            "MPU6050-GY521",
            f"{LIB}:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical",
            [
                kt.SchPin("1", "VCC", "V3_3", "left"),
                kt.SchPin("2", "GND", "GND", "left"),
                kt.SchPin("3", "SCL", "I2C_SCL", "right"),
                kt.SchPin("4", "SDA", "I2C_SDA", "right"),
            ],
            mpn="GY-521",
            manufacturer="InvenSense",
            rating="3.3V / I2C 0x68",
        )
    )

    # U4-U8: 5x VL53L0X ToF (2.50 mm JST-XH)
    for idx in range(5):
        ref = f"U{4 + idx}"
        sb.add_component(
            kt.SchComponent(
                ref,
                f"VL53L0X_{idx}",
                f"{LIB}:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical",
                [
                    kt.SchPin("1", "VIN", "V3_3", "left"),
                    kt.SchPin("2", "GND", "GND", "left"),
                    kt.SchPin("3", "SCL", "I2C_SCL", "right"),
                    kt.SchPin("4", "SDA", "I2C_SDA", "right"),
                    kt.SchPin("5", "XSHUT", f"TOF_XSHUT_{idx}", "right"),
                ],
                mpn="GY-530",
                manufacturer="STMicroelectronics",
                rating="3.3V / I2C",
            )
        )

    # U9: ESP32-CAM 5V power + UART0 link header
    sb.add_component(
        kt.SchComponent(
            "U9",
            "ESP32-CAM",
            f"{LIB}:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical",
            [
                kt.SchPin("1", "5V", "V5_REG", "left"),
                kt.SchPin("2", "GND", "GND", "left"),
                kt.SchPin("3", "U0TX", "CAM_UART_TX", "right"),
                kt.SchPin("4", "U0RX", "CAM_UART_RX", "right"),
            ],
            mpn="ESP32-CAM",
            manufacturer="Ai-Thinker",
            rating="5.0V / 500mA + UART",
        )
    )

    # U10: LM2596 5V Buck
    sb.add_component(
        kt.SchComponent(
            "U10",
            "LM2596-5.0",
            f"{LIB}:Buck_4Pin_Module",
            [
                kt.SchPin("1", "VIN+", "VBAT_SW", "left"),
                kt.SchPin("2", "GND_IN", "GND", "left"),
                kt.SchPin("3", "GND_OUT", "GND", "right"),
                kt.SchPin("4", "VOUT_5V", "V5_REG", "right"),
            ],
            mpn="LM2596SX-5.0",
            manufacturer="TI",
            rating="7-35V -> 5V / 3A",
        )
    )

    # M1 & M2: Left/Right JGA25-370 Motor + Encoder Headers (2.50 mm JST-XH)
    sb.add_component(
        kt.SchComponent(
            "M1",
            "JGA25-370_LEFT",
            f"{LIB}:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical",
            [
                kt.SchPin("1", "M+", "M_L_A", "left"),
                kt.SchPin("2", "M-", "M_L_B", "left"),
                kt.SchPin("3", "ENC_VCC", "V3_3", "left"),
                kt.SchPin("4", "ENC_A", "ENC_L_A", "right"),
                kt.SchPin("5", "ENC_B", "ENC_L_B", "right"),
                kt.SchPin("6", "ENC_GND", "GND", "right"),
            ],
            mpn="JGA25-370",
            manufacturer="Generic",
            rating="7.4V / 1.5A",
        )
    )
    sb.add_component(
        kt.SchComponent(
            "M2",
            "JGA25-370_RIGHT",
            f"{LIB}:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical",
            [
                kt.SchPin("1", "M+", "M_R_A", "left"),
                kt.SchPin("2", "M-", "M_R_B", "left"),
                kt.SchPin("3", "ENC_VCC", "V3_3", "left"),
                kt.SchPin("4", "ENC_A", "ENC_R_A", "right"),
                kt.SchPin("5", "ENC_B", "ENC_R_B", "right"),
                kt.SchPin("6", "ENC_GND", "GND", "right"),
            ],
            mpn="JGA25-370",
            manufacturer="Generic",
            rating="7.4V / 1.5A",
        )
    )

    # Battery, ATO Blade Fuse (9.2mm pitch), Main Switch Terminal (5.08mm pitch), TVS, E-STOP
    sb.add_component(kt.SchComponent("J1", "XT60_2S_BAT", f"{LIB}:AMASS_XT60_Vertical", [kt.SchPin("1", "VBAT+", "VBAT_RAW", "left"), kt.SchPin("2", "GND", "GND", "right")], mpn="XT60-M", rating="2S 7.4V / 30A"))
    sb.add_component(kt.SchComponent("F1", "5A_ATO_FUSE", f"{LIB}:Fuseholder_Blade_ATO_Littelfuse_Pudenz_2_Pin", [kt.SchPin("1", "1", "VBAT_RAW", "left"), kt.SchPin("2", "2", "VBAT_FUSED", "right")], mpn="0287005", rating="32V / 5A ATO"))
    sb.add_component(kt.SchComponent("SW1", "MAIN_PWR_SW", f"{LIB}:TerminalBlock_bornier-2_P5.08mm", [kt.SchPin("1", "1", "VBAT_FUSED", "left"), kt.SchPin("2", "2", "VBAT_SW", "right")], mpn="R13-112 / TB-5.08", rating="10A / 5.08mm"))
    sb.add_component(kt.SchComponent("SW2", "ESTOP_NC_SW", f"{LIB}:PinHeader_1x02_P2.54mm", [kt.SchPin("1", "NC", "ESTOP_HW", "left"), kt.SchPin("2", "GND", "GND", "right")], mpn="ESTOP-16MM", rating="3.3V Logic"))
    sb.add_component(kt.SchComponent("D1", "SMBJ28A_TVS", f"{LIB}:D_SMB", [kt.SchPin("1", "K", "VBAT_SW", "left"), kt.SchPin("2", "A", "GND", "right")], mpn="SMBJ28A", rating="28V / 600W"))
    sb.add_component(kt.SchComponent("D2", "STATUS_LED", f"{LIB}:SMD_0805", [kt.SchPin("1", "A", "STATUS_LED", "left"), kt.SchPin("2", "K", "STAT_K", "right")], mpn="LED-0805", rating="2.1V / 20mA"))

    # Resistors & Caps
    res = [
        ("R1", "22k", "VBAT_SW", "VBAT_ADC"),
        ("R2", "10k", "VBAT_ADC", "GND"),
        ("R3", "10k", "TB6612_STBY", "GND"),
        ("R4", "4.7k", "V3_3", "I2C_SDA"),
        ("R5", "4.7k", "V3_3", "I2C_SCL"),
        ("R6", "10k", "V3_3", "ENC_L_A"),
        ("R7", "10k", "V3_3", "ENC_L_B"),
        ("R8", "10k", "V3_3", "ENC_R_A"),
        ("R9", "10k", "V3_3", "ENC_R_B"),
        ("R10", "10k", "V3_3", "ESTOP_HW"),
        ("R11", "330R", "STAT_K", "GND"),
    ]
    for ref, val, n1, n2 in res:
        sb.add_component(kt.SchComponent(ref, val, f"{LIB}:SMD_0805", [kt.SchPin("1", "1", n1, "left"), kt.SchPin("2", "2", n2, "right")], rating="50V / 0.125W"))

    sb.add_component(kt.SchComponent("C1", "100nF", f"{LIB}:SMD_0805", [kt.SchPin("1", "1", "VBAT_ADC", "left"), kt.SchPin("2", "2", "GND", "right")], rating="50V"))
    sb.add_component(kt.SchComponent("C2", "470uF_16V", f"{LIB}:CP_Radial_D8mm", [kt.SchPin("1", "+", "V5_REG", "left"), kt.SchPin("2", "-", "GND", "right")], rating="16V / 470uF"))
    for idx in range(3, 9):
        sb.add_component(kt.SchComponent(f"C{idx}", "100nF", f"{LIB}:SMD_0805", [kt.SchPin("1", "1", "V3_3", "left"), kt.SchPin("2", "2", "GND", "right")], rating="25V"))

    for h in ("H1", "H2", "H3", "H4"):
        sb.add_component(kt.SchComponent(h, "M3_Mount", f"{LIB}:MountingHole_3.2mm_M3_Pad_Via", [kt.SchPin("1", "GND", "GND", "left")], rating="M3"))

    sb.write(HW_DIR)

    # ---------------- PCB BUILD (100.0 mm x 80.0 mm) ----------------
    pb = kt.PCBBuilder(PROJ, LIB, 100.0, 80.0, "ROVERSAR CARRIER v1.1 (E-STOP + 5xToF)", sb.root_uuid)

    fp_esp = kt.fp_dual_row_module("ESP32_DevKit_30P", 15, 25.4)
    fp_tb = kt.fp_dual_row_module("Breakout_16Pin_DIP", 8, 15.24)
    fp_h6 = kt.fp_jst_xh_1xN("JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical", 6)
    fp_h5 = kt.fp_jst_xh_1xN("JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical", 5)
    fp_h4 = kt.fp_jst_xh_1xN("JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", 4)
    fp_h2 = kt.fp_pin_header_1xN("PinHeader_1x02_P2.54mm", 2)
    fp_fuse = kt.fp_ato_blade_fuse()
    fp_pwr_sw = kt.fp_power_terminal_508_2p()
    fp_xt = kt.fp_xt60("AMASS_XT60_Vertical")
    fp_0805 = kt.fp_smd_2pin("SMD_0805", 1.825, 1.025, 1.4, "Resistor_SMD.3dshapes/R_0805_2012Metric.step")
    fp_smb = kt.fp_smd_2pin("D_SMB", 4.3, 2.0, 2.4, "Diode_SMD.3dshapes/D_SMB.step")
    fp_buck = kt.FootprintDef(
        "Buck_4Pin_Module",
        "through_hole",
        [
            kt.PadDef("1", "thru_hole", "rect", -6.0, -4.0, 1.8, 1.8, 1.0, ("*.Cu", "*.Mask")),
            kt.PadDef("2", "thru_hole", "circle", -6.0, +4.0, 1.8, 1.8, 1.0, ("*.Cu", "*.Mask")),
            kt.PadDef("3", "thru_hole", "circle", +6.0, +4.0, 1.8, 1.8, 1.0, ("*.Cu", "*.Mask")),
            kt.PadDef("4", "thru_hole", "circle", +6.0, -4.0, 1.8, 1.8, 1.0, ("*.Cu", "*.Mask")),
        ],
        15.0,
        11.0,
    )
    fp_cp8 = kt.FootprintDef(
        "CP_Radial_D8mm",
        "through_hole",
        [
            kt.PadDef("1", "thru_hole", "rect", -1.75, 0.0, 1.8, 1.8, 0.9, ("*.Cu", "*.Mask")),
            kt.PadDef("2", "thru_hole", "circle", +1.75, 0.0, 1.8, 1.8, 0.9, ("*.Cu", "*.Mask")),
        ],
        8.2,
        8.2,
    )
    fp_mh = kt.fp_mounting_hole()

    for ref, (mx, my) in [("H1", (4.0, 4.0)), ("H2", (96.0, 4.0)), ("H3", (96.0, 76.0)), ("H4", (4.0, 76.0))]:
        pb.add_component(kt.PcbComponent(ref, "M3_Mount", fp_mh, mx, my, 0.0, {"1": "GND"}))

    pb.add_component(kt.PcbComponent("U1", "ESP32-WROOM-32D", fp_esp, 48.0, 44.0, 0.0, {
        "2": "ENC_R_A", "3": "ENC_R_B", "4": "ENC_L_A", "5": "ENC_L_B", "6": "ESTOP_HW",
        "7": "VBAT_ADC", "8": "TB6612_PWMA", "9": "AIN1", "10": "AIN2", "11": "TOF_XSHUT_1",
        "13": "TOF_XSHUT_0", "14": "GND", "15": "V5_REG", "16": "V3_3", "17": "GND",
        "19": "STATUS_LED", "20": "TB6612_STBY", "21": "TOF_XSHUT_2", "22": "TOF_XSHUT_3",
        "23": "TOF_XSHUT_4", "24": "BIN1", "25": "TB6612_PWMB", "26": "I2C_SDA",
        "27": "CAM_UART_TX", "28": "CAM_UART_RX", "29": "I2C_SCL", "30": "BIN2",
    }))
    pb.add_component(kt.PcbComponent("U2", "TB6612FNG", fp_tb, 80.0, 42.0, 0.0, {
        "1": "VBAT_SW", "2": "V3_3", "3": "GND", "4": "TB6612_PWMA", "5": "AIN1", "6": "AIN2",
        "7": "TB6612_STBY", "8": "TB6612_PWMB", "9": "BIN1", "10": "BIN2", "11": "M_L_A",
        "12": "M_L_B", "13": "M_R_A", "14": "M_R_B", "15": "GND", "16": "GND",
    }))
    pb.add_component(kt.PcbComponent("J1", "XT60_2S_BAT", fp_xt, 7.5, 19.0, 0.0, {"1": "VBAT_RAW", "2": "GND"}))
    pb.add_component(kt.PcbComponent("F1", "5A_ATO_FUSE", fp_fuse, 23.0, 13.0, 0.0, {"1": "VBAT_RAW", "2": "VBAT_FUSED"}))
    pb.add_component(kt.PcbComponent("SW1", "MAIN_PWR_SW", fp_pwr_sw, 21.5, 24.5, 0.0, {"1": "VBAT_FUSED", "2": "VBAT_SW"}))
    pb.add_component(kt.PcbComponent("D1", "SMBJ28A_TVS", fp_smb, 18.0, 35.0, 0.0, {"1": "VBAT_SW", "2": "GND"}))
    pb.add_component(kt.PcbComponent("U10", "LM2596-5.0", fp_buck, 12.0, 52.0, 0.0, {"1": "VBAT_SW", "2": "GND", "3": "GND", "4": "V5_REG"}))
    pb.add_component(kt.PcbComponent("C2", "470uF_16V", fp_cp8, 25.5, 53.0, 0.0, {"1": "V5_REG", "2": "GND"}))
    pb.add_component(kt.PcbComponent("U9", "ESP32-CAM", fp_h4, 15.0, 69.0, 0.0, {"1": "V5_REG", "2": "GND", "3": "CAM_UART_TX", "4": "CAM_UART_RX"}))
    pb.add_component(kt.PcbComponent("SW2", "ESTOP_NC_SW", fp_h2, 24.0, 69.0, 0.0, {"1": "ESTOP_HW", "2": "GND"}))

    pb.add_component(kt.PcbComponent("U3", "MPU6050-GY521", fp_h4, 38.0, 72.5, 0.0, {"1": "V3_3", "2": "GND", "3": "I2C_SCL", "4": "I2C_SDA"}))
    for i in range(5):
        pb.add_component(kt.PcbComponent(f"U{4 + i}", f"VL53L0X_{i}", fp_h5, 41.0 + i * 9.0, 8.5, 0.0, {
            "1": "V3_3", "2": "GND", "3": "I2C_SCL", "4": "I2C_SDA", "5": f"TOF_XSHUT_{i}",
        }))

    pb.add_component(kt.PcbComponent("M1", "JGA25-370_LEFT", fp_h6, 94.0, 26.0, 0.0, {
        "1": "M_L_A", "2": "M_L_B", "3": "V3_3", "4": "ENC_L_A", "5": "ENC_L_B", "6": "GND",
    }))
    pb.add_component(kt.PcbComponent("M2", "JGA25-370_RIGHT", fp_h6, 94.0, 56.0, 0.0, {
        "1": "M_R_A", "2": "M_R_B", "3": "V3_3", "4": "ENC_R_A", "5": "ENC_R_B", "6": "GND",
    }))

    pb.add_component(kt.PcbComponent("D2", "STATUS_LED", fp_0805, 68.0, 68.0, 0.0, {"1": "STATUS_LED", "2": "STAT_K"}))

    passives = [
        ("R1", "22k", 27.5, 34.0, {"1": "VBAT_SW", "2": "VBAT_ADC"}),
        ("R2", "10k", 27.5, 39.0, {"1": "VBAT_ADC", "2": "GND"}),
        ("C1", "100nF", 27.5, 44.0, {"1": "VBAT_ADC", "2": "GND"}),
        ("R3", "10k", 76.0, 58.0, {"1": "TB6612_STBY", "2": "GND"}),
        ("R4", "4.7k", 46.0, 68.5, {"1": "V3_3", "2": "I2C_SDA"}),
        ("R5", "4.7k", 52.0, 68.5, {"1": "V3_3", "2": "I2C_SCL"}),
        ("R6", "10k", 72.0, 20.5, {"1": "V3_3", "2": "ENC_L_A"}),
        ("R7", "10k", 78.0, 20.5, {"1": "V3_3", "2": "ENC_L_B"}),
        ("R8", "10k", 72.0, 62.0, {"1": "V3_3", "2": "ENC_R_A"}),
        ("R9", "10k", 79.0, 63.5, {"1": "V3_3", "2": "ENC_R_B"}),
        ("R10", "10k", 31.0, 68.0, {"1": "V3_3", "2": "ESTOP_HW"}),
        ("R11", "330R", 68.0, 74.0, {"1": "STAT_K", "2": "GND"}),
        ("C3", "100nF", 30.5, 22.0, {"1": "V3_3", "2": "GND"}),
        ("C4", "100nF", 84.5, 7.5, {"1": "V3_3", "2": "GND"}),
        ("C5", "100nF", 84.5, 13.0, {"1": "V3_3", "2": "GND"}),
        ("C6", "100nF", 84.5, 18.5, {"1": "V3_3", "2": "GND"}),
        ("C7", "100nF", 84.5, 24.0, {"1": "V3_3", "2": "GND"}),
        ("C8", "100nF", 46.0, 74.0, {"1": "V3_3", "2": "GND"}),
    ]
    for ref, val, px, py, pnets in passives:
        pb.add_component(kt.PcbComponent(ref, val, fp_0805, px, py, 0.0, pnets))

    kt.sync_pcb_with_schematic(sb, pb)
    overlaps = kt.check_courtyards(pb)
    assert not overlaps, f"Courtyard overlaps: {overlaps}"

    # High-current 2.5 mm dual-layer main power bus (VBAT_RAW -> F1 -> SW1 -> VBAT_SW)
    j1_p1 = pb.pad_abs_xy("J1", "1")
    f1_p1 = pb.pad_abs_xy("F1", "1")
    f1_p2 = pb.pad_abs_xy("F1", "2")
    sw1_p1 = pb.pad_abs_xy("SW1", "1")
    sw1_p2 = pb.pad_abs_xy("SW1", "2")
    pb.add_track("VBAT_RAW", 2.5, [j1_p1, (18.4, j1_p1[1]), f1_p1], "F.Cu")
    pb.add_track("VBAT_FUSED", 2.5, [f1_p2, (27.6, 19.0), (18.96, 19.0), sw1_p1], "F.Cu")
    pb.add_track("VBAT_SW", 2.0, [sw1_p2, (24.04, 30.0)], "F.Cu")
    pb.add_track("VBAT_SW", 2.0, [sw1_p2, (24.04, 30.0)], "B.Cu")
    pb.add_via("VBAT_SW", 24.04, 30.0, 1.0, 0.5)
    pb.add_track("TB6612_STBY", 0.25, [pb.pad_abs_xy("U2", "7"), (70.0, 48.35), (70.0, 56.0), (75.088, 56.0), pb.pad_abs_xy("R3", "1")], "F.Cu")

    router = kt.AutoRouter2Layer(pb, grid_mm=0.5, clearance_mm=0.22)
    route_res = router.route_all(net_widths={
        "VBAT_RAW": 2.0,
        "VBAT_FUSED": 2.0,
        "VBAT_SW": 1.5,
        "V5_REG": 0.8,
        "V3_3": 0.6,
        "M_L_A": 1.0,
        "M_L_B": 1.0,
        "M_R_A": 1.0,
        "M_R_B": 1.0,
    })
    print("RoverSAR AutoRouter failed list:", route_res["failed"])
    pb.write(HW_DIR)

    summary = kt.build_and_verify_project(HW_DIR, PROJ, gerber_dir=HW_DIR / "gerbers", models_out_dir=HW_DIR / "3d-models")
    print("RoverSAR KiCad Summary:", summary)
    kt.run_cli(
        [
            "sch", "export", "bom",
            "--fields", "Reference,Value,Footprint,MPN,Manufacturer,Rating,QUANTITY",
            "--labels", "Designator,Value,Footprint,MPN,Manufacturer,Rating,Qty",
            "--output", str(HW_DIR / "BOM-kicad.csv"),
            str(HW_DIR / f"{PROJ}.kicad_sch"),
        ],
        check=True,
    )
    if summary["erc_violations"] or summary["drc_violations"] or summary["drc_unconnected"] or summary["drc_parity_issues"]:
        raise SystemExit(2)


if __name__ == "__main__":
    build()
