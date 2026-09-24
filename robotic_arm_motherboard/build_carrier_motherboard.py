#!/usr/bin/env python3
"""
Builds the definitive ROBOMOTION 5-DOF Robotic Arm Carrier Motherboard PCB.
Key Principles:
1. NO duplicate servo connectors on motherboard. Servos plug directly into
   the physical PCA9685 module's existing 16x3 header.
2. PCA9685 is positioned along the RIGHT EDGE of the motherboard with its
   16x3 servo header bank facing directly outward off the PCB edge, 100% accessible.
3. 4x M2.5 standoff mounting holes (MH_PCA1-4) on the PCB mechanically secure the
   PCA9685 module using 11mm standoffs.
4. Motherboard provides 1x6 female control socket J_PCA9685_CTRL (GND, OE, SCL, SDA, VCC, V+)
   and 1x6 female auxiliary socket J_PCA9685_AUX for mechanical retention & redundant power.
5. ESP32 DevKit V1 removable female socket (25.4mm / 1.0" row pitch) on top-left.
6. High-current power stage (XT30, 5A fuse, Schottky, 1000uF cap, AMS1117-3.3 LDO) in upper-center.
7. VL53L1X #1 & #2 sockets + I2C expansion + pull-ups in bottom-left.
8. Safety & debug headers (UART, GPIO expansion, E-Stop, Servo Enable, LEDs) in lower-center.
9. 4x M3 motherboard mounting holes.
10. Solid B.Cu ground plane, F.Cu ground fill, and wide 2.0mm SERVO_V+ power bus.
"""

import pcbnew
import os

OUTPUT_DIR = "/home/chakradhar/Nurobots_kerela/robotic_arm_motherboard"
PCB_PATH = os.path.join(OUTPUT_DIR, "robotic_arm_motherboard.kicad_pcb")

board = pcbnew.BOARD()

# 1. Edge.Cuts: 100mm x 80mm with rounded corners (r=2.5mm)
w, h, r = 100.0, 80.0, 2.5
thickness = int(0.15 * 1000000)

lines = [
    ((r, 0), (w - r, 0)),
    ((w, r), (w, h - r)),
    ((w - r, h), (r, h)),
    ((0, h - r), (0, r))
]
for p1, p2 in lines:
    seg = pcbnew.PCB_SHAPE(board)
    seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
    seg.SetStart(pcbnew.VECTOR2I_MM(p1[0], p1[1]))
    seg.SetEnd(pcbnew.VECTOR2I_MM(p2[0], p2[1]))
    seg.SetLayer(pcbnew.Edge_Cuts)
    seg.SetWidth(thickness)
    board.Add(seg)

arcs = [
    ((w - r, r), (w - r, 0), (w, r)),
    ((w - r, h - r), (w, h - r), (w - r, h)),
    ((r, h - r), (r, h), (0, h - r)),
    ((r, r), (0, r), (r, 0))
]
for center, start, end in arcs:
    arc = pcbnew.PCB_SHAPE(board)
    arc.SetShape(pcbnew.SHAPE_T_ARC)
    arc.SetCenter(pcbnew.VECTOR2I_MM(center[0], center[1]))
    arc.SetStart(pcbnew.VECTOR2I_MM(start[0], start[1]))
    arc.SetEnd(pcbnew.VECTOR2I_MM(end[0], end[1]))
    arc.SetLayer(pcbnew.Edge_Cuts)
    arc.SetWidth(thickness)
    board.Add(arc)

# 2. Nets Definition
net_names = {
    1: "GND", 2: "SERVO_V+", 3: "+3V3", 4: "5V_LOGIC",
    5: "SDA", 6: "SCL", 7: "XSHUT1", 8: "XSHUT2",
    9: "OE", 10: "UART_TX", 11: "UART_RX",
    12: "E_STOP", 13: "SERVO_EN"
}
for i, g in enumerate([32, 33, 25, 26, 27, 14, 12, 13], 14):
    net_names[i] = f"GPIO{g}"

nets = {}
for code, name in net_names.items():
    net = pcbnew.NETINFO_ITEM(board, name, code)
    board.Add(net)
    nets[name] = net

def place_footprint(lib, name, ref, x, y, rot=0):
    fp = pcbnew.FootprintLoad(lib, name)
    if not fp:
        raise RuntimeError(f"Failed to load {lib}:{name}")
    fp.SetReference(ref)
    fp.SetPosition(pcbnew.VECTOR2I_MM(x, y))
    fp.SetOrientation(pcbnew.EDA_ANGLE(rot, pcbnew.DEGREES_T))
    board.Add(fp)
    return fp

fp_mount    = "/usr/share/kicad/footprints/MountingHole.pretty"
fp_conn_skt = "/usr/share/kicad/footprints/Connector_PinSocket_2.54mm.pretty"
fp_conn_hdr = "/usr/share/kicad/footprints/Connector_PinHeader_2.54mm.pretty"
fp_amass    = "/usr/share/kicad/footprints/Connector_AMASS.pretty"
fp_fuse     = "/usr/share/kicad/footprints/Fuse.pretty"
fp_diode    = "/usr/share/kicad/footprints/Diode_SMD.pretty"
fp_cap_tht  = "/usr/share/kicad/footprints/Capacitor_THT.pretty"
fp_cap_smd  = "/usr/share/kicad/footprints/Capacitor_SMD.pretty"
fp_sot      = "/usr/share/kicad/footprints/Package_TO_SOT_SMD.pretty"
fp_res      = "/usr/share/kicad/footprints/Resistor_SMD.pretty"
fp_led      = "/usr/share/kicad/footprints/LED_SMD.pretty"
fp_tp       = "/usr/share/kicad/footprints/TestPoint.pretty"

# 3. Four M3 Motherboard Corner Mounting Holes
place_footprint(fp_mount, "MountingHole_3.2mm_M3_Pad", "MH1", 4.0, 4.0)
place_footprint(fp_mount, "MountingHole_3.2mm_M3_Pad", "MH2", 96.0, 4.0)
place_footprint(fp_mount, "MountingHole_3.2mm_M3_Pad", "MH3", 4.0, 76.0)
place_footprint(fp_mount, "MountingHole_3.2mm_M3_Pad", "MH4", 96.0, 76.0)

# 4. PCA9685 MODULE MECHANICAL STANDOFF MOUNT ON PCB (4x M2.5, 55.88mm x 19.05mm spacing)
# Module is positioned along the RIGHT EDGE (X = 72..97.4, Y = 8.5..71.0)
# Center X = 84.0, Center Y = 39.5
# Hole spacing: dx = 19.05 mm, dy = 55.88 mm
place_footprint(fp_mount, "MountingHole_2.7mm_M2.5_Pad", "MH_PCA1", 74.48, 11.56)
place_footprint(fp_mount, "MountingHole_2.7mm_M2.5_Pad", "MH_PCA2", 93.52, 11.56)
place_footprint(fp_mount, "MountingHole_2.7mm_M2.5_Pad", "MH_PCA3", 74.48, 67.44)
place_footprint(fp_mount, "MountingHole_2.7mm_M2.5_Pad", "MH_PCA4", 93.52, 67.44)

# 5. PCA9685 Female Socket Headers (Removable Module Interface)
# Top 1x6 control socket: GND, OE, SCL, SDA, VCC, V+
place_footprint(fp_conn_skt, "PinSocket_1x06_P2.54mm_Vertical", "J_PCA9685_CTRL", 84.0, 16.0, rot=90)
# Bottom 1x6 auxiliary socket (mechanical support & redundant GND/V+)
place_footprint(fp_conn_skt, "PinSocket_1x06_P2.54mm_Vertical", "J_PCA9685_AUX", 84.0, 63.0, rot=90)

# 6. ESP32 DevKit V1 Removable Female Socket (Top-Left)
# 25.4mm (1.0") row spacing, vertical span: 11.0 to 46.56 mm
place_footprint(fp_conn_skt, "PinSocket_1x15_P2.54mm_Vertical", "J_ESP32_LEFT", 8.0, 11.0)
place_footprint(fp_conn_skt, "PinSocket_1x15_P2.54mm_Vertical", "J_ESP32_RIGHT", 33.4, 11.0)

# 7. High-Current Power Stage (Upper-Center, X=42..68, Y=8..38)
place_footprint(fp_amass, "AMASS_XT30U-F_1x02_P5.0mm_Vertical", "J_SERVO_PWR", 48.0, 10.0)
place_footprint(fp_fuse, "Fuse_1206_3216Metric", "F1", 58.0, 10.0)
place_footprint(fp_diode, "D_SMA", "D1", 58.0, 16.0)
place_footprint(fp_cap_tht, "CP_Radial_D10.0mm_P5.00mm", "C1", 48.0, 22.0)
place_footprint(fp_cap_smd, "C_0805_2012Metric", "C2", 58.0, 22.0)
place_footprint(fp_cap_smd, "C_0603_1608Metric", "C3", 64.0, 22.0)
place_footprint(fp_sot, "SOT-223-3_TabPin2", "U1", 58.0, 31.0)
place_footprint(fp_cap_smd, "C_0603_1608Metric", "C4", 48.0, 34.0)
place_footprint(fp_cap_smd, "C_0603_1608Metric", "C5", 52.0, 34.0)
place_footprint(fp_cap_smd, "C_0603_1608Metric", "C6", 44.0, 34.0)

# 8. Sensors & I2C (Bottom-Left, X=8..36, Y=54..68)
place_footprint(fp_conn_skt, "PinSocket_1x05_P2.54mm_Vertical", "J_VL53_1", 10.0, 56.0)
place_footprint(fp_conn_skt, "PinSocket_1x05_P2.54mm_Vertical", "J_VL53_2", 19.0, 56.0)
place_footprint(fp_res, "R_0603_1608Metric", "R1", 26.0, 54.0)
place_footprint(fp_res, "R_0603_1608Metric", "R2", 26.0, 58.0)
place_footprint(fp_conn_skt, "PinSocket_1x04_P2.54mm_Vertical", "J_I2C_EXP", 32.0, 56.0)

# 9. Test Points (Neatly aligned at Y=70.0mm in bottom-left)
for i in range(7):
    place_footprint(fp_tp, "TestPoint_Pad_D1.5mm", f"TP{i+1}", 8.0 + i*5.0, 70.0)

# 10. Safety, Debug & GPIO Expansion (Lower-Center, X=42..68, Y=46..72)
place_footprint(fp_conn_hdr, "PinHeader_1x04_P2.54mm_Vertical", "J_UART", 45.0, 48.0)
place_footprint(fp_conn_hdr, "PinHeader_1x10_P2.54mm_Vertical", "J_GPIO_EXP", 56.0, 48.0)
place_footprint(fp_conn_hdr, "PinHeader_1x02_P2.54mm_Vertical", "J_ESTOP", 45.0, 62.0)
place_footprint(fp_conn_hdr, "PinHeader_1x02_P2.54mm_Vertical", "J_SRV_EN", 52.0, 62.0)
place_footprint(fp_res, "R_0603_1608Metric", "R7", 58.0, 62.0)

# 11. Status LEDs (Vertical column at X=65.0)
led_y = [46.0, 52.0, 58.0, 64.0]
for i in range(4):
    place_footprint(fp_led, "LED_0805_2012Metric", f"LED{i+1}", 65.0, led_y[i])
    place_footprint(fp_res, "R_0603_1608Metric", f"R{i+3}", 65.0, led_y[i] + 3.0)

# Net Assignment to Pads
def assign_net(ref, pad_num, net_name):
    fp = board.FindFootprintByReference(ref)
    if fp and net_name in nets:
        pad = fp.FindPadByNumber(str(pad_num))
        if pad:
            pad.SetNetCode(nets[net_name].GetNetCode())

# Mounting holes to GND
for mh in ["MH1", "MH2", "MH3", "MH4", "MH_PCA1", "MH_PCA2", "MH_PCA3", "MH_PCA4"]:
    assign_net(mh, "1", "GND")

# ESP32 LEFT row
esp_l_nets = ["+3V3", "", "", "", "", "", "GPIO32", "GPIO33", "GPIO25", "GPIO26", "GPIO27", "GPIO14", "GPIO12", "GND", "GPIO13"]
for p, n in enumerate(esp_l_nets, 1):
    if n: assign_net("J_ESP32_LEFT", p, n)

# ESP32 RIGHT row
esp_r_nets = ["5V_LOGIC", "GND", "", "", "", "XSHUT1", "XSHUT2", "", "", "", "SDA", "UART_RX", "UART_TX", "SCL", ""]
for p, n in enumerate(esp_r_nets, 1):
    if n: assign_net("J_ESP32_RIGHT", p, n)

# PCA9685 Control Socket (GND, OE, SCL, SDA, VCC, V+)
pca_ctrl_nets = ["GND", "OE", "SCL", "SDA", "+3V3", "SERVO_V+"]
for p, n in enumerate(pca_ctrl_nets, 1):
    assign_net("J_PCA9685_CTRL", p, n)

# PCA9685 Auxiliary Socket (GND, OE, SCL, SDA, VCC, V+)
pca_aux_nets = ["GND", "OE", "SCL", "SDA", "+3V3", "SERVO_V+"]
for p, n in enumerate(pca_aux_nets, 1):
    assign_net("J_PCA9685_AUX", p, n)

# Sensors
vl_nets = ["+3V3", "GND", "SDA", "SCL"]
for p, n in enumerate(vl_nets, 1):
    assign_net("J_VL53_1", p, n)
    assign_net("J_VL53_2", p, n)
    assign_net("J_I2C_EXP", p, n)
assign_net("J_VL53_1", 5, "XSHUT1")
assign_net("J_VL53_2", 5, "XSHUT2")

# I2C Pullups
assign_net("R1", 1, "+3V3")
assign_net("R1", 2, "SDA")
assign_net("R2", 1, "+3V3")
assign_net("R2", 2, "SCL")

# Power Input & Protection
assign_net("J_SERVO_PWR", 1, "SERVO_V+")
assign_net("J_SERVO_PWR", 2, "GND")
assign_net("F1", 1, "SERVO_V+")
assign_net("F1", 2, "SERVO_V+")
assign_net("D1", 1, "GND")
assign_net("D1", 2, "SERVO_V+")
assign_net("C1", 1, "SERVO_V+")
assign_net("C1", 2, "GND")
assign_net("C2", 1, "SERVO_V+")
assign_net("C2", 2, "GND")
assign_net("C3", 1, "SERVO_V+")
assign_net("C3", 2, "GND")

# AMS1117-3.3 LDO
assign_net("U1", 1, "GND")
assign_net("U1", 2, "+3V3")
assign_net("U1", 3, "SERVO_V+")
assign_net("U1", 4, "+3V3")
assign_net("C4", 1, "+3V3")
assign_net("C4", 2, "GND")
assign_net("C5", 1, "+3V3")
assign_net("C5", 2, "GND")
assign_net("C6", 1, "+3V3")
assign_net("C6", 2, "GND")

# Debug & Safety
assign_net("J_UART", 1, "+3V3")
assign_net("J_UART", 2, "GND")
assign_net("J_UART", 3, "UART_TX")
assign_net("J_UART", 4, "UART_RX")
assign_net("J_ESTOP", 1, "E_STOP")
assign_net("J_ESTOP", 2, "GND")
assign_net("J_SRV_EN", 1, "SERVO_EN")
assign_net("J_SRV_EN", 2, "GND")
assign_net("R7", 1, "+3V3")
assign_net("R7", 2, "OE")

# GPIO Expansion
gpio_pins = [32, 33, 25, 26, 27, 14, 12, 13]
for idx, g in enumerate(gpio_pins, 1):
    assign_net("J_GPIO_EXP", idx, f"GPIO{g}")
assign_net("J_GPIO_EXP", 9, "+3V3")
assign_net("J_GPIO_EXP", 10, "GND")

# Test Points
tp_map = ["+3V3", "5V_LOGIC", "SERVO_V+", "GND", "SDA", "SCL", "OE"]
for i, n in enumerate(tp_map, 1):
    assign_net(f"TP{i}", 1, n)

# LEDs
assign_net("LED1", 1, "+3V3")
assign_net("LED1", 2, "GND")
assign_net("LED2", 1, "SERVO_V+")
assign_net("LED2", 2, "GND")
assign_net("LED3", 1, "+3V3")
assign_net("LED3", 2, "GND")
assign_net("LED4", 1, "OE")
assign_net("LED4", 2, "GND")

# Silkscreen helpers
def add_text(b, txt, x, y, layer=pcbnew.F_SilkS, size=1.0, thickness=0.15):
    t = pcbnew.PCB_TEXT(b)
    t.SetText(txt)
    t.SetPosition(pcbnew.VECTOR2I_MM(x, y))
    t.SetTextSize(pcbnew.VECTOR2I_MM(size, size))
    t.SetTextThickness(int(thickness * 1000000))
    t.SetLayer(layer)
    b.Add(t)

def add_box(b, x, y, bw, bh, layer=pcbnew.F_SilkS, thickness=0.15):
    coords = [
        ((x, y), (x+bw, y)),
        ((x+bw, y), (x+bw, y+bh)),
        ((x+bw, y+bh), (x, y+bh)),
        ((x, y+bh), (x, y))
    ]
    for p1, p2 in coords:
        seg = pcbnew.PCB_SHAPE(b)
        seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(pcbnew.VECTOR2I_MM(p1[0], p1[1]))
        seg.SetEnd(pcbnew.VECTOR2I_MM(p2[0], p2[1]))
        seg.SetLayer(layer)
        seg.SetWidth(int(thickness * 1000000))
        b.Add(seg)

# Titles
add_text(board, "ROBOMOTION 5-DOF ROBOT ARM", 20.0, 3.5, size=1.5, thickness=0.25)
add_text(board, "MODULAR CARRIER MOTHERBOARD", 20.0, 5.8, size=0.9, thickness=0.16)

# ESP32 Box & Antenna Keepout
add_box(board, 5.0, 9.5, 31.0, 39.0)
add_text(board, "ESP32 DEVKIT V1", 20.5, 11.5, size=1.0, thickness=0.18)
add_text(board, "25.4mm ROW PITCH", 20.5, 13.5, size=0.8, thickness=0.14)
add_text(board, "USB", 20.5, 46.5, size=0.8, thickness=0.14)

# Antenna Keepout Rule Area
kz = pcbnew.ZONE(board)
kz.SetLayer(pcbnew.F_Cu)
kz.SetIsRuleArea(True)
kz.SetDoNotAllowCopperPour(True)
kz.SetDoNotAllowTracks(True)
kz.SetDoNotAllowVias(True)
poly = pcbnew.SHAPE_LINE_CHAIN()
poly.Append(pcbnew.VECTOR2I_MM(5, 0.5))
poly.Append(pcbnew.VECTOR2I_MM(35, 0.5))
poly.Append(pcbnew.VECTOR2I_MM(35, 9.0))
poly.Append(pcbnew.VECTOR2I_MM(5, 9.0))
poly.SetClosed(True)
kz.AddPolygon(poly)
board.Add(kz)

# PCA9685 Module Outline on Right Edge (X=71.25..96.75, Y=8.25..70.75)
add_box(board, 71.25, 8.25, 25.5, 62.5, thickness=0.2)
add_text(board, "PCA9685 MODULE", 84.0, 20.0, size=0.9, thickness=0.16)
add_text(board, "I2C: 0x40", 84.0, 22.0, size=0.8, thickness=0.14)
add_text(board, "4x M2.5 STANDOFFS", 84.0, 24.0, size=0.75, thickness=0.14)
add_text(board, "USE 11mm HEIGHT", 84.0, 26.0, size=0.75, thickness=0.14)

# Silkscreen: Indication of Servo Headers on Module Edge
add_text(board, "16x3 SERVO HEADER BANK (ON MODULE) ---> PLUG SERVOS HERE", 84.0, 42.0, size=0.7, thickness=0.13)

# Channel Map Silkscreen alongside the PCA9685 module
ch_labels = [
    "CH0 BASE", "CH1 SHLDR 1", "CH2 SHLDR 2", "CH3 ELBOW 1",
    "CH4 ELBOW 2", "CH5 WRIST", "CH6 GRIPPER", "CH7 LIDAR",
    "CH8-15 AUX"
]
for idx, lbl in enumerate(ch_labels):
    add_text(board, lbl, 68.0, 24.0 + idx*3.5, size=0.65, thickness=0.12)

# Power section labels
add_text(board, "XT30 5-6V", 48.0, 5.0, size=0.8, thickness=0.15)
add_text(board, "SERVO PWR", 48.0, 6.8, size=0.75, thickness=0.14)

# Sensors labels
add_text(board, "VL53 #1", 10.0, 51.5, size=0.75, thickness=0.14)
add_text(board, "VL53 #2", 19.0, 51.5, size=0.75, thickness=0.14)
add_text(board, "I2C EXP", 32.0, 51.5, size=0.75, thickness=0.14)

# Debug & Safety labels
add_text(board, "UART", 45.0, 43.5, size=0.75, thickness=0.14)
add_text(board, "GPIO EXP", 56.0, 43.5, size=0.75, thickness=0.14)
add_text(board, "E-STOP", 45.0, 58.0, size=0.75, thickness=0.14)
add_text(board, "SRV_EN", 52.0, 58.0, size=0.75, thickness=0.14)

# Test point labels
add_text(board, "TP1:3V3 TP2:5V TP3:SRV_V+ TP4:GND TP5:SDA TP6:SCL TP7:OE", 24.0, 74.5, size=0.65, thickness=0.12)

# Solid B.Cu Ground Plane
z_gnd = pcbnew.ZONE(board)
z_gnd.SetLayer(pcbnew.B_Cu)
z_gnd.SetNetCode(nets["GND"].GetNetCode())
p_gnd = pcbnew.SHAPE_LINE_CHAIN()
p_gnd.Append(pcbnew.VECTOR2I_MM(1, 1))
p_gnd.Append(pcbnew.VECTOR2I_MM(99, 1))
p_gnd.Append(pcbnew.VECTOR2I_MM(99, 79))
p_gnd.Append(pcbnew.VECTOR2I_MM(1, 79))
p_gnd.SetClosed(True)
z_gnd.AddPolygon(p_gnd)
board.Add(z_gnd)

# Solid F.Cu Ground Plane
z_fgnd = pcbnew.ZONE(board)
z_fgnd.SetLayer(pcbnew.F_Cu)
z_fgnd.SetNetCode(nets["GND"].GetNetCode())
p_fgnd = pcbnew.SHAPE_LINE_CHAIN()
p_fgnd.Append(pcbnew.VECTOR2I_MM(1, 10))
p_fgnd.Append(pcbnew.VECTOR2I_MM(70, 10))
p_fgnd.Append(pcbnew.VECTOR2I_MM(70, 78))
p_fgnd.Append(pcbnew.VECTOR2I_MM(1, 78))
p_fgnd.SetClosed(True)
z_fgnd.AddPolygon(p_fgnd)
board.Add(z_fgnd)

# Tracks
def add_track(b, x1, y1, x2, y2, net_name, width_mm, layer=pcbnew.F_Cu):
    t = pcbnew.PCB_TRACK(b)
    t.SetStart(pcbnew.VECTOR2I_MM(x1, y1))
    t.SetEnd(pcbnew.VECTOR2I_MM(x2, y2))
    t.SetWidth(int(width_mm * 1000000))
    t.SetLayer(layer)
    if net_name in nets:
        t.SetNetCode(nets[net_name].GetNetCode())
    b.Add(t)

# Main 2.0mm High-Current Power Bus:
# XT30 Pin 1 (48, 10) -> F1 (58, 10) -> D1 (58, 16) -> C1 (48, 22) -> straight to PCA9685 V+ pin (84, 16)
add_track(board, 48.0, 10.0, 58.0, 10.0, "SERVO_V+", 2.0)
add_track(board, 58.0, 10.0, 58.0, 16.0, "SERVO_V+", 2.0)
add_track(board, 58.0, 16.0, 48.0, 22.0, "SERVO_V+", 2.0)
add_track(board, 58.0, 16.0, 80.0, 16.0, "SERVO_V+", 2.0) # Feed to J_PCA9685_CTRL V+ pin!

# 0.5mm +3V3 Power Routing
# U1 VO (58, 31) -> ESP32 3V3 (8, 11) & PCA9685 VCC (84, 16)
add_track(board, 58.0, 31.0, 58.0, 26.0, "+3V3", 0.5)
add_track(board, 58.0, 26.0, 33.4, 11.0, "+3V3", 0.5)
add_track(board, 58.0, 26.0, 82.0, 16.0, "+3V3", 0.5)

# I2C Signal Routing (0.3mm):
# ESP32 SDA (GPIO21, 33.4, 36.4) -> PCA9685 SDA (84, 16)
add_track(board, 33.4, 36.4, 42.0, 36.4, "SDA", 0.3)
add_track(board, 42.0, 36.4, 78.0, 16.0, "SDA", 0.3)
# ESP32 SCL (GPIO22, 33.4, 44.0) -> PCA9685 SCL (84, 16)
add_track(board, 33.4, 44.0, 42.0, 44.0, "SCL", 0.3)
add_track(board, 42.0, 44.0, 76.0, 16.0, "SCL", 0.3)

# Save board
board.Save(PCB_PATH)
print(f"Carrier Motherboard successfully saved to: {PCB_PATH}")
print(f"Total Footprints: {len(board.GetFootprints())}")
print(f"Total Nets: {len(board.GetNetInfo().NetsByName())}")
print(f"Total Tracks: {len(board.GetTracks())}")
