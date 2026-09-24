#!/usr/bin/env python3
"""
Generates:
1. pca9685_mount_bracket.scad (OpenSCAD 3D model)
2. pca9685_mount_bracket.stl  (Binary STL file for 3D printing)

Mount specifications:
- Board: PCA9685 16-Channel Servo Driver Module
- Hole spacing: 55.88mm (2.20 in) x 19.05mm (0.75 in)
- Base footprint: 66mm x 29mm
- Standoff height: 11.0mm (matches 8.5mm socket + 2.5mm clearance)
- M2.5 screw / standoff holes matching motherboard PCB MH_PCA1..4
"""

import numpy as np
import struct
import os

OUTPUT_DIR = "/home/chakradhar/Nurobots_kerela/robotic_arm_motherboard"
SCAD_PATH = os.path.join(OUTPUT_DIR, "pca9685_mount_bracket.scad")
STL_PATH = os.path.join(OUTPUT_DIR, "pca9685_mount_bracket.stl")

# 1. Write OpenSCAD file
scad_content = """// ROBOMOTION 5-DOF Robotic Arm - PCA9685 Servo Controller Mounting Bracket
// Designed for mounting PCA9685 16-ch module to motherboard PCB via 4x M2.5 standoffs
// Hole spacing: 55.88mm x 19.05mm

$fn = 40;

// Dimensions (mm)
hole_dx = 55.88; // Hole spacing long axis
hole_dy = 19.05; // Hole spacing short axis
bracket_length = 66.0;
bracket_width  = 29.0;
base_thick     = 2.5;
standoff_h     = 11.0; // Height matching female header socket
pillar_d       = 6.0;
screw_hole_d   = 2.7;  // Clearance for M2.5 screws

module pca9685_mount() {
    difference() {
        union() {
            // Main base plate with rounded corners
            hull() {
                translate([-(bracket_length/2 - 3), -(bracket_width/2 - 3), 0])
                    cylinder(r=3, h=base_thick);
                translate([(bracket_length/2 - 3), -(bracket_width/2 - 3), 0])
                    cylinder(r=3, h=base_thick);
                translate([(bracket_length/2 - 3), (bracket_width/2 - 3), 0])
                    cylinder(r=3, h=base_thick);
                translate([-(bracket_length/2 - 3), (bracket_width/2 - 3), 0])
                    cylinder(r=3, h=base_thick);
            }
            
            // 4 Standoff pillars
            for (sx = [-1, 1]) {
                for (sy = [-1, 1]) {
                    translate([sx * hole_dx/2, sy * hole_dy/2, 0])
                        cylinder(d=pillar_d, h=standoff_h);
                }
            }
            
            // Side alignment ribs (prevent module sliding)
            translate([-bracket_length/2, -(bracket_width/2), base_thick])
                cube([bracket_length, 1.5, standoff_h - base_thick + 1.6]);
            translate([-bracket_length/2, (bracket_width/2 - 1.5), base_thick])
                cube([bracket_length, 1.5, standoff_h - base_thick + 1.6]);
        }
        
        // Center cutout for header pins and airflow
        translate([-(hole_dx/2 - 5), -(hole_dy/2 - 3), -1])
            cube([hole_dx - 10, hole_dy - 6, base_thick + 2]);
            
        // 4 screw holes (M2.5 clearance throughout pillars & base)
        for (sx = [-1, 1]) {
            for (sy = [-1, 1]) {
                translate([sx * hole_dx/2, sy * hole_dy/2, -1])
                    cylinder(d=screw_hole_d, h=standoff_h + 4);
            }
        }
    }
}

pca9685_mount();
"""

with open(SCAD_PATH, "w") as f:
    f.write(scad_content)
print(f"Written OpenSCAD: {SCAD_PATH}")

# 2. Generate Binary STL Mesh using Python
def write_box_triangles(triangles, x0, y0, z0, dx, dy, dz):
    # 8 vertices
    p = [
        [x0, y0, z0], [x0+dx, y0, z0], [x0+dx, y0+dy, z0], [x0, y0+dy, z0],
        [x0, y0, z0+dz], [x0+dx, y0, z0+dz], [x0+dx, y0+dy, z0+dz], [x0, y0+dy, z0+dz]
    ]
    # 12 triangles (2 per face)
    faces = [
        (0, 2, 1), (0, 3, 2), # bottom
        (4, 5, 6), (4, 6, 7), # top
        (0, 1, 5), (0, 5, 4), # front
        (2, 3, 7), (2, 7, 6), # back
        (0, 4, 7), (0, 7, 3), # left
        (1, 2, 6), (1, 6, 5)  # right
    ]
    for i1, i2, i3 in faces:
        v1 = np.array(p[i1], dtype=np.float32)
        v2 = np.array(p[i2], dtype=np.float32)
        v3 = np.array(p[i3], dtype=np.float32)
        # Normal
        norm = np.cross(v2 - v1, v3 - v1)
        norm_len = np.linalg.norm(norm)
        if norm_len > 0: norm = norm / norm_len
        triangles.append((norm, v1, v2, v3))

def write_tube_triangles(triangles, cx, cy, z0, h, r_out, r_in, n_sides=24):
    angles = np.linspace(0, 2*np.pi, n_sides, endpoint=False)
    for i in range(n_sides):
        a1 = angles[i]
        a2 = angles[(i + 1) % n_sides]
        
        # Outer ring
        x_o1, y_o1 = cx + r_out * np.cos(a1), cy + r_out * np.sin(a1)
        x_o2, y_o2 = cx + r_out * np.cos(a2), cy + r_out * np.sin(a2)
        # Inner ring
        x_i1, y_i1 = cx + r_in * np.cos(a1), cy + r_in * np.sin(a1)
        x_i2, y_i2 = cx + r_in * np.cos(a2), cy + r_in * np.sin(a2)
        
        # Outer wall
        v_ob1 = np.array([x_o1, y_o1, z0], dtype=np.float32)
        v_ob2 = np.array([x_o2, y_o2, z0], dtype=np.float32)
        v_ot1 = np.array([x_o1, y_o1, z0 + h], dtype=np.float32)
        v_ot2 = np.array([x_o2, y_o2, z0 + h], dtype=np.float32)
        
        norm_o = np.array([np.cos((a1+a2)/2), np.sin((a1+a2)/2), 0], dtype=np.float32)
        triangles.append((norm_o, v_ob1, v_ob2, v_ot2))
        triangles.append((norm_o, v_ob1, v_ot2, v_ot1))
        
        # Inner wall (facing inward)
        v_ib1 = np.array([x_i1, y_i1, z0], dtype=np.float32)
        v_ib2 = np.array([x_i2, y_i2, z0], dtype=np.float32)
        v_it1 = np.array([x_i1, y_i1, z0 + h], dtype=np.float32)
        v_it2 = np.array([x_i2, y_i2, z0 + h], dtype=np.float32)
        
        norm_i = -norm_o
        triangles.append((norm_i, v_ib1, v_it2, v_ib2))
        triangles.append((norm_i, v_ib1, v_it1, v_it2))
        
        # Top annular ring
        norm_t = np.array([0, 0, 1], dtype=np.float32)
        triangles.append((norm_t, v_ot1, v_ot2, v_it2))
        triangles.append((norm_t, v_ot1, v_it2, v_it1))
        
        # Bottom annular ring
        norm_b = np.array([0, 0, -1], dtype=np.float32)
        triangles.append((norm_b, v_ob1, v_ib2, v_ob2))
        triangles.append((norm_b, v_ob1, v_ib1, v_ib2))

triangles = []

# Base perimeter frame
# Left, right, top, bottom bars
bl, bw, bt = 66.0, 29.0, 2.5
bar_w = 6.0
write_box_triangles(triangles, -bl/2, -bw/2, 0, bar_w, bw, bt)
write_box_triangles(triangles, bl/2 - bar_w, -bw/2, 0, bar_w, bw, bt)
write_box_triangles(triangles, -bl/2 + bar_w, -bw/2, 0, bl - 2*bar_w, bar_w, bt)
write_box_triangles(triangles, -bl/2 + bar_w, bw/2 - bar_w, 0, bl - 2*bar_w, bar_w, bt)

# Side retention walls
rw, rh = 1.5, 12.6
write_box_triangles(triangles, -bl/2, -bw/2, bt, bl, rw, rh - bt)
write_box_triangles(triangles, -bl/2, bw/2 - rw, bt, bl, rw, rh - bt)

# 4 Standoff pillars
h_dx = 55.88
h_dy = 19.05
sh = 11.0
r_out = 3.2
r_in = 1.35

for sx in [-1, 1]:
    for sy in [-1, 1]:
        cx = sx * h_dx / 2.0
        cy = sy * h_dy / 2.0
        write_tube_triangles(triangles, cx, cy, 0, sh, r_out, r_in, n_sides=32)

# Write binary STL
with open(STL_PATH, "wb") as f:
    header = b"PCA9685 Servo Controller Mounting Bracket (Robomotion 5-DOF Arm)"
    f.write(header.ljust(80, b" "))
    f.write(struct.pack("<I", len(triangles)))
    for norm, v1, v2, v3 in triangles:
        f.write(struct.pack("<3f", *norm))
        f.write(struct.pack("<3f", *v1))
        f.write(struct.pack("<3f", *v2))
        f.write(struct.pack("<3f", *v3))
        f.write(struct.pack("<H", 0))

print(f"Written Binary STL: {STL_PATH} ({len(triangles)} triangles, {os.path.getsize(STL_PATH)} bytes)")
