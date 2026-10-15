// ROBOMOTION 5-DOF Robotic Arm - PCA9685 Servo Controller Mounting Bracket
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
