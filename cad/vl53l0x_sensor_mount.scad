// RoverSAR 5-Channel VL53L0X Time-of-Flight Sensor Arc Mount
// Angles: -60 deg, -30 deg, 0 deg, +30 deg, +60 deg | Mounting: M3 base bolts + M2.5 PCB screws

$fn = 40;

arc_radius = 68.0;
mount_height = 24.0;
wall_thick = 4.0;
angles = [-60, -30, 0, 30, 60];

module sensor_pocket() {
    // Breakout board cavity (25mm W x 12mm H) + optical emitter/SPAD window
    translate([0, 0, mount_height / 2 + 2.0]) {
        cube([4.5, 25.5, 13.0], center = true);
        translate([2.5, 0, 0])
            cube([6.0, 14.0, 8.0], center = true);
    }
}

difference() {
    union() {
        // Curved mounting flange base
        difference() {
            cylinder(h = 4.0, r = arc_radius + 6.0);
            translate([0, 0, -0.5]) cylinder(h = 5.0, r = arc_radius - 12.0);
            translate([-80, -80, -0.5]) cube([80, 160, 5.0]);
        }

        // Sensor bay bosses
        for (a = angles) {
            rotate([0, 0, a])
                translate([arc_radius, 0, 0])
                    translate([0, -14.0, 0])
                        cube([wall_thick + 4.0, 28.0, mount_height]);
        }
    }

    // Carve optical windows & breakout slots
    for (a = angles) {
        rotate([0, 0, a])
            translate([arc_radius + 1.5, 0, 0])
                sensor_pocket();
    }

    // M3 base-plate mounting holes
    for (a = [-45, 0, 45]) {
        rotate([0, 0, a])
            translate([arc_radius - 5.0, 0, -0.5])
                cylinder(h = 5.0, d = 3.2);
    }
}
