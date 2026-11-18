// RoverSAR Main Chassis Base Plate (240mm x 180mm x 4mm deck + 14mm motor mounts)
// Material: PETG / PLA-CF | Infill: 35% Gyroid | Hardware: M3x8mm & M3x12mm SHCS

$fn = 48;

plate_l = 240.0;
plate_w = 180.0;
plate_t = 4.0;
corner_r = 12.0;

module rounded_plate(l, w, t, r) {
    hull() {
        for (sx = [-1, 1], sy = [-1, 1]) {
            translate([sx * (l/2 - r), sy * (w/2 - r), 0])
                cylinder(h = t, r = r);
        }
    }
}

module motor_cradle(side_sign) {
    // JGA25-370 / N20 dual-rail cradle positioned at X = +18mm, Y = +/- 66mm
    translate([18.0, side_sign * 66.0, plate_t]) {
        difference() {
            cube([44.0, 26.0, 14.0], center = true);
            translate([0, 0, 2.0])
                rotate([0, 90, 0])
                    cylinder(h = 46.0, d = 24.6, center = true);
        }
    }
}

difference() {
    union() {
        rounded_plate(plate_l, plate_w, plate_t, corner_r);
        motor_cradle(1);
        motor_cradle(-1);
        // Battery retention ribs centered slightly aft (X = -22mm) for optimal CG
        for (sy = [-1, 1]) {
            translate([-22.0, sy * 28.0, plate_t + 3.0])
                cube([76.0, 4.0, 6.0], center = true);
        }
    }

    // Left & Right Wheel Arch Cutouts (for 65mm x 26mm wheels at X = +18mm)
    for (sy = [-1, 1]) {
        translate([18.0, sy * 82.0, -0.5])
            cube([74.0, 24.0, plate_t + 1.0], center = true);
    }

    // Cable routing oval slots
    for (sx = [-55.0, 58.0]) {
        translate([sx, 0, -0.5])
            hull() {
                translate([0, -14, 0]) cylinder(h = plate_t + 1.0, d = 12.0);
                translate([0,  14, 0]) cylinder(h = plate_t + 1.0, d = 12.0);
            }
    }

    // M3 20mm Electronics Standoff Grid + Sensor Bracket Mounting Holes (3.2mm dia)
    for (gx = [-90, -60, -30, 0, 30, 60, 90]) {
        for (gy = [-45, 0, 45]) {
            translate([gx, gy, -0.5])
                cylinder(h = plate_t + 1.0, d = 3.2);
        }
    }

    // Rear swivel caster 4-bolt M3 pattern (X = -92mm)
    for (cx = [-102, -82], cy = [-14, 14]) {
        translate([cx, cy, -0.5])
            cylinder(h = plate_t + 1.0, d = 3.2);
    }
}
