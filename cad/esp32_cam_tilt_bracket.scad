// RoverSAR ESP32-CAM AI-Thinker Adjustable Tilt Bracket (0 to 45 deg pitch)
// Target nominal pitch: 15 deg downward for ground SAR victim & obstacle scanning

$fn = 36;

base_w = 48.0;
base_d = 38.0;
base_t = 4.0;
tower_h = 40.0;
cam_tilt_deg = -15.0;

module base_foot() {
    difference() {
        cube([base_d, base_w, base_t], center = true);
        for (sx = [-12, 12], sy = [-18, 18]) {
            translate([sx, sy, 0])
                cylinder(h = base_t + 1.0, d = 3.2, center = true);
        }
    }
}

module side_upright(sy) {
    translate([0, sy * 21.0, tower_h / 2 + base_t / 2]) {
        difference() {
            cube([24.0, 5.0, tower_h], center = true);
            translate([0, 0, tower_h / 2 - 8.0])
                rotate([90, 0, 0])
                    cylinder(h = 7.0, d = 3.2, center = true);
        }
    }
}

module cam_cradle() {
    // Holds 40.5mm x 27.0mm AI-Thinker ESP32-CAM PCB with OV2640 lens hole
    translate([0, 0, tower_h - 6.0])
        rotate([0, cam_tilt_deg, 0])
            difference() {
                cube([8.0, 34.0, 44.0], center = true);
                // OV2640 lens cutout
                translate([0, 0, 5.0])
                    rotate([0, 90, 0])
                        cylinder(h = 10.0, d = 10.0, center = true);
                // Rear PCB recess
                translate([-2.5, 0, 0])
                    cube([4.0, 28.0, 41.0], center = true);
            }
}

union() {
    translate([0, 0, base_t / 2]) base_foot();
    side_upright(-1);
    side_upright(1);
    cam_cradle();
}
