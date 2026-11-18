"""Tests for VFH+ obstacle avoidance around walls and safety fail-safes."""

from __future__ import annotations

from simulator.engine import RoverSimulationEngine
from simulator.safety import DriveMode, SafetyFault
from simulator.world import BoxObstacle, RescueWorld, WallSegment


def test_autonomous_obstacle_avoidance_navigates_around_wall() -> None:
    obstacle = BoxObstacle(x=0.85, y=-0.45, width=0.35, height=0.90, label="WallBarrier")
    world = RescueWorld(
        bounds_min=-4.0,
        bounds_max=4.0,
        obstacles=[obstacle],
        walls=[
            WallSegment(-4.0, -4.0, 4.0, -4.0),
            WallSegment(4.0, -4.0, 4.0, 4.0),
            WallSegment(4.0, 4.0, -4.0, 4.0),
            WallSegment(-4.0, 4.0, -4.0, -4.0),
            *obstacle.to_segments(),
        ],
    )
    engine = RoverSimulationEngine(world=world, sensor_noise=False)
    engine.reset(x_m=0.0, y_m=0.0, theta_rad=0.0)
    engine.set_waypoint(2.0, 0.0)

    avoidance_triggered = False
    for step in range(900):  # up to 18s simulated time
        if step % 10 == 0:
            engine.feed_heartbeat()
        snap = engine.step(0.02)
        if snap["avoidance"]:
            avoidance_triggered = True
        assert not world.collides(engine.true_x, engine.true_y, engine.cfg.collision_radius_m * 0.85)
        if engine.nav.waypoint is None and engine.nav.reached_count >= 1:
            break

    assert avoidance_triggered is True
    assert engine.nav.reached_count >= 1


def test_comm_loss_500ms_watchdog_halts_robot(sim_engine: RoverSimulationEngine) -> None:
    sim_engine.reset(x_m=-2.2, y_m=0.0, theta_rad=0.0)
    sim_engine.set_manual_velocity(0.45, 0.0)

    # Drive normally with regular heartbeats for 0.6s
    for step in range(30):
        if step % 5 == 0:
            sim_engine.feed_heartbeat()
        sim_engine.step(0.02)

    assert sim_engine.safety.active_fault == SafetyFault.NONE
    assert sim_engine.safety.motors_enabled is True
    assert sim_engine.odom.pose.linear_mps > 0.25

    # Stop heartbeats (comm loss) and verify halt triggers within 500ms
    tripped_ms = None
    start_ms = sim_engine.sim_time_ms
    for _ in range(30):  # 600ms window
        sim_engine.step(0.02)
        if sim_engine.safety.active_fault == SafetyFault.COMM_LOSS_500MS:
            tripped_ms = sim_engine.sim_time_ms - start_ms
            break

    assert tripped_ms is not None
    assert tripped_ms <= 520  # 500ms watchdog rounded to 20ms step
    assert sim_engine.safety.motors_enabled is False

    # Coast down under electromagnetic brake to full zero velocity
    for _ in range(15):
        sim_engine.step(0.02)
    assert abs(sim_engine.plant.true_left_rad_s) < 0.1
    assert abs(sim_engine.plant.true_right_rad_s) < 0.1
    assert abs(sim_engine.odom.pose.linear_mps) < 0.01


def test_software_estop_latch_and_clear(sim_engine: RoverSimulationEngine) -> None:
    sim_engine.set_manual_velocity(0.35, 0.5)
    for _ in range(10):
        sim_engine.feed_heartbeat()
        sim_engine.step(0.02)

    sim_engine.trigger_estop()
    sim_engine.step(0.02)
    assert sim_engine.mode == DriveMode.ESTOP
    assert sim_engine.safety.active_fault == SafetyFault.ESTOP_SOFTWARE
    assert sim_engine.safety.motors_enabled is False

    # Attempting manual velocity while E-STOP latched must remain blocked
    sim_engine.set_manual_velocity(0.5, 0.0)
    sim_engine.step(0.02)
    assert sim_engine.mode == DriveMode.ESTOP
    assert sim_engine.safety.motors_enabled is False

    assert sim_engine.clear_estop() is True
    sim_engine.step(0.02)
    assert sim_engine.mode == DriveMode.IDLE
    assert sim_engine.safety.active_fault == SafetyFault.NONE


def test_tilt_rollover_and_low_battery_interlocks(sim_engine: RoverSimulationEngine) -> None:
    sim_engine.set_manual_velocity(0.30, 0.0)
    sim_engine.feed_heartbeat()
    sim_engine.imu.roll_deg = 42.0  # > 35 deg rollover threshold
    sim_engine.step(0.02)
    assert sim_engine.safety.active_fault == SafetyFault.TILT_ROLLOVER
    assert sim_engine.safety.motors_enabled is False

    # Reset and test critical low battery cutoff (< 6.0V sustained >= 1s)
    sim_engine.reset()
    sim_engine.set_manual_velocity(0.25, 0.0)
    sim_engine.battery.resting_v = 5.80
    for _ in range(60):  # 1.2s
        sim_engine.feed_heartbeat()
        sim_engine.step(0.02)

    assert sim_engine.safety.active_fault == SafetyFault.LOW_BATTERY
    assert sim_engine.safety.motors_enabled is False
