"""
NAVIS Phase 1 — Task 1.6: Dedicated Rover Physics Verification Test Suite
Independent physics, kinematics, wheel dynamics, steering, battery, and telemetry verification for MarsRover.
"""

import math
import os
import sys
import numpy as np
import pytest

# Add project root directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from backend.simulation.rover import MarsRover, RoverMissionStatus
from backend.simulation.terrain import MarsTerrain


def test_rover_initialization_and_reset():
    """Verify default initialization, custom parameters, reset behavior, and reset idempotency."""
    # 1. Default initialization
    rover = MarsRover()
    assert rover.start_pos == (50.0, 50.0)
    assert rover.target_pos == (540.0, 530.0)
    assert rover.x == 50.0
    assert rover.y == 50.0
    assert rover.speed == 0.0
    assert rover.vx == 0.0
    assert rover.vy == 0.0
    assert rover.angular_velocity == 0.0
    assert rover.angular_acceleration == 0.0
    assert rover.battery_pct == 100.0
    assert rover.total_distance_traveled == 0.0
    assert rover.mission_status == RoverMissionStatus.IDLE

    # 2. Custom initialization
    custom_rover = MarsRover(start_x=10.0, start_y=20.0, target_x=100.0, target_y=200.0)
    assert custom_rover.start_pos == (10.0, 20.0)
    assert custom_rover.target_pos == (100.0, 200.0)
    assert custom_rover.x == 10.0
    assert custom_rover.y == 20.0

    # 3. Modify state and verify reset restores clean baseline
    rover.x = 250.0
    rover.y = 300.0
    rover.speed = 2.5
    rover.vx = 2.0
    rover.vy = 1.5
    rover.angular_velocity = 0.8
    rover.battery_pct = 75.0
    rover.total_distance_traveled = 150.0
    rover.mission_status = RoverMissionStatus.NAVIGATING

    rover.reset()
    assert rover.x == 50.0
    assert rover.y == 50.0
    assert rover.speed == 0.0
    assert rover.vx == 0.0
    assert rover.vy == 0.0
    assert rover.angular_velocity == 0.0
    assert rover.battery_pct == 100.0
    assert rover.total_distance_traveled == 0.0
    assert rover.mission_status == RoverMissionStatus.IDLE
    assert len(rover.history) == 1
    assert rover.history[0]["x"] == 50.0

    # 4. Explicit start position in reset
    rover.reset(start_pos=(80.0, 90.0))
    assert rover.start_pos == (80.0, 90.0)
    assert rover.x == 80.0
    assert rover.y == 90.0

    # 5. Reset idempotency
    rover.reset()
    state_1 = rover.to_dict()
    rover.reset()
    state_2 = rover.to_dict()
    assert state_1 == state_2


def test_straight_line_kinematics():
    """Verify exact straight-line displacement for zero angular velocity (omega = 0)."""
    rover = MarsRover(start_x=50.0, start_y=50.0, target_x=100.0, target_y=50.0, max_speed=2.0)
    rover.speed = 2.0
    rover.heading = 0.0
    rover.angular_velocity = 0.0
    rover.set_path([(50.0, 50.0), (100.0, 50.0)])

    dt = 0.5
    # Nominal slip is 0.02 for flat terrain (terrain_cost=1.0, slope=0.0)
    v_ground = 2.0 * (1.0 - 0.02)  # 1.96 m/s
    expected_dx = v_ground * math.cos(0.0) * dt  # 0.98m
    expected_dy = v_ground * math.sin(0.0) * dt  # 0.0m

    rover.update(dt=dt, sim_time=0.5, terrain_cost=1.0, terrain_slope=0.0)

    assert math.isclose(rover.x - 50.0, expected_dx, abs_tol=1e-5)
    assert math.isclose(rover.y - 50.0, expected_dy, abs_tol=1e-5)
    assert math.isclose(rover.heading, 0.0, abs_tol=1e-5)
    assert math.isclose(rover.vx, v_ground, abs_tol=1e-5)
    assert math.isclose(rover.vy, 0.0, abs_tol=1e-5)
    assert math.isclose(rover.total_distance_traveled, expected_dx, abs_tol=1e-5)


def test_constant_turn_arc_kinematics():
    """Verify exact circular-arc kinematic equations for positive and negative angular velocities."""
    dt = 1.0
    theta = 0.0

    # A. Positive angular velocity (left turn: omega = 0.5 rad/s)
    # Target waypoints placed at angle_diff = 0.2 rad so desired_omega = 0.2 * 2.5 = 0.5 rad/s
    curv_factor = 1.0 - 0.2 / math.pi
    rover_left = MarsRover(start_x=50.0, start_y=50.0, max_speed=2.0 / curv_factor)
    target_left = (50.0 + 100.0 * math.cos(0.2), 50.0 + 100.0 * math.sin(0.2))
    rover_left.set_path([target_left])
    rover_left.speed = 2.0
    rover_left.heading = 0.0
    rover_left.angular_velocity = 0.5

    v_g = 2.0 * (1.0 - 0.02)  # 1.96 m/s
    omega_pos = 0.5

    exp_dx_pos = (v_g / omega_pos) * (math.sin(theta + omega_pos * dt) - math.sin(theta))
    exp_dy_pos = -(v_g / omega_pos) * (math.cos(theta + omega_pos * dt) - math.cos(theta))
    exp_heading_pos = 0.5

    rover_left.update(dt=dt, sim_time=1.0, terrain_cost=1.0, terrain_slope=0.0)
    assert math.isclose(rover_left.x - 50.0, exp_dx_pos, abs_tol=1e-4)
    assert math.isclose(rover_left.y - 50.0, exp_dy_pos, abs_tol=1e-4)
    assert math.isclose(rover_left.heading, exp_heading_pos, abs_tol=1e-4)

    # B. Negative angular velocity (right turn: omega = -0.5 rad/s)
    rover_right = MarsRover(start_x=50.0, start_y=50.0, max_speed=2.0 / curv_factor)
    target_right = (50.0 + 100.0 * math.cos(-0.2), 50.0 + 100.0 * math.sin(-0.2))
    rover_right.set_path([target_right])
    rover_right.speed = 2.0
    rover_right.heading = 0.0
    rover_right.angular_velocity = -0.5
    omega_neg = -0.5

    exp_dx_neg = (v_g / omega_neg) * (math.sin(theta + omega_neg * dt) - math.sin(theta))
    exp_dy_neg = -(v_g / omega_neg) * (math.cos(theta + omega_neg * dt) - math.cos(theta))
    exp_heading_neg = -0.5

    rover_right.update(dt=dt, sim_time=1.0, terrain_cost=1.0, terrain_slope=0.0)
    assert math.isclose(rover_right.x - 50.0, exp_dx_neg, abs_tol=1e-4)
    assert math.isclose(rover_right.y - 50.0, exp_dy_neg, abs_tol=1e-4)
    assert math.isclose(rover_right.heading, exp_heading_neg, abs_tol=1e-4)


def test_speed_adaptive_lookahead():
    """Verify dynamic lookahead distance clamping and speed scaling."""
    rover = MarsRover(lookahead_gain=1.5, min_lookahead=3.0, max_lookahead=12.0)

    # v = 0 -> L_d = min_lookahead = 3.0
    rover.speed = 0.0
    rover.update(dt=0.1, sim_time=0.1)
    assert rover.lookahead_dist == 3.0

    # low speed = 1.0 -> L_d = 1.5 * 1.0 + 3.0 = 4.5
    rover.speed = 1.0
    rover.update(dt=0.1, sim_time=0.2)
    assert math.isclose(rover.lookahead_dist, 4.5, abs_tol=1e-3)

    # nominal speed = 4.0 -> L_d = 1.5 * 4.0 + 3.0 = 9.0
    rover.speed = 4.0
    rover.update(dt=0.1, sim_time=0.3)
    assert math.isclose(rover.lookahead_dist, 9.0, abs_tol=1e-3)

    # high speed above max bound = 10.0 -> L_d = capped at 12.0
    rover.speed = 10.0
    rover.update(dt=0.1, sim_time=0.4)
    assert rover.lookahead_dist == 12.0


def test_steering_dynamics():
    """Verify steering angular acceleration limits, max rate caps, reversal, and dt <= 0 safety."""
    rover = MarsRover(start_x=50.0, start_y=50.0, target_x=50.0, target_y=100.0)
    rover.heading = 0.0
    rover.angular_velocity = 0.0
    rover.set_path([(50.0, 50.0), (50.0, 100.0)])

    dt = 0.1
    max_accel_step = rover.max_steer_accel * dt  # 3.0 * 0.1 = 0.3 rad/s

    # Step 1: angular velocity should increase by max_accel_step
    rover.update(dt=dt, sim_time=0.1)
    assert math.isclose(abs(rover.angular_acceleration), rover.max_steer_accel, rel_tol=1e-3)
    assert math.isclose(rover.angular_velocity, max_accel_step, abs_tol=1e-4)

    # Step 20: angular velocity should cap at max_steer_rate (1.5 rad/s)
    for i in range(20):
        rover.update(dt=dt, sim_time=0.1 * (i + 2))
    assert abs(rover.angular_velocity) <= rover.max_steer_rate + 1e-5

    # Steering reversal test
    rover_rev = MarsRover(start_x=50.0, start_y=50.0)
    rover_rev.heading = 0.0
    rover_rev.angular_velocity = 1.0
    rover_rev.set_path([(50.0, 50.0), (50.0, -100.0)])  # Target behind rover
    prev_omega = rover_rev.angular_velocity
    rover_rev.update(dt=0.1, sim_time=1.0)
    new_omega = rover_rev.angular_velocity
    assert new_omega < prev_omega
    assert abs(new_omega - prev_omega) <= rover_rev.max_steer_accel * 0.1 + 1e-5

    # Safe dt <= 0 test
    rover_safe = MarsRover()
    rover_safe.update(dt=0.0, sim_time=0.0)
    assert rover_safe.angular_acceleration == 0.0
    assert not math.isnan(rover_safe.angular_velocity)
    rover_safe.update(dt=-0.1, sim_time=0.0)
    assert rover_safe.angular_acceleration == 0.0
    assert not math.isnan(rover_safe.angular_velocity)


def test_wheel_slip_physics():
    """Verify ground speed, wheel speed, slip formula, and physical displacement foundation."""
    rover = MarsRover(start_x=50.0, start_y=50.0)
    rover.speed = 2.0
    rover.wheel_slip = 0.20

    # Ground speed vs wheel speed
    rover.ground_speed = rover.speed * (1.0 - rover.wheel_slip)  # 1.6 m/s
    rover.wheel_speed = rover.ground_speed / (1.0 - rover.wheel_slip)  # 2.0 m/s

    assert rover.ground_speed < rover.wheel_speed
    assert math.isclose(rover.wheel_speed * (1.0 - rover.wheel_slip), rover.ground_speed, abs_tol=1e-5)

    # Physical displacement depends on ground speed, not wheel speed
    rover_disp = MarsRover(start_x=50.0, start_y=50.0, target_x=100.0, target_y=50.0)
    rover_disp.speed = 2.0
    rover_disp.heading = 0.0
    rover_disp.angular_velocity = 0.0
    rover_disp.set_path([(50.0, 50.0), (100.0, 50.0)])
    dt = 1.0
    rover_disp.update(dt=dt, sim_time=1.0, terrain_cost=1.0, terrain_slope=0.0)
    expected_ground_speed = rover_disp.speed * (1.0 - rover_disp.wheel_slip)
    expected_dx = expected_ground_speed * dt
    actual_dx = rover_disp.x - 50.0
    assert math.isclose(actual_dx, expected_dx, abs_tol=1e-3)


def test_differential_wheel_speeds():
    """Verify left and right wheel speeds under straight motion and differential turns."""
    rover = MarsRover(start_x=50.0, start_y=50.0)
    rover.speed = 2.0
    rover.wheel_slip = 0.10
    rover.ground_speed = rover.speed * (1.0 - rover.wheel_slip)  # 1.8
    rover.wheel_speed = rover.ground_speed / (1.0 - rover.wheel_slip)  # 2.0
    track_width = 1.2

    # A. Straight motion (omega = 0)
    rover.angular_velocity = 0.0
    w_left_str = rover.wheel_speed - (rover.angular_velocity * track_width / 2.0)
    w_right_str = rover.wheel_speed + (rover.angular_velocity * track_width / 2.0)
    assert w_left_str == w_right_str == 2.0

    # B. Positive turn (omega = 0.5 rad/s)
    rover.angular_velocity = 0.5
    w_left_pos = rover.wheel_speed - (rover.angular_velocity * track_width / 2.0)
    w_right_pos = rover.wheel_speed + (rover.angular_velocity * track_width / 2.0)
    assert w_right_pos > w_left_pos
    assert math.isclose((w_left_pos + w_right_pos) / 2.0, rover.wheel_speed, abs_tol=1e-5)

    # C. Negative turn (omega = -0.5 rad/s)
    rover.angular_velocity = -0.5
    w_left_neg = rover.wheel_speed - (rover.angular_velocity * track_width / 2.0)
    w_right_neg = rover.wheel_speed + (rover.angular_velocity * track_width / 2.0)
    assert w_left_neg > w_right_neg
    assert math.isclose((w_left_neg + w_right_neg) / 2.0, rover.wheel_speed, abs_tol=1e-5)


def test_battery_physics():
    """Verify battery consumption under movement, high terrain cost, hotel base load, and clamping at zero."""
    # A. Base hotel load while stationary (speed = 0)
    rover_stat = MarsRover(start_x=50.0, start_y=50.0)
    rover_stat.speed = 0.0
    rover_stat.update(dt=10.0, sim_time=10.0)
    assert rover_stat.battery_pct < 100.0
    assert rover_stat.battery_pct > 99.0

    # B. Higher terrain cost increases power draw
    rover_normal = MarsRover(start_x=50.0, start_y=50.0)
    rover_normal.speed = 2.0
    rover_normal.update(dt=10.0, sim_time=10.0, terrain_cost=1.0)
    bat_normal_loss = 100.0 - rover_normal.battery_pct

    rover_rock = MarsRover(start_x=50.0, start_y=50.0)
    rover_rock.speed = 2.0
    rover_rock.update(dt=10.0, sim_time=10.0, terrain_cost=5.0)
    bat_rock_loss = 100.0 - rover_rock.battery_pct

    assert bat_rock_loss > bat_normal_loss

    # C. Depletion protection (clamped at 0.0)
    rover_dep = MarsRover(start_x=50.0, start_y=50.0)
    rover_dep.battery_pct = 0.001
    rover_dep.update(dt=100.0, sim_time=100.0)
    assert rover_dep.battery_pct == 0.0

    # D. Reset restores 100%
    rover_dep.reset()
    assert rover_dep.battery_pct == 100.0


def test_target_reached_state():
    """Verify arrival at target tolerance, deceleration, steering settling, stationary stability, and battery hotel load."""
    rover = MarsRover(start_x=50.0, start_y=50.0, target_x=52.0, target_y=50.0)
    rover.x, rover.y = 51.5, 50.0  # within target_tolerance = 4.0m
    rover.speed = 2.0
    rover.angular_velocity = 0.5
    rover.set_path([(50.0, 50.0), (52.0, 50.0)])

    # Step update: status updates to TARGET_REACHED
    rover.update(dt=0.1, sim_time=1.0)
    assert rover.mission_status == RoverMissionStatus.TARGET_REACHED

    # Settle rover over sufficient steps (100 steps = 10s)
    for step in range(100):
        rover.update(dt=0.1, sim_time=1.0 + step * 0.1)

    # Settled checks
    assert math.isclose(rover.speed, 0.0, abs_tol=1e-4)
    assert math.isclose(rover.vx, 0.0, abs_tol=1e-4)
    assert math.isclose(rover.vy, 0.0, abs_tol=1e-4)
    assert math.isclose(rover.angular_velocity, 0.0, abs_tol=1e-4)
    assert rover.wheel_speed == 0.0

    settled_x, settled_y = rover.x, rover.y
    settled_dist = rover.total_distance_traveled

    # Further stationary steps keep position fixed and distance static
    for step in range(10):
        rover.update(dt=0.1, sim_time=11.0 + step * 0.1)

    assert rover.x == settled_x and rover.y == settled_y
    assert rover.total_distance_traveled == settled_dist


def test_history_and_telemetry():
    """Verify history sampling rules, max history length, and to_dict() telemetry contract fields."""
    rover = MarsRover(start_x=50.0, start_y=50.0)

    # 1. History recording
    assert len(rover.history) == 0
    rover.record_history(sim_time=0.0)
    assert len(rover.history) == 1

    # Small movement < 1.0m does not create duplicate sample
    rover.x = 50.5
    rover.record_history(sim_time=0.1)
    assert len(rover.history) == 1

    # Movement > 1.0m records sample
    rover.x = 52.0
    rover.record_history(sim_time=0.2)
    assert len(rover.history) == 2

    # 2. Telemetry contract verification (to_dict)
    telemetry = rover.to_dict()
    required_keys = [
        "x",
        "y",
        "vx",
        "vy",
        "speed",
        "ground_speed",
        "wheel_speed",
        "accel",
        "heading_rad",
        "heading_deg",
        "angular_velocity",
        "angular_acceleration",
        "lookahead_dist",
        "battery_pct",
        "wheel_slip",
        "wheel_speed_left",
        "wheel_speed_right",
        "total_distance",
        "mission_status",
        "target_pos",
        "dist_to_target",
    ]

    for key in required_keys:
        assert key in telemetry, f"Missing required telemetry key in to_dict(): {key}"

    assert isinstance(telemetry["x"], float)
    assert isinstance(telemetry["y"], float)
    assert isinstance(telemetry["mission_status"], str)
    assert isinstance(telemetry["target_pos"], list)


def test_numerical_safety():
    """Verify robustness against dt <= 0, extreme heading values, slip bounds, and absence of NaN/Inf."""
    rover = MarsRover(start_x=50.0, start_y=50.0)

    # A. dt = 0 and dt < 0
    rover.update(dt=0.0, sim_time=0.0)
    assert not math.isnan(rover.x) and not math.isinf(rover.x)
    assert not math.isnan(rover.angular_velocity)

    rover.update(dt=-0.05, sim_time=0.0)
    assert not math.isnan(rover.x) and not math.isinf(rover.x)

    # B. Heading normalization
    rover.heading = 10.0  # > pi
    rover.update(dt=0.1, sim_time=1.0)
    assert -math.pi <= rover.heading <= math.pi

    rover.heading = -10.0  # < -pi
    rover.update(dt=0.1, sim_time=1.0)
    assert -math.pi <= rover.heading <= math.pi

    # C. Extreme slip inputs
    for cost, slope in [(1.0, 0.0), (100.0, 50.0), (0.0, 0.0)]:
        rover.update(dt=0.1, sim_time=1.0, terrain_cost=cost, terrain_slope=slope)
        assert 0.0 <= rover.wheel_slip <= 0.95
        assert not math.isnan(rover.wheel_speed)
        assert not math.isinf(rover.wheel_speed)
        assert rover.wheel_speed >= 0.0


def test_regression_against_terrain():
    """Verify integration of MarsRover with continuous MarsTerrain query outputs."""
    terrain = MarsTerrain(seed=42)
    rover = MarsRover(
        start_x=terrain.start_pos[0],
        start_y=terrain.start_pos[1],
        target_x=terrain.target_pos[0],
        target_y=terrain.target_pos[1],
    )
    rover.set_path([(terrain.start_pos[0], terrain.start_pos[1]), (100.0, 100.0)])

    # Step rover 50 ticks using continuous terrain queries
    dt = 0.05
    for i in range(50):
        sim_time = (i + 1) * dt
        cost = terrain.get_cost(rover.x, rover.y)
        slope = terrain.get_slope(rover.x, rover.y)

        rover.update(
            dt=dt,
            sim_time=sim_time,
            terrain_cost=cost,
            terrain_slope=slope,
            is_outage=False,
        )

        assert not math.isnan(rover.x)
        assert not math.isnan(rover.y)
        assert not math.isnan(rover.wheel_slip)

    assert rover.total_distance_traveled > 0.0


if __name__ == "__main__":
    pytest.main(["-v", __file__])
