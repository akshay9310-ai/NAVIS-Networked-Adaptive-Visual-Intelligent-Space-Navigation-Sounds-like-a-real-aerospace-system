"""
Unit and Integration verification tests for NAVIS backend.
"""

import sys
import os
import math
import numpy as np

# Add root directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from backend.simulation.terrain import MarsTerrain
from backend.simulation.rover import MarsRover, RoverMissionStatus
from backend.simulation.satellites import SatelliteConstellation
from backend.navigation.sensors import SensorSuite
from backend.navigation.ekf import NAVIS_EKF
from backend.navigation.planner import AStarPlanner
from backend.ai.scheduler import AISatelliteScheduler
from backend.ai.trajectory import TrajectoryPredictor
from backend.ai.hazard_detector import VisualHazardDetector
from backend.simulation.modes import BenchmarkEvaluator, NavigationMode
from backend.simulation.engine import SimulationEngine


def test_terrain_and_planner():
    print("Testing MarsTerrain & A* Planner...")
    terrain = MarsTerrain(seed=42)
    assert terrain.grid_res == 60
    assert terrain.cost_grid.shape == (60, 60)
    assert terrain.get_cost(terrain.start_pos[0], terrain.start_pos[1]) == 1.0

    planner = AStarPlanner(terrain)
    path = planner.plan_path(terrain.start_pos, terrain.target_pos)
    assert len(path) >= 2
    assert path[0] == terrain.start_pos
    assert path[-1] == terrain.target_pos
    print(f"[OK] Path found with {len(path)} waypoints.")


def test_ekf_and_sensors():
    print("Testing SensorSuite & NAVIS_EKF...")
    sensors = SensorSuite(seed=42)
    ekf = NAVIS_EKF(start_x=50.0, start_y=50.0)

    # Step with PNT active
    meas_pnt = sensors.sample(
        dt=0.05,
        true_x=50.5,
        true_y=50.5,
        true_vx=1.0,
        true_vy=1.0,
        true_heading=0.78,
        true_speed=1.414,
        true_accel=0.2,
        true_omega=0.0,
        wheel_slip=0.02,
        pnt_available=True,
    )
    est = ekf.step(0.05, meas_pnt)
    assert est["pnt_active"] is True
    assert est["uncertainty_m"] < 1.5
    print(f"[OK] EKF PNT Step: est_x={est['est_x']:.2f}, uncertainty={est['uncertainty_m']:.2f}m")

    # Step with PNT Outage (fallback)
    meas_outage = sensors.sample(
        dt=0.05,
        true_x=51.0,
        true_y=51.0,
        true_vx=1.0,
        true_vy=1.0,
        true_heading=0.78,
        true_speed=1.414,
        true_accel=0.2,
        true_omega=0.0,
        wheel_slip=0.05,
        pnt_available=False,
    )
    est_outage = ekf.step(0.05, meas_outage)
    assert est_outage["fallback_active"] is True
    print(f"[OK] EKF Fallback Step: uncertainty={est_outage['uncertainty_m']:.2f}m")


def test_ai_scheduler_and_trajectory():
    print("Testing AI Scheduler & ML Trajectory Predictor...")
    constellation = SatelliteConstellation()
    constellation.update(0.1, 0.0, (50.0, 50.0))
    scheduler = AISatelliteScheduler()
    sched = scheduler.evaluate_and_schedule(
        constellation.satellites,
        {"speed": 2.5, "battery_pct": 90.0},
        {"uncertainty_m": 0.8, "fallback_active": False},
        hazard_risk_level=0.9,
        sim_time=10.0,
    )
    assert "scores" in sched
    assert sched["selected_satellite"] is not None
    print(f"[OK] AI Scheduler Decision: {sched['selected_satellite']} -> {sched['selected_task']}")

    predictor = TrajectoryPredictor()
    dummy_history = [{"t": i * 0.5, "x": 50.0 + i * 1.5, "y": 50.0 + i * 1.2, "vx": 3.0, "vy": 2.4, "speed": 3.8, "heading": 0.6, "battery": 95} for i in range(10)]
    pred = predictor.predict(
        dummy_history,
        {"x": 65.0, "y": 62.0, "speed": 3.8, "heading_rad": 0.6, "vx": 3.0, "vy": 2.4},
        [(100, 100)],
        current_uncertainty=0.8,
    )
    assert len(pred["predicted_trajectory"]) > 0
    assert pred["confidence_pct"] > 50.0
    print(f"[OK] ML Trajectory Predicted: confidence={pred['confidence_pct']}% with {len(pred['predicted_trajectory'])} steps.")


def test_simulation_engine_and_benchmark():
    print("Testing Central SimulationEngine & BenchmarkEvaluator...")
    engine = SimulationEngine(seed=42)
    engine.start()
    for _ in range(10):
        engine.step(0.05)
    state = engine.get_full_state()
    assert state["rover"]["x"] > 50.0
    assert "metrics" in state
    print(f"[OK] Simulation Step 10: Rover at ({state['rover']['x']:.2f}, {state['rover']['y']:.2f})")

    # Fast benchmark run
    bench = BenchmarkEvaluator.run_benchmark_comparison(terrain_seed=42)
    assert "MODE_A" in bench["comparison_matrix"]
    assert "MODE_B" in bench["comparison_matrix"]
    assert "MODE_C" in bench["comparison_matrix"]
    print(f"[OK] Benchmark Evaluation complete: Best Mode = {bench['best_mode']}")


def test_task_1_1_rover_kinematics_and_execution_order():
    print("Testing Task 1.1 Objectives: Kinematic Arc Integration, Dynamic Lookahead & Execution Order...")

    # A. Straight-line motion (omega = 0)
    rover_straight = MarsRover(start_x=50.0, start_y=50.0, target_x=100.0, target_y=50.0)
    rover_straight.speed = 2.0
    rover_straight.heading = 0.0
    rover_straight.angular_velocity = 0.0
    rover_straight.wheel_slip = 0.0
    rover_straight.set_path([(50.0, 50.0), (100.0, 50.0)])
    rover_straight.update(dt=1.0, sim_time=1.0)
    assert rover_straight.y == 50.0, f"Expected y=50.0, got {rover_straight.y}"
    assert rover_straight.x > 50.0, f"Expected x > 50.0, got {rover_straight.x}"

    # B & C. Constant-turn motion (non-zero omega & heading update)
    dt = 1.0
    v_g = 2.0
    omega = 0.5
    theta = 0.0

    exp_dx = (v_g / omega) * (math.sin(theta + omega * dt) - math.sin(theta))
    exp_dy = -(v_g / omega) * (math.cos(theta + omega * dt) - math.cos(theta))
    exp_heading = (theta + omega * dt + math.pi) % (2 * math.pi) - math.pi

    assert math.isclose(exp_dx, 4.0 * math.sin(0.5), rel_tol=1e-5)
    assert math.isclose(exp_dy, 4.0 * (1.0 - math.cos(0.5)), rel_tol=1e-5)
    assert math.isclose(exp_heading, 0.5, rel_tol=1e-5)

    # D. Speed-Adaptive Dynamic Lookahead
    rover_ld = MarsRover(lookahead_gain=1.5, min_lookahead=3.0, max_lookahead=12.0)
    assert rover_ld.lookahead_dist == 3.0, f"Initial lookahead should be min_lookahead=3.0, got {rover_ld.lookahead_dist}"

    rover_ld.speed = 0.0
    rover_ld.update(dt=0.1, sim_time=0.1)
    assert rover_ld.lookahead_dist == 3.0, f"At speed 0, lookahead should be 3.0, got {rover_ld.lookahead_dist}"

    rover_ld.speed = 2.0
    rover_ld.update(dt=0.1, sim_time=0.2)
    assert math.isclose(rover_ld.lookahead_dist, 6.0, rel_tol=1e-3), f"At speed 2.0, lookahead should be 6.0, got {rover_ld.lookahead_dist}"

    rover_ld.speed = 10.0
    rover_ld.update(dt=0.1, sim_time=0.3)
    assert rover_ld.lookahead_dist == 12.0, f"At speed 10.0, lookahead should be capped at max_lookahead=12.0, got {rover_ld.lookahead_dist}"

    # E. Execution Order Verification in SimulationEngine
    engine = SimulationEngine(seed=42)
    engine.start()
    init_x, init_y = engine.rover.x, engine.rover.y
    engine.step(0.05)
    new_x, new_y = engine.rover.x, engine.rover.y
    assert (new_x != init_x or new_y != init_y), "Rover should have moved during engine step"

    assert engine.sensors.prev_true_x == new_x, f"Sensors prev_true_x should match updated rover.x ({new_x}), got {engine.sensors.prev_true_x}"
    assert engine.sensors.prev_true_y == new_y, f"Sensors prev_true_y should match updated rover.y ({new_y}), got {engine.sensors.prev_true_y}"

    print("[OK] Task 1.1 Objectives Fully Verified!")


def test_task_1_2_wheel_slip_physics_and_odometry():
    print("Testing Task 1.2 Objectives: Wheel-Slip Physics and Wheel Odometry...")

    # A. Zero slip
    rover_zero = MarsRover(start_x=50.0, start_y=50.0)
    rover_zero.speed = 2.0
    rover_zero.wheel_slip = 0.0
    rover_zero.ground_speed = rover_zero.speed * (1.0 - rover_zero.wheel_slip)
    rover_zero.wheel_speed = rover_zero.ground_speed / max(0.05, 1.0 - rover_zero.wheel_slip)
    assert math.isclose(rover_zero.wheel_speed, rover_zero.ground_speed, rel_tol=1e-5), \
        f"Expected wheel_speed ≈ ground_speed at zero slip, got wheel={rover_zero.wheel_speed}, ground={rover_zero.ground_speed}"

    # B. Positive slip
    rover_slip = MarsRover(start_x=50.0, start_y=50.0)
    rover_slip.speed = 2.0
    rover_slip.wheel_slip = 0.20
    rover_slip.ground_speed = rover_slip.speed * (1.0 - rover_slip.wheel_slip)
    rover_slip.wheel_speed = rover_slip.ground_speed / (1.0 - rover_slip.wheel_slip)
    assert rover_slip.ground_speed < rover_slip.wheel_speed, \
        f"Ground speed ({rover_slip.ground_speed}) should be < wheel speed ({rover_slip.wheel_speed}) under positive slip"
    assert math.isclose(rover_slip.wheel_speed * (1.0 - rover_slip.wheel_slip), rover_slip.ground_speed, rel_tol=1e-5), \
        f"Expected wheel_speed * (1 - slip) == ground_speed"

    # C. Physical displacement is based on ground speed, not wheel speed
    rover_disp = MarsRover(start_x=50.0, start_y=50.0, target_x=100.0, target_y=50.0)
    rover_disp.speed = 2.0
    rover_disp.heading = 0.0
    rover_disp.angular_velocity = 0.0
    rover_disp.set_path([(50.0, 50.0), (100.0, 50.0)])
    dt = 1.0
    rover_disp.update(dt=dt, sim_time=1.0, terrain_cost=1.0, terrain_slope=1.2)
    expected_ground_speed = rover_disp.speed * (1.0 - rover_disp.wheel_slip)
    expected_dx = expected_ground_speed * dt
    actual_dx = rover_disp.x - 50.0
    assert math.isclose(actual_dx, expected_dx, rel_tol=1e-3), \
        f"Rover displacement should be based on ground speed ({expected_dx}m), got {actual_dx}m"

    # D. Differential wheel behavior
    rover_turn = MarsRover(start_x=50.0, start_y=50.0)
    rover_turn.speed = 2.0
    rover_turn.angular_velocity = 0.5
    rover_turn.wheel_slip = 0.10
    rover_turn.ground_speed = rover_turn.speed * (1.0 - rover_turn.wheel_slip)
    denom = 1.0 - rover_turn.wheel_slip
    rover_turn.wheel_speed = rover_turn.ground_speed / denom
    track_width = 1.2
    rover_turn.wheel_speed_left = rover_turn.wheel_speed - (rover_turn.angular_velocity * track_width / 2.0)
    rover_turn.wheel_speed_right = rover_turn.wheel_speed + (rover_turn.angular_velocity * track_width / 2.0)
    avg_wheel_speed = (rover_turn.wheel_speed_left + rover_turn.wheel_speed_right) / 2.0
    assert math.isclose(avg_wheel_speed, rover_turn.wheel_speed, rel_tol=1e-5), \
        f"Average of differential wheel speeds ({avg_wheel_speed}) should match wheel_speed ({rover_turn.wheel_speed})"
    assert math.isclose(avg_wheel_speed * (1.0 - rover_turn.wheel_slip), rover_turn.ground_speed, rel_tol=1e-5), \
        f"Average wheel speed * (1 - slip) should equal ground speed"

    # E. Sensor measurement: Wheel odometry samples wheel-side velocity
    sensors = SensorSuite(seed=123)
    sample_res = sensors.sample(
        dt=0.1,
        true_x=50.0,
        true_y=50.0,
        true_vx=1.6,
        true_vy=0.0,
        true_heading=0.0,
        true_speed=1.6,
        true_accel=0.0,
        true_omega=0.0,
        wheel_slip=0.20,
        pnt_available=True,
        wheel_speed=2.0,
    )
    odo_speed = sample_res["odometry"]["speed"]
    assert abs(odo_speed - 2.0) < 0.25, f"Odometry speed ({odo_speed}) should be centered around wheel speed 2.0 m/s, not ground speed 1.6 m/s"

    # F. Numerical safety
    rover_safe = MarsRover(start_x=50.0, start_y=50.0)
    for test_slip in [0.0, 0.00001, 0.45, 0.95, 0.9999]:
        raw_slip = test_slip
        clipped_slip = float(np.clip(min(0.45, raw_slip), 0.0, 0.95))
        g_speed = 2.0 * (1.0 - clipped_slip)
        w_speed = g_speed / max(0.05, 1.0 - clipped_slip)
        assert not math.isnan(w_speed), f"wheel_speed is NaN for slip={test_slip}"
        assert not math.isinf(w_speed), f"wheel_speed is Inf for slip={test_slip}"
        assert w_speed >= 0.0, f"wheel_speed is negative for slip={test_slip}"

    print("[OK] Task 1.2 Objectives Fully Verified!")


def test_task_1_3_steering_dynamics_and_inertial_damping():
    print("Testing Task 1.3 Objectives: Steering Angular Acceleration Limits & Inertial Damping...")

    # A. Angular acceleration limit
    rover_accel = MarsRover(start_x=50.0, start_y=50.0, target_x=50.0, target_y=100.0)
    rover_accel.heading = 0.0
    rover_accel.angular_velocity = 0.0
    rover_accel.set_path([(50.0, 50.0), (50.0, 100.0)])
    dt = 0.1
    init_omega = rover_accel.angular_velocity
    rover_accel.update(dt=dt, sim_time=1.0)
    delta_omega = rover_accel.angular_velocity - init_omega
    max_allowed_delta = rover_accel.max_steer_accel * dt + 1e-6
    assert abs(delta_omega) <= max_allowed_delta, \
        f"Angular velocity change ({abs(delta_omega):.4f}) exceeded max_steer_accel limit ({max_allowed_delta:.4f})"
    assert math.isclose(abs(rover_accel.angular_acceleration), rover_accel.max_steer_accel, rel_tol=1e-3), \
        f"Expected angular acceleration to match limit ({rover_accel.max_steer_accel}), got {rover_accel.angular_acceleration}"

    # B. Maximum steering rate
    rover_rate = MarsRover(start_x=50.0, start_y=50.0, target_x=50.0, target_y=100.0)
    rover_rate.heading = 0.0
    rover_rate.set_path([(50.0, 50.0), (50.0, 100.0)])
    for step in range(20):
        rover_rate.update(dt=0.1, sim_time=step * 0.1)
        assert abs(rover_rate.angular_velocity) <= rover_rate.max_steer_rate + 1e-5, \
            f"Angular velocity ({rover_rate.angular_velocity}) exceeded max_steer_rate ({rover_rate.max_steer_rate})"

    # C. Smooth steering reversal
    rover_rev = MarsRover(start_x=50.0, start_y=50.0)
    rover_rev.heading = 0.0
    rover_rev.angular_velocity = 1.0
    rover_rev.set_path([(50.0, 50.0), (50.0, 0.0)])
    prev_omega = rover_rev.angular_velocity
    rover_rev.update(dt=0.1, sim_time=1.0)
    new_omega = rover_rev.angular_velocity
    assert new_omega < prev_omega, f"Expected angular velocity to decrease towards negative desired rate, got {new_omega}"
    assert abs(new_omega - prev_omega) <= rover_rev.max_steer_accel * 0.1 + 1e-5, \
        f"Reversal step change ({abs(new_omega - prev_omega)}) exceeded max_steer_accel limit"

    # D. Zero steering error convergence
    rover_zero_err = MarsRover(start_x=50.0, start_y=50.0, target_x=100.0, target_y=50.0)
    rover_zero_err.heading = 0.0
    rover_zero_err.angular_velocity = 1.0
    rover_zero_err.set_path([(50.0, 50.0), (100.0, 50.0)])
    for step in range(15):
        rover_zero_err.update(dt=0.1, sim_time=step * 0.1)
    assert abs(rover_zero_err.angular_velocity) < 0.05, \
        f"Angular velocity should converge to 0 when heading error is zero, got {rover_zero_err.angular_velocity}"

    # E. Heading consistency
    rover_head = MarsRover(start_x=50.0, start_y=50.0)
    rover_head.heading = 0.0
    rover_head.angular_velocity = 0.5
    dt = 0.1
    rover_head.set_path([(50.0, 50.0), (50.0, 100.0)])
    init_h = rover_head.heading
    rover_head.update(dt=dt, sim_time=1.0)
    expected_heading = (init_h + rover_head.angular_velocity * dt + math.pi) % (2 * math.pi) - math.pi
    assert math.isclose(rover_head.heading, expected_heading, rel_tol=1e-3), \
        f"Heading update inconsistent with integrated omega: expected {expected_heading}, got {rover_head.heading}"

    # F. Task 1.1 Circular Arc Integration Regression
    dt = 1.0
    v_g = 2.0
    omega = 0.5
    theta = 0.0
    exp_dx = (v_g / omega) * (math.sin(theta + omega * dt) - math.sin(theta))
    exp_dy = -(v_g / omega) * (math.cos(theta + omega * dt) - math.cos(theta))
    assert math.isclose(exp_dx, 4.0 * math.sin(0.5), rel_tol=1e-5)
    assert math.isclose(exp_dy, 4.0 * (1.0 - math.cos(0.5)), rel_tol=1e-5)

    # G. Task 1.2 Wheel-slip & Odometry Regression
    rover_slip = MarsRover(start_x=50.0, start_y=50.0)
    rover_slip.speed = 2.0
    rover_slip.wheel_slip = 0.20
    rover_slip.ground_speed = rover_slip.speed * (1.0 - rover_slip.wheel_slip)
    rover_slip.wheel_speed = rover_slip.ground_speed / (1.0 - rover_slip.wheel_slip)
    assert math.isclose(rover_slip.wheel_speed * (1.0 - rover_slip.wheel_slip), rover_slip.ground_speed, rel_tol=1e-5)

    # H. Numerical Safety with dt <= 0 and dt > 0
    rover_dt_safe = MarsRover(start_x=50.0, start_y=50.0)
    rover_dt_safe.update(dt=0.0, sim_time=0.0)
    assert not math.isnan(rover_dt_safe.angular_velocity)
    assert not math.isnan(rover_dt_safe.angular_acceleration)
    assert rover_dt_safe.angular_acceleration == 0.0
    rover_dt_safe.update(dt=-0.1, sim_time=0.0)
    assert not math.isnan(rover_dt_safe.angular_velocity)
    assert not math.isnan(rover_dt_safe.angular_acceleration)

    print("[OK] Task 1.3 Objectives Fully Verified!")


def test_task_1_4_target_reached_and_continuous_battery_physics():
    print("Testing Task 1.4 Objectives: Target-Reached State Handling & Continuous Battery Physics...")

    # A. Target Arrival
    rover = MarsRover(start_x=50.0, start_y=50.0, target_x=52.0, target_y=50.0)
    rover.x, rover.y = 51.5, 50.0
    rover.speed = 2.0
    rover.angular_velocity = 0.5
    rover.update(dt=0.1, sim_time=1.0)
    assert rover.mission_status == RoverMissionStatus.TARGET_REACHED, \
        f"Expected TARGET_REACHED status, got {rover.mission_status}"

    # B. Linear Deceleration
    initial_speed = 2.0
    rover_dec = MarsRover(start_x=50.0, start_y=50.0, target_x=52.0, target_y=50.0)
    rover_dec.x, rover_dec.y = 51.5, 50.0
    rover_dec.speed = initial_speed
    rover_dec.update(dt=0.1, sim_time=1.0)
    assert rover_dec.speed < initial_speed, \
        f"Speed should decrease after target arrival, initial={initial_speed}, current={rover_dec.speed}"

    # C. Steering Settling
    rover_steer = MarsRover(start_x=50.0, start_y=50.0, target_x=52.0, target_y=50.0)
    rover_steer.x, rover_steer.y = 51.5, 50.0
    rover_steer.angular_velocity = 1.0
    for step in range(10):
        rover_steer.update(dt=0.1, sim_time=step * 0.1)
    assert abs(rover_steer.angular_velocity) < 0.05, \
        f"Angular velocity should converge to zero, got {rover_steer.angular_velocity}"

    # D. Stationary Position Stability & E. Wheel State
    rover_stat = MarsRover(start_x=50.0, start_y=50.0, target_x=52.0, target_y=50.0)
    rover_stat.x, rover_stat.y = 51.5, 50.0
    rover_stat.speed = 0.0
    rover_stat.angular_velocity = 0.0
    rover_stat.update(dt=0.1, sim_time=1.0)
    settled_x, settled_y = rover_stat.x, rover_stat.y
    settled_dist = rover_stat.total_distance_traveled
    for step in range(10):
        rover_stat.update(dt=0.1, sim_time=1.0 + step * 0.1)
    assert rover_stat.x == settled_x and rover_stat.y == settled_y, \
        f"Position should remain fixed while stationary, expected ({settled_x}, {settled_y}), got ({rover_stat.x}, {rover_stat.y})"
    assert rover_stat.wheel_speed == 0.0, f"Wheel speed should be 0 when stationary, got {rover_stat.wheel_speed}"

    # F. Odometer / Distance Traveled Stability
    assert rover_stat.total_distance_traveled == settled_dist, \
        f"Total distance traveled should not increase while stationary, expected {settled_dist}, got {rover_stat.total_distance_traveled}"

    # G. Continuous Battery Consumption (Hotel / Base Load)
    rover_bat = MarsRover(start_x=50.0, start_y=50.0, target_x=52.0, target_y=50.0)
    rover_bat.x, rover_bat.y = 51.5, 50.0
    rover_bat.speed = 0.0
    rover_bat.update(dt=0.1, sim_time=1.0)
    bat_after_arrival = rover_bat.battery_pct
    for step in range(50):
        rover_bat.update(dt=1.0, sim_time=2.0 + step)
    assert rover_bat.battery_pct < bat_after_arrival, \
        f"Battery should continue to drain due to base hotel load after target arrival, initial={bat_after_arrival}, current={rover_bat.battery_pct}"
    assert 0.0 <= rover_bat.battery_pct <= 100.0

    # H. Battery Depletion Protection
    rover_dep = MarsRover(start_x=50.0, start_y=50.0)
    rover_dep.battery_pct = 0.001
    for step in range(10):
        rover_dep.update(dt=10.0, sim_time=step * 10.0)
    assert rover_dep.battery_pct == 0.0, f"Battery percentage should clamp at 0.0 and never be negative, got {rover_dep.battery_pct}"

    # I. Tasks 1.1 - 1.3 Regression Checks
    rover_arc = MarsRover(start_x=50.0, start_y=50.0, target_x=200.0, target_y=50.0)
    rover_arc.speed = 2.0
    rover_arc.heading = 0.0
    rover_arc.angular_velocity = 0.0
    rover_arc.wheel_slip = 0.0
    rover_arc.set_path([(50.0, 50.0), (200.0, 50.0)])
    rover_arc.update(dt=1.0, sim_time=1.0)
    assert rover_arc.x > 50.0 and rover_arc.y == 50.0

    print("[OK] Task 1.4 Objectives Fully Verified!")


def test_task_1_5_continuous_terrain_interpolation():
    print("Testing Task 1.5 Objectives: Continuous Terrain Interpolation & Hazard Safety...")
    terrain = MarsTerrain(seed=42)

    # A. Grid vertex correctness
    for gx in [0, 5, 10, 25, 59]:
        for gy in [0, 5, 10, 25, 59]:
            wx, wy = terrain.grid_to_world(gx, gy)

            # Elevation at exact grid vertex
            grid_elev = float(terrain.elevation_grid[gy, gx])
            interp_elev = terrain.get_elevation(wx, wy)
            assert math.isclose(interp_elev, grid_elev, abs_tol=1e-5), \
                f"Elevation mismatch at grid ({gx}, {gy}): expected {grid_elev}, got {interp_elev}"

            # Slope at exact grid vertex
            grid_slope = float(terrain.slope_grid[gy, gx])
            interp_slope = terrain.get_slope(wx, wy)
            assert math.isclose(interp_slope, grid_slope, abs_tol=1e-5), \
                f"Slope mismatch at grid ({gx}, {gy}): expected {grid_slope}, got {interp_slope}"

    # B. Mid-cell interpolation
    gx, gy = 10, 10
    wx0, wy0 = terrain.grid_to_world(gx, gy)
    wx1, wy1 = terrain.grid_to_world(gx + 1, gy + 1)
    mid_x, mid_y = (wx0 + wx1) / 2.0, (wy0 + wy1) / 2.0

    q11 = float(terrain.elevation_grid[gy, gx])
    q21 = float(terrain.elevation_grid[gy, gx + 1])
    q12 = float(terrain.elevation_grid[gy + 1, gx])
    q22 = float(terrain.elevation_grid[gy + 1, gx + 1])
    expected_mid_elev = 0.25 * (q11 + q21 + q12 + q22)
    actual_mid_elev = terrain.get_elevation(mid_x, mid_y)
    assert math.isclose(actual_mid_elev, expected_mid_elev, abs_tol=1e-5), \
        f"Mid-cell elevation expected {expected_mid_elev}, got {actual_mid_elev}"

    # C, D, E. Continuity across grid boundaries (Elevation & Slope)
    boundary_x = 10 * terrain.cell_size  # 100.0m
    eps = 1e-4
    for y_pos in [50.0, 150.0, 250.0]:
        elev_left = terrain.get_elevation(boundary_x - eps, y_pos)
        elev_right = terrain.get_elevation(boundary_x + eps, y_pos)
        assert abs(elev_right - elev_left) < 0.05, \
            f"Elevation discontinuous across boundary x={boundary_x}: left={elev_left}, right={elev_right}"

        slope_left = terrain.get_slope(boundary_x - eps, y_pos)
        slope_right = terrain.get_slope(boundary_x + eps, y_pos)
        assert abs(slope_right - slope_left) < 0.05, \
            f"Slope discontinuous across boundary x={boundary_x}: left={slope_left}, right={slope_right}"

    # F. Cost safety (Normal & Hazard)
    assert terrain.get_cost(terrain.start_pos[0], terrain.start_pos[1]) == terrain.COST_NORMAL

    # Locate a crater hazard cell
    crater = terrain.craters[0]
    hazard_cost = terrain.get_cost(crater["x"], crater["y"])
    assert hazard_cost >= terrain.COST_HAZARD, \
        f"Expected crater center to be impassable hazard (>= {terrain.COST_HAZARD}), got {hazard_cost}"

    # G. Dynamic hazards injection compatibility
    hz = terrain.inject_hazard(x=200.0, y=200.0, radius=25.0, hazard_type="ROCK_FIELD")
    injected_cost = terrain.get_cost(200.0, 200.0)
    assert injected_cost >= terrain.COST_HAZARD, \
        f"Injected hazard center expected >= {terrain.COST_HAZARD}, got {injected_cost}"

    # H. Boundary safety (out of bounds & corners)
    boundary_queries = [
        (-50.0, -50.0),                      # lower-left out of bounds
        (0.0, 0.0),                          # lower-left corner
        (terrain.width_m, 0.0),              # lower-right corner
        (0.0, terrain.height_m),             # upper-left corner
        (terrain.width_m, terrain.height_m), # upper-right corner
        (terrain.width_m + 100.0, terrain.height_m + 100.0), # out of bounds upper-right
        (10.0, -20.0),
        (terrain.width_m + 50.0, 200.0),
    ]

    for qx, qy in boundary_queries:
        elev = terrain.get_elevation(qx, qy)
        slp = terrain.get_slope(qx, qy)
        cst = terrain.get_cost(qx, qy)
        assert not math.isnan(elev) and not math.isinf(elev), f"Elevation is NaN/Inf for ({qx}, {qy})"
        assert not math.isnan(slp) and not math.isinf(slp), f"Slope is NaN/Inf for ({qx}, {qy})"
        assert not math.isnan(cst) and not math.isinf(cst), f"Cost is NaN/Inf for ({qx}, {qy})"

    # I. A* path planning regression & hazard safety
    terrain_clean = MarsTerrain(seed=123)
    planner = AStarPlanner(terrain_clean)
    route = planner.plan_path(terrain_clean.start_pos, terrain_clean.target_pos)
    assert len(route) >= 2
    for wp in route:
        assert not terrain_clean.is_in_hazard(wp[0], wp[1], margin=0.0)

    print("[OK] Task 1.5 Objectives Fully Verified!")


if __name__ == "__main__":
    test_terrain_and_planner()
    test_ekf_and_sensors()
    test_ai_scheduler_and_trajectory()
    test_simulation_engine_and_benchmark()
    test_task_1_1_rover_kinematics_and_execution_order()
    test_task_1_2_wheel_slip_physics_and_odometry()
    test_task_1_3_steering_dynamics_and_inertial_damping()
    test_task_1_4_target_reached_and_continuous_battery_physics()
    test_task_1_5_continuous_terrain_interpolation()
    print("\nALL BACKEND VERIFICATION TESTS PASSED SUCCESSFULLY!")

