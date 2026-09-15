"""
NAVIS Phase 1 — Task 1.7: Final Phase 1 Integration & Hardening Verification Suite
Comprehensive integration tests verifying end-to-end data flow across Terrain, Rover,
Sensors, EKF, AI Constellation Scheduler, Trajectory Predictor, Hazard Detector, and A* Planner.
"""

import math
import os
import sys
import numpy as np
import pytest

# Add project root directory to sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from backend.simulation.terrain import MarsTerrain
from backend.simulation.rover import MarsRover, RoverMissionStatus
from backend.navigation.sensors import SensorSuite
from backend.navigation.ekf import NAVIS_EKF
from backend.navigation.planner import AStarPlanner
from backend.ai.scheduler import AISatelliteScheduler
from backend.ai.trajectory import TrajectoryPredictor
from backend.ai.hazard_detector import VisualHazardDetector
from backend.simulation.engine import SimulationEngine
from backend.simulation.modes import NavigationMode


def test_normal_navigation_integration():
    """Verify standard multi-step execution loop across Terrain, Rover, Sensors, and EKF."""
    terrain = MarsTerrain(seed=42)
    rover = MarsRover(
        start_x=terrain.start_pos[0],
        start_y=terrain.start_pos[1],
        target_x=terrain.target_pos[0],
        target_y=terrain.target_pos[1],
    )
    sensors = SensorSuite(seed=42)
    ekf = NAVIS_EKF(start_x=terrain.start_pos[0], start_y=terrain.start_pos[1])
    planner = AStarPlanner(terrain)

    path = planner.plan_path(rover.start_pos, rover.target_pos)
    rover.set_path(path)

    init_x, init_y = rover.x, rover.y
    dt = 0.05

    for step in range(50):
        sim_time = (step + 1) * dt
        cost = terrain.get_cost(rover.x, rover.y)
        slope = terrain.get_slope(rover.x, rover.y)

        # 1. Rover kinematics step
        rover.update(dt=dt, sim_time=sim_time, terrain_cost=cost, terrain_slope=slope, is_outage=False)

        # 2. Sensor sampling step
        meas = sensors.sample(
            dt=dt,
            true_x=rover.x,
            true_y=rover.y,
            true_vx=rover.vx,
            true_vy=rover.vy,
            true_heading=rover.heading,
            true_speed=rover.ground_speed,
            true_accel=rover.accel,
            true_omega=rover.angular_velocity,
            wheel_slip=rover.wheel_slip,
            pnt_available=True,
            wheel_speed=rover.wheel_speed,
        )

        # 3. EKF step
        est = ekf.step(dt=dt, sensors=meas)

        # Telemetry & numerical validity checks
        assert math.isfinite(rover.x) and math.isfinite(rover.y)
        assert math.isfinite(rover.vx) and math.isfinite(rover.vy)
        assert math.isfinite(rover.heading)
        assert rover.speed >= 0.0
        assert 0.0 <= rover.battery_pct <= 100.0
        assert 0.0 <= rover.wheel_slip <= 0.95
        assert math.isfinite(rover.wheel_speed)
        assert math.isfinite(est["est_x"]) and math.isfinite(est["est_y"])
        assert math.isfinite(est["uncertainty_m"])

    # Confirm rover moved
    assert rover.x != init_x or rover.y != init_y
    assert rover.total_distance_traveled > 0.0


def test_wheel_slip_integration():
    """Verify physical ground speed vs wheel speed consistency across varying terrain costs and slopes."""
    terrain = MarsTerrain(seed=42)
    rover = MarsRover(start_x=50.0, start_y=50.0)
    sensors = SensorSuite(seed=42)
    dt = 0.05

    for test_cost, test_slope in [(1.0, 0.0), (5.0, 0.1), (8.0, 0.35)]:
        rover.speed = 2.0
        rover.update(dt=dt, sim_time=1.0, terrain_cost=test_cost, terrain_slope=test_slope)

        # 1. Ground speed responds to slip
        expected_slip = float(np.clip(min(0.45, 0.02 + 0.04 * (test_cost - 1.0) + 0.15 * test_slope), 0.0, 0.95))
        assert math.isclose(rover.wheel_slip, expected_slip, abs_tol=1e-3)
        assert math.isclose(rover.ground_speed, rover.speed * (1.0 - rover.wheel_slip), abs_tol=1e-4)

        # 2. Wheel speed consistency: wheel_speed * (1 - slip) ≈ ground_speed
        assert math.isclose(rover.wheel_speed * (1.0 - rover.wheel_slip), rover.ground_speed, abs_tol=1e-4)

        # 3. Odometry receives wheel-side speed
        meas = sensors.sample(
            dt=dt,
            true_x=rover.x,
            true_y=rover.y,
            true_vx=rover.vx,
            true_vy=rover.vy,
            true_heading=rover.heading,
            true_speed=rover.ground_speed,
            true_accel=rover.accel,
            true_omega=rover.angular_velocity,
            wheel_slip=rover.wheel_slip,
            pnt_available=True,
            wheel_speed=rover.wheel_speed,
        )

        assert abs(meas["odometry"]["speed"] - rover.wheel_speed) < 0.25


def test_ekf_sensor_integration():
    """Verify execution ordering (rover -> sensors -> ekf) and state synchronization without 1-frame lags."""
    rover = MarsRover(start_x=50.0, start_y=50.0)
    sensors = SensorSuite(seed=42)
    ekf = NAVIS_EKF(start_x=50.0, start_y=50.0)

    dt = 0.05
    rover.set_path([(50.0, 50.0), (100.0, 50.0)])
    rover.speed = 2.0
    rover.heading = 0.0

    # Execute tick
    rover.update(dt=dt, sim_time=0.05, terrain_cost=1.0, terrain_slope=0.0)

    # Verify sensor prev state matches updated rover position immediately
    meas = sensors.sample(
        dt=dt,
        true_x=rover.x,
        true_y=rover.y,
        true_vx=rover.vx,
        true_vy=rover.vy,
        true_heading=rover.heading,
        true_speed=rover.ground_speed,
        true_accel=rover.accel,
        true_omega=rover.angular_velocity,
        wheel_slip=rover.wheel_slip,
        pnt_available=True,
        wheel_speed=rover.wheel_speed,
    )

    assert sensors.prev_true_x == rover.x
    assert sensors.prev_true_y == rover.y

    # EKF update
    est = ekf.step(dt=dt, sensors=meas)
    assert est["pnt_active"] is True
    assert est["uncertainty_m"] < 1.5
    assert math.isfinite(est["est_x"])


def test_satellite_outage_restoration_integration():
    """Verify seamless transition: PNT Active -> PNT Outage -> Fallback Navigation -> PNT Restoration."""
    engine = SimulationEngine(seed=42)
    engine.start()

    # 1. Normal navigation with PNT active
    for _ in range(20):
        engine.step(0.05)
    unc_pnt = engine.latest_ekf["uncertainty_m"]
    assert engine.latest_ekf["fallback_active"] is False

    # 2. Inject PNT outage
    engine.set_satellite_outage(True)
    for _ in range(30):
        engine.step(0.05)
    unc_outage = engine.latest_ekf["uncertainty_m"]
    assert engine.latest_ekf["fallback_active"] is True
    assert engine.rover.speed > 0.0  # rover continues moving during outage
    assert unc_outage >= unc_pnt

    # 3. Restore PNT
    engine.set_satellite_outage(False)
    for _ in range(15):
        engine.step(0.05)
    unc_restored = engine.latest_ekf["uncertainty_m"]
    assert engine.latest_ekf["fallback_active"] is False
    assert unc_restored < unc_outage


def test_dynamic_hazard_integration():
    """Verify hazard injection -> AI hazard detection -> A* replanning -> safe route execution pipeline."""
    engine = SimulationEngine(seed=42)
    engine.set_mode(NavigationMode.MODE_C)
    engine.start()

    # Initial route waypoints
    initial_route_len = len(engine.planner.current_route)
    assert initial_route_len >= 2

    # Inject dynamic rock field hazard ahead of rover along route
    hz = engine.inject_hazard(x=240.0, y=230.0, radius=30.0, hazard_type="ROCK_FIELD")

    # Step engine to allow AI hazard detector and planner to process
    for _ in range(10):
        engine.step(0.05)

    # Verify replan was triggered and current route updated
    assert engine.planner.replanned_count >= 1
    assert len(engine.planner.old_route) >= 2
    assert len(engine.rover.active_path) >= 2

    # Verify hazard center cost remains COST_HAZARD (1000.0)
    center_cost = engine.terrain.get_cost(hz["x"], hz["y"])
    assert center_cost >= engine.terrain.COST_HAZARD


def test_target_reached_integration():
    """Verify target arrival detection, linear deceleration, steering settling, stationary stability, and battery hotel load."""
    rover = MarsRover(start_x=50.0, start_y=50.0, target_x=52.0, target_y=50.0)
    rover.x, rover.y = 51.5, 50.0  # inside target_tolerance = 4.0m
    rover.speed = 2.0
    rover.angular_velocity = 0.5
    rover.set_path([(50.0, 50.0), (52.0, 50.0)])

    # Step 1: status updates to TARGET_REACHED
    rover.update(dt=0.1, sim_time=1.0)
    assert rover.mission_status == RoverMissionStatus.TARGET_REACHED

    # Settle over 100 steps (10s)
    for step in range(100):
        rover.update(dt=0.1, sim_time=1.0 + step * 0.1)

    assert math.isclose(rover.speed, 0.0, abs_tol=1e-4)
    assert math.isclose(rover.angular_velocity, 0.0, abs_tol=1e-4)

    settled_x, settled_y = rover.x, rover.y
    settled_dist = rover.total_distance_traveled
    bat_settled = rover.battery_pct

    # Run 20 additional stationary steps
    for step in range(20):
        rover.update(dt=0.1, sim_time=11.0 + step * 0.1)

    assert rover.x == settled_x and rover.y == settled_y
    assert rover.total_distance_traveled == settled_dist
    assert rover.battery_pct < bat_settled  # base hotel load drains battery
    assert rover.battery_pct >= 0.0


def test_long_run_stability():
    """Verify simulation stability across 500 steps (25.0s sim time) with zero NaNs, Infs, or invalid states."""
    engine = SimulationEngine(seed=42)
    engine.start()

    for _ in range(500):
        engine.step(0.05)

        state = engine.get_full_state()
        r = state["rover"]
        ekf = state["ekf"]

        assert math.isfinite(r["x"]) and math.isfinite(r["y"])
        assert math.isfinite(r["vx"]) and math.isfinite(r["vy"])
        assert 0.0 <= r["speed"] <= engine.rover.max_speed + 1e-4
        assert -math.pi <= r["heading_rad"] <= math.pi
        assert 0.0 <= r["battery_pct"] <= 100.0
        assert 0.0 <= r["wheel_slip"] <= 0.95
        assert math.isfinite(r["wheel_speed"])
        assert math.isfinite(ekf["est_x"]) and math.isfinite(ekf["est_y"])
        assert math.isfinite(ekf["uncertainty_m"])


def test_cross_module_telemetry_contract():
    """Verify completeness and field types of exported telemetry payload across rover and simulation engine."""
    engine = SimulationEngine(seed=42)
    engine.start()
    engine.step(0.05)

    full_state = engine.get_full_state()
    rover_data = full_state["rover"]

    contract_fields = [
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

    for key in contract_fields:
        assert key in rover_data, f"Telemetry contract broken: missing '{key}'"

    assert isinstance(rover_data["x"], float)
    assert isinstance(rover_data["battery_pct"], float)
    assert isinstance(rover_data["mission_status"], str)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
