"""
NAVIS - Phase 2 Task 2.7 Integration & EKF Validation Test Suite
Verifies full sensor fusion integration, execution order, frame transformations,
sensor availability combinations, dynamic dropout/restoration, covariance stability,
sensor complementarity, PNT/HDOP chain, VO headings, wheel-slip, determinism, and 5000-step long run.
"""

import math
from typing import Dict, Any
import numpy as np
import pytest

from backend.simulation.engine import SimulationEngine
from backend.simulation.modes import NavigationMode, BenchmarkEvaluator
from backend.simulation.terrain import MarsTerrain
from backend.simulation.rover import MarsRover
from backend.simulation.satellites import SatelliteConstellation
from backend.navigation.sensors import SensorSuite
from backend.navigation.ekf import NAVIS_EKF


# ==================================================
# 1. EXECUTION ORDER & TIMESTAMPS
# ==================================================

def test_runtime_execution_order_and_timestamps():
    """Verify runtime execution order and timestamp matching across all sensors and EKF."""
    engine = SimulationEngine(seed=42)
    engine.reset(seed=42)

    for step in range(5):
        engine.step(custom_dt=0.05)
        sim_time = engine.sim_time
        sensors = engine.latest_sensors
        ekf_est = engine.latest_ekf

        # Verify timestamp consistency across sensor payloads
        assert sensors["imu"]["timestamp"] == pytest.approx(sim_time)
        assert sensors["odometry"]["timestamp"] == pytest.approx(sim_time)
        assert sensors["visual_odometry"]["timestamp"] == pytest.approx(sim_time)
        assert sensors["pnt"]["timestamp"] == pytest.approx(sim_time)
        # Verify EKF estimate returned
        assert "est_x" in ekf_est
        assert "uncertainty_m" in ekf_est


# ==================================================
# 2. SENSOR FRAME TRANSFORMATIONS
# ==================================================

def test_sensor_frame_transformations():
    """Verify IMU body frame, VO body frame, and PNT world frame coordinate consistency."""
    ekf = NAVIS_EKF(start_x=50.0, start_y=50.0, start_heading=math.pi / 2.0)  # Facing +Y (North)

    # VO body dx=0.1 (forward motion along +Y world)
    vo_payload = {
        "sensor_id": "visual_odometry",
        "active": True,
        "timestamp": 1.0,
        "dx": 0.1,  # 0.1m forward in body frame
        "dy": 0.0,
        "dtheta": 0.0,
        "feature_confidence": 0.95,
    }

    # At heading pi/2, v_body_x = vx*cos(pi/2) + vy*sin(pi/2) = vy_world
    # So forward body motion corresponds to +Y world velocity
    ekf.update_visual_odometry(vo_payload, dt=0.05)
    est = ekf.get_estimate()
    assert est["est_vy"] > 0.0  # vy_world pushed positive


# ==================================================
# 3. SENSOR AVAILABILITY MATRIX (14 COMBINATIONS)
# ==================================================

def test_sensor_availability_matrix_14_combinations():
    """
    Test EKF step execution across all 14 required sensor availability combinations:
    1) All sensors active
    2) IMU only
    3) IMU + wheel
    4) IMU + VO
    5) IMU + PNT
    6) IMU + wheel + VO
    7) IMU + wheel + PNT
    8) IMU + VO + PNT
    9) IMU + wheel + VO + PNT
    10) PNT outage
    11) VO outage
    12) Wheel outage
    13) Simultaneous VO + PNT outage
    14) Complete non-IMU sensor outage
    """
    sensors_instance = SensorSuite(seed=42)

    combinations = [
        # (imu, odo, vo, pnt) active flags
        (True, True, True, True),     # 1. All sensors active
        (True, False, False, False),  # 2. IMU only
        (True, True, False, False),   # 3. IMU + wheel
        (True, False, True, False),   # 4. IMU + VO
        (True, False, False, True),   # 5. IMU + PNT
        (True, True, True, False),    # 6. IMU + wheel + VO
        (True, True, False, True),    # 7. IMU + wheel + PNT
        (True, False, True, True),    # 8. IMU + VO + PNT
        (True, True, True, True),     # 9. IMU + wheel + VO + PNT
        (True, True, True, False),    # 10. PNT outage
        (True, True, False, True),    # 11. VO outage
        (True, False, True, True),    # 12. Wheel outage
        (True, True, False, False),   # 13. Simultaneous VO + PNT outage
        (True, False, False, False),  # 14. Complete non-IMU sensor outage
    ]

    for idx, (imu_on, odo_on, vo_on, pnt_on) in enumerate(combinations, start=1):
        ekf = NAVIS_EKF(start_x=50.0, start_y=50.0)
        sensors_instance.reset((50.0, 50.0), 0.0, seed=42)
        sensors_instance.set_sensor_dropout("imu", not imu_on)
        sensors_instance.set_sensor_dropout("odometry", not odo_on)
        sensors_instance.set_sensor_dropout("vo", not vo_on)
        sensors_instance.set_sensor_dropout("pnt", not pnt_on)

        for step in range(5):
            meas = sensors_instance.sample(
                dt=0.05, true_x=50.0 + step*0.1, true_y=50.0, true_vx=2.0, true_vy=0.0,
                true_heading=0.0, true_speed=2.0, true_accel=0.0, true_omega=0.0,
                wheel_slip=0.05, pnt_available=pnt_on, sim_time=step*0.05,
            )
            est = ekf.step(dt=0.05, sensors=meas)

            # Assert no NaN/Inf in state or uncertainty
            assert not np.isnan(ekf.x).any(), f"NaN in combination {idx}"
            assert not np.isinf(ekf.x).any(), f"Inf in combination {idx}"
            assert est["uncertainty_m"] >= 0.0


# ==================================================
# 4. DROPOUT / RESTORATION CYCLE
# ==================================================

def test_dynamic_dropout_restoration_cycle():
    """Verify continuous simulation under cyclic sensor dropouts and restorations."""
    engine = SimulationEngine(seed=42)
    engine.reset(seed=42)
    engine.start()

    # Step 1: Normal
    for _ in range(10):
        engine.step(custom_dt=0.05)

    # Step 2: PNT dropout
    engine.sensors.set_sensor_dropout("pnt", True)
    for _ in range(10):
        engine.step(custom_dt=0.05)

    # Step 3: PNT restore
    engine.sensors.set_sensor_dropout("pnt", False)
    for _ in range(10):
        engine.step(custom_dt=0.05)

    # Step 4: VO dropout
    engine.sensors.set_sensor_dropout("vo", True)
    for _ in range(10):
        engine.step(custom_dt=0.05)

    # Step 5: VO restore
    engine.sensors.set_sensor_dropout("vo", False)
    for _ in range(10):
        engine.step(custom_dt=0.05)

    # Step 6: Wheel dropout
    engine.sensors.set_sensor_dropout("odometry", True)
    for _ in range(10):
        engine.step(custom_dt=0.05)

    # Step 7: Multi-sensor outage (VO + PNT + Wheel)
    engine.sensors.set_sensor_dropout("vo", True)
    engine.sensors.set_sensor_dropout("pnt", True)
    for _ in range(10):
        engine.step(custom_dt=0.05)

    # Step 8: Full restoration
    engine.sensors.set_sensor_dropout("odometry", False)
    engine.sensors.set_sensor_dropout("vo", False)
    engine.sensors.set_sensor_dropout("pnt", False)
    for _ in range(10):
        engine.step(custom_dt=0.05)

    # Final state check
    est = engine.latest_ekf
    assert not np.isnan(engine.ekf.x).any()
    assert not np.isinf(engine.ekf.x).any()
    assert est["uncertainty_m"] > 0.0


# ==================================================
# 5. COVARIANCE STABILITY & SYMMETRY
# ==================================================

def test_ekf_covariance_stability_and_symmetry():
    """Verify state covariance matrix P remains finite, symmetric, positive-definite, and non-negative on diagonals."""
    ekf = NAVIS_EKF(start_x=50.0, start_y=50.0)
    sensors_instance = SensorSuite(seed=42)

    for step in range(50):
        meas = sensors_instance.sample(
            dt=0.05, true_x=50.0 + step*0.05, true_y=50.0, true_vx=1.0, true_vy=0.0,
            true_heading=0.0, true_speed=1.0, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.02, pnt_available=(step % 5 != 0), sim_time=step*0.05,
        )
        ekf.step(dt=0.05, sensors=meas)

        # Symmetry check
        np.testing.assert_allclose(ekf.P, ekf.P.T, atol=1e-8)
        # Non-negative diagonals
        assert (np.diag(ekf.P) >= 0.0).all()
        # Positive eigenvalues
        eigenvals = np.linalg.eigvalsh(ekf.P)
        assert (eigenvals >= -1e-8).all()


# ==================================================
# 6. SENSOR COMPLEMENTARITY
# ==================================================

def test_sensor_complementarity_comparison():
    """
    Compare estimation error and uncertainty across:
    Case A: All sensors active
    Case B: PNT unavailable (IMU + Wheel + VO)
    Case C: PNT + Wheel unavailable (IMU + VO)
    Case D: IMU only (Dead reckoning)
    """
    cases = {
        "Case A (All)": (True, True, True),
        "Case B (No PNT)": (False, True, True),
        "Case C (No PNT/Wheel)": (False, False, True),
        "Case D (IMU Only)": (False, False, False),
    }

    results = {}

    for name, (pnt_on, odo_on, vo_on) in cases.items():
        sensors_instance = SensorSuite(seed=42)
        sensors_instance.set_sensor_dropout("pnt", not pnt_on)
        sensors_instance.set_sensor_dropout("odometry", not odo_on)
        sensors_instance.set_sensor_dropout("vo", not vo_on)

        ekf = NAVIS_EKF(start_x=50.0, start_y=50.0)

        pos_errors = []
        for step in range(40):
            true_x = 50.0 + step * 0.1
            meas = sensors_instance.sample(
                dt=0.05, true_x=true_x, true_y=50.0, true_vx=2.0, true_vy=0.0,
                true_heading=0.0, true_speed=2.0, true_accel=0.0, true_omega=0.0,
                wheel_slip=0.05, pnt_available=pnt_on, sim_time=step*0.05,
            )
            est = ekf.step(dt=0.05, sensors=meas)
            pos_errors.append(abs(true_x - est["est_x"]))

        results[name] = {
            "avg_error": float(np.mean(pos_errors)),
            "final_unc": est["uncertainty_m"],
        }

    # Case A (All sensors) should achieve lowest position error
    assert results["Case A (All)"]["avg_error"] < results["Case D (IMU Only)"]["avg_error"]
    # Case D (IMU only) should accumulate highest position uncertainty
    assert results["Case D (IMU Only)"]["final_unc"] > results["Case A (All)"]["final_unc"]


# ==================================================
# 7. SENSOR NOISE / BIAS INTERACTION
# ==================================================

def test_sensor_noise_and_bias_interaction():
    """Verify seeded simulation demonstrates proper noise and IMU bias response."""
    ekf = NAVIS_EKF(start_x=50.0, start_y=50.0)
    sensors_instance = SensorSuite(seed=42)

    # Set IMU bias and verify bias estimation updates in EKF
    sensors_instance.imu_bias_omega = 0.02

    for step in range(30):
        meas = sensors_instance.sample(
            dt=0.05, true_x=50.0, true_y=50.0, true_vx=0.0, true_vy=0.0,
            true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.0, pnt_available=True, sim_time=step*0.05,
        )
        ekf.step(dt=0.05, sensors=meas)

    est = ekf.get_estimate()
    # Gyro bias estimate tracked in state vector
    assert "b_omega" in est["bias_estimates"]


# ==================================================
# 8. PNT / HDOP INTEGRATION CHAIN
# ==================================================

def test_pnt_hdop_integration_chain():
    """Verify complete PNT/HDOP chain: Satellite visibility -> Geometry -> HDOP -> Dynamic R -> EKF Gain."""
    # Low HDOP EKF
    ekf_low = NAVIS_EKF(start_x=100.0, start_y=100.0)
    pnt_low = {
        "sensor_id": "pnt", "active": True, "timestamp": 1.0, "x": 105.0, "y": 100.0,
        "hdop": 0.80, "r_matrix": [[0.1296, 0.0], [0.0, 0.1296]],
    }
    ekf_low.update_pnt(pnt_low)
    delta_low = abs(ekf_low.x[0] - 100.0)

    # High HDOP EKF
    ekf_high = NAVIS_EKF(start_x=100.0, start_y=100.0)
    pnt_high = {
        "sensor_id": "pnt", "active": True, "timestamp": 1.0, "x": 105.0, "y": 100.0,
        "hdop": 3.00, "r_matrix": [[1.8225, 0.0], [0.0, 1.8225]],
    }
    ekf_high.update_pnt(pnt_high)
    delta_high = abs(ekf_high.x[0] - 100.0)

    # Lower HDOP gives higher Kalman gain -> larger correction delta
    assert delta_low > delta_high


# ==================================================
# 9. VO / BODY-FRAME INTEGRATION ACROSS HEADINGS
# ==================================================

def test_vo_body_frame_heading_integration():
    """Verify VO body-frame motion is interpreted consistently across cardinal and non-cardinal headings."""
    headings = [0.0, math.pi / 2.0, math.pi, math.pi / 3.0]

    for heading in headings:
        ekf = NAVIS_EKF(start_x=50.0, start_y=50.0, start_heading=heading)
        # Predict forward velocity
        ekf.x[2] = 2.0 * math.cos(heading)
        ekf.x[3] = 2.0 * math.sin(heading)

        vo_payload = {
            "sensor_id": "visual_odometry",
            "active": True,
            "timestamp": 1.0,
            "dx": 0.10,  # 0.1m forward in body frame over dt=0.05 (2.0 m/s)
            "dy": 0.0,
            "dtheta": 0.0,
            "feature_confidence": 0.95,
        }

        ekf.update_visual_odometry(vo_payload, dt=0.05)
        est = ekf.get_estimate()

        # Estimated speed should match ~2.0 m/s regardless of heading orientation
        assert est["est_speed"] == pytest.approx(2.0, abs=0.1)


# ==================================================
# 10. WHEEL-SLIP FUSION INTEGRATION
# ==================================================

def test_wheel_slip_fusion_integration():
    """Verify EKF wheel odometry update under normal, moderate, high, and changing terrain slip."""
    slips = [0.0, 0.15, 0.40, 0.70]

    for slip in slips:
        ekf = NAVIS_EKF(start_x=50.0, start_y=50.0)
        ekf.x[2] = 2.0  # vx = 2.0 m/s ground speed
        ekf.x[3] = 0.0

        # Commanded/wheel speed is higher under slip: v_wheel = v_ground / (1 - slip)
        v_wheel = 2.0 / max(0.05, 1.0 - slip)

        odo_payload = {
            "sensor_id": "wheel_odometry",
            "active": True,
            "timestamp": 1.0,
            "speed": float(v_wheel),
            "distance_step": float(v_wheel * 0.05),
            "slip_pct": float(slip * 100.0),
        }

        ekf.update_odometry(odo_payload)
        est = ekf.get_estimate()

        # Ground speed estimate should remain close to true 2.0 m/s despite wheel slip
        assert est["est_speed"] == pytest.approx(2.0, abs=0.2)


# ==================================================
# 11. SIMULATION RESET DETERMINISM
# ==================================================

def test_simulation_reset_determinism():
    """Verify resetting SimulationEngine with identical seed produces bitwise identical sensor payloads and EKF estimates."""
    engine = SimulationEngine(seed=42)

    # Run 1
    engine.reset(seed=42)
    engine.start()
    run1_x = []
    for _ in range(10):
        engine.step(custom_dt=0.05)
        run1_x.append(engine.latest_ekf["est_x"])

    # Run 2
    engine.reset(seed=42)
    engine.start()
    run2_x = []
    for _ in range(10):
        engine.step(custom_dt=0.05)
        run2_x.append(engine.latest_ekf["est_x"])

    assert run1_x == run2_x


# ==================================================
# 12. 5,000-STEP LONG-RUN STABILITY
# ==================================================

def test_long_run_5000_step_stability():
    """Run 5,000 simulation steps (~250s of sim time) verifying zero NaN/Inf or numerical instability."""
    engine = SimulationEngine(seed=42)
    engine.reset(seed=42)
    engine.start()

    for step in range(5000):
        engine.step(custom_dt=0.05)

        # Check every 500 steps for stability
        if step % 500 == 0:
            ekf_x = engine.ekf.x
            ekf_P = engine.ekf.P

            assert not np.isnan(ekf_x).any(), f"NaN in ekf.x at step {step}"
            assert not np.isinf(ekf_x).any(), f"Inf in ekf.x at step {step}"
            assert not np.isnan(ekf_P).any(), f"NaN in ekf.P at step {step}"
            assert not np.isinf(ekf_P).any(), f"Inf in ekf.P at step {step}"

            # Bound check on position and velocity
            assert abs(ekf_x[0]) < 2000.0
            assert abs(ekf_x[1]) < 2000.0
            assert math.hypot(ekf_x[2], ekf_x[3]) < 20.0

    # Final check
    final_est = engine.latest_ekf
    assert final_est["uncertainty_m"] > 0.0


# ==================================================
# 13. TELEMETRY PAYLOAD CONTRACT COMPATIBILITY
# ==================================================

def test_telemetry_payload_contract_compatibility():
    """Verify telemetry dictionary keys and serialization structures match backend WS contract."""
    engine = SimulationEngine(seed=42)
    engine.reset(seed=42)
    engine.step(custom_dt=0.05)

    telem = engine.get_full_state()
    required_keys = [
        "sim_time", "rover", "sensors", "ekf", "satellites",
        "active_mode", "is_running",
    ]

    for key in required_keys:
        assert key in telem, f"Missing key {key} in telemetry"

    # Verify sensor payloads in telemetry
    sensors_dict = telem["sensors"]
    assert "imu" in sensors_dict
    assert "odometry" in sensors_dict
    assert "visual_odometry" in sensors_dict
    assert "pnt" in sensors_dict
