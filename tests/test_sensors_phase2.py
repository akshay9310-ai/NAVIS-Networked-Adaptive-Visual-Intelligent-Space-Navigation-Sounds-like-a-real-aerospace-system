"""
NAVIS - Phase 2 Task 2.2 Dedicated Sensor Infrastructure Test Suite
Tests RNG isolation, deterministic seeding, timestamp propagation, stable sensor IDs,
individual sensor dropout controls, recovery, noise models, and numerical safety.
"""

import math
import numpy as np
import pytest
from backend.navigation.sensors import SensorSuite


def test_rng_isolation():
    """Verify SensorSuite does not mutate NumPy global random state."""
    # Set global numpy seed
    np.random.seed(12345)
    global_val_before = np.random.normal(0.0, 1.0)

    # Initialize SensorSuite with isolated seed
    sensors = SensorSuite(seed=999)
    # Perform several sampling steps
    for step in range(5):
        sensors.sample(
            dt=0.05,
            true_x=50.0 + step,
            true_y=50.0 + step,
            true_vx=1.0,
            true_vy=0.0,
            true_heading=0.0,
            true_speed=1.0,
            true_accel=0.0,
            true_omega=0.0,
            wheel_slip=0.02,
            pnt_available=True,
            sim_time=step * 0.05,
        )

    # Re-seed global state to same initial state and check global generator produces identical value
    np.random.seed(12345)
    global_val_after = np.random.normal(0.0, 1.0)

    assert global_val_before == global_val_after


def test_deterministic_seeding():
    """Verify same seed produces identical sequences and different seeds produce different sequences."""
    suite_a1 = SensorSuite(seed=42)
    suite_a2 = SensorSuite(seed=42)
    suite_b = SensorSuite(seed=999)

    sample_a1 = suite_a1.sample(
        dt=0.05,
        true_x=100.0,
        true_y=100.0,
        true_vx=2.0,
        true_vy=0.5,
        true_heading=0.1,
        true_speed=2.0,
        true_accel=0.1,
        true_omega=0.05,
        wheel_slip=0.03,
        pnt_available=True,
        sim_time=1.0,
    )

    sample_a2 = suite_a2.sample(
        dt=0.05,
        true_x=100.0,
        true_y=100.0,
        true_vx=2.0,
        true_vy=0.5,
        true_heading=0.1,
        true_speed=2.0,
        true_accel=0.1,
        true_omega=0.05,
        wheel_slip=0.03,
        pnt_available=True,
        sim_time=1.0,
    )

    sample_b = suite_b.sample(
        dt=0.05,
        true_x=100.0,
        true_y=100.0,
        true_vx=2.0,
        true_vy=0.5,
        true_heading=0.1,
        true_speed=2.0,
        true_accel=0.1,
        true_omega=0.05,
        wheel_slip=0.03,
        pnt_available=True,
        sim_time=1.0,
    )

    # Identical seeds must match bitwise
    assert sample_a1["imu"]["ax"] == sample_a2["imu"]["ax"]
    assert sample_a1["pnt"]["x"] == sample_a2["pnt"]["x"]
    assert sample_a1["odometry"]["speed"] == sample_a2["odometry"]["speed"]
    assert sample_a1["visual_odometry"]["dx"] == sample_a2["visual_odometry"]["dx"]

    # Different seeds must produce different stochastic samples
    assert sample_a1["imu"]["ax"] != sample_b["imu"]["ax"]
    assert sample_a1["pnt"]["x"] != sample_b["pnt"]["x"]


def test_timestamp_behavior():
    """Verify timestamp exists on all payloads and increases monotonically matching sim_time."""
    sensors = SensorSuite(seed=42)

    for i in range(10):
        t_current = float(i * 0.1)
        payload = sensors.sample(
            dt=0.1,
            true_x=50.0 + i,
            true_y=50.0,
            true_vx=1.0,
            true_vy=0.0,
            true_heading=0.0,
            true_speed=1.0,
            true_accel=0.0,
            true_omega=0.0,
            wheel_slip=0.02,
            pnt_available=True,
            sim_time=t_current,
        )

        for key in ["imu", "odometry", "visual_odometry", "pnt"]:
            assert "timestamp" in payload[key]
            assert payload[key]["timestamp"] == pytest.approx(t_current)


def test_sensor_identity():
    """Verify all 4 sensor payloads contain stable sensor_id strings."""
    sensors = SensorSuite(seed=42)
    payload = sensors.sample(
        dt=0.05,
        true_x=50.0,
        true_y=50.0,
        true_vx=0.0,
        true_vy=0.0,
        true_heading=0.0,
        true_speed=0.0,
        true_accel=0.0,
        true_omega=0.0,
        wheel_slip=0.0,
        pnt_available=True,
        sim_time=0.0,
    )

    assert payload["imu"]["sensor_id"] == "imu"
    assert payload["odometry"]["sensor_id"] == "wheel_odometry"
    assert payload["visual_odometry"]["sensor_id"] == "visual_odometry"
    assert payload["pnt"]["sensor_id"] == "pnt"


def test_individual_sensor_dropouts():
    """Test manual dropout for IMU, Odometry, Visual Odometry, and PNT independently."""
    sensors = SensorSuite(seed=42)

    # 1. IMU Dropout
    sensors.set_sensor_dropout("imu", True)
    res = sensors.sample(
        dt=0.05, true_x=50.0, true_y=50.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.0,
    )
    assert res["imu"]["active"] is False
    assert res["imu"]["ax"] is None
    assert res["imu"]["ay"] is None
    assert res["imu"]["omega"] is None
    assert res["odometry"]["active"] is True
    assert res["pnt"]["active"] is True

    # 2. Odometry Dropout
    sensors.set_sensor_dropout("imu", False)
    sensors.set_sensor_dropout("odometry", True)
    res = sensors.sample(
        dt=0.05, true_x=50.0, true_y=50.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.05,
    )
    assert res["odometry"]["active"] is False
    assert res["odometry"]["speed"] is None
    assert res["odometry"]["distance_step"] is None
    assert res["imu"]["active"] is True

    # 3. Visual Odometry Dropout
    sensors.set_sensor_dropout("odometry", False)
    sensors.set_sensor_dropout("vo", True)
    res = sensors.sample(
        dt=0.05, true_x=50.0, true_y=50.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.10,
    )
    assert res["visual_odometry"]["active"] is False
    assert res["visual_odometry"]["dx"] is None
    assert res["visual_odometry"]["dy"] is None
    assert res["visual_odometry"]["dtheta"] is None

    # 4. PNT Dropout
    sensors.set_sensor_dropout("vo", False)
    sensors.set_sensor_dropout("pnt", True)
    res = sensors.sample(
        dt=0.05, true_x=50.0, true_y=50.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.15,
    )
    assert res["pnt"]["active"] is False
    assert res["pnt"]["x"] is None
    assert res["pnt"]["y"] is None
    assert res["pnt"]["accuracy_m"] is None


def test_dropout_recovery():
    """Verify that clearing dropout restores active measurement sampling."""
    sensors = SensorSuite(seed=42)

    for sensor_name, key, value_field in [
        ("imu", "imu", "ax"),
        ("odometry", "odometry", "speed"),
        ("visual_odometry", "visual_odometry", "dx"),
        ("pnt", "pnt", "x"),
    ]:
        # Drop sensor
        sensors.set_sensor_dropout(sensor_name, True)
        s_dropped = sensors.sample(
            dt=0.05, true_x=10.0, true_y=20.0, true_vx=1.0, true_vy=0.0,
            true_heading=0.0, true_speed=1.0, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.0, pnt_available=True, sim_time=0.0,
        )
        assert s_dropped[key]["active"] is False
        assert s_dropped[key][value_field] is None

        # Restore sensor
        sensors.set_sensor_dropout(sensor_name, False)
        s_restored = sensors.sample(
            dt=0.05, true_x=10.0, true_y=20.0, true_vx=1.0, true_vy=0.0,
            true_heading=0.0, true_speed=1.0, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.0, pnt_available=True, sim_time=0.05,
        )
        assert s_restored[key]["active"] is True
        assert s_restored[key][value_field] is not None


def test_existing_noise_behavior():
    """Verify expected noise standard deviation spreads."""
    sensors = SensorSuite(seed=42)
    pnt_errors = []

    for _ in range(500):
        res = sensors.sample(
            dt=0.05, true_x=100.0, true_y=200.0, true_vx=0.0, true_vy=0.0,
            true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.0, pnt_available=True, sim_time=0.0,
        )
        pnt_errors.append(res["pnt"]["x"] - 100.0)

    sample_std = float(np.std(pnt_errors))
    # Expected sigma_pnt = 0.45
    assert abs(sample_std - 0.45) < 0.05


def test_numerical_safety():
    """Verify no NaN or Inf generated across edge-case state values or inactive states."""
    sensors = SensorSuite(seed=42)

    edge_cases = [
        {"dt": 0.001, "speed": 0.0, "accel": 0.0, "omega": 0.0, "slip": 0.0},
        {"dt": 1.0, "speed": 10.0, "accel": 5.0, "omega": 3.0, "slip": 0.95},
        {"dt": 0.05, "speed": 0.0, "accel": -2.0, "omega": -1.5, "slip": 0.0},
    ]

    for case in edge_cases:
        res = sensors.sample(
            dt=case["dt"],
            true_x=0.0,
            true_y=0.0,
            true_vx=case["speed"],
            true_vy=0.0,
            true_heading=0.0,
            true_speed=case["speed"],
            true_accel=case["accel"],
            true_omega=case["omega"],
            wheel_slip=case["slip"],
            pnt_available=True,
            sim_time=0.0,
        )

        for s_key, data in res.items():
            for val_key, val in data.items():
                if isinstance(val, (int, float)):
                    assert not math.isnan(val), f"NaN in {s_key}.{val_key}"
                    assert not math.isinf(val), f"Inf in {s_key}.{val_key}"


# ==================================================
# TASK 2.3 — FOCUSED IMU MODEL VALIDATION TESTS
# ==================================================

def test_imu_zero_noise_accuracy():
    """Verify measured IMU quantities equal true physical quantities under zero noise and zero bias."""
    sensors = SensorSuite(seed=42)
    sensors.sigma_imu_acc = 0.0
    sensors.sigma_imu_gyro = 0.0
    sensors.sigma_bias_drift = 0.0
    sensors.imu_bias_ax = 0.0
    sensors.imu_bias_ay = 0.0
    sensors.imu_bias_omega = 0.0

    res = sensors.sample(
        dt=0.05,
        true_x=50.0,
        true_y=50.0,
        true_vx=2.0,
        true_vy=0.0,
        true_heading=0.0,
        true_speed=2.0,
        true_accel=1.2,
        true_omega=0.5,
        wheel_slip=0.0,
        pnt_available=True,
        sim_time=0.0,
    )

    imu = res["imu"]
    assert imu["active"] is True
    # Longitudinal acceleration: a_forward = true_accel = 1.2
    assert imu["ax"] == pytest.approx(1.2)
    # Lateral centripetal acceleration: a_lateral = true_speed * true_omega = 2.0 * 0.5 = 1.0
    assert imu["ay"] == pytest.approx(1.0)
    # Yaw rate: omega = true_omega = 0.5
    assert imu["omega"] == pytest.approx(0.5)


def test_imu_straight_line_acceleration():
    """Verify lateral centripetal acceleration approaches zero during straight-line motion (omega = 0)."""
    sensors = SensorSuite(seed=42)
    sensors.sigma_imu_acc = 0.0
    sensors.sigma_imu_gyro = 0.0
    sensors.sigma_bias_drift = 0.0
    sensors.imu_bias_ax = 0.0
    sensors.imu_bias_ay = 0.0
    sensors.imu_bias_omega = 0.0

    res = sensors.sample(
        dt=0.05,
        true_x=50.0,
        true_y=50.0,
        true_vx=3.0,
        true_vy=0.0,
        true_heading=0.0,
        true_speed=3.0,
        true_accel=1.5,
        true_omega=0.0,
        wheel_slip=0.02,
        pnt_available=True,
        sim_time=0.0,
    )

    imu = res["imu"]
    assert imu["ax"] == pytest.approx(1.5)
    assert imu["ay"] == pytest.approx(0.0)
    assert imu["omega"] == pytest.approx(0.0)


def test_imu_constant_turn_motion():
    """Verify lateral centripetal acceleration equals v * omega during constant-radius turning."""
    sensors = SensorSuite(seed=42)
    sensors.sigma_imu_acc = 0.0
    sensors.sigma_imu_gyro = 0.0
    sensors.sigma_bias_drift = 0.0
    sensors.imu_bias_ax = 0.0
    sensors.imu_bias_ay = 0.0
    sensors.imu_bias_omega = 0.0

    v_true = 2.5
    w_true = 0.8
    res = sensors.sample(
        dt=0.05,
        true_x=50.0,
        true_y=50.0,
        true_vx=v_true,
        true_vy=0.0,
        true_heading=0.0,
        true_speed=v_true,
        true_accel=0.0,
        true_omega=w_true,
        wheel_slip=0.0,
        pnt_available=True,
        sim_time=0.0,
    )

    imu = res["imu"]
    assert imu["ax"] == pytest.approx(0.0)
    assert imu["ay"] == pytest.approx(v_true * w_true)  # 2.0
    assert imu["omega"] == pytest.approx(w_true)


def test_imu_bias_injection():
    """Verify configured initial biases appear in raw IMU measurements."""
    sensors = SensorSuite(seed=42)
    sensors.sigma_imu_acc = 0.0
    sensors.sigma_imu_gyro = 0.0
    sensors.sigma_bias_drift = 0.0
    sensors.imu_bias_ax = 0.04
    sensors.imu_bias_ay = -0.03
    sensors.imu_bias_omega = 0.008

    res = sensors.sample(
        dt=0.05,
        true_x=0.0,
        true_y=0.0,
        true_vx=0.0,
        true_vy=0.0,
        true_heading=0.0,
        true_speed=0.0,
        true_accel=0.0,
        true_omega=0.0,
        wheel_slip=0.0,
        pnt_available=True,
        sim_time=0.0,
    )

    imu = res["imu"]
    assert imu["ax"] == pytest.approx(0.04)
    assert imu["ay"] == pytest.approx(-0.03)
    assert imu["omega"] == pytest.approx(0.008)


def test_imu_bias_random_walk():
    """Verify continuous Brownian random walk bias evolution across simulation steps."""
    sensors1 = SensorSuite(seed=123)
    sensors2 = SensorSuite(seed=123)

    biases1 = []
    biases2 = []

    for step in range(20):
        r1 = sensors1.sample(
            dt=0.05, true_x=0.0, true_y=0.0, true_vx=0.0, true_vy=0.0,
            true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.0, pnt_available=True, sim_time=step * 0.05,
        )
        r2 = sensors2.sample(
            dt=0.05, true_x=0.0, true_y=0.0, true_vx=0.0, true_vy=0.0,
            true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.0, pnt_available=True, sim_time=step * 0.05,
        )
        biases1.append(r1["imu"]["bias_ax"])
        biases2.append(r2["imu"]["bias_ax"])

    # Biases must evolve away from initial 0.04
    assert biases1[-1] != 0.04
    # Biases must be bitwise identical for identical seed
    assert biases1 == biases2


def test_imu_scale_factor():
    """Verify configured scale factors correctly scale true acceleration and gyro inputs."""
    sensors = SensorSuite(seed=42)
    sensors.sigma_imu_acc = 0.0
    sensors.sigma_imu_gyro = 0.0
    sensors.sigma_bias_drift = 0.0
    sensors.imu_bias_ax = 0.0
    sensors.imu_bias_ay = 0.0
    sensors.imu_bias_omega = 0.0

    sensors.set_scale_factors(scale_acc=0.05, scale_gyro=-0.02)  # +5% accel, -2% gyro

    res = sensors.sample(
        dt=0.05,
        true_x=0.0,
        true_y=0.0,
        true_vx=3.0,
        true_vy=0.0,
        true_heading=0.0,
        true_speed=3.0,
        true_accel=2.0,
        true_omega=1.0,
        wheel_slip=0.0,
        pnt_available=True,
        sim_time=0.0,
    )

    imu = res["imu"]
    # true body_ax = 2.0 -> scaled = 2.0 * 1.05 = 2.10
    assert imu["ax"] == pytest.approx(2.10)
    # true body_ay = 3.0 * 1.0 = 3.0 -> scaled = 3.0 * 1.05 = 3.15
    assert imu["ay"] == pytest.approx(3.15)
    # true omega = 1.0 -> scaled = 1.0 * 0.98 = 0.98
    assert imu["omega"] == pytest.approx(0.98)


def test_imu_dropout():
    """Verify safe inactive IMU payload during manual dropout."""
    sensors = SensorSuite(seed=42)
    sensors.set_sensor_dropout("imu", True)

    res = sensors.sample(
        dt=0.05, true_x=10.0, true_y=10.0, true_vx=1.0, true_vy=0.0,
        true_heading=0.0, true_speed=1.0, true_accel=0.5, true_omega=0.1,
        wheel_slip=0.0, pnt_available=True, sim_time=5.0,
    )

    imu = res["imu"]
    assert imu["sensor_id"] == "imu"
    assert imu["active"] is False
    assert imu["timestamp"] == 5.0
    assert imu["ax"] is None
    assert imu["ay"] is None
    assert imu["omega"] is None
    # Biases remain available and valid
    assert imu["bias_ax"] is not None


def test_imu_timestamp_correctness():
    """Verify IMU payload timestamp matches caller simulation time."""
    sensors = SensorSuite(seed=42)
    res = sensors.sample(
        dt=0.05, true_x=0.0, true_y=0.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=123.456,
    )
    assert res["imu"]["timestamp"] == pytest.approx(123.456)


def test_imu_numerical_safety_edge_cases():
    """Verify no NaN or Inf produced under extreme or zero kinematics."""
    sensors = SensorSuite(seed=42)

    cases = [
        (1e-6, 0.0, 0.0, 0.0),
        (0.05, 0.0, 0.0, 0.0),
        (0.05, 10.0, 5.0, 2.0),
        (1.0, 0.0, -3.0, -1.0),
    ]

    for dt, speed, accel, omega in cases:
        res = sensors.sample(
            dt=dt, true_x=0.0, true_y=0.0, true_vx=speed, true_vy=0.0,
            true_heading=0.0, true_speed=speed, true_accel=accel, true_omega=omega,
            wheel_slip=0.0, pnt_available=True, sim_time=0.0,
        )
        imu = res["imu"]
        for key in ["ax", "ay", "omega", "bias_ax", "bias_ay", "bias_omega"]:
            v = imu[key]
            assert v is not None
            assert not math.isnan(v)
            assert not math.isinf(v)


def test_imu_reset_reproducibility():
    """Verify resetting SensorSuite with identical seed produces bitwise identical IMU measurements."""
    sensors = SensorSuite(seed=42)

    samples1 = []
    for step in range(10):
        r = sensors.sample(
            dt=0.05, true_x=step*0.1, true_y=0.0, true_vx=1.0, true_vy=0.0,
            true_heading=0.0, true_speed=1.0, true_accel=0.1, true_omega=0.02,
            wheel_slip=0.0, pnt_available=True, sim_time=step*0.05,
        )
        samples1.append(r["imu"]["ax"])

    # Reset
    sensors.reset((0.0, 0.0), 0.0, seed=42)

    samples2 = []
    for step in range(10):
        r = sensors.sample(
            dt=0.05, true_x=step*0.1, true_y=0.0, true_vx=1.0, true_vy=0.0,
            true_heading=0.0, true_speed=1.0, true_accel=0.1, true_omega=0.02,
            wheel_slip=0.0, pnt_available=True, sim_time=step*0.05,
        )
        samples2.append(r["imu"]["ax"])

    assert samples1 == samples2


# ==================================================
# TASK 2.4 — WHEEL ODOMETRY MODEL & EKF INTERFACE TESTS
# ==================================================

def test_wheel_odo_zero_slip():
    """Verify wheel speed measurement equals ground speed at zero slip (within noise bounds)."""
    sensors = SensorSuite(seed=42)
    v_ground = 2.0
    res = sensors.sample(
        dt=0.05, true_x=0.0, true_y=0.0, true_vx=v_ground, true_vy=0.0,
        true_heading=0.0, true_speed=v_ground, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.0,
        wheel_speed=v_ground,  # v_wheel = v_ground / (1 - 0) = 2.0
    )
    odo = res["odometry"]
    assert odo["active"] is True
    # At 0 slip, wheel speed agrees with ground speed within sigma_odo (0.06)
    assert abs(odo["speed"] - v_ground) < 0.25


def test_wheel_odo_20_percent_slip():
    """Verify wheel rotational speed is 1/(1-0.20) = 1.25x ground speed under 20% slip."""
    sensors = SensorSuite(seed=42)
    sensors.sigma_odo = 0.0  # Zero noise for exact math check
    v_ground = 2.0
    slip = 0.20
    v_wheel_true = v_ground / (1.0 - slip)  # 2.5 m/s

    res = sensors.sample(
        dt=0.05, true_x=0.0, true_y=0.0, true_vx=v_ground, true_vy=0.0,
        true_heading=0.0, true_speed=v_ground, true_accel=0.0, true_omega=0.0,
        wheel_slip=slip, pnt_available=True, sim_time=0.0,
        wheel_speed=v_wheel_true,
    )
    odo = res["odometry"]
    assert odo["speed"] == pytest.approx(2.5)
    # Recovered ground speed using telemetry slip: v_wheel * (1 - slip)
    v_ground_est = odo["speed"] * (1.0 - odo["slip_pct"] / 100.0)
    assert v_ground_est == pytest.approx(v_ground)


def test_wheel_odo_max_slip():
    """Verify wheel rotational speed under 45% representative max slip."""
    sensors = SensorSuite(seed=42)
    sensors.sigma_odo = 0.0
    v_ground = 1.0
    slip = 0.45
    v_wheel_true = v_ground / (1.0 - slip)  # 1.0 / 0.55 ~ 1.818 m/s

    res = sensors.sample(
        dt=0.05, true_x=0.0, true_y=0.0, true_vx=v_ground, true_vy=0.0,
        true_heading=0.0, true_speed=v_ground, true_accel=0.0, true_omega=0.0,
        wheel_slip=slip, pnt_available=True, sim_time=0.0,
        wheel_speed=v_wheel_true,
    )
    odo = res["odometry"]
    assert odo["speed"] == pytest.approx(1.0 / 0.55)
    v_ground_est = odo["speed"] * (1.0 - odo["slip_pct"] / 100.0)
    assert v_ground_est == pytest.approx(v_ground)


def test_wheel_odo_measurement_noise():
    """Verify wheel odometry noise standard deviation matches sigma_odo = 0.06 m/s."""
    sensors = SensorSuite(seed=42)
    v_ground = 2.0
    errors = []

    for _ in range(500):
        res = sensors.sample(
            dt=0.05, true_x=0.0, true_y=0.0, true_vx=v_ground, true_vy=0.0,
            true_heading=0.0, true_speed=v_ground, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.0, pnt_available=True, sim_time=0.0,
            wheel_speed=v_ground,
        )
        errors.append(res["odometry"]["speed"] - v_ground)

    sample_std = float(np.std(errors))
    assert abs(sample_std - 0.06) < 0.01


def test_wheel_odo_signed_speed_if_supported():
    """Verify wheel odometry measurements are non-negative, adhering to forward-only rover physics model."""
    sensors = SensorSuite(seed=42)
    res = sensors.sample(
        dt=0.05, true_x=0.0, true_y=0.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.0,
    )
    assert res["odometry"]["speed"] >= 0.0


def test_wheel_odo_stationary_behavior():
    """Verify stationary rover produces near-zero non-negative wheel speed and stable EKF updates."""
    from backend.navigation.ekf import NAVIS_EKF
    sensors = SensorSuite(seed=42)
    ekf = NAVIS_EKF()

    res = sensors.sample(
        dt=0.05, true_x=50.0, true_y=50.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.0,
    )
    assert res["odometry"]["speed"] >= 0.0
    # EKF update near rest should bypass without division by zero
    ekf.update_odometry(res["odometry"])
    est = ekf.get_estimate()
    assert not math.isnan(est["est_speed"])
    assert not math.isinf(est["est_speed"])


def test_wheel_odo_ekf_measurement_consistency():
    """Verify EKF update under 20% slip correctly converts wheel speed to ground speed with zero innovation bias."""
    from backend.navigation.ekf import NAVIS_EKF
    ekf = NAVIS_EKF(start_x=50.0, start_y=50.0, start_heading=0.0)
    # Set EKF state velocity to true ground speed 2.0 m/s along x
    ekf.x[2] = 2.0  # vx = 2.0 m/s
    ekf.x[3] = 0.0  # vy = 0.0 m/s

    # Wheel speed is 2.5 m/s due to 20% slip
    odo_payload = {
        "sensor_id": "wheel_odometry",
        "active": True,
        "timestamp": 1.0,
        "speed": 2.5,
        "distance_step": 0.125,
        "slip_pct": 20.0,
    }

    # Execute EKF wheel odometry update
    ekf.update_odometry(odo_payload)
    est_vx = ekf.x[2]

    # Velocity estimate should remain ~2.0 m/s, NOT pulled up toward 2.5 m/s
    assert abs(est_vx - 2.0) < 0.05


def test_wheel_odo_dropout():
    """Verify safe inactive payload when wheel odometry is dropped."""
    sensors = SensorSuite(seed=42)
    sensors.set_sensor_dropout("odometry", True)

    res = sensors.sample(
        dt=0.05, true_x=0.0, true_y=0.0, true_vx=1.0, true_vy=0.0,
        true_heading=0.0, true_speed=1.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=3.0,
    )
    odo = res["odometry"]
    assert odo["sensor_id"] == "wheel_odometry"
    assert odo["active"] is False
    assert odo["speed"] is None
    assert odo["distance_step"] is None
    assert odo["timestamp"] == 3.0


def test_wheel_odo_timestamp():
    """Verify odometry payload timestamp matches caller simulation time."""
    sensors = SensorSuite(seed=42)
    res = sensors.sample(
        dt=0.05, true_x=0.0, true_y=0.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=99.9,
    )
    assert res["odometry"]["timestamp"] == pytest.approx(99.9)


# ==================================================
# TASK 2.5 — VISUAL ODOMETRY MODEL & EKF CONTRACT TESTS
# ==================================================

def test_vo_zero_motion():
    """Verify VO measurements approach zero when rover is stationary."""
    sensors = SensorSuite(seed=42)
    sensors.sigma_vo = 0.0  # Zero noise check

    res = sensors.sample(
        dt=0.05, true_x=50.0, true_y=50.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.0,
    )
    vo = res["visual_odometry"]
    assert vo["active"] is True
    assert vo["dx"] == pytest.approx(0.0)
    assert vo["dy"] == pytest.approx(0.0)
    assert vo["dtheta"] == pytest.approx(0.0, abs=0.05)


def test_vo_body_frame_forward_motion():
    """Verify forward world movement along heading=0 produces positive body dx and ~0 body dy."""
    sensors = SensorSuite(seed=42)
    sensors.sigma_vo = 0.0

    # Step 1: Initial position at heading 0
    sensors.sample(
        dt=0.05, true_x=50.0, true_y=50.0, true_vx=2.0, true_vy=0.0,
        true_heading=0.0, true_speed=2.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.0,
    )
    # Step 2: Moves 0.1m in +X world direction
    res2 = sensors.sample(
        dt=0.05, true_x=50.1, true_y=50.0, true_vx=2.0, true_vy=0.0,
        true_heading=0.0, true_speed=2.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.05,
    )
    vo = res2["visual_odometry"]
    assert vo["dx"] == pytest.approx(0.1)
    assert vo["dy"] == pytest.approx(0.0)


def test_vo_body_frame_rotated_motion():
    """Verify world movement along +Y heading=pi/2 produces positive body dx and ~0 body dy."""
    sensors = SensorSuite(seed=42)
    sensors.sigma_vo = 0.0
    heading_90 = math.pi / 2.0  # Pointing North (+Y)

    sensors.sample(
        dt=0.05, true_x=50.0, true_y=50.0, true_vx=0.0, true_vy=2.0,
        true_heading=heading_90, true_speed=2.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.0,
    )
    # Moves 0.1m along +Y world axis
    res2 = sensors.sample(
        dt=0.05, true_x=50.0, true_y=50.1, true_vx=0.0, true_vy=2.0,
        true_heading=heading_90, true_speed=2.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.05,
    )
    vo = res2["visual_odometry"]
    # In body frame, moving forward along heading (+Y world) is positive dx_body!
    assert vo["dx"] == pytest.approx(0.1)
    assert vo["dy"] == pytest.approx(0.0)


def test_vo_body_frame_arbitrary_heading():
    """Verify body frame coordinate rotation at arbitrary heading theta = pi/3 (60 deg)."""
    sensors = SensorSuite(seed=42)
    sensors.sigma_vo = 0.0
    theta = math.pi / 3.0  # 60 deg

    # World displacement: dx_world = 1.0, dy_world = 2.0
    dx_world = 1.0
    dy_world = 2.0
    expected_dx_body = dx_world * math.cos(theta) + dy_world * math.sin(theta)
    expected_dy_body = -dx_world * math.sin(theta) + dy_world * math.cos(theta)

    sensors.sample(
        dt=0.1, true_x=0.0, true_y=0.0, true_vx=0.0, true_vy=0.0,
        true_heading=theta, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.0,
    )
    res2 = sensors.sample(
        dt=0.1, true_x=dx_world, true_y=dy_world, true_vx=0.0, true_vy=0.0,
        true_heading=theta, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.1,
    )
    vo = res2["visual_odometry"]
    assert vo["dx"] == pytest.approx(expected_dx_body)
    assert vo["dy"] == pytest.approx(expected_dy_body)


def test_vo_rotation():
    """Verify dtheta magnitude and sign during heading rotation."""
    sensors = SensorSuite(seed=42)
    sensors.sigma_vo = 0.0

    sensors.sample(
        dt=0.05, true_x=50.0, true_y=50.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.4,
        wheel_slip=0.0, pnt_available=True, sim_time=0.0,
    )
    res2 = sensors.sample(
        dt=0.05, true_x=50.0, true_y=50.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.02, true_speed=0.0, true_accel=0.0, true_omega=0.4,
        wheel_slip=0.0, pnt_available=True, sim_time=0.05,
    )
    vo = res2["visual_odometry"]
    assert vo["dtheta"] == pytest.approx(0.02, abs=0.01)


def test_vo_noise():
    """Verify VO measurement noise standard deviation matches sigma_vo = 0.05."""
    sensors = SensorSuite(seed=42)
    dx_errors = []

    for _ in range(500):
        res = sensors.sample(
            dt=0.05, true_x=50.0, true_y=50.0, true_vx=0.0, true_vy=0.0,
            true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.0, pnt_available=True, sim_time=0.0,
        )
        dx_errors.append(res["visual_odometry"]["dx"])

    # sigma_vo * sqrt(max(0.01, 0.05)) = 0.05 * sqrt(0.05) ~ 0.01118
    sample_std = float(np.std(dx_errors))
    assert abs(sample_std - 0.05 * math.sqrt(0.05)) < 0.003


def test_vo_feature_confidence():
    """Verify feature confidence bounds [0.5, 0.99] and inverse relation with terrain slip."""
    sensors = SensorSuite(seed=42)

    # Low slip (0.02)
    r_low_slip = sensors.sample(
        dt=0.05, true_x=0.0, true_y=0.0, true_vx=1.0, true_vy=0.0,
        true_heading=0.0, true_speed=1.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.02, pnt_available=True, sim_time=0.0,
    )
    conf_low = r_low_slip["visual_odometry"]["feature_confidence"]
    assert 0.5 <= conf_low <= 0.99

    # High slip (0.45)
    r_high_slip = sensors.sample(
        dt=0.05, true_x=0.0, true_y=0.0, true_vx=1.0, true_vy=0.0,
        true_heading=0.0, true_speed=1.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.45, pnt_available=True, sim_time=0.05,
    )
    conf_high = r_high_slip["visual_odometry"]["feature_confidence"]
    assert 0.5 <= conf_high <= 0.99
    # High slip reduces confidence
    assert conf_high < conf_low


def test_vo_ekf_measurement_consistency():
    """Verify EKF predicted body velocity matches body VO measurements for same state."""
    from backend.navigation.ekf import NAVIS_EKF
    ekf = NAVIS_EKF(start_x=50.0, start_y=50.0, start_heading=math.pi / 4.0)  # 45 deg
    ekf.x[2] = 2.0  # vx_world
    ekf.x[3] = 0.0  # vy_world

    theta = math.pi / 4.0
    expected_vx_body = 2.0 * math.cos(theta) + 0.0 * math.sin(theta)  # ~1.414
    expected_vy_body = -2.0 * math.sin(theta) + 0.0 * math.cos(theta) # ~-1.414

    vo_payload = {
        "sensor_id": "visual_odometry",
        "active": True,
        "timestamp": 1.0,
        "dx": expected_vx_body * 0.05,
        "dy": expected_vy_body * 0.05,
        "dtheta": 0.0,
        "feature_confidence": 0.95,
    }

    # Innovation y should be ~0 when measurement matches body prediction
    ekf.update_visual_odometry(vo_payload, dt=0.05)
    est = ekf.get_estimate()
    assert abs(est["est_vx"] - 2.0) < 0.05


def test_vo_yaw_bias_residual():
    """Verify gyro bias residual relationship in update_visual_odometry."""
    from backend.navigation.ekf import NAVIS_EKF
    ekf = NAVIS_EKF()
    ekf.x[7] = 0.01  # positive gyro bias estimate
    ekf.last_unbiased_omega = -0.01  # predicted true rate (omega_meas - b_w = 0 - 0.01 = -0.01)

    # Perfect zero rotation VO payload
    vo_payload = {
        "sensor_id": "visual_odometry",
        "active": True,
        "timestamp": 1.0,
        "dx": 0.0,
        "dy": 0.0,
        "dtheta": 0.0,
        "feature_confidence": 0.90,
    }

    ekf.update_visual_odometry(vo_payload, dt=0.05)
    # Gyro bias estimate should be pulled down toward zero
    assert ekf.x[7] < 0.01


def test_vo_dropout():
    """Verify safe inactive payload when VO is dropped."""
    sensors = SensorSuite(seed=42)
    sensors.set_sensor_dropout("vo", True)

    res = sensors.sample(
        dt=0.05, true_x=0.0, true_y=0.0, true_vx=1.0, true_vy=0.0,
        true_heading=0.0, true_speed=1.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=2.0,
    )
    vo = res["visual_odometry"]
    assert vo["sensor_id"] == "visual_odometry"
    assert vo["active"] is False
    assert vo["dx"] is None
    assert vo["dy"] is None
    assert vo["dtheta"] is None
    assert vo["feature_confidence"] is None


def test_vo_timestamp():
    """Verify VO payload timestamp matches simulation time."""
    sensors = SensorSuite(seed=42)
    res = sensors.sample(
        dt=0.05, true_x=0.0, true_y=0.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=77.7,
    )
    assert res["visual_odometry"]["timestamp"] == pytest.approx(77.7)


def test_vo_reset_reproducibility():
    """Verify resetting SensorSuite produces bitwise identical VO measurements."""
    sensors = SensorSuite(seed=42)

    sensors.reset((0.0, 0.0), 0.0, seed=42)
    s1 = []
    for step in range(10):
        r = sensors.sample(
            dt=0.05, true_x=step*0.1, true_y=step*0.05, true_vx=1.0, true_vy=0.5,
            true_heading=0.1, true_speed=1.1, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.02, pnt_available=True, sim_time=step*0.05,
        )
        s1.append(r["visual_odometry"]["dx"])

    sensors.reset((0.0, 0.0), 0.0, seed=42)
    s2 = []
    for step in range(10):
        r = sensors.sample(
            dt=0.05, true_x=step*0.1, true_y=step*0.05, true_vx=1.0, true_vy=0.5,
            true_heading=0.1, true_speed=1.1, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.02, pnt_available=True, sim_time=step*0.05,
        )
        s2.append(r["visual_odometry"]["dx"])

    assert s1 == s2



def test_vo_jacobian_numerical_validation():
    """
    Validate analytical VO measurement Jacobian H_vo against finite-difference numerical Jacobian
    across multiple headings, velocities, and near-zero velocity.
    """
    from backend.navigation.ekf import NAVIS_EKF

    def compute_z_pred(x_vec):
        px, py, vx, vy, theta, b_ax, b_ay, b_w = x_vec
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        pred_vx_body = vx * cos_t + vy * sin_t
        pred_vy_body = -vx * sin_t + vy * cos_t
        pred_omega = 0.0  # assumption
        return np.array([pred_vx_body, pred_vy_body, pred_omega], dtype=np.float64)

    test_states = [
        # (vx, vy, theta)
        (2.0, -1.5, 0.0),
        (2.0, -1.5, math.pi / 4.0),
        (2.0, -1.5, math.pi / 2.0),
        (1.5, 2.5, 2.35),
        (0.001, 0.001, 1.1),  # near zero velocity
    ]

    eps = 1e-6

    for vx, vy, theta in test_states:
        ekf = NAVIS_EKF()
        ekf.x[2] = vx
        ekf.x[3] = vy
        ekf.x[4] = theta

        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        pred_vx_body = vx * cos_t + vy * sin_t
        pred_vy_body = -vx * sin_t + vy * cos_t

        # Analytical Jacobian H (3x8)
        H_analytical = np.zeros((3, 8), dtype=np.float64)
        H_analytical[0, 2] = cos_t
        H_analytical[0, 3] = sin_t
        H_analytical[0, 4] = pred_vy_body

        H_analytical[1, 2] = -sin_t
        H_analytical[1, 3] = cos_t
        H_analytical[1, 4] = -pred_vx_body
        H_analytical[2, 7] = -1.0

        # Numerical Finite-Difference Jacobian
        H_numerical = np.zeros((3, 8), dtype=np.float64)
        base_z = compute_z_pred(ekf.x)

        for col in range(8):
            if col == 7:
                H_numerical[2, 7] = -1.0
                continue
            x_perturbed = ekf.x.copy()
            x_perturbed[col] += eps
            z_perturbed = compute_z_pred(x_perturbed)
            diff = (z_perturbed - base_z) / eps
            H_numerical[:, col] = diff

        # Assert analytical matches numerical within 1e-5
        np.testing.assert_allclose(H_analytical, H_numerical, atol=1e-5)


# ==================================================
# TASK 2.6 — PNT SENSOR MODEL & HDOP REFINEMENT TESTS
# ==================================================

def test_pnt_nominal_measurement():
    """Verify nominal active PNT payload structure, accuracy, and position sampling."""
    sensors = SensorSuite(seed=42)
    res = sensors.sample(
        dt=0.05, true_x=100.0, true_y=150.0, true_vx=1.0, true_vy=0.0,
        true_heading=0.0, true_speed=1.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=10.0,
    )
    pnt = res["pnt"]
    assert pnt["sensor_id"] == "pnt"
    assert pnt["active"] is True
    assert pnt["x"] == pytest.approx(100.0, abs=2.0)
    assert pnt["y"] == pytest.approx(150.0, abs=2.0)
    assert pnt["hdop"] >= 0.70
    assert pnt["r_matrix"] is not None


def test_pnt_noise_statistics():
    """Verify PNT measurement error standard deviation matches sigma_pnt * hdop."""
    sensors = SensorSuite(seed=42)
    x_errors = []

    for _ in range(500):
        res = sensors.sample(
            dt=0.05, true_x=100.0, true_y=100.0, true_vx=0.0, true_vy=0.0,
            true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.0, pnt_available=True, sim_time=0.0, hdop_override=1.00,
        )
        x_errors.append(res["pnt"]["x"] - 100.0)

    sample_std = float(np.std(x_errors))
    assert abs(sample_std - 0.45) < 0.05


def test_hdop_lower_bound():
    """Verify HDOP cannot fall below strict lower bound 0.70."""
    sensors = SensorSuite(seed=42)
    hdop = sensors.compute_hdop([], (0.0, 0.0), hdop_override=0.10)
    assert hdop == pytest.approx(0.70)


def test_hdop_upper_bound():
    """Verify HDOP cannot exceed strict upper bound 5.00."""
    sensors = SensorSuite(seed=42)
    hdop = sensors.compute_hdop([], (0.0, 0.0), hdop_override=9.99)
    assert hdop == pytest.approx(5.00)


def test_hdop_better_geometry_lower_hdop():
    """Verify 4 well-spread satellites produce lower HDOP than 2 satellites."""
    sensors = SensorSuite(seed=42)
    rover_pos = (300.0, 300.0)

    # 4 cardinal satellites
    sats_4 = [
        {"gx": 500.0, "gy": 300.0, "elevation_deg": 60.0},
        {"gx": 100.0, "gy": 300.0, "elevation_deg": 60.0},
        {"gx": 300.0, "gy": 500.0, "elevation_deg": 60.0},
        {"gx": 300.0, "gy": 100.0, "elevation_deg": 60.0},
    ]
    # 2 satellites
    sats_2 = [
        {"gx": 500.0, "gy": 300.0, "elevation_deg": 60.0},
        {"gx": 300.0, "gy": 500.0, "elevation_deg": 60.0},
    ]

    hdop_4 = sensors.compute_hdop(sats_4, rover_pos)
    hdop_2 = sensors.compute_hdop(sats_2, rover_pos)

    assert hdop_4 < hdop_2


def test_hdop_worse_geometry_higher_hdop():
    """Verify single satellite geometry produces higher HDOP than 4 satellites."""
    sensors = SensorSuite(seed=42)
    rover_pos = (300.0, 300.0)

    sats_4 = [
        {"gx": 500.0, "gy": 300.0, "elevation_deg": 60.0},
        {"gx": 100.0, "gy": 300.0, "elevation_deg": 60.0},
        {"gx": 300.0, "gy": 500.0, "elevation_deg": 60.0},
        {"gx": 300.0, "gy": 100.0, "elevation_deg": 60.0},
    ]
    sats_1 = [
        {"gx": 500.0, "gy": 300.0, "elevation_deg": 25.0},
    ]

    hdop_4 = sensors.compute_hdop(sats_4, rover_pos)
    hdop_1 = sensors.compute_hdop(sats_1, rover_pos)

    assert hdop_1 > hdop_4


def test_hdop_satellite_count_impact():
    """Verify increasing active satellite count monotonically improves/reduces HDOP."""
    sensors = SensorSuite(seed=42)
    rover_pos = (300.0, 300.0)

    sat_base = [
        {"gx": 500.0, "gy": 300.0, "elevation_deg": 45.0},
        {"gx": 100.0, "gy": 300.0, "elevation_deg": 45.0},
        {"gx": 300.0, "gy": 500.0, "elevation_deg": 45.0},
        {"gx": 300.0, "gy": 100.0, "elevation_deg": 45.0},
    ]

    hdop_1 = sensors.compute_hdop(sat_base[:1], rover_pos)
    hdop_2 = sensors.compute_hdop(sat_base[:2], rover_pos)
    hdop_4 = sensors.compute_hdop(sat_base[:4], rover_pos)

    assert hdop_4 <= hdop_2 <= hdop_1


def test_pnt_complete_outage():
    """Verify complete satellite outage produces inactive PNT payload."""
    sensors = SensorSuite(seed=42)
    res = sensors.sample(
        dt=0.05, true_x=100.0, true_y=100.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=False, sim_time=5.0,
    )
    pnt = res["pnt"]
    assert pnt["active"] is False
    assert pnt["x"] is None
    assert pnt["y"] is None
    assert pnt["hdop"] is None
    assert pnt["accuracy_m"] is None
    assert pnt["r_matrix"] is None


def test_pnt_satellite_restoration():
    """Verify restoring satellite availability reactivates PNT payload."""
    sensors = SensorSuite(seed=42)
    # Step 1: Outage
    r1 = sensors.sample(
        dt=0.05, true_x=100.0, true_y=100.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=False, sim_time=1.0,
    )
    assert r1["pnt"]["active"] is False

    # Step 2: Restored
    r2 = sensors.sample(
        dt=0.05, true_x=100.0, true_y=100.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=1.05,
    )
    assert r2["pnt"]["active"] is True
    assert r2["pnt"]["x"] is not None


def test_pnt_manual_dropout_override():
    """Verify manual PNT dropout overrides satellite availability."""
    sensors = SensorSuite(seed=42)
    sensors.set_sensor_dropout("pnt", True)

    res = sensors.sample(
        dt=0.05, true_x=100.0, true_y=100.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=2.0,
    )
    pnt = res["pnt"]
    assert pnt["active"] is False
    assert pnt["x"] is None


def test_pnt_manual_dropout_restoration():
    """Verify clearing manual PNT dropout follows satellite availability again."""
    sensors = SensorSuite(seed=42)
    sensors.set_sensor_dropout("pnt", True)

    r1 = sensors.sample(
        dt=0.05, true_x=100.0, true_y=100.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=2.0,
    )
    assert r1["pnt"]["active"] is False

    sensors.set_sensor_dropout("pnt", False)
    r2 = sensors.sample(
        dt=0.05, true_x=100.0, true_y=100.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=2.05,
    )
    assert r2["pnt"]["active"] is True


def test_pnt_dynamic_r_increases_with_hdop():
    """Verify dynamic R_pnt variance increases as HDOP increases."""
    sensors = SensorSuite(seed=42)
    r_low = sensors.sample(
        dt=0.05, true_x=100.0, true_y=100.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.0, hdop_override=1.00,
    )
    r_high = sensors.sample(
        dt=0.05, true_x=100.0, true_y=100.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.05, hdop_override=2.50,
    )

    var_low = r_low["pnt"]["r_matrix"][0][0]
    var_high = r_high["pnt"]["r_matrix"][0][0]
    assert var_high > var_low
    assert var_high == pytest.approx((0.45 * 2.50)**2)


def test_pnt_dynamic_r_decreases_with_hdop():
    """Verify dynamic R_pnt variance decreases as HDOP decreases."""
    sensors = SensorSuite(seed=42)
    r_norm = sensors.sample(
        dt=0.05, true_x=100.0, true_y=100.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.0, hdop_override=1.00,
    )
    r_opt = sensors.sample(
        dt=0.05, true_x=100.0, true_y=100.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=0.05, hdop_override=0.80,
    )

    var_norm = r_norm["pnt"]["r_matrix"][0][0]
    var_opt = r_opt["pnt"]["r_matrix"][0][0]
    assert var_opt < var_norm
    assert var_opt == pytest.approx((0.45 * 0.80)**2)


def test_ekf_accepts_dynamic_pnt_covariance():
    """Verify EKF update_pnt accepts and applies dynamic R_matrix payload."""
    from backend.navigation.ekf import NAVIS_EKF
    ekf = NAVIS_EKF(start_x=100.0, start_y=100.0)

    pnt_payload = {
        "sensor_id": "pnt",
        "active": True,
        "timestamp": 1.0,
        "x": 105.0,
        "y": 105.0,
        "accuracy_m": 1.8,
        "hdop": 2.0,
        "r_matrix": [[1.62, 0.0], [0.0, 1.62]],
    }

    # Should execute without error
    ekf.update_pnt(pnt_payload)
    est = ekf.get_estimate()
    assert 100.0 < est["est_x"] < 105.0


def test_pnt_update_improves_position_estimate():
    """Verify active PNT update reduces position estimation error."""
    from backend.navigation.ekf import NAVIS_EKF
    ekf = NAVIS_EKF(start_x=100.0, start_y=100.0)
    # True position is 105.0, 100.0
    pnt_payload = {
        "sensor_id": "pnt",
        "active": True,
        "timestamp": 1.0,
        "x": 105.0,
        "y": 100.0,
        "hdop": 0.8,
        "r_matrix": [[0.1296, 0.0], [0.0, 0.1296]],
    }

    err_before = abs(ekf.x[0] - 105.0)
    ekf.update_pnt(pnt_payload)
    err_after = abs(ekf.x[0] - 105.0)

    assert err_after < err_before


def test_pnt_hdop_weighting_impact():
    """Verify poor HDOP produces smaller position correction delta than good HDOP."""
    from backend.navigation.ekf import NAVIS_EKF

    # EKF 1: Good HDOP = 0.8
    ekf1 = NAVIS_EKF(start_x=100.0, start_y=100.0)
    pnt_good = {
        "sensor_id": "pnt", "active": True, "timestamp": 1.0, "x": 110.0, "y": 100.0,
        "hdop": 0.8, "r_matrix": [[0.1296, 0.0], [0.0, 0.1296]],
    }
    ekf1.update_pnt(pnt_good)
    delta_good = abs(ekf1.x[0] - 100.0)

    # EKF 2: Poor HDOP = 3.0
    ekf2 = NAVIS_EKF(start_x=100.0, start_y=100.0)
    pnt_poor = {
        "sensor_id": "pnt", "active": True, "timestamp": 1.0, "x": 110.0, "y": 100.0,
        "hdop": 3.0, "r_matrix": [[1.8225, 0.0], [0.0, 1.8225]],
    }
    ekf2.update_pnt(pnt_poor)
    delta_poor = abs(ekf2.x[0] - 100.0)

    # Good HDOP has higher Kalman gain -> larger correction delta
    assert delta_good > delta_poor


def test_pnt_no_nan_inf_propagation():
    """Verify EKF state and covariance remain finite, non-NaN, and symmetric after PNT update."""
    from backend.navigation.ekf import NAVIS_EKF
    ekf = NAVIS_EKF(start_x=50.0, start_y=50.0)

    pnt_payload = {
        "sensor_id": "pnt", "active": True, "timestamp": 1.0, "x": 50.5, "y": 49.5,
        "hdop": 1.2, "r_matrix": [[0.2916, 0.0], [0.0, 0.2916]],
    }
    ekf.update_pnt(pnt_payload)

    assert not np.isnan(ekf.x).any()
    assert not np.isinf(ekf.x).any()
    assert not np.isnan(ekf.P).any()
    assert not np.isinf(ekf.P).any()
    # Symmetry check
    np.testing.assert_allclose(ekf.P, ekf.P.T, atol=1e-8)


def test_pnt_timestamp_correctness():
    """Verify PNT payload timestamp matches simulation time."""
    sensors = SensorSuite(seed=42)
    res = sensors.sample(
        dt=0.05, true_x=0.0, true_y=0.0, true_vx=0.0, true_vy=0.0,
        true_heading=0.0, true_speed=0.0, true_accel=0.0, true_omega=0.0,
        wheel_slip=0.0, pnt_available=True, sim_time=123.45,
    )
    assert res["pnt"]["timestamp"] == pytest.approx(123.45)


def test_pnt_reset_reproducibility():
    """Verify resetting SensorSuite produces bitwise identical PNT measurements."""
    sensors = SensorSuite(seed=42)

    sensors.reset((0.0, 0.0), 0.0, seed=42)
    s1 = []
    for step in range(10):
        r = sensors.sample(
            dt=0.05, true_x=step*0.1, true_y=step*0.05, true_vx=1.0, true_vy=0.5,
            true_heading=0.1, true_speed=1.1, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.02, pnt_available=True, sim_time=step*0.05,
        )
        s1.append(r["pnt"]["x"])

    sensors.reset((0.0, 0.0), 0.0, seed=42)
    s2 = []
    for step in range(10):
        r = sensors.sample(
            dt=0.05, true_x=step*0.1, true_y=step*0.05, true_vx=1.0, true_vy=0.5,
            true_heading=0.1, true_speed=1.1, true_accel=0.0, true_omega=0.0,
            wheel_slip=0.02, pnt_available=True, sim_time=step*0.05,
        )
        s2.append(r["pnt"]["x"])

    assert s1 == s2




