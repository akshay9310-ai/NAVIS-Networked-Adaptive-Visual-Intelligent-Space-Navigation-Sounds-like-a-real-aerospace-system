"""
NAVIS - Simulated Sensor Suite
Simulates Satellite PNT, IMU (with bias & drift), Wheel Odometry (with terrain slip),
and Visual Navigation Camera (with feature-tracking noise).
"""

from typing import Dict, Any, Optional, Tuple
import numpy as np
import math


class SensorSuite:
    """
    Produces realistic noisy measurements for the sensor fusion filter.
    """

    def __init__(self, seed: Optional[int] = 42):
        if seed is not None:
            np.random.seed(seed)

        # True previous state for differential sensors
        self.prev_true_x = 50.0
        self.prev_true_y = 50.0
        self.prev_true_heading = 0.0

        # IMU Bias States (slow random walk drift)
        self.imu_bias_ax = 0.04  # m/s^2 initial bias
        self.imu_bias_ay = -0.03  # m/s^2 initial bias
        self.imu_bias_omega = 0.008  # rad/s initial bias (~0.45 deg/s)

        # Standard Deviations
        self.sigma_pnt = 0.45  # meters (when satellite lock active)
        self.sigma_imu_acc = 0.08  # m/s^2
        self.sigma_imu_gyro = 0.012  # rad/s
        self.sigma_bias_drift = 0.0005  # random walk step
        self.sigma_odo = 0.06  # m/s
        self.sigma_vo = 0.05  # m per step relative

    def reset(self, start_pos: Tuple[float, float], heading: float) -> None:
        """Reset sensor states."""
        self.prev_true_x, self.prev_true_y = start_pos
        self.prev_true_heading = heading
        self.imu_bias_ax = 0.04
        self.imu_bias_ay = -0.03
        self.imu_bias_omega = 0.008

    def sample(
        self,
        dt: float,
        true_x: float,
        true_y: float,
        true_vx: float,
        true_vy: float,
        true_heading: float,
        true_speed: float,
        true_accel: float,
        true_omega: float,
        wheel_slip: float,
        pnt_available: bool,
    ) -> Dict[str, Any]:
        """
        Sample all 4 navigation sensors with realistic physical noise and drift.
        """
        # 1. IMU Random Walk Bias Update
        self.imu_bias_ax += np.random.normal(0.0, self.sigma_bias_drift) * math.sqrt(dt)
        self.imu_bias_ay += np.random.normal(0.0, self.sigma_bias_drift) * math.sqrt(dt)
        self.imu_bias_omega += np.random.normal(0.0, self.sigma_bias_drift * 0.2) * math.sqrt(dt)

        # True body accelerations
        body_ax = true_accel
        body_ay = true_speed * true_omega  # centripetal acceleration

        meas_ax = body_ax + self.imu_bias_ax + np.random.normal(0.0, self.sigma_imu_acc)
        meas_ay = body_ay + self.imu_bias_ay + np.random.normal(0.0, self.sigma_imu_acc)
        meas_omega = true_omega + self.imu_bias_omega + np.random.normal(0.0, self.sigma_imu_gyro)

        imu_data = {
            "active": True,
            "ax": float(meas_ax),
            "ay": float(meas_ay),
            "omega": float(meas_omega),
            "bias_ax": float(self.imu_bias_ax),
            "bias_ay": float(self.imu_bias_ay),
            "bias_omega": float(self.imu_bias_omega),
        }

        # 2. Wheel Odometry (affected by slip and wheel noise)
        effective_speed = true_speed * (1.0 - wheel_slip)
        meas_wheel_speed = max(0.0, effective_speed + np.random.normal(0.0, self.sigma_odo))
        meas_distance_step = meas_wheel_speed * dt

        odo_data = {
            "active": True,
            "speed": float(meas_wheel_speed),
            "distance_step": float(meas_distance_step),
            "slip_pct": float(wheel_slip * 100.0),
        }

        # 3. Visual Odometry (Camera Relative Frame Translation)
        true_dx = true_x - self.prev_true_x
        true_dy = true_y - self.prev_true_y
        true_dheading = (true_heading - self.prev_true_heading + math.pi) % (2 * math.pi) - math.pi

        meas_vo_dx = true_dx + np.random.normal(0.0, self.sigma_vo * math.sqrt(max(0.01, dt)))
        meas_vo_dy = true_dy + np.random.normal(0.0, self.sigma_vo * math.sqrt(max(0.01, dt)))
        meas_vo_dtheta = true_dheading + np.random.normal(0.0, 0.01 * math.sqrt(max(0.01, dt)))

        vo_data = {
            "active": True,
            "dx": float(meas_vo_dx),
            "dy": float(meas_vo_dy),
            "dtheta": float(meas_vo_dtheta),
            "feature_confidence": float(
                np.clip(0.92 - wheel_slip * 0.5 + np.random.normal(0.0, 0.03), 0.5, 0.99)
            ),
        }

        # 4. Satellite PNT (Absolute position when satellite is locked)
        if pnt_available:
            pnt_noise_x = np.random.normal(0.0, self.sigma_pnt)
            pnt_noise_y = np.random.normal(0.0, self.sigma_pnt)
            meas_pnt_x = true_x + pnt_noise_x
            meas_pnt_y = true_y + pnt_noise_y
            pnt_data = {
                "active": True,
                "x": float(meas_pnt_x),
                "y": float(meas_pnt_y),
                "accuracy_m": float(self.sigma_pnt * 2.0),
                "hdop": float(round(0.85 + np.random.uniform(0.0, 0.25), 2)),
            }
        else:
            pnt_data = {
                "active": False,
                "x": None,
                "y": None,
                "accuracy_m": None,
                "hdop": None,
            }

        # Update historical references
        self.prev_true_x = true_x
        self.prev_true_y = true_y
        self.prev_true_heading = true_heading

        return {
            "pnt": pnt_data,
            "imu": imu_data,
            "odometry": odo_data,
            "visual_odometry": vo_data,
        }
