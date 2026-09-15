"""
NAVIS - Simulated Sensor Suite
Simulates Satellite PNT, IMU (with bias & drift), Wheel Odometry (with terrain slip),
and Visual Navigation Camera (with feature-tracking noise).
"""

from typing import Dict, Any, Optional, Tuple, List
import numpy as np
import math


class SensorSuite:
    """
    Produces realistic noisy measurements for the sensor fusion filter.
    Uses an isolated NumPy random Generator for deterministic reproducibility.
    """

    def __init__(self, seed: Optional[int] = 42):
        self.seed = seed
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        else:
            self.rng = np.random.default_rng()

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

        # Scale Factor Errors (default 0.0 = no scale error)
        self.scale_factor_acc = 0.0
        self.scale_factor_gyro = 0.0

        # Saturation Limits (default None = unlimited)
        self.max_meas_acc: Optional[float] = None
        self.max_meas_gyro: Optional[float] = None

        # Individual sensor dropout flags (True = dropped/inactive)
        self.sensor_dropout: Dict[str, bool] = {
            "imu": False,
            "wheel_odometry": False,
            "visual_odometry": False,
            "pnt": False,
        }

    def reset(self, start_pos: Tuple[float, float], heading: float, seed: Optional[int] = None) -> None:
        """Reset sensor states."""
        if seed is not None:
            self.seed = seed
        if self.seed is not None:
            self.rng = np.random.default_rng(self.seed)
        self.prev_true_x, self.prev_true_y = start_pos
        self.prev_true_heading = heading
        self.imu_bias_ax = 0.04
        self.imu_bias_ay = -0.03
        self.imu_bias_omega = 0.008

    def set_scale_factors(self, scale_acc: float = 0.0, scale_gyro: float = 0.0) -> None:
        """Set IMU accelerometer and gyroscope scale factor errors (unitless fraction)."""
        self.scale_factor_acc = float(scale_acc)
        self.scale_factor_gyro = float(scale_gyro)

    def set_saturation_limits(self, max_acc: Optional[float] = None, max_gyro: Optional[float] = None) -> None:
        """Set IMU dynamic range saturation clipping limits (m/s^2 for accel, rad/s for gyro)."""
        self.max_meas_acc = float(max_acc) if max_acc is not None else None
        self.max_meas_gyro = float(max_gyro) if max_gyro is not None else None

    def set_sensor_dropout(self, sensor_name: str, dropped: bool = True) -> None:
        """
        Set individual sensor dropout status.
        If dropped is True, the sensor is rendered inactive.
        """
        name_map = {
            "imu": "imu",
            "odometry": "wheel_odometry",
            "wheel_odometry": "wheel_odometry",
            "visual_odometry": "visual_odometry",
            "vo": "visual_odometry",
            "pnt": "pnt",
        }
        key = name_map.get(sensor_name.lower())
        if key:
            self.sensor_dropout[key] = dropped
        else:
            raise ValueError(f"Unknown sensor name '{sensor_name}'. Valid names: imu, odometry, visual_odometry, pnt")

    def is_sensor_dropped(self, sensor_name: str) -> bool:
        """Check if a sensor is manually dropped."""
        name_map = {
            "imu": "imu",
            "odometry": "wheel_odometry",
            "wheel_odometry": "wheel_odometry",
            "visual_odometry": "visual_odometry",
            "vo": "visual_odometry",
            "pnt": "pnt",
        }
        key = name_map.get(sensor_name.lower(), sensor_name.lower())
        return self.sensor_dropout.get(key, False)

    def compute_hdop(
        self,
        active_satellites: Optional[List[Any]],
        rover_pos: Tuple[float, float],
        hdop_override: Optional[float] = None,
    ) -> float:
        """
        Derive 2D Horizontal Dilution of Precision (HDOP) deterministically from satellite geometry.
        - Multiple well-spread satellites -> low HDOP (~0.70 to 1.20)
        - Single satellite or poor geometry -> higher HDOP (~2.00 to 3.50)
        - Strictly bounded within [0.70, 5.00]
        """
        if hdop_override is not None:
            return float(np.clip(hdop_override, 0.70, 5.00))

        if not active_satellites:
            return 1.00  # Default nominal HDOP when satellite constellation geometry is not explicitly supplied

        rover_x, rover_y = rover_pos
        rows = []
        for sat in active_satellites:
            if isinstance(sat, dict):
                gx = sat.get("gx", rover_x)
                gy = sat.get("gy", rover_y)
                el_deg = sat.get("elevation_deg", 45.0)
            else:
                gx = getattr(sat, "gx", rover_x)
                gy = getattr(sat, "gy", rover_y)
                el_deg = getattr(sat, "elevation_angle_deg", 45.0)

            dx = gx - rover_x
            dy = gy - rover_y
            dist = math.hypot(dx, dy)
            el_rad = math.radians(max(10.0, min(90.0, el_deg)))
            cos_el = math.cos(el_rad)

            if dist > 1e-3:
                u_x = (dx / dist) * cos_el
                u_y = (dy / dist) * cos_el
            else:
                u_x = 0.0
                u_y = 0.0
            rows.append([u_x, u_y])

        if not rows:
            return 1.00

        G = np.array(rows, dtype=np.float64)
        GtG = G.T @ G
        reg = 0.05 * np.eye(2, dtype=np.float64)
        inv_GtG = np.linalg.inv(GtG + reg)

        raw_hdop = math.sqrt(max(0.01, float(np.trace(inv_GtG))))
        hdop = float(np.clip(0.707 * raw_hdop, 0.70, 5.00))
        return hdop

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
        wheel_speed: Optional[float] = None,
        sim_time: float = 0.0,
        active_satellites: Optional[List[Any]] = None,
        hdop_override: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        Sample all 4 navigation sensors with realistic physical noise and drift.
        """
        timestamp = float(sim_time)

        # 1. IMU Random Walk Bias Update & Sampling
        self.imu_bias_ax += float(self.rng.normal(0.0, self.sigma_bias_drift) * math.sqrt(dt))
        self.imu_bias_ay += float(self.rng.normal(0.0, self.sigma_bias_drift) * math.sqrt(dt))
        self.imu_bias_omega += float(self.rng.normal(0.0, self.sigma_bias_drift * 0.2) * math.sqrt(dt))

        imu_dropped = self.sensor_dropout["imu"]
        if not imu_dropped:
            body_ax = true_accel
            body_ay = true_speed * true_omega  # centripetal acceleration

            raw_ax = body_ax * (1.0 + self.scale_factor_acc) + self.imu_bias_ax + float(self.rng.normal(0.0, self.sigma_imu_acc))
            raw_ay = body_ay * (1.0 + self.scale_factor_acc) + self.imu_bias_ay + float(self.rng.normal(0.0, self.sigma_imu_acc))
            raw_omega = true_omega * (1.0 + self.scale_factor_gyro) + self.imu_bias_omega + float(self.rng.normal(0.0, self.sigma_imu_gyro))

            if self.max_meas_acc is not None:
                meas_ax = float(np.clip(raw_ax, -self.max_meas_acc, self.max_meas_acc))
                meas_ay = float(np.clip(raw_ay, -self.max_meas_acc, self.max_meas_acc))
            else:
                meas_ax = raw_ax
                meas_ay = raw_ay

            if self.max_meas_gyro is not None:
                meas_omega = float(np.clip(raw_omega, -self.max_meas_gyro, self.max_meas_gyro))
            else:
                meas_omega = raw_omega

            imu_data = {
                "sensor_id": "imu",
                "active": True,
                "timestamp": timestamp,
                "ax": float(meas_ax),
                "ay": float(meas_ay),
                "omega": float(meas_omega),
                "bias_ax": float(self.imu_bias_ax),
                "bias_ay": float(self.imu_bias_ay),
                "bias_omega": float(self.imu_bias_omega),
            }
        else:
            imu_data = {
                "sensor_id": "imu",
                "active": False,
                "timestamp": timestamp,
                "ax": None,
                "ay": None,
                "omega": None,
                "bias_ax": float(self.imu_bias_ax),
                "bias_ay": float(self.imu_bias_ay),
                "bias_omega": float(self.imu_bias_omega),
            }

        # 2. Wheel Odometry (measures wheel rotational-equivalent linear speed + sensor noise)
        odo_dropped = self.sensor_dropout["wheel_odometry"]
        if not odo_dropped:
            if wheel_speed is not None:
                base_wheel_speed = wheel_speed
            else:
                safe_slip = min(0.95, max(0.0, wheel_slip))
                base_wheel_speed = true_speed / max(0.05, 1.0 - safe_slip)

            meas_wheel_speed = max(0.0, base_wheel_speed + float(self.rng.normal(0.0, self.sigma_odo)))
            meas_distance_step = meas_wheel_speed * dt

            odo_data = {
                "sensor_id": "wheel_odometry",
                "active": True,
                "timestamp": timestamp,
                "speed": float(meas_wheel_speed),
                "distance_step": float(meas_distance_step),
                "slip_pct": float(wheel_slip * 100.0),
            }
        else:
            odo_data = {
                "sensor_id": "wheel_odometry",
                "active": False,
                "timestamp": timestamp,
                "speed": None,
                "distance_step": None,
                "slip_pct": float(wheel_slip * 100.0),
            }

        # 3. Visual Odometry (Camera Relative Frame Translation)
        vo_dropped = self.sensor_dropout["visual_odometry"]
        true_dx_world = true_x - self.prev_true_x
        true_dy_world = true_y - self.prev_true_y
        true_dheading = (true_heading - self.prev_true_heading + math.pi) % (2 * math.pi) - math.pi

        if not vo_dropped:
            # Transform world displacement to camera/rover body frame
            cos_h = math.cos(true_heading)
            sin_h = math.sin(true_heading)
            true_dx_body = true_dx_world * cos_h + true_dy_world * sin_h
            true_dy_body = -true_dx_world * sin_h + true_dy_world * cos_h

            meas_vo_dx = true_dx_body + float(self.rng.normal(0.0, self.sigma_vo * math.sqrt(max(0.01, dt))))
            meas_vo_dy = true_dy_body + float(self.rng.normal(0.0, self.sigma_vo * math.sqrt(max(0.01, dt))))
            meas_vo_dtheta = true_dheading + float(self.rng.normal(0.0, 0.01 * math.sqrt(max(0.01, dt))))

            conf_raw = 0.92 - wheel_slip * 0.5 + float(self.rng.normal(0.0, 0.03))
            conf_clipped = max(0.5, min(0.99, conf_raw))

            vo_data = {
                "sensor_id": "visual_odometry",
                "active": True,
                "timestamp": timestamp,
                "dx": float(meas_vo_dx),
                "dy": float(meas_vo_dy),
                "dtheta": float(meas_vo_dtheta),
                "feature_confidence": float(conf_clipped),
            }
        else:
            vo_data = {
                "sensor_id": "visual_odometry",
                "active": False,
                "timestamp": timestamp,
                "dx": None,
                "dy": None,
                "dtheta": None,
                "feature_confidence": None,
            }

        # 4. Satellite PNT (Absolute position when satellite is locked and not dropped)
        pnt_dropped = self.sensor_dropout["pnt"]
        effective_pnt_active = pnt_available and (not pnt_dropped)

        if effective_pnt_active:
            hdop = self.compute_hdop(active_satellites, (true_x, true_y), hdop_override)
            sigma_pnt_dynamic = self.sigma_pnt * hdop

            pnt_noise_x = float(self.rng.normal(0.0, sigma_pnt_dynamic))
            pnt_noise_y = float(self.rng.normal(0.0, sigma_pnt_dynamic))
            meas_pnt_x = true_x + pnt_noise_x
            meas_pnt_y = true_y + pnt_noise_y

            r_var = float(sigma_pnt_dynamic**2)
            pnt_data = {
                "sensor_id": "pnt",
                "active": True,
                "timestamp": timestamp,
                "x": float(meas_pnt_x),
                "y": float(meas_pnt_y),
                "accuracy_m": float(round(sigma_pnt_dynamic * 2.0, 2)),
                "hdop": float(round(hdop, 2)),
                "r_matrix": [[r_var, 0.0], [0.0, r_var]],
            }
        else:
            pnt_data = {
                "sensor_id": "pnt",
                "active": False,
                "timestamp": timestamp,
                "x": None,
                "y": None,
                "accuracy_m": None,
                "hdop": None,
                "r_matrix": None,
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

