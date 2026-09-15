"""
NAVIS - Extended Kalman Filter (EKF) Multi-Sensor Fusion Engine
Fuses Satellite PNT, IMU (accelerometer/gyro), Wheel Odometry, and Visual Odometry.
Supports seamless autonomous fallback when PNT is denied/lost.
"""

from typing import Dict, Any, Tuple, Optional
import numpy as np
import math


class NAVIS_EKF:
    """
    8-State Extended Kalman Filter for planetary rover localization:
    State: x = [x, y, vx, vy, heading (theta), bias_ax, bias_ay, bias_omega]^T
    """

    def __init__(
        self,
        start_x: float = 50.0,
        start_y: float = 50.0,
        start_heading: float = 0.0,
    ):
        # State Vector (8x1)
        self.x = np.array(
            [start_x, start_y, 0.0, 0.0, start_heading, 0.0, 0.0, 0.0],
            dtype=np.float64,
        )

        # State Covariance Matrix P (8x8)
        self.P = np.diag(
            [
                0.25,  # var(x)
                0.25,  # var(y)
                0.1,  # var(vx)
                0.1,  # var(vy)
                0.01,  # var(heading)
                0.01,  # var(bias_ax)
                0.01,  # var(bias_ay)
                0.001,  # var(bias_omega)
            ]
        ).astype(np.float64)

        # Process Noise Covariance Q (8x8)
        self.Q = np.diag(
            [
                0.04,  # q_x
                0.04,  # q_y
                0.15,  # q_vx
                0.15,  # q_vy
                0.02,  # q_theta
                0.0001,  # q_bias_ax
                0.0001,  # q_bias_ay
                0.00005,  # q_bias_omega
            ]
        ).astype(np.float64)

        # Measurement Noise Covariances
        self.R_pnt = np.diag([0.25, 0.25])  # PNT pos noise
        self.R_vo = np.diag([0.08, 0.08, 0.015])  # VO dx, dy, dtheta noise
        self.R_odo = np.array([[0.06]])  # Odometry speed noise

        self.last_pnt_active = True
        self.fallback_active = False
        self.last_unbiased_omega = 0.0

    def reset(
        self,
        start_x: float = 50.0,
        start_y: float = 50.0,
        start_heading: float = 0.0,
    ) -> None:
        """Reset EKF state and covariance."""
        self.x = np.array(
            [start_x, start_y, 0.0, 0.0, start_heading, 0.0, 0.0, 0.0],
            dtype=np.float64,
        )
        self.P = np.diag([0.25, 0.25, 0.1, 0.1, 0.01, 0.01, 0.01, 0.001]).astype(np.float64)
        self.last_pnt_active = True
        self.fallback_active = False
        self.last_unbiased_omega = 0.0

    def predict(self, dt: float, imu_meas: Dict[str, Any]) -> None:
        """
        EKF Time Propagation / Prediction step driven by IMU acceleration & gyro.
        """
        # Unpack state
        px, py, vx, vy, theta, b_ax, b_ay, b_w = self.x

        if not imu_meas.get("active", True) or imu_meas.get("ax") is None:
            meas_ax, meas_ay, meas_w = b_ax, b_ay, b_w
        else:
            meas_ax = imu_meas["ax"]
            meas_ay = imu_meas["ay"]
            meas_w = imu_meas["omega"]

        # Compensate for estimated sensor bias
        unbiased_ax = meas_ax - b_ax
        unbiased_ay = meas_ay - b_ay
        unbiased_w = meas_w - b_w
        self.last_unbiased_omega = float(unbiased_w)

        # Transform body accelerations to navigation frame
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        world_ax = unbiased_ax * cos_t - unbiased_ay * sin_t
        world_ay = unbiased_ax * sin_t + unbiased_ay * cos_t

        # Non-linear State Transition
        new_px = px + vx * dt + 0.5 * world_ax * (dt**2)
        new_py = py + vy * dt + 0.5 * world_ay * (dt**2)
        new_vx = vx + world_ax * dt
        new_vy = vy + world_ay * dt
        new_theta = (theta + unbiased_w * dt + math.pi) % (2 * math.pi) - math.pi

        self.x = np.array(
            [new_px, new_py, new_vx, new_vy, new_theta, b_ax, b_ay, b_w],
            dtype=np.float64,
        )

        # State Transition Jacobian Matrix F (8x8)
        F = np.eye(8, dtype=np.float64)
        F[0, 2] = dt
        F[1, 3] = dt

        # Derivatives with respect to heading (theta)
        d_world_ax_dtheta = -unbiased_ax * sin_t - unbiased_ay * cos_t
        d_world_ay_dtheta = unbiased_ax * cos_t - unbiased_ay * sin_t

        F[0, 4] = 0.5 * d_world_ax_dtheta * (dt**2)
        F[1, 4] = 0.5 * d_world_ay_dtheta * (dt**2)
        F[2, 4] = d_world_ax_dtheta * dt
        F[3, 4] = d_world_ay_dtheta * dt

        # Derivatives with respect to accelerometer bias
        F[0, 5] = -0.5 * cos_t * (dt**2)
        F[0, 6] = 0.5 * sin_t * (dt**2)
        F[1, 5] = -0.5 * sin_t * (dt**2)
        F[1, 6] = -0.5 * cos_t * (dt**2)

        F[2, 5] = -cos_t * dt
        F[2, 6] = sin_t * dt
        F[3, 5] = -sin_t * dt
        F[3, 6] = -cos_t * dt

        # Derivative with respect to gyro bias
        F[4, 7] = -dt

        # Covariance propagation: P = F * P * F^T + Q * dt
        self.P = F @ self.P @ F.T + self.Q * dt

    def update_pnt(self, pnt_meas: Dict[str, Any], R_override: Optional[np.ndarray] = None) -> None:
        """
        EKF Measurement update with absolute Satellite PNT.
        z = [x_pnt, y_pnt]^T
        Accepts optional dynamic covariance R_override or extracts r_matrix / hdop from payload.
        """
        if not pnt_meas.get("active", False) or pnt_meas.get("x") is None:
            return

        z = np.array([pnt_meas["x"], pnt_meas["y"]], dtype=np.float64)

        # Dynamic R selection based on measurement quality / HDOP
        if R_override is not None:
            R_use = np.array(R_override, dtype=np.float64)
        elif pnt_meas.get("r_matrix") is not None:
            R_use = np.array(pnt_meas["r_matrix"], dtype=np.float64)
        elif pnt_meas.get("hdop") is not None:
            hdop = float(pnt_meas["hdop"])
            sigma_dyn = self.sigma_pnt * hdop
            R_use = np.diag([sigma_dyn**2, sigma_dyn**2])
        else:
            R_use = self.R_pnt

        # Measurement Jacobian H_pnt (2x8)
        H = np.zeros((2, 8), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0

        # Predicted measurement
        z_pred = self.x[0:2]

        # Innovation / Residual
        y = z - z_pred

        # Innovation Covariance S = H * P * H^T + R
        S = H @ self.P @ H.T + R_use

        # Kalman Gain K = P * H^T * inv(S)
        K = self.P @ H.T @ np.linalg.inv(S)

        # State & Covariance Update
        self.x = self.x + K @ y
        I = np.eye(8, dtype=np.float64)
        self.P = (I - K @ H) @ self.P

        # Ensure covariance matrix symmetry
        self.P = 0.5 * (self.P + self.P.T)

        # Normalize heading state
        self.x[4] = (self.x[4] + math.pi) % (2 * math.pi) - math.pi

    def update_odometry(self, odo_meas: Dict[str, Any]) -> None:
        """
        EKF Measurement update with Wheel Odometry speed.
        Compensates wheel rotational speed for terrain slip to obtain estimated ground speed:
        v_ground_est = v_wheel_meas * (1 - slip_ratio)
        """
        if not odo_meas.get("active", True) or odo_meas.get("speed") is None:
            return

        meas_wheel_speed = odo_meas["speed"]
        slip_pct = odo_meas.get("slip_pct", 0.0)
        slip_ratio = min(0.95, max(0.0, slip_pct / 100.0))

        # Convert measured wheel rotational speed to estimated ground speed using slip estimate
        meas_ground_speed = max(0.0, meas_wheel_speed * (1.0 - slip_ratio))

        vx, vy = self.x[2], self.x[3]
        pred_speed = math.hypot(vx, vy)

        if pred_speed < 0.05:
            return  # avoid division by zero near rest

        # Measurement Jacobian H_odo (1x8)
        H = np.zeros((1, 8), dtype=np.float64)
        H[0, 2] = vx / pred_speed
        H[0, 3] = vy / pred_speed

        z = np.array([meas_ground_speed], dtype=np.float64)
        z_pred = np.array([pred_speed], dtype=np.float64)
        y = z - z_pred

        S = H @ self.P @ H.T + self.R_odo
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K.flatten() * y[0]
        I = np.eye(8, dtype=np.float64)
        self.P = (I - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)
        self.x[4] = (self.x[4] + math.pi) % (2 * math.pi) - math.pi

    def update_visual_odometry(self, vo_meas: Dict[str, Any], dt: float) -> None:
        """
        EKF Measurement update with Visual Odometry relative body displacement:
        z = [dx_body/dt, dy_body/dt, dtheta/dt]^T => [v_body_x, v_body_y, omega]^T
        """
        if not vo_meas.get("active", True) or dt <= 0 or vo_meas.get("dx") is None:
            return

        meas_vx_body = vo_meas["dx"] / dt
        meas_vy_body = vo_meas["dy"] / dt
        meas_omega = vo_meas["dtheta"] / dt

        # Scaled noise based on feature confidence
        conf = vo_meas.get("feature_confidence", 0.9)
        R_scaled = self.R_vo / max(0.2, conf)

        z = np.array([meas_vx_body, meas_vy_body, meas_omega], dtype=np.float64)

        # Predicted measurement in body frame
        px, py, vx, vy, theta, b_ax, b_ay, b_w = self.x
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        pred_vx_body = vx * cos_t + vy * sin_t
        pred_vy_body = -vx * sin_t + vy * cos_t
        pred_omega = getattr(self, "last_unbiased_omega", -b_w)

        z_pred = np.array([pred_vx_body, pred_vy_body, pred_omega], dtype=np.float64)
        y = z - z_pred

        # Measurement Jacobian H_vo (3x8)
        H = np.zeros((3, 8), dtype=np.float64)
        H[0, 2] = cos_t
        H[0, 3] = sin_t
        H[0, 4] = pred_vy_body

        H[1, 2] = -sin_t
        H[1, 3] = cos_t
        H[1, 4] = -pred_vx_body

        H[2, 7] = -1.0

        S = H @ self.P @ H.T + R_scaled
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(8, dtype=np.float64)
        self.P = (I - K @ H) @ self.P
        self.P = 0.5 * (self.P + self.P.T)
        self.x[4] = (self.x[4] + math.pi) % (2 * math.pi) - math.pi

    def step(
        self,
        dt: float,
        sensors: Dict[str, Any],
    ) -> Dict[str, Any]:
        """
        Execute full EKF cycle (Predict + Update with available sensors).
        """
        # 1. Prediction with IMU
        self.predict(dt, sensors["imu"])

        # 2. Update with Wheel Odometry
        self.update_odometry(sensors["odometry"])

        # 3. Update with Visual Odometry
        self.update_visual_odometry(sensors["visual_odometry"], dt)

        # 4. Update with Satellite PNT (if locked)
        pnt_active = sensors["pnt"]["active"]
        if pnt_active:
            self.update_pnt(sensors["pnt"])
            self.fallback_active = False
        else:
            self.fallback_active = True

        self.last_pnt_active = pnt_active

        return self.get_estimate()

    def get_estimate(self) -> Dict[str, Any]:
        """
        Extract estimated state, uncertainty metrics, and 2-sigma covariance ellipse.
        """
        px, py, vx, vy, theta, b_ax, b_ay, b_w = self.x

        # 2x2 Position covariance sub-matrix
        P_pos = self.P[0:2, 0:2]

        # Eigen-decomposition for uncertainty ellipse
        eigenvals, eigenvecs = np.linalg.eigh(P_pos)
        # Ensure positive eigenvalues
        eigenvals = np.maximum(eigenvals, 1e-6)

        # 2-sigma semi-major & semi-minor axes (95.4% confidence interval)
        semi_major = 2.0 * math.sqrt(float(eigenvals[1]))
        semi_minor = 2.0 * math.sqrt(float(eigenvals[0]))

        # Orientation of the ellipse major axis
        angle_rad = math.atan2(float(eigenvecs[1, 1]), float(eigenvecs[0, 1]))

        # Total Position Uncertainty (scalar root-sum-square variance)
        pos_uncertainty = math.sqrt(float(self.P[0, 0] + self.P[1, 1]))

        speed_est = math.hypot(vx, vy)

        return {
            "est_x": float(px),
            "est_y": float(py),
            "est_vx": float(vx),
            "est_vy": float(vy),
            "est_speed": float(speed_est),
            "est_heading_rad": float(theta),
            "est_heading_deg": float((math.degrees(theta) + 360) % 360),
            "uncertainty_m": float(round(pos_uncertainty, 2)),
            "fallback_active": self.fallback_active,
            "pnt_active": self.last_pnt_active,
            "covariance_ellipse": {
                "semi_major_m": float(round(semi_major, 2)),
                "semi_minor_m": float(round(semi_minor, 2)),
                "angle_rad": float(round(angle_rad, 4)),
                "angle_deg": float(round(math.degrees(angle_rad), 1)),
            },
            "bias_estimates": {
                "b_ax": float(round(b_ax, 4)),
                "b_ay": float(round(b_ay, 4)),
                "b_omega": float(round(b_w, 5)),
            },
        }
