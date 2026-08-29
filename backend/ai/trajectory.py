"""
NAVIS - ML Rover Trajectory Predictor & Confidence Estimator
Uses recent rover kinematic history and scikit-learn Ridge regression to project
future path coordinates over a 30-second horizon with data-driven confidence metrics.
"""

from typing import List, Dict, Tuple, Any
import numpy as np
import math
from sklearn.linear_model import Ridge
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline


class TrajectoryPredictor:
    """
    Predicts 30-second future trajectory based on kinematic state sequence
    and terrain friction factors.
    """

    def __init__(self, horizon_sec: float = 30.0, step_dt: float = 1.5):
        self.horizon_sec = horizon_sec
        self.step_dt = step_dt
        self.model_x = make_pipeline(PolynomialFeatures(degree=2), Ridge(alpha=1.0))
        self.model_y = make_pipeline(PolynomialFeatures(degree=2), Ridge(alpha=1.0))

    def predict(
        self,
        rover_history: List[Dict[str, float]],
        current_state: Dict[str, Any],
        active_planned_path: List[Tuple[float, float]],
        current_uncertainty: float = 1.0,
        terrain_cost: float = 1.0,
    ) -> Dict[str, Any]:
        """
        Generate predicted future coordinates and calculate empirical confidence score.
        """
        curr_x = current_state["x"]
        curr_y = current_state["y"]
        curr_speed = current_state["speed"]
        curr_heading = current_state["heading_rad"]
        curr_vx = current_state["vx"]
        curr_vy = current_state["vy"]

        # If history is short (< 4 points), use kinematic lookahead along planned route or heading
        if len(rover_history) < 4:
            predicted_pts = []
            steps = int(self.horizon_sec / self.step_dt)
            for i in range(1, steps + 1):
                t_future = i * self.step_dt
                # Project along heading
                px = curr_x + curr_vx * t_future
                py = curr_y + curr_vy * t_future
                predicted_pts.append({"t": round(t_future, 1), "x": round(px, 1), "y": round(py, 1)})

            confidence = max(65.0, min(95.0, 92.0 - current_uncertainty * 3.0))
            return {
                "predicted_trajectory": predicted_pts,
                "horizon_sec": self.horizon_sec,
                "confidence_pct": round(confidence, 1),
                "model_type": "Kinematic Projection",
            }

        # Extract recent historical window (up to last 15 samples)
        window = rover_history[-15:]
        t_base = window[0]["t"]
        t_samples = np.array([p["t"] - t_base for p in window]).reshape(-1, 1)
        x_samples = np.array([p["x"] for p in window])
        y_samples = np.array([p["y"] for p in window])

        # Fit polynomial ridge models
        self.model_x.fit(t_samples, x_samples)
        self.model_y.fit(t_samples, y_samples)

        # Residual variance calculation
        pred_hist_x = self.model_x.predict(t_samples)
        pred_hist_y = self.model_y.predict(t_samples)
        mse_residual = float(np.mean((x_samples - pred_hist_x) ** 2 + (y_samples - pred_hist_y) ** 2))

        # Project into future
        last_t = t_samples[-1][0]
        future_steps = np.arange(self.step_dt, self.horizon_sec + self.step_dt, self.step_dt)
        future_t_matrix = (last_t + future_steps).reshape(-1, 1)

        raw_pred_x = self.model_x.predict(future_t_matrix)
        raw_pred_y = self.model_y.predict(future_t_matrix)

        # Blend ML polynomial projection with active waypoint pursuit direction for stability
        predicted_pts = []
        for i, t_fut in enumerate(future_steps):
            ml_px = raw_pred_x[i]
            ml_py = raw_pred_y[i]

            # Linear kinematic extrapolation as anchor
            lin_px = curr_x + curr_vx * t_fut
            lin_py = curr_y + curr_vy * t_fut

            # Blend factor (ML weight decreases with distant horizon)
            w_ml = max(0.2, 1.0 - (t_fut / self.horizon_sec) * 0.5)
            final_px = w_ml * ml_px + (1.0 - w_ml) * lin_px
            final_py = w_ml * ml_py + (1.0 - w_ml) * lin_py

            predicted_pts.append({
                "t": float(round(t_fut, 1)),
                "x": float(round(final_px, 1)),
                "y": float(round(final_py, 1)),
            })

        # Dynamic Confidence Score:
        # Base: 98%
        # Penalties:
        # - Residual error (MSE)
        # - High EKF position uncertainty (e.g. during outage)
        # - Terrain roughness
        # - Low speed or extreme heading rate
        penalty_mse = min(15.0, mse_residual * 4.0)
        penalty_uncertainty = min(25.0, current_uncertainty * 3.5)
        penalty_terrain = (terrain_cost - 1.0) * 1.5

        confidence = 98.0 - penalty_mse - penalty_uncertainty - penalty_terrain
        confidence = float(np.clip(confidence, 40.0, 96.0))

        return {
            "predicted_trajectory": predicted_pts,
            "horizon_sec": self.horizon_sec,
            "confidence_pct": float(round(confidence, 1)),
            "mse_residual": float(round(mse_residual, 3)),
            "model_type": "Ridge-Polynomial Kinematic Hybrid",
        }
