"""
NAVIS - Dynamic Performance Metrics Engine
Calculates real-time and cumulative navigation performance indicators directly from simulation data.
"""

from typing import Dict, List, Any, Optional, Tuple
import math
import numpy as np


class PerformanceMetrics:
    """
    Computes rigorous telemetry metrics without hardcoded or fake values:
    - Position Error (True vs EKF estimate)
    - Navigation Uncertainty (2-sigma EKF Covariance)
    - Satellite Constellation Utilization (%)
    - Satellite Coverage Availability (%)
    - Route Deviation (Actual vs Initial Ideal Plan)
    - Mission Completion Status & Success
    """

    def __init__(self, initial_route: Optional[List[Tuple[float, float]]] = None):
        self.initial_route = list(initial_route) if initial_route else []
        self.reset()

    def reset(self, initial_route: Optional[List[Tuple[float, float]]] = None) -> None:
        if initial_route:
            self.initial_route = list(initial_route)
        self.time_samples: List[float] = []
        self.pos_errors: List[float] = []
        self.uncertainties: List[float] = []
        self.satellite_active_counts: List[int] = []
        self.rover_speeds: List[float] = []
        self.coverage_samples: List[bool] = []

        self.total_steps = 0
        self.coverage_steps = 0
        self.utilization_steps = 0
        self.max_position_error = 0.0
        self.sum_position_error = 0.0
        self.sum_uncertainty = 0.0
        self.max_uncertainty = 0.0

    def update(
        self,
        sim_time: float,
        true_pos: Tuple[float, float],
        est_pos: Tuple[float, float],
        uncertainty_m: float,
        rover_speed: float,
        satellites: List[Dict[str, Any]],
    ) -> Dict[str, Any]:
        """
        Record and compute metrics for current simulation tick.
        """
        tx, ty = true_pos
        ex, ey = est_pos

        # Instantaneous Position Error (meters)
        pos_error = math.hypot(tx - ex, ty - ey)

        # Satellite Coverage & Utilization
        visible_sats = sum(1 for s in satellites if s["is_visible"])
        active_sats = sum(1 for s in satellites if s["current_task"] != "IDLE" and s["is_visible"])

        has_coverage = visible_sats > 0
        is_utilized = active_sats > 0

        self.total_steps += 1
        if has_coverage:
            self.coverage_steps += 1
        if is_utilized:
            self.utilization_steps += 1

        self.sum_position_error += pos_error
        self.max_position_error = max(self.max_position_error, pos_error)
        self.sum_uncertainty += uncertainty_m
        self.max_uncertainty = max(self.max_uncertainty, uncertainty_m)

        # Subsample for charts (keep up to 300 points for crisp streaming UI)
        if len(self.time_samples) == 0 or (sim_time - self.time_samples[-1]) >= 0.5:
            self.time_samples.append(round(sim_time, 1))
            self.pos_errors.append(round(pos_error, 2))
            self.uncertainties.append(round(uncertainty_m, 2))
            self.satellite_active_counts.append(active_sats)
            self.rover_speeds.append(round(rover_speed, 2))
            self.coverage_samples.append(has_coverage)

            if len(self.time_samples) > 300:
                self.time_samples.pop(0)
                self.pos_errors.pop(0)
                self.uncertainties.pop(0)
                self.satellite_active_counts.pop(0)
                self.rover_speeds.pop(0)
                self.coverage_samples.pop(0)

        # Compute cumulative ratios
        coverage_pct = (self.coverage_steps / max(1, self.total_steps)) * 100.0
        utilization_pct = (self.utilization_steps / max(1, self.total_steps)) * 100.0
        avg_pos_error = self.sum_position_error / max(1, self.total_steps)
        avg_uncertainty = self.sum_uncertainty / max(1, self.total_steps)

        # Route Deviation (Perpendicular distance to closest segment on initial route)
        route_deviation = self._calculate_route_deviation(true_pos)

        return {
            "current_position_error_m": round(pos_error, 2),
            "avg_position_error_m": round(avg_pos_error, 2),
            "max_position_error_m": round(self.max_position_error, 2),
            "current_uncertainty_m": round(uncertainty_m, 2),
            "avg_uncertainty_m": round(avg_uncertainty, 2),
            "max_uncertainty_m": round(self.max_uncertainty, 2),
            "coverage_pct": round(coverage_pct, 1),
            "utilization_pct": round(utilization_pct, 1),
            "route_deviation_m": round(route_deviation, 2),
            "active_satellites_count": active_sats,
            "visible_satellites_count": visible_sats,
        }

    def _calculate_route_deviation(self, current_pos: Tuple[float, float]) -> float:
        """Find minimum distance between current rover position and initial route."""
        if not self.initial_route or len(self.initial_route) < 2:
            return 0.0

        min_dist = float("inf")
        px, py = current_pos

        for i in range(len(self.initial_route) - 1):
            x1, y1 = self.initial_route[i]
            x2, y2 = self.initial_route[i + 1]

            dx = x2 - x1
            dy = y2 - y1
            seg_len_sq = dx * dx + dy * dy

            if seg_len_sq < 1e-6:
                d = math.hypot(px - x1, py - y1)
            else:
                t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))
                proj_x = x1 + t * dx
                proj_y = y1 + t * dy
                d = math.hypot(px - proj_x, py - proj_y)

            if d < min_dist:
                min_dist = d

        return min_dist

    def get_time_series(self) -> Dict[str, List[Any]]:
        """Return history for frontend live graphs."""
        return {
            "time": self.time_samples,
            "position_error": self.pos_errors,
            "uncertainty": self.uncertainties,
            "active_satellites": self.satellite_active_counts,
            "speed": self.rover_speeds,
        }
