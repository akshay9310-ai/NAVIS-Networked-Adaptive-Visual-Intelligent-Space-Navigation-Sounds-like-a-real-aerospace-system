"""
NAVIS - Visual Hazard Detection and Reconnaissance Engine
Simulates camera-based and orbital imaging visual recognition of Martian surface hazards
(Boulders, Crater Pits, Dangerous Slopes, Injected Anomalies).
"""

from typing import Dict, List, Any, Optional, Tuple
import math
import numpy as np


class VisualHazardDetector:
    """
    Scans forward path using rover camera FOV (up to 40m) and orbital imaging (ground footprint ~240m).
    Triggers replan events when dangerous obstacles lie along the rover's course.
    """

    def __init__(self, terrain):
        self.terrain = terrain
        self.detected_hazards: List[Dict[str, Any]] = []
        self.alert_history: List[Dict[str, Any]] = []
        self.last_detection_time = -100.0

    def scan_environment(
        self,
        rover_pos: Tuple[float, float],
        rover_heading: float,
        predicted_trajectory: List[Dict[str, float]],
        planned_path: List[Tuple[float, float]],
        imaging_sat_active: bool,
        sim_time: float,
    ) -> Dict[str, Any]:
        """
        Check for hazards intersecting the predicted path or planned path.
        """
        rx, ry = rover_pos

        # Check all terrain hazard sources (craters, injected hazards, steep rock clusters)
        all_hazards = []
        for cr in self.terrain.craters:
            all_hazards.append({
                "id": f"CR-{int(cr['x'])}-{int(cr['y'])}",
                "type": "CRATER PIT",
                "x": cr["x"],
                "y": cr["y"],
                "radius": cr["radius"] * 0.85,
            })
        for hz in self.terrain.injected_hazards:
            all_hazards.append({
                "id": hz["id"],
                "type": hz["type"],
                "x": hz["x"],
                "y": hz["y"],
                "radius": hz["radius"],
            })

        hazard_risk_level = 0.0
        closest_hazard_alert: Optional[Dict[str, Any]] = None
        min_dist_to_hazard = float("inf")

        # 1. Forward Camera Scan Range (up to 45m ahead in forward 70° FOV)
        camera_range = 45.0
        # 2. Orbital Satellite Scan (if SAT-03 or imaging satellite assigned)
        orbital_scan_range = 180.0 if imaging_sat_active else camera_range

        # Inspect points along predicted trajectory and remaining planned path
        points_to_check = []
        for pt in predicted_trajectory:
            points_to_check.append((pt["x"], pt["y"]))
        for wp in planned_path[:10]:
            points_to_check.append(wp)

        for hz in all_hazards:
            hx, hy, hrad = hz["x"], hz["y"], hz["radius"]
            dist_from_rover = math.hypot(hx - rx, hy - ry)

            # Is within orbital / camera recon coverage?
            if dist_from_rover > orbital_scan_range:
                continue

            # Check collision with predicted or planned points
            for px, py in points_to_check:
                dist_to_point = math.hypot(hx - px, hy - py)
                if dist_to_point <= (hrad + 6.0):  # hazard safety margin
                    # Found an upcoming collision!
                    threat_severity = max(0.5, 1.0 - (dist_from_rover / orbital_scan_range))
                    hazard_risk_level = max(hazard_risk_level, threat_severity)

                    if dist_from_rover < min_dist_to_hazard:
                        min_dist_to_hazard = dist_from_rover

                        # Calculate detection confidence (higher if orbital imaging active)
                        base_conf = 95.0 if imaging_sat_active else 88.0
                        detection_conf = float(np.clip(base_conf - (dist_from_rover / 25.0) + np.random.normal(0, 1.0), 75.0, 99.0))

                        closest_hazard_alert = {
                            "hazard_id": hz["id"],
                            "type": hz["type"],
                            "confidence_pct": round(detection_conf, 1),
                            "distance_m": round(dist_from_rover, 1),
                            "location": [round(hx, 1), round(hy, 1)],
                            "source": "SAT-03 ORBITAL IMAGERY" if imaging_sat_active else "ROVER FORWARD VISION",
                            "timestamp": sim_time,
                        }
                    break

        # Register alert if new hazard detected
        needs_replan = False
        if closest_hazard_alert is not None:
            # Check if this alert was already raised recently
            if (sim_time - self.last_detection_time) > 4.0:
                self.last_detection_time = sim_time
                self.alert_history.append(closest_hazard_alert)
                needs_replan = True

        return {
            "hazard_risk_level": float(round(hazard_risk_level, 2)),
            "active_alert": closest_hazard_alert,
            "needs_replan": needs_replan,
            "detected_count": len(self.alert_history),
        }
