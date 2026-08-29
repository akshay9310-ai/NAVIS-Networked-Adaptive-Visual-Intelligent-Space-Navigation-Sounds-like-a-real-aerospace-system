"""
NAVIS - Dynamic AI Satellite Scheduler
Allocates constrained satellite constellation resources (PNT, IMAGING, COMMUNICATION, IDLE)
using multi-criteria explainable scoring.
"""

from typing import Dict, List, Any, Optional

from backend.simulation.satellites import Satellite


class AISatelliteScheduler:
    """
    Intelligent constellation resource scheduler.
    Computes dynamic scores for all orbiters based on:
    - Visibility & Elevation angle (30%)
    - Rover Navigation Need / Uncertainty (25%)
    - Imaging / Hazard Reconnaissance Value (20%)
    - Ground Communication / Telemetry Need (15%)
    - Satellite Resource Availability / Power (10%)
    """

    # Weights
    W_VISIBILITY = 0.30
    W_ROVER_NEED = 0.25
    W_IMAGING_VAL = 0.20
    W_COMM_NEED = 0.15
    W_RESOURCE = 0.10

    def __init__(self) -> None:
        self.decisions_log: List[Dict[str, Any]] = []
        self.selected_satellite_id: Optional[str] = None
        self.selected_task: str = "IDLE"
        self.last_reason_summary: str = ""

    def evaluate_and_schedule(
        self,
        satellites: Dict[str, Satellite],
        rover_telemetry: Dict[str, Any],
        ekf_telemetry: Dict[str, Any],
        hazard_risk_level: float,  # 0.0 to 1.0 (high if hazard in predicted path)
        sim_time: float,
    ) -> Dict[str, Any]:
        """
        Run scoring algorithm across all satellites and assign optimal tasks.
        """
        uncertainty = ekf_telemetry.get("uncertainty_m", 1.0)
        is_fallback = ekf_telemetry.get("fallback_active", False)
        rover_speed = rover_telemetry.get("speed", 0.0)

        # 1. Compute Rover Need Score (0 to 1): High if uncertainty is high or in fallback
        rover_need_score = min(1.0, (uncertainty / 6.0) + (0.4 if is_fallback else 0.0))

        # 2. Compute Imaging Need Score (0 to 1): High if upcoming hazard risk is high
        imaging_need_score = min(1.0, hazard_risk_level * 0.9 + (0.1 if rover_speed > 1.5 else 0.0))

        # 3. Compute Comm Need Score (0 to 1): Periodic telemetry dump / mission status
        comm_need_score = min(1.0, 0.25 + 0.5 * (1.0 - rover_telemetry.get("battery_pct", 100.0) / 100.0))

        scores: Dict[str, float] = {}
        sub_scores: Dict[str, Dict[str, float]] = {}
        explanations: Dict[str, List[str]] = {}

        for sat_id, sat in satellites.items():
            if sat.is_outage_forced or not sat.is_visible_to_rover:
                sat.score = 0.0
                sat.set_task("IDLE", "Out of visible orbital cone or forced outage")
                scores[sat_id] = 0.0
                continue

            # Normalized visibility component (elevation angle from 15 to 90 deg)
            norm_vis = min(1.0, max(0.0, (sat.elevation_angle_deg - 15.0) / 75.0))

            # Resource availability (battery)
            norm_res = min(1.0, sat.battery_pct / 100.0)

            # Capability specific factors
            has_pnt = "PNT" in sat.capabilities
            has_img = "IMAGING" in sat.capabilities
            has_comm = "COMMUNICATION" in sat.capabilities

            # Determine task relevance for this specific satellite
            pnt_val = rover_need_score if has_pnt else 0.1
            img_val = imaging_need_score if has_img else 0.0
            comm_val = comm_need_score if has_comm else 0.1

            # Compute weighted composite score
            sat_score = (
                self.W_VISIBILITY * norm_vis
                + self.W_ROVER_NEED * pnt_val
                + self.W_IMAGING_VAL * img_val
                + self.W_COMM_NEED * comm_val
                + self.W_RESOURCE * norm_res
            )

            scores[sat_id] = sat_score
            sat.score = sat_score

            sub_scores[sat_id] = {
                "visibility": round(norm_vis, 2),
                "rover_need": round(pnt_val, 2),
                "imaging_val": round(img_val, 2),
                "comm_need": round(comm_val, 2),
                "resources": round(norm_res, 2),
            }

            # Build explainability reasons
            reasons = []
            if norm_vis > 0.6:
                reasons.append(f"High elevation angle ({sat.elevation_angle_deg:.0f}°)")
            if has_img and hazard_risk_level > 0.5:
                reasons.append("Hazard reconnaissance priority")
            if has_pnt and (uncertainty > 2.0 or is_fallback):
                reasons.append("PNT precision recovery needed")
            if norm_res > 0.8:
                reasons.append(f"Optimal battery ({sat.battery_pct:.0f}%)")

            explanations[sat_id] = reasons

        # Multi-task constellation assignment
        # Primary: Assign best satellite to highest need task
        visible_sats = [s for s in satellites.values() if s.is_visible_to_rover and not s.is_outage_forced]
        # Sort by score descending
        visible_sats.sort(key=lambda s: s.score, reverse=True)

        assigned_tasks = {}
        if visible_sats:
            # Decide tasks based on priority
            # Priority 1: If hazard risk > 0.6 and imaging satellite available, assign imaging
            img_sat = next((s for s in visible_sats if "IMAGING" in s.capabilities), None)
            pnt_sat = next((s for s in visible_sats if "PNT" in s.capabilities and s != img_sat), None)

            if hazard_risk_level > 0.5 and img_sat:
                img_sat.set_task("IMAGING", "Assigned for forward hazard reconnaissance")
                assigned_tasks[img_sat.sat_id] = "IMAGING"
                self.selected_satellite_id = img_sat.sat_id
                self.selected_task = "IMAGING"
                self.last_reason_summary = f"{img_sat.sat_id} selected for orbital hazard imaging (Risk: {hazard_risk_level*100:.0f}%)"
            elif pnt_sat:
                pnt_sat.set_task("PNT", "Assigned for precision localization lock")
                assigned_tasks[pnt_sat.sat_id] = "PNT"
                self.selected_satellite_id = pnt_sat.sat_id
                self.selected_task = "PNT"
                self.last_reason_summary = f"{pnt_sat.sat_id} selected for PNT precision navigation"
            elif visible_sats:
                primary = visible_sats[0]
                task = "PNT" if "PNT" in primary.capabilities else "COMMUNICATION"
                primary.set_task(task, "Selected as highest scoring multi-role asset")
                assigned_tasks[primary.sat_id] = task
                self.selected_satellite_id = primary.sat_id
                self.selected_task = task
                self.last_reason_summary = f"{primary.sat_id} selected for {task}"

            # Secondary assignments for remaining visible satellites
            for s in visible_sats:
                if s.sat_id in assigned_tasks:
                    continue
                if "PNT" in s.capabilities and "PNT" not in assigned_tasks.values():
                    s.set_task("PNT", "Secondary PNT constellation support")
                    assigned_tasks[s.sat_id] = "PNT"
                elif "COMMUNICATION" in s.capabilities and "COMMUNICATION" not in assigned_tasks.values():
                    s.set_task("COMMUNICATION", "Telemetry downlink relay")
                    assigned_tasks[s.sat_id] = "COMMUNICATION"
                else:
                    s.set_task("IDLE", "Standby in orbital pass")
                    assigned_tasks[s.sat_id] = "IDLE"

        return {
            "selected_satellite": self.selected_satellite_id,
            "selected_task": self.selected_task,
            "reason_summary": self.last_reason_summary,
            "scores": scores,
            "sub_scores": sub_scores,
            "explanations": explanations,
            "assigned_tasks": assigned_tasks,
        }
