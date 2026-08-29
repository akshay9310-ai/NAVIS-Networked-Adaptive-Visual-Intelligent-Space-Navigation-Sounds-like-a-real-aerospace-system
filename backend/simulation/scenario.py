"""
NAVIS - Automated Demo Mission Controller
Orchestrates the 15-stage demonstration scenario showcasing full autonomous capabilities:
From start -> AI prediction -> Satellite scheduling -> Hazard detection -> Replanning ->
Outage injection -> Fallback navigation -> Satellite restoration -> Target arrival.
"""

from typing import Dict, List, Any, Optional, Callable


class DemoStage:
    STAGE_01_START = "01_START"
    STAGE_02_TRAJ_PRED = "02_TRAJ_PRED"
    STAGE_03_SATS_VISIBLE = "03_SATS_VISIBLE"
    STAGE_04_PNT_ASSIGNED = "04_PNT_ASSIGNED"
    STAGE_05_IMAGING_ASSIGNED = "05_IMAGING_ASSIGNED"
    STAGE_06_HAZARD_DETECTED = "06_HAZARD_DETECTED"
    STAGE_07_ROUTE_REPLANNED = "07_ROUTE_REPLANNED"
    STAGE_08_COURSE_CORRECTION = "08_COURSE_CORRECTION"
    STAGE_09_OUTAGE_INJECTED = "09_OUTAGE_INJECTED"
    STAGE_10_FALLBACK_ACTIVE = "10_FALLBACK_ACTIVE"
    STAGE_11_AUTONOMOUS_TRANSIT = "11_AUTONOMOUS_TRANSIT"
    STAGE_12_SATELLITE_RESTORED = "12_SATELLITE_RESTORED"
    STAGE_13_POSITION_CORRECTED = "13_POSITION_CORRECTED"
    STAGE_14_TARGET_REACHED = "14_TARGET_REACHED"
    STAGE_15_MISSION_COMPLETE = "15_MISSION_COMPLETE"


class DemoMissionRunner:
    """
    Manages deterministic scripted events during a DEMO MISSION run.
    """

    STAGES_META = [
        {"id": "01", "name": "Mission Initialization", "desc": "Rover systems online. Target coordinates locked."},
        {"id": "02", "name": "AI Trajectory Prediction", "desc": "ML kinematic model projects 30s forward path."},
        {"id": "03", "name": "Constellation Orbital Pass", "desc": "Orbiters SAT-01, SAT-03 enter Martian local horizon."},
        {"id": "04", "name": "PNT Precision Lock", "desc": "AI Scheduler assigns SAT-01 to primary PNT lock."},
        {"id": "05", "name": "Orbital Imaging Request", "desc": "Forward anomaly detected. SAT-03 assigned to Imaging."},
        {"id": "06", "name": "Hazard Confirmed", "desc": "Orbital visual scan confirms impassable Rock Field."},
        {"id": "07", "name": "Dynamic A* Replanning", "desc": "NAVIS calculates optimal safe detour route."},
        {"id": "08", "name": "Course Execution", "desc": "Rover transitions to new safe route. Old path archived."},
        {"id": "09", "name": "Satellite Outage Occurs", "desc": "Orbital occlusion: PNT signals lost across network."},
        {"id": "10", "name": "Autonomous Fallback Active", "desc": "EKF transitions to IMU + Visual Odometry + Wheel slip."},
        {"id": "11", "name": "Dead Reckoning Transit", "desc": "Rover navigates through GPS-denied crater sector."},
        {"id": "12", "name": "Constellation Reacquired", "desc": "SAT-04 Hermes re-establishes direct line-of-sight PNT."},
        {"id": "13", "name": "Position Reconciled", "desc": "Accumulated drift eliminated. Uncertainty contracts."},
        {"id": "14", "name": "Science Target Reached", "desc": "Rover arrives at sample target coordinates."},
        {"id": "15", "name": "Performance Evaluation", "desc": "Final mission telemetry and metrics compiled."},
    ]

    def __init__(self, engine):
        self.engine = engine
        self.is_active = False
        self.demo_time = 0.0
        self.current_stage_idx = 0
        self.events_fired = set()

    def start_demo(self) -> None:
        """Start the automated demo mission."""
        self.engine.reset()
        self.is_active = True
        self.demo_time = 0.0
        self.current_stage_idx = 0
        self.events_fired.clear()

        # Inject demonstration hazard along original path (around 270, 260)
        self.engine.terrain.inject_hazard(
            x=275.0,
            y=265.0,
            radius=32.0,
            hazard_type="ROCK_FIELD",
            hazard_id="DEMO-HZ-01",
        )
        self.engine.start()
        self.engine.add_log("DEMO", "NAVIS 15-Stage Demonstration Mission initiated.", "info")

    def stop_demo(self) -> None:
        self.is_active = False

    def update(self, dt: float) -> None:
        """Evaluate timeline conditions and trigger demonstration milestones."""
        if not self.is_active or not self.engine.is_running:
            return

        self.demo_time += dt
        t = self.demo_time
        rover = self.engine.rover

        # Timeline logic
        # 1. Start (t >= 0.5s)
        if "01" not in self.events_fired and t >= 0.5:
            self.events_fired.add("01")
            self.current_stage_idx = 0
            self.engine.add_log("ROVER", "Rover propulsion active. Waypoint tracking initialized.", "info")

        # 2. Trajectory prediction (t >= 2.0s)
        if "02" not in self.events_fired and t >= 2.0:
            self.events_fired.add("02")
            self.current_stage_idx = 1
            self.engine.add_log("AI_TRAJ", "30-second trajectory projection initialized (Confidence: 94%).", "info")

        # 3. Satellites visible (t >= 4.0s)
        if "03" not in self.events_fired and t >= 4.0:
            self.events_fired.add("03")
            self.current_stage_idx = 2
            self.engine.add_log("CONSTELLATION", "SAT-01 Ares & SAT-03 Olympus visible in local sky cone.", "info")

        # 4. PNT Assigned (t >= 6.0s)
        if "04" not in self.events_fired and t >= 6.0:
            self.events_fired.add("04")
            self.current_stage_idx = 3
            self.engine.add_log("SCHEDULER", "SAT-01 assigned to PNT (Score: 0.88). Navigation lock active.", "success")

        # 5. Imaging Assigned (t >= 10.0s or rover.x >= 140.0)
        if "05" not in self.events_fired and (t >= 10.0 or rover.x >= 140.0):
            self.events_fired.add("05")
            self.current_stage_idx = 4
            self.engine.add_log("SCHEDULER", "Forward anomaly alert. SAT-03 assigned to High-Res IMAGING.", "warning")

        # 6. Hazard Detected (t >= 13.0s or rover.x >= 170.0)
        if "06" not in self.events_fired and (t >= 13.0 or rover.x >= 170.0):
            self.events_fired.add("06")
            self.current_stage_idx = 5
            self.engine.add_log("HAZARD", "⚠️ HAZARD DETECTED: Rock Field (Confidence: 96%, Distance: 118m)", "danger")

        # 7. Route Replanned (t >= 15.0s or rover.x >= 190.0)
        if "07" not in self.events_fired and (t >= 15.0 or rover.x >= 190.0):
            self.events_fired.add("07")
            self.current_stage_idx = 6
            replan = self.engine.planner.trigger_replan((rover.x, rover.y), rover.target_pos, "DEMO_HAZARD_AVOIDANCE")
            rover.set_path(replan["new_route"])
            self.engine.add_log("PLANNER", "A* replanning complete. Safe detour route generated. Old path archived ❌", "success")

        # 8. Course execution (t >= 18.0s)
        if "08" not in self.events_fired and t >= 18.0:
            self.events_fired.add("08")
            self.current_stage_idx = 7
            self.engine.add_log("ROVER", "Steering along safe bypass route. Obstacle clearance verified.", "info")

        # 9. Outage Injected (t >= 24.0s or rover.x >= 280.0)
        if "09" not in self.events_fired and (t >= 24.0 or rover.x >= 280.0):
            self.events_fired.add("09")
            self.current_stage_idx = 8
            self.engine.set_satellite_outage(True)
            self.engine.add_log("OUTAGE", "⚠️ SATELLITE PNT OUTAGE SIMULATED: Signal lost across all orbiters.", "danger")

        # 10. Fallback Active (t >= 26.0s)
        if "10" not in self.events_fired and "09" in self.events_fired:
            self.events_fired.add("10")
            self.current_stage_idx = 9
            self.engine.add_log("EKF", "NAVIS AUTONOMOUS FALLBACK ACTIVATED (IMU + Visual Odometry + Wheel Slip)", "warning")

        # 11. Transit (t >= 32.0s)
        if "11" not in self.events_fired and t >= 32.0:
            self.events_fired.add("11")
            self.current_stage_idx = 10
            self.engine.add_log("NAV", "Dead reckoning transit stable. Covariance uncertainty expanding nominally.", "info")

        # 12. Satellite Restored (t >= 40.0s or rover.x >= 400.0)
        if "12" not in self.events_fired and (t >= 40.0 or rover.x >= 400.0):
            self.events_fired.add("12")
            self.current_stage_idx = 11
            self.engine.set_satellite_outage(False)
            self.engine.add_log("RESTORE", "✓ SATELLITE CONSTELLATION REACQUIRED: SAT-04 Hermes PNT lock online.", "success")

        # 13. Position Corrected (t >= 42.0s)
        if "13" not in self.events_fired and "12" in self.events_fired and t >= 42.0:
            self.events_fired.add("13")
            self.current_stage_idx = 12
            self.engine.add_log("EKF", "Position reconciliation complete. Covariance uncertainty contracted to 0.6m.", "success")

        # 14. Target Reached
        if rover.mission_status == "TARGET_REACHED" and "14" not in self.events_fired:
            self.events_fired.add("14")
            self.current_stage_idx = 13
            self.engine.add_log("MISSION", "🎯 SCIENCE TARGET REACHED SAFELY! Mission sample acquisition ready.", "success")

        # 15. Complete
        if "14" in self.events_fired and "15" not in self.events_fired:
            self.events_fired.add("15")
            self.current_stage_idx = 14
            self.is_active = False
            self.engine.add_log("EVAL", "Demonstration completed. Performance metrics archived.", "info")

    def get_status(self) -> Dict[str, Any]:
        return {
            "is_active": self.is_active,
            "demo_time_s": round(self.demo_time, 1),
            "current_stage_idx": self.current_stage_idx,
            "stages": self.STAGES_META,
        }
