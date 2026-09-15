"""
NAVIS - Central Simulation & Navigation Engine
Integrates all physics, sensor models, EKF fusion, AI constellation scheduling,
trajectory prediction, hazard detection, A* planning, and WebSocket broadcast loops.
"""

from typing import Dict, List, Any, Optional
import math
from datetime import datetime

from backend.simulation.terrain import MarsTerrain
from backend.simulation.rover import MarsRover
from backend.simulation.satellites import SatelliteConstellation
from backend.simulation.metrics import PerformanceMetrics
from backend.simulation.modes import NavigationMode, ModeConfig
from backend.simulation.scenario import DemoMissionRunner
from backend.navigation.sensors import SensorSuite
from backend.navigation.ekf import NAVIS_EKF
from backend.navigation.planner import AStarPlanner
from backend.ai.scheduler import AISatelliteScheduler
from backend.ai.trajectory import TrajectoryPredictor
from backend.ai.hazard_detector import VisualHazardDetector


class SimulationEngine:
    """
    Core state machine and high-frequency real-time physics and navigation loop.
    """

    def __init__(self, seed: int = 42) -> None:
        self.seed = seed
        self.is_running = False
        self.sim_time = 0.0
        self.speed_multiplier = 1.0
        self.target_fps = 20.0
        self.base_dt = 1.0 / self.target_fps  # 0.05s (20Hz)

        # Active Navigation Mode (MODE_A, MODE_B, MODE_C)
        self.active_mode = NavigationMode.MODE_C

        # Subsystems
        self.terrain = MarsTerrain(seed=seed)
        self.rover = MarsRover(
            start_x=self.terrain.start_pos[0],
            start_y=self.terrain.start_pos[1],
            target_x=self.terrain.target_pos[0],
            target_y=self.terrain.target_pos[1],
        )
        self.constellation = SatelliteConstellation(self.terrain.width_m, self.terrain.height_m)
        self.sensors = SensorSuite(seed=seed)
        self.ekf = NAVIS_EKF(start_x=self.terrain.start_pos[0], start_y=self.terrain.start_pos[1])
        self.planner = AStarPlanner(self.terrain)
        self.scheduler = AISatelliteScheduler()
        self.traj_predictor = TrajectoryPredictor()
        self.hazard_detector = VisualHazardDetector(self.terrain)
        self.metrics = PerformanceMetrics()
        self.demo_runner = DemoMissionRunner(self)

        # AI Decision & Event Log
        self.logs: List[Dict[str, Any]] = []
        self.max_logs = 200

        # Latest computed predictions and states
        self.latest_prediction: Dict[str, Any] = {
            "predicted_trajectory": [],
            "horizon_sec": 30.0,
            "confidence_pct": 94.0,
        }
        self.latest_scheduler_decision: Dict[str, Any] = {}
        self.latest_sensors: Dict[str, Any] = {}
        self.latest_ekf: Dict[str, Any] = {}
        self.latest_hazard_alert: Optional[Dict[str, Any]] = None
        self.last_ai_tick = -1.0

        # Plan initial route
        self._initialize_mission()

    def _initialize_mission(self) -> None:
        """Calculate primary route and initialize telemetry."""
        initial_route = self.planner.plan_path(self.rover.start_pos, self.rover.target_pos)
        self.rover.set_path(initial_route)
        self.planner.current_route = list(initial_route)
        self.planner.old_route = []
        self.metrics.reset(initial_route)
        self.add_log("SYSTEM", "NAVIS Mission Navigation Subsystems Initialized.", "info")

    def add_log(self, category: str, message: str, level: str = "info") -> None:
        """Append an explainable event to the mission log."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        sim_timestamp = f"MET +{int(self.sim_time//60):02d}:{int(self.sim_time%60):02d}"
        entry = {
            "id": len(self.logs) + 1,
            "time": timestamp,
            "met": sim_timestamp,
            "category": category,
            "message": message,
            "level": level,  # 'info', 'warning', 'danger', 'success'
        }
        self.logs.append(entry)
        if len(self.logs) > self.max_logs:
            self.logs.pop(0)

    def start(self) -> None:
        self.is_running = True
        self.add_log("CONTROL", "Mission Simulation Running.", "success")

    def pause(self) -> None:
        self.is_running = False
        self.add_log("CONTROL", "Mission Simulation Paused.", "warning")

    def reset(self, seed: Optional[int] = None) -> None:
        """Reset whole simulation to starting state."""
        if seed is not None:
            self.seed = seed
        self.is_running = False
        self.sim_time = 0.0
        self.terrain.clear_injected_hazards()
        self.rover.reset(self.terrain.start_pos)
        self.constellation = SatelliteConstellation(self.terrain.width_m, self.terrain.height_m)
        self.sensors.reset(self.terrain.start_pos, self.rover.heading)
        self.ekf.reset(self.terrain.start_pos[0], self.terrain.start_pos[1], self.rover.heading)
        self.hazard_detector = VisualHazardDetector(self.terrain)
        self.demo_runner.stop_demo()
        self._initialize_mission()
        self.add_log("CONTROL", "Mission Simulation Reset to Origin.", "info")

    def set_speed(self, multiplier: float) -> None:
        self.speed_multiplier = max(0.25, min(10.0, float(multiplier)))
        self.add_log("CONTROL", f"Simulation speed multiplier set to {self.speed_multiplier}x", "info")

    def set_mode(self, mode: str) -> None:
        if mode in [NavigationMode.MODE_A, NavigationMode.MODE_B, NavigationMode.MODE_C]:
            self.active_mode = mode
            desc = ModeConfig.DESCRIPTIONS[mode]
            self.add_log("MODE", f"Navigation Mode switched to: {desc['name']}", "warning")

    def set_satellite_outage(self, outage: bool) -> None:
        self.constellation.set_global_outage(outage)
        if outage:
            self.add_log("OUTAGE", "Constellation Satellite Outage Injected: PNT lost.", "danger")
        else:
            self.add_log("RESTORE", "Constellation Satellite PNT service restored.", "success")

    def inject_hazard(
        self,
        x: float,
        y: float,
        radius: float = 30.0,
        hazard_type: str = "ROCK_FIELD",
    ) -> Dict[str, Any]:
        """Manually or dynamically inject a hazard to prompt route replanning."""
        hz = self.terrain.inject_hazard(x, y, radius, hazard_type)
        self.add_log("HAZARD", f"Hazard Injected at ({x:.0f}m, {y:.0f}m): {hazard_type}", "danger")

        # If in Mode C, evaluate immediate replan if along path
        if self.active_mode == NavigationMode.MODE_C:
            replan = self.planner.trigger_replan((self.rover.x, self.rover.y), self.rover.target_pos, "USER_INJECTED_HAZARD")
            self.rover.set_path(replan["new_route"])
            self.add_log("PLANNER", "A* replanned route to avoid newly injected hazard ✓", "success")

        return hz

    def step(self, custom_dt: Optional[float] = None) -> None:
        """
        Execute one physical & algorithmic tick.
        """
        dt = (custom_dt if custom_dt is not None else self.base_dt) * self.speed_multiplier
        self.sim_time += dt

        # Update demo script if active
        if self.demo_runner.is_active:
            self.demo_runner.update(dt)

        rover_pos = (self.rover.x, self.rover.y)

        # 1. Update Satellites
        if self.active_mode == NavigationMode.MODE_A:
            # Rover Only: No satellites
            self.constellation.set_global_outage(True)
            self.constellation.update(dt, self.sim_time, rover_pos)
            pnt_available = False
            imaging_available = False
        elif self.active_mode == NavigationMode.MODE_B:
            # Fixed SAT-01 only
            self.constellation.set_global_outage(False)
            self.constellation.update(dt, self.sim_time, rover_pos)
            sat01 = self.constellation.satellites["SAT-01"]
            sat01.set_task("PNT" if sat01.is_visible_to_rover else "IDLE")
            # Other satellites kept idle
            for sid, s in self.constellation.satellites.items():
                if sid != "SAT-01":
                    s.set_task("IDLE")
            pnt_available = sat01.is_visible_to_rover and not sat01.is_outage_forced
            imaging_available = False
        else:
            # Mode C: Full Constellation + AI Scheduler
            self.constellation.update(dt, self.sim_time, rover_pos)

            # Run AI Scheduler every ~0.5s of sim time
            if (self.sim_time - self.last_ai_tick) >= 0.4:
                self.last_ai_tick = self.sim_time

                hazard_risk = (
                    self.latest_hazard_alert["hazard_risk_level"]
                    if self.latest_hazard_alert
                    else 0.0
                )
                self.latest_scheduler_decision = self.scheduler.evaluate_and_schedule(
                    self.constellation.satellites,
                    self.rover.to_dict(),
                    self.latest_ekf,
                    hazard_risk_level=hazard_risk,
                    sim_time=self.sim_time,
                )

            pnt_sat = self.constellation.get_pnt_satellite()
            pnt_available = (pnt_sat is not None) and (not self.constellation.global_outage)
            img_sat = self.constellation.get_imaging_satellite()
            imaging_available = (img_sat is not None) and (not self.constellation.global_outage)

        # Collect active visible PNT satellites for dynamic geometry HDOP
        if pnt_available and not self.constellation.global_outage:
            active_pnt_sats = [
                s for s in self.constellation.satellites.values()
                if s.is_visible_to_rover and s.pnt_available and not s.is_outage_forced
            ]
        else:
            active_pnt_sats = []

        # 2. Step Rover Kinematics
        terrain_cost = self.terrain.get_cost(self.rover.x, self.rover.y)
        terrain_slope = self.terrain.get_slope(self.rover.x, self.rover.y)

        self.rover.update(
            dt=dt,
            sim_time=self.sim_time,
            terrain_cost=terrain_cost,
            terrain_slope=terrain_slope,
            is_outage=(not pnt_available),
        )

        # 3. Sample Sensors from newly propagated rover state
        self.latest_sensors = self.sensors.sample(
            dt=dt,
            true_x=self.rover.x,
            true_y=self.rover.y,
            true_vx=self.rover.vx,
            true_vy=self.rover.vy,
            true_heading=self.rover.heading,
            true_speed=self.rover.ground_speed,
            true_accel=self.rover.accel,
            true_omega=self.rover.angular_velocity,
            wheel_slip=self.rover.wheel_slip,
            pnt_available=pnt_available,
            wheel_speed=self.rover.wheel_speed,
            sim_time=self.sim_time,
            active_satellites=active_pnt_sats,
        )

        # 4. EKF Sensor Fusion Step
        self.latest_ekf = self.ekf.step(dt, self.latest_sensors)

        # 5. AI Trajectory Prediction & Hazard Reconnaissance (Mode C)
        if self.active_mode == NavigationMode.MODE_C:
            self.latest_prediction = self.traj_predictor.predict(
                rover_history=self.rover.history,
                current_state=self.rover.to_dict(),
                active_planned_path=self.rover.active_path,
                current_uncertainty=self.latest_ekf.get("uncertainty_m", 1.0),
                terrain_cost=terrain_cost,
            )

            # Hazard Scanning
            hz_scan = self.hazard_detector.scan_environment(
                rover_pos=(self.rover.x, self.rover.y),
                rover_heading=self.rover.heading,
                predicted_trajectory=self.latest_prediction["predicted_trajectory"],
                planned_path=self.rover.active_path,
                imaging_sat_active=imaging_available,
                sim_time=self.sim_time,
            )
            self.latest_hazard_alert = hz_scan

            if hz_scan["needs_replan"]:
                alert = hz_scan["active_alert"]
                self.add_log(
                    "HAZARD",
                    f"⚠️ {alert['type']} identified by {alert['source']} at {alert['distance_m']}m ahead (Conf: {alert['confidence_pct']}%)",
                    "danger",
                )
                replan_res = self.planner.trigger_replan(
                    (self.rover.x, self.rover.y),
                    self.rover.target_pos,
                    reason=f"Detected {alert['type']}",
                )
                self.rover.set_path(replan_res["new_route"])
                self.add_log("PLANNER", "Dynamic route replanning complete. Executing safe bypass ✓", "success")

        # 6. Update Performance Metrics
        est_pos = (self.latest_ekf.get("est_x", self.rover.x), self.latest_ekf.get("est_y", self.rover.y))
        self.metrics.update(
            sim_time=self.sim_time,
            true_pos=(self.rover.x, self.rover.y),
            est_pos=est_pos,
            uncertainty_m=self.latest_ekf.get("uncertainty_m", 1.0),
            rover_speed=self.rover.speed,
            satellites=self.constellation.to_list(),
        )

    def get_full_state(self) -> Dict[str, Any]:
        """Compile comprehensive state payload for API and WebSocket streaming."""
        est_pos = (self.latest_ekf.get("est_x", self.rover.x), self.latest_ekf.get("est_y", self.rover.y))
        err = math.hypot(self.rover.x - est_pos[0], self.rover.y - est_pos[1])

        return {
            "sim_time": round(self.sim_time, 2),
            "is_running": self.is_running,
            "speed_multiplier": self.speed_multiplier,
            "active_mode": self.active_mode,
            "mode_meta": ModeConfig.DESCRIPTIONS[self.active_mode],
            "rover": self.rover.to_dict(),
            "satellites": self.constellation.to_list(),
            "ekf": self.latest_ekf,
            "sensors": self.latest_sensors,
            "metrics": {
                **self.metrics.update(
                    self.sim_time,
                    (self.rover.x, self.rover.y),
                    est_pos,
                    self.latest_ekf.get("uncertainty_m", 1.0),
                    self.rover.speed,
                    self.constellation.to_list(),
                ),
                "position_error_instant_m": round(err, 2),
            },
            "trajectory_prediction": self.latest_prediction,
            "ai_scheduler": self.latest_scheduler_decision,
            "hazard_alert": self.latest_hazard_alert,
            "routes": {
                "active_path": self.rover.active_path,
                "current_planned": self.planner.current_route,
                "old_avoided_path": self.planner.old_route,
                "breadcrumb_history": [
                    {"x": round(p["x"], 1), "y": round(p["y"], 1)}
                    for p in self.rover.history
                ],
            },
            "demo_mission": self.demo_runner.get_status(),
            "logs": self.logs[-40:],  # last 40 entries
            "terrain_summary": {
                "width_m": self.terrain.width_m,
                "height_m": self.terrain.height_m,
                "start_pos": list(self.terrain.start_pos),
                "target_pos": list(self.terrain.target_pos),
                "injected_hazards": self.terrain.injected_hazards,
                "craters": self.terrain.craters,
                "rock_fields": self.terrain.rock_fields,
            },
        }
