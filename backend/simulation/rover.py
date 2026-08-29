"""
NAVIS - Rover Kinematic & Physical Simulation Model
Simulates rover motion, wheel dynamics, terrain interaction, battery consumption, and waypoint tracking.
"""

from typing import List, Tuple, Dict, Any, Optional
import numpy as np
import math


class RoverMissionStatus:
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    HAZARD_AVOIDANCE = "HAZARD_AVOIDANCE"
    OUTAGE_FALLBACK = "OUTAGE_FALLBACK"
    TARGET_REACHED = "TARGET_REACHED"


class MarsRover:
    """
    Kinematic model of a Martian rover with terrain slip, battery drain,
    and pure-pursuit waypoint tracking.
    """

    def __init__(
        self,
        start_x: float = 50.0,
        start_y: float = 50.0,
        target_x: float = 540.0,
        target_y: float = 530.0,
        max_speed: float = 3.5,  # m/s
        max_accel: float = 1.2,  # m/s^2
        max_steer_rate: float = 1.5,  # rad/s (~85 deg/s)
    ):
        self.start_pos = (start_x, start_y)
        self.target_pos = (target_x, target_y)
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.max_steer_rate = max_steer_rate

        # True Physical State
        self.x = start_x
        self.y = start_y
        self.vx = 0.0
        self.vy = 0.0
        self.speed = 0.0
        self.accel = 0.0
        self.heading = math.atan2(target_y - start_y, target_x - start_x)  # radians
        self.angular_velocity = 0.0

        # Subsystems
        self.battery_pct = 100.0  # 100%
        self.wheel_slip = 0.02  # nominal 2% slip
        self.wheel_speed_left = 0.0
        self.wheel_speed_right = 0.0
        self.total_distance_traveled = 0.0
        self.mission_status = RoverMissionStatus.IDLE

        # Path tracking
        self.active_path: List[Tuple[float, float]] = []
        self.current_waypoint_idx = 0
        self.lookahead_dist = 6.0  # meters
        self.target_tolerance = 4.0  # meters

        # Historical breadcrumb trajectory
        self.history: List[Dict[str, float]] = []
        self.max_history_len = 1000

    def reset(self, start_pos: Optional[Tuple[float, float]] = None) -> None:
        """Reset rover state to starting conditions."""
        if start_pos is not None:
            self.start_pos = start_pos
        self.x, self.y = self.start_pos
        self.vx = 0.0
        self.vy = 0.0
        self.speed = 0.0
        self.accel = 0.0
        self.heading = math.atan2(
            self.target_pos[1] - self.start_pos[1],
            self.target_pos[0] - self.start_pos[0],
        )
        self.angular_velocity = 0.0
        self.battery_pct = 100.0
        self.wheel_slip = 0.02
        self.wheel_speed_left = 0.0
        self.wheel_speed_right = 0.0
        self.total_distance_traveled = 0.0
        self.mission_status = RoverMissionStatus.IDLE
        self.current_waypoint_idx = 0
        self.history.clear()
        self.record_history(0.0)

    def set_path(self, path: List[Tuple[float, float]]) -> None:
        """Set the calculated route for the rover to follow."""
        self.active_path = list(path)
        self.current_waypoint_idx = 0
        if len(self.active_path) > 0 and self.mission_status == RoverMissionStatus.IDLE:
            self.mission_status = RoverMissionStatus.NAVIGATING

    def update(
        self,
        dt: float,
        sim_time: float,
        terrain_cost: float = 1.0,
        terrain_slope: float = 0.0,
        is_outage: bool = False,
    ) -> None:
        """
        Step kinematics using pure pursuit along active_path.
        """
        # Update mission status state machine
        dist_to_target = math.hypot(self.x - self.target_pos[0], self.y - self.target_pos[1])
        if dist_to_target <= self.target_tolerance:
            self.mission_status = RoverMissionStatus.TARGET_REACHED
            self.speed = max(0.0, self.speed - self.max_accel * dt)
            self.vx = self.speed * math.cos(self.heading)
            self.vy = self.speed * math.sin(self.heading)
            self.record_history(sim_time)
            return

        if is_outage and self.mission_status != RoverMissionStatus.TARGET_REACHED:
            self.mission_status = RoverMissionStatus.OUTAGE_FALLBACK
        elif self.mission_status == RoverMissionStatus.OUTAGE_FALLBACK and not is_outage:
            self.mission_status = RoverMissionStatus.NAVIGATING

        if not self.active_path or self.current_waypoint_idx >= len(self.active_path):
            # No path or path finished
            target_pt = self.target_pos
        else:
            # Advance waypoints within lookahead
            while (
                self.current_waypoint_idx < len(self.active_path) - 1
                and math.hypot(
                    self.x - self.active_path[self.current_waypoint_idx][0],
                    self.y - self.active_path[self.current_waypoint_idx][1],
                )
                < self.lookahead_dist
            ):
                self.current_waypoint_idx += 1

            target_pt = self.active_path[self.current_waypoint_idx]

        # Steering control towards target waypoint
        target_heading = math.atan2(target_pt[1] - self.y, target_pt[0] - self.x)
        angle_diff = (target_heading - self.heading + math.pi) % (2 * math.pi) - math.pi

        # Limit angular rate
        self.angular_velocity = np.clip(
            angle_diff * 2.5, -self.max_steer_rate, self.max_steer_rate
        )
        self.heading += self.angular_velocity * dt
        self.heading = (self.heading + math.pi) % (2 * math.pi) - math.pi

        # Target Speed adjusted by terrain roughness and curvature
        curvature_factor = max(0.3, 1.0 - abs(angle_diff) / math.pi)
        cost_penalty = 1.0 / (1.0 + 0.2 * (terrain_cost - 1.0))
        target_speed = self.max_speed * curvature_factor * cost_penalty

        # Accelerate / Decelerate
        speed_error = target_speed - self.speed
        accel_cmd = np.clip(speed_error * 2.0, -self.max_accel, self.max_accel)
        self.accel = accel_cmd
        self.speed = np.clip(self.speed + self.accel * dt, 0.0, self.max_speed)

        # Wheel slip calculation
        self.wheel_slip = min(0.45, 0.02 + 0.04 * (terrain_cost - 1.0) + 0.15 * terrain_slope)
        effective_speed = self.speed * (1.0 - self.wheel_slip)

        # Differential wheel speeds (track width ~ 1.2m)
        track_width = 1.2
        self.wheel_speed_left = self.speed - (self.angular_velocity * track_width / 2.0)
        self.wheel_speed_right = self.speed + (self.angular_velocity * track_width / 2.0)

        # Update True Coordinates
        dx = effective_speed * math.cos(self.heading) * dt
        dy = effective_speed * math.sin(self.heading) * dt
        self.x += dx
        self.y += dy
        self.vx = effective_speed * math.cos(self.heading)
        self.vy = effective_speed * math.sin(self.heading)
        dist_step = math.hypot(dx, dy)
        self.total_distance_traveled += dist_step

        # Battery consumption model: base hotel load (10W) + mechanical work (150W * speed * cost)
        power_draw_kw = 0.015 + 0.12 * (self.speed / self.max_speed) * terrain_cost
        battery_drain = (power_draw_kw * (dt / 3600.0) / 0.5) * 100.0  # assuming 0.5 kWh pack
        self.battery_pct = max(0.0, self.battery_pct - battery_drain)

        # History recording
        self.record_history(sim_time)

    def record_history(self, sim_time: float) -> None:
        """Store historical sample for plotting and trajectory modeling."""
        if (
            not self.history
            or math.hypot(self.x - self.history[-1]["x"], self.y - self.history[-1]["y"]) > 1.0
        ):
            self.history.append(
                {
                    "t": float(sim_time),
                    "x": float(self.x),
                    "y": float(self.y),
                    "vx": float(self.vx),
                    "vy": float(self.vy),
                    "speed": float(self.speed),
                    "heading": float(self.heading),
                    "battery": float(self.battery_pct),
                }
            )
            if len(self.history) > self.max_history_len:
                self.history.pop(0)

    def to_dict(self) -> Dict[str, Any]:
        """Export rover telemetry for API and WebSocket clients."""
        return {
            "x": float(self.x),
            "y": float(self.y),
            "vx": float(self.vx),
            "vy": float(self.vy),
            "speed": float(self.speed),
            "accel": float(self.accel),
            "heading_rad": float(self.heading),
            "heading_deg": float((math.degrees(self.heading) + 360) % 360),
            "angular_velocity": float(self.angular_velocity),
            "battery_pct": float(round(self.battery_pct, 1)),
            "wheel_slip": float(round(self.wheel_slip, 3)),
            "wheel_speed_left": float(round(self.wheel_speed_left, 2)),
            "wheel_speed_right": float(round(self.wheel_speed_right, 2)),
            "total_distance": float(round(self.total_distance_traveled, 1)),
            "mission_status": self.mission_status,
            "target_pos": list(self.target_pos),
            "dist_to_target": float(
                round(math.hypot(self.x - self.target_pos[0], self.y - self.target_pos[1]), 1)
            ),
        }
