"""
NAVIS - Martian Satellite Constellation Simulation
Simulates orbital ground tracks, field of view (FOV) coverage cones, visibility,
power levels, and task execution for SAT-01, SAT-02, SAT-03, and SAT-04.
"""

from typing import List, Dict, Any, Tuple, Optional
import math
import numpy as np


class SatelliteTask:
    PNT = "PNT"
    IMAGING = "IMAGING"
    COMMUNICATION = "COMMUNICATION"
    IDLE = "IDLE"


class Satellite:
    """Individual simulated orbiter."""

    def __init__(
        self,
        sat_id: str,
        name: str,
        altitude_km: float,
        orbit_period_s: float,
        inclination_deg: float,
        fov_radius_m: float,
        initial_phase_rad: float,
        center_x: float = 300.0,
        center_y: float = 300.0,
        orbit_radius_m: float = 240.0,
        capabilities: Optional[List[str]] = None,
    ):
        self.sat_id = sat_id
        self.name = name
        self.altitude_km = altitude_km
        self.orbit_period_s = orbit_period_s
        self.inclination_deg = inclination_deg
        self.fov_radius_m = fov_radius_m
        self.phase = initial_phase_rad
        self.center_x = center_x
        self.center_y = center_y
        self.orbit_radius_m = orbit_radius_m
        self.capabilities = capabilities or ["PNT", "IMAGING", "COMMUNICATION"]

        # Current Ground-track Coordinates
        self.gx = center_x + orbit_radius_m * math.cos(self.phase)
        self.gy = center_y + orbit_radius_m * math.sin(self.phase) * math.cos(
            math.radians(inclination_deg)
        )

        # Dynamic Status
        self.is_visible_to_rover = False
        self.elevation_angle_deg = 0.0
        self.distance_to_rover_m = 0.0
        self.pnt_available = "PNT" in self.capabilities
        self.comm_available = "COMMUNICATION" in self.capabilities
        self.imaging_available = "IMAGING" in self.capabilities
        self.battery_pct = 95.0
        self.current_task = SatelliteTask.IDLE
        self.is_outage_forced = False
        self.score = 0.0
        self.decision_reason = ""

    def update_orbit(
        self,
        dt: float,
        sim_time: float,
        rover_pos: Tuple[float, float],
    ) -> None:
        """Step orbital ground track and evaluate visibility/elevation relative to rover."""
        # Angular speed
        omega = 2.0 * math.pi / self.orbit_period_s
        self.phase = (self.phase + omega * dt) % (2.0 * math.pi)

        # Ground projection path (elliptical track on Mars surface plane)
        incl_factor = math.cos(math.radians(self.inclination_deg))
        self.gx = self.center_x + self.orbit_radius_m * math.cos(self.phase)
        self.gy = self.center_y + self.orbit_radius_m * math.sin(self.phase) * incl_factor

        # Distance to rover ground position
        rover_x, rover_y = rover_pos
        ground_dist = math.hypot(self.gx - rover_x, self.gy - rover_y)
        self.distance_to_rover_m = ground_dist

        # Compute 3D slant range and elevation angle (degrees above rover horizon)
        alt_m = self.altitude_km * 1000.0
        slant_range = math.hypot(ground_dist, alt_m)
        self.elevation_angle_deg = math.degrees(math.atan2(alt_m, max(1.0, ground_dist)))

        # Visibility test: Rover is within Satellite FOV cone footprint and elevation >= 15 deg
        in_cone = ground_dist <= self.fov_radius_m
        has_los = self.elevation_angle_deg >= 18.0

        if self.is_outage_forced:
            self.is_visible_to_rover = False
            self.pnt_available = False
            self.comm_available = False
            self.imaging_available = False
        else:
            self.is_visible_to_rover = in_cone and has_los
            self.pnt_available = self.is_visible_to_rover and ("PNT" in self.capabilities)
            self.comm_available = self.is_visible_to_rover and (
                "COMMUNICATION" in self.capabilities
            )
            self.imaging_available = self.is_visible_to_rover and ("IMAGING" in self.capabilities)

        # Solar power cycling: sunlit when cos(phase) > -0.3, otherwise battery discharge
        is_sunlit = math.cos(self.phase) > -0.3
        if is_sunlit:
            self.battery_pct = min(100.0, self.battery_pct + 0.15 * dt)
        else:
            # Active task uses more power
            task_drain = 0.25 if self.current_task != SatelliteTask.IDLE else 0.08
            self.battery_pct = max(15.0, self.battery_pct - task_drain * dt)

    def set_task(self, task: str, reason: str = "") -> None:
        """Assign mission task."""
        self.current_task = task
        self.decision_reason = reason

    def force_outage(self, outage: bool = True) -> None:
        """Force artificial outage for test scenarios."""
        self.is_outage_forced = outage
        if outage:
            self.current_task = SatelliteTask.IDLE

    def to_dict(self) -> Dict[str, Any]:
        """Serialize satellite state for clients."""
        return {
            "sat_id": self.sat_id,
            "name": self.name,
            "altitude_km": self.altitude_km,
            "fov_radius_m": self.fov_radius_m,
            "gx": float(round(self.gx, 1)),
            "gy": float(round(self.gy, 1)),
            "elevation_deg": float(round(self.elevation_angle_deg, 1)),
            "distance_to_rover_m": float(round(self.distance_to_rover_m, 1)),
            "is_visible": self.is_visible_to_rover,
            "pnt_available": self.pnt_available,
            "comm_available": self.comm_available,
            "imaging_available": self.imaging_available,
            "battery_pct": float(round(self.battery_pct, 1)),
            "current_task": self.current_task,
            "is_outage": self.is_outage_forced,
            "score": float(round(self.score, 3)),
            "decision_reason": self.decision_reason,
            "capabilities": self.capabilities,
        }


class SatelliteConstellation:
    """Manages the 4-satellite Mars orbit network."""

    def __init__(self, terrain_width: float = 600.0, terrain_height: float = 600.0):
        cx = terrain_width / 2.0
        cy = terrain_height / 2.0

        # Define 4 distinct satellites with staggered orbits and specialized capabilities
        self.satellites: Dict[str, Satellite] = {
            "SAT-01": Satellite(
                sat_id="SAT-01",
                name="Ares-PNT1",
                altitude_km=380.0,
                orbit_period_s=45.0,  # Fast orbital pass (~45s per cycle)
                inclination_deg=35.0,
                fov_radius_m=220.0,
                initial_phase_rad=0.0,
                center_x=cx,
                center_y=cy,
                orbit_radius_m=230.0,
                capabilities=["PNT", "COMMUNICATION"],
            ),
            "SAT-02": Satellite(
                sat_id="SAT-02",
                name="Phobos-Relay",
                altitude_km=620.0,
                orbit_period_s=75.0,
                inclination_deg=65.0,
                fov_radius_m=260.0,
                initial_phase_rad=math.pi * 0.55,
                center_x=cx,
                center_y=cy,
                orbit_radius_m=260.0,
                capabilities=["COMMUNICATION", "PNT"],
            ),
            "SAT-03": Satellite(
                sat_id="SAT-03",
                name="Olympus-Eye",
                altitude_km=420.0,
                orbit_period_s=55.0,
                inclination_deg=20.0,
                fov_radius_m=240.0,
                initial_phase_rad=math.pi * 1.15,
                center_x=cx,
                center_y=cy,
                orbit_radius_m=210.0,
                capabilities=["IMAGING", "PNT", "COMMUNICATION"],
            ),
            "SAT-04": Satellite(
                sat_id="SAT-04",
                name="Hermes-PNT2",
                altitude_km=510.0,
                orbit_period_s=62.0,
                inclination_deg=48.0,
                fov_radius_m=235.0,
                initial_phase_rad=math.pi * 1.70,
                center_x=cx,
                center_y=cy,
                orbit_radius_m=240.0,
                capabilities=["PNT", "COMMUNICATION", "IMAGING"],
            ),
        }
        self.global_outage = False

    def update(self, dt: float, sim_time: float, rover_pos: Tuple[float, float]) -> None:
        """Step all satellites."""
        for sat in self.satellites.values():
            sat.update_orbit(dt, sim_time, rover_pos)

    def set_global_outage(self, outage: bool) -> None:
        """Toggle outage on primary PNT satellites."""
        self.global_outage = outage
        for sat in self.satellites.values():
            if outage:
                sat.force_outage(True)
            else:
                sat.force_outage(False)

    def get_pnt_satellite(self) -> Optional[Satellite]:
        """Return the satellite currently providing active PNT service."""
        if self.global_outage:
            return None
        for sat in self.satellites.values():
            if (
                sat.is_visible_to_rover
                and sat.pnt_available
                and sat.current_task == SatelliteTask.PNT
            ):
                return sat
        # If no satellite assigned PNT but visible PNT available
        for sat in self.satellites.values():
            if sat.is_visible_to_rover and sat.pnt_available and not sat.is_outage_forced:
                return sat
        return None

    def get_imaging_satellite(self) -> Optional[Satellite]:
        """Return active imaging satellite."""
        for sat in self.satellites.values():
            if (
                sat.is_visible_to_rover
                and sat.imaging_available
                and sat.current_task == SatelliteTask.IMAGING
            ):
                return sat
        return None

    def to_list(self) -> List[Dict[str, Any]]:
        """Return list of satellite states."""
        return [sat.to_dict() for sat in self.satellites.values()]
