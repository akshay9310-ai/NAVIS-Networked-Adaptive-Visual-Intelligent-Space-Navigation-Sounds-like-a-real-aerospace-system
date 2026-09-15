"""
NAVIS - Martian Procedural Terrain Generator and Cost Surface
Generates multi-feature terrain (Flat, Rocks, Slopes, Craters, Hazards, Science Target).
"""

from typing import Dict, List, Tuple, Any, Optional
import numpy as np
import math


class TerrainType:
    NORMAL = "normal"
    ROCK = "rock"
    SLOPE = "slope"
    HAZARD = "hazard"
    CRATER = "crater"
    TARGET = "target"


class MarsTerrain:
    """
    Simulates a 2D Martian grid with continuous coordinate mappings.
    Grid Size: width_m x height_m (default 600m x 600m), discrete resolution (e.g., 60x60 cells).
    """

    COST_NORMAL = 1.0
    COST_ROCK = 5.0
    COST_SLOPE = 8.0
    COST_HAZARD = 1000.0

    def __init__(
        self,
        width_m: float = 600.0,
        height_m: float = 600.0,
        grid_res: int = 60,
        seed: Optional[int] = 42,
    ):
        self.width_m = width_m
        self.height_m = height_m
        self.grid_res = grid_res
        self.cell_size = width_m / grid_res
        self.seed = seed

        # Positions
        self.start_pos = (50.0, 50.0)
        self.target_pos = (540.0, 530.0)

        # Dynamic hazards list: [{'x': float, 'y': float, 'radius': float, 'type': str, 'id': str}]
        self.injected_hazards: List[Dict[str, Any]] = []

        # Data grids
        self.elevation_grid = np.zeros((grid_res, grid_res), dtype=np.float32)
        self.slope_grid = np.zeros((grid_res, grid_res), dtype=np.float32)
        self.cost_grid = np.ones((grid_res, grid_res), dtype=np.float32)
        self.type_grid = np.full((grid_res, grid_res), TerrainType.NORMAL, dtype=object)

        # Crater records
        self.craters: List[Dict[str, float]] = []
        self.rock_fields: List[Dict[str, float]] = []

        self.generate(seed=seed)

    def generate(self, seed: Optional[int] = None) -> None:
        """Procedurally generate realistic Mars topography."""
        if seed is not None:
            self.seed = seed
            np.random.seed(seed)

        N = self.grid_res
        x = np.linspace(0, self.width_m, N)
        y = np.linspace(0, self.height_m, N)
        X, Y = np.meshgrid(x, y)

        # 1. Harmonic Multi-Octave Elevation Surface (Martian plains & ridges)
        elevation = np.zeros((N, N), dtype=np.float32)
        harmonics = [
            (0.003, 18.0),
            (0.007, 10.0),
            (0.015, 5.0),
            (0.035, 2.2),
            (0.08, 0.8),
        ]
        for freq, amp in harmonics:
            phase_x = np.random.uniform(0, 2 * np.pi)
            phase_y = np.random.uniform(0, 2 * np.pi)
            elevation += amp * (
                np.sin(freq * X + phase_x) * np.cos(freq * Y + phase_y)
                + 0.5 * np.cos(freq * 1.4 * X - phase_y)
            )

        # Base elevation range ~ 0 to 45 meters
        elevation -= elevation.min()

        # 2. Procedural Craters
        self.craters = []
        num_craters = 6
        for _ in range(num_craters):
            cx = np.random.uniform(100.0, self.width_m - 100.0)
            cy = np.random.uniform(100.0, self.height_m - 100.0)
            # Avoid placing directly on start or target
            if math.hypot(cx - self.start_pos[0], cy - self.start_pos[1]) < 80:
                continue
            if math.hypot(cx - self.target_pos[0], cy - self.target_pos[1]) < 80:
                continue

            radius = np.random.uniform(25.0, 55.0)
            depth = np.random.uniform(12.0, 25.0)
            self.craters.append({"x": cx, "y": cy, "radius": radius, "depth": depth})

            # Depress center and create raised rim
            dist = np.hypot(X - cx, Y - cy)
            # Crater bowl profile
            bowl = dist <= radius
            elevation[bowl] -= depth * (1.0 - (dist[bowl] / radius) ** 2)
            # Crater rim
            rim = (dist > radius) & (dist <= radius * 1.3)
            elevation[rim] += depth * 0.35 * np.exp(-((dist[rim] - radius) ** 2) / (2 * (radius * 0.15) ** 2))

        self.elevation_grid = elevation

        # 3. Compute Slope Gradient
        gy, gx = np.gradient(elevation, self.cell_size)
        self.slope_grid = np.sqrt(gx**2 + gy**2)

        # 4. Rock Fields
        self.rock_fields = []
        num_rock_fields = 7
        for _ in range(num_rock_fields):
            rx = np.random.uniform(80.0, self.width_m - 80.0)
            ry = np.random.uniform(80.0, self.height_m - 80.0)
            rad = np.random.uniform(20.0, 45.0)
            self.rock_fields.append({"x": rx, "y": ry, "radius": rad})

        # 5. Populate Types and Costs
        self.cost_grid = np.ones((N, N), dtype=np.float32) * self.COST_NORMAL
        self.type_grid = np.full((N, N), TerrainType.NORMAL, dtype=object)

        # Assign slopes (slope > 0.35 rad or ~20 deg)
        slope_mask = self.slope_grid > 0.30
        self.cost_grid[slope_mask] = self.COST_SLOPE
        self.type_grid[slope_mask] = TerrainType.SLOPE

        # Assign rocks
        for rf in self.rock_fields:
            dist = np.hypot(X - rf["x"], Y - rf["y"])
            rock_mask = dist <= rf["radius"]
            self.cost_grid[rock_mask] = np.maximum(self.cost_grid[rock_mask], self.COST_ROCK)
            self.type_grid[rock_mask] = TerrainType.ROCK

        # Assign crater hazards (steep inner walls or deep pits)
        for cr in self.craters:
            dist = np.hypot(X - cr["x"], Y - cr["y"])
            hazard_mask = dist <= (cr["radius"] * 0.85)
            self.cost_grid[hazard_mask] = self.COST_HAZARD
            self.type_grid[hazard_mask] = TerrainType.HAZARD

        # Apply any active injected hazards
        self._apply_injected_hazards()

    def _apply_injected_hazards(self) -> None:
        """Overlay dynamic/user injected hazards onto cost grid."""
        N = self.grid_res
        x = np.linspace(0, self.width_m, N)
        y = np.linspace(0, self.height_m, N)
        X, Y = np.meshgrid(x, y)

        for hz in self.injected_hazards:
            dist = np.hypot(X - hz["x"], Y - hz["y"])
            mask = dist <= hz["radius"]
            self.cost_grid[mask] = self.COST_HAZARD
            self.type_grid[mask] = TerrainType.HAZARD

    def inject_hazard(
        self,
        x: float,
        y: float,
        radius: float = 28.0,
        hazard_type: str = "ROCK_FIELD",
        hazard_id: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Dynamically adds a hazard zone to trigger AI replanning."""
        if hazard_id is None:
            hazard_id = f"HZ-{len(self.injected_hazards)+1:02d}"

        hazard = {
            "id": hazard_id,
            "x": float(np.clip(x, 0, self.width_m)),
            "y": float(np.clip(y, 0, self.height_m)),
            "radius": float(radius),
            "type": hazard_type,
        }
        self.injected_hazards.append(hazard)
        self._apply_injected_hazards()
        return hazard

    def clear_injected_hazards(self) -> None:
        """Clear dynamic hazards and regenerate base costs."""
        self.injected_hazards.clear()
        self.generate(seed=self.seed)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Map world coordinates (meters) to grid indices (col, row)."""
        gx = int(np.clip(round(x / self.cell_size), 0, self.grid_res - 1))
        gy = int(np.clip(round(y / self.cell_size), 0, self.grid_res - 1))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Map grid indices (col, row) to world coordinates (meters)."""
        return gx * self.cell_size, gy * self.cell_size

    def _bilinear_interpolate(self, grid: np.ndarray, x: float, y: float) -> float:
        """
        Bilinear interpolation on a 2D numpy grid for continuous world coordinates (x, y).
        Grid is indexed as grid[gy, gx] where gx is x-axis (col) and gy is y-axis (row).
        Clamps coordinates safely within grid boundaries [0, grid_res - 1].
        """
        fx = float(np.clip(x / self.cell_size, 0.0, float(self.grid_res - 1)))
        fy = float(np.clip(y / self.cell_size, 0.0, float(self.grid_res - 1)))

        gx0 = int(math.floor(fx))
        gy0 = int(math.floor(fy))

        gx1 = min(gx0 + 1, self.grid_res - 1)
        gy1 = min(gy0 + 1, self.grid_res - 1)

        tx = fx - gx0
        ty = fy - gy0

        q11 = float(grid[gy0, gx0])
        q21 = float(grid[gy0, gx1])
        q12 = float(grid[gy1, gx0])
        q22 = float(grid[gy1, gx1])

        val = (
            (1.0 - tx) * (1.0 - ty) * q11
            + tx * (1.0 - ty) * q21
            + (1.0 - tx) * ty * q12
            + tx * ty * q22
        )
        return float(val)

    def get_cost(self, x: float, y: float) -> float:
        """
        Get traversal cost at continuous world position (x, y).
        Uses bilinear interpolation for continuous non-hazard costs (normal, rock, slope).
        For cells containing impassable hazard values (>= COST_HAZARD), returns COST_HAZARD
        to preserve hazard non-traversability semantics for path safety.
        """
        fx = float(np.clip(x / self.cell_size, 0.0, float(self.grid_res - 1)))
        fy = float(np.clip(y / self.cell_size, 0.0, float(self.grid_res - 1)))

        gx0 = int(math.floor(fx))
        gy0 = int(math.floor(fy))
        gx1 = min(gx0 + 1, self.grid_res - 1)
        gy1 = min(gy0 + 1, self.grid_res - 1)

        q11 = float(self.cost_grid[gy0, gx0])
        q21 = float(self.cost_grid[gy0, gx1])
        q12 = float(self.cost_grid[gy1, gx0])
        q22 = float(self.cost_grid[gy1, gx1])

        if max(q11, q21, q12, q22) >= self.COST_HAZARD:
            return float(self.COST_HAZARD)

        return self._bilinear_interpolate(self.cost_grid, x, y)

    def get_elevation(self, x: float, y: float) -> float:
        """Get continuous interpolated terrain elevation (meters) at world position (x, y)."""
        return self._bilinear_interpolate(self.elevation_grid, x, y)

    def get_slope(self, x: float, y: float) -> float:
        """Get continuous interpolated terrain slope at world position (x, y)."""
        return self._bilinear_interpolate(self.slope_grid, x, y)

    def is_in_hazard(self, x: float, y: float, margin: float = 2.0) -> bool:
        """Check if position is inside or dangerously close to a hazard."""
        cost = self.get_cost(x, y)
        if cost >= self.COST_HAZARD:
            return True
        for hz in self.injected_hazards:
            if math.hypot(x - hz["x"], y - hz["y"]) <= (hz["radius"] + margin):
                return True
        for cr in self.craters:
            if math.hypot(x - cr["x"], y - cr["y"]) <= (cr["radius"] * 0.85 + margin):
                return True
        return False

    def to_dict(self) -> Dict[str, Any]:
        """Export compact data structure for frontend visualization."""
        # Subsample or return grid for frontend rendering
        return {
            "width_m": self.width_m,
            "height_m": self.height_m,
            "grid_res": self.grid_res,
            "cell_size": self.cell_size,
            "start_pos": list(self.start_pos),
            "target_pos": list(self.target_pos),
            "craters": self.craters,
            "rock_fields": self.rock_fields,
            "injected_hazards": self.injected_hazards,
            "elevation_summary": {
                "min": float(self.elevation_grid.min()),
                "max": float(self.elevation_grid.max()),
            },
            # Return downsampled elevation grid (e.g. 30x30 or full 60x60)
            "elevation": self.elevation_grid.tolist(),
            "cost_grid": self.cost_grid.tolist(),
        }
