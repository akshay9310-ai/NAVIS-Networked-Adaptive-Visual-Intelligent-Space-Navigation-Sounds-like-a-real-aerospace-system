"""
NAVIS - A* Path Planning & Dynamic Route Replanning
Computes optimal, safe paths on continuous Mars terrain grid with terrain cost weighting.
Supports dynamic replanning when hazards are injected or detected by imaging satellites.
"""

from typing import List, Tuple, Dict, Any, Optional
import heapq
import math
import numpy as np


class AStarPlanner:
    """
    8-Directional A* Path Planner with terrain cost field and path smoothing.
    """

    def __init__(self, terrain):
        self.terrain = terrain
        self.old_route: List[Tuple[float, float]] = []
        self.current_route: List[Tuple[float, float]] = []
        self.replanned_count = 0
        self.last_replan_reason = ""

    def plan_path(
        self,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        cost_grid: Optional[np.ndarray] = None,
    ) -> List[Tuple[float, float]]:
        """
        Execute A* pathfinding on discrete grid and convert to smooth world waypoints.
        """
        if cost_grid is None:
            cost_grid = self.terrain.cost_grid

        grid_res = self.terrain.grid_res
        cell_size = self.terrain.cell_size

        start_gx, start_gy = self.terrain.world_to_grid(start_world[0], start_world[1])
        goal_gx, goal_gy = self.terrain.world_to_grid(goal_world[0], goal_world[1])

        # Priority queue for open set: (f_score, h_score, (gx, gy))
        open_set: List[Tuple[float, float, Tuple[int, int]]] = []
        heapq.heappush(open_set, (0.0, 0.0, (start_gx, start_gy)))

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_score: Dict[Tuple[int, int], float] = {(start_gx, start_gy): 0.0}
        closed_set = set()

        # 8-Directional Neighbors with step lengths
        neighbors = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, 1.414),
            (1, -1, 1.414),
            (-1, 1, 1.414),
            (-1, -1, 1.414),
        ]

        def heuristic(gx: int, gy: int) -> float:
            # Euclidean distance with slight tie-breaker
            dx = gx - goal_gx
            dy = gy - goal_gy
            return math.hypot(dx, dy) * 1.001

        found = False

        while open_set:
            _, _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue
            closed_set.add(current)

            if current == (goal_gx, goal_gy):
                found = True
                break

            cgx, cgy = current
            current_g = g_score[current]

            for dx, dy, step_len in neighbors:
                ngx, ngy = cgx + dx, cgy + dy

                # Bounds check
                if not (0 <= ngx < grid_res and 0 <= ngy < grid_res):
                    continue

                cell_cost = float(cost_grid[ngy, ngx])
                # Impassable hazard
                if cell_cost >= self.terrain.COST_HAZARD:
                    continue

                # Edge cost = step distance * cell traversal cost
                edge_cost = step_len * cell_cost
                tentative_g = current_g + edge_cost

                neighbor = (ngx, ngy)
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    h = heuristic(ngx, ngy)
                    f = tentative_g + h
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f, h, neighbor))

        if not found:
            # Fallback direct line if completely blocked
            return [start_world, goal_world]

        # Reconstruct grid path
        grid_path = []
        curr = (goal_gx, goal_gy)
        while curr in came_from:
            grid_path.append(curr)
            curr = came_from[curr]
        grid_path.append((start_gx, start_gy))
        grid_path.reverse()

        # Convert to world coordinates
        world_path = [
            (
                gx * cell_size + cell_size * 0.5,
                gy * cell_size + cell_size * 0.5,
            )
            for gx, gy in grid_path
        ]

        # Smooth waypoints (reduce redundant collinear grid nodes)
        smoothed = self._smooth_path(world_path, cost_grid)
        # Ensure exact start and goal
        if smoothed:
            smoothed[0] = start_world
            smoothed[-1] = goal_world

        return smoothed

    def _smooth_path(
        self,
        raw_path: List[Tuple[float, float]],
        cost_grid: np.ndarray,
    ) -> List[Tuple[float, float]]:
        """String-pulling / shortcut path smoother to eliminate grid jaggedness."""
        if len(raw_path) <= 2:
            return raw_path

        smoothed = [raw_path[0]]
        current_idx = 0

        while current_idx < len(raw_path) - 1:
            furthest_visible = current_idx + 1
            for look_idx in range(len(raw_path) - 1, current_idx + 1, -1):
                if self._line_of_sight(
                    raw_path[current_idx], raw_path[look_idx], cost_grid
                ):
                    furthest_visible = look_idx
                    break
            smoothed.append(raw_path[furthest_visible])
            current_idx = furthest_visible

        return smoothed

    def _line_of_sight(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        cost_grid: np.ndarray,
    ) -> bool:
        """Check if straight segment between p1 and p2 crosses high-cost hazards or rocks."""
        dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        steps = max(2, int(dist / (self.terrain.cell_size * 0.5)))
        for i in range(steps + 1):
            alpha = i / steps
            qx = p1[0] + alpha * (p2[0] - p1[0])
            qy = p1[1] + alpha * (p2[1] - p1[1])
            gx, gy = self.terrain.world_to_grid(qx, qy)
            if cost_grid[gy, gx] >= (self.terrain.COST_HAZARD * 0.5):
                return False
        return True

    def trigger_replan(
        self,
        current_rover_pos: Tuple[float, float],
        goal_world: Tuple[float, float],
        reason: str = "HAZARD_DETECTED",
    ) -> Dict[str, Any]:
        """
        Archive previous path and calculate new safe route avoiding the latest hazard.
        """
        # Save previous route
        self.old_route = list(self.current_route)
        self.last_replan_reason = reason
        self.replanned_count += 1

        new_route = self.plan_path(current_rover_pos, goal_world)
        self.current_route = new_route

        return {
            "replan_id": f"REPLAN-{self.replanned_count:02d}",
            "reason": reason,
            "old_route": self.old_route,
            "new_route": self.current_route,
            "waypoints_count": len(self.current_route),
        }
