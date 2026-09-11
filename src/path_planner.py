#!/usr/bin/env python3
"""
Simple A* Path Planner operating on the 2D occupancy grid.
"""

import numpy as np
import heapq
from typing import List, Tuple, Optional
from occupancy_grid import OccupancyGrid2D


class AStarPlanner:
    def __init__(self, grid: OccupancyGrid2D):
        self.grid = grid

    def plan(self, start_xy: Tuple[float, float], goal_xy: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        sx, sy = self.grid.world_to_grid(*start_xy)
        gx, gy = self.grid.world_to_grid(*goal_xy)

        if not (self.grid.is_valid(sx, sy) and self.grid.is_valid(gx, gy)):
            return None

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_set = []
        heapq.heappush(open_set, (0, (sx, sy)))
        came_from = {}
        g_score = {(sx, sy): 0}

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == (gx, gy):
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(self.grid.grid_to_world(*current))
                    current = came_from[current]
                path.append(self.grid.grid_to_world(sx, sy))
                path.reverse()
                return path

            for dx, dy in directions:
                nx, ny = current[0] + dx, current[1] + dy
                if not self.grid.is_valid(nx, ny):
                    continue
                cell = self.grid.grid[ny, nx]
                if cell == 100:  # occupied
                    continue

                tentative_g = g_score[current] + (1.414 if dx and dy else 1.0)
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = tentative_g
                    f = tentative_g + heuristic((nx, ny), (gx, gy))
                    heapq.heappush(open_set, (f, (nx, ny)))
                    came_from[(nx, ny)] = current

        return None  # no path
