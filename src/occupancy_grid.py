#!/usr/bin/env python3
"""
Real-time 2D Occupancy Grid for NIDAR AirMouse
Resolution typically 5-10 cm. Origin at start position.
"""

import numpy as np
from typing import Tuple, List, Optional
import cv2


class OccupancyGrid2D:
    def __init__(
        self,
        width_m: float = 20.0,
        height_m: float = 20.0,
        resolution: float = 0.05,  # 5 cm
        origin_x: float = -2.0,
        origin_y: float = -2.0,
    ):
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width_cells = int(width_m / resolution)
        self.height_cells = int(height_m / resolution)

        # -1 = unknown, 0 = free, 100 = occupied
        self.grid = np.full((self.height_cells, self.width_cells), -1, dtype=np.int8)

        # Survivor tags: list of dicts {id, x, y, conf, last_seen}
        self.survivors: List[dict] = []
        self.next_survivor_id = 1

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = gx * self.resolution + self.origin_x + self.resolution / 2
        y = gy * self.resolution + self.origin_y + self.resolution / 2
        return x, y

    def is_valid(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width_cells and 0 <= gy < self.height_cells

    def mark_free(self, x: float, y: float, radius_m: float = 0.3):
        """Mark a circular area as free (0)."""
        gx, gy = self.world_to_grid(x, y)
        r = int(radius_m / self.resolution)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    nx, ny = gx + dx, gy + dy
                    if self.is_valid(nx, ny):
                        if self.grid[ny, nx] == -1:  # only fill unknown
                            self.grid[ny, nx] = 0

    def mark_occupied(self, x: float, y: float, radius_m: float = 0.15):
        """Mark a circular area as occupied (100)."""
        gx, gy = self.world_to_grid(x, y)
        r = max(1, int(radius_m / self.resolution))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    nx, ny = gx + dx, gy + dy
                    if self.is_valid(nx, ny):
                        self.grid[ny, nx] = 100

    def mark_ray_free(self, x0: float, y0: float, x1: float, y1: float):
        """Bresenham-style ray: mark cells from robot to a point as free."""
        gx0, gy0 = self.world_to_grid(x0, y0)
        gx1, gy1 = self.world_to_grid(x1, y1)

        dx = abs(gx1 - gx0)
        dy = abs(gy1 - gy0)
        sx = 1 if gx0 < gx1 else -1
        sy = 1 if gy0 < gy1 else -1
        err = dx - dy

        x, y = gx0, gy0
        while True:
            if self.is_valid(x, y) and self.grid[y, x] != 100:
                self.grid[y, x] = 0
            if x == gx1 and y == gy1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def add_or_update_survivor(self, x: float, y: float, conf: float = 0.8, max_dist: float = 1.0):
        """Associate detection with existing survivor or create new one."""
        for s in self.survivors:
            dist = np.hypot(s["x"] - x, s["y"] - y)
            if dist < max_dist:
                # Update existing
                s["x"] = 0.7 * s["x"] + 0.3 * x
                s["y"] = 0.7 * s["y"] + 0.3 * y
                s["conf"] = max(s["conf"], conf)
                s["last_seen"] = 0
                return s["id"]

        # New survivor
        sid = self.next_survivor_id
        self.next_survivor_id += 1
        self.survivors.append({
            "id": sid,
            "x": x,
            "y": y,
            "conf": conf,
            "last_seen": 0,
        })
        return sid

    def age_survivors(self):
        for s in self.survivors:
            s["last_seen"] += 1

    def get_occupancy_image(self, scale: int = 2) -> np.ndarray:
        """Return a color visualization of the grid + survivors."""
        # Map: unknown=gray, free=white, occupied=black
        img = np.zeros((self.height_cells, self.width_cells, 3), dtype=np.uint8)
        img[self.grid == -1] = (80, 80, 80)
        img[self.grid == 0] = (240, 240, 240)
        img[self.grid == 100] = (30, 30, 30)

        # Draw survivors (red circles + ID)
        for s in self.survivors:
            gx, gy = self.world_to_grid(s["x"], s["y"])
            if self.is_valid(gx, gy):
                cv2.circle(img, (gx, gy), 4, (0, 0, 255), -1)
                cv2.putText(
                    img, f"S{s['id']}", (gx + 6, gy - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1
                )

        if scale != 1:
            img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
        return img

    def save_pgm(self, path: str):
        """Save classic ROS-style PGM map (optional)."""
        # Convert to 0-255: free=254, occupied=0, unknown=205
        out = np.full_like(self.grid, 205, dtype=np.uint8)
        out[self.grid == 0] = 254
        out[self.grid == 100] = 0
        cv2.imwrite(path, out)
