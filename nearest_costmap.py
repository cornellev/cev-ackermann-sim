from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class Grid:
    """Lightweight clone of cev_planner::Grid for simulator use."""

    data: np.ndarray
    origin: Tuple[float, float]
    resolution: float


class NearestCostMap:
    """Python equivalent of cev_planner::cost_map::NearestCostMap."""

    def __init__(self, grid: Grid):
        self.grid = grid

    def cost(self, x: float, y: float) -> float:
        """Lookup the cost of a world-space coordinate."""
        ix = int((x - self.grid.origin[0]) / self.grid.resolution)
        iy = int((y - self.grid.origin[1]) / self.grid.resolution)
        rows, cols = self.grid.data.shape
        if ix < 0 or iy < 0 or ix >= rows or iy >= cols:
            return float("inf")
        return float(self.grid.data[ix, iy])


class NearestGenerator:
    """Gaussian nearest cost-map generator that mirrors the C++ implementation."""

    def __init__(self, search_radius: int, sigma: float):
        if search_radius < 0:
            raise ValueError("search_radius must be non-negative")
        if sigma <= 0:
            raise ValueError("sigma must be positive")
        self.search_radius = int(search_radius)
        self.kernel = self._gen_kernel(self.search_radius, float(sigma))

    @staticmethod
    def _gen_kernel(search_radius: int, sigma: float) -> np.ndarray:
        offsets = np.arange(-search_radius, search_radius + 1, dtype=np.float32)
        return np.exp(-(offsets * offsets) / (2.0 * sigma * sigma))

    def generate_cost_map(self, grid: Grid) -> NearestCostMap:
        """Replicates NearestGenerator::generate_cost_map from C++."""
        data = np.array(grid.data, dtype=np.float32, copy=True)
        data = np.where(data < 0.0, 0.3, data)
        rows, cols = data.shape

        row_conv = np.zeros_like(data)
        for i in range(rows):
            for j in range(cols):
                best = 0.0
                for k in range(-self.search_radius, self.search_radius + 1):
                    idx = j + k
                    if 0 <= idx < cols:
                        tmp = data[i, idx] * self.kernel[k + self.search_radius]
                        if tmp > best:
                            best = tmp
                row_conv[i, j] = best

        cost_map = np.zeros_like(row_conv)
        for j in range(cols):
            for i in range(rows):
                best = 0.0
                for k in range(-self.search_radius, self.search_radius + 1):
                    idx = i + k
                    if 0 <= idx < rows:
                        tmp = row_conv[idx, j] * self.kernel[k + self.search_radius]
                        if tmp > best:
                            best = tmp
                cost_map[i, j] = best

        cost_grid = Grid(data=cost_map, origin=grid.origin, resolution=grid.resolution)
        return NearestCostMap(cost_grid)
