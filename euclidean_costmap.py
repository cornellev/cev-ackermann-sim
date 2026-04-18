from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple
import time

import numpy as np

INF = 1e10


@dataclass
class Grid:
    """Lightweight clone of cev_planner::Grid for simulator use."""

    data: np.ndarray
    origin: Tuple[float, float]
    resolution: float


class EuclideanCostMap:
    """Python equivalent of cev_planner::cost_map::EuclideanCostMap."""

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


class Euclidean:
    """
    Exact Euclidean Distance Transform generator.
    Mirrors cev_planner::cost_map::Euclidean (C++).
    """

    @staticmethod
    def _edt_1d(f: np.ndarray, out: np.ndarray) -> None:
        """
        1D squared Euclidean Distance Transform.
        Direct translation of the C++ implementation.
        """
        n = f.shape[0]
        if n == 0:
            return

        v = np.zeros(n, dtype=np.int32)
        z = np.zeros(n + 1, dtype=np.float32)

        k = 0
        v[0] = 0
        z[0] = -INF
        z[1] = INF

        for q in range(1, n):
            x = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (
                2.0 * q - 2.0 * v[k]
            )

            while x <= z[k]:
                k -= 1
                x = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (
                    2.0 * q - 2.0 * v[k]
                )

            k += 1
            v[k] = q
            z[k] = x
            z[k + 1] = INF

        k_idx = 0
        for q in range(n):
            while z[k_idx + 1] < q:
                k_idx += 1

            dx = q - v[k_idx]
            out[q] = dx * dx + f[v[k_idx]]

    def generate_cost_map(self, grid: Grid) -> EuclideanCostMap:
        """
        Replicates Euclidean::generate_cost_map from C++ using EDT.
        """
        rows, cols = grid.data.shape

        # 0 = occupied, INF = free (matches C++ semantics)
        cost_data = np.where(
            grid.data < 0.5,
            INF,
            0.0,
        ).astype(np.float32)

        # Pass 1: row-wise EDT
        for i in range(rows):
            row_in = cost_data[i, :]
            row_out = np.empty(cols, dtype=np.float32)
            self._edt_1d(row_in, row_out)
            cost_data[i, :] = row_out

        # Pass 2: column-wise EDT
        for j in range(cols):
            col_in = cost_data[:, j]
            col_out = np.empty(rows, dtype=np.float32)
            self._edt_1d(col_in, col_out)
            cost_data[:, j] = col_out

        cost_grid = Grid(
            data=np.sqrt(cost_data),
            origin=grid.origin,
            resolution=grid.resolution,
        )

        return EuclideanCostMap(cost_grid)
