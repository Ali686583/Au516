"""Exploitation de la carte SLAM (OccupancyGrid) et mémoire des impasses."""

import math
import numpy as np


class GridMap:
    """Copie légère d'un nav_msgs/OccupancyGrid (valeurs -1 inconnu, 0..100 occupé)."""

    def __init__(self, data, width, height, resolution, origin_x, origin_y, origin_yaw=0.0):
        self.grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.res = float(resolution)
        self.ox, self.oy, self.oyaw = float(origin_x), float(origin_y), float(origin_yaw)
        self.width, self.height = width, height

    def world_to_cell(self, x, y):
        dx, dy = x - self.ox, y - self.oy
        c, s = math.cos(-self.oyaw), math.sin(-self.oyaw)
        gx, gy = c * dx - s * dy, s * dx + c * dy
        return int(math.floor(gx / self.res)), int(math.floor(gy / self.res))

    def value(self, x, y):
        i, j = self.world_to_cell(x, y)
        if 0 <= i < self.width and 0 <= j < self.height:
            return int(self.grid[j, i])
        return -1

    def ray_stats(self, x, y, angle, depth, occ_threshold=65):
        """Parcourt la carte le long d'un rayon.

        Retourne (distance au premier obstacle connu, fraction de cases inconnues).
        Les cases inconnues ne sont PAS considérées comme libres : elles pénalisent le score.
        """
        n = max(2, int(depth / self.res))
        unknown = 0
        for k in range(1, n + 1):
            d = k * self.res
            v = self.value(x + d * math.cos(angle), y + d * math.sin(angle))
            if v < 0:
                unknown += 1
            elif v >= occ_threshold:
                return d, unknown / k
        return float('inf'), unknown / n

    def rays_stats(self, x, y, angles, depths, occ_threshold=65):
        """Version vectorisée de ray_stats pour toutes les directions à la fois."""
        angles = np.asarray(angles, dtype=float)
        depths = np.asarray(depths, dtype=float)
        n = max(2, int(depths.max() / self.res))
        d = self.res * np.arange(1, n + 1)  # (n,)
        px = x + np.cos(angles)[:, None] * d[None, :] - self.ox  # (H, n)
        py = y + np.sin(angles)[:, None] * d[None, :] - self.oy
        c, s = math.cos(-self.oyaw), math.sin(-self.oyaw)
        gi = np.floor((c * px - s * py) / self.res).astype(int)
        gj = np.floor((s * px + c * py) / self.res).astype(int)
        inside = (gi >= 0) & (gi < self.width) & (gj >= 0) & (gj < self.height)
        val = np.full(gi.shape, -1, dtype=np.int16)
        val[inside] = self.grid[gj[inside], gi[inside]]
        valid = d[None, :] <= depths[:, None]
        occ = valid & (val >= occ_threshold)
        first = np.where(occ.any(axis=1), d[np.argmax(occ, axis=1)], np.inf)
        before = valid & (d[None, :] < first[:, None])
        unk = ((val < 0) & before).sum(axis=1) / np.maximum(before.sum(axis=1), 1)
        return first, unk


class DeadEndMemory:
    """Positions (repère map) des impasses déjà identifiées."""

    def __init__(self, radius=0.8):
        self.radius = radius
        self.points = []

    def add(self, x, y):
        for px, py in self.points:
            if math.hypot(px - x, py - y) < self.radius:
                return
        self.points.append((x, y))

    def penalty(self, x, y):
        """1 si (x, y) est dans une impasse connue, décroît jusqu'à 0 à 2 x rayon."""
        best = 0.0
        for px, py in self.points:
            d = math.hypot(px - x, py - y)
            best = max(best, min(1.0, max(0.0, 2.0 - d / self.radius)))
        return best
