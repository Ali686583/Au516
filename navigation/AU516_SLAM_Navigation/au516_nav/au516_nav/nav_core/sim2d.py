"""Simulateur 2D minimal : murs en segments, LiDAR 360° par lancer de rayons, voiture Ackermann.

Sert aux tests automatiques des scénarios (tests/test_scenarios.py) et au nœud ROS `sim_2d`
qui remplace le vrai robot (publie /scan, /odom, TF) quand Gazebo n'est pas disponible.
"""

import math
import numpy as np


class World:
    def __init__(self, segments):
        s = np.asarray(segments, dtype=float).reshape(-1, 4)
        self.a, self.b = s[:, :2], s[:, 2:]

    def raycast(self, x, y, yaw, n_beams=360, r_max=12.0, noise=0.0, rng=None):
        ang = yaw + np.linspace(-math.pi, math.pi, n_beams, endpoint=False)
        d = np.column_stack((np.cos(ang), np.sin(ang)))  # (B, 2)
        e = self.b - self.a  # (S, 2)
        w = self.a - np.array([x, y])  # (S, 2)
        den = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]  # (B, S)
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (w[None, :, 0] * e[None, :, 1] - w[None, :, 1] * e[None, :, 0]) / den
            u = (w[None, :, 0] * d[:, None, 1] - w[None, :, 1] * d[:, None, 0]) / den
        hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        r = np.where(hit, t, np.inf).min(axis=1)
        if noise > 0 and rng is not None:
            r = r + rng.normal(0, noise, r.shape)
        r[r > r_max] = np.inf
        return r, -math.pi, 2 * math.pi / n_beams

    def collides(self, poly):
        """Vrai si le polygone (4, 2) touche un mur."""
        for k in range(4):
            p, q = poly[k], poly[(k + 1) % 4]
            if np.any(_seg_intersect(p, q, self.a, self.b)):
                return True
        return False


def _seg_intersect(p, q, a, b):
    def orient(p1, p2, p3):
        return (p2[..., 0] - p1[..., 0]) * (p3[..., 1] - p1[..., 1]) - (
            p2[..., 1] - p1[..., 1]
        ) * (p3[..., 0] - p1[..., 0])

    o1, o2 = orient(p, q, a), orient(p, q, b)
    o3, o4 = orient(a, b, p), orient(a, b, q)
    return (o1 * o2 <= 0) & (o3 * o4 <= 0)


class Car:
    """Modèle bicyclette, origine au centre de l'essieu arrière."""

    def __init__(self, vehicle, x=0.0, y=0.0, yaw=0.0):
        self.veh = vehicle
        self.x, self.y, self.yaw = x, y, yaw
        self.v, self.delta = 0.0, 0.0

    def step(self, v_cmd, delta_cmd, dt, steer_rate=4.0):
        lim = self.veh.max_steer
        target = max(-lim, min(lim, delta_cmd))
        self.delta += max(-steer_rate * dt, min(steer_rate * dt, target - self.delta))
        self.v = v_cmd
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v * math.tan(self.delta) / self.veh.wheelbase * dt

    def polygon(self):
        v = self.veh
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        local = np.array(
            [
                [v.front_overhang, v.width / 2],
                [-v.rear_overhang, v.width / 2],
                [-v.rear_overhang, -v.width / 2],
                [v.front_overhang, -v.width / 2],
            ]
        )
        return np.column_stack(
            (
                self.x + c * local[:, 0] - s * local[:, 1],
                self.y + s * local[:, 0] + c * local[:, 1],
            )
        )

    def laser_pose(self):
        v = self.veh
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (
            self.x + c * v.laser_x - s * v.laser_y,
            self.y + s * v.laser_x + c * v.laser_y,
            self.yaw + v.laser_yaw,
        )
