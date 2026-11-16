"""Analyse géométrique du scan LiDAR 2D (sans dépendance ROS).

Idée centrale : pour chaque direction candidate φ, on "balaye" une BANDE de largeur
W = largeur robot + 2 x marge le long de φ. La profondeur libre d(φ) est la distance
jusqu'au premier point LiDAR qui tombe dans cette bande. Une ouverture plus étroite
que W n'est donc jamais vue comme libre : c'est le test de traversabilité réelle.
"""

import math
import numpy as np


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max, vehicle):
    """LaserScan -> points (N, 2) exprimés dans base_link."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(r.size)
    ok = np.isfinite(r) & (r >= max(range_min, 0.05)) & (r <= range_max)
    r, a = r[ok], a[ok] + vehicle.laser_yaw
    pts = np.column_stack((r * np.cos(a) + vehicle.laser_x, r * np.sin(a) + vehicle.laser_y))
    # On retire les points qui tombent sur la carrosserie (auto-occultation).
    body = (
        (pts[:, 0] > -vehicle.rear_overhang)
        & (pts[:, 0] < vehicle.front_overhang)
        & (np.abs(pts[:, 1]) < vehicle.width / 2)
    )
    return pts[~body]


class StripAnalysis:
    """Résultat de l'analyse de toutes les directions pour un scan."""

    def __init__(self, headings, depth, left, right, left_min, right_min):
        self.headings = headings  # (H,) angles φ
        self.depth = depth  # (H,) profondeur libre d(φ)
        self.left = left  # (H,) distance robuste à la paroi gauche (médiane)
        self.right = right  # (H,) distance robuste à la paroi droite
        self.left_min = left_min  # (H,) distance mini à gauche (sécurité)
        self.right_min = right_min


def analyse_strips(points, vehicle, params):
    """Calcule d(φ) et les dégagements latéraux pour chaque direction candidate."""
    headings = np.arange(-params.heading_span, params.heading_span + 1e-9, params.heading_step)
    half_w = vehicle.width / 2 + params.safety_margin
    cap = params.lookahead_cap * 2
    H = headings.size
    if points.shape[0] == 0:
        inf = np.full(H, cap)
        return StripAnalysis(headings, inf, inf.copy(), inf.copy(), inf.copy(), inf.copy())

    # Les bandes partent du centre de l'essieu avant (là où la voiture "pivote" en pratique)
    origin = np.array([vehicle.wheelbase * 0.5, 0.0])
    p = points - origin
    c, s = np.cos(headings)[:, None], np.sin(headings)[:, None]
    u = c * p[:, 0] + s * p[:, 1]  # (H, N) avance le long de φ
    w = -s * p[:, 0] + c * p[:, 1]  # (H, N) écart latéral (gauche > 0)

    inside = (np.abs(w) < half_w) & (u > -0.05)
    depth = np.where(inside, u, np.inf).min(axis=1)
    depth = np.minimum(depth, cap)

    lo, hi = params.side_window
    win = (u > lo) & (u < hi)
    left_mask = win & (w > 0)
    right_mask = win & (w < 0)
    left = _masked_percentile(np.where(left_mask, w, np.nan), 30, cap)
    right = _masked_percentile(np.where(right_mask, -w, np.nan), 30, cap)
    left_min = np.where(left_mask, w, np.inf).min(axis=1)
    right_min = np.where(right_mask, -w, np.inf).min(axis=1)
    return StripAnalysis(
        headings,
        depth,
        np.minimum(left, cap),
        np.minimum(right, cap),
        np.minimum(left_min, cap),
        np.minimum(right_min, cap),
    )


def _masked_percentile(arr, q, default):
    """Percentile ligne par ligne en ignorant les NaN (valeur par défaut si ligne vide)."""
    out = np.full(arr.shape[0], float(default))
    valid = ~np.isnan(arr)
    for i in np.nonzero(valid.any(axis=1))[0]:
        out[i] = np.percentile(arr[i, valid[i]], q)
    return out


def min_distance_in_cone(points, direction, half_angle, max_range=10.0):
    """Distance minimale des points situés dans un cône (pour la sécurité)."""
    if points.shape[0] == 0:
        return max_range
    ang = np.arctan2(points[:, 1], points[:, 0])
    d = np.hypot(points[:, 0], points[:, 1])
    diff = np.abs((ang - direction + math.pi) % (2 * math.pi) - math.pi)
    sel = diff < half_angle
    return float(d[sel].min()) if sel.any() else max_range


def rear_clearance(points, vehicle):
    """Espace libre derrière le pare-choc arrière, dans la largeur du robot."""
    half = vehicle.width / 2 + 0.03
    behind = points[(points[:, 0] < -vehicle.rear_overhang) & (np.abs(points[:, 1]) < half)]
    if behind.shape[0] == 0:
        return 10.0
    return float(-behind[:, 0].max() - vehicle.rear_overhang)


def front_clearance(points, vehicle, params):
    """Espace libre devant le pare-choc avant, dans la largeur du robot."""
    half = vehicle.width / 2 + params.safety_margin * 0.5
    ahead = points[(points[:, 0] > vehicle.front_overhang) & (np.abs(points[:, 1]) < half)]
    if ahead.shape[0] == 0:
        return 10.0
    return float(ahead[:, 0].min() - vehicle.front_overhang)
