"""Environnements de test correspondant aux scénarios de navigation_lidar.txt.

Chaque monde : segments de murs (x1, y1, x2, y2) + pose de départ (x, y, yaw).
"""

import math
import numpy as np

from .sim2d import World, Car
from .geometry import scan_to_points
from .control import twist_to_ackermann, differential_wheel_speeds
from .navigator import LocalNavigator
from .mapping import DeadEndMemory


def _poly(points):
    return [
        (points[i][0], points[i][1], points[i + 1][0], points[i + 1][1])
        for i in range(len(points) - 1)
    ]


def _box(x0, y0, x1, y1):
    return _poly([(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)])


def straight():  # Scénario A
    return _poly([(-1, 0.5), (10, 0.5)]) + _poly([(-1, -0.5), (10, -0.5)]) + [
        (-1, -0.5, -1, 0.5)
    ], (0.0, 0.12, math.radians(3))


def turn_left():  # Scénario B
    walls = _poly([(-1, 0.5), (3, 0.5), (3, 6)]) + _poly([(-1, -0.5), (4, -0.5), (4, 6)])
    return walls + [(-1, -0.5, -1, 0.5)], (0.0, 0.0, 0.0)


def turn_right():  # Scénario C
    walls, start = turn_left()
    return [(x1, -y1, x2, -y2) for x1, y1, x2, y2 in walls], start


def zigzag(amp=0.3, period=1.0, half=0.7, length=10.0):  # Scénario D
    left, right = [(-1, half)], [(-1, -half)]
    x, k = 0.0, 0
    while x < length:
        yl = half - amp if k % 2 == 0 else half  # avancées du mur gauche
        yr = -half if k % 2 == 0 else -half + amp  # le mur droit avance en décalé
        left += [(x, left[-1][1]), (x, yl)]
        right += [(x, right[-1][1]), (x, yr)]
        x += period
        k += 1
    left.append((length, left[-1][1]))
    right.append((length, right[-1][1]))
    return _poly(left) + _poly(right) + [(-1, -half, -1, half)], (0.0, 0.0, 0.0)


def narrow(gap=0.46):  # Scénario E
    h, g = 0.6, gap / 2
    left = _poly([(-1, h), (3, h), (3.3, g), (4.3, g), (4.6, h), (9, h)])
    right = _poly([(-1, -h), (3, -h), (3.3, -g), (4.3, -g), (4.6, -h), (9, -h)])
    return left + right + [(-1, -h, -1, h)], (0.0, 0.08, 0.0)


def obstacle(blocked=False):  # Scénario F
    if blocked:
        w = _poly([(-1, 0.5), (8, 0.5)]) + _poly([(-1, -0.5), (8, -0.5)])
        return w + _box(3.0, -0.5, 3.3, 0.5) + [(-1, -0.5, -1, 0.5)], (0.0, 0.0, 0.0)
    w = _poly([(-1, 0.8), (8, 0.8)]) + _poly([(-1, -0.8), (8, -0.8)])
    return w + _box(2.5, -0.15, 2.8, 0.15) + [(-1, -0.8, -1, 0.8)], (0.0, 0.0, 0.0)


def dead_end(width=1.6):  # Scénario G
    h = width / 2
    return _poly([(-7, h), (4, h), (4, -h), (-7, -h)]), (0.0, 0.0, 0.0)


def t_junction():  # carrefour en T (tests mémoire d'impasse / carte)
    walls = (
        _poly([(-1, 0.5), (3, 0.5), (3, 4)])
        + _poly([(-1, -0.5), (3, -0.5), (3, -4)])
        + [(4, -4, 4, 4), (-1, -0.5, -1, 0.5)]
    )
    return walls, (1.9, 0.0, 0.0)


def rasterize(segments, resolution=0.05, extent=(-8, -6, 8, 6)):
    """Grille d'occupation 'parfaite' du monde (pour tester l'usage de la carte SLAM)."""
    x0, y0, x1, y1 = extent
    w, h = int((x1 - x0) / resolution), int((y1 - y0) / resolution)
    grid = np.zeros((h, w), dtype=np.int16)
    for ax, ay, bx, by in segments:
        n = int(math.hypot(bx - ax, by - ay) / (resolution / 2)) + 2
        for t in np.linspace(0, 1, n):
            i, j = int((ax + t * (bx - ax) - x0) / resolution), int(
                (ay + t * (by - ay) - y0) / resolution
            )
            if 0 <= i < w and 0 <= j < h:
                grid[j, i] = 100
    return grid, (x0, y0, resolution)


def ring():  # circuit de course : anneau avec chicane
    outer = _poly([(-4, -2.5), (4, -2.5), (4, 2.5), (-4, 2.5), (-4, -2.5)])
    inner = _poly([(-3, -1.5), (3, -1.5), (3, 1.5), (-3, 1.5), (-3, -1.5)])
    chicane = [(0.0, -2.5, 0.0, -2.15), (1.2, -1.5, 1.2, -1.85)]
    return outer + inner + chicane, (-1.5, -2.0, 0.0)


SCENARIOS = {
    'A_straight': straight,
    'B_left': turn_left,
    'C_right': turn_right,
    'D_zigzag': zigzag,
    'E_narrow_ok': lambda: narrow(0.46),
    'E_narrow_refused': lambda: narrow(0.30),
    'F_obstacle': obstacle,
    'F_blocked': lambda: obstacle(True),
    'G_dead_end': dead_end,
    'ring': ring,
    'T_junction': t_junction,
}


def run_closed_loop(name, vehicle, params, duration=20.0, dt=0.05, stop_when=None, seed=0):
    """Fait rouler la voiture simulée avec le navigateur ; renvoie l'historique."""
    segs, (x, y, yaw) = SCENARIOS[name]()
    world, car = World(segs), Car(vehicle, x, y, yaw)
    nav = LocalNavigator(vehicle, params, DeadEndMemory(params.deadend_memory_radius))
    rng = np.random.default_rng(seed)
    hist = {
        k: []
        for k in (
            't',
            'x',
            'y',
            'yaw',
            'v',
            'omega',
            'theta',
            'delta',
            'state',
            'vl',
            'vr',
            'n_passages',
            'narrow',
        )
    }
    hist['collision'] = False
    delta = 0.0
    for k in range(int(duration / dt)):
        lx, ly, lyaw = car.laser_pose()
        ranges, amin, inc = world.raycast(lx, ly, lyaw, noise=0.005, rng=rng)
        pts = scan_to_points(ranges, amin, inc, 0.05, 12.0, vehicle)
        out = nav.step(pts, dt, pose=(car.x, car.y, car.yaw))
        _, delta = twist_to_ackermann(out.v, out.omega, vehicle, delta)
        car.step(out.v, delta, dt)
        vl, vr = differential_wheel_speeds(out.v, out.omega, vehicle.track)
        for key, val in (
            ('t', k * dt),
            ('x', car.x),
            ('y', car.y),
            ('yaw', car.yaw),
            ('v', out.v),
            ('omega', out.omega),
            ('theta', out.theta),
            ('delta', car.delta),
            ('state', out.state),
            ('vl', vl),
            ('vr', vr),
            ('n_passages', len(out.passages)),
            ('narrow', out.narrow),
        ):
            hist[key].append(val)
        if world.collides(car.polygon()):
            hist['collision'] = True
            break
        if stop_when is not None and stop_when(car):
            break
    hist['deadends'] = list(nav.deadends.points)
    hist['segments'] = segs
    return hist
