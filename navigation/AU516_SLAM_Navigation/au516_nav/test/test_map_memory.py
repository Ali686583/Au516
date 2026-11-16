"""Tests de l'exploitation de la carte SLAM et de la mémoire d'impasses (sans ROS)."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from au516_nav.nav_core import VehicleParams, NavParams  # noqa: E402
from au516_nav.nav_core import LocalNavigator, GridMap, DeadEndMemory  # noqa: E402
from au516_nav.nav_core.geometry import scan_to_points, analyse_strips  # noqa: E402
from au516_nav.nav_core.sim2d import World, Car  # noqa: E402
from au516_nav.nav_core.worlds import t_junction, rasterize  # noqa: E402

V, P = VehicleParams(), NavParams()


def setup():
    segs, (x, y, yaw) = t_junction()
    world, car = World(segs), Car(V, 3.0, y, yaw)
    lx, ly, lyaw = car.laser_pose()
    r, amin, inc = world.raycast(lx, ly, lyaw)
    pts = scan_to_points(r, amin, inc, 0.05, 12.0, V)
    return segs, car, pts


def chosen_side(nav, pts, car, gm=None):
    strips = analyse_strips(pts, V, P)
    passages = nav.find_passages(strips, (car.x, car.y, car.yaw), gm)
    return np.sign(nav.select(passages).direction), passages


def test_deadend_memory_redirects_choice():
    segs, car, pts = setup()
    mem = DeadEndMemory(P.deadend_memory_radius)
    nav = LocalNavigator(V, P, mem)
    side0, passages = chosen_side(nav, pts, car)
    assert len(passages) >= 2  # gauche et droite détectées
    mem.add(3.5, 1.6 * side0)  # la branche choisie est une impasse connue
    nav2 = LocalNavigator(V, P, mem)
    side1, _ = chosen_side(nav2, pts, car)
    assert side1 == -side0


def test_unknown_area_not_treated_as_free():
    segs, car, pts = setup()
    grid, (x0, y0, res) = rasterize(segs)
    # moitié droite du carrefour inconnue sur la carte
    j_mid = int((0.0 - y0) / res)
    grid[:j_mid, int((3.0 - x0) / res):] = -1
    gm = GridMap(grid.ravel(), grid.shape[1], grid.shape[0], res, x0, y0)
    nav = LocalNavigator(V, P, None)
    side, passages = chosen_side(nav, pts, car, gm)
    assert side > 0  # préfère la branche connue (gauche)
    right = [q for q in passages if q.direction < 0]
    assert right and right[0].unknown > 0.3


def test_map_obstacle_blocks_direction():
    segs, car, pts = setup()
    grid, (x0, y0, res) = rasterize(segs + [(3.0, 0.6, 4.0, 0.6)])  # obstacle connu à gauche
    gm = GridMap(grid.ravel(), grid.shape[1], grid.shape[0], res, x0, y0)
    nav = LocalNavigator(V, P, None)
    side, _ = chosen_side(nav, pts, car, gm)
    assert side < 0
