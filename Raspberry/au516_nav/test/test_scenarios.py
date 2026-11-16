"""Validation en boucle fermée des scénarios A–G de navigation_lidar.txt (simulateur 2D).

Lancer :  python3 -m pytest test/test_scenarios.py -v     (aucune dépendance ROS)
"""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from au516_nav.nav_core import VehicleParams, NavParams  # noqa: E402
from au516_nav.nav_core.worlds import run_closed_loop  # noqa: E402

V, P = VehicleParams(), NavParams()


def run(name, duration, stop_when=None):
    h = run_closed_loop(name, V, P, duration=duration, stop_when=stop_when)
    for k in ('x', 'y', 'yaw', 'v', 'omega', 'theta', 'delta', 'vl', 'vr', 't'):
        h[k] = np.asarray(h[k])
    return h


def after(h, t0):
    return h['t'] >= t0


def test_A_straight_corridor():
    h = run('A_straight', 30, lambda c: c.x > 8)
    m = after(h, 4.0)
    assert not h['collision']
    assert h['x'][-1] > 8
    assert abs(h['y'][-1]) < 0.08  # décalage initial corrigé
    assert np.degrees(np.abs(h['theta'][m])).max() < 4.0  # θ_cible ≈ 0°
    assert np.degrees(np.abs(h['delta'][m])).max() < 6.0  # pas de correction excessive


@pytest.mark.parametrize('name,sign', [('B_left', 1), ('C_right', -1)])
def test_B_C_turns(name, sign):
    h = run(name, 30, lambda c: sign * c.y > 3.0)
    assert not h['collision']
    assert sign * h['y'][-1] > 3.0  # virage effectué
    turning = sign * h['omega'] > 0.3
    assert turning.any()
    if sign > 0:
        assert np.all(h['vl'][turning] < h['vr'][turning])  # gauche : V_G < V_D
    else:
        assert np.all(h['vl'][turning] > h['vr'][turning])  # droite : V_G > V_D
    assert h['v'][turning].max() < P.v_max  # ralentit dans le virage


def test_D_zigzag_keeps_straight():
    h = run('D_zigzag', 30, lambda c: c.x > 9)
    m = after(h, 2.0)
    assert not h['collision']
    assert h['x'][-1] > 9
    assert np.abs(h['y']).max() < 0.12  # ne suit pas les irrégularités
    assert math.degrees(np.sqrt(np.mean(h['theta'][m] ** 2))) < 4.0  # θ_cible ≈ 0°
    assert np.abs(h['vl'][m] - h['vr'][m]).max() < 0.05  # V_G ≈ V_D


def test_E_narrow_passage_ok():
    h = run('E_narrow_ok', 30, lambda c: c.x > 6.5)
    assert not h['collision']
    assert h['x'][-1] > 6.5
    inside = (h['x'] + V.front_overhang > 3.3) & (h['x'] - V.rear_overhang < 4.3)
    assert h['v'][inside].max() <= P.v_narrow + 1e-6  # ralentit dans le passage


def test_E_narrow_passage_refused():
    h = run('E_narrow_refused', 30)
    assert not h['collision']
    assert (h['x'] + V.front_overhang).max() < 3.3  # n'entre pas dans le goulet
    assert 'STOPPING' in h['state']  # refus explicite


def test_F_frontal_obstacle_avoided():
    h = run('F_obstacle', 30, lambda c: c.x > 5)
    assert not h['collision']
    assert h['x'][-1] > 5
    near = np.abs(h['x'] - 2.65) < 0.2
    assert np.abs(h['y'][near]).min() > 0.25  # contourne l'obstacle


def test_F_fully_blocked_stops_safely():
    h = run('F_blocked', 40)
    assert not h['collision']
    assert (h['x'] + V.front_overhang).max() < 3.0
    assert 'STOPPING' in h['state']


def test_G_dead_end_turnaround():
    h = run('G_dead_end', 60, lambda c: c.x < -2.0)
    assert not h['collision']
    assert len(h['deadends']) >= 1  # impasse mémorisée
    assert {'TURN_REVERSE', 'TURN_FORWARD'} <= set(h['state'])
    assert h['x'][-1] < -2.0  # repart dans l'autre sens
    assert abs(math.cos(h['yaw'][-1]) + 1) < 0.1


def test_ring_circuit_laps():
    h = run('ring', 120)
    assert not h['collision']
    laps = (np.unwrap(h['yaw'])[-1] - h['yaw'][0]) / (2 * math.pi)
    assert laps >= 2.0
