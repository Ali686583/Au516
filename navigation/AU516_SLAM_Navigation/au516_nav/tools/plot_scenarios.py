"""Trace les trajectoires des scénarios A–G (simulateur 2D) et écrit un tableau de résultats.

    python3 tools/plot_scenarios.py doc/
"""
import math
import os
import sys

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from au516_nav.nav_core import VehicleParams, NavParams  # noqa: E402
from au516_nav.nav_core.worlds import run_closed_loop  # noqa: E402

RUNS = [
    ('A_straight', 'A — couloir droit', 30, lambda c: c.x > 8),
    ('B_left', 'B — virage à gauche', 30, lambda c: c.y > 3),
    ('C_right', 'C — virage à droite', 30, lambda c: c.y < -3),
    ('D_zigzag', 'D — parois en zigzag', 30, lambda c: c.x > 9),
    ('E_narrow_ok', 'E — passage étroit 0,46 m (accepté)', 30, lambda c: c.x > 6.5),
    ('E_narrow_refused', 'E — passage 0,30 m (refusé)', 30, None),
    ('F_obstacle', 'F — obstacle frontal', 30, lambda c: c.x > 5),
    ('F_blocked', 'F — obstacle bloquant', 40, None),
    ('G_dead_end', 'G — impasse + demi-tour', 60, lambda c: c.x < -2),
    ('ring', 'Circuit en anneau (120 s)', 120, None),
]


def main(out_dir):
    V, P = VehicleParams(), NavParams()
    fig, axes = plt.subplots(5, 2, figsize=(12, 20))
    rows = []
    for ax, (name, title, T, goal) in zip(axes.ravel(), RUNS):
        h = run_closed_loop(name, V, P, duration=T, stop_when=goal)
        for x1, y1, x2, y2 in h['segments']:
            ax.plot([x1, x2], [y1, y2], color='0.3', lw=2)
        x, y = np.array(h['x']), np.array(h['y'])
        sc = ax.scatter(x, y, c=h['v'], s=3, cmap='viridis', vmin=0, vmax=P.v_max)
        for px, py in h['deadends']:
            ax.add_patch(plt.Circle((px, py), P.deadend_memory_radius, color='r', alpha=0.2))
        ax.set_title(title + (' — COLLISION' if h['collision'] else ''))
        ax.set_aspect('equal')
        ax.grid(alpha=0.3)
        th = np.degrees(np.array(h['theta']))
        laps = (np.unwrap(h['yaw'])[-1] - h['yaw'][0]) / (2 * math.pi)
        rows.append((title, h['collision'], h['t'][-1], float(np.abs(th[40:]).mean()),
                     max(h['v']), sorted(set(h['state'])), laps))
    fig.colorbar(sc, ax=axes, shrink=0.3, label='v (m/s)')
    fig.savefig(os.path.join(out_dir, 'scenarios.png'), dpi=110, bbox_inches='tight')
    with open(os.path.join(out_dir, 'resultats_simulation.md'), 'w') as f:
        f.write('| Scénario | Collision | Durée (s) | moy. abs(θ_cible) (°) | v max (m/s) | États |\n')
        f.write('|---|---|---|---|---|---|\n')
        for t, c, d, th, vm, st, laps in rows:
            extra = f' ({laps:.1f} tours)' if 'anneau' in t else ''
            f.write(f'| {t}{extra} | {"OUI" if c else "non"} | {d:.1f} | {th:.1f} | {vm:.2f} | '
                    f'{", ".join(st)} |\n')
    print(open(os.path.join(out_dir, 'resultats_simulation.md')).read())


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else '.')
