"""
Suivi de trajectoire « pure pursuit » (docs/SLAM_2D.md §11.3, docs/braquage_vitesse.md formule C).

On vise un point du chemin situé à la distance Ld devant la voiture, puis on
braque pour suivre l'arc de cercle qui y mène. Les valeurs par défaut sont
celles de la voiture (odometry_node.py) : empattement 0,165 m, braquage 20 degrés.

Démonstration :  python3 pure_pursuit.py
"""
import math

import numpy as np

WHEELBASE = 0.165                  # empattement L (m)
DELTA_MAX = math.radians(20.0)     # braquage maximal : 1000 us = 20 deg à gauche


def pure_pursuit(path, pose, L=WHEELBASE, Ld=0.6):
    """
    path : points (N x 2) dans le sens de la course. pose : (x, y, theta) dans la carte.
    Renvoie le braquage delta (rad, positif = gauche) et la courbure kappa (1/m).
    """
    x, y, th = pose
    d = np.hypot(path[:, 0] - x, path[:, 1] - y)
    i0 = int(np.argmin(d))
    n = len(path)
    for k in range(n):                     # circuit fermé : on boucle sur les indices
        i = (i0 + k) % n
        if d[i] >= Ld:
            break
    dx, dy = path[i, 0] - x, path[i, 1] - y
    xr = np.cos(th) * dx + np.sin(th) * dy     # point visé dans le repère robot
    yr = -np.sin(th) * dx + np.cos(th) * dy
    alpha = np.arctan2(yr, xr)
    kappa = 2 * np.sin(alpha) / Ld
    delta = np.arctan(L * kappa)
    return delta, kappa


def delta_to_pulse(delta, delta_max=DELTA_MAX):
    """Braquage (rad, positif = gauche) -> impulsion du servo en us (1000 = gauche, 2000 = droite)."""
    pulse = 1500 - 500 * delta / delta_max
    return int(round(min(max(pulse, 1000), 2000)))


if __name__ == "__main__":
    # Un cercle de rayon 2 m parcouru dans le sens trigonométrique (virage à gauche)
    ang = np.linspace(0, 2 * np.pi, 200, endpoint=False)
    cercle = np.c_[2 * np.cos(ang), 2 * np.sin(ang)]

    delta, kappa = pure_pursuit(cercle, (2.0, 0.0, np.pi / 2))
    attendu = math.degrees(math.atan(WHEELBASE / 2.0))
    print(f"courbure  : {kappa:.3f} 1/m   (cercle de 2 m : 0.500)")
    print(f"braquage  : {math.degrees(delta):.2f} deg  (géométrie exacte : {attendu:.2f} deg)")
    print(f"impulsion : {delta_to_pulse(delta)} us   (< 1500 : braque à gauche)")
