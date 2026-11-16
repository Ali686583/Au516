"""
Grille d'occupation en log-odds (docs/SLAM_2D.md, étape 4).

Chaque rayon du LiDAR marque les cases qu'il traverse comme libres et la case
où il s'arrête comme occupée. Code pour comprendre le principe : sur le robot,
c'est slam_toolbox qui construit la carte.

Démonstration :  python3 grille_occupation.py
"""
import numpy as np

RES = 0.05                       # taille d'une case (m)
L_OCC, L_FREE = 0.85, -0.85      # log(0.7/0.3) et log(0.3/0.7)
L_MIN, L_MAX = -4.0, 4.0         # bornes, pour que la carte puisse encore évoluer

origin = np.array([-5.0, -5.0])  # coordonnées monde de la case (0, 0)


def to_cell(p):
    """Point (x, y) en mètres -> indices (i, j) de la case."""
    return tuple(((p - origin) / RES).astype(int))


def bresenham(i0, j0, i1, j1):
    """Cases traversées par le rayon (i0,j0) -> (i1,j1), sans la case d'arrivée."""
    cells = []
    di, dj = abs(i1 - i0), abs(j1 - j0)
    si, sj = (1 if i1 > i0 else -1), (1 if j1 > j0 else -1)
    err = di - dj
    while (i0, j0) != (i1, j1):
        cells.append((i0, j0))
        e2 = 2 * err
        if e2 > -dj:
            err -= dj
            i0 += si
        if e2 < di:
            err += di
            j0 += sj
    return cells


def update(grid, pose, ranges, angles, r_min=0.05, r_max=8.0):
    """Ajoute un scan à la grille. pose = (x, y, theta) du LiDAR dans le repère carte."""
    x, y, th = pose
    c0 = to_cell(np.array([x, y]))
    for r, a in zip(ranges, angles):
        if not np.isfinite(r) or r < r_min or r > r_max:
            continue
        hit = np.array([x + r * np.cos(th + a), y + r * np.sin(th + a)])
        c1 = to_cell(hit)
        for c in bresenham(*c0, *c1):
            grid[c] += L_FREE        # cases traversées : libres
        grid[c1] += L_OCC            # case d'impact : occupée
    np.clip(grid, L_MIN, L_MAX, out=grid)


def probability(grid):
    """Log-odds -> probabilité d'occupation (0.5 = inconnu)."""
    return 1 - 1 / (1 + np.exp(grid))


if __name__ == "__main__":
    # Un mur vertical à 2 m devant le LiDAR, vu 3 fois
    grid = np.zeros((200, 200))
    angles = np.linspace(-0.5, 0.5, 50)
    ranges = 2.0 / np.cos(angles)
    for _ in range(3):
        update(grid, (0.0, 0.0, 0.0), ranges, angles)

    p = probability(grid)
    print(f"case du mur      (x = 2 m)  : p = {p[to_cell(np.array([2.0, 0.0]))]:.2f}   (attendu 0.93 : occupée)")
    print(f"case traversée   (x = 1 m)  : p = {p[to_cell(np.array([1.0, 0.0]))]:.2f}   (proche de 0 : libre)")
    print(f"case jamais vue  (x = -3 m) : p = {p[to_cell(np.array([-3.0, 0.0]))]:.2f}   (0.50 : inconnue)")
