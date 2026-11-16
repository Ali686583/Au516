"""
ICP (Iterative Closest Point) en 2D (docs/SLAM_2D.md, étape 5).

Trouve la rotation R et la translation t qui superposent au mieux un nouveau
scan Q sur un scan de référence P. Code pour comprendre le principe : sur le
robot, c'est slam_toolbox qui fait le recalage (par corrélation).

Démonstration :  python3 icp.py      (nécessite numpy et scipy)
"""
import numpy as np
from scipy.spatial import cKDTree


def icp(Q, P, iters=30, d_max=0.3, tol=1e-6):
    """Aligne le nuage Q (N x 2) sur le nuage P (M x 2). Renvoie R (2x2) et t (2,)."""
    tree = cKDTree(P)
    R, t = np.eye(2), np.zeros(2)
    prev_err = np.inf
    for _ in range(iters):
        Qt = Q @ R.T + t                       # Q transformé avec l'estimation courante
        dist, idx = tree.query(Qt)             # association : plus proche voisin
        keep = dist < d_max                    # rejet des paires aberrantes
        A, B = Qt[keep], P[idx[keep]]
        a_bar, b_bar = A.mean(axis=0), B.mean(axis=0)
        H = (A - a_bar).T @ (B - b_bar)        # matrice de covariance croisée
        U, _, Vt = np.linalg.svd(H)
        dR = Vt.T @ U.T
        if np.linalg.det(dR) < 0:              # éviter une réflexion
            Vt[1] *= -1
            dR = Vt.T @ U.T
        dt = b_bar - dR @ a_bar
        R, t = dR @ R, dR @ t + dt             # composition avec l'estimation précédente
        err = np.mean(dist[keep] ** 2)
        if abs(prev_err - err) < tol:
            break
        prev_err = err
    return R, t


if __name__ == "__main__":
    # Couloir en L (deux murs parallèles + un mur au fond), comme un bout de piste
    s = np.linspace(0, 3, 150)
    P = np.vstack([
        np.c_[s, np.full_like(s, 0.5)],
        np.c_[s, np.full_like(s, -0.5)],
        np.c_[np.full(60, 3.0), np.linspace(-0.5, 2, 60)],
    ])

    # Le même couloir vu après un déplacement connu : 5 degrés et (8 cm, -4 cm), avec du bruit
    th = np.deg2rad(5)
    R_vrai = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    t_vrai = np.array([0.08, -0.04])
    Q = (P - t_vrai) @ R_vrai                  # donc P = R_vrai Q + t_vrai
    Q += np.random.default_rng(0).normal(0, 0.005, Q.shape)

    R, t = icp(Q, P)
    angle = np.rad2deg(np.arctan2(R[1, 0], R[0, 0]))
    print(f"rotation    trouvée : {angle:.2f} deg      (vraie : 5.00 deg)")
    print(f"translation trouvée : ({t[0]:.3f}, {t[1]:.3f}) m (vraie : (0.080, -0.040) m)")
