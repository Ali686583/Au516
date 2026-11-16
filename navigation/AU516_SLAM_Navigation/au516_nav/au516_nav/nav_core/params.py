"""Paramètres de la navigation AU516 (valeurs par défaut à recalibrer sur la vraie voiture).

Toutes les distances sont en mètres, les angles en radians, les vitesses en m/s.
Repère `base_link` : origine au centre de l'essieu ARRIÈRE, x vers l'avant, y vers la gauche.
"""

from dataclasses import dataclass, fields
import math


@dataclass
class VehicleParams:
    wheelbase: float = 0.25  # L : essieu avant <-> essieu arrière (à mesurer)
    track: float = 0.18  # voie arrière (uniquement pour le diagnostic V_G / V_D)
    width: float = 0.20  # largeur hors-tout
    rear_overhang: float = 0.06  # distance essieu arrière -> pare-choc arrière
    front_overhang: float = 0.30  # distance essieu arrière -> pare-choc avant
    max_steer: float = 0.45  # braquage max delta_max (rad), ~26°
    laser_x: float = 0.22  # position du LiDAR dans base_link
    laser_y: float = 0.0
    laser_yaw: float = 0.0  # mettre pi si le LiDAR est monté "à l'envers"

    @property
    def length(self):
        return self.rear_overhang + self.front_overhang

    @property
    def max_curvature(self):
        return math.tan(self.max_steer) / self.wheelbase


@dataclass
class NavParams:
    # --- encombrement et sécurité -------------------------------------------
    safety_margin: float = 0.07  # marge de chaque côté du robot (configurable)
    # --- analyse des directions ---------------------------------------------
    heading_span: float = math.radians(100)  # on évalue θ dans [-span, +span]
    heading_step: float = math.radians(2)
    lookahead_cap: float = 2.5  # au-delà, une direction est considérée "très libre"
    min_passage_depth: float = 0.75  # profondeur libre mini pour qu'un passage soit traversable
    side_window: tuple = (0.15, 1.6)  # fenêtre de profondeur pour estimer les parois
    # --- score ---------------------------------------------------------------
    w_depth: float = 1.0
    w_side: float = 0.6
    w_forward: float = 0.35  # préférence pour continuer tout droit
    w_stability: float = 0.6  # pénalité de changement par rapport à θ précédent
    w_unknown: float = 0.4  # pénalité zones inconnues de la carte SLAM
    w_deadend: float = 1.5  # pénalité directions menant à une impasse connue
    side_cap: float = 0.35
    switch_margin: float = 0.08  # hystérésis entre passages
    # --- centrage et lissage -------------------------------------------------
    centering_gain: float = 0.8  # gain sur l'écart latéral (rad par m relatif)
    centering_deadband: float = 0.04  # m : pas de correction sous ce décalage
    theta_alpha: float = 0.35  # filtre passe-bas sur θ_cible
    theta_rate: float = math.radians(120)  # variation max de θ_cible (rad/s)
    # --- vitesses -------------------------------------------------------------
    v_max: float = 0.60
    v_min: float = 0.12
    v_narrow: float = 0.22  # vitesse max dans un passage étroit
    narrow_width: float = 0.55  # passage "étroit" si largeur dispo < cette valeur
    a_max: float = 0.8  # accélération max (m/s²)
    d_max: float = 1.6  # décélération max (m/s²)
    a_lat_max: float = 0.9  # accélération latérale max dans les virages
    turn_slowdown: float = 0.5  # réduction de vitesse proportionnelle à |θ_cible|
    stop_distance: float = 0.30  # distance d'arrêt devant un obstacle
    slow_distance: float = 1.4  # début du ralentissement
    emergency_distance: float = 0.18  # arrêt immédiat
    lookahead_min: float = 0.45  # pure pursuit L_d = lookahead_min + k*v
    lookahead_gain: float = 0.6
    # --- impasse / demi-tour --------------------------------------------------
    stop_before_turnaround: float = 0.6  # s
    reverse_speed: float = 0.15
    turnaround_max_cycles: int = 4
    rear_clearance_min: float = 0.18
    deadend_memory_radius: float = 0.8
    # --- sécurité capteurs -----------------------------------------------------
    scan_timeout: float = 0.5  # s sans /scan -> arrêt
    pose_timeout: float = 1.0  # s sans pose SLAM -> mode local, vitesse réduite
    degraded_speed_factor: float = 0.6


def update_dataclass(obj, values):
    """Copie dans `obj` les clés de `values` qui correspondent à un champ."""
    names = {f.name for f in fields(obj)}
    for k, v in values.items():
        if k in names:
            cur = getattr(obj, k)
            setattr(obj, k, type(cur)(v) if not isinstance(cur, tuple) else tuple(v))
    return obj
