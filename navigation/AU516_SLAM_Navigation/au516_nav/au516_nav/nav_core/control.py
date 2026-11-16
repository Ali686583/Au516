"""Transformation θ_cible -> (v, ω) et cinématique du véhicule."""

import math


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def twist_to_ackermann(v, omega, vehicle, last_delta=0.0):
    """Modèle bicyclette : ω = v·tan(δ)/L  ->  δ = atan(L·ω / v)."""
    if abs(v) < 1e-3:
        return v, last_delta
    delta = math.atan(vehicle.wheelbase * omega / v)
    return v, clamp(delta, -vehicle.max_steer, vehicle.max_steer)


def ackermann_to_twist(v, delta, vehicle):
    return v, v * math.tan(delta) / vehicle.wheelbase


def differential_wheel_speeds(v, omega, track):
    """Diagnostic : vitesses idéales des roues arrière (V_G = v - ωL/2, V_D = v + ωL/2).

    Sur la voiture AU516 les deux moteurs reçoivent la même consigne ; ces valeurs
    servent à visualiser le comportement attendu (gauche : V_G < V_D, droite : V_G > V_D).
    """
    return v - omega * track / 2.0, v + omega * track / 2.0


class SpeedController:
    """Pure pursuit sur la direction θ_cible + limitations de vitesse et d'accélération."""

    def __init__(self, vehicle, params):
        self.veh, self.p = vehicle, params
        self.v = 0.0

    def reset(self):
        self.v = 0.0

    def compute(self, theta, free_depth, front_clear, narrow, dt, speed_factor=1.0):
        p, veh = self.p, self.veh
        # 1) courbure : point visé à L_d dans la direction θ (pure pursuit)
        ld = p.lookahead_min + p.lookahead_gain * abs(self.v)
        kappa = clamp(2.0 * math.sin(theta) / ld, -veh.max_curvature, veh.max_curvature)

        # 2) vitesse cible : obstacles, virages, passage étroit
        d = min(free_depth, front_clear + 0.3)
        f_obst = clamp((d - p.stop_distance) / (p.slow_distance - p.stop_distance), 0.0, 1.0)
        v_curve = math.sqrt(p.a_lat_max / abs(kappa)) if abs(kappa) > 1e-6 else p.v_max
        f_turn = 1.0 - p.turn_slowdown * min(1.0, abs(theta) / (math.pi / 3))
        v_t = min(p.v_max * f_obst * f_turn, v_curve) * speed_factor
        if narrow:
            v_t = min(v_t, p.v_narrow)
        if f_obst > 0.0:
            v_t = max(v_t, min(p.v_min, p.v_max * speed_factor))
        if front_clear < p.emergency_distance:
            v_t = 0.0  # arrêt d'urgence logique

        # 3) limitation d'accélération
        dv = clamp(v_t - self.v, -p.d_max * dt, p.a_max * dt)
        if v_t == 0.0 and front_clear < p.emergency_distance:
            self.v = 0.0
        else:
            self.v = max(0.0, self.v + dv)
        return self.v, self.v * kappa, kappa
