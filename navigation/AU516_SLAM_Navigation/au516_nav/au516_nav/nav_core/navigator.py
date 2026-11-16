"""Navigation locale:passages traversables -> θ_cible -> (v, ω), plus gestion des impasses.

Ce module ne dépend pas de ROS:il est testé tel quel par le simulateur 2D (tests/).
"""

from dataclasses import dataclass, field
import math
import numpy as np

from .geometry import analyse_strips, front_clearance, rear_clearance
from .control import SpeedController, clamp

DRIVE, STOPPING, TURN_REVERSE, TURN_FORWARD, STOPPED = (
    'DRIVE',
    'STOPPING',
    'TURN_REVERSE',
    'TURN_FORWARD',
    'STOPPED',
)


@dataclass
class Passage:
    lo: float  # bornes angulaires (rad, repère robot)
    hi: float
    direction: float  # meilleure direction dans le passage
    depth: float  # dégagement libre sur la profondeur observable
    width: float  # largeur disponible estimée
    distance: float  # distance de l'entrée du passage (bord d'obstacle le plus proche)
    side_min: float  # plus petite distance latérale à la bande du robot
    risk: float  # 0 = sûr, 1 = collision probable
    unknown: float  # fraction de cases inconnues sur la carte SLAM
    deadend: float  # 0..1 proximité d'une impasse connue
    score: float
    feasible: bool = True


@dataclass
class NavOutput:
    v: float
    omega: float
    theta: float
    state: str
    passages: list = field(default_factory=list)
    selected: Passage = None
    narrow: bool = False
    info: dict = field(default_factory=dict)


class LocalNavigator:
    def __init__(self, vehicle, params, deadends=None):
        self.veh, self.p = vehicle, params
        self.ctrl = SpeedController(vehicle, params)
        self.deadends = deadends
        self.theta = 0.0  # θ_cible filtré
        self.state = DRIVE
        self.timer = 0.0
        self.turn_yaw = 0.0  # cap intégré pendant le demi-tour
        self.turn_side = 1.0
        self.cycles = 0
        self.prev_sel = None
        self.stall = 0.0
        self.narrow_hold = 0.0
        self.moved = 0.0

    # ------------------------------------------------------------------ passages
    def find_passages(self, strips, pose=None, gridmap=None):
        p, veh = self.p, self.veh
        H = strips.headings
        depth = strips.depth.copy()
        unknown = np.zeros_like(depth)
        dead = np.zeros_like(depth)
        if pose is not None:
            x, y, yaw = pose
            fx, fy = x + veh.wheelbase * 0.5 * math.cos(yaw), y + veh.wheelbase * 0.5 * math.sin(
                yaw
            )
            reach = np.minimum(depth, p.lookahead_cap)
            if gridmap is not None:
                occ, unknown = gridmap.rays_stats(fx, fy, yaw + H, reach)
                depth = np.minimum(depth, occ)  # obstacle connu sur la carte
            if self.deadends is not None and self.deadends.points:
                ex, ey = fx + reach * np.cos(yaw + H), fy + reach * np.sin(yaw + H)
                dead = np.array([self.deadends.penalty(a, b) for a, b in zip(ex, ey)])

        kappa_needed = (
            2.0 * np.abs(np.sin(H)) / np.maximum(np.minimum(depth, p.lookahead_cap), 1e-3)
        )
        free = (depth >= p.min_passage_depth) & (kappa_needed <= veh.max_curvature)

        side_min = np.minimum(strips.left_min, strips.right_min)
        span = p.heading_span
        score = (
            p.w_depth * np.minimum(depth, p.lookahead_cap) / p.lookahead_cap
            + p.w_side * np.minimum(side_min, p.side_cap) / p.side_cap
            - p.w_forward * np.abs(H) / span
            - p.w_stability * np.abs(H - self.theta) / span
            - p.w_unknown * unknown
            - p.w_deadend * dead
        )

        passages = []
        dcap = np.minimum(depth, p.lookahead_cap * 1.6)
        n = H.size
        for i, j in _free_runs(free):
            for a, b in _split_run(dcap, i, j):  # une vallée de profondeur sépare 2 passages
                seg = dcap[a:b + 1]
                core = np.nonzero(seg >= 0.8 * seg.max())[0]  # partie profonde du passage
                k = a + int(core[np.argmax(score[a:b + 1][core])])
                border = [depth[m] for m in (a - 1, b + 1) if 0 <= m < n]
                width = float(
                    strips.left_min[k] + strips.right_min[k]
                )  # largeur dispo la plus faible devant
                sm = float(side_min[k])
                passages.append(
                    Passage(
                        lo=float(H[a]),
                        hi=float(H[b]),
                        direction=float(H[k]),
                        depth=float(depth[k]),
                        width=width,
                        distance=float(min(border)) if border else 0.0,
                        side_min=sm,
                        risk=float(1.0 - clamp(sm / (3 * p.safety_margin), 0.0, 1.0)),
                        unknown=float(unknown[k]),
                        deadend=float(dead[k]),
                        score=float(score[k]),
                    )
                )
        return passages

    def select(self, passages):
        if not passages:
            return None
        best = max(passages, key=lambda q: q.score)
        if self.prev_sel is not None:
            for q in passages:  # hystérésis:on garde le passage courant s'il reste bon
                if (
                    q.lo - 0.05 <= self.prev_sel <= q.hi + 0.05
                    and q.score >= best.score - self.p.switch_margin
                ):
                    return q
        return best

    # ------------------------------------------------------------------ boucle
    def step(self, points, dt, pose=None, gridmap=None, speed_factor=1.0):
        p, veh = self.p, self.veh
        strips = analyse_strips(points, veh, p)
        passages = self.find_passages(strips, pose, gridmap)
        fc = front_clearance(points, veh, p)
        rc = rear_clearance(points, veh)
        mid = int(np.argmin(np.abs(strips.headings)))
        info = {
            'front_clear': fc,
            'rear_clear': rc,
            'left_space': float(strips.left[mid]),
            'right_space': float(strips.right[mid]),
        }

        if self.state in (TURN_REVERSE, TURN_FORWARD):
            return self._turnaround(points, passages, fc, rc, dt, info)

        sel = self.select(passages)
        if sel is None or self.state == STOPPED:
            return self._no_passage(passages, fc, rc, dt, pose, info)

        # θ brut = direction du passage + recentrage progressif (zone morte anti-oscillation)
        k = int(np.argmin(np.abs(strips.headings - sel.direction)))
        e = (strips.left[k] - strips.right[k]) / 2.0
        e = math.copysign(max(0.0, abs(e) - p.centering_deadband), e)
        raw = sel.direction + math.atan(p.centering_gain * e)
        raw = clamp(raw, sel.lo, sel.hi)
        dtheta = clamp(p.theta_alpha * (raw - self.theta), -p.theta_rate * dt, p.theta_rate * dt)
        self.theta += dtheta
        self.prev_sel = sel.direction

        # Le LiDAR est à l'avant:on garde la vitesse réduite jusqu'à ce que l'arrière soit sorti
        if sel.width < p.narrow_width:
            self.narrow_hold = self.veh.length + 0.1
        narrow = self.narrow_hold > 0.0
        self.narrow_hold = max(0.0, self.narrow_hold - self.ctrl.v * dt)
        v, omega, kappa = self.ctrl.compute(self.theta, sel.depth, fc, narrow, dt, speed_factor)
        self.state = DRIVE
        self.moved += v * dt
        if self.moved > 1.0:
            self.cycles = 0  # on a vraiment repris la route
        self.stall = self.stall + dt if v < 1e-3 else 0.0
        if self.stall > self.p.stop_before_turnaround:
            # bloqué malgré un passage apparent (ex. trop serré pour braquer):impasse
            self.stall, self.state, self.timer = 0.0, STOPPING, self.p.stop_before_turnaround
            return self._no_passage(passages, fc, rc, dt, pose, info)
        info.update(raw_theta=raw, kappa=kappa, lateral_error=e)
        return NavOutput(v, omega, self.theta, self.state, passages, sel, narrow, info)

    # ------------------------------------------------------------------ cas dégradés
    def _no_passage(self, passages, fc, rc, dt, pose, info):
        """Obstacle frontal sans alternative, ou impasse:arrêt puis demi-tour si possible."""
        v, omega, _ = self.ctrl.compute(self.theta, 0.0, fc, False, dt)
        if self.state == DRIVE:
            self.state, self.timer = STOPPING, 0.0
        if v < 1e-3:
            self.ctrl.v = 0.0
            self.timer += dt
            if self.state == STOPPING and self.timer >= self.p.stop_before_turnaround:
                if pose is not None and self.deadends is not None:
                    x, y, yaw = pose
                    self.deadends.add(x + 0.5 * math.cos(yaw), y + 0.5 * math.sin(yaw))
                if rc > self.p.rear_clearance_min and self.cycles < self.p.turnaround_max_cycles:
                    self._start_turn(info)
                else:
                    self.state = STOPPED
        if self.state == STOPPED:
            self.timer += dt
            # on retente seulement si un passage dégagé réapparaît (obstacle enlevé)
            if (
                passages
                and max(q.depth for q in passages) > 2 * self.p.min_passage_depth
                and fc > self.p.stop_distance
            ):
                self.state = DRIVE
        return NavOutput(v, omega, self.theta, self.state, passages, None, False, info)

    def _start_turn(self, info):
        self.moved = 0.0
        # côté le plus dégagé:on fera tourner le nez de ce côté
        self.turn_side = (
            1.0 if info.get('left_space', 0.0) >= info.get('right_space', 0.0) else -1.0
        )
        self.state, self.timer, self.turn_yaw = TURN_REVERSE, 0.0, 0.0
        self.cycles += 1

    def _turnaround(self, points, passages, fc, rc, dt, info):
        """Demi-tour en plusieurs manœuvres (voiture Ackermann:pas de rotation sur place)."""
        p, veh = self.p, self.veh
        self.timer += dt
        d = veh.max_steer
        if self.state == TURN_REVERSE:
            v, delta = (
                -p.reverse_speed,
                -self.turn_side * d,
            )  # en marche arrière, braquer à l'opposé
            blocked = _corner_blocked(points, veh, rear=True, side=-self.turn_side)
            if (
                rc < p.rear_clearance_min
                or blocked
                or self.timer > 4.0
                or abs(self.turn_yaw) > math.radians(100)
            ):
                self.state, self.timer = TURN_FORWARD, 0.0
                v = 0.0
        else:
            v, delta = p.reverse_speed * 1.3, self.turn_side * d
            exit_ok = [
                q for q in passages if abs(q.direction) < math.radians(40) and q.deadend < 0.5
            ]
            if abs(self.turn_yaw) > math.radians(150) and exit_ok:
                self.state = DRIVE
                self.theta = 0.0
                self.ctrl.reset()
                v = 0.0
            elif (
                fc < p.stop_distance * 0.8
                or self.timer > 5.0
                or _corner_blocked(points, veh, rear=False, side=self.turn_side)
            ):
                if self.cycles >= p.turnaround_max_cycles:
                    self.state = STOPPED
                else:
                    self.state, self.timer = TURN_REVERSE, 0.0
                    self.cycles += 1
                v = 0.0
        omega = v * math.tan(delta) / veh.wheelbase
        self.turn_yaw += omega * dt
        info.update(turn_yaw=self.turn_yaw, cycles=self.cycles)
        return NavOutput(v, omega, self.theta, self.state, passages, None, False, info)


def _free_runs(free):
    runs, i, n = [], 0, free.size
    while i < n:
        if free[i]:
            j = i
            while j + 1 < n and free[j + 1]:
                j += 1
            runs.append((i, j))
            i = j + 1
        else:
            i += 1
    return runs


def _split_run(d, i, j, ratio=0.7):
    """Découpe récursive d'une plage libre au niveau des creux de profondeur marqués."""
    if j - i < 4:
        return [(i, j)]
    m = i + 2 + int(np.argmin(d[i + 2:j - 1]))
    lp, rp = d[i:m].max(), d[m + 1:j + 1].max()
    if d[m] < ratio * min(lp, rp):
        return _split_run(d, i, m - 1, ratio) + _split_run(d, m + 1, j, ratio)
    return [(i, j)]


def _corner_blocked(points, veh, rear, side, margin=0.10):
    """Vrai si un obstacle est trop proche du coin de la voiture qui balaie l'espace."""
    if points.shape[0] == 0:
        return False
    cx = -veh.rear_overhang if rear else veh.front_overhang
    cy = side * veh.width / 2
    d = np.hypot(points[:, 0] - cx, points[:, 1] - cy)
    zone = (points[:, 0] < cx + 0.05) if rear else (points[:, 0] > cx - 0.05)
    return bool(np.any(zone & (d < margin)))
