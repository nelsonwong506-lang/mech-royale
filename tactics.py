"""Survival decisions based on the observations available to the bot."""

from __future__ import annotations

import math
from dataclasses import dataclass


Point3 = tuple[float, float, float]
RETALIATE_SECONDS = 3.0
CUBE_DAMAGE_QUIET_SECONDS = 5.0
PROJECTILE_LOOKAHEAD = 0.4
CUBE_DETOUR_METERS = 20.0


@dataclass(frozen=True)
class Threat:
    position: Point3 | None
    actor_id: int | None
    hostile: bool
    attack_position: Point3 | None = None
    attack_id: int | None = None
    attack_velocity: Point3 | None = None
    attack_radius: float = 2.5
    attack_height: float = 9.0


def _point(values) -> Point3:
    return values[0], values[1], values[2]


def _above(position, height) -> Point3:
    return position[0], position[1], position[2] + height


def _distance_xy(first, second) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def _angles(origin, target) -> list[float]:
    dx, dy, dz = (target[i] - origin[i] for i in range(3))
    return [math.atan2(dy, dx), math.atan2(dz, math.hypot(dx, dy))]


def _possible_shooters(projectile, opponents, mech, size_per_cube, gravity, blocked):
    """Projectiles have no owner field. Only a unique geometric match returns fire."""
    pos = projectile["pos"]
    vx, vy, vz = projectile["vel"]
    kind = projectile["kind"]
    matches = []
    for actor in opponents.values():
        aim = actor.get("aim")
        if aim is None or kind not in actor.get("weapons", ()):
            continue
        ax, ay, az = actor["pos"]
        avx, avy, avz = actor["vel"]
        dx, dy = pos[0] - ax, pos[1] - ay
        rvx, rvy = vx - avx, vy - avy
        speed_sq = rvx * rvx + rvy * rvy
        if speed_sq < 1:
            continue
        age = (dx * rvx + dy * rvy) / speed_sq
        if not 0 <= age <= (0.9 if kind == "gun" else 1.5):
            continue
        scale = 1 + size_per_cube * actor.get("cubes", 0)
        muzzle = mech.get("muzzle_height", 7) if kind == "gun" else mech.get("height", 9) + 0.5
        height_at_launch = pos[2] - (vz - avz) * age - gravity * age * age / 2
        error = math.hypot(dx - age * rvx, dy - age * rvy, height_at_launch - az - muzzle * scale)
        if error > mech.get("radius", 2.5) * scale + 1.5:
            continue
        launch_yaw = math.atan2(vy, vx)
        launch_pitch = math.atan2(vz + gravity * age, math.hypot(vx, vy))
        yaw_change = abs(math.remainder(launch_yaw - aim[0], math.tau))
        pitch_change = abs(launch_pitch - aim[1])
        if yaw_change > min(0.8, 0.15 + mech.get("turn_rate", 4) * age):
            continue
        if pitch_change > min(0.7, 0.15 + mech.get("pitch_rate", 3) * age):
            continue
        origin = (ax - avx * age, ay - avy * age, az - avz * age + muzzle * scale)
        if kind == "gun" and blocked and blocked(origin, _point(pos)):
            continue
        matches.append(actor["id"])
    return matches


def _imminent_projectile(me, projectile, gravity, blocked) -> float | None:
    pos = projectile["pos"]
    vx, vy, vz = projectile["vel"]
    mx, my, mz = me["pos"]
    mvx, mvy, mvz = me["vel"]
    dx, dy = pos[0] - mx, pos[1] - my
    rvx, rvy = vx - mvx, vy - mvy
    speed_sq = rvx * rvx + rvy * rvy
    if speed_sq < 1:
        return None
    time_to_me = -(dx * rvx + dy * rvy) / speed_sq
    if not 0 < time_to_me <= PROJECTILE_LOOKAHEAD:
        return None
    stats = me.get("stats", {})
    radius = stats.get("radius", 2.5) + 0.5
    if math.hypot(dx + rvx * time_to_me, dy + rvy * time_to_me) > radius:
        return None
    shell = (pos[0] + vx * time_to_me, pos[1] + vy * time_to_me,
             pos[2] + vz * time_to_me - gravity * time_to_me**2 / 2)
    feet = mz + mvz * time_to_me
    if not feet - 0.5 <= shell[2] <= feet + stats.get("height", 9) + 0.5:
        return None
    if blocked and blocked(_point(pos), shell):
        return None
    return time_to_me


class ThreatTracker:
    def __init__(self, config=None):
        self.config = config or {}
        self.attacked_at: dict[int, float] = {}
        self.last_attacker_pos: dict[int, Point3] = {}
        self.unknown_position: Point3 | None = None
        self.unknown_until = -math.inf
        self.last_damage_at = -math.inf

    def can_collect_cubes(self, obs) -> bool:
        return obs.get("t", obs["tick"] / 20) - self.last_damage_at >= CUBE_DAMAGE_QUIET_SECONDS

    def update(self, obs, blocked=None) -> Threat | None:
        me = obs["self"]
        now = obs.get("t", obs["tick"] / 20)
        enemies = {actor["id"]: actor for actor in obs.get("mechs", ()) if actor["kind"] == "mech"}
        for actor_id, time in tuple(self.attacked_at.items()):
            if now - time >= RETALIATE_SECONDS:
                del self.attacked_at[actor_id]
                self.last_attacker_pos.pop(actor_id, None)

        for event in obs.get("events", ()):
            if event["type"] != "damaged":
                continue
            self.last_damage_at = now
            actor_id = event.get("by")
            if actor_id is None or not 0 <= actor_id < 16:
                continue
            if event.get("weapon") in ("zone", "npc") or actor_id == me.get("id"):
                continue
            self.attacked_at[actor_id] = now
            if actor_id not in enemies and event.get("bearing") is not None:
                bearing = event["bearing"]
                x, y, z = me["pos"]
                self.last_attacker_pos[actor_id] = (x + 40 * math.cos(bearing), y + 40 * math.sin(bearing), z)

        mech = self.config.get("mech", {})
        mortar = self.config.get("weapons", {}).get("mortar", {})
        size_per_cube = self.config.get("cubes", {}).get("size_per_cube", 0.05)
        soonest = math.inf
        imminent = None
        for projectile in obs.get("projectiles", ()):
            kind = projectile["kind"]
            if kind not in ("gun", "mortar"):
                continue
            gravity = mortar.get("gravity", 22) if kind == "mortar" else 0
            time = _imminent_projectile(me, projectile, gravity, blocked)
            if time is not None and time < soonest:
                soonest, imminent = time, projectile
        if imminent is not None:
            gravity = mortar.get("gravity", 22) if imminent["kind"] == "mortar" else 0
            matches = _possible_shooters(imminent, enemies, mech, size_per_cube, gravity, blocked)
            if len(matches) == 1:
                self.attacked_at[matches[0]] = now
            else:
                x, y, z = imminent["pos"]
                vx, vy, _ = imminent["vel"]
                speed = max(math.hypot(vx, vy), 1)
                self.unknown_position = (x - 40 * vx / speed, y - 40 * vy / speed, z)
                self.unknown_until = now + 1.0

        for actor_id in self.attacked_at.keys() & enemies.keys():
            self.last_attacker_pos[actor_id] = _point(enemies[actor_id]["pos"])
        if self.attacked_at:
            attack_id = min(self.attacked_at, key=lambda i: (i not in enemies, -self.attacked_at[i], i))
            actor = enemies.get(attack_id)
            attack_pos = _point(actor["pos"]) if actor else None
            scale = 1 + size_per_cube * (actor.get("cubes", 0) if actor else 0)
            position = self.last_attacker_pos.get(attack_id)
            return Threat(position, attack_id, True, attack_pos, attack_id,
                          _point(actor["vel"]) if actor else None,
                          mech.get("radius", 2.5) * scale, mech.get("height", 9) * scale)

        if now < self.unknown_until:
            return Threat(self.unknown_position, None, True)
        if enemies:
            nearest = min(enemies.values(), key=lambda actor: (_distance_xy(me["pos"], actor["pos"]), actor["id"]))
            return Threat(_point(nearest["pos"]), nearest["id"], False)
        return None


def _gun(me):
    return next((weapon for weapon in me.get("weapons", ()) if weapon and weapon["kind"] == "gun"), None)


def _shot_points(me, threat):
    origin = _above(me["pos"], me.get("stats", {}).get("muzzle_height", 7))
    target = _above(threat.attack_position, threat.attack_height / 2)
    gun = _gun(me)
    speed = gun.get("stats", {}).get("speed", 252) if gun else 252
    velocity = threat.attack_velocity or (0, 0, 0)
    # Refine the intercept estimate twice for a moving target.
    intercept = target
    for _ in range(2):
        flight_time = math.dist(origin, intercept) / max(speed, 1)
        intercept = tuple(target[i] + velocity[i] * flight_time for i in range(3))
    return origin, target, intercept


def retreat_aim(obs, threat: Threat | None) -> list[float] | None:
    if threat is None or threat.position is None:
        return None
    me = obs["self"]
    if threat.attack_position is not None:
        origin, _, intercept = _shot_points(me, threat)
        return _angles(origin, intercept)
    return _angles(_above(me["pos"], me.get("stats", {}).get("muzzle_height", 7)), _above(threat.position, 4.5))


def return_fire(obs, threat: Threat | None, move, clear_shot) -> bool:
    if threat is None or not threat.hostile or threat.attack_id is None or threat.attack_position is None:
        return False
    me = obs["self"]
    gun = _gun(me)
    if gun is None or gun.get("cooldown", 0) > 0.1 or gun.get("reload", 0) > 0 or gun.get("ammo", 1) <= 0:
        return False
    dx, dy = (threat.attack_position[i] - me["pos"][i] for i in range(2))
    # A cover move may be lateral, but closing on the attacker is not a withdrawal.
    if math.hypot(*move) < 0.15 or dx * move[0] + dy * move[1] > 0.05 * math.hypot(dx, dy):
        return False
    origin, target, intercept = _shot_points(me, threat)
    distance = math.dist(origin, intercept)
    if distance > gun.get("stats", {}).get("range", 230):
        return False
    yaw, pitch = _angles(origin, intercept)
    actual_yaw, actual_pitch = me["aim"]
    yaw_error = abs(math.remainder(yaw - actual_yaw, math.tau))
    pitch_error = abs(pitch - actual_pitch)
    # Keep the shot inside a conservative part of the mech's projected hitbox.
    tolerance = min(0.18, math.atan2(0.75 * threat.attack_radius, max(distance, 1)))
    if math.hypot(yaw_error * math.cos(pitch), pitch_error) > tolerance:
        return False
    actual_end = (origin[0] + distance * math.cos(actual_pitch) * math.cos(actual_yaw),
                  origin[1] + distance * math.cos(actual_pitch) * math.sin(actual_yaw),
                  origin[2] + distance * math.sin(actual_pitch))
    return clear_shot(origin, target) and clear_shot(origin, intercept) and clear_shot(origin, actual_end)


def safe_cubes(obs, threat: Threat | None, pickup_radius: float, safe_point) -> list[dict]:
    """Filter nearby cubes. The route planner vets walking height and the detour."""
    if threat is not None or any(event["type"] == "damaged" for event in obs.get("events", ())):
        return []
    position = obs["self"]["pos"]
    if any(actor["kind"] == "npc" and _distance_xy(actor["pos"], position) < 45
           for actor in obs.get("mechs", ())):
        return []
    cubes = []
    for cube in obs.get("cubes", ()):
        x, y = cube["pos"][:2]
        distance = _distance_xy(cube["pos"], position)
        if distance <= CUBE_DETOUR_METERS + pickup_radius and safe_point(x, y):
            cubes.append((distance, cube["id"], cube))
    cubes.sort()
    return [cube for _, _, cube in cubes]


def cover_quality(position, threat: Threat | None, blocked, height=9.0) -> int:
    """Score a candidate by how much of the mech body the solid geometry shields."""
    if threat is None or threat.position is None:
        return 0
    source = _above(threat.position, threat.attack_height * 0.8)
    return sum(blocked(source, _above(position, height * fraction)) for fraction in (0.2, 0.5, 0.9))
