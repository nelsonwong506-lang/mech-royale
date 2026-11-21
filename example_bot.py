"""A defense-first Mech Royale entrant."""

import math
import time

import mechfight as mf
from arena import Arena
from navigation import Motion, Navigator, Zone, direction, distance
from tactics import ThreatTracker, cover_quality, retreat_aim, return_fire, safe_cubes


SAMPLE_ANGLES = (0, 30, -30, 60, -60, 90, -90, 135, -135, 180)


class Bot:
    name = "circle-guard"

    def __init__(self, hello: mf.Hello):
        self.config = hello["config"]
        self.arena = Arena(hello["map"]["dir"], hello["map"].get("files", ()))
        self.nav = Navigator(self.arena)
        self.threats = ThreatTracker(self.config)
        self.flight = None
        self.consider_flight_at = -math.inf
        self.goals = []
        self.alternatives = []
        self.goal_key = None
        self.goal_deadline = -math.inf
        self.route_id = 0
        self.progress = None

    def _ground_goal(self, xy, motion):
        floor = self.arena.ground(*xy)
        return self.nav.support(xy, motion, floor + motion.step + 0.05)

    def _rotation_goals(self, start, zone, motion):
        x, y = zone.next_center
        if zone.in_next(start, motion.radius + 5):
            return [start]
        min_x, min_y, max_x, max_y = self.arena.bounds
        margin = motion.radius + 0.3
        safe_radius = max(0.0, zone.next_radius - zone.buffer(zone.next_radius, motion.radius + 6))
        angle = math.atan2(start[1] - y, start[0] - x)
        points = []
        offsets = (0, 24, -24, 50, -50, 85, -85, 135, -135, 180)
        for radius in (safe_radius, safe_radius * 0.66, safe_radius * 0.32, 0):
            for offset in offsets if radius else (0,):
                yaw = angle + math.radians(offset)
                xy = (min(max(x + radius * math.cos(yaw), min_x + margin), max_x - margin),
                      min(max(y + radius * math.sin(yaw), min_y + margin), max_y - margin))
                if zone.in_next(xy) and all(distance(xy, point) > 3 for point in points):
                    goal = self._ground_goal(xy, motion)
                    if goal:
                        points.append(goal)
        return sorted(points, key=lambda p: distance(p, start))[:16]

    def _retreat_goals(self, start, threat, zone, motion, rotation, deadline):
        if threat.position is None:
            return rotation, []
        position = threat.position
        protected = cover_quality(start, threat, self.arena.blocks_line, motion.height) == 3
        if protected and zone.in_next(start, motion.radius + 4):
            return [start], []
        away = math.atan2(start[1] - position[1], start[0] - position[0])
        initial_distance = distance(start, position)
        candidates = []
        for radius in (13.0, 23.0, 6.0):
            for offset in SAMPLE_ANGLES:
                if time.perf_counter() >= deadline:
                    break
                angle = away + math.radians(offset)
                xy = (start[0] + radius * math.cos(angle), start[1] + radius * math.sin(angle))
                if not zone.in_next(xy, min(motion.radius, 3)):
                    continue
                point = self.nav.support(xy, motion, start[2] + motion.step)
                if point is None or not zone.segment(start, point, zone.now, radius / motion.walk):
                    continue
                cover = cover_quality(point, threat, self.arena.blocks_line, motion.height)
                gain = distance(point, position) - initial_distance
                if gain < 1 and cover < 2:
                    continue
                candidates.append((cover, gain - 0.5 * radius, point))
        if not candidates:
            return rotation, []
        # Try hard cover first. A separate ground route handles solid walls between us and it.
        candidates.sort(reverse=True)
        best_cover = candidates[0][0]
        goals = [point for cover, _, point in candidates if cover == best_cover][:5]
        alternatives = [point for cover, _, point in candidates if cover < best_cover][:5]
        if not zone.in_next(start, motion.radius + 4):
            alternatives.extend(rotation[:3])
        return goals, alternatives

    def _get_goals(self, start, threat, zone, motion, deadline):
        role = (zone.start, zone.end)
        if threat is not None:
            role += (threat.actor_id, threat.hostile)
        live_goal = any(zone.in_next(point) for point in self.goals)
        if role == self.goal_key and zone.now < self.goal_deadline and (live_goal or not self.goals):
            return self.goals, self.route_id
        rotation = self._rotation_goals(start, zone, motion)
        goals, alternatives = (rotation, []) if threat is None else self._retreat_goals(
            start, threat, zone, motion, rotation, deadline
        )
        if goals == [start] or not self.goals or role != self.goal_key or not live_goal:
            self.goals = goals
            self.alternatives = alternatives
            self.route_id += 1
        elif goals and min(distance(old, new) for old in self.goals for new in goals) > 8:
            self.goals = goals
            self.alternatives = alternatives
            self.route_id += 1
        self.goal_key = role
        self.goal_deadline = zone.now + 0.7
        return self.goals, self.route_id

    def _pickup(self, obs, start, threat, zone, motion, route, complete):
        if not self.threats.can_collect_cubes(obs) or not obs["self"]["in_zone"]:
            return None
        if not complete and not zone.in_next(start, motion.radius + 4):
            return None
        pickup_radius = self.config.get("cubes", {}).get("pickup_radius", 3.5)
        cubes = safe_cubes(obs, threat, pickup_radius,
                           lambda x, y: zone.clearance((x, y), zone.now) > motion.radius)
        base = self.nav.path_time(start, route, motion.walk)
        for cube in cubes[:4]:
            x, y, z = cube["pos"]
            if distance(start, (x, y)) <= pickup_radius and abs(start[2] - z) <= 4:
                continue
            point = self.nav.support((x, y), motion, z + 4)
            if point is None or abs(point[2] - z) > 4:
                continue
            if not self.nav.direct(start, point, zone, motion):
                continue
            tail = [] if not route else route
            if tail and not self.nav.walkable(point, tail[0], motion):
                continue
            path = [point, *tail]
            total = self.nav.path_time(start, path, motion.walk)
            if total - base > 3 or not self.nav.path_safe(start, path, zone, motion.walk):
                continue
            if not zone.in_next(path[-1], motion.radius) or (
                not zone.in_next(start, motion.radius) and zone.now + total + 2 > zone.end
            ):
                continue
            return point
        return None

    def _recover_nearby(self, start, zone, motion, deadline):
        """Keep approaching the ring if no route can actually reach its inside."""
        center = zone.circle(zone.now + 0.8)[0]
        if zone.next_radius < zone.radius:
            center = zone.next_center
        yaw = math.atan2(center[1] - start[1], center[0] - start[0])
        best_distance = distance(start, center) - 0.1
        best = None
        for length in (7.0, 3.0):
            for offset in SAMPLE_ANGLES:
                if time.perf_counter() >= deadline:
                    return best
                angle = yaw + math.radians(offset)
                xy = start[0] + length * math.cos(angle), start[1] + length * math.sin(angle)
                closeness = distance(xy, center)
                if closeness >= best_distance:
                    continue
                point = self.nav.support(xy, motion, start[2] + motion.step)
                if point is not None and self.nav.direct(start, point, zone, motion):
                    best, best_distance = point, closeness
        return best

    def _ground_eta(self, start, route, me, motion):
        length = self.nav.path_time(start, route, motion.walk) * motion.walk
        usable = max(0.0, me["energy"] - 0.2 * me["max_energy"])
        boost_distance = min(length, motion.boost * usable / max(1.0, motion.boost_drain))
        return boost_distance / motion.boost + (length - boost_distance) / motion.walk

    def _monitor_progress(self, start, target, now):
        if target is None or distance(start, target) < 1:
            self.progress = None
            return
        if self.progress is None or distance(self.progress[0], target) > 2:
            self.progress = target, start, now
            return
        _, previous, at = self.progress
        if now - at < 1.0:
            return
        if distance(previous, target) - distance(start, target) < 0.75:
            self.nav.reset()
            self.goal_deadline = -math.inf
        self.progress = target, start, now

    def _ground_command(self, start, target, me, zone, motion, boost):
        move = (0.0, 0.0)
        if target is not None:
            magnitude = min(1.0, distance(start, target) / 1.3)
            move = direction(start, target, magnitude)
        if self.nav.guard(move, boost, me, zone, motion):
            return move, boost
        if boost and self.nav.guard(move, False, me, zone, motion):
            return move, False
        inward = direction(start, zone.circle(zone.now + 0.4)[0], 0.66)
        if self.nav.guard((0, 0), False, me, zone, motion):
            if target is None or distance(start, target) < 1.2:
                return (0.0, 0.0), False
            angle = math.atan2(move[1], move[0])
            for offset in (30, -30, 60, -60, 90, -90):
                theta = angle + math.radians(offset)
                adjusted = (0.5 * math.cos(theta), 0.5 * math.sin(theta))
                ahead = (start[0] + adjusted[0] * 3, start[1] + adjusted[1] * 3)
                if distance(ahead, target) >= distance(start, target) + 0.1:
                    continue
                if self.nav.guard(adjusted, False, me, zone, motion):
                    return adjusted, False
            return (0.0, 0.0), False
        if self.nav.guard(inward, False, me, zone, motion):
            return inward, False
        # When already committed by momentum, use the strongest feasible inward correction.
        correction = direction(start, zone.circle(zone.now + 0.4)[0])
        if self.nav.guard(correction, False, me, zone, motion):
            return correction, False
        return (0.0, 0.0), False

    def _fly(self, me, zone, motion):
        flight = self.flight
        start = tuple(me["pos"])
        max_energy = me["max_energy"]
        if me["grounded"]:
            if flight.airborne or me["energy"] <= 0.2 * max_energy:
                self.flight = None
                return None
            offset = distance(start, flight.takeoff)
            if offset > 1.0 or math.hypot(*me["vel"][:2]) > 2:
                move, boost = self._ground_command(start, flight.takeoff, me, zone, motion, False)
                return move, boost, False
            vertical = (*start[:2], flight.apex)
            if (not self.arena.flight_segment(start, vertical, motion.radius, motion.height)
                    or not zone.segment(start, start, zone.now, flight.climb)
                    or not zone.segment(start, flight.landing, zone.now + flight.climb,
                                        flight.seconds - flight.climb)):
                self.flight = None
                return None
            flight.airborne = True
            return (0.0, 0.0), False, True
        landing = flight.landing
        if not zone.segment(start, landing, zone.now, distance(start, landing) / motion.walk + 0.3):
            options = [zone.circle(zone.now + 0.6)[0], start[:2]]
            for xy in options:
                point = self.nav.support(xy, motion, start[2] + 0.1)
                if point and zone.segment(start, point, zone.now, distance(start, point) / motion.walk):
                    landing = flight.landing = point
                    break
        offset = distance(start, landing)
        desired = min(1.0, math.sqrt(2 * motion.air_acceleration * max(0, offset - 0.35)) / motion.walk)
        room_to_cross = start[2] >= flight.apex - 0.4
        over_landing = offset <= 1.1
        jet = (not over_landing and me["energy"] > 0.2 * max_energy + 0.2 * motion.jet_drain
               and start[2] + me["vel"][2] * 0.16 < flight.apex + 0.4)
        move = direction(start, landing, desired) if room_to_cross else (0.0, 0.0)
        if self.nav.guard(move, jet, me, zone, motion, jet=True):
            return move, jet, False
        return (0.0, 0.0), False, False

    def act(self, obs: mf.Observation) -> mf.Action:
        budget_start = time.perf_counter()
        me = obs["self"]
        start = tuple(me["pos"])
        now = obs["t"]
        zone = Zone(obs["zone"], now)
        motion = Motion(self.config.get("mech", {}) | me["stats"])
        threat = self.threats.update(obs, self.arena.blocks_line)
        action: mf.Action = {"tick": obs["tick"], "fire": [False, False]}
        if obs["offer"]:
            action["pick"] = 0

        command = self._fly(me, zone, motion) if self.flight else None
        if command is None:
            goals, role = self._get_goals(start, threat, zone, motion, budget_start + 0.009)
            route, complete = self.nav.plan(start, goals, zone, motion, role, budget_start + 0.025)
            if self.alternatives and self.nav.search is not None and self.nav.search.exhausted:
                goals = self.goals = self.alternatives
                self.alternatives = []
                self.route_id += 1
                route, complete = self.nav.plan(start, goals, zone, motion, self.route_id, budget_start + 0.026)
            remaining = max(0, zone.end - now)
            too_late = complete and self._ground_eta(start, route, me, motion) > remaining
            no_route = not goals or (self.nav.search is not None and self.nav.search.exhausted)
            if (me["grounded"] and now >= self.consider_flight_at and not zone.in_next(start)
                    and me["energy"] > me["max_energy"] * 0.35 and (no_route or too_late)):
                self.consider_flight_at = now + 0.7
                self.flight = self.nav.flight_plan(start, goals, zone, motion, me["energy"],
                                                  me["max_energy"], budget_start + 0.035)
                if self.flight:
                    command = self._fly(me, zone, motion)
            if command is None:
                pickup = self._pickup(obs, start, threat, zone, motion, route, complete)
                target = pickup or (route[0] if route else None)
                if target is None and not zone.in_next(start, motion.radius):
                    target = self._recover_nearby(start, zone, motion, budget_start + 0.033)
                self._monitor_progress(start, target, now)
                eta = self.nav.path_time(start, route, motion.walk)
                boost = (me["grounded"] and target is not None and not zone.in_next(start)
                         and (not me["in_zone"] or eta + 2 > remaining)
                         and me["energy"] > 0.2 * me["max_energy"] + motion.boost_drain * 0.2)
                move, boost = self._ground_command(start, target, me, zone, motion, boost)
                command = move, boost, False

        move, action["boost"], action["jump"] = command
        action["move"] = list(move)
        aim = retreat_aim(obs, threat)
        if aim is not None:
            action["aim"] = aim
        elif math.hypot(*move) > 0.1:
            action["aim"] = [math.atan2(move[1], move[0]), 0.0]
        if return_fire(obs, threat, move, lambda a, b: not self.arena.blocks_line(a, b)):
            gun_slot = next(i for i, weapon in enumerate(me["weapons"]) if weapon and weapon["kind"] == "gun")
            action["fire"][gun_slot] = True
        if threat is None:
            action["reload"] = any(weapon and weapon["kind"] == "gun" and weapon.get("ammo", 0) <= 3
                                   for weapon in me["weapons"])
        return action


if __name__ == "__main__":
    mf.run(Bot)
