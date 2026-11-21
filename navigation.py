"""Time-aware routing and movement for the arena."""

import heapq
import math
import time
from dataclasses import dataclass
from itertools import count


GRID = 5.0
NEIGHBORS = tuple(
    (dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy
)


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def direction(start, end, limit=1.0):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    scale = limit / length if length > 1e-6 else 0.0
    return dx * scale, dy * scale


class Zone:
    def __init__(self, state, now):
        self.now = now
        self.center = tuple(state["center"])
        self.radius = state["radius"]
        self.next_center = tuple(state["next_center"])
        self.next_radius = state["next_radius"]
        self.start = state["shrink_start"]
        self.end = state["shrink_end"]

    def circle(self, at):
        start = max(self.now, self.start)
        if at <= start or self.end <= start:
            return self.center, self.radius
        factor = min(1.0, (at - start) / (self.end - start))
        x, y = self.center
        nx, ny = self.next_center
        return (x + (nx - x) * factor, y + (ny - y) * factor), (
            self.radius + (self.next_radius - self.radius) * factor
        )

    def clearance(self, point, at):
        center, radius = self.circle(at)
        return radius - distance(center, point)

    def buffer(self, radius, desired):
        return min(desired, radius * 0.24)

    def in_next(self, point, margin=0.0):
        return distance(point, self.next_center) <= max(
            0.0, self.next_radius - self.buffer(self.next_radius, margin)
        ) + 1e-7

    def segment(self, start, end, at, duration):
        """Never knowingly leave the safe circle after the path enters it."""
        entered = self.clearance(start, at) >= -1e-7
        for i in range(1, 5):
            ratio = i / 4
            point = (start[0] + (end[0] - start[0]) * ratio,
                     start[1] + (end[1] - start[1]) * ratio)
            inside = self.clearance(point, at + duration * ratio) >= -1e-7
            if entered and not inside:
                return False
            entered |= inside
        return True


class Motion:
    def __init__(self, stats):
        self.radius = stats.get("radius", 2.5)
        self.height = stats.get("height", 9.0)
        self.step = stats.get("step_up", 1.2)
        self.slope = stats.get("max_slope_deg", 50.0)
        self.walk = stats.get("walk_speed", 11.7)
        self.boost = stats.get("boost_speed", 19.5)
        self.acceleration = stats.get("ground_accel", 58.5)
        self.air_acceleration = stats.get("air_accel", 12.0)
        self.jump = stats.get("jump_speed", 11.0)
        self.gravity = stats.get("gravity", 22.0)
        self.jet = stats.get("jet_accel", 34.0)
        self.rise = stats.get("max_rise_speed", 10.0)
        self.jet_drain = stats.get("jet_drain", 15.0)
        self.boost_drain = stats.get("boost_drain", 17.0)

    def shape(self):
        return self.radius, self.height, self.step, self.slope


class Search:
    def __init__(self, start, goals, started_at):
        self.start = start
        self.goals = goals
        self.started_at = started_at
        self.serial = count()
        self.queue = [(self.heuristic(start), next(self.serial), start, 0.0)]
        self.cost = {start: 0.0}
        self.traveled = {start: 0.0}
        self.previous = {}
        self.best = start
        self.complete = None
        self.exhausted = False

    def heuristic(self, point):
        return min(distance(point, goal) for goal in self.goals)

    def path(self, endpoint):
        path = [endpoint]
        while path[-1] in self.previous:
            path.append(self.previous[path[-1]])
        return list(reversed(path))


@dataclass
class Flight:
    takeoff: tuple
    landing: tuple
    apex: float
    climb: float
    seconds: float
    airborne: bool = False


class Navigator:
    def __init__(self, arena):
        self.arena = arena
        self.search = None
        self.role = None
        self.route = []
        self.edge_cache = {}
        self.shape = None

    def reset(self):
        self.search = None
        self.role = None
        self.route = []

    def support(self, point, motion, ceiling=math.inf):
        x, y = point[:2]
        try:
            z = self.arena.surface(x, y, ceiling)
            if z is None or not math.isfinite(z):
                return None
            if self.arena.body_clear(x, y, z, motion.radius, motion.height):
                return x, y, z
        except (ValueError, IndexError):
            pass
        return None

    def walkable(self, start, end, motion):
        shape = motion.shape()
        if shape != self.shape or len(self.edge_cache) > 14000:
            self.edge_cache.clear()
            self.shape = shape
        key = start, end
        if key not in self.edge_cache:
            self.edge_cache[key] = self.arena.walk_segment(start, end, *shape)
        return self.edge_cache[key]

    def _neighbors(self, point, motion):
        px, py = (round(v / GRID) for v in point[:2])
        for dx, dy in ((0, 0), *NEIGHBORS):
            xy = (px + dx) * GRID, (py + dy) * GRID
            if distance(point, xy) < 0.3:
                continue
            neighbor = self.support(xy, motion, point[2] + motion.step + 0.1)
            if (neighbor is not None and abs(neighbor[2] - point[2]) <= motion.step * 3
                    and self.walkable(point, neighbor, motion)):
                yield neighbor

    def _advance(self, search, zone, motion, deadline):
        for _ in range(65):
            if time.perf_counter() >= deadline or not search.queue:
                break
            _, _, point, old_cost = heapq.heappop(search.queue)
            if old_cost != search.cost[point]:
                continue
            at = max(zone.now, search.started_at + search.traveled[point] / motion.walk)
            direct = [(distance(point, goal), goal) for goal in search.goals]
            for length, goal in sorted(direct):
                if length > GRID * 1.55:
                    break
                if (zone.segment(point, goal, at, length / motion.walk)
                        and self.walkable(point, goal, motion)):
                    search.complete = search.path(point)
                    if point != goal:
                        search.complete.append(goal)
                    return
            for next_point in self._neighbors(point, motion):
                length = distance(point, next_point)
                duration = length / motion.walk
                if not zone.segment(point, next_point, at, duration):
                    continue
                clearance = zone.clearance(next_point, at + duration)
                exposure = min(8.0, max(0.0, -clearance) / 8)
                cost = old_cost + length * (1 + exposure)
                if cost >= search.cost.get(next_point, math.inf):
                    continue
                search.cost[next_point] = cost
                search.traveled[next_point] = search.traveled[point] + length
                search.previous[next_point] = point
                h = search.heuristic(next_point)
                if h + max(0, -clearance) < search.heuristic(search.best) + max(
                    0, -zone.clearance(search.best, at)
                ):
                    search.best = next_point
                heapq.heappush(search.queue, (cost + h, next(search.serial), next_point, cost))
        search.exhausted = not search.queue

    def path_time(self, start, path, speed):
        return sum(distance(a, b) for a, b in zip([start, *path], path)) / speed

    def path_safe(self, start, path, zone, speed, at=None):
        at = zone.now if at is None else at
        for end in path:
            duration = distance(start, end) / speed
            if not zone.segment(start, end, at, duration):
                return False
            start, at = end, at + duration
        return True

    def direct(self, start, end, zone, motion, speed=None):
        return (zone.segment(start, end, zone.now, distance(start, end) / (speed or motion.walk))
                and self.walkable(start, end, motion))

    def _usable(self, start, path, zone, motion):
        for i in range(min(5, len(path) - 1), -1, -1):
            point = path[i]
            if distance(start, point) > 12.0:
                continue
            if self.direct(start, point, zone, motion):
                return path[i:]
        return []

    def plan(self, start, goals, zone, motion, role, deadline):
        """Return validated waypoints, and whether a complete route was found."""
        if not goals:
            self.reset()
            return [], False
        if self.role != role:
            self.reset()
            self.role = role
        if self.route:
            self.route = self._usable(start, self.route, zone, motion)
            if self.route and self.path_safe(start, self.route, zone, motion.walk):
                return self.route, True
            self.route = []
            self.search = None
        for goal in goals[:5]:
            if time.perf_counter() >= deadline:
                break
            if distance(start, goal) <= 19 and self.direct(start, goal, zone, motion):
                self.route = [goal]
                return self.route, True
        search = self.search
        if (search is None or distance(start, search.start) > 17
                or (search.exhausted and zone.now - search.started_at > 1.0)):
            search = self.search = Search(start, tuple(goals), zone.now)
        self._advance(search, zone, motion, deadline)
        if search.complete:
            self.route = self._usable(start, search.complete, zone, motion)
            if self.route and self.path_safe(start, self.route, zone, motion.walk):
                return self.route, True
            self.search = None
        if search.best != search.start:
            return self._usable(start, search.path(search.best), zone, motion), False
        return [], False

    def _roof_targets(self, start, goals, zone, motion):
        min_x, min_y, max_x, max_y = self.arena.bounds
        for x, y in [zone.next_center, *[g[:2] for g in goals[:4]]]:
            for dx, dy in ((0, 0), *NEIGHBORS):
                xy = (x + 6 * dx, y + 6 * dy)
                if not (min_x < xy[0] < max_x and min_y < xy[1] < max_y):
                    continue
                if distance(start, xy) > 55 or not zone.in_next(xy):
                    continue
                point = self.support(xy, motion)
                if point and (point[2] >= self.arena.ground(*xy) + 2.0
                              or start[2] >= point[2] + 2.0):
                    yield point

    def flight_plan(self, start, goals, zone, motion, energy, maximum, deadline):
        ticks = 0
        for landing in self._roof_targets(start, goals, zone, motion):
            away = direction(landing, start)
            for shift in (0.0, 5.0, 10.0):
                if time.perf_counter() >= deadline or ticks >= 12:
                    return None
                ticks += 1
                xy = (start[0] + away[0] * shift, start[1] + away[1] * shift)
                takeoff = self.support(xy, motion, start[2] + motion.step)
                if takeoff is None or not self.direct(start, takeoff, zone, motion):
                    continue
                apex = max(takeoff[2], landing[2]) + 2.2
                up = max(0.3, (apex - takeoff[2] - 0.6) / max(1.0, motion.rise) + 0.22)
                across = distance(takeoff, landing) / max(1.0, motion.walk) + 0.35
                down = math.sqrt(2 * (apex - landing[2]) / motion.gravity) + 0.3
                seconds = up + across + down
                # Level flight needs intermittent thrust; budget a conservative full burn.
                if energy - (up + across) * motion.jet_drain < 0.2 * maximum:
                    continue
                at = zone.now + distance(start, takeoff) / motion.walk
                if at + seconds > zone.end and zone.next_radius > 0:
                    continue
                if not (zone.segment(takeoff, takeoff, at, up)
                        and zone.segment(takeoff, landing, at + up, across)
                        and zone.segment(landing, landing, at + up + across, down)):
                    continue
                top = (*takeoff[:2], apex)
                above_landing = (*landing[:2], apex)
                if all(self.arena.flight_segment(a, b, motion.radius, motion.height) for a, b in (
                        (takeoff, top), (top, above_landing), (above_landing, landing))):
                    return Flight(takeoff, landing, apex, up, seconds)
        return None

    def guard(self, move, boost, me, zone, motion, jet=False):
        """Screen the next command and its braking distance against the moving ring."""
        start = tuple(me["pos"])
        vx, vy = me["vel"][:2]
        air = jet or not me.get("grounded", True)
        speed = motion.boost if boost and not air else motion.walk
        acceleration = motion.air_acceleration if air else motion.acceleration
        target = move[0] * speed, move[1] * speed
        step = start[:2]
        for tick in range(2):
            dx, dy = target[0] - vx, target[1] - vy
            size = math.hypot(dx, dy)
            fraction = min(1.0, acceleration * 0.05 / size) if size else 0.0
            vx, vy = vx + dx * fraction, vy + dy * fraction
            following = step[0] + vx * 0.05, step[1] + vy * 0.05
            if not zone.segment(step, following, zone.now + tick * 0.05, 0.05):
                return False
            step = following
        stop_time = math.hypot(vx, vy) / max(1.0, acceleration)
        stop = step[0] + vx * stop_time / 2, step[1] + vy * stop_time / 2
        if not zone.segment(step, stop, zone.now + 0.1, stop_time):
            return False
        if air:
            vz = me["vel"][2]
            az = motion.jet - motion.gravity if boost else -motion.gravity
            z = start[2] + vz * 0.1 + az * 0.005
            surface = self.arena.surface(*step, start[2] + 0.05)
            if surface is not None and math.isfinite(surface):
                z = max(z, surface)
            return self.arena.flight_segment(
                start, (*step, z), motion.radius, motion.height
            )
        end = self.support(step, motion, start[2] + motion.step + 0.1)
        return end is not None and self.walkable(start, end, motion)
