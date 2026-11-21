"""Read-only geometry for the Mech Royale arena's terrain and solid prisms."""

import json
import math
import sys
from array import array
from pathlib import Path


_BIN_SIZE = 24.0
_SUPPORT_EPSILON = 0.08
_SAMPLE_DISTANCE = 1.2


def _ring_edges(coords):
    points = list(zip(coords[::2], coords[1::2]))
    edges = []
    for (x, y), (end_x, end_y) in zip(points, points[1:] + points[:1]):
        dx, dy = end_x - x, end_y - y
        length_sq = dx * dx + dy * dy
        if length_sq:
            edges.append((x, y, dx, dy, 1.0 / length_sq))
    return edges


def _inside_ring(edges, x, y):
    inside = False
    for edge_x, edge_y, dx, dy, _ in edges:
        if (edge_y > y) != (edge_y + dy > y) and x < edge_x + (y - edge_y) * dx / dy:
            inside = not inside
    return inside


def _ring_near_point(edges, x, y, radius_sq):
    for edge_x, edge_y, dx, dy, inverse_length_sq in edges:
        relative_x, relative_y = x - edge_x, y - edge_y
        fraction = (relative_x * dx + relative_y * dy) * inverse_length_sq
        fraction = max(0.0, min(1.0, fraction))
        gap_x = relative_x - fraction * dx
        gap_y = relative_y - fraction * dy
        if gap_x * gap_x + gap_y * gap_y <= radius_sq:
            return True
    return False


def _crosses_ring(edges, start_x, start_y, dx, dy):
    for edge_x, edge_y, edge_dx, edge_dy, _ in edges:
        offset_x, offset_y = edge_x - start_x, edge_y - start_y
        determinant = dx * edge_dy - dy * edge_dx
        if abs(determinant) <= 1e-9:
            if abs(offset_x * dy - offset_y * dx) > 1e-9:
                continue
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-12:
                continue
            first = (offset_x * dx + offset_y * dy) / length_sq
            last = first + (edge_dx * dx + edge_dy * dy) / length_sq
            if max(first, last) >= 0 and min(first, last) <= 1:
                return True
            continue
        t = (offset_x * edge_dy - offset_y * edge_dx) / determinant
        u = (offset_x * dy - offset_y * dx) / determinant
        if 0 <= t <= 1 and 0 <= u <= 1:
            return True
    return False


class _Prism:
    def __init__(self, definition):
        ring = definition["ring"]
        xs, ys = ring[::2], ring[1::2]
        self.min_x, self.max_x = min(xs), max(xs)
        self.min_y, self.max_y = min(ys), max(ys)
        self.base, self.top = definition["base"], definition["top"]
        self.outer = _ring_edges(ring)
        self.holes = [_ring_edges(hole) for hole in definition.get("holes", [])]
        self.edges = [*self.outer, *(edge for hole in self.holes for edge in hole)]

    def contains(self, x, y):
        return (
            self.min_x <= x <= self.max_x
            and self.min_y <= y <= self.max_y
            and _inside_ring(self.outer, x, y)
            and not any(_inside_ring(hole, x, y) for hole in self.holes)
        )

    def touches(self, x, y, radius):
        if not (
            self.min_x - radius <= x <= self.max_x + radius
            and self.min_y - radius <= y <= self.max_y + radius
        ):
            return False
        return self.contains(x, y) or _ring_near_point(self.edges, x, y, radius * radius)

    def blocks_line(self, start, end):
        dz = end[2] - start[2]
        if abs(dz) < 1e-9:
            if not self.base <= start[2] <= self.top:
                return False
            first, last = 0.0, 1.0
        else:
            entry, exit = sorted(((self.base - start[2]) / dz, (self.top - start[2]) / dz))
            first, last = max(0.0, entry), min(1.0, exit)
            if first > last:
                return False

        dx, dy = end[0] - start[0], end[1] - start[1]
        entry_x, exit_x = start[0] + dx * first, start[0] + dx * last
        entry_y, exit_y = start[1] + dy * first, start[1] + dy * last
        if (
            max(entry_x, exit_x) < self.min_x or min(entry_x, exit_x) > self.max_x
            or max(entry_y, exit_y) < self.min_y or min(entry_y, exit_y) > self.max_y
        ):
            return False
        return (
            self.contains(entry_x, entry_y)
            or self.contains(exit_x, exit_y)
            or _crosses_ring(self.edges, entry_x, entry_y, exit_x - entry_x, exit_y - entry_y)
        )


class Arena:
    """Arena coordinates are in meters, with z representing ground or feet height."""

    def __init__(self, map_dir, files=()):
        path = Path(map_dir)
        world_path = path / next((name for name in files if Path(name).name == "world.json"), "v2/world.json")
        world = json.loads(world_path.read_text(encoding="utf-8"))
        bounds = world["bounds"]
        self.bounds = (bounds["minX"], bounds["minY"], bounds["maxX"], bounds["maxY"])

        terrain = world["terrain"]
        self._terrain_origin = (terrain["minX"], terrain["minY"])
        self._terrain_step = terrain["step"]
        self._terrain_scale = terrain.get("scale", 0.01)
        self._terrain_width = terrain["nx"]
        self._terrain_height = terrain["ny"]
        terrain_name = terrain.get("file", "terrain.i16")
        terrain_path = next((path / name for name in files if Path(name).name == terrain_name), None)
        terrain_path = terrain_path or world_path.with_name(terrain_name)
        self._terrain = array("h", terrain_path.read_bytes())
        if sys.byteorder != "little":
            self._terrain.byteswap()
        if len(self._terrain) != self._terrain_width * self._terrain_height:
            raise ValueError("Arena terrain dimensions do not match the terrain file")

        self._bins = {}
        for definition in world["prisms"]:
            prism = _Prism(definition)
            for cell in self._bin_keys(prism.min_x, prism.min_y, prism.max_x, prism.max_y):
                self._bins.setdefault(cell, []).append(prism)

    @staticmethod
    def _bin_keys(min_x, min_y, max_x, max_y):
        for cell_x in range(math.floor(min_x / _BIN_SIZE), math.floor(max_x / _BIN_SIZE) + 1):
            for cell_y in range(math.floor(min_y / _BIN_SIZE), math.floor(max_y / _BIN_SIZE) + 1):
                yield cell_x, cell_y

    def _candidates(self, min_x, min_y, max_x, max_y):
        cells = self._bin_keys(min_x, min_y, max_x, max_y)
        first = next(cells)
        candidates = self._bins.get(first, ())
        second = next(cells, None)
        if second is None:
            return candidates
        result = set(candidates)
        result.update(self._bins.get(second, ()))
        for cell in cells:
            result.update(self._bins.get(cell, ()))
        return result

    def ground(self, x, y):
        """Bilinear interpolation of the height field, which excludes rooftops."""
        min_x, min_y = self._terrain_origin
        width, height = self._terrain_width, self._terrain_height
        tx = min(width - 1, max(0.0, (x - min_x) / self._terrain_step))
        ty = min(height - 1, max(0.0, (y - min_y) / self._terrain_step))
        col, row = min(int(tx), width - 2), min(int(ty), height - 2)
        fx, fy = tx - col, ty - row
        index = row * width + col
        low_left, low_right = self._terrain[index], self._terrain[index + 1]
        high_left, high_right = self._terrain[index + width], self._terrain[index + width + 1]
        lower = low_left + (low_right - low_left) * fx
        upper = high_left + (high_right - high_left) * fx
        return (lower + (upper - lower) * fy) * self._terrain_scale

    def surface(self, x, y, max_z=math.inf):
        """Highest support below the specified height, including prism tops."""
        return self._surface(x, y, max_z, self.ground(x, y))

    def _surface(self, x, y, max_z, ground):
        candidates = self._bins.get((math.floor(x / _BIN_SIZE), math.floor(y / _BIN_SIZE)), ())
        support = ground
        for prism in candidates:
            if support < prism.top <= max_z and prism.contains(x, y):
                support = prism.top
        return support

    def body_clear(self, x, y, z, radius, height):
        """Check clearance for an upright cylindrical mech at the given feet position."""
        return z >= self.ground(x, y) - _SUPPORT_EPSILON and self._prism_clear(x, y, z, radius, height)

    def _prism_clear(self, x, y, z, radius, height, step_up=0.0):
        min_x, min_y, max_x, max_y = self.bounds
        if not min_x + radius <= x <= max_x - radius or not min_y + radius <= y <= max_y - radius:
            return False
        for prism in self._candidates(x - radius, y - radius, x + radius, y + radius):
            if (
                z + step_up + _SUPPORT_EPSILON < prism.top
                and z + height > prism.base and prism.touches(x, y, radius)
            ):
                return False
        return True

    def walk_segment(self, start, end, radius, height, step_up, max_slope_deg):
        """Walk a short link, following support instead of cutting a straight 3D line."""
        x, y, z = start
        dx, dy = end[0] - x, end[1] - y
        distance = math.hypot(dx, dy)
        count = max(1, math.ceil(distance / _SAMPLE_DISTANCE))
        max_rise = math.tan(math.radians(max_slope_deg)) * distance / count
        previous_ground = self.ground(x, y)
        supported = self._surface(x, y, z + _SUPPORT_EPSILON, previous_ground)
        if abs(supported - z) > _SUPPORT_EPSILON or not self._prism_clear(x, y, supported, radius, height, step_up):
            return False
        z = supported
        for index in range(1, count + 1):
            fraction = index / count
            next_x, next_y = x + dx * fraction, y + dy * fraction
            ground = self.ground(next_x, next_y)
            support = self._surface(next_x, next_y, z + step_up + _SUPPORT_EPSILON, ground)
            rise = support - z
            both_on_terrain = z == previous_ground and support == ground
            if both_on_terrain:
                if rise > max_rise + _SUPPORT_EPSILON or rise < -step_up - _SUPPORT_EPSILON:
                    return False
            elif abs(rise) > step_up + _SUPPORT_EPSILON:
                return False
            if not self._prism_clear(next_x, next_y, support, radius, height, step_up):
                return False
            z, previous_ground = support, ground
        return abs(z - end[2]) <= _SUPPORT_EPSILON

    def flight_segment(self, start, end, radius, height):
        """Check body clearance along a straight three-dimensional segment."""
        dx, dy, dz = (b - a for a, b in zip(start, end))
        count = max(1, math.ceil(math.dist(start, end) / _SAMPLE_DISTANCE))
        for index in range(count + 1):
            fraction = index / count
            if not self.body_clear(
                start[0] + dx * fraction, start[1] + dy * fraction,
                start[2] + dz * fraction, radius, height,
            ):
                return False
        return True

    def blocks_line(self, start, end):
        """Check solid cover. Trees and decorative barriers do not stop shots."""
        x, y, z = start
        dx, dy, dz = end[0] - x, end[1] - y, end[2] - z
        for prism in self._candidates(min(x, end[0]), min(y, end[1]), max(x, end[0]), max(y, end[1])):
            if prism.blocks_line(start, end):
                return True
        count = max(1, math.ceil(math.hypot(dx, dy) / _SAMPLE_DISTANCE))
        for index in range(count + 1):
            fraction = index / count
            if z + dz * fraction <= self.ground(x + dx * fraction, y + dy * fraction):
                return True
        return False
