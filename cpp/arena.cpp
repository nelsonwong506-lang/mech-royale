#include "arena.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace mr {
namespace {

constexpr double binSize = 24.0;
constexpr double supportEpsilon = 0.08;
constexpr double sampleSpacing = 1.2;

int bin(double coordinate) {
    return static_cast<int>(std::floor(coordinate / binSize));
}

std::uint64_t binKey(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32)
         | static_cast<std::uint32_t>(y);
}

bool inRange(double value, double low, double high) {
    return low <= value && value <= high;
}

std::filesystem::path findMapFile(const std::filesystem::path& directory,
                                  const std::vector<std::string>& files,
                                  const std::filesystem::path& defaultPath) {
    for (const auto& file : files) {
        if (std::filesystem::path(file).filename() == defaultPath.filename()) {
            return directory / file;
        }
    }
    return defaultPath;
}

}  // namespace

Arena::Ring Arena::ringEdges(const std::vector<double>& coordinates) {
    if (coordinates.size() < 6 || coordinates.size() % 2 != 0) {
        throw std::runtime_error("Invalid polygon in arena map");
    }
    Ring ring;
    ring.reserve(coordinates.size() / 2);
    for (std::size_t i = 0; i < coordinates.size(); i += 2) {
        const std::size_t next = (i + 2) % coordinates.size();
        const double x = coordinates[i], y = coordinates[i + 1];
        const double dx = coordinates[next] - x, dy = coordinates[next + 1] - y;
        const double lengthSquared = dx * dx + dy * dy;
        if (lengthSquared > 0) {
            ring.push_back({x, y, dx, dy, 1 / lengthSquared});
        }
    }
    return ring;
}

bool Arena::insideRing(const Ring& edges, double x, double y) {
    bool inside = false;
    for (const auto& edge : edges) {
        if ((edge.y > y) != (edge.y + edge.dy > y)
            && x < edge.x + (y - edge.y) * edge.dx / edge.dy) {
            inside = !inside;
        }
    }
    return inside;
}

bool Arena::ringNearPoint(const Ring& edges, double x, double y, double radiusSquared) {
    for (const auto& edge : edges) {
        const double relativeX = x - edge.x, relativeY = y - edge.y;
        const double fraction = std::clamp(
            (relativeX * edge.dx + relativeY * edge.dy) * edge.inverseLengthSquared, 0.0, 1.0);
        const double gapX = relativeX - fraction * edge.dx;
        const double gapY = relativeY - fraction * edge.dy;
        if (gapX * gapX + gapY * gapY <= radiusSquared) {
            return true;
        }
    }
    return false;
}

bool Arena::crossesRing(const Ring& edges, double x, double y, double dx, double dy) {
    const double lengthSquared = dx * dx + dy * dy;
    if (lengthSquared <= 1e-12) {
        return ringNearPoint(edges, x, y, 1e-12);
    }
    for (const auto& edge : edges) {
        const double offsetX = edge.x - x, offsetY = edge.y - y;
        const double determinant = dx * edge.dy - dy * edge.dx;
        if (std::abs(determinant) <= 1e-9) {
            if (std::abs(offsetX * dy - offsetY * dx) > 1e-9) {
                continue;
            }
            const double first = (offsetX * dx + offsetY * dy) / lengthSquared;
            const double last = first + (edge.dx * dx + edge.dy * dy) / lengthSquared;
            if (std::max(first, last) >= 0 && std::min(first, last) <= 1) {
                return true;
            }
            continue;
        }
        const double t = (offsetX * edge.dy - offsetY * edge.dx) / determinant;
        const double u = (offsetX * dy - offsetY * dx) / determinant;
        if (inRange(t, 0, 1) && inRange(u, 0, 1)) {
            return true;
        }
    }
    return false;
}

Arena::Prism::Prism(const Json& definition)
    : base(definition.at("base").get<double>()), top(definition.at("top").get<double>()) {
    const auto coordinates = definition.at("ring").get<std::vector<double>>();
    outer = ringEdges(coordinates);
    bounds = {coordinates[0], coordinates[1], coordinates[0], coordinates[1]};
    for (std::size_t i = 2; i < coordinates.size(); i += 2) {
        bounds[0] = std::min(bounds[0], coordinates[i]);
        bounds[1] = std::min(bounds[1], coordinates[i + 1]);
        bounds[2] = std::max(bounds[2], coordinates[i]);
        bounds[3] = std::max(bounds[3], coordinates[i + 1]);
    }
    firstBinX = bin(bounds[0]);
    firstBinY = bin(bounds[1]);
    if (definition.contains("holes")) {
        for (const auto& hole : definition.at("holes")) {
            holes.push_back(ringEdges(hole.get<std::vector<double>>()));
        }
    }
}

bool Arena::Prism::contains(double x, double y) const {
    return inRange(x, bounds[0], bounds[2]) && inRange(y, bounds[1], bounds[3])
        && insideRing(outer, x, y)
        && std::none_of(holes.begin(), holes.end(), [&](const Ring& hole) { return insideRing(hole, x, y); });
}

bool Arena::Prism::touches(double x, double y, double radius) const {
    if (!inRange(x, bounds[0] - radius, bounds[2] + radius)
        || !inRange(y, bounds[1] - radius, bounds[3] + radius)) {
        return false;
    }
    const double radiusSquared = radius * radius;
    return contains(x, y) || ringNearPoint(outer, x, y, radiusSquared)
        || std::any_of(holes.begin(), holes.end(), [&](const Ring& hole) {
            return ringNearPoint(hole, x, y, radiusSquared);
        });
}

bool Arena::Prism::blocksLine(const Vec3& start, const Vec3& end) const {
    const double dz = end[2] - start[2];
    double first = 0.0, last = 1.0;
    if (std::abs(dz) < 1e-9) {
        if (!inRange(start[2], base, top)) {
            return false;
        }
    } else {
        const auto [entry, exit] = std::minmax({(base - start[2]) / dz, (top - start[2]) / dz});
        first = std::max(first, entry);
        last = std::min(last, exit);
        if (first > last) {
            return false;
        }
    }

    const double dx = end[0] - start[0], dy = end[1] - start[1];
    const double entryX = start[0] + dx * first, exitX = start[0] + dx * last;
    const double entryY = start[1] + dy * first, exitY = start[1] + dy * last;
    if (std::max(entryX, exitX) < bounds[0] || std::min(entryX, exitX) > bounds[2]
        || std::max(entryY, exitY) < bounds[1] || std::min(entryY, exitY) > bounds[3]) {
        return false;
    }
    auto crosses = [&](const Ring& ring) {
        return crossesRing(ring, entryX, entryY, exitX - entryX, exitY - entryY);
    };
    return contains(entryX, entryY) || contains(exitX, exitY) || crosses(outer)
        || std::any_of(holes.begin(), holes.end(), crosses);
}

Arena::Arena(const std::string& mapDir, const std::vector<std::string>& files) {
    const std::filesystem::path directory(mapDir);
    const auto worldPath = findMapFile(directory, files, directory / "v2" / "world.json");
    std::ifstream worldFile(worldPath);
    if (!worldFile) {
        throw std::runtime_error("Cannot open arena world: " + worldPath.string());
    }
    const Json world = Json::parse(worldFile);
    const auto& mapBounds = world.at("bounds");
    bounds = {mapBounds.at("minX").get<double>(), mapBounds.at("minY").get<double>(),
              mapBounds.at("maxX").get<double>(), mapBounds.at("maxY").get<double>()};

    const auto& terrain = world.at("terrain");
    terrainOrigin_ = {terrain.at("minX").get<double>(), terrain.at("minY").get<double>()};
    terrainStep_ = terrain.at("step").get<double>();
    terrainScale_ = terrain.value("scale", 0.01);
    terrainWidth_ = terrain.at("nx").get<int>();
    terrainHeight_ = terrain.at("ny").get<int>();
    if (terrainWidth_ < 2 || terrainHeight_ < 2 || terrainStep_ <= 0) {
        throw std::runtime_error("Invalid arena terrain dimensions or spacing");
    }
    const auto terrainName = terrain.value("file", std::string("terrain.i16"));
    const auto terrainPath = findMapFile(directory, files, worldPath.parent_path() / terrainName);
    std::ifstream terrainFile(terrainPath, std::ios::binary | std::ios::ate);
    if (!terrainFile) {
        throw std::runtime_error("Cannot open arena terrain: " + terrainPath.string());
    }
    const auto cellCount = static_cast<std::size_t>(terrainWidth_) * terrainHeight_;
    const auto bytesExpected = static_cast<std::streamoff>(cellCount) * 2;
    if (terrainFile.tellg() != bytesExpected) {
        throw std::runtime_error("Arena terrain dimensions do not match the terrain file");
    }
    std::vector<unsigned char> bytes(cellCount * 2);
    terrainFile.seekg(0);
    if (!terrainFile.read(reinterpret_cast<char*>(bytes.data()), bytesExpected)) {
        throw std::runtime_error("Cannot read arena terrain: " + terrainPath.string());
    }
    terrain_.resize(cellCount);
    std::int16_t maxRaw = std::numeric_limits<std::int16_t>::min();
    for (std::size_t i = 0; i < cellCount; ++i) {
        const int value = bytes[2 * i] | (bytes[2 * i + 1] << 8);
        terrain_[i] = static_cast<std::int16_t>(value < 32768 ? value : value - 65536);
        maxRaw = std::max(maxRaw, terrain_[i]);
    }
    maxTerrain_ = static_cast<double>(maxRaw) * terrainScale_;

    const auto& definitions = world.at("prisms");
    prisms_.reserve(definitions.size());
    for (const auto& definition : definitions) {
        const auto index = prisms_.size();
        const auto& prism = prisms_.emplace_back(definition);
        for (int x = prism.firstBinX; x <= bin(prism.bounds[2]); ++x) {
            for (int y = prism.firstBinY; y <= bin(prism.bounds[3]); ++y) {
                bins_[binKey(x, y)].push_back(index);
            }
        }
    }
}

double Arena::ground(double x, double y) const {
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return 0.0;
    }
    const double tx = std::clamp((x - terrainOrigin_[0]) / terrainStep_, 0.0, double(terrainWidth_ - 1));
    const double ty = std::clamp((y - terrainOrigin_[1]) / terrainStep_, 0.0, double(terrainHeight_ - 1));
    const int column = std::min(static_cast<int>(tx), terrainWidth_ - 2);
    const int row = std::min(static_cast<int>(ty), terrainHeight_ - 2);
    const double fx = tx - column, fy = ty - row;
    const auto index = static_cast<std::size_t>(row) * terrainWidth_ + column;
    const double lower = terrain_[index] + (terrain_[index + 1] - terrain_[index]) * fx;
    const auto above = index + terrainWidth_;
    const double upper = terrain_[above] + (terrain_[above + 1] - terrain_[above]) * fx;
    return (lower + (upper - lower) * fy) * terrainScale_;
}

double Arena::surface(double x, double y, double maxZ) const {
    return surfaceAboveTerrain(x, y, maxZ, ground(x, y));
}

double Arena::surfaceAboveTerrain(double x, double y, double maxZ, double support) const {
    const auto cell = bins_.find(binKey(bin(x), bin(y)));
    if (cell != bins_.end()) {
        for (auto index : cell->second) {
            const auto& prism = prisms_[index];
            if (support < prism.top && prism.top <= maxZ && prism.contains(x, y)) {
                support = prism.top;
            }
        }
    }
    return support;
}

bool Arena::bodyClear(double x, double y, double z, double radius, double height) const {
    return z >= ground(x, y) - supportEpsilon && prismClear(x, y, z, radius, height);
}

bool Arena::prismClear(double x, double y, double z, double radius, double height, double stepUp) const {
    if (!inRange(x, bounds[0] + radius, bounds[2] - radius)
        || !inRange(y, bounds[1] + radius, bounds[3] - radius)) {
        return false;
    }
    const int firstX = bin(x - radius), lastX = bin(x + radius);
    const int firstY = bin(y - radius), lastY = bin(y + radius);
    for (int bx = firstX; bx <= lastX; ++bx) {
        for (int by = firstY; by <= lastY; ++by) {
            const auto cell = bins_.find(binKey(bx, by));
            if (cell == bins_.end()) {
                continue;
            }
            for (auto index : cell->second) {
                const auto& prism = prisms_[index];
                if (z + stepUp + supportEpsilon < prism.top
                    && z + height > prism.base && prism.touches(x, y, radius)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool Arena::walkSegment(const Vec3& start, const Vec3& end, double radius, double height,
                        double stepUp, double maxSlopeDeg) const {
    const double dx = end[0] - start[0], dy = end[1] - start[1];
    const double distance = std::hypot(dx, dy);
    const int count = std::max(1, static_cast<int>(std::ceil(distance / sampleSpacing)));
    const double maxRise = std::tan(radians(maxSlopeDeg)) * distance / count;
    double previousGround = ground(start[0], start[1]);
    double z = surfaceAboveTerrain(start[0], start[1], start[2] + supportEpsilon, previousGround);
    if (std::abs(z - start[2]) > supportEpsilon || !prismClear(start[0], start[1], z, radius, height, stepUp)) {
        return false;
    }

    for (int i = 1; i <= count; ++i) {
        const double fraction = static_cast<double>(i) / count;
        const double x = start[0] + dx * fraction, y = start[1] + dy * fraction;
        const double terrainZ = ground(x, y);
        const double support = surfaceAboveTerrain(x, y, z + stepUp + supportEpsilon, terrainZ);
        const double rise = support - z;
        const bool bothOnTerrain = z == previousGround && support == terrainZ;
        const double maxClimb = (bothOnTerrain ? maxRise : stepUp) + supportEpsilon;
        const bool steepDrop = rise < -(bothOnTerrain ? maxRise : stepUp) - supportEpsilon;
        if (rise > maxClimb || !prismClear(x, y, support, radius, height, stepUp)) {
            return false;
        }
        if (steepDrop) {
            // Allow stepping down off a rooftop or ledge when the drop path is unobstructed.
            if (bothOnTerrain || rise < -24.0 || !prismClear(x, y, z, radius, height, 0.0)) {
                return false;
            }
        }
        z = support;
        previousGround = terrainZ;
    }
    return std::abs(z - end[2]) <= supportEpsilon;
}

bool Arena::flightSegment(const Vec3& start, const Vec3& end, double radius, double height) const {
    const double dx = end[0] - start[0], dy = end[1] - start[1], dz = end[2] - start[2];
    const int count = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy, dz) / sampleSpacing)));
    for (int i = 0; i <= count; ++i) {
        const double fraction = static_cast<double>(i) / count;
        if (!bodyClear(start[0] + dx * fraction, start[1] + dy * fraction,
                       start[2] + dz * fraction, radius, height)) {
            return false;
        }
    }
    return true;
}

bool Arena::blocksPrismLine(const Vec3& start, const Vec3& end) const {
    int bx = bin(start[0]), by = bin(start[1]);
    const int endBx = bin(end[0]), endBy = bin(end[1]);
    const double dx = end[0] - start[0], dy = end[1] - start[1];
    const int stepX = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
    const int stepY = dy > 0.0 ? 1 : (dy < 0.0 ? -1 : 0);
    constexpr double inf = std::numeric_limits<double>::infinity();
    const double tDeltaX = stepX != 0 ? std::abs(binSize / dx) : inf;
    const double tDeltaY = stepY != 0 ? std::abs(binSize / dy) : inf;
    double tMaxX = stepX != 0
        ? (((stepX > 0 ? bx + 1 : bx) * binSize) - start[0]) / dx
        : inf;
    double tMaxY = stepY != 0
        ? (((stepY > 0 ? by + 1 : by) * binSize) - start[1]) / dy
        : inf;
    const int maxSteps = std::abs(endBx - bx) + std::abs(endBy - by) + 2;

    std::array<std::size_t, 32> checked{};
    std::size_t checkedCount = 0;
    for (int step = 0; step < maxSteps; ++step) {
        const auto cell = bins_.find(binKey(bx, by));
        if (cell != bins_.end()) {
            for (const auto index : cell->second) {
                bool seen = false;
                for (std::size_t k = 0; k < checkedCount; ++k) {
                    if (checked[k] == index) {
                        seen = true;
                        break;
                    }
                }
                if (seen) {
                    continue;
                }
                if (checkedCount < checked.size()) {
                    checked[checkedCount++] = index;
                }
                if (prisms_[index].blocksLine(start, end)) {
                    return true;
                }
            }
        }
        if (bx == endBx && by == endBy) {
            break;
        }
        if (tMaxX < tMaxY) {
            bx += stepX;
            tMaxX += tDeltaX;
        } else {
            by += stepY;
            tMaxY += tDeltaY;
        }
    }
    return false;
}

bool Arena::blocksLine(const Vec3& start, const Vec3& end) const {
    if (blocksPrismLine(start, end)) {
        return true;
    }
    if (start[2] > maxTerrain_ && end[2] > maxTerrain_) {
        return false;
    }
    const double dx = end[0] - start[0], dy = end[1] - start[1], dz = end[2] - start[2];
    const double horiz = std::hypot(dx, dy);
    const double spacing = std::max(sampleSpacing, horiz / 64.0);
    const int count = std::max(1, static_cast<int>(std::ceil(horiz / spacing)));
    for (int i = 0; i <= count; ++i) {
        const double fraction = static_cast<double>(i) / count;
        if (start[2] + dz * fraction <= ground(start[0] + dx * fraction, start[1] + dy * fraction)) {
            return true;
        }
    }
    return false;
}

std::vector<Vec3> Arena::nearbyRooftops(const Vec3& from, double maxRange, double minRise,
                                        double maxRise, double radius, double height) const {
    std::vector<Vec3> roofs;
    const int firstX = bin(from[0] - maxRange), lastX = bin(from[0] + maxRange);
    const int firstY = bin(from[1] - maxRange), lastY = bin(from[1] + maxRange);
    std::vector<std::size_t> visited;
    visited.reserve(32);

    for (int bx = firstX; bx <= lastX; ++bx) {
        for (int by = firstY; by <= lastY; ++by) {
            const auto cell = bins_.find(binKey(bx, by));
            if (cell == bins_.end()) continue;
            for (const auto idx : cell->second) {
                if (std::find(visited.begin(), visited.end(), idx) != visited.end()) continue;
                visited.push_back(idx);
                const auto& prism = prisms_[idx];
                const double rise = prism.top - from[2];
                if (rise < minRise || rise > maxRise) continue;
                const double minX = prism.bounds[0], minY = prism.bounds[1];
                const double maxX = prism.bounds[2], maxY = prism.bounds[3];
                if (maxX - minX < radius * 2.4 || maxY - minY < radius * 2.4) continue;
                const double cx = 0.5 * (minX + maxX), cy = 0.5 * (minY + maxY);
                const double inset = radius + 1.4;
                // Sample a point near the closest climbable edge of the roof plus its center.
                const double nearX = std::clamp(from[0], minX + inset, maxX - inset);
                const double nearY = std::clamp(from[1], minY + inset, maxY - inset);
                const std::array<Vec2, 3> samples{{
                    {nearX, nearY},
                    {0.5 * (nearX + cx), 0.5 * (nearY + cy)},
                    {cx, cy}
                }};
                for (const auto& s : samples) {
                    if (distanceXY(from, s) > maxRange) continue;
                    if (!prism.contains(s[0], s[1])) continue;
                    if (bodyClear(s[0], s[1], prism.top, radius, height) &&
                        std::abs(surface(s[0], s[1], prism.top + 0.5) - prism.top) <= 0.2) {
                        roofs.push_back(Vec3{s[0], s[1], prism.top});
                        break;
                    }
                }
            }
        }
    }
    std::sort(roofs.begin(), roofs.end(), [&](const Vec3& a, const Vec3& b) {
        return distanceXY(from, a) < distanceXY(from, b);
    });
    if (roofs.size() > 8) {
        roofs.resize(8);
    }
    return roofs;
}

}  // namespace mr
