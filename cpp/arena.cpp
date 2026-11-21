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

// Compute 3D segment length inside a vertical cylinder [zMin, zMax] with center (cx, cy) and radius r.
double cylinderChordLength(const Vec3& start, const Vec3& end,
                           double cx, double cy, double r, double zMin, double zMax) {
    if (r <= 0.0 || zMax <= zMin) {
        return 0.0;
    }
    const double dx = end[0] - start[0];
    const double dy = end[1] - start[1];
    const double dz = end[2] - start[2];
    const double fx = start[0] - cx;
    const double fy = start[1] - cy;
    const double a = dx * dx + dy * dy;
    double t0 = 0.0, t1 = 1.0;
    if (a <= 1e-10) {
        if (fx * fx + fy * fy > r * r) {
            return 0.0;
        }
    } else {
        const double b = 2.0 * (fx * dx + fy * dy);
        const double c = fx * fx + fy * fy - r * r;
        const double disc = b * b - 4.0 * a * c;
        if (disc <= 0.0) {
            return 0.0;
        }
        const double root = std::sqrt(disc);
        t0 = std::max(t0, (-b - root) / (2.0 * a));
        t1 = std::min(t1, (-b + root) / (2.0 * a));
        if (t0 >= t1) {
            return 0.0;
        }
    }
    if (std::abs(dz) <= 1e-9) {
        if (start[2] < zMin || start[2] > zMax) {
            return 0.0;
        }
    } else {
        const auto [zt0, zt1] = std::minmax({(zMin - start[2]) / dz, (zMax - start[2]) / dz});
        t0 = std::max(t0, zt0);
        t1 = std::min(t1, zt1);
        if (t0 >= t1) {
            return 0.0;
        }
    }
    return (t1 - t0) * std::hypot(dx, dy, dz);
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

double Arena::ringDistanceSquared(const Ring& edges, double x, double y) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& edge : edges) {
        const double relativeX = x - edge.x, relativeY = y - edge.y;
        const double fraction = std::clamp(
            (relativeX * edge.dx + relativeY * edge.dy) * edge.inverseLengthSquared, 0.0, 1.0);
        const double gapX = relativeX - fraction * edge.dx;
        const double gapY = relativeY - fraction * edge.dy;
        best = std::min(best, gapX * gapX + gapY * gapY);
    }
    return best;
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

double Arena::Prism::edgeClearance(double x, double y) const {
    double minSq = ringDistanceSquared(outer, x, y);
    for (const auto& hole : holes) {
        minSq = std::min(minSq, ringDistanceSquared(hole, x, y));
    }
    return std::sqrt(std::max(0.0, minSq));
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

    loadTrees(world);
    buildRooftopGraph();
}

void Arena::loadTrees(const Json& world) {
    if (const auto rulesIt = world.find("rules"); rulesIt != world.end() && rulesIt->is_object()) {
        const double density = std::max(0.05, rulesIt->value("canopyDensity", 0.35));
        const double blockT = std::clamp(rulesIt->value("canopyBlockT", 0.25), 0.01, 0.95);
        canopyBlockDist_ = -std::log(blockT) / density;
    }
    const auto treesIt = world.find("trees");
    if (treesIt == world.end() || !treesIt->is_object()) {
        return;
    }
    const auto fieldsIt = treesIt->find("fields");
    const auto dataIt = treesIt->find("data");
    if (fieldsIt == treesIt->end() || !fieldsIt->is_array() || fieldsIt->empty() ||
        dataIt == treesIt->end() || !dataIt->is_array() || dataIt->empty()) {
        return;
    }
    const std::size_t stride = fieldsIt->size();
    int idxX = -1, idxY = -1, idxZ = -1, idxH = -1, idxCrownR = -1, idxCrownBase = -1, idxTrunkR = -1;
    int idxCrownX = -1, idxCrownY = -1;
    for (std::size_t i = 0; i < stride; ++i) {
        if (!(*fieldsIt)[i].is_string()) continue;
        const auto& name = (*fieldsIt)[i].get_ref<const std::string&>();
        if (name == "x") idxX = static_cast<int>(i);
        else if (name == "y") idxY = static_cast<int>(i);
        else if (name == "z") idxZ = static_cast<int>(i);
        else if (name == "height") idxH = static_cast<int>(i);
        else if (name == "crownR") idxCrownR = static_cast<int>(i);
        else if (name == "crownBase") idxCrownBase = static_cast<int>(i);
        else if (name == "trunkR") idxTrunkR = static_cast<int>(i);
        else if (name == "crownX") idxCrownX = static_cast<int>(i);
        else if (name == "crownY") idxCrownY = static_cast<int>(i);
    }
    if (idxX < 0 || idxY < 0 || idxZ < 0) {
        return;
    }
    const auto& data = *dataIt;
    const std::size_t count = data.size() / stride;
    trees_.reserve(count);
    auto readVal = [&](std::size_t base, int offset, double fallback) -> double {
        if (offset < 0) return fallback;
        const auto& v = data[base + static_cast<std::size_t>(offset)];
        return v.is_number() ? finiteOr(v.get<double>(), fallback) : fallback;
    };
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t base = i * stride;
        Tree t;
        t.x = readVal(base, idxX, 0.0);
        t.y = readVal(base, idxY, 0.0);
        t.z = readVal(base, idxZ, ground(t.x, t.y));
        t.height = readVal(base, idxH, 20.0);
        t.crownR = readVal(base, idxCrownR, 2.5);
        t.crownBase = readVal(base, idxCrownBase, 6.0);
        t.trunkR = readVal(base, idxTrunkR, 0.6);
        t.crownX = readVal(base, idxCrownX, t.x);
        t.crownY = readVal(base, idxCrownY, t.y);
        const std::size_t tIdx = trees_.size();
        trees_.push_back(t);
        const double maxR = std::max(t.trunkR, t.crownR + std::hypot(t.crownX - t.x, t.crownY - t.y));
        const int bx0 = bin(t.x - maxR), bx1 = bin(t.x + maxR);
        const int by0 = bin(t.y - maxR), by1 = bin(t.y + maxR);
        for (int bx = bx0; bx <= bx1; ++bx) {
            for (int by = by0; by <= by1; ++by) {
                treeBins_[binKey(bx, by)].push_back(tIdx);
            }
        }
    }
}

void Arena::buildRooftopGraph() {
    prismToRoof_.assign(prisms_.size(), -1);
    constexpr double standRadius = 2.6;
    constexpr double standHeight = 9.1;
    constexpr double edgeInset = 2.95;

    for (std::size_t idx = 0; idx < prisms_.size(); ++idx) {
        const auto& prism = prisms_[idx];
        const double heightAboveBase = prism.top - prism.base;
        if (heightAboveBase < 4.5 || heightAboveBase > 42.0) {
            continue;
        }
        const double minX = prism.bounds[0], minY = prism.bounds[1];
        const double maxX = prism.bounds[2], maxY = prism.bounds[3];
        if (maxX - minX < 9.5 || maxY - minY < 9.5) {
            continue;
        }

        auto validRoofPoint = [&](double x, double y) -> bool {
            return prism.contains(x, y) &&
                bodyClear(x, y, prism.top, standRadius, standHeight) &&
                std::abs(surface(x, y, prism.top + 0.45) - prism.top) <= 0.22;
        };

        RooftopNode node;
        node.index = rooftops_.size();
        node.prismIndex = idx;
        node.bounds = prism.bounds;
        node.baseZ = prism.base;
        node.topZ = prism.top;

        // 1. Sample interior points to find safe 360-deg tuck spots (>= 3.1m from edges) and center.
        struct CandidateSpot {
            Vec3 pos;
            double clearance;
        };
        std::vector<CandidateSpot> interior;
        const double spanX = maxX - minX, spanY = maxY - minY;
        const int stepsX = std::clamp(static_cast<int>(std::ceil(spanX / 4.5)), 2, 10);
        const int stepsY = std::clamp(static_cast<int>(std::ceil(spanY / 4.5)), 2, 10);
        for (int ix = 1; ix < stepsX; ++ix) {
            const double x = minX + spanX * (static_cast<double>(ix) / stepsX);
            for (int iy = 1; iy < stepsY; ++iy) {
                const double y = minY + spanY * (static_cast<double>(iy) / stepsY);
                if (!validRoofPoint(x, y)) continue;
                const double clr = prism.edgeClearance(x, y);
                interior.push_back({Vec3{x, y, prism.top}, clr});
            }
        }
        const double cx = 0.5 * (minX + maxX), cy = 0.5 * (minY + maxY);
        if (validRoofPoint(cx, cy)) {
            interior.push_back({Vec3{cx, cy, prism.top}, prism.edgeClearance(cx, cy)});
        }
        std::sort(interior.begin(), interior.end(), [](const CandidateSpot& a, const CandidateSpot& b) {
            return a.clearance > b.clearance;
        });
        for (const auto& c : interior) {
            if (c.clearance < 3.05 && !node.tuckSpots.empty()) {
                break;
            }
            if (std::any_of(node.tuckSpots.begin(), node.tuckSpots.end(), [&](const Vec3& p) {
                    return distanceXY(p, c.pos) < 4.2;
                })) {
                continue;
            }
            node.tuckSpots.push_back(c.pos);
            if (node.tuckSpots.size() >= 4) {
                break;
            }
        }

        // 2. Sample perimeter edge shooting ledges along outer ring segments.
        for (const auto& edge : prism.outer) {
            const double len = std::hypot(edge.dx, edge.dy);
            if (len < 4.5) continue;
            const double tx = edge.dx / len, ty = edge.dy / len;
            // Test both perpendicular normals to identify which points inward vs outward.
            const Vec2 n1{-ty, tx};
            const Vec2 n2{ty, -tx};
            const double midX = edge.x + 0.5 * edge.dx;
            const double midY = edge.y + 0.5 * edge.dy;
            Vec2 inward = n1;
            Vec2 outward = n2;
            if (!prism.contains(midX + inward[0] * edgeInset, midY + inward[1] * edgeInset) &&
                prism.contains(midX + n2[0] * edgeInset, midY + n2[1] * edgeInset)) {
                inward = n2;
                outward = n1;
            }
            const int samples = std::clamp(static_cast<int>(std::floor(len / 8.0)), 1, 4);
            for (int s = 1; s <= samples; ++s) {
                const double frac = static_cast<double>(s) / (samples + 1);
                const double ex = edge.x + edge.dx * frac + inward[0] * edgeInset;
                const double ey = edge.y + edge.dy * frac + inward[1] * edgeInset;
                if (!validRoofPoint(ex, ey)) continue;
                if (std::any_of(node.edgeSpots.begin(), node.edgeSpots.end(), [&](const RoofEdgeSpot& spot) {
                        return distanceXY(spot.pos, Vec2{ex, ey}) < 4.0;
                    })) {
                    continue;
                }
                node.edgeSpots.push_back(RoofEdgeSpot{Vec3{ex, ey, prism.top}, outward});
                if (node.edgeSpots.size() >= 16) {
                    break;
                }
            }
            if (node.edgeSpots.size() >= 16) {
                break;
            }
        }

        if (node.tuckSpots.empty() && node.edgeSpots.empty()) {
            continue;
        }
        if (!node.tuckSpots.empty()) {
            node.center = node.tuckSpots.front();
        } else {
            node.center = node.edgeSpots.front().pos;
            node.tuckSpots.push_back(node.center);
        }
        if (node.edgeSpots.empty()) {
            node.edgeSpots.push_back(RoofEdgeSpot{node.center, {1.0, 0.0}});
        }

        prismToRoof_[idx] = static_cast<int>(rooftops_.size());
        rooftops_.push_back(std::move(node));
    }

    // 3. Precompute roof-to-roof jump-jet links (within 36m horizontally, climb <= 24m).
    for (std::size_t i = 0; i < rooftops_.size(); ++i) {
        auto& src = rooftops_[i];
        for (std::size_t j = 0; j < rooftops_.size(); ++j) {
            if (i == j) continue;
            const auto& dst = rooftops_[j];
            const double climb = dst.topZ - src.topZ;
            if (climb > 24.0 || climb < -28.0) continue;
            // Bounding box quick reject (> 38m gap).
            const double gapX = std::max(0.0, std::max(src.bounds[0] - dst.bounds[2], dst.bounds[0] - src.bounds[2]));
            const double gapY = std::max(0.0, std::max(src.bounds[1] - dst.bounds[3], dst.bounds[1] - src.bounds[3]));
            if ( std::hypot(gapX, gapY) > 36.0) continue;

            double bestDist = 36.0;
            Vec3 bestTakeoff = src.center;
            Vec3 bestLanding = dst.center;
            bool found = false;
            for (const auto& se : src.edgeSpots) {
                for (const auto& de : dst.edgeSpots) {
                    const double d = distanceXY(se.pos, de.pos);
                    if (d < bestDist && d >= 3.5) {
                        bestDist = d;
                        bestTakeoff = se.pos;
                        bestLanding = de.pos;
                        found = true;
                    }
                }
            }
            if (found) {
                src.jumpLinks.push_back(RoofJumpLink{j, bestTakeoff, bestLanding, bestDist, climb});
            }
        }
        std::sort(src.jumpLinks.begin(), src.jumpLinks.end(), [](const RoofJumpLink& a, const RoofJumpLink& b) {
            return a.horizDist + std::max(0.0, a.climb) * 0.8 < b.horizDist + std::max(0.0, b.climb) * 0.8;
        });
        if (src.jumpLinks.size() > 6) {
            src.jumpLinks.resize(6);
        }
    }
}

const RooftopNode* Arena::roofAt(const Vec3& pos) const {
    if (pos[2] <= ground(pos[0], pos[1]) + 4.0) {
        return nullptr;
    }
    const auto cell = bins_.find(binKey(bin(pos[0]), bin(pos[1])));
    if (cell == bins_.end()) {
        return nullptr;
    }
    const RooftopNode* best = nullptr;
    double bestDz = 1.3;
    for (const auto prismIdx : cell->second) {
        if (prismIdx >= prismToRoof_.size() || prismToRoof_[prismIdx] < 0) continue;
        const auto& prism = prisms_[prismIdx];
        const double dz = std::abs(pos[2] - prism.top);
        if (dz <= bestDz && prism.touches(pos[0], pos[1], 2.2)) {
            bestDz = dz;
            best = &rooftops_[static_cast<std::size_t>(prismToRoof_[prismIdx])];
        }
    }
    return best;
}

std::vector<const RooftopNode*> Arena::nearbyRoofNodes(const Vec3& from, double maxRange,
                                                       double minRise, double maxRise) const {
    std::vector<const RooftopNode*> result;
    const int firstX = bin(from[0] - maxRange), lastX = bin(from[0] + maxRange);
    const int firstY = bin(from[1] - maxRange), lastY = bin(from[1] + maxRange);
    std::vector<std::size_t> visited;
    visited.reserve(32);

    for (int bx = firstX; bx <= lastX; ++bx) {
        for (int by = firstY; by <= lastY; ++by) {
            const auto cell = bins_.find(binKey(bx, by));
            if (cell == bins_.end()) continue;
            for (const auto idx : cell->second) {
                if (idx >= prismToRoof_.size() || prismToRoof_[idx] < 0) continue;
                const auto rIdx = static_cast<std::size_t>(prismToRoof_[idx]);
                if (std::find(visited.begin(), visited.end(), rIdx) != visited.end()) continue;
                visited.push_back(rIdx);
                const auto& roof = rooftops_[rIdx];
                const double rise = roof.topZ - from[2];
                if (rise < minRise || rise > maxRise) continue;
                const double cx = std::clamp(from[0], roof.bounds[0], roof.bounds[2]);
                const double cy = std::clamp(from[1], roof.bounds[1], roof.bounds[3]);
                if (std::hypot(from[0] - cx, from[1] - cy) <= maxRange) {
                    result.push_back(&roof);
                }
            }
        }
    }
    std::sort(result.begin(), result.end(), [&](const RooftopNode* a, const RooftopNode* b) {
        const double da = distanceXY(from, a->center);
        const double db = distanceXY(from, b->center);
        return da < db;
    });
    if (result.size() > 8) {
        result.resize(8);
    }
    return result;
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
    if (std::abs(z - start[2]) > supportEpsilon) {
        // Mech perched overhanging a rooftop/ledge edge (protocol.md §7): allow stepping off or back onto roof.
        if (start[2] > z && start[2] - z <= 24.0) {
            z = start[2];
        } else {
            return false;
        }
    }
    const bool startGrazing = !prismClear(start[0], start[1], z, radius, height, stepUp);
    if (startGrazing && !prismClear(start[0], start[1], z, std::max(0.5, radius - 0.45), height, stepUp)) {
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
        const double checkR = (startGrazing && i < count && distance * fraction <= 1.6)
            ? std::max(0.5, radius - 0.35)
            : radius;
        if (steepDrop && !bothOnTerrain && rise >= -24.0 &&
            !prismClear(x, y, support, checkR, height, stepUp) &&
            prismClear(x, y, z, checkR, height, 0.0)) {
            // Still overhanging the rooftop edge we are stepping off of: remain at ledge height z
            // until our cylinder radius clears the building face, then drop to ground support!
            continue;
        }
        if (rise > maxClimb || !prismClear(x, y, support, checkR, height, stepUp)) {
            return false;
        }
        if (steepDrop) {
            // Allow stepping down off a rooftop or ledge when the drop path is unobstructed.
            if (bothOnTerrain || rise < -24.0 || !prismClear(x, y, z, checkR, height, 0.0)) {
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
        const double checkR = (i == 0 && count >= 1) ? std::max(0.5, radius - 0.45) : radius;
        if (!bodyClear(start[0] + dx * fraction, start[1] + dy * fraction,
                       start[2] + dz * fraction, checkR, height)) {
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

bool Arena::blocksVisionTree(const Vec3& start, const Vec3& end) const {
    if (trees_.empty()) {
        return false;
    }
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

    std::array<std::size_t, 48> checked{};
    std::size_t checkedCount = 0;
    double canopyAccum = 0.0;

    for (int step = 0; step < maxSteps; ++step) {
        const auto cell = treeBins_.find(binKey(bx, by));
        if (cell != treeBins_.end()) {
            for (const auto index : cell->second) {
                bool seen = false;
                for (std::size_t k = 0; k < checkedCount; ++k) {
                    if (checked[k] == index) {
                        seen = true;
                        break;
                    }
                }
                if (seen) continue;
                if (checkedCount < checked.size()) {
                    checked[checkedCount++] = index;
                }
                const auto& tree = trees_[index];
                // 1. Solid tree trunk blocks sight outright.
                if (cylinderChordLength(start, end, tree.x, tree.y, tree.trunkR,
                                        tree.z, tree.z + tree.crownBase) > 0.05) {
                    return true;
                }
                // 2. Canopy crown attenuates sight (Beer-Lambert).
                canopyAccum += cylinderChordLength(start, end, tree.crownX, tree.crownY, tree.crownR,
                                                   tree.z + tree.crownBase, tree.z + tree.height);
                if (canopyAccum >= canopyBlockDist_) {
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

bool Arena::blocksVision(const Vec3& start, const Vec3& end) const {
    return blocksLine(start, end) || blocksVisionTree(start, end);
}

std::optional<Vec2> Arena::nearestTreeCover(const Vec3& from, const Vec3& threatPos,
                                            double maxRange, double radius) const {
    if (trees_.empty() || distanceXY(from, threatPos) <= 46.0) {
        return std::nullopt;
    }
    const int firstX = bin(from[0] - maxRange), lastX = bin(from[0] + maxRange);
    const int firstY = bin(from[1] - maxRange), lastY = bin(from[1] + maxRange);
    const Vec3 threatEye{threatPos[0], threatPos[1], threatPos[2] + 8.0};
    double bestDist = maxRange + 1.0;
    std::optional<Vec2> bestSpot;

    for (int bx = firstX; bx <= lastX; ++bx) {
        for (int by = firstY; by <= lastY; ++by) {
            const auto cell = treeBins_.find(binKey(bx, by));
            if (cell == treeBins_.end()) continue;
            for (const auto idx : cell->second) {
                const auto& tree = trees_[idx];
                if (tree.crownBase > 9.0 || tree.crownR < 2.0) continue;
                const double dist = std::hypot(from[0] - tree.crownX, from[1] - tree.crownY);
                if (dist >= bestDist || dist > maxRange) continue;
                const Vec2 away = direction(threatPos, Vec2{tree.crownX, tree.crownY}, tree.trunkR + radius + 1.6);
                const Vec2 spot{tree.crownX + away[0], tree.crownY + away[1]};
                const double g = ground(spot[0], spot[1]);
                if (!bodyClear(spot[0], spot[1], g, radius, 9.0)) continue;
                const Vec3 ourEye{spot[0], spot[1], g + 8.0};
                if (blocksVisionTree(threatEye, ourEye)) {
                    bestDist = dist;
                    bestSpot = spot;
                }
            }
        }
    }
    return bestSpot;
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
                if (maxX - minX < 9.5 || maxY - minY < 9.5) continue;
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

double Arena::distanceToNearestWall(double x, double y, double z, double maxRange) const {
    const int firstX = bin(x - maxRange), lastX = bin(x + maxRange);
    const int firstY = bin(y - maxRange), lastY = bin(y + maxRange);
    double best = maxRange + 1.0;
    for (int bx = firstX; bx <= lastX; ++bx) {
        for (int by = firstY; by <= lastY; ++by) {
            const auto cell = bins_.find(binKey(bx, by));
            if (cell == bins_.end()) continue;
            for (const auto idx : cell->second) {
                const auto& p = prisms_[idx];
                if (p.top < z + 5.0 || p.base > z + 4.0) continue;
                if (p.bounds[2] - p.bounds[0] < 5.0 || p.bounds[3] - p.bounds[1] < 5.0) continue;
                const double cx = std::clamp(x, p.bounds[0], p.bounds[2]);
                const double cy = std::clamp(y, p.bounds[1], p.bounds[3]);
                const double d = std::hypot(x - cx, y - cy);
                if (d < best) {
                    best = d;
                }
            }
        }
    }
    return best;
}

std::optional<Vec2> Arena::nearestBuildingAnchor(const Vec3& from, double maxRange,
                                                 double radius, const Vec2& biasTarget) const {
    const int firstX = bin(from[0] - maxRange), lastX = bin(from[0] + maxRange);
    const int firstY = bin(from[1] - maxRange), lastY = bin(from[1] + maxRange);
    std::vector<std::size_t> visited;
    visited.reserve(48);
    double bestScore = std::numeric_limits<double>::infinity();
    std::optional<Vec2> bestAnchor;

    for (int bx = firstX; bx <= lastX; ++bx) {
        for (int by = firstY; by <= lastY; ++by) {
            const auto cell = bins_.find(binKey(bx, by));
            if (cell == bins_.end()) continue;
            for (const auto idx : cell->second) {
                if (std::find(visited.begin(), visited.end(), idx) != visited.end()) continue;
                visited.push_back(idx);
                const auto& p = prisms_[idx];
                // Require a real solid building tall enough to give full cover and wide enough to split angles.
                if (p.top < from[2] + 5.5 || p.base > from[2] + 3.0) continue;
                if (p.bounds[2] - p.bounds[0] < 7.0 || p.bounds[3] - p.bounds[1] < 7.0) continue;
                const double cx = std::clamp(from[0], p.bounds[0], p.bounds[2]);
                const double cy = std::clamp(from[1], p.bounds[1], p.bounds[3]);
                const double dist = std::hypot(from[0] - cx, from[1] - cy);
                if (dist < 4.0 || dist > maxRange) continue;
                // Anchor just outside the nearest building face so we pull right into cover range.
                const double ux = (from[0] - cx) / dist;
                const double uy = (from[1] - cy) / dist;
                const Vec2 anchor{cx + ux * (radius + 3.0), cy + uy * (radius + 3.0)};
                const double biasDist = distanceXY(anchor, biasTarget);
                const double score = dist + 0.25 * biasDist;
                if (score < bestScore) {
                    bestScore = score;
                    bestAnchor = anchor;
                }
            }
        }
    }
    return bestAnchor;
}

}  // namespace mr
