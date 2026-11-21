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

Arena::Arena(const std::string& mapDir, const std::vector<std::string>& files,
             double radius, double height) {
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
    buildRoofMap(definitions, radius, height);
    loadTrees(world);
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

void Arena::loadTrees(const Json& world) {
    if (!world.contains("trees")) return;
    const auto& trees = world.at("trees");
    if (!trees.contains("fields") || !trees.contains("data")) return;
    const auto fields = trees.at("fields").get<std::vector<std::string>>();
    auto column = [&](const char* name) {
        return static_cast<std::size_t>(std::find(fields.begin(), fields.end(), name) - fields.begin());
    };
    const auto x = column("x"), y = column("y"), z = column("z"), h = column("height");
    const auto cr = column("crownR"), cb = column("crownBase"), tr = column("trunkR");
    const auto cx = column("crownX"), cy = column("crownY"), stride = fields.size();
    if (std::max({x, y, z, h, cr, cb, tr}) >= stride) return;
    const auto& data = trees.at("data");
    trees_.reserve(data.size() / stride);
    const auto rules = world.value("rules", Json::object());
    const double density = rules.value("canopyDensity", 0.35);
    // Estimate transmission along ellipsoidal crowns using the map's attenuation parameters.
    const double transmission = std::clamp(rules.value("canopyBlockT", 0.25), 1e-9, 1.0);
    canopyLength_ = density > 0 ? -std::log(transmission) / density : std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + stride <= data.size(); i += stride) {
        const auto read = [&](std::size_t field) { return data[i + field].get<double>(); };
        const Vec3 base{read(x), read(y), read(z)};
        const double height = read(h), bottom = std::clamp(read(cb), 0.0, std::max(0., height));
        const double half = (height - bottom) * 0.5;
        const Vec3 crown{cx < stride ? read(cx) : base[0], cy < stride ? read(cy) : base[1],
                         base[2] + bottom + half};
        const double crownRadius = read(cr), trunkRadius = read(tr);
        const auto index = trees_.size();
        trees_.push_back({base, crown, trunkRadius, bottom, crownRadius, half});
        const int left = bin(std::min(base[0] - trunkRadius, crown[0] - crownRadius));
        const int right = bin(std::max(base[0] + trunkRadius, crown[0] + crownRadius));
        const int low = bin(std::min(base[1] - trunkRadius, crown[1] - crownRadius));
        const int high = bin(std::max(base[1] + trunkRadius, crown[1] + crownRadius));
        for (int bx = left; bx <= right; ++bx)
            for (int by = low; by <= high; ++by) treeBins_[binKey(bx, by)].push_back(index);
    }
}

bool Arena::blocksVision(const Vec3& eye, const Vec3& target, double nearRange) const {
    if (blocksLine(eye, target)) return true;
    if (trees_.empty()) return false;
    const double dx = target[0] - eye[0], dy = target[1] - eye[1], dz = target[2] - eye[2];
    const double length = std::hypot(dx, dy, dz);
    if (length < 1e-6) return false;
    const bool checkCanopy = length > nearRange;
    double foliage = 0;
    auto interval = [](double a, double b, double c) -> std::pair<double, double> {
        // Inside the cylinder/ellipsoid when a*t*t + 2*b*t + c <= 0.
        if (a < 1e-14) return c <= 0 ? std::pair{0., 1.} : std::pair{1., 0.};
        const double disc = b*b - a*c;
        if (disc < 0) return {1., 0.};
        const double root = std::sqrt(disc);
        return {std::max(0., (-b-root)/a), std::min(1., (-b+root)/a)};
    };
    auto occludes = [&](const Tree& tree) {
        const double ox = eye[0] - tree.base[0], oy = eye[1] - tree.base[1];
        auto [lo, hi] = interval(dx*dx + dy*dy, ox*dx + oy*dy,
                                 ox*ox + oy*oy - tree.trunkRadius*tree.trunkRadius);
        if (lo <= hi && tree.trunkRadius > 0 && tree.trunkHeight > 0) {
            const double z0 = eye[2] + dz*lo, z1 = eye[2] + dz*hi;
            if (std::max(z0,z1) >= tree.base[2] && std::min(z0,z1) <= tree.base[2] + tree.trunkHeight)
                return true;
        }
        if (!checkCanopy || tree.crownRadius <= 0 || tree.crownHalfHeight <= 0) return false;
        const double x = (eye[0] - tree.crown[0]) / tree.crownRadius;
        const double y = (eye[1] - tree.crown[1]) / tree.crownRadius;
        const double z = (eye[2] - tree.crown[2]) / tree.crownHalfHeight;
        const double u = dx / tree.crownRadius, v = dy / tree.crownRadius, w = dz / tree.crownHalfHeight;
        const auto [entry, exit] = interval(u*u + v*v + w*w, x*u + y*v + z*w, x*x + y*y + z*z - 1);
        if (exit <= entry) return false;
        foliage += (exit-entry) * length;
        return foliage >= canopyLength_;
    };
    std::vector<unsigned char> checked(trees_.size());
    int bx = bin(eye[0]), by = bin(eye[1]);
    const int endX = bin(target[0]), endY = bin(target[1]);
    const int sx = dx > 0 ? 1 : -1, sy = dy > 0 ? 1 : -1;
    const double inf = std::numeric_limits<double>::infinity();
    const double stepX = dx == 0 ? inf : std::abs(binSize/dx);
    const double stepY = dy == 0 ? inf : std::abs(binSize/dy);
    double tx = dx == 0 ? inf : ((bx + (sx > 0)) * binSize - eye[0]) / dx;
    double ty = dy == 0 ? inf : ((by + (sy > 0)) * binSize - eye[1]) / dy;
    const int maxSteps = std::abs(endX - bx) + std::abs(endY - by) + 2;
    for (int step = 0; step < maxSteps; ++step) {
        const auto cell = treeBins_.find(binKey(bx, by));
        if (cell != treeBins_.end()) {
            for (const auto i : cell->second) {
                if (checked[i]) continue;
                checked[i] = 1;
                if (occludes(trees_[i])) return true;
            }
        }
        if (bx == endX && by == endY) break;
        if (tx < ty) { bx += sx; tx += stepX; }
        else { by += sy; ty += stepY; }
    }
    return false;
}

void Arena::buildRoofMap(const Json& definitions, double radius, double height) {
    prismRoof_.assign(prisms_.size(), -1);
    for (std::size_t i = 0; i < prisms_.size(); ++i) {
        if (definitions[i].value("kind", std::string("building")) == "bridge") continue;
        const auto& prism = prisms_[i];
        const auto [x0, y0, x1, y1] = prism.bounds;
        if (std::min(x1 - x0, y1 - y0) < 2 * radius) continue;
        auto supported = [&](double x, double y) {
            return prism.contains(x, y) && !ringNearPoint(prism.outer, x, y, radius * radius) &&
                std::none_of(prism.holes.begin(), prism.holes.end(), [&](const Ring& hole) {
                    return ringNearPoint(hole, x, y, radius * radius);
                }) && bodyClear(x, y, prism.top, radius, height);
        };
        auto depth = [&](const Ring& ring, double x, double y, double best) {
            for (const auto& e : ring) {
                const double dx = x - e.x, dy = y - e.y;
                const double t = std::clamp((dx * e.dx + dy * e.dy) * e.inverseLengthSquared, 0., 1.);
                const double a = dx - t * e.dx, b = dy - t * e.dy;
                best = std::min(best, a * a + b * b);
            }
            return best;
        };
        RoofSite site{};
        double bestDepth = -1;
        auto tryInterior = [&](double x, double y) {
            if (!supported(x, y)) return;
            double d = depth(prism.outer, x, y, std::numeric_limits<double>::infinity());
            for (const auto& hole : prism.holes) d = depth(hole, x, y, d);
            if (d > bestDepth) {
                bestDepth = d;
                site.interior = {x, y, prism.top};
            }
        };
        // Bounding-box centres can lie in courtyards or on another, taller solid.
        for (int row = 0; row < 5; ++row) {
            for (int col = 0; col < 5; ++col) {
                tryInterior(x0 + (col + 0.5) * (x1 - x0) / 5, y0 + (row + 0.5) * (y1 - y0) / 5);
            }
        }
        std::array<std::optional<Vec3>, 4> edges{}, stops{};
        std::array<double, 4> edgeLengths{}, stopLengths{};
        for (const auto& edge : prism.outer) {
            const double length = 1.0 / std::sqrt(edge.inverseLengthSquared);
            if (length < 2 * radius) continue;
            const double mx = edge.x + 0.5 * edge.dx, my = edge.y + 0.5 * edge.dy;
            Vec2 normal{-edge.dy / length, edge.dx / length};
            if (!prism.contains(mx + 0.05 * normal[0], my + 0.05 * normal[1])) {
                normal[0] *= -1;
                normal[1] *= -1;
            }
            const int side = std::abs(normal[0]) > std::abs(normal[1])
                ? (normal[0] > 0 ? 0 : 1) : (normal[1] > 0 ? 2 : 3);
            const double sx = mx - normal[0] * (radius + 1.5);
            const double sy = my - normal[1] * (radius + 1.5);
            const double sz = ground(sx, sy);
            if (length > stopLengths[side] && prism.top > sz + 5 && prism.base < sz + height &&
                bodyClear(sx, sy, sz, radius, height)) {
                stops[side] = Vec3{sx, sy, sz};
                stopLengths[side] = length;
            }
            const double ex = mx + normal[0] * (radius + 0.35);
            const double ey = my + normal[1] * (radius + 0.35);
            if (length > edgeLengths[side] && supported(ex, ey)) {
                edges[side] = Vec3{ex, ey, prism.top};
                edgeLengths[side] = length;
                if (bestDepth < 0) tryInterior(ex, ey);
            }
        }
        for (const auto& stop : stops) if (stop) wallStops_.push_back(*stop);
        if (bestDepth < 0 || prism.top < ground(site.interior[0], site.interior[1]) + 2) continue;
        for (const auto& edge : edges) {
            if (edge && distanceXY(*edge, site.interior) > 1.5 &&
                walkSegment(site.interior, *edge, radius, height, 0.1, 50.0)) {
                site.edges.push_back(*edge);
            }
        }
        prismRoof_[i] = static_cast<int>(roofs_.size());
        roofs_.push_back(std::move(site));
    }

    // Link only nearby, unobstructed air corridors. Flight execution checks current energy and zone.
    for (std::size_t i = 0; i < roofs_.size(); ++i) {
        auto& site = roofs_[i];
        struct Link { double cost; std::size_t index; Vec3 from, to; };
        std::vector<Link> candidates;
        for (std::size_t j = 0; j < roofs_.size(); ++j) {
            if (i == j) continue;
            const auto& other = roofs_[j];
            const double rise = other.interior[2] - site.interior[2];
            if (rise > 32 || distanceXY(site.interior, other.interior) > 150) continue;
            Vec3 from = site.interior, to = other.interior;
            double dist = distanceXY(from, to);
            auto compare = [&](const Vec3& a, const Vec3& b) {
                const double d = distanceXY(a, b);
                if (d < dist) { from = a; to = b; dist = d; }
            };
            for (const auto& b : other.edges) compare(site.interior, b);
            for (const auto& a : site.edges) {
                compare(a, other.interior);
                for (const auto& b : other.edges) compare(a, b);
            }
            if (dist >= 3 && dist <= 45) candidates.push_back({dist + 0.65 * std::max(0., rise), j, from, to});
        }
        std::sort(candidates.begin(), candidates.end(), [](const Link& a, const Link& b) {
            return a.cost < b.cost;
        });
        for (const auto& link : candidates) {
            const double apex = std::max(link.from[2], link.to[2]) + 2.2;
            const Vec3 a{link.from[0], link.from[1], apex}, b{link.to[0], link.to[1], apex};
            if (!flightSegment(link.from, a, radius, height) || !flightSegment(a, b, radius, height) ||
                !flightSegment(b, link.to, radius, height)) continue;
            site.links.push_back(link.index);
            if (site.links.size() == 5) break;
        }
    }
}

const RoofSite* Arena::roofAt(const Vec3& feet) const {
    const auto cell = bins_.find(binKey(bin(feet[0]), bin(feet[1])));
    if (cell == bins_.end()) return nullptr;
    for (const auto idx : cell->second) {
        if (prismRoof_[idx] >= 0 && std::abs(feet[2] - prisms_[idx].top) <= 0.65 &&
            prisms_[idx].contains(feet[0], feet[1])) return &roofs_[prismRoof_[idx]];
    }
    return nullptr;
}

bool Arena::roofFits(const Vec3& feet, double radius, double height) const {
    if (!bodyClear(feet[0], feet[1], feet[2], radius, height)) return false;
    const auto cell = bins_.find(binKey(bin(feet[0]), bin(feet[1])));
    if (cell == bins_.end()) return false;
    const double radiusSquared = radius * radius;
    for (auto idx : cell->second) {
        const auto& prism = prisms_[idx];
        if (std::abs(prism.top - feet[2]) > 0.65 ||
            !prism.contains(feet[0], feet[1]) || ringNearPoint(prism.outer, feet[0], feet[1], radiusSquared)) continue;
        const bool atHole = std::any_of(prism.holes.begin(), prism.holes.end(), [&](const Ring& hole) {
            return ringNearPoint(hole, feet[0], feet[1], radiusSquared);
        });
        if (!atHole) return true;
    }
    return false;
}

std::vector<Vec3> Arena::nearbyRooftops(const Vec3& from, double maxRange, double minRise,
                                        double maxRise, double radius, double height) const {
    std::vector<Vec3> result;
    for (const auto& roof : roofs_) {
        if (!inRange(roof.interior[2] - from[2], minRise, maxRise)) continue;
        const Vec3* best = nullptr;
        double nearest = maxRange;
        auto consider = [&](const Vec3& p) {
            const double d = distanceXY(from, p);
            if (d <= nearest && roofFits(p, radius, height)) { nearest = d; best = &p; }
        };
        consider(roof.interior);
        for (const auto& edge : roof.edges) consider(edge);
        if (best) result.push_back(*best);
    }
    std::sort(result.begin(), result.end(), [&](const Vec3& a, const Vec3& b) {
        return distanceXY(from, a) < distanceXY(from, b);
    });
    if (result.size() > 8) result.resize(8);
    return result;
}

std::optional<Vec3> Arena::rotationWaypoint(const Vec3& from, const Vec2& goal, double maxRange,
                                            double radius, double height) const {
    const double remaining = distanceXY(from, goal);
    std::optional<Vec3> best;
    double bestScore = 0;
    auto consider = [&](const Vec3& p, bool rooftop) {
        const double d = distanceXY(from, p), progress = remaining - distanceXY(p, goal);
        if (d < 3 || d > maxRange || progress < 3) return;
        const double score = progress - 0.55 * d;
        if (score > bestScore && (rooftop ? roofFits(p, radius, height) : bodyClear(p[0], p[1], p[2], radius, height))) {
            best = p;
            bestScore = score;
        }
    };
    if (const auto* roof = roofAt(from)) {
        for (auto index : roof->links) {
            const auto& next = roofs_[index];
            consider(next.interior, true);
            for (const auto& edge : next.edges) consider(edge, true);
        }
        if (best) return best;
    }
    for (const auto& stop : wallStops_) consider(stop, false);
    return best;
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
