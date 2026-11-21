#pragma once

#include "common.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace mr {

class Arena {
public:
    Arena(const std::string& mapDir, const std::vector<std::string>& files = {});

    std::array<double, 4> bounds;  // min X, min Y, max X, max Y

    double ground(double x, double y) const;
    double surface(double x, double y, double maxZ = std::numeric_limits<double>::infinity()) const;
    bool bodyClear(double x, double y, double z, double radius, double height) const;
    bool walkSegment(const Vec3& start, const Vec3& end, double radius, double height,
                     double stepUp, double maxSlopeDeg) const;
    bool flightSegment(const Vec3& start, const Vec3& end, double radius, double height) const;
    bool blocksLine(const Vec3& start, const Vec3& end) const;

private:
    struct Edge {
        double x, y;
        double dx, dy;
        double inverseLengthSquared;
    };

    using Ring = std::vector<Edge>;

    struct Prism {
        std::array<double, 4> bounds;
        double base, top;
        int firstBinX, firstBinY;
        Ring outer;
        std::vector<Ring> holes;

        explicit Prism(const Json& definition);
        bool contains(double x, double y) const;
        bool touches(double x, double y, double radius) const;
        bool blocksLine(const Vec3& start, const Vec3& end) const;
    };

    static bool insideRing(const Ring& edges, double x, double y);
    static bool ringNearPoint(const Ring& edges, double x, double y, double radiusSquared);
    static bool crossesRing(const Ring& edges, double x, double y, double dx, double dy);
    static Ring ringEdges(const std::vector<double>& coordinates);

    double surfaceAboveTerrain(double x, double y, double maxZ, double terrainZ) const;
    bool prismClear(double x, double y, double z, double radius, double height, double stepUp = 0) const;

    Vec2 terrainOrigin_;
    int terrainWidth_, terrainHeight_;
    double terrainStep_, terrainScale_;
    std::vector<std::int16_t> terrain_;
    std::vector<Prism> prisms_;
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> bins_;
};

}  // namespace mr
