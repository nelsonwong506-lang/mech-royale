#pragma once

#include "common.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace mr {

struct RoofSite {
    Vec3 interior;
    std::vector<Vec3> edges;
    std::vector<std::size_t> links;
};

class Arena {
public:
    Arena(const std::string& mapDir, const std::vector<std::string>& files = {},
          double radius = 2.5, double height = 9.0);

    std::array<double, 4> bounds;  // min X, min Y, max X, max Y

    double ground(double x, double y) const;
    double surface(double x, double y, double maxZ = std::numeric_limits<double>::infinity()) const;
    bool bodyClear(double x, double y, double z, double radius, double height) const;
    bool walkSegment(const Vec3& start, const Vec3& end, double radius, double height,
                     double stepUp, double maxSlopeDeg) const;
    bool flightSegment(const Vec3& start, const Vec3& end, double radius, double height) const;
    bool blocksPrismLine(const Vec3& start, const Vec3& end) const;
    bool blocksLine(const Vec3& start, const Vec3& end) const;
    bool blocksVision(const Vec3& eye, const Vec3& target, double nearRange = 45.0) const;
    const RoofSite* roofAt(const Vec3& feet) const;
    bool roofFits(const Vec3& feet, double radius, double height) const;
    const std::vector<RoofSite>& roofSites() const { return roofs_; }
    std::optional<Vec3> rotationWaypoint(const Vec3& from, const Vec2& goal, double maxRange,
                                         double radius, double height) const;
    std::vector<Vec3> nearbyRooftops(const Vec3& from, double maxRange, double minRise,
                                     double maxRise, double radius, double height) const;
    double distanceToNearestWall(double x, double y, double z, double maxRange = 28.0) const;
    std::optional<Vec2> nearestBuildingAnchor(const Vec3& from, double maxRange,
                                              double radius, const Vec2& biasTarget) const;

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

    struct Tree {
        Vec3 base;
        Vec3 crown;
        double trunkRadius, trunkHeight, crownRadius, crownHalfHeight;
    };

    static bool insideRing(const Ring& edges, double x, double y);
    static bool ringNearPoint(const Ring& edges, double x, double y, double radiusSquared);
    static bool crossesRing(const Ring& edges, double x, double y, double dx, double dy);
    static Ring ringEdges(const std::vector<double>& coordinates);

    double surfaceAboveTerrain(double x, double y, double maxZ, double terrainZ) const;
    bool prismClear(double x, double y, double z, double radius, double height, double stepUp = 0) const;
    void buildRoofMap(const Json& definitions, double radius, double height);
    void loadTrees(const Json& world);

    Vec2 terrainOrigin_;
    int terrainWidth_, terrainHeight_;
    double terrainStep_, terrainScale_;
    double maxTerrain_ = -std::numeric_limits<double>::infinity();
    std::vector<std::int16_t> terrain_;
    std::vector<Prism> prisms_;
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> bins_;
    std::vector<RoofSite> roofs_;
    std::vector<int> prismRoof_;
    std::vector<Vec3> wallStops_;
    std::vector<Tree> trees_;
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> treeBins_;
    double canopyLength_ = 4.0;
};

}  // namespace mr
