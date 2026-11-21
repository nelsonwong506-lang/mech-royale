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

struct RoofEdgeSpot {
    Vec3 pos{};
    Vec2 outwardNormal{};
};

struct RoofJumpLink {
    std::size_t targetRoofIndex = 0;
    Vec3 takeoff{};
    Vec3 landing{};
    double horizDist = 0.0;
    double climb = 0.0;
};

struct RooftopNode {
    std::size_t index = 0;
    std::size_t prismIndex = 0;
    std::array<double, 4> bounds{};
    double baseZ = 0.0;
    double topZ = 0.0;
    Vec3 center{};
    std::vector<Vec3> tuckSpots{};
    std::vector<RoofEdgeSpot> edgeSpots{};
    std::vector<RoofJumpLink> jumpLinks{};
};

class Arena {
public:
    Arena(const std::string& mapDir, const std::vector<std::string>& files = {});

    std::array<double, 4> bounds;  // min X, min Y, max X, max Y

    double ground(double x, double y) const;
    double surface(double x, double y, double maxZ = std::numeric_limits<double>::infinity()) const;
    std::pair<double, bool> walkSurfaceInfo(double x, double y) const;
    double topSurface(double x, double y) const;
    bool isSolidPoint(double x, double y, double z) const;
    bool bodyClear(double x, double y, double z, double radius, double height) const;
    bool prismClear(double x, double y, double z, double radius, double height, double stepUp = 0) const;
    bool walkSegment(const Vec3& start, const Vec3& end, double radius, double height,
                     double stepUp, double maxSlopeDeg) const;
    bool flightSegment(const Vec3& start, const Vec3& end, double radius, double height) const;
    bool blocksPrismLine(const Vec3& start, const Vec3& end) const;
    bool blocksLine(const Vec3& start, const Vec3& end) const;

    // Tree canopy & trunk vision occlusion (protocol.md §7-8: trees block vision > 45m, NOT shots).
    bool blocksVisionTree(const Vec3& start, const Vec3& end) const;
    bool blocksVision(const Vec3& start, const Vec3& end) const;
    std::optional<Vec2> nearestTreeCover(const Vec3& from, const Vec3& threatPos,
                                         double maxRange, double radius) const;

    // Precomputed OSU campus rooftop tactical graph.
    const std::vector<RooftopNode>& rooftops() const { return rooftops_; }
    const RooftopNode* roofAt(const Vec3& pos) const;
    std::vector<const RooftopNode*> nearbyRoofNodes(const Vec3& from, double maxRange,
                                                    double minRise, double maxRise) const;

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
        bool isBridge = false;
        int firstBinX, firstBinY;
        Ring outer;
        std::vector<Ring> holes;

        explicit Prism(const Json& definition);
        bool contains(double x, double y) const;
        bool touches(double x, double y, double radius) const;
        bool blocksLine(const Vec3& start, const Vec3& end) const;
        double edgeClearance(double x, double y) const;
    };

    struct Tree {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double height = 20.0;
        double crownR = 2.5;
        double crownBase = 6.0;
        double trunkR = 0.6;
        double crownX = 0.0;
        double crownY = 0.0;
    };

    static bool insideRing(const Ring& edges, double x, double y);
    static bool ringNearPoint(const Ring& edges, double x, double y, double radiusSquared);
    static double ringDistanceSquared(const Ring& edges, double x, double y);
    static bool crossesRing(const Ring& edges, double x, double y, double dx, double dy);
    static Ring ringEdges(const std::vector<double>& coordinates);

    double surfaceAboveTerrain(double x, double y, double maxZ, double terrainZ) const;
    void loadTrees(const Json& world);
    void buildRooftopGraph();

    Vec2 terrainOrigin_;
    int terrainWidth_, terrainHeight_;
    double terrainStep_, terrainScale_;
    double maxTerrain_ = -std::numeric_limits<double>::infinity();
    std::vector<std::int16_t> terrain_;
    std::vector<Prism> prisms_;
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> bins_;

    double canopyBlockDist_ = 3.96;
    std::vector<Tree> trees_;
    std::unordered_map<std::uint64_t, std::vector<std::size_t>> treeBins_;

    std::vector<RooftopNode> rooftops_;
    std::vector<int> prismToRoof_;
};

}  // namespace mr
