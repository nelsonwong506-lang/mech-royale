#pragma once

#include "common.hpp"

#include <array>
#include <cstddef>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mr {

class Arena;

struct Zone {
    Zone(const Json& state, double now);

    std::pair<Vec2, double> circle(double at) const;
    double clearance(const Vec2& point, double at) const;
    double clearance(const Vec3& point, double at) const;
    double buffer(double availableRadius, double desired) const;
    bool inNext(const Vec2& point, double margin = 0.0) const;
    bool inNext(const Vec3& point, double margin = 0.0) const;
    bool segment(const Vec2& from, const Vec2& to, double at, double duration) const;
    bool segment(const Vec3& from, const Vec3& to, double at, double duration) const;

    double now;
    Vec2 center;
    double radius;
    Vec2 nextCenter;
    double nextRadius;
    double start;
    double end;
};

struct Motion {
    explicit Motion(const Json& stats);

    std::array<double, 4> shape() const;

    double radius;
    double height;
    double step;
    double slope;
    double walk;
    double boost;
    double acceleration;
    double airAcceleration;
    double jump;
    double gravity;
    double jet;
    double rise;
    double jetDrain;
    double boostDrain;
};

struct Flight {
    Vec3 takeoff;
    Vec3 landing;
    double apex;
    double climb;
    double seconds;
    bool airborne = false;
};

struct RouteResult {
    std::vector<Vec3> route;
    bool complete = false;
};

class Navigator {
public:
    explicit Navigator(const Arena& arena);

    void reset();
    bool searchExhausted() const;
    bool isSearching() const;
    std::optional<Vec3> support(const Vec2& point, const Motion& motion,
                                double ceiling = std::numeric_limits<double>::infinity()) const;
    bool walkable(const Vec3& from, const Vec3& to, const Motion& motion);
    double pathTime(const Vec3& from, const std::vector<Vec3>& path, double speed) const;
    bool pathSafe(const Vec3& from, const std::vector<Vec3>& path, const Zone& zone,
                  double speed, std::optional<double> at = std::nullopt) const;
    bool direct(const Vec3& from, const Vec3& to, const Zone& zone, const Motion& motion,
                std::optional<double> speed = std::nullopt);
    RouteResult plan(const Vec3& from, const std::vector<Vec3>& goals, const Zone& zone,
                     const Motion& motion, std::size_t role, Clock::time_point deadline);
    std::optional<Flight> flightPlan(const Vec3& from, const std::vector<Vec3>& goals, const Zone& zone,
                                     const Motion& motion, double energy, double maximum,
                                     Clock::time_point deadline);
    bool guard(const Vec2& move, bool boost, const Json& me, const Zone& zone,
               const Motion& motion, bool jet = false);

private:
    struct PointHash {
        std::size_t operator()(const Vec3& point) const noexcept;
    };

    struct EdgeHash {
        std::size_t operator()(const std::array<Vec3, 2>& edge) const noexcept;
    };

    struct SearchNode {
        double cost;
        double traveled;
        std::optional<Vec3> previous;
    };

    struct QueueNode {
        double rank;
        std::size_t serial;
        Vec3 point;
        double cost;
    };

    struct LaterNode {
        bool operator()(const QueueNode& left, const QueueNode& right) const;
    };

    struct Search {
        Search(const Vec3& from, const std::vector<Vec3>& targets, double startedAt);

        double heuristic(const Vec3& point) const;
        std::vector<Vec3> path(const Vec3& endpoint) const;

        Vec3 from;
        std::vector<Vec3> goals;
        double startedAt;
        std::size_t serial = 0;
        std::priority_queue<QueueNode, std::vector<QueueNode>, LaterNode> queue;
        std::unordered_map<Vec3, SearchNode, PointHash> nodes;
        Vec3 best;
        std::vector<Vec3> complete;
        bool exhausted = false;
    };

    void advance(Search& search, const Zone& zone, const Motion& motion, Clock::time_point deadline);
    std::vector<Vec3> usable(const Vec3& from, const std::vector<Vec3>& path,
                             const Zone& zone, const Motion& motion);

    const Arena& arena_;
    std::optional<Search> search_;
    std::optional<std::size_t> role_;
    std::vector<Vec3> route_;
    std::unordered_map<std::array<Vec3, 2>, bool, EdgeHash> edgeCache_;
    std::optional<std::array<double, 4>> shape_;
};

}  // namespace mr
