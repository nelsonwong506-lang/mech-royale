#pragma once

#include "arena.hpp"
#include "common.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace mr {

struct EscapeRoute {
    Vec2 launch{};
    Vec2 land{};
};

class NavGrid {
public:
    explicit NavGrid(const Arena& arena, double navRadius = 2.6, double step = 2.0);

    int cell(double x, double y) const;
    Vec2 cellPos(int c) const;
    bool inBounds(double x, double y) const;
    bool free(double x, double y) const;
    bool isBlockedCell(int c) const { return blocked_[static_cast<std::size_t>(c)] != 0; }
    float clearanceAt(double x, double y) const { return clearance_[static_cast<std::size_t>(cell(x, y))]; }
    float surfAt(double x, double y) const { return surf_[static_cast<std::size_t>(cell(x, y))]; }
    bool onRoofObst(double x, double y, double z) const;

    int nearestFree(int c) const;
    bool offMain(double x, double y) const;
    bool isMainReachable(double x, double y) const;
    bool canStep(int a, int b, bool diag) const;
    bool lineWalkable(double x0, double y0, double x1, double y1) const;

    std::pair<std::vector<int32_t>, bool> path(double sx, double sy, double gx, double gy,
                                               int maxExpand = 22000, float extraRadius = 0.0f);

    std::optional<Vec2> findCoverBFS(double sx, double sy,
                                     const std::function<bool(double, double)>& isHidden,
                                     const std::function<double(double, double)>& zonePenaltyFn,
                                     const std::vector<Vec2>& threatPositions,
                                     int maxSteps = 55, int maxChecks = 480);

    std::optional<EscapeRoute> escapePlan(double x, double y);
    double exposure(double x, double y) const;

    int nnx() const { return nnx_; }
    int nny() const { return nny_; }
    double cellStep() const { return ns_; }

private:
    struct HeapItem {
        float f;
        int32_t idx;
    };

    void ensureMainNear();
    void pushHeap(HeapItem it);
    HeapItem popHeap();

    const Arena& arena_;
    double minX_ = 0.0, minY_ = 0.0;
    double ns_ = 2.0;
    int nnx_ = 0, nny_ = 0;
    std::vector<std::uint8_t> blocked_;
    std::vector<float> surf_;
    std::vector<float> clearance_;
    std::vector<int32_t> comp_;
    int32_t mainComp_ = 0;
    std::vector<int32_t> mainNear_;

    // Reusable A* / BFS scratch buffers (zero per-tick heap allocations)
    std::vector<float> g_;
    std::vector<int32_t> came_;
    std::vector<std::uint32_t> stamp_;
    std::vector<std::uint32_t> closed_;
    std::uint32_t gen_ = 0;
    std::vector<HeapItem> heap_;
    std::vector<int32_t> queue_;
};

}  // namespace mr
