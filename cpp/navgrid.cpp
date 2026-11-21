#include "navgrid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace mr {
namespace {

constexpr std::array<std::array<int, 2>, 8> kNbrs{{
    {1, 0}, {-1, 0}, {0, 1}, {0, -1},
    {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
}};

}  // namespace

NavGrid::NavGrid(const Arena& arena, double navRadius, double step)
    : arena_(arena), minX_(arena.bounds[0]), minY_(arena.bounds[1]), ns_(step) {
    nnx_ = std::max(1, static_cast<int>(std::ceil((arena.bounds[2] - arena.bounds[0]) / ns_)) + 1);
    nny_ = std::max(1, static_cast<int>(std::ceil((arena.bounds[3] - arena.bounds[1]) / ns_)) + 1);
    const std::size_t n = static_cast<std::size_t>(nnx_) * static_cast<std::size_t>(nny_);

    blocked_.assign(n, 0);
    surf_.assign(n, 0.0f);
    clearance_.assign(n, 20.0f);
    comp_.assign(n, 0);
    g_.assign(n, 0.0f);
    came_.assign(n, -1);
    stamp_.assign(n, 0);
    closed_.assign(n, 0);
    heap_.reserve(4096);
    queue_.reserve(n);

    // 1. Rasterize walkability at 2m resolution (mark non-bridge roofs & prism-colliding cells blocked).
    for (int j = 0; j < nny_; ++j) {
        const double y = minY_ + j * ns_;
        for (int i = 0; i < nnx_; ++i) {
            const double x = minX_ + i * ns_;
            const std::size_t c = static_cast<std::size_t>(j) * nnx_ + i;
            const auto [z, nonBridgeRoof] = arena_.walkSurfaceInfo(x, y);
            surf_[c] = static_cast<float>(z);
            const bool inMargin = x >= arena_.bounds[0] + navRadius && x <= arena_.bounds[2] - navRadius &&
                                  y >= arena_.bounds[1] + navRadius && y <= arena_.bounds[3] - navRadius;
            const bool standable = inMargin && !nonBridgeRoof && arena_.prismClear(x, y, z, navRadius, 9.0, 1.2);
            blocked_[c] = standable ? 0 : 1;
        }
    }

    // 2. Multi-source BFS wall-clearance distance field (meters from nearest blocked cell, capped at 20m).
    queue_.clear();
    for (std::size_t c = 0; c < n; ++c) {
        if (blocked_[c]) {
            clearance_[c] = 0.0f;
            queue_.push_back(static_cast<int32_t>(c));
        } else {
            clearance_[c] = 1e9f;
        }
    }
    for (std::size_t h = 0; h < queue_.size(); ++h) {
        const int c = queue_[h];
        const float d = clearance_[static_cast<std::size_t>(c)] + static_cast<float>(ns_);
        if (d > 20.0f) continue;
        const int ci = c % nnx_, cj = c / nnx_;
        for (int k = 0; k < 4; ++k) {
            const int i = ci + kNbrs[k][0], j = cj + kNbrs[k][1];
            if (i < 0 || j < 0 || i >= nnx_ || j >= nny_) continue;
            const int nb = j * nnx_ + i;
            if (clearance_[static_cast<std::size_t>(nb)] > d) {
                clearance_[static_cast<std::size_t>(nb)] = d;
                queue_.push_back(static_cast<int32_t>(nb));
            }
        }
    }
    for (float& cl : clearance_) {
        if (cl > 20.0f) cl = 20.0f;
    }

    // 3. Connected-component flood-fill to identify MainComp (the connected OSU street network)
    //    vs enclosed courtyards / isolated ledges!
    int32_t label = 0;
    int32_t bestLabel = 0;
    std::size_t bestSize = 0;
    for (std::size_t c0 = 0; c0 < n; ++c0) {
        if (blocked_[c0] || comp_[c0] != 0) continue;
        ++label;
        comp_[c0] = label;
        queue_.clear();
        queue_.push_back(static_cast<int32_t>(c0));
        for (std::size_t h = 0; h < queue_.size(); ++h) {
            const int c = queue_[h];
            const int ci = c % nnx_, cj = c / nnx_;
            for (int k = 0; k < 4; ++k) {
                const int i = ci + kNbrs[k][0], j = cj + kNbrs[k][1];
                if (i < 0 || j < 0 || i >= nnx_ || j >= nny_) continue;
                const int nb = j * nnx_ + i;
                if (comp_[static_cast<std::size_t>(nb)] == 0 && canStep(c, nb, false) && canStep(nb, c, false)) {
                    comp_[static_cast<std::size_t>(nb)] = label;
                    queue_.push_back(static_cast<int32_t>(nb));
                }
            }
        }
        if (queue_.size() > bestSize) {
            bestSize = queue_.size();
            bestLabel = label;
        }
    }
    mainComp_ = bestLabel;
}

int NavGrid::cell(double x, double y) const {
    const int ci = std::clamp(static_cast<int>(std::llround((x - minX_) / ns_)), 0, nnx_ - 1);
    const int cj = std::clamp(static_cast<int>(std::llround((y - minY_) / ns_)), 0, nny_ - 1);
    return cj * nnx_ + ci;
}

Vec2 NavGrid::cellPos(int c) const {
    return {minX_ + (c % nnx_) * ns_, minY_ + (c / nnx_) * ns_};
}

bool NavGrid::inBounds(double x, double y) const {
    return x > arena_.bounds[0] && x < arena_.bounds[2] && y > arena_.bounds[1] && y < arena_.bounds[3];
}

bool NavGrid::free(double x, double y) const {
    return !blocked_[static_cast<std::size_t>(cell(x, y))];
}

bool NavGrid::onRoofObst(double x, double y, double z) const {
    return z - arena_.ground(x, y) > 3.0 && blocked_[static_cast<std::size_t>(cell(x, y))] != 0;
}

int NavGrid::nearestFree(int c) const {
    if (!blocked_[static_cast<std::size_t>(c)]) {
        return c;
    }
    const int ci = c % nnx_, cj = c / nnx_;
    int best = -1;
    int bestD = std::numeric_limits<int>::max();
    for (int r = 1; r <= 20; ++r) {
        for (int dj = -r; dj <= r; ++dj) {
            for (int di = -r; di <= r; ++di) {
                if (std::max(std::abs(di), std::abs(dj)) != r) continue;
                const int i = ci + di, j = cj + dj;
                if (i < 0 || j < 0 || i >= nnx_ || j >= nny_) continue;
                const int nb = j * nnx_ + i;
                if (!blocked_[static_cast<std::size_t>(nb)]) {
                    const int d = di * di + dj * dj;
                    if (d < bestD) {
                        best = nb;
                        bestD = d;
                    }
                }
            }
        }
        if (best >= 0) return best;
    }
    return -1;
}

bool NavGrid::offMain(double x, double y) const {
    const int c = nearestFree(cell(x, y));
    return c < 0 || comp_[static_cast<std::size_t>(c)] != mainComp_;
}

bool NavGrid::isMainReachable(double x, double y) const {
    const int c = nearestFree(cell(x, y));
    return c >= 0 && comp_[static_cast<std::size_t>(c)] == mainComp_;
}

bool NavGrid::canStep(int a, int b, bool diag) const {
    if (blocked_[static_cast<std::size_t>(b)]) return false;
    const float rise = surf_[static_cast<std::size_t>(b)] - surf_[static_cast<std::size_t>(a)];
    const float lim = diag ? 3.1f : 2.2f;
    return rise <= lim;
}

bool NavGrid::lineWalkable(double x0, double y0, double x1, double y1) const {
    const double dx = x1 - x0, dy = y1 - y0;
    const double d = std::hypot(dx, dy);
    const int steps = static_cast<int>(d) + 1;
    int prev = cell(x0, y0);
    for (int k = 1; k <= steps; ++k) {
        const double t = static_cast<double>(k) / steps;
        const int c = cell(x0 + dx * t, y0 + dy * t);
        if (c == prev) continue;
        if (blocked_[static_cast<std::size_t>(c)] ||
            surf_[static_cast<std::size_t>(c)] - surf_[static_cast<std::size_t>(prev)] > 3.1f) {
            return false;
        }
        prev = c;
    }
    return true;
}

void NavGrid::pushHeap(HeapItem it) {
    heap_.push_back(it);
    std::size_t i = heap_.size() - 1;
    while (i > 0) {
        const std::size_t p = (i - 1) / 2;
        if (heap_[p].f <= heap_[i].f) break;
        std::swap(heap_[p], heap_[i]);
        i = p;
    }
}

NavGrid::HeapItem NavGrid::popHeap() {
    const HeapItem top = heap_.front();
    const HeapItem last = heap_.back();
    heap_.pop_back();
    if (heap_.empty()) return top;
    heap_[0] = last;
    const std::size_t n = heap_.size();
    std::size_t i = 0;
    for (;;) {
        const std::size_t l = 2 * i + 1, r = 2 * i + 2;
        std::size_t m = i;
        if (l < n && heap_[l].f < heap_[m].f) m = l;
        if (r < n && heap_[r].f < heap_[m].f) m = r;
        if (m == i) break;
        std::swap(heap_[m], heap_[i]);
        i = m;
    }
    return top;
}

std::pair<std::vector<int32_t>, bool> NavGrid::path(double sx, double sy, double gx, double gy,
                                                    int maxExpand, float extraRadius) {
    const int s = nearestFree(cell(sx, sy));
    const int g = nearestFree(cell(gx, gy));
    if (s < 0 || g < 0) return {{}, false};
    if (s == g) return {{static_cast<int32_t>(s)}, true};

    ++gen_;
    const std::uint32_t gen = gen_;
    heap_.clear();
    const int gi = g % nnx_, gj = g / nnx_;
    auto h = [&](int c) -> float {
        const double dx = std::abs((c % nnx_) - gi);
        const double dy = std::abs((c / nnx_) - gj);
        const double mn = std::min(dx, dy), mx = std::max(dx, dy);
        return static_cast<float>((mx + 0.4142 * mn) * ns_ * 1.15);
    };

    g_[static_cast<std::size_t>(s)] = 0.0f;
    came_[static_cast<std::size_t>(s)] = -1;
    stamp_[static_cast<std::size_t>(s)] = gen;
    pushHeap({h(s), static_cast<int32_t>(s)});

    int best = s;
    float bestH = h(s);
    bool found = false;

    for (int n = 0; !heap_.empty() && n < maxExpand; ++n) {
        const HeapItem it = popHeap();
        const int c = it.idx;
        if (closed_[static_cast<std::size_t>(c)] == gen) continue;
        closed_[static_cast<std::size_t>(c)] = gen;
        if (c == g) {
            found = true;
            best = c;
            break;
        }
        const float hc = h(c);
        if (hc < bestH) {
            best = c;
            bestH = hc;
        }
        const int ci = c % nnx_, cj = c / nnx_;
        for (int k = 0; k < 8; ++k) {
            const int i = ci + kNbrs[k][0], j = cj + kNbrs[k][1];
            if (i < 0 || j < 0 || i >= nnx_ || j >= nny_) continue;
            const int nb = j * nnx_ + i;
            const bool diag = k >= 4;
            if (closed_[static_cast<std::size_t>(nb)] == gen || !canStep(c, nb, diag)) continue;
            float step = static_cast<float>(ns_);
            if (diag) {
                if (blocked_[static_cast<std::size_t>(cj * nnx_ + i)] ||
                    blocked_[static_cast<std::size_t>(j * nnx_ + ci)]) {
                    continue;
                }
                step *= 1.4142f;
            }
            const float cl = clearance_[static_cast<std::size_t>(nb)];
            if (extraRadius > 0.2f && cl <= extraRadius) {
                step *= 8.0f;
            } else if (cl < 4.0f + extraRadius) {
                step *= 1.0f + (4.0f + extraRadius - cl) * 0.15f;
            }
            const float ng = g_[static_cast<std::size_t>(c)] + step;
            if (stamp_[static_cast<std::size_t>(nb)] != gen || ng < g_[static_cast<std::size_t>(nb)]) {
                g_[static_cast<std::size_t>(nb)] = ng;
                came_[static_cast<std::size_t>(nb)] = static_cast<int32_t>(c);
                stamp_[static_cast<std::size_t>(nb)] = gen;
                pushHeap({ng + h(nb), static_cast<int32_t>(nb)});
            }
        }
    }

    std::vector<int32_t> out;
    for (int c = best; c >= 0; c = came_[static_cast<std::size_t>(c)]) {
        out.push_back(static_cast<int32_t>(c));
        if (c == s) break;
    }
    std::reverse(out.begin(), out.end());
    return {std::move(out), found};
}

std::optional<Vec2> NavGrid::findCoverBFS(double sx, double sy,
                                          const std::function<bool(double, double)>& isHidden,
                                          const std::function<double(double, double)>& zonePenaltyFn,
                                          const std::vector<Vec2>& threatPositions,
                                          int maxSteps, int maxChecks) {
    const int s = nearestFree(cell(sx, sy));
    if (s < 0) return std::nullopt;

    ++gen_;
    const std::uint32_t gen = gen_;
    queue_.clear();
    queue_.push_back(static_cast<int32_t>(s));
    stamp_[static_cast<std::size_t>(s)] = gen;
    g_[static_cast<std::size_t>(s)] = 0.0f;

    int bestC = -1;
    double bestS = std::numeric_limits<double>::infinity();
    int checked = 0;

    for (std::size_t h = 0; h < queue_.size() && checked < maxChecks; ++h) {
        const int c = queue_[h];
        const int ci = c % nnx_, cj = c / nnx_;
        const double steps = g_[static_cast<std::size_t>(c)];
        if (steps > maxSteps || (bestC >= 0 && steps * 2.0 > bestS + 30.0)) {
            break;
        }
        if (((ci + cj) & 1) == 0 && steps > 0.0) {
            const Vec2 pos = cellPos(c);
            ++checked;
            if (isHidden(pos[0], pos[1])) {
                double sc = steps * 2.0 + zonePenaltyFn(pos[0], pos[1]);
                double near = std::numeric_limits<double>::infinity();
                for (const auto& tp : threatPositions) {
                    near = std::min(near, std::hypot(tp[0] - pos[0], tp[1] - pos[1]));
                }
                sc -= std::min(near, 80.0) * 0.25;
                if (sc < bestS) {
                    bestC = c;
                    bestS = sc;
                }
            }
        }
        for (int k = 0; k < 8; ++k) {
            const int i = ci + kNbrs[k][0], j = cj + kNbrs[k][1];
            if (i < 0 || j < 0 || i >= nnx_ || j >= nny_) continue;
            const int nb = j * nnx_ + i;
            if (stamp_[static_cast<std::size_t>(nb)] == gen || !canStep(c, nb, k >= 4)) continue;
            if (k >= 4 && (blocked_[static_cast<std::size_t>(cj * nnx_ + i)] ||
                           blocked_[static_cast<std::size_t>(j * nnx_ + ci)])) {
                continue;
            }
            stamp_[static_cast<std::size_t>(nb)] = gen;
            g_[static_cast<std::size_t>(nb)] = g_[static_cast<std::size_t>(c)] + 1.0f;
            queue_.push_back(static_cast<int32_t>(nb));
        }
    }
    if (bestC < 0) return std::nullopt;
    return cellPos(bestC);
}

void NavGrid::ensureMainNear() {
    if (!mainNear_.empty()) return;
    const std::size_t n = blocked_.size();
    mainNear_.assign(n, -1);
    std::vector<std::uint8_t> depth(n, 0);
    queue_.clear();
    for (std::size_t i = 0; i < n; ++i) {
        if (!blocked_[i] && comp_[i] == mainComp_) {
            mainNear_[i] = static_cast<int32_t>(i);
            queue_.push_back(static_cast<int32_t>(i));
        }
    }
    for (std::size_t h = 0; h < queue_.size(); ++h) {
        const int c = queue_[h];
        if (depth[static_cast<std::size_t>(c)] >= 24) continue;
        const int ci = c % nnx_, cj = c / nnx_;
        for (int k = 0; k < 8; ++k) {
            const int i = ci + kNbrs[k][0], j = cj + kNbrs[k][1];
            if (i < 0 || j < 0 || i >= nnx_ || j >= nny_) continue;
            const int nb = j * nnx_ + i;
            if (mainNear_[static_cast<std::size_t>(nb)] < 0) {
                mainNear_[static_cast<std::size_t>(nb)] = mainNear_[static_cast<std::size_t>(c)];
                depth[static_cast<std::size_t>(nb)] = static_cast<std::uint8_t>(depth[static_cast<std::size_t>(c)] + 1);
                queue_.push_back(static_cast<int32_t>(nb));
            }
        }
    }
}

std::optional<EscapeRoute> NavGrid::escapePlan(double x, double y) {
    ensureMainNear();
    const int s = nearestFree(cell(x, y));
    if (s < 0) return std::nullopt;

    ++gen_;
    const std::uint32_t gen = gen_;
    queue_.clear();
    queue_.push_back(static_cast<int32_t>(s));
    stamp_[static_cast<std::size_t>(s)] = gen;
    g_[static_cast<std::size_t>(s)] = 0.0f;

    int best = -1;
    double bestS = std::numeric_limits<double>::infinity();
    for (std::size_t h = 0; h < queue_.size() && h < 45000; ++h) {
        const int c = queue_[h];
        const int ci = c % nnx_, cj = c / nnx_;
        const int m = mainNear_[static_cast<std::size_t>(c)];
        if (m >= 0) {
            const Vec2 cp = cellPos(c);
            const Vec2 mp = cellPos(m);
            const double sc = g_[static_cast<std::size_t>(c)] * ns_ + 4.0 * distanceXY(mp, cp);
            if (sc < bestS) {
                best = c;
                bestS = sc;
            }
        }
        for (int k = 0; k < 8; ++k) {
            const int i = ci + kNbrs[k][0], j = cj + kNbrs[k][1];
            if (i < 0 || j < 0 || i >= nnx_ || j >= nny_) continue;
            const int nb = j * nnx_ + i;
            if (stamp_[static_cast<std::size_t>(nb)] == gen || !canStep(c, nb, k >= 4)) continue;
            if (k >= 4 && (blocked_[static_cast<std::size_t>(cj * nnx_ + i)] ||
                           blocked_[static_cast<std::size_t>(j * nnx_ + ci)])) {
                continue;
            }
            stamp_[static_cast<std::size_t>(nb)] = gen;
            g_[static_cast<std::size_t>(nb)] = g_[static_cast<std::size_t>(c)] + 1.0f;
            queue_.push_back(static_cast<int32_t>(nb));
        }
    }
    if (best < 0) return std::nullopt;
    const Vec2 launch = cellPos(best);
    Vec2 land = cellPos(mainNear_[static_cast<std::size_t>(best)]);
    const double d = distanceXY(launch, land);
    if (d > 0.1) {
        // Push landing 2m deeper into MainComp so we don't catch the building lip!
        const Vec2 probe{land[0] + (land[0] - launch[0]) / d * 2.0,
                         land[1] + (land[1] - launch[1]) / d * 2.0};
        if (inBounds(probe[0], probe[1]) && free(probe[0], probe[1])) {
            land = probe;
        }
    }
    return EscapeRoute{launch, land};
}

double NavGrid::exposure(double x, double y) const {
    const double z = surfAt(x, y) + 6.0;
    double openSum = 0.0;
    constexpr int kRays = 8;
    for (int r = 0; r < kRays; ++r) {
        const double ang = r * (2.0 * std::numbers::pi / kRays);
        const double ux = std::cos(ang), uy = std::sin(ang);
        double clearDist = 100.0;
        for (const double dist : {15.0, 35.0, 60.0, 95.0}) {
            const Vec3 end{x + ux * dist, y + uy * dist, z};
            if (arena_.blocksPrismLine({x, y, z}, end)) {
                clearDist = dist;
                break;
            }
        }
        openSum += clearDist / 100.0;
    }
    return openSum / kRays;
}

}  // namespace mr
