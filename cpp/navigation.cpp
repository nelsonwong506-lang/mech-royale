#include "navigation.hpp"

#include "arena.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <tuple>

namespace mr {
namespace {

constexpr double gridSize = 5.0;
constexpr double zoneTolerance = 1e-7;
constexpr std::array<Vec2, 9> offsets = {{
    {0, 0}, {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1},
}};

std::size_t combineHashes(std::size_t seed, std::size_t next) {
    return seed ^ (next + 0x9e3779b9U + (seed << 6U) + (seed >> 2U));
}

}  // namespace

Zone::Zone(const Json& state, double observedAt)
    : now(observedAt), center(state.at("center").get<Vec2>()), radius(state.at("radius").get<double>()),
      nextCenter(state.at("next_center").get<Vec2>()), nextRadius(state.at("next_radius").get<double>()),
      start(state.at("shrink_start").get<double>()), end(state.at("shrink_end").get<double>()) {}

std::pair<Vec2, double> Zone::circle(double at) const {
    const double effectiveStart = std::max(now, start);
    if (at <= effectiveStart || end <= effectiveStart) {
        return {center, radius};
    }
    const double progress = std::min(1.0, (at - effectiveStart) / (end - effectiveStart));
    return {{center[0] + (nextCenter[0] - center[0]) * progress,
             center[1] + (nextCenter[1] - center[1]) * progress},
            radius + (nextRadius - radius) * progress};
}

double Zone::clearance(const Vec2& point, double at) const {
    const auto [atCenter, atRadius] = circle(at);
    return atRadius - distanceXY(atCenter, point);
}

double Zone::clearance(const Vec3& point, double at) const {
    return clearance(horizontal(point), at);
}

double Zone::buffer(double availableRadius, double desired) const {
    return std::min(desired, availableRadius * 0.24);
}

bool Zone::inNext(const Vec2& point, double margin) const {
    return distanceXY(point, nextCenter)
        <= std::max(0.0, nextRadius - buffer(nextRadius, margin)) + zoneTolerance;
}

bool Zone::inNext(const Vec3& point, double margin) const {
    return inNext(horizontal(point), margin);
}

bool Zone::segment(const Vec2& from, const Vec2& to, double at, double duration) const {
    // A mech already outside may recover; once this path enters, keep it inside.
    bool entered = clearance(from, at) >= -zoneTolerance;
    for (int i = 1; i <= 4; ++i) {
        const double fraction = i / 4.0;
        const Vec2 point{from[0] + (to[0] - from[0]) * fraction,
                         from[1] + (to[1] - from[1]) * fraction};
        const bool inside = clearance(point, at + duration * fraction) >= -zoneTolerance;
        if (entered && !inside) {
            return false;
        }
        entered |= inside;
    }
    return true;
}

bool Zone::segment(const Vec3& from, const Vec3& to, double at, double duration) const {
    return segment(horizontal(from), horizontal(to), at, duration);
}

Motion::Motion(const Json& stats)
    : radius(stats.value("radius", 2.5)), height(stats.value("height", 9.0)),
      step(stats.value("step_up", 1.2)), slope(stats.value("max_slope_deg", 50.0)),
      walk(stats.value("walk_speed", 11.7)), boost(stats.value("boost_speed", 19.5)),
      acceleration(stats.value("ground_accel", 58.5)), airAcceleration(stats.value("air_accel", 12.0)),
      jump(stats.value("jump_speed", 11.0)), gravity(stats.value("gravity", 22.0)),
      jet(stats.value("jet_accel", 34.0)), rise(stats.value("max_rise_speed", 10.0)),
      jetDrain(stats.value("jet_drain", 15.0)), boostDrain(stats.value("boost_drain", 17.0)) {}

std::array<double, 4> Motion::shape() const {
    return {radius, height, step, slope};
}

std::size_t Navigator::PointHash::operator()(const Vec3& point) const noexcept {
    std::size_t hash = 0;
    for (const double component : point) {
        hash = combineHashes(hash, std::hash<double>{}(component));
    }
    return hash;
}

std::size_t Navigator::EdgeHash::operator()(const std::array<Vec3, 2>& edge) const noexcept {
    const PointHash hash;
    return combineHashes(hash(edge[0]), hash(edge[1]));
}

bool Navigator::LaterNode::operator()(const QueueNode& left, const QueueNode& right) const {
    return std::tie(left.rank, left.serial) > std::tie(right.rank, right.serial);
}

Navigator::Search::Search(const Vec3& startPoint, const std::vector<Vec3>& targets, double initialTime)
    : from(startPoint), goals(targets), startedAt(initialTime), best(startPoint) {
    queue.push({heuristic(from), serial++, from, 0.0});
    nodes.emplace(from, SearchNode{0.0, 0.0, std::nullopt});
}

double Navigator::Search::heuristic(const Vec3& point) const {
    double closest = std::numeric_limits<double>::infinity();
    for (const auto& target : goals) {
        closest = std::min(closest, distanceXY(point, target));
    }
    return closest;
}

std::vector<Vec3> Navigator::Search::path(const Vec3& endpoint) const {
    std::vector<Vec3> path{endpoint};
    while (const auto parent = nodes.at(path.back()).previous) {
        path.push_back(*parent);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

Navigator::Navigator(const Arena& arena) : arena_(arena) {}

void Navigator::reset() {
    search_.reset();
    role_.reset();
    route_.clear();
}

bool Navigator::searchExhausted() const {
    return search_ && search_->exhausted;
}

std::optional<Vec3> Navigator::support(const Vec2& point, const Motion& motion, double ceiling) const {
    const auto [x, y] = point;
    const double z = arena_.surface(x, y, ceiling);
    if (std::isfinite(z) && arena_.bodyClear(x, y, z, motion.radius, motion.height)) {
        return Vec3{x, y, z};
    }
    return std::nullopt;
}

bool Navigator::walkable(const Vec3& from, const Vec3& to, const Motion& motion) {
    if (shape_ != motion.shape() || edgeCache_.size() > 14000) {
        edgeCache_.clear();
        shape_ = motion.shape();
    }
    const std::array<Vec3, 2> edge{from, to};
    auto [entry, inserted] = edgeCache_.try_emplace(edge, false);
    if (inserted) {
        entry->second = arena_.walkSegment(from, to, motion.radius, motion.height, motion.step, motion.slope);
    }
    return entry->second;
}

void Navigator::advance(Search& search, const Zone& zone, const Motion& motion, Clock::time_point deadline) {
    std::vector<std::pair<double, Vec3>> nearbyGoals;
    nearbyGoals.reserve(search.goals.size());
    for (int expansion = 0; expansion < 65 && !search.queue.empty() && Clock::now() < deadline; ++expansion) {
        const QueueNode entry = search.queue.top();
        search.queue.pop();
        const SearchNode node = search.nodes.at(entry.point);
        if (entry.cost != node.cost) {
            continue;
        }
        const double at = std::max(zone.now, search.startedAt + node.traveled / motion.walk);
        nearbyGoals.clear();
        for (const auto& goal : search.goals) {
            const double length = distanceXY(entry.point, goal);
            if (length <= gridSize * 1.55) {
                nearbyGoals.emplace_back(length, goal);
            }
        }
        std::sort(nearbyGoals.begin(), nearbyGoals.end());
        for (const auto& [length, goal] : nearbyGoals) {
            if (zone.segment(entry.point, goal, at, length / motion.walk) && walkable(entry.point, goal, motion)) {
                search.complete = search.path(entry.point);
                if (goal != entry.point) {
                    search.complete.push_back(goal);
                }
                return;
            }
        }

        const Vec2 gridPoint{std::nearbyint(entry.point[0] / gridSize), std::nearbyint(entry.point[1] / gridSize)};
        for (const auto& offset : offsets) {
            const Vec2 point{(gridPoint[0] + offset[0]) * gridSize, (gridPoint[1] + offset[1]) * gridSize};
            const double length = distanceXY(entry.point, point);
            if (length < 0.3) {
                continue;
            }
            const double duration = length / motion.walk;
            if (!zone.segment(horizontal(entry.point), point, at, duration)) {
                continue;
            }
            const auto next = support(point, motion, entry.point[2] + motion.step + 0.1);
            if (!next || std::abs((*next)[2] - entry.point[2]) > motion.step * 3) {
                continue;
            }
            const double clearance = zone.clearance(*next, at + duration);
            const double exposure = std::min(8.0, std::max(0.0, -clearance) / 8);
            const double cost = node.cost + length * (1 + exposure);
            const auto old = search.nodes.find(*next);
            if ((old != search.nodes.end() && cost >= old->second.cost) || !walkable(entry.point, *next, motion)) {
                continue;
            }
            search.nodes.insert_or_assign(*next, SearchNode{cost, node.traveled + length, entry.point});
            const double heuristic = search.heuristic(*next);
            if (heuristic + std::max(0.0, -clearance)
                < search.heuristic(search.best) + std::max(0.0, -zone.clearance(search.best, at))) {
                search.best = *next;
            }
            search.queue.push({cost + heuristic, search.serial++, *next, cost});
        }
    }
    search.exhausted = search.queue.empty();
}

double Navigator::pathTime(const Vec3& from, const std::vector<Vec3>& path, double speed) const {
    double length = 0.0;
    const Vec3* previous = &from;
    for (const auto& point : path) {
        length += distanceXY(*previous, point);
        previous = &point;
    }
    return length / speed;
}

bool Navigator::pathSafe(const Vec3& from, const std::vector<Vec3>& path, const Zone& zone,
                         double speed, std::optional<double> at) const {
    double time = at.value_or(zone.now);
    const Vec3* previous = &from;
    for (const auto& point : path) {
        const double duration = distanceXY(*previous, point) / speed;
        if (!zone.segment(*previous, point, time, duration)) {
            return false;
        }
        time += duration;
        previous = &point;
    }
    return true;
}

bool Navigator::direct(const Vec3& from, const Vec3& to, const Zone& zone, const Motion& motion,
                       std::optional<double> speed) {
    const double duration = distanceXY(from, to) / speed.value_or(motion.walk);
    return zone.segment(from, to, zone.now, duration) && walkable(from, to, motion);
}

std::vector<Vec3> Navigator::usable(const Vec3& from, const std::vector<Vec3>& path,
                                    const Zone& zone, const Motion& motion) {
    for (std::size_t end = std::min<std::size_t>(6, path.size()); end > 0; --end) {
        const auto& point = path[end - 1];
        if (distanceXY(from, point) <= 12.0 && direct(from, point, zone, motion)) {
            return {path.begin() + static_cast<std::ptrdiff_t>(end - 1), path.end()};
        }
    }
    return {};
}

RouteResult Navigator::plan(const Vec3& from, const std::vector<Vec3>& goals, const Zone& zone,
                            const Motion& motion, std::size_t role, Clock::time_point deadline) {
    if (goals.empty()) {
        reset();
        return {};
    }
    if (role_ != role) {
        reset();
        role_ = role;
    }
    if (!route_.empty()) {
        route_ = usable(from, route_, zone, motion);
        if (!route_.empty() && pathSafe(from, route_, zone, motion.walk)) {
            return {route_, true};
        }
        route_.clear();
        search_.reset();
    }
    for (std::size_t i = 0; i < std::min<std::size_t>(5, goals.size()) && Clock::now() < deadline; ++i) {
        const auto& goal = goals[i];
        if (distanceXY(from, goal) <= 19.0 && direct(from, goal, zone, motion)) {
            route_ = {goal};
            return {route_, true};
        }
    }
    if (!search_ || distanceXY(from, search_->from) > 17.0
        || (search_->exhausted && zone.now - search_->startedAt > 1.0)) {
        search_.emplace(from, goals, zone.now);
    }
    auto& search = *search_;
    advance(search, zone, motion, deadline);
    if (!search.complete.empty()) {
        route_ = usable(from, search.complete, zone, motion);
        if (!route_.empty() && pathSafe(from, route_, zone, motion.walk)) {
            return {route_, true};
        }
        route_.clear();
    }
    std::vector<Vec3> partial;
    if (search.best != search.from) {
        partial = usable(from, search.path(search.best), zone, motion);
    }
    if (!search.complete.empty()) {
        search_.reset();
    }
    return {std::move(partial), false};
}

std::optional<Flight> Navigator::flightPlan(const Vec3& from, const std::vector<Vec3>& goals,
                                           const Zone& zone, const Motion& motion, double energy,
                                           double maximum, Clock::time_point deadline) {
    const auto [minX, minY, maxX, maxY] = arena_.bounds;
    std::vector<Vec2> centers{zone.nextCenter};
    for (std::size_t i = 0; i < std::min<std::size_t>(4, goals.size()); ++i) {
        centers.push_back(horizontal(goals[i]));
    }
    int attempts = 0;
    for (const auto& center : centers) {
        for (const auto& offset : offsets) {
            if (Clock::now() >= deadline || attempts >= 12) {
                return std::nullopt;
            }
            const Vec2 xy{center[0] + 6 * offset[0], center[1] + 6 * offset[1]};
            if (xy[0] <= minX || xy[0] >= maxX || xy[1] <= minY || xy[1] >= maxY
                || distanceXY(from, xy) > 55.0 || !zone.inNext(xy)) {
                continue;
            }
            const auto landing = support(xy, motion);
            if (!landing || ((*landing)[2] < arena_.ground(xy[0], xy[1]) + 2.0 && from[2] < (*landing)[2] + 2.0)) {
                continue;
            }
            const Vec2 away = direction(xy, from);
            for (const double shift : {0.0, 5.0, 10.0}) {
                if (Clock::now() >= deadline || attempts >= 12) {
                    return std::nullopt;
                }
                ++attempts;
                const Vec2 startPoint{from[0] + away[0] * shift, from[1] + away[1] * shift};
                const auto takeoff = support(startPoint, motion, from[2] + motion.step);
                if (!takeoff || !direct(from, *takeoff, zone, motion)) {
                    continue;
                }
                const double apex = std::max((*takeoff)[2], (*landing)[2]) + 2.2;
                const double climb = std::max(0.3, (apex - (*takeoff)[2] - 0.6) / std::max(1.0, motion.rise) + 0.22);
                const double across = distanceXY(*takeoff, *landing) / std::max(1.0, motion.walk) + 0.35;
                const double down = std::sqrt(2 * (apex - (*landing)[2]) / motion.gravity) + 0.3;
                const double seconds = climb + across + down;
                // Budget full thrust for climbing and crossing to leave reserve for a recovery.
                if (energy - (climb + across) * motion.jetDrain < 0.2 * maximum) {
                    continue;
                }
                const double at = zone.now + distanceXY(from, *takeoff) / motion.walk;
                if ((at + seconds > zone.end && zone.nextRadius > 0.0)
                    || !zone.segment(*takeoff, *takeoff, at, climb)
                    || !zone.segment(*takeoff, *landing, at + climb, across)
                    || !zone.segment(*landing, *landing, at + climb + across, down)) {
                    continue;
                }
                const Vec3 aboveTakeoff{(*takeoff)[0], (*takeoff)[1], apex};
                const Vec3 aboveLanding{(*landing)[0], (*landing)[1], apex};
                if (arena_.flightSegment(*takeoff, aboveTakeoff, motion.radius, motion.height)
                    && arena_.flightSegment(aboveTakeoff, aboveLanding, motion.radius, motion.height)
                    && arena_.flightSegment(aboveLanding, *landing, motion.radius, motion.height)) {
                    return Flight{*takeoff, *landing, apex, climb, seconds};
                }
            }
        }
    }
    return std::nullopt;
}

bool Navigator::guard(const Vec2& move, bool boost, const Json& me, const Zone& zone,
                       const Motion& motion, bool jet) {
    const Vec3 start = me.at("pos").get<Vec3>();
    const Vec3 velocity = me.at("vel").get<Vec3>();
    const bool air = jet || !me.value("grounded", true);
    const double speed = boost && !air ? motion.boost : motion.walk;
    const double acceleration = air ? motion.airAcceleration : motion.acceleration;
    const Vec2 target{move[0] * speed, move[1] * speed};
    Vec2 step = horizontal(start);
    double vx = velocity[0];
    double vy = velocity[1];
    for (int tick = 0; tick < 2; ++tick) {
        const double dx = target[0] - vx;
        const double dy = target[1] - vy;
        const double delta = std::hypot(dx, dy);
        const double fraction = delta > 0.0 ? std::min(1.0, acceleration * 0.05 / delta) : 0.0;
        vx += dx * fraction;
        vy += dy * fraction;
        const Vec2 next{step[0] + vx * 0.05, step[1] + vy * 0.05};
        if (!zone.segment(step, next, zone.now + tick * 0.05, 0.05)) {
            return false;
        }
        step = next;
    }
    const double stopTime = std::hypot(vx, vy) / std::max(1.0, acceleration);
    const Vec2 stop{step[0] + vx * stopTime / 2, step[1] + vy * stopTime / 2};
    if (!zone.segment(step, stop, zone.now + 0.1, stopTime)) {
        return false;
    }
    if (air) {
        const double az = boost ? motion.jet - motion.gravity : -motion.gravity;
        double z = start[2] + velocity[2] * 0.1 + az * 0.005;
        const double ground = arena_.surface(step[0], step[1], start[2] + 0.05);
        if (std::isfinite(ground)) {
            z = std::max(z, ground);
        }
        return arena_.flightSegment(start, {step[0], step[1], z}, motion.radius, motion.height);
    }
    const auto end = support(step, motion, start[2] + motion.step + 0.1);
    return end && walkable(start, *end, motion);
}

}  // namespace mr
