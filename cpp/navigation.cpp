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
    : now(finiteOr(observedAt, 0.0)),
      center(safeVec2(state, "center")),
      radius(finiteOr(state.value("radius", 0.0), 0.0)),
      nextCenter(safeVec2(state, "next_center", center)),
      nextRadius(finiteOr(state.value("next_radius", radius), radius)),
      start(finiteOr(state.value("shrink_start", now), now)),
      end(finiteOr(state.value("shrink_end", now), now)) {}

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

bool Navigator::isSearching() const {
    return search_ && !search_->exhausted && search_->complete.empty();
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
    for (int expansion = 0; expansion < 120 && !search.queue.empty() && Clock::now() < deadline; ++expansion) {
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
            const double maxClimb = std::max(motion.step * 3.0, std::tan(radians(motion.slope)) * length + 0.2);
            const auto next = support(point, motion, entry.point[2] + maxClimb);
            if (!next || (*next)[2] > entry.point[2] + maxClimb || (*next)[2] < entry.point[2] - 24.0) {
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
    if (!search_ || distanceXY(from, search_->from) > 45.0
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

std::optional<Flight> Navigator::checkedFlight(const Vec3& takeoff, const Vec3& landing, const Motion& motion,
                                               const Zone& zone, double at, Vec2 velocity, double energyAvailable,
                                               Clock::time_point deadline) const {
    if (energyAvailable <= 0 || motion.walk <= 0 || motion.rise <= 0 || motion.gravity <= 0) return {};
    double highPoint = std::max(takeoff[2], landing[2]);
    if (landing[2] < arena_.ground(landing[0], landing[1]) + 2.0) {
        // A ground-to-ground recovery may cross a solid higher than both endpoints.
        const double length = distanceXY(takeoff, landing);
        const Vec2 along = direction(takeoff, landing);
        const int samples = std::max(1, static_cast<int>(std::ceil(length / 2.)));
        for (int n = 1; n < samples; ++n) {
            const double d = length * n / samples;
            for (double offset : {-motion.radius, 0., motion.radius}) {
                highPoint = std::max(highPoint, arena_.surface(takeoff[0] + d*along[0] - offset*along[1],
                                                              takeoff[1] + d*along[1] + offset*along[0]));
            }
        }
    }
    const double apex = highPoint + 2.2;
    const double climb = std::max(0.3, (apex - takeoff[2]) / motion.rise + 0.2);
    const double down = std::sqrt(2 * (apex - landing[2]) / motion.gravity) + 0.3;
    const Vec3 aboveLanding{landing[0], landing[1], apex};
    if (!arena_.flightSegment(aboveLanding, landing, motion.radius, motion.height)) return {};
    const double advance = std::min(distanceXY(takeoff, landing), climb * motion.walk);
    const Vec2 forward = direction(takeoff, landing);
    for (double share : {1., 0.5, 0.}) {
        if (Clock::now() >= deadline) return {};
        const Vec3 toward{takeoff[0] + forward[0]*advance*share, takeoff[1] + forward[1]*advance*share, apex};
        Vec3 p = takeoff;
        Vec2 speed = velocity;
        double vz = motion.jump, elapsed = 0;
        bool clear = true;
        while (p[2] < apex - 0.02 && elapsed < climb + 1.) {
            if (Clock::now() >= deadline) return {};
            constexpr double dt = 0.1;
            const double remaining = distanceXY(p, toward);
            const auto wanted = direction(p, toward, std::min(motion.walk, std::sqrt(2*motion.airAcceleration*remaining)));
            const Vec2 change{wanted[0] - speed[0], wanted[1] - speed[1]};
            const double magnitude = std::hypot(change[0], change[1]);
            const double fraction = magnitude > 0 ? std::min(1., motion.airAcceleration*dt/magnitude) : 0.;
            speed[0] += fraction * change[0];
            speed[1] += fraction * change[1];
            vz = std::min(motion.rise, vz + (motion.jet - motion.gravity) * dt);
            if (vz <= 0) { clear = false; break; }
            const Vec3 next{p[0] + speed[0]*dt, p[1] + speed[1]*dt, std::min(apex, p[2] + vz*dt)};
            if (!arena_.flightSegment(p, next, motion.radius, motion.height) ||
                !zone.segment(p, next, at + elapsed, dt)) { clear = false; break; }
            p = next;
            elapsed += dt;
        }
        if (!clear || p[2] < apex - 0.02) continue;
        const double across = distanceXY(p, landing) / motion.walk + 0.35;
        if ((elapsed + across) * motion.jetDrain > energyAvailable ||
            !zone.segment(p, aboveLanding, at + elapsed, across) ||
            !zone.segment(aboveLanding, landing, at + elapsed + across, down) ||
            !arena_.flightSegment(p, aboveLanding, motion.radius, motion.height)) continue;
        return Flight{takeoff, landing, apex, elapsed, elapsed + across + down, false, toward};
    }
    return {};
}

std::optional<Flight> Navigator::flightPlan(const Vec3& from, const std::vector<Vec3>& goals,
                                           const Zone& zone, const Motion& motion, double energy,
                                           double maximum, Clock::time_point deadline, Vec2 velocity) {
    const auto [minX, minY, maxX, maxY] = arena_.bounds;
    const double startToNext = distanceXY(from, zone.nextCenter);
    std::vector<Vec2> centers;
    centers.reserve(9);
    if (startToNext <= 62.0) {
        centers.push_back(zone.nextCenter);
    }
    for (std::size_t i = 0; i < std::min<std::size_t>(4, goals.size()); ++i) {
        if (distanceXY(from, goals[i]) <= 62.0) {
            centers.push_back(horizontal(goals[i]));
        }
    }
    // Also offer intermediate landings toward nextCenter so a rooftop > 55 m away can be escaped.
    const Vec2 towardNext = direction(from, zone.nextCenter);
    if (std::hypot(towardNext[0], towardNext[1]) > 0.1) {
        for (const double dist : {20.0, 35.0, 48.0}) {
            if (dist < startToNext) {
                centers.push_back({from[0] + towardNext[0] * dist, from[1] + towardNext[1] * dist});
            }
        }
    }
    int attempts = 0;
    for (const auto& center : centers) {
        for (const auto& offset : offsets) {
            if (Clock::now() >= deadline || attempts >= 20) {
                return std::nullopt;
            }
            const Vec2 xy{center[0] + 6 * offset[0], center[1] + 6 * offset[1]};
            const bool insideNext = zone.inNext(xy);
            const double arrivalClearance = zone.clearance(xy, zone.now + 3.);
            const bool intermediateSafe = distanceXY(xy, zone.nextCenter) < startToNext - 4.0 &&
                (arrivalClearance >= 0. || (zone.clearance(from, zone.now) < 0. &&
                 arrivalClearance > zone.clearance(from, zone.now + 3.) + 4.));
            if (xy[0] <= minX || xy[0] >= maxX || xy[1] <= minY || xy[1] >= maxY
                || distanceXY(from, xy) > 55.0 || (!insideNext && !intermediateSafe)) {
                continue;
            }
            const auto landing = support(xy, motion);
            if (!landing) continue;
            const bool onRoof = (*landing)[2] > arena_.ground(xy[0], xy[1]) + 2.;
            if ((onRoof && !arena_.roofFits(*landing, motion.radius, motion.height)) ||
                (!onRoof && from[2] < (*landing)[2] + 2. && walkable(from, *landing, motion))) {
                continue;
            }
            const Vec2 away = direction(xy, from);
            for (const double shift : {0.0, 5.0, 10.0}) {
                if (Clock::now() >= deadline || attempts >= 20) {
                    return std::nullopt;
                }
                ++attempts;
                const Vec2 startPoint{from[0] + away[0] * shift, from[1] + away[1] * shift};
                const auto takeoff = support(startPoint, motion, from[2] + motion.step);
                if (!takeoff || !direct(from, *takeoff, zone, motion)) {
                    continue;
                }
                const double at = zone.now + distanceXY(from, *takeoff) / motion.walk;
                const auto flight = checkedFlight(*takeoff, *landing, motion, zone, at, velocity,
                                                  energy - 0.2 * maximum, deadline);
                if (flight && !(insideNext && at + flight->seconds > zone.end && zone.nextRadius > 0.0))
                    return flight;
            }
        }
    }
    return std::nullopt;
}

std::optional<Flight> Navigator::flightToRoof(const Vec3& from, const Vec3& landing, const Zone& zone,
                                              const Motion& motion, double energy, double maximum,
                                              Clock::time_point deadline, Vec2 velocity) {
    if (Clock::now() >= deadline || distanceXY(from, landing) > 45.0) {
        return std::nullopt;
    }
    if (!arena_.roofFits(landing, motion.radius, motion.height)) return {};
    if (zone.clearance(landing, zone.now + 10.0) < motion.radius + 1.0 && !zone.inNext(landing, motion.radius)) {
        return std::nullopt;
    }
    const Vec2 away = direction(landing, from);
    for (const double shift : {0.0, 4.5, 9.0}) {
        if (Clock::now() >= deadline) {
            return std::nullopt;
        }
        const Vec2 startPoint{from[0] + away[0] * shift, from[1] + away[1] * shift};
        const auto takeoff = support(startPoint, motion, from[2] + motion.step);
        if (!takeoff || !direct(from, *takeoff, zone, motion)) {
            continue;
        }
        const double at = zone.now + distanceXY(from, *takeoff) / motion.walk;
        const auto flight = checkedFlight(*takeoff, landing, motion, zone, at, velocity, energy - 0.18 * maximum, deadline);
        if (flight) return flight;
    }
    return std::nullopt;
}

bool Navigator::guard(const Vec2& move, bool boost, const Json& me, const Zone& zone,
                       const Motion& motion, bool jet) {
    const Vec3 start = safeVec3(me, "pos");
    const Vec3 velocity = safeVec3(me, "vel");
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
