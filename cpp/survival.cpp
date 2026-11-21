#include "survival.hpp"

#include "arena.hpp"
#include "navigation.hpp"
#include "tactics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace mr {
namespace {

constexpr double rotationReserve = 7.0;
constexpr double infinity = std::numeric_limits<double>::infinity();

int coverFrom(const Vec3& feet, const Vec3& source, double height, const Arena& arena) {
    int protectedPoints = 0;
    for (const double fraction : {0.22, 0.54, 0.9}) {
        const Vec3 sample{feet[0], feet[1], feet[2] + height * fraction};
        protectedPoints += arena.blocksPrismLine(source, sample);
    }
    return protectedPoints;
}

bool avoidsOne(const Vec3& from, const Vec2& toward, const Vec3& enemy, double travel) {
    const double dx = enemy[0] - from[0], dy = enemy[1] - from[1];
    const double initialDistance = std::hypot(dx, dy);
    const double closing = (toward[0] - from[0]) * dx + (toward[1] - from[1]) * dy;
    return closing <= 0.05 * initialDistance * travel && distanceXY(toward, enemy) + 0.05 >= initialDistance;
}

std::optional<DefensiveHop> traceHop(const Vec3& start, const Vec3& initialVelocity, const Vec2& launch,
                                    const Vec3& primaryOrigin, bool strictAvoid, const Zone& zone,
                                    const Motion& motion, const Arena& arena, Navigator& navigator,
                                    double steerAfter, Clock::time_point deadline) {
    constexpr double tickSeconds = 0.05;
    const int ticks = static_cast<int>(std::min(1.6, 2 * motion.jump / motion.gravity + 0.35) / tickSeconds);
    Vec3 position = start;
    Vec3 velocity{initialVelocity[0], initialVelocity[1], motion.jump};
    for (int tick = 0; tick < ticks; ++tick) {
        if (Clock::now() >= deadline) {
            return std::nullopt;
        }
        const double at = zone.now + tick * tickSeconds;
        const double pace = tick * tickSeconds + 1e-6 >= steerAfter ? motion.walk : 0.0;
        const Vec2 desiredVelocity{launch[0] * pace, launch[1] * pace};
        const double dx = desiredVelocity[0] - velocity[0], dy = desiredVelocity[1] - velocity[1];
        const double error = std::hypot(dx, dy);
        const double correction = error > 0 ? std::min(1.0, motion.airAcceleration * tickSeconds / error) : 0;
        velocity[0] += dx * correction;
        velocity[1] += dy * correction;
        velocity[2] -= motion.gravity * tickSeconds;
        Vec3 next{position[0] + velocity[0] * tickSeconds, position[1] + velocity[1] * tickSeconds,
                  position[2] + velocity[2] * tickSeconds};
        const bool falling = velocity[2] < 0;
        const double support = falling ? arena.surface(next[0], next[1], position[2] + 0.05) : -infinity;
        const bool landed = falling && next[2] <= support;
        if (landed) {
            next[2] = support;
        }
        const double stepTravel = distanceXY(position, next);
        if (zone.clearance(next, at + tickSeconds) < 0 ||
            !zone.segment(position, next, at, tickSeconds) ||
            (strictAvoid && !avoidsOne(position, horizontal(next), primaryOrigin, stepTravel)) ||
            !arena.flightSegment(position, next, motion.radius, motion.height + 0.12)) {
            return std::nullopt;
        }
        if (landed) {
            const double brakingTime = std::hypot(velocity[0], velocity[1]) / std::max(1.0, motion.acceleration);
            const Vec2 stop{next[0] + velocity[0] * brakingTime * 0.5,
                            next[1] + velocity[1] * brakingTime * 0.5};
            const auto supportedStop = navigator.support(stop, motion, next[2] + motion.step + 0.08);
            const double arrival = at + tickSeconds;
            const double margin = zone.buffer(zone.circle(arrival).second, motion.radius + 0.25);
            if (start[2] - next[2] > 4.0 || distanceXY(start, next) < 2.0 ||
                zone.clearance(next, arrival) < margin || !supportedStop ||
                !zone.segment(next, *supportedStop, arrival, brakingTime) ||
                !navigator.walkable(next, *supportedStop, motion)) {
                return std::nullopt;
            }
            return DefensiveHop{next, launch, (tick + 1) * tickSeconds, steerAfter};
        }
        position = next;
    }
    return std::nullopt;
}

}  // namespace

RotationWindow rotationWindow(const Vec3& from, const Zone& zone, const Motion& motion,
                              std::optional<double> routeSeconds) {
    const double margin = zone.buffer(zone.nextRadius, motion.radius + 3.0);
    const double clearance = zone.clearance(from, zone.now);
    double untilRisk = infinity;
    if (clearance < margin) {
        untilRisk = 0;
    } else if (zone.end > zone.now && zone.clearance(from, zone.end) < margin) {
        double low = zone.now, high = zone.end;
        for (int i = 0; i < 20; ++i) {
            const double midpoint = (low + high) * 0.5;
            if (zone.clearance(from, midpoint) >= margin) {
                low = midpoint;
            } else {
                high = midpoint;
            }
        }
        untilRisk = high - zone.now;
    }

    const double inset = zone.buffer(zone.nextRadius, motion.radius + 6.0);
    const double directDistance = std::max(0.0, distanceXY(from, zone.nextCenter) - (zone.nextRadius - inset));
    const double directTravel = directDistance / std::max(1.0, motion.walk);
    const double travel = routeSeconds && std::isfinite(*routeSeconds) && *routeSeconds >= 0.0
        ? std::max(directTravel, *routeSeconds) : 1.45 * directTravel;
    return {untilRisk <= travel + rotationReserve, untilRisk, travel};
}

bool avoidsApproach(const Vec3& from, const Vec2& toward, const std::vector<Vec3>& enemies) {
    const double travel = distanceXY(from, toward);
    return std::all_of(enemies.begin(), enemies.end(), [&](const Vec3& enemy) {
        return avoidsOne(from, toward, enemy, travel);
    });
}

double approachPenalty(const Vec3& from, const Vec2& toward, const std::vector<Vec3>& enemies) {
    const double travel = distanceXY(from, toward);
    if (travel <= 1e-6 || enemies.empty()) {
        return 0.0;
    }
    double penalty = 0.0;
    for (const auto& enemy : enemies) {
        const double initial = distanceXY(from, enemy);
        if (initial < 0.1) {
            continue;
        }
        const double after = distanceXY(toward, enemy);
        const double closure = std::max(0.0, initial - after);
        if (closure > 0.0) {
            // Closer enemies exert much stronger directional repulsion.
            penalty += closure * (40.0 / std::max(8.0, initial));
        }
    }
    return penalty;
}

int wallCover(const Vec3& feet, const Vec3& threat, double ownHeight, const Arena& arena) {
    const Vec3 source{threat[0], threat[1], threat[2] + 7.0};
    return coverFrom(feet, source, ownHeight, arena);
}

std::optional<RetreatPlan> planRetreat(const Vec3& from, const Threat& threat, const std::vector<Vec3>& enemies,
                                       const Zone& zone, const Motion& motion, const Arena& arena,
                                       Navigator& navigator, Clock::time_point deadline) {
    const auto origin = threat.attack_position ? threat.attack_position : threat.position;
    if (!origin || zone.clearance(from, zone.now) < 0 || Clock::now() >= deadline) {
        return std::nullopt;
    }

    const bool knownSource = threat.attack_position ? threat.attack_id.has_value() : threat.actor_id.has_value();
    const Vec3 source{(*origin)[0], (*origin)[1], (*origin)[2] + threat.enemy_muzzle_height};
    const auto localWindow = rotationWindow(from, zone, motion);
    if (knownSource && !localWindow.urgent && coverFrom(from, source, motion.height, arena) == 3 &&
        std::all_of(enemies.begin(), enemies.end(), [&](const Vec3& enemy) {
            return distanceXY(from, enemy) > 230.0 || distanceXY(enemy, *origin) < 0.1 ||
                wallCover(from, enemy, motion.height, arena) == 3;
        })) {
        return RetreatPlan{from, true};
    }

    const double away = std::atan2(from[1] - (*origin)[1], from[0] - (*origin)[0]);
    const double startingSeparation = distanceXY(from, *origin);
    const double distanceToNext = distanceXY(from, zone.nextCenter);
    double bestScore = -infinity;
    std::optional<RetreatPlan> best;
    constexpr std::array offsets{0, 24, -24, 48, -48, 72, -72, 90, -90, 115, -115};
    for (const double length : {8.0, 14.0, 20.0, 4.5}) {
        for (const int offset : offsets) {
            if (Clock::now() >= deadline) {
                return best;
            }
            const double heading = away + radians(offset);
            const Vec2 xy{from[0] + length * std::cos(heading), from[1] + length * std::sin(heading)};
            const double arrival = zone.now + length / std::max(1.0, motion.walk) + 0.35;
            const double arrivalRadius = zone.circle(arrival).second;
            const double clearance = zone.clearance(xy, arrival);
            if (clearance < zone.buffer(arrivalRadius, motion.radius + 0.8)) {
                continue;
            }
            const double progress = distanceToNext - distanceXY(xy, zone.nextCenter);
            if (localWindow.urgent && progress < 0.8 && !zone.inNext(xy, motion.radius + 1.0)) {
                continue;
            }
            const auto point = navigator.support(xy, motion, from[2] + motion.step + 0.05);
            if (!point || !navigator.direct(from, *point, zone, motion)) {
                continue;
            }

            const bool primaryCovered = knownSource && coverFrom(*point, source, motion.height, arena) == 3;
            int exposedOthers = 0;
            for (const auto& enemy : enemies) {
                if (distanceXY(enemy, *origin) < 0.5 || distanceXY(*point, enemy) > 160.0) {
                    continue;
                }
                if (wallCover(*point, enemy, motion.height, arena) < 3) {
                    ++exposedOthers;
                }
            }
            const bool behindWall = primaryCovered && exposedOthers == 0;
            const double gain = distanceXY(*point, *origin) - startingSeparation;
            // Allow stepping toward cover even if slightly oblique, but avoid running straight into the threat without cover.
            if (!behindWall && gain < -0.35 * length) {
                continue;
            }
            const double penalty = approachPenalty(from, xy, enemies);
            const double preference = (primaryCovered ? 8.0 : 0.0) - 4.0 * exposedOthers + gain -
                (behindWall ? 0.9 : 0.28) * length - 0.55 * penalty +
                (localWindow.urgent ? 0.95 : 0.35) * progress + 0.03 * std::min(40.0, clearance);
            if (!best || (behindWall != best->hardCover ? behindWall : preference > bestScore)) {
                bestScore = preference;
                best = RetreatPlan{*point, behindWall};
            }
        }
    }
    return best;
}

std::optional<Vec3> planEngageStep(const Vec3& from, const Threat& threat, const std::vector<Vec3>& enemies,
                                   const Zone& zone, const Motion& motion, const Arena& arena,
                                   Navigator& navigator, double desiredRange, double strafeSign,
                                   double muzzleHeight, Clock::time_point deadline) {
    const auto origin = threat.attack_position ? threat.attack_position : threat.position;
    if (!origin || zone.clearance(from, zone.now) < 0 || Clock::now() >= deadline) {
        return std::nullopt;
    }
    const double toEnemy = std::atan2((*origin)[1] - from[1], (*origin)[0] - from[0]);
    const double distanceToNext = distanceXY(from, zone.nextCenter);
    const Vec3 enemyTorso{(*origin)[0], (*origin)[1], (*origin)[2] + threat.attack_height * 0.5};

    double bestScore = -infinity;
    std::optional<Vec3> best;
    // Prefer perpendicular strafing (keeps moving across enemy aim) while tuning combat range.
    const std::array offsets{
        static_cast<int>(strafeSign * 85), static_cast<int>(strafeSign * 65),
        static_cast<int>(strafeSign * 110), static_cast<int>(-strafeSign * 85),
        static_cast<int>(-strafeSign * 65), static_cast<int>(-strafeSign * 110),
        static_cast<int>(strafeSign * 40), static_cast<int>(-strafeSign * 40),
        static_cast<int>(strafeSign * 135), static_cast<int>(-strafeSign * 135),
        0, 180
    };
    for (const double length : {8.0, 13.0, 5.0}) {
        for (const int offset : offsets) {
            if (Clock::now() >= deadline) {
                return best;
            }
            const double heading = toEnemy + radians(offset);
            const Vec2 xy{from[0] + length * std::cos(heading), from[1] + length * std::sin(heading)};
            const double arrival = zone.now + length / std::max(1.0, motion.walk) + 0.35;
            const double clearance = zone.clearance(xy, arrival);
            if (clearance < zone.buffer(zone.circle(arrival).second, motion.radius + 1.5)) {
                continue;
            }
            const auto point = navigator.support(xy, motion, from[2] + motion.step + 0.1);
            if (!point || !navigator.direct(from, *point, zone, motion)) {
                continue;
            }
            const double nextDist = distanceXY(*point, *origin);
            if (nextDist < 12.0) {
                continue;
            }
            const Vec3 muzzlePos{(*point)[0], (*point)[1], (*point)[2] + muzzleHeight};
            const bool hasLos = !arena.blocksLine(muzzlePos, enemyTorso);
            const double rangeError = std::abs(nextDist - desiredRange);
            const double lateral = std::abs(std::sin(radians(offset)));
            const bool preferredSide = (offset * strafeSign) > 0;
            const double progress = distanceToNext - distanceXY(xy, zone.nextCenter);
            // Penalize walking toward any OTHER nearby enemy.
            double otherPenalty = 0.0;
            for (const auto& other : enemies) {
                if (distanceXY(other, *origin) < 0.5) continue;
                const double before = distanceXY(from, other);
                const double after = distanceXY(xy, other);
                if (after < before) {
                    otherPenalty += (before - after) * (40.0 / std::max(10.0, before));
                }
            }
            const double score = (hasLos ? 28.0 : -15.0) + 7.5 * lateral + (preferredSide ? 2.5 : 0.0) -
                0.55 * rangeError + 0.25 * progress - 0.8 * otherPenalty +
                0.02 * std::min(40.0, clearance);
            if (score > bestScore) {
                bestScore = score;
                best = *point;
            }
        }
    }
    return best;
}

std::optional<DefensiveHop> planDefensiveHop(const Json& me, const Threat& threat,
                                             const std::vector<Vec3>& enemies, const Zone& zone,
                                             const Motion& motion, const Arena& arena, Navigator& navigator,
                                             bool preferLateral, bool liftFirst, Clock::time_point deadline) {
    const auto origin = threat.attack_position ? threat.attack_position : threat.position;
    const auto start = safeVec3(me, "pos");
    if (!origin || !me.value("grounded", true) || zone.clearance(start, zone.now) < 0 ||
        motion.jump <= 0 || motion.gravity <= 0 || Clock::now() >= deadline) {
        return std::nullopt;
    }
    const auto velocity = safeVec3(me, "vel");
    const double away = std::atan2(start[1] - (*origin)[1], start[0] - (*origin)[0]);
    const double aimYaw = threat.enemy_aim ? (*threat.enemy_aim)[0] : away;
    const double separation = distanceXY(start, *origin);
    const double nextDistance = distanceXY(start, zone.nextCenter);
    const bool urgent = rotationWindow(start, zone, motion).urgent;
    const bool knownSource = threat.attack_position ? threat.attack_id.has_value() : threat.actor_id.has_value();
    const Vec3 source{(*origin)[0], (*origin)[1], (*origin)[2] + threat.enemy_muzzle_height};
    constexpr std::array lateralOffsets{90, -90, 74, -74, 55, -55, 35, -35, 0};
    constexpr std::array retreatOffsets{36, -36, 60, -60, 0, 18, -18, 90, -90};
    const auto& offsets = preferLateral ? lateralOffsets : retreatOffsets;
    double bestScore = -infinity;
    std::optional<DefensiveHop> best;
    for (const auto& [pace, delay] : {std::pair{1.0, 0.0}, std::pair{1.0, 0.3}, std::pair{0.66, 0.0}}) {
        if (delay > 0 && !liftFirst) continue;
        for (const int offset : offsets) {
            if (Clock::now() >= deadline) {
                return best;
            }
            const double yaw = away + radians(offset);
            const Vec2 launch{pace * std::cos(yaw), pace * std::sin(yaw)};
            const auto hop = traceHop(start, velocity, launch, *origin, !preferLateral, zone, motion, arena, navigator, delay, deadline);
            if (!hop) {
                continue;
            }
            const double progress = nextDistance - distanceXY(hop->landing, zone.nextCenter);
            if (urgent && progress < 0.25 && !zone.inNext(hop->landing, motion.radius)) {
                continue;
            }
            const double dx = hop->landing[0] - start[0], dy = hop->landing[1] - start[1];
            const double lateral = std::abs(dy * std::cos(aimYaw) - dx * std::sin(aimYaw));
            if (preferLateral && lateral < 0.58 * std::hypot(dx, dy)) {
                continue;
            }
            const bool cover = knownSource && coverFrom(hop->landing, source, motion.height, arena) == 3;
            const double gain = distanceXY(hop->landing, *origin) - separation;
            const double penalty = approachPenalty(start, horizontal(hop->landing), enemies);
            const double score = (cover ? 26.0 : 0.0) + (preferLateral ? 2.2 : 0.38) * lateral +
                0.72 * gain + (urgent ? 0.8 : 0.08) * progress - (preferLateral ? 0.15 : 0.45) * penalty;
            if (score > bestScore) {
                bestScore = score;
                best = hop;
            }
        }
    }
    return best;
}

}  // namespace mr
