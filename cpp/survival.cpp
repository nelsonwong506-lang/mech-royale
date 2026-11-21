#include "survival.hpp"

#include "arena.hpp"
#include "navigation.hpp"
#include "tactics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

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

}  // namespace

RotationWindow rotationWindow(const Vec3& from, const Zone& zone, const Motion& motion,
                              std::optional<double> routeSeconds) {
    const double margin = zone.buffer(zone.nextRadius, motion.radius + 3.0);
    const double clearance = zone.clearance(from, zone.now);
    double untilRisk = infinity;
    if (clearance < margin) {
        untilRisk = 0;
    } else if (zone.end > zone.now && zone.clearance(from, zone.end) < margin) {
        // Successive circles are nested, so a point leaves the protected region only once.
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
    // Prefer a complete route estimate when known. Otherwise allow extra distance around buildings.
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
    const Vec3 source{(*origin)[0], (*origin)[1], (*origin)[2] + threat.attack_height * 0.8};
    const auto localWindow = rotationWindow(from, zone, motion);
    if (knownSource && !localWindow.urgent && coverFrom(from, source, motion.height, arena) == 3 &&
        std::all_of(enemies.begin(), enemies.end(), [&](const Vec3& enemy) {
            return distanceXY(from, enemy) > 85 || distanceXY(enemy, *origin) < 0.1 ||
                wallCover(from, enemy, motion.height, arena) == 3;
        })) {
        return RetreatPlan{from, true};
    }

    const double away = std::atan2(from[1] - (*origin)[1], from[0] - (*origin)[0]);
    const double startingSeparation = distanceXY(from, *origin);
    const double distanceToNext = distanceXY(from, zone.nextCenter);
    double bestScore = -infinity;
    std::optional<RetreatPlan> best;
    constexpr std::array offsets{0, 24, -24, 48, -48, 72, -72, 90, -90};
    for (const double length : {8.0, 14.0, 22.0, 4.5}) {
        for (const int offset : offsets) {
            if (Clock::now() >= deadline) {
                return best;
            }
            const double heading = away + radians(offset);
            const Vec2 xy{from[0] + length * std::cos(heading), from[1] + length * std::sin(heading)};
            if (!avoidsOne(from, xy, *origin, length) || !avoidsApproach(from, xy, enemies)) {
                continue;
            }
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

            const bool behindWall = knownSource && coverFrom(*point, source, motion.height, arena) == 3;
            const double gain = distanceXY(*point, *origin) - startingSeparation;
            const double preference = gain - (behindWall ? 1.15 : 0.32) * length +
                (localWindow.urgent ? 0.75 : 0.12) * progress + 0.025 * std::min(40.0, clearance);
            if (!best || (behindWall != best->hardCover ? behindWall : preference > bestScore)) {
                bestScore = preference;
                best = RetreatPlan{*point, behindWall};
            }
        }
    }
    return best;
}

}  // namespace mr
