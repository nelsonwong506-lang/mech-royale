#include "survival.hpp"

#include "arena.hpp"
#include "navigation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace mr {
namespace {

constexpr double rotationReserve = 7.0;
constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr double tau = 6.283185307179586;

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

// Compute the minimum angular span covering a set of bearings and its center yaw.
std::pair<double, double> angularCoverage(std::vector<double>& yaws) {
    if (yaws.empty()) {
        return {0.0, 0.0};
    }
    if (yaws.size() == 1) {
        return {0.0, yaws[0]};
    }
    std::sort(yaws.begin(), yaws.end());
    double maxGap = -1.0;
    std::size_t gapAfter = 0;
    for (std::size_t i = 0; i < yaws.size(); ++i) {
        const double next = (i + 1 < yaws.size()) ? yaws[i + 1] : (yaws[0] + tau);
        const double gap = next - yaws[i];
        if (gap > maxGap) {
            maxGap = gap;
            gapAfter = i;
        }
    }
    const double span = std::max(0.0, tau - maxGap);
    const double startYaw = yaws[(gapAfter + 1) % yaws.size()];
    const double centerYaw = std::remainder(startYaw + span * 0.5, tau);
    return {span, centerYaw};
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
            penalty += closure * (40.0 / std::max(8.0, initial));
        }
    }
    return penalty;
}

int wallCover(const Vec3& feet, const Vec3& threat, double ownHeight, const Arena& arena) {
    const Vec3 source{threat[0], threat[1], threat[2] + 7.0};
    return coverFrom(feet, source, ownHeight, arena);
}

std::optional<double> frontArmorYaw(const Vec3& from, const std::vector<EnemyInfo>& enemies,
                                    double ownHeight, const Arena& arena) {
    // Point our 180-deg front armor arc at the centroid of exposed close threats to avoid 1.5x rear damage.
    double sumX = 0.0, sumY = 0.0;
    for (const auto& e : enemies) {
        const double dist = distanceXY(from, e.pos);
        if (dist > 180.0 || wallCover(from, e.pos, ownHeight, arena) == 3) {
            continue;
        }
        const double weight = (e.facing_us ? 2.5 : 1.0) * (e.charging ? 2.0 : 1.0) / std::max(10.0, dist);
        const double yaw = std::atan2(e.pos[1] - from[1], e.pos[0] - from[0]);
        sumX += weight * std::cos(yaw);
        sumY += weight * std::sin(yaw);
    }
    if (std::hypot(sumX, sumY) < 1e-6) {
        return std::nullopt;
    }
    return std::atan2(sumY, sumX);
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

std::optional<CombatMove> planCombatPosition(const Vec3& from, const Threat& primary,
                                             const std::vector<EnemyInfo>& enemies,
                                             const CombatContext& ctx, const Zone& zone,
                                             const Motion& motion, const Arena& arena,
                                             Navigator& navigator, Clock::time_point deadline) {
    const auto origin = primary.attack_position ? primary.attack_position : primary.position;
    if (!origin || zone.clearance(from, zone.now) < 0 || Clock::now() >= deadline) {
        return std::nullopt;
    }

    // Filter enemies within tactical combat radius (<= 160 m) for fast multi-enemy geometry evaluation.
    std::vector<const EnemyInfo*> nearby;
    nearby.reserve(enemies.size());
    int closeCount = 0;
    for (const auto& e : enemies) {
        const double d = distanceXY(from, e.pos);
        if (d <= 165.0) {
            nearby.push_back(&e);
            if (d <= 90.0) {
                ++closeCount;
            }
        }
    }
    const bool multiEnemy = closeCount >= 2;
    const double hpFrac = ctx.hp / std::max(1.0, ctx.maxHp);
    const bool enemyFinishable = primary.hp <= 420.0 && hpFrac >= 0.25;
    const bool defensive = ctx.wantRegen || ctx.reloading || (hpFrac < 0.38 && !enemyFinishable);

    // Choose optimal combat distance based on OUR equipped weapons and enemy loadout.
    double optimalRange = 42.0;
    if (ctx.hasLightning && ctx.energy >= 18.0 && !primary.has_mortar) {
        optimalRange = enemyFinishable ? 24.0 : 31.0;
    } else if (ctx.hasLaser && !ctx.hasLightning) {
        optimalRange = 75.0;
    } else if (ctx.hasMortar && !ctx.hasLightning) {
        optimalRange = 55.0;
    } else if (enemyFinishable) {
        optimalRange = 28.0;
    }
    if (primary.has_lightning && !(ctx.hasLightning && enemyFinishable)) {
        optimalRange = std::max(optimalRange, 52.0);
    }

    // Pick our focus target (weakest close enemy to isolate in 1vMany cover-splitting).
    Vec3 focusPos = *origin;
    double focusHeight = primary.attack_height;
    int focusId = primary.actor_id.value_or(-1);
    if (multiEnemy && !defensive) {
        double bestFocus = -infinity;
        for (const auto* e : nearby) {
            const double d = distanceXY(from, e->pos);
            if (d > 110.0) continue;
            const double s = (1400.0 - std::min(1400.0, e->hp)) * 0.25 - d + (e->charging ? 120.0 : 0.0);
            if (s > bestFocus) {
                bestFocus = s;
                focusPos = e->pos;
                focusHeight = e->height;
                focusId = e->id;
            }
        }
    }
    const Vec3 focusTorso{focusPos[0], focusPos[1], focusPos[2] + focusHeight * 0.5};

    // Compute pinch escape vector (away from enemy centroid) so we break out of flanks.
    double enemySumX = 0.0, enemySumY = 0.0, enemyWeightSum = 0.0;
    for (const auto* e : nearby) {
        const double d = std::max(8.0, distanceXY(from, e->pos));
        const double w = (d <= 95.0 ? 2.5 : 1.0) / d;
        enemySumX += w * e->pos[0];
        enemySumY += w * e->pos[1];
        enemyWeightSum += w;
    }
    const Vec2 centroid = enemyWeightSum > 0.0
        ? Vec2{enemySumX / enemyWeightSum, enemySumY / enemyWeightSum}
        : horizontal(focusPos);
    const double escapeYaw = std::atan2(from[1] - centroid[1], from[0] - centroid[0]);
    const double toFocus = std::atan2(focusPos[1] - from[1], focusPos[0] - from[0]);
    const double distanceToNext = distanceXY(from, zone.nextCenter);

    // Check if staying put in hard cover is ideal (during regen or reload).
    if (defensive && !ctx.needReposition) {
        int exposedHere = 0;
        for (const auto* e : nearby) {
            if (wallCover(from, e->pos, motion.height, arena) < 3) {
                ++exposedHere;
            }
        }
        if (exposedHere == 0 && zone.clearance(from, zone.now + 1.5) > motion.radius + 1.5) {
            return CombatMove{from, false, true, 0, ctx.wantRegen ? "regen-cover" : "reload-cover"};
        }
    }

    // Build candidate offsets anchored on focus direction + pinch escape direction.
    const std::array<int, 12> baseOffsets{
        static_cast<int>(ctx.strafeSign * 85), static_cast<int>(ctx.strafeSign * 60),
        static_cast<int>(ctx.strafeSign * 115), static_cast<int>(-ctx.strafeSign * 85),
        static_cast<int>(-ctx.strafeSign * 60), static_cast<int>(-ctx.strafeSign * 115),
        static_cast<int>(ctx.strafeSign * 145), static_cast<int>(-ctx.strafeSign * 145),
        180, static_cast<int>(ctx.strafeSign * 35), static_cast<int>(-ctx.strafeSign * 35), 0
    };

    double bestScore = -infinity;
    std::optional<CombatMove> best;
    std::vector<double> exposedYaws;
    exposedYaws.reserve(nearby.size());

    auto evaluateCandidate = [&](const Vec2& xy, double length, bool preferredSide, double lateral) {
        const double arrival = zone.now + length / std::max(1.0, motion.walk) + 0.35;
        const double clearance = zone.clearance(xy, arrival);
        if (clearance < zone.buffer(zone.circle(arrival).second, motion.radius + 1.2)) {
            return;
        }
        const auto point = navigator.support(xy, motion, from[2] + motion.step + 0.1);
        if (!point || !navigator.direct(from, *point, zone, motion)) {
            return;
        }

        int exposedCount = 0;
        int lightningCluster = 0;
        double minEnemyDist = infinity;
        double weaponDangerPenalty = 0.0;
        exposedYaws.clear();
        for (const auto* e : nearby) {
            const double d = distanceXY(*point, e->pos);
            minEnemyDist = std::min(minEnemyDist, d);
            const bool exposed = wallCover(*point, e->pos, motion.height, arena) < 3;
            if (exposed) {
                ++exposedCount;
                exposedYaws.push_back(std::atan2(e->pos[1] - (*point)[1], e->pos[0] - (*point)[0]));
                if (e->has_lightning && d < 47.0 && !(ctx.hasLightning && e->hp < 350.0)) {
                    weaponDangerPenalty += (47.0 - d) * 1.3;
                }
                if (e->charging) {
                    weaponDangerPenalty += 30.0;
                }
            }
            if (ctx.hasLightning && d <= 42.0 && distanceXY(e->pos, focusPos) <= 14.0) {
                ++lightningCluster;
            }
        }
        if (minEnemyDist < 10.0) {
            return;
        }

        // Front-armor protection: penalize positions where exposed enemies span > 150 deg (1.5x rear-armor risk!).
        const auto [span, centerYaw] = angularCoverage(exposedYaws);
        (void)centerYaw;
        double rearRiskPenalty = 0.0;
        if (exposedCount >= 2 && span > radians(145.0)) {
            rearRiskPenalty = 24.0 + 18.0 * (span - radians(145.0));
        }

        const Vec3 muzzlePos{(*point)[0], (*point)[1], (*point)[2] + ctx.muzzleHeight};
        const bool hasFocusLos = !arena.blocksLine(muzzlePos, focusTorso);
        const double focusDist = distanceXY(*point, focusPos);
        const double rangeError = std::abs(focusDist - optimalRange);
        const double progress = distanceToNext - distanceXY(xy, zone.nextCenter);
        const double anchorShift = distanceXY(*point, ctx.anchor);
        const double repositionBonus = ctx.needReposition ? std::min(12.0, anchorShift * 0.65) : 0.0;

        double exposureScore = 0.0;
        const char* label = "engage";
        if (defensive) {
            // Regen or reload: strongly prefer full cover (0 exposed) or breaking LOS.
            if (exposedCount == 0) {
                exposureScore = 48.0;
                label = ctx.wantRegen ? "regen-cover" : "reload-cover";
            } else if (exposedCount == 1) {
                exposureScore = 10.0;
                label = "kite-defend";
            } else {
                exposureScore = -24.0 * exposedCount;
                label = "split-out";
            }
        } else if (multiEnemy) {
            // 1vMany core tactic: isolate EXACTLY 1 exposed enemy via cover-splitting!
            if (exposedCount == 1 && hasFocusLos) {
                exposureScore = 44.0;
                label = "split-1v1";
            } else if (exposedCount == 0) {
                exposureScore = ctx.hasMortar ? 32.0 : 18.0;
                label = ctx.hasMortar ? "mortar-cover" : "reposition";
            } else if (exposedCount == 2 && ctx.hasLightning && ctx.energy >= 20.0 &&
                       hpFrac >= 0.55 && lightningCluster >= 2 && span <= radians(115.0)) {
                // Lightning chains across both enemies inside front armor arc!
                exposureScore = 26.0;
                label = "chain-kite";
            } else {
                exposureScore = -22.0 * (exposedCount - 1) + (hasFocusLos ? 6.0 : -8.0);
                label = "kite-multi";
            }
        } else {
            // 1v1: keep LOS, kite laterally at optimal range, or use cover with mortar.
            if (hasFocusLos) {
                exposureScore = 28.0;
                label = "engage";
            } else if (ctx.hasMortar && exposedCount == 0 && focusDist >= 18.0) {
                exposureScore = 24.0;
                label = "mortar-cover";
            } else {
                exposureScore = -12.0;
                label = "flank";
            }
        }

        // Escape-pinch alignment bonus when outnumbered or defensive.
        const double moveYaw = std::atan2(xy[1] - from[1], xy[0] - from[0]);
        const Vec2 moveDir{std::cos(moveYaw), std::sin(moveYaw)};
        const double escapeAlign = std::cos(moveYaw - escapeYaw);
        const double pinchBonus = (multiEnemy || defensive) ? 8.5 * escapeAlign : 0.0;

        // Momentum continuity: smooth high-speed circular strafing instead of left-right jitter.
        const double velSpeed = std::hypot(ctx.velocity[0], ctx.velocity[1]);
        const double momentumBonus = velSpeed > 1.5
            ? 3.8 * (moveDir[0] * ctx.velocity[0] + moveDir[1] * ctx.velocity[1]) / velSpeed
            : 0.0;

        // Incoming bullet sidestep bias.
        const double dodgeBonus = ctx.bulletDodge
            ? 14.0 * (moveDir[0] * (*ctx.bulletDodge)[0] + moveDir[1] * (*ctx.bulletDodge)[1])
            : 0.0;

        // Rooftop high-ground retention: don't accidentally step off a roof while fighting ground bots!
        const bool staysElevated = ctx.onRoof && (*point)[2] >= from[2] - 1.5;
        const double roofBonus = ctx.onRoof ? (staysElevated ? 22.0 : -26.0) : 0.0;

        // Penalize walking closer to non-focus enemies.
        double otherApproachPenalty = 0.0;
        for (const auto* e : nearby) {
            if (e->id == focusId) continue;
            const double before = distanceXY(from, e->pos);
            const double after = distanceXY(xy, e->pos);
            if (after < before) {
                otherApproachPenalty += (before - after) * (35.0 / std::max(10.0, before));
            }
        }

        const double score = exposureScore + pinchBonus + repositionBonus + momentumBonus + dodgeBonus + roofBonus +
            (defensive ? 3.5 : 8.0) * lateral + (preferredSide ? 2.5 : 0.0) -
            (defensive ? 0.15 : 0.45) * rangeError - weaponDangerPenalty - rearRiskPenalty -
            0.55 * otherApproachPenalty + 0.32 * progress + 0.025 * std::min(40.0, clearance);

        if (score > bestScore) {
            bestScore = score;
            const double energyReserve = ctx.hasLightning ? 0.45 : 0.30;
            const bool wantBoost = !ctx.laserCharging &&
                ctx.energy > energyReserve * ctx.maxEnergy &&
                (multiEnemy || defensive || (primary.facing_us && focusDist < 55.0) || ctx.needReposition);
            best = CombatMove{*point, wantBoost, exposedCount == 0, exposedCount, label};
        }
    };

    for (const double length : {9.0, 15.0, 21.0, 5.5}) {
        for (const int offset : baseOffsets) {
            if (Clock::now() >= deadline) {
                return best;
            }
            const double heading = toFocus + radians(offset);
            const Vec2 xy{from[0] + length * std::cos(heading), from[1] + length * std::sin(heading)};
            const double lateral = std::abs(std::sin(radians(offset)));
            const bool preferredSide = (offset * ctx.strafeSign) > 0;
            evaluateCandidate(xy, length, preferredSide, lateral);
        }
        // Also explicitly sample the pinch escape bisector in multi-enemy fights.
        if (multiEnemy && Clock::now() < deadline) {
            const Vec2 xy{from[0] + length * std::cos(escapeYaw), from[1] + length * std::sin(escapeYaw)};
            evaluateCandidate(xy, length, true, 0.7);
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
