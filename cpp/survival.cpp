#include "survival.hpp"

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

// Check if gun/laser depressed to min_pitch (-0.7 rad) from 3.0m forward barrel can hit target cylinder.
bool canDepressToTarget(const Vec3& muzzlePos, const Vec3& targetFeet, double targetHeight, double selfRadius) {
    const double horiz = distanceXY(muzzlePos, targetFeet);
    if (horiz <= selfRadius + 1.0) {
        return true;
    }
    constexpr double minPitch = -0.70;
    const double barrel = selfRadius + 0.5;
    const double remHoriz = std::max(0.5, horiz - barrel * std::cos(minPitch));
    const double minRayZ = muzzlePos[2] + barrel * std::sin(minPitch) + remHoriz * std::tan(minPitch);
    return (targetFeet[2] + targetHeight >= minRayZ - 0.35);
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

// Rooftop Guerrilla Edge-Peek-and-Tuck Evaluator:
// Evaluates precomputed interior tuck spots + perimeter shooting ledges + local roof steps.
std::optional<CombatMove> planRooftopCombat(const Vec3& from, const RooftopNode& roof,
                                            const Threat& primary,
                                            const std::vector<const EnemyInfo*>& nearby,
                                            int closeCount,
                                            const CombatContext& ctx, const Zone& zone,
                                            const Motion& motion, const Arena& arena,
                                            Navigator& navigator, Clock::time_point deadline) {
    const auto origin = primary.attack_position ? primary.attack_position : primary.position;
    if (!origin) return std::nullopt;

    const double hpFrac = ctx.hp / std::max(1.0, ctx.maxHp);
    const bool multiEnemy = closeCount >= 2;
    const bool enemyFinishable = primary.hp <= 460.0 && hpFrac >= 0.24;
    const bool defensive = ctx.wantRegen || ctx.reloading || (hpFrac < 0.36 && !enemyFinishable);
    const bool wantMortarTuck = !defensive && ctx.hasReadyMortar && !enemyFinishable && closeCount >= 1;

    // Detect if any enemy is actively jet-climbing toward our rooftop!
    const EnemyInfo* climber = nullptr;
    double climberDist = infinity;
    for (const auto* e : nearby) {
        const double d = distanceXY(from, e->pos);
        if (d <= 32.0 && e->pos[2] >= roof.topZ - 14.0 && (e->vel[2] > 1.5 || e->pos[2] >= roof.topZ - 3.5)) {
            if (d < climberDist) {
                climberDist = d;
                climber = e;
            }
        }
    }

    // Choose focus target (climber first, then weakest isolatable enemy).
    Vec3 focusPos = climber ? climber->pos : *origin;
    double focusHeight = climber ? climber->height : primary.attack_height;
    if (!climber && multiEnemy && !defensive) {
        double bestFocus = -infinity;
        for (const auto* e : nearby) {
            const double d = distanceXY(from, e->pos);
            if (d > 125.0) continue;
            const double s = (1400.0 - std::min(1400.0, e->hp)) * 0.26 - d + (e->charging ? 130.0 : 0.0);
            if (s > bestFocus) {
                bestFocus = s;
                focusPos = e->pos;
                focusHeight = e->height;
            }
        }
    }
    const Vec3 focusTorso{focusPos[0], focusPos[1], focusPos[2] + focusHeight * 0.55};
    const Vec3 focusHead{focusPos[0], focusPos[1], focusPos[2] + focusHeight * 0.85};

    // If we are in defensive/regen/reload/mortar mode and already tucked (0 exposed), hold still!
    if ((defensive || wantMortarTuck) && !ctx.needReposition && !climber) {
        int exposedHere = 0;
        for (const auto* e : nearby) {
            if (wallCover(from, e->pos, motion.height, arena) < 3) {
                ++exposedHere;
            }
        }
        if (exposedHere == 0 && zone.clearance(from, zone.now + 1.5) > motion.radius + 1.2) {
            const char* lbl = ctx.wantRegen ? "roof-regen" : ctx.reloading ? "roof-reload" : "roof-mortar";
            return CombatMove{from, false, true, 0, lbl};
        }
    }

    double bestScore = -infinity;
    std::optional<CombatMove> best;
    std::vector<double> exposedYaws;
    exposedYaws.reserve(nearby.size());

    auto evalRoofPoint = [&](const Vec2& xy, bool isTuckCore, bool isEdgeLedge) {
        if (Clock::now() >= deadline) return;
        const double len = distanceXY(from, xy);
        const double arrival = zone.now + len / std::max(1.0, motion.walk) + 0.35;
        const double clearance = zone.clearance(xy, arrival);
        if (clearance < zone.buffer(zone.circle(arrival).second, motion.radius + 1.0)) {
            return;
        }
        const auto point = navigator.support(xy, motion, roof.topZ + 0.6);
        if (!point || (*point)[2] < roof.topZ - 1.2 || !navigator.direct(from, *point, zone, motion)) {
            return;
        }

        int exposedCount = 0;
        double incomingThreat = 0.0;
        exposedYaws.clear();
        for (const auto* e : nearby) {
            const double d = distanceXY(*point, e->pos);
            const int cov = wallCover(*point, e->pos, motion.height, arena);
            if (cov < 3) {
                ++exposedCount;
                exposedYaws.push_back(std::atan2(e->pos[1] - (*point)[1], e->pos[0] - (*point)[0]));
                incomingThreat += (e->charging ? 95.0 : 28.0) * (e->facing_us ? 1.35 : 0.8) * ((3 - cov) / 3.0);
                if (e->has_lightning && std::hypot(d, (*point)[2] - e->pos[2]) < 46.0) {
                    incomingThreat += 35.0;
                }
            }
        }

        const Vec3 muzzlePos{(*point)[0], (*point)[1], (*point)[2] + ctx.muzzleHeight};
        const double focusDist = distanceXY(*point, focusPos);
        const bool pitchOk = (ctx.hasLightning && std::hypot(focusDist, muzzlePos[2] - focusTorso[2]) <= 44.0) ||
            canDepressToTarget(muzzlePos, focusPos, focusHeight, motion.radius);
        const bool hasFocusLos = pitchOk &&
            (!arena.blocksLine(muzzlePos, focusTorso) || !arena.blocksLine(muzzlePos, focusHead));

        const auto [span, centerYaw] = angularCoverage(exposedYaws);
        (void)centerYaw;
        const double rearPenalty = (exposedCount >= 2 && span > radians(140.0)) ? 32.0 : 0.0;

        double score = 0.0;
        const char* label = "roof-peek";
        if (climber) {
            // Enemy climbing our roof: step to an edge with LOS on the climber at ~14-26m and shred them!
            const double climbDist = distanceXY(*point, climber->pos);
            score = (hasFocusLos ? 62.0 : -20.0) - std::abs(climbDist - 18.0) * 1.4 - 18.0 * std::max(0, exposedCount - 1);
            label = "roof-ambush";
        } else if (defensive || wantMortarTuck) {
            // Tuck into the safe interior core (exposedCount == 0)!
            if (exposedCount == 0) {
                score = 85.0 + (isTuckCore ? 14.0 : 0.0) - len * 0.45;
                label = ctx.wantRegen ? "roof-regen" : ctx.reloading ? "roof-reload" : "roof-mortar";
            } else {
                score = -35.0 * exposedCount - incomingThreat * 0.4 - len * 0.3;
                label = "roof-tuck";
            }
        } else {
            // Active Rooftop Guerrilla Edge-Peek: expose ONLY our #1 focus target (exposedCount == 1)!
            if (exposedCount == 1 && hasFocusLos) {
                score = 72.0 + (isEdgeLedge ? 8.0 : 0.0) - incomingThreat * 0.25;
                label = "roof-peek";
            } else if (hasFocusLos && exposedCount == 0) {
                // Rare jackpot: we can shoot their head over the lip while our torso/legs are 3/3 covered!
                score = 84.0;
                label = "roof-headglitch";
            } else if (exposedCount == 0) {
                score = (ctx.hasMortar ? 52.0 : 28.0) + (isTuckCore ? 6.0 : 0.0);
                label = ctx.hasMortar ? "roof-mortar" : "roof-tuck";
            } else if (hasFocusLos && exposedCount == 2 && !multiEnemy) {
                score = 38.0 - incomingThreat * 0.35;
                label = "roof-peek";
            } else {
                score = (hasFocusLos ? 16.0 : -18.0) - 28.0 * std::max(0, exposedCount - 1) - incomingThreat * 0.4;
                label = "roof-split";
            }
            // Prefer comfortable range (22-55m) from edge without stepping too far needlessly.
            score -= 0.22 * std::abs(focusDist - (ctx.hasLightning ? 26.0 : 44.0));
            score -= 0.25 * len;
        }

        const double dodgeBonus = (ctx.bulletDodge && len > 0.5)
            ? 10.0 * (((xy[0] - from[0]) * (*ctx.bulletDodge)[0] + (xy[1] - from[1]) * (*ctx.bulletDodge)[1]) / len)
            : 0.0;
        score += dodgeBonus - rearPenalty + 0.02 * std::min(40.0, clearance);

        if (score > bestScore) {
            bestScore = score;
            const bool wantBoost = !ctx.laserCharging && ctx.energy > 0.38 * ctx.maxEnergy &&
                (defensive || exposedCount >= 2) && len > 3.5;
            best = CombatMove{*point, wantBoost, exposedCount == 0, exposedCount, label};
        }
    };

    // 1. Evaluate precomputed interior tuck spots (360-deg cover from ground).
    for (const auto& tuck : roof.tuckSpots) {
        evalRoofPoint(horizontal(tuck), true, false);
    }
    // 2. Evaluate precomputed perimeter edge shooting spots.
    for (const auto& edge : roof.edgeSpots) {
        evalRoofPoint(horizontal(edge.pos), false, true);
    }
    // 3. Evaluate fine local steps (2.6m and 5.5m) around our current rooftop position.
    const double toFocus = std::atan2(focusPos[1] - from[1], focusPos[0] - from[0]);
    for (const double len : {2.6, 5.5, 9.0}) {
        for (const int off : {0, 45, -45, 90, -90, 135, -135, 180}) {
            if (Clock::now() >= deadline) break;
            const double yaw = toFocus + radians(off);
            evalRoofPoint({from[0] + len * std::cos(yaw), from[1] + len * std::sin(yaw)}, false, false);
        }
    }
    return best;
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
    const double startClearance = zone.clearance(from, zone.now);
    if (!origin || startClearance < -40.0 || Clock::now() >= deadline) {
        return std::nullopt;
    }
    const bool stormFight = startClearance < 0.0;

    // Filter enemies within tactical combat radius (<= 165 m) for fast multi-enemy geometry evaluation.
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

    // Dispatch to specialized Rooftop Guerrilla Edge-Peek-and-Tuck planner when standing on a rooftop node!
    if (ctx.currentRoof != nullptr) {
        if (const auto roofMove = planRooftopCombat(from, *ctx.currentRoof, primary, nearby, closeCount,
                                                    ctx, zone, motion, arena, navigator, deadline)) {
            return roofMove;
        }
    }

    const bool multiEnemy = closeCount >= 2;
    const double hpFrac = ctx.hp / std::max(1.0, ctx.maxHp);
    const bool enemyFinishable = primary.hp <= 420.0 && hpFrac >= 0.25;
    const bool defensive = ctx.wantRegen || ctx.reloading || (hpFrac < 0.38 && !enemyFinishable);
    const double myCubeMult = 1.0 + 0.04 * std::min(10.0, ctx.cubes);

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
        const double reqClearance = stormFight
            ? (startClearance - 1.5)
            : zone.buffer(zone.circle(arrival).second, motion.radius + 1.2);
        if (clearance < reqClearance) {
            return;
        }
        const auto point = navigator.support(xy, motion, from[2] + motion.step + 0.1);
        if (!point || !(stormFight ? navigator.walkable(from, *point, motion) : navigator.direct(from, *point, zone, motion))) {
            return;
        }

        int exposedCount = 0;
        int lightningCluster = 0;
        double minEnemyDist = infinity;
        double expectedDpsIn = 0.0;
        double weaponDangerPenalty = 0.0;
        double treeStealthBonus = 0.0;
        exposedYaws.clear();
        const Vec3 ptEye{(*point)[0], (*point)[1], (*point)[2] + 8.0};

        for (const auto* e : nearby) {
            const double d = distanceXY(*point, e->pos);
            minEnemyDist = std::min(minEnemyDist, d);
            const int cov = wallCover(*point, e->pos, motion.height, arena);
            const bool exposed = cov < 3;
            if (exposed) {
                ++exposedCount;
                exposedYaws.push_back(std::atan2(e->pos[1] - (*point)[1], e->pos[0] - (*point)[0]));
                const double eMult = 1.0 + 0.04 * std::min(10.0, e->cubes);
                const double covFactor = (cov == 2) ? 0.55 : (cov == 1 ? 0.82 : 1.0);
                double eDps = e->has_gun ? 85.0 * eMult * std::clamp(1.15 - d / 220.0, 0.35, 1.0) : 25.0;
                if (e->has_lightning && d < 47.0 && !(ctx.hasLightning && e->hp < 350.0)) {
                    weaponDangerPenalty += (47.0 - d) * 1.3;
                    eDps += 110.0 * eMult;
                }
                if (e->charging) {
                    weaponDangerPenalty += 30.0;
                    eDps += 150.0 * eMult;
                }
                // Check if OSU Oval tree canopy breaks enemy vision (> 46m) while we can still shoot!
                if (d > 46.0 && !e->facing_us) {
                    const Vec3 eEye{e->pos[0], e->pos[1], e->pos[2] + 8.0};
                    if (arena.blocksVisionTree(eEye, ptEye)) {
                        eDps *= 0.55;
                        treeStealthBonus += 6.5;
                    }
                }
                expectedDpsIn += eDps * covFactor * (e->facing_us ? 1.25 : 0.78);
            }
            if (ctx.hasLightning && d <= 42.0 && distanceXY(e->pos, focusPos) <= 14.0) {
                ++lightningCluster;
            }
        }
        if (minEnemyDist < 10.0) {
            return;
        }

        // Front-armor protection: penalize positions where exposed enemies span > 145 deg (1.5x rear-armor risk!).
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

        // Expected outgoing DPS contribution from this position.
        double expectedDpsOut = 0.0;
        if (!defensive && hasFocusLos) {
            expectedDpsOut += 95.0 * myCubeMult * std::clamp(1.15 - focusDist / 220.0, 0.4, 1.0);
            if (ctx.hasLightning && focusDist <= 43.0 && ctx.energy >= 12.0) {
                expectedDpsOut += 135.0 * myCubeMult * (1.0 + 0.35 * std::max(0, lightningCluster - 1));
            } else if (ctx.hasLaser && focusDist >= 48.0) {
                expectedDpsOut += 90.0 * myCubeMult;
            }
        } else if (!defensive && !hasFocusLos && ctx.hasMortar && exposedCount == 0 && focusDist >= 18.0) {
            expectedDpsOut += 105.0 * myCubeMult;
        }
        const double tradeUtility = 0.055 * (expectedDpsOut - (defensive ? 1.8 : multiEnemy ? 1.35 : 0.95) * expectedDpsIn);

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

        // Open-ground cover pull: drag fights from the open Oval lawn toward the nearest building ring!
        double pullBonus = 0.0;
        if (ctx.coverPull && !ctx.onRoof && (exposedCount >= 1 || defensive)) {
            const double pullProgress = distanceXY(from, *ctx.coverPull) - distanceXY(xy, *ctx.coverPull);
            const double pullWeight = (multiEnemy || defensive) ? 1.25 : 0.60;
            pullBonus = pullWeight * pullProgress;
            if (pullProgress > 4.0 && multiEnemy && exposedCount >= 2) {
                label = "pull-cover";
            }
        }

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

        const double score = exposureScore + tradeUtility + treeStealthBonus + pinchBonus +
            repositionBonus + momentumBonus + dodgeBonus + roofBonus + pullBonus +
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

    const bool nearCornerWall = !ctx.onRoof && arena.distanceToNearestWall(from[0], from[1], from[2], 10.0) <= 8.5;
    const std::array<double, 4> roofLengths{2.8, 5.5, 9.0, 14.0};
    const std::array<double, 4> cornerLengths{3.2, 8.5, 14.5, 20.0};
    const std::array<double, 4> groundLengths{9.0, 15.0, 21.0, 5.5};
    const auto& stepLengths = ctx.onRoof ? roofLengths : (nearCornerWall ? cornerLengths : groundLengths);
    for (const double length : stepLengths) {
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
        // Sample directly toward nearest building cover when caught in open ground.
        if (ctx.coverPull && Clock::now() < deadline) {
            const double pullYaw = std::atan2((*ctx.coverPull)[1] - from[1], (*ctx.coverPull)[0] - from[0]);
            for (const int po : {0, 28, -28}) {
                if (Clock::now() >= deadline) break;
                const double yaw = pullYaw + radians(po);
                const Vec2 xy{from[0] + length * std::cos(yaw), from[1] + length * std::sin(yaw)};
                const double lat = std::abs(std::sin(yaw - toFocus));
                evaluateCandidate(xy, length, true, lat);
            }
        }
    }
    // Also evaluate nearest OSU Oval tree visual cover spot when fighting > 48m in open lawn.
    if (ctx.treeCover && Clock::now() < deadline) {
        const double len = distanceXY(from, *ctx.treeCover);
        const double yaw = std::atan2((*ctx.treeCover)[1] - from[1], (*ctx.treeCover)[0] - from[0]);
        evaluateCandidate(*ctx.treeCover, len, true, std::abs(std::sin(yaw - toFocus)));
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
