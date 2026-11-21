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

int coverFrom(const Vec3& feet, const Vec3& source, double height, const Arena& arena) {
    int protectedPoints = 0;
    for (const double fraction : {0.22, 0.54, 0.9}) {
        const Vec3 sample{feet[0], feet[1], feet[2] + height * fraction};
        protectedPoints += arena.blocksLine(source, sample);
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
            penalty += closure * (40.0 / std::max(8.0, initial));
        }
    }
    return penalty;
}

int wallCover(const Vec3& feet, const Vec3& threat, double ownHeight, const Arena& arena) {
    const Vec3 source{threat[0], threat[1], threat[2] + 7.0};
    return coverFrom(feet, source, ownHeight, arena);
}

int wallCover(const Vec3& feet, const EnemyInfo& threat, double ownHeight, const Arena& arena) {
    return coverFrom(feet, {threat.pos[0], threat.pos[1], threat.pos[2] + threat.muzzleHeight}, ownHeight, arena);
}

std::optional<double> frontArmorYaw(const Vec3& from, const std::vector<EnemyInfo>& enemies,
                                    double ownHeight, const Arena& arena) {
    // Point our 180-deg front armor arc at the centroid of exposed close threats to avoid 1.5x rear damage.
    double sumX = 0.0, sumY = 0.0;
    for (const auto& e : enemies) {
        const double dist = distanceXY(from, e.pos);
        if (dist > 280.0 || wallCover(from, e, ownHeight, arena) == 3) {
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

namespace {

constexpr double exchangeSeconds = 6.0;

const Json& child(const Json* obj, const char* key) {
    static const Json empty = Json::object();
    if (!obj || !obj->is_object()) return empty;
    const auto it = obj->find(key);
    return it != obj->end() && it->is_object() ? *it : empty;
}

enum class Weapon { Gun, Mortar, Laser, Lightning };

struct LoadoutSlot {
    Weapon kind;
    double damage, range, cycle, ready, speed, gravity, energyCost, reload;
    int ammo, mag;
};

LoadoutSlot readSlot(Weapon kind, const Json& stats, const Json& state) {
    const bool gun = kind == Weapon::Gun, mortar = kind == Weapon::Mortar, laser = kind == Weapon::Laser;
    const double speed = stats.value("speed", mortar ? 72.0 : 252.0);
    const double gravity = std::max(1.0, stats.value("gravity", 22.0));
    const double charge = laser ? stats.value("charge", 0.75) : 0.0;
    const int mag = std::max(1, stats.value("mag", 12));
    return {kind, stats.value("damage", gun ? 45.0 : mortar ? 200.0 : laser ? 315.0 : 60.0),
        stats.value("range", gun ? 230.0 : mortar ? speed * speed / gravity : laser ? 280.0 : 45.0),
        std::max(0.05, stats.value("cooldown", gun ? 0.24 : mortar ? 2.6 : laser ? 1.65 : 0.25) + charge),
        std::max(state.value("cooldown", 0.0), state.value("reload", 0.0)) +
            std::max(0.0, charge - state.value("charge", 0.0)),
        speed, gravity, stats.value("energy", 3.0), stats.value("reload", 2.4),
        state.value("ammo", mag), mag};
}

constexpr std::array weaponNames{"gun", "mortar", "laser", "lightning"};
using Loadout = std::vector<LoadoutSlot>;

double expectedHits(const LoadoutSlot& slot, double start, double duration, double energy) {
    if (duration <= 0) return 0;
    // Expected trigger opportunities in an interval. Magazine changes consume real match time.
    const auto until = [&](double end) {
        if (end <= slot.ready) return 0.0;
        double time = end - slot.ready;
        if (slot.kind != Weapon::Gun) return 1.0 + std::floor((time - 1e-6) / slot.cycle);
        const double first = std::min<double>(slot.ammo, 1 + std::floor((time - 1e-6) / slot.cycle));
        const double firstReload = std::max(0, slot.ammo - 1) * slot.cycle + slot.reload;
        if (time <= firstReload) return first;
        time -= firstReload;
        const double cycle = (slot.mag - 1) * slot.cycle + std::max(0.05, slot.reload);
        return first + slot.mag * std::floor(time / cycle) +
               std::min<double>(slot.mag, 1.0 + std::floor(std::fmod(time, cycle) / slot.cycle));
    };
    double hits = std::max(0.0, until(start + duration) - until(start));
    if (slot.kind == Weapon::Lightning) hits = std::min(hits, std::floor(energy / std::max(0.1, slot.energyCost)));
    return hits;
}

struct Exchange {
    double hp = 0.0;
    int exposed = 0;
};

class ExchangeModel {
public:
    ExchangeModel(const Threat& focus, const std::vector<EnemyInfo>& enemies, const CombatContext& ctx,
                  const Motion& motion, const Arena& arena)
        : focus_(focus), enemies_(enemies), ctx_(ctx), motion_(motion), arena_(arena) {
        // update() gives a neutral visible actor a position and id, while a bearing-only
        // alert or a past attacker may have an estimated position without a current observation.
        canAttack_ = focus.attack_position.has_value() ||
            (focus.actor_id && std::any_of(enemies.begin(), enemies.end(),
                [&](const EnemyInfo& enemy) { return enemy.id == *focus.actor_id; }));
        const auto& catalog = child(ctx.config, "weapons");
        const Json empty = Json::object();
        for (std::size_t i = 0; i < defaults_.size(); ++i)
            defaults_[i] = readSlot(static_cast<Weapon>(i), child(&catalog, weaponNames[i]), empty);
        if (ctx.weapons && ctx.weapons->is_array()) {
            for (const auto& w : *ctx.weapons) {
                if (!w.is_object()) continue;
                for (std::size_t i = 0; i < defaults_.size(); ++i) {
                    if (w.value("kind", "") != std::string_view(weaponNames[i]) ||
                        (i == 2 && ctx.laserShorted)) continue;
                    auto slot = readSlot(static_cast<Weapon>(i), child(&w, "stats"), w);
                    if (slot.kind == Weapon::Gun && w.value("reload", 0.0) > 0)
                        slot.ammo = slot.mag;
                    own_.push_back(slot);
                }
            }
        } else {
            if (!ctx.reloading) own_.push_back(defaults_[0]);
            if (ctx.hasMortar) own_.push_back(defaults_[1]);
            if (ctx.hasLaser && !ctx.laserShorted) own_.push_back(defaults_[2]);
            if (ctx.hasLightning) own_.push_back(defaults_[3]);
        }
        const auto& cubeRules = child(ctx.config, "cubes");
        perCube_ = cubeRules.value("damage_per_cube", 0.04);
        cap_ = cubeRules.value("stat_cap", 10.0);
        memory_ = child(ctx.config, "vision").value("memory_s", 5.0);
    }

    Exchange at(const Vec3& from, const Vec3& point) const {
        Exchange result;
        const auto target = focus_.attack_position ? focus_.attack_position : focus_.position;
        if (!target) return result;
        const double distance = distanceXY(from, point);
        const double walk = std::max(1.0, motion_.walk * (ctx_.laserCharging ? 0.5 : 1.0));
        const double arrival = std::min(exchangeSeconds, distance / walk +
            std::max(0.0, point[2] - from[2] - motion_.step) / std::max(1.0, motion_.rise));
        // A planned peek must leave time to return to its chosen interior cover.
        const double returnTime = ctx_.returnCover ? std::min(exchangeSeconds - arrival,
            distanceXY(point, *ctx_.returnCover) / walk) : 0.0;
        const double dwell = std::max(0.0, exchangeSeconds - arrival - returnTime);
        const Vec3 muzzle{point[0], point[1], point[2] + ctx_.muzzleHeight};
        const Vec3 torso{(*target)[0], (*target)[1], (*target)[2] + focus_.attack_height * 0.5};
        const double range = std::hypot(distanceXY(muzzle, torso), muzzle[2] - torso[2]);
        const bool direct = !arena_.blocksLine(muzzle, torso);
        double outgoing = 0.0;
        // Legs and torso are independent. Translation permits firing until geometry closes the lane.
        const Vec3 startMount{from[0], from[1], from[2] + ctx_.muzzleHeight};
        const double firingStart = direct && !arena_.blocksLine(startMount, torso) ? arrival * 0.25 : arrival;
        for (const auto& slot : own_) {
            if (ctx_.wantRegen || !canAttack_ || range > slot.range) continue;
            double contact = direct ? 1.0 : 0.0;
            if (slot.kind == Weapon::Mortar) {
                if (range < 14.0) continue;
                const Vec3 mount{point[0], point[1], point[2] + motion_.height + 0.5};
                const Vec3 low{(*target)[0], (*target)[1], (*target)[2] + 1.5};
                contact = mortarLane(mount, low, slot.speed, slot.gravity, arena_) ? 0.65 : 0.0;
            }
            const double weakness = slot.kind == Weapon::Laser && focus_.has_mortar ? 1.6 : 1.0;
            outgoing += expectedHits(slot, firingStart, dwell + arrival - firingStart, ctx_.energy) *
                        slot.damage * weakness * contact;
        }
        outgoing *= 1.0 + perCube_ * std::min(cap_, ctx_.cubes);
        const double focusActive = outgoing >= focus_.hp && outgoing > 0.0
            ? std::min(exchangeSeconds, firingStart + (dwell + arrival - firingStart) * focus_.hp / outgoing)
            : exchangeSeconds;
        double incoming = 0.0;
        const double aimYaw = std::atan2((*target)[1] - point[1], (*target)[0] - point[0]);
        for (const auto& e : enemies_) {
            const double d = distanceXY(point, e.pos);
            if (d > 300.0) continue;
            const double open = 1.0 - wallCover(point, e, motion_.height, arena_) / 3.0;
            result.exposed += open > 0.0;
            const double initialOpen = distance > 0.01 ? 1.0 - wallCover(from, e, motion_.height, arena_) / 3.0 : open;
            const double yaw = std::atan2(e.pos[1] - point[1], e.pos[0] - point[0]);
            const Vec2 travel = direction(from, point);
            const double lateral = std::abs(travel[0] * std::sin(yaw) - travel[1] * std::cos(yaw));
            const double armor = std::clamp(ctx_.stats ? ctx_.stats->value("armor", 0.0) : 0.0, 0.0, 1.0);
            const double rear = std::cos(yaw - aimYaw) < 0 ? 1.5 : 1.0;
            const double scale = (1.0 + perCube_ * std::min(cap_, e.cubes)) * (1.0 - armor) * rear;
            const Vec3 eye{e.pos[0], e.pos[1], e.pos[2] + e.height * (8.0 / 9.0)};
            const Vec3 selfTop{point[0], point[1], point[2] + motion_.height};
            const Vec3 selfCenter{point[0], point[1], point[2] + motion_.height * 0.5};
            // Assume an opponent retains the full tracking window after we break vision.
            const bool hidden = dwell > memory_ && open > 0 && d > 45 &&
                                arena_.blocksVision(eye, selfTop) && arena_.blocksVision(eye, selfCenter);
            const double visibility = hidden ? 1.0 - 0.5 * std::clamp((dwell - memory_) / exchangeSeconds, 0.0, 1.0) : 1.0;
            const double exposure = (arrival * (open + initialOpen) * 0.5 + dwell * open * visibility +
                                     returnTime * open * 0.5) / exchangeSeconds;
            const double bearingToUs = yaw + std::numbers::pi;
            const double yawError = e.aim ? std::abs(std::remainder(bearingToUs - (*e.aim)[0], 2 * std::numbers::pi)) : 0.0;
            const double turnTime = yawError / std::max(0.1, child(ctx_.config, "mech").value("turn_rate", 4.0));
            const std::array equipped{e.hasGun, e.has_mortar, e.has_laser, e.has_lightning};
            for (std::size_t i = 0; i < equipped.size(); ++i) {
                const auto& slot = defaults_[i];
                if (!equipped[i] || d > slot.range) continue;
                double fraction = exposure;
                if (slot.kind == Weapon::Mortar) {
                    if (d < 14.0) continue;
                    const Vec3 tube{e.pos[0],e.pos[1],e.pos[2] + e.height + 0.5};
                    const Vec3 low{point[0],point[1],point[2] + 1.5};
                    if (mortarLane(tube, low, slot.speed, slot.gravity, arena_)) fraction = std::max(fraction, 0.45);
                }
                const double hitChance = slot.kind == Weapon::Gun || slot.kind == Weapon::Mortar
                    ? 1.0 / (1.0 + lateral * std::min(1.0, distance / walk) * d / std::max(1.0, slot.speed)) : 1.0;
                const double weakness = slot.kind == Weapon::Laser && ctx_.hasMortar ? 1.6 : 1.0;
                auto estimated = slot;
                estimated.ready = (slot.kind == Weapon::Laser && e.charging ? 0 : slot.ready) + turnTime;
                const double active = focus_.actor_id && e.id == *focus_.actor_id ? focusActive : exchangeSeconds;
                incoming += expectedHits(estimated, 0.0, active, 100.0) *
                            slot.damage * fraction * scale * hitChance * weakness;
            }
        }
        result.hp = std::min(focus_.hp, outgoing) - 1.5 * incoming;
        if (ctx_.wantRegen && result.exposed == 0) {
            const double healingTime = std::max(0.0, dwell - std::max(0.0, ctx_.regenWait - arrival));
            result.hp += std::min(ctx_.maxHp - ctx_.hp, healingTime *
                                  (ctx_.stats ? ctx_.stats->value("regen_per_s", 45.0) : 45.0));
        }
        return result;
    }

private:
    const Threat& focus_;
    const std::vector<EnemyInfo>& enemies_;
    const CombatContext& ctx_;
    const Motion& motion_;
    const Arena& arena_;
    std::array<LoadoutSlot, 4> defaults_{};
    Loadout own_;
    bool canAttack_ = false;
    double perCube_, cap_, memory_;
};
} // namespace

double exchangeValue(const Vec3& from, const Vec3& candidate, const Threat& primary,
                     const std::vector<EnemyInfo>& enemies, const CombatContext& ctx,
                     const Motion& motion, const Arena& arena) {
    return ExchangeModel(primary, enemies, ctx, motion, arena).at(from, candidate).hp;
}

std::optional<CombatMove> planCombatPosition(const Vec3& from, const Threat& primary,
                                             const std::vector<EnemyInfo>& enemies,
                                             const CombatContext& ctx, const Zone& zone,
                                             const Motion& motion, const Arena& arena,
                                             Navigator& navigator, Clock::time_point deadline) {
    const auto focus = primary.attack_position ? primary.attack_position : primary.position;
    if (!focus || zone.clearance(from, zone.now) < 0 || Clock::now() >= deadline) return std::nullopt;
    ExchangeModel model(primary, enemies, ctx, motion, arena);
    const auto current = model.at(from, from);
    CombatMove best{from, false, current.exposed == 0, current.exposed, "engage"};
    double bestScore = current.hp, bestTie = 0.0;
    const double angle = std::atan2((*focus)[1] - from[1], (*focus)[0] - from[0]);
    const double side = ctx.strafeSign < 0 ? -1.0 : 1.0;
    const std::array lengths = ctx.onRoof ? std::array{3.0, 6.0, 10.0} : std::array{5.0, 10.0, 16.0};
    for (const double length : lengths) {
        for (const int offset : {90, -90, 145, -145, 45, -45, 180, 0}) {
            if (Clock::now() >= deadline) return best;
            const double yaw = angle + side * radians(offset);
            const Vec2 xy{from[0] + length * std::cos(yaw), from[1] + length * std::sin(yaw)};
            auto point = navigator.support(xy, motion, from[2] + motion.step + 0.1);
            if (!point || (ctx.onRoof && (*point)[2] < from[2] - motion.step) ||
                !navigator.direct(from, *point, zone, motion)) continue;
            const auto estimate = model.at(from, *point);
            const double lateral = std::abs(std::sin(radians(offset)));
            // Prefer continuous lateral motion among equally valuable exchanges.
            const double tie = lateral + 0.01 * (ctx.velocity[0] * std::cos(yaw) + ctx.velocity[1] * std::sin(yaw));
            if (estimate.hp > bestScore + 0.5 || (std::abs(estimate.hp - bestScore) <= 0.5 &&
                current.exposed > 0 && !ctx.wantRegen && tie > bestTie)) {
                bestScore = estimate.hp;
                bestTie = tie;
                const bool boost = !ctx.laserCharging && ctx.energy > motion.boostDrain + (ctx.hasLightning ? 12.0 : 0.0) &&
                                   estimate.exposed < current.exposed;
                best = {*point, boost, estimate.exposed == 0, estimate.exposed,
                        estimate.exposed == 0 ? "combat-cover" : "engage"};
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
