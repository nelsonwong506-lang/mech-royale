#include "arena.hpp"
#include "navigation.hpp"
#include "survival.hpp"
#include "tactics.hpp"

#include <algorithm>
#include <cerrno>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unistd.h>
#include <utility>
#include <vector>

namespace mr {
namespace {

using namespace std::chrono_literals;
constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr std::array angles{0, 30, -30, 60, -60, 90, -90, 135, -135, 180};

struct Command {
    Vec2 move{};
    bool boost = false;
    bool jump = false;
};

struct Progress {
    Vec3 target;
    Vec3 origin;
    double time;
};

struct RetreatIntent {
    Vec3 target;
    std::optional<int> opponent;
    double expires;
};

struct Peek {
    Vec3 cover;
    Vec3 edge;
    int opponent;
    double started;
    bool returning = false;
    bool fired = false;
};

struct Decision {
    Command command;
    const char* label;
    bool fireAllowed = true;
    bool returningToCover = false;
};

struct ActiveHop {
    DefensiveHop path;
    double started;
    const char* label;
};

class Bot {
public:
    explicit Bot(const Json& hello)
        : config_(hello.at("config")),
          arena_(hello.at("map").at("dir").get<std::string>(),
                 hello.at("map").value("files", std::vector<std::string>{})),
          navigator_(arena_), threats_(config_),
          pickupRadius_(config_.value("cubes", Json::object()).value("pickup_radius", 3.5)) {}

    Json act(const Json& obs) {
        const auto started = Clock::now();
        const auto& me = obs.at("self");
        const auto start = me.at("pos").get<Vec3>();
        const Zone zone(obs.at("zone"), obs.at("t").get<double>());
        auto stats = config_.value("mech", Json::object());
        stats.update(me.at("stats"));
        const Motion motion(stats);
        const auto threat = threats_.update(obs, arena_);
        const auto duel = threats_.duel_target(obs, arena_);
        const auto enemies = localEnemies(obs, start, threat);
        const auto decision = choose(obs, start, zone, motion, threat, enemies, started);
        const auto& command = decision.command;
        Json action = {{"move", command.move}, {"boost", command.boost}, {"jump", command.jump}};
        const bool underThreat = threat && (threat->hostile || threat->damaged_recently || threat->laser_charging);
        const bool canJoin = duel && !underThreat && !peek_ && !hop_ && me.value("in_zone", false) &&
            !rotationWindow(start, zone, motion, routeEstimate_).urgent;
        const bool joining = decision.fireAllowed && canJoin && opportunity_fire(obs, duel, arena_);
        const bool defending = decision.fireAllowed && return_fire(obs, threat, command.move, arena_, decision.returningToCover);
        const auto shot = decision.fireAllowed && !joining && !defending ? threats_.visible_shot(obs, arena_) : std::nullopt;
        const auto aim = shot ? std::optional{shot->aim} : canJoin ? retreat_aim(obs, duel) :
            threat && threat->facing_us ? retreat_aim(obs, threat) : std::nullopt;
        if (aim) {
            action["aim"] = *aim;
        } else {
            action["aim"] = scanAim(me, command.move, zone.now);
        }
        std::array fire{false, false};
        const auto& weapons = me.at("weapons");
        if (joining || defending || (shot && shot->fire)) {
            for (std::size_t i = 0; i < std::min(fire.size(), weapons.size()); ++i) {
                if (weapons[i].is_object() && weapons[i].value("kind", "") == std::string_view("gun")) {
                    fire[i] = true;
                    if (decision.returningToCover && peek_) peek_->fired = true;
                    break;
                }
            }
        }
        action["fire"] = fire;
        const bool canReloadEarly = !canJoin && (!threat || shelteredAt_.has_value());
        action["reload"] = std::any_of(weapons.begin(), weapons.end(), [canReloadEarly](const Json& weapon) {
            return weapon.is_object() && weapon.value("kind", "") == std::string_view("gun") &&
                weapon.value("reload", 0.0) <= 0 && weapon.value("ammo", 0.0) <= (canReloadEarly ? 3 : 0);
        });
        if (!obs.at("offer").is_null()) {
            action["pick"] = 0;
        }
        if (config_.value("debug", Json::object()).value("max_text_chars", 0) >= 16) {
            action["debug"] = {{"label", joining ? "duel-fire" : canJoin ? "duel-aim" : decision.label}};
        }
        return action;
    }

private:
    using Goals = std::vector<Vec3>;

    static std::vector<Vec3> localEnemies(const Json& obs, const Vec3& start, const std::optional<Threat>& threat) {
        std::vector<Vec3> enemies;
        for (const auto& actor : obs.at("mechs")) {
            if (actor.value("kind", "") != std::string_view("mech")) continue;
            const auto point = actor.at("pos").get<Vec3>();
            if (distanceXY(start, point) <= 95) enemies.push_back(point);
        }
        if (threat && threat->position && std::none_of(enemies.begin(), enemies.end(), [&](const Vec3& enemy) {
                return distanceXY(*threat->position, enemy) < 1;
            })) {
            enemies.push_back(*threat->position);
        }
        return enemies;
    }

    Vec2 scanAim(const Json& me, const Vec2& move, double now) {
        if (std::hypot(move[0], move[1]) > 0.12) {
            const double heading = std::atan2(move[1], move[0]);
            scanHeading_ = heading;
        }
        if (!scanHeading_) scanHeading_ = me.at("aim").at(0).get<double>();
        const double cycle = std::fmod(now, 8.0);
        const double offset = cycle >= 6.6 ? std::numbers::pi : 1.48 * std::sin(cycle * (2 * std::numbers::pi / 3.3));
        return {std::remainder(*scanHeading_ + offset, 2 * std::numbers::pi), 0.0};
    }

    Goals rotationGoals(const Vec3& start, const Zone& zone, const Motion& motion) const {
        if (zone.inNext(start, motion.radius + 5)) {
            return {start};
        }
        const auto [x, y] = zone.nextCenter;
        const auto [minX, minY, maxX, maxY] = arena_.bounds;
        const double margin = motion.radius + 0.3;
        const double safe = std::max(0.0, zone.nextRadius - zone.buffer(zone.nextRadius, motion.radius + 6));
        const double angle = std::atan2(start[1] - y, start[0] - x);
        constexpr std::array offsets{0, 24, -24, 50, -50, 85, -85, 135, -135, 180};
        Goals points;
        for (const double radius : {safe, safe * 0.66, safe * 0.32, 0.0}) {
            for (const int offset : offsets) {
                if (radius == 0.0 && offset != 0) {
                    break;
                }
                const double yaw = angle + radians(offset);
                const Vec2 point{
                    std::clamp(x + radius * std::cos(yaw), minX + margin, maxX - margin),
                    std::clamp(y + radius * std::sin(yaw), minY + margin, maxY - margin),
                };
                if (!zone.inNext(point) || std::any_of(points.begin(), points.end(), [&](const Vec3& p) {
                        return distanceXY(point, p) <= 3;
                    })) {
                    continue;
                }
                const double floor = arena_.ground(point[0], point[1]);
                if (const auto goal = navigator_.support(point, motion, floor + motion.step + 0.05)) {
                    points.push_back(*goal);
                }
            }
        }
        std::stable_sort(points.begin(), points.end(), [&](const Vec3& a, const Vec3& b) {
            return distanceXY(start, a) < distanceXY(start, b);
        });
        if (points.size() > 16) {
            points.resize(16);
        }
        return points;
    }

    Decision choose(const Json& obs, const Vec3& start, const Zone& zone, const Motion& motion,
                    const std::optional<Threat>& threat, const std::vector<Vec3>& enemies,
                    Clock::time_point started) {
        const auto& me = obs.at("self");
        const auto phase = std::pair{zone.start, zone.end};
        if (rotationPhase_ != phase) {
            rotationPhase_ = phase;
            goalDeadline_ = -infinity;
            routeEstimate_.reset();
            goals_.clear();
            navigator_.reset();
        }
        const auto window = rotationWindow(start, zone, motion, routeEstimate_);
        if (const auto evasive = evasiveMove(me, start, zone, motion, threat, enemies, window, started + 14ms)) {
            if (flight_ && !flight_->airborne) flight_.reset();
            return *evasive;
        }
        if (flight_ && !flight_->airborne && threat && !window.urgent) flight_.reset();
        if (const auto command = fly(me, start, zone, motion)) {
            return {*command, "rotate-air"};
        }

        if (threat && threat->position && !window.urgent && me.value("in_zone", false)) {
            if (const auto peeking = peekCommand(obs, start, zone, motion, *threat, enemies, window)) {
                return *peeking;
            }
            auto retreat = retreatTarget(start, *threat, enemies, zone, motion, started + 15ms);
            if (retreat) {
                if (retreat->hardCover && distanceXY(start, retreat->target) < 0.9) {
                    if (!shelteredAt_) shelteredAt_ = zone.now;
                    return {groundCommand(start, std::nullopt, me, zone, motion, false, enemies), "hold-wall", false};
                }
                shelteredAt_.reset();
                const bool boost = me.value("grounded", true) && threat->hostile && distanceXY(start, *threat->position) < 55 &&
                    me.value("energy", 0.0) > 0.38 * me.value("max_energy", 100.0);
                const auto command = groundCommand(start, retreat->target, me, zone, motion, boost, enemies);
                if (std::hypot(command.move[0], command.move[1]) > 0.12) {
                    return {command, "withdraw"};
                }
                retreat_.reset();
            }
        } else {
            peek_.reset();
            retreat_.reset();
        }
        shelteredAt_.reset();
        return rotate(obs, start, zone, motion, threat, enemies, window, started);
    }

    bool wallBehind(const Vec3& start, const Vec2& away, const Motion& motion) const {
        const double reach = motion.radius + 2.2;
        // Test low and waist-high rays so a roof above the head isn't mistaken for a back wall.
        for (const double height : {0.55, motion.height * 0.45}) {
            const Vec3 from{start[0], start[1], start[2] + height};
            const Vec3 end{from[0] + reach * away[0], from[1] + reach * away[1], from[2]};
            if (arena_.blocksPrismLine(from, end)) return true;
        }
        return false;
    }

    bool keepsRotation(const Vec3& from, const Vec3& to, const Zone& zone, const RotationWindow& window) const {
        return !window.urgent || zone.inNext(from) ||
            distanceXY(to, zone.nextCenter) <= distanceXY(from, zone.nextCenter) + 0.05;
    }

    std::optional<Decision> evasiveMove(const Json& me, const Vec3& start, const Zone& zone, const Motion& motion,
                                       const std::optional<Threat>& threat, const std::vector<Vec3>& enemies,
                                       const RotationWindow& window, Clock::time_point deadline) {
        const bool grounded = me.value("grounded", false);
        if (hop_) {
            if (!grounded && zone.now - hop_->started < hop_->path.duration + 0.35) {
                // Keep the checked air input, including a vertical lift before a low wall.
                const auto move = zone.now - hop_->started + 1e-6 < hop_->path.steerAfter ? Vec2{} : hop_->path.launch;
                if (navigator_.guard(move, false, me, zone, motion)) {
                    return Decision{{move}, hop_->label};
                }
                const auto command = groundCommand(start, hop_->path.landing, me, zone, motion, false, enemies);
                return Decision{command, hop_->label};
            }
            hop_.reset();
        }
        if (!grounded || !me.value("in_zone", false) || !threat || !threat->position) return {};
        const Vec3& enemy = *threat->position;
        const auto away = direction(enemy, start);
        if (std::hypot(away[0], away[1]) < 0.1) return {};
        const bool laser = threat->laser_charging;
        const bool exposed = wallCover(start, enemy, motion.height, arena_) < 3;
        const bool blocked = threat->facing_us && wallBehind(start, away, motion);
        if (zone.now >= nextHopAt_ && exposed && (laser || threat->damaged_recently || blocked)) {
            const auto path = planDefensiveHop(me, *threat, enemies, zone, motion, arena_, navigator_, laser, blocked, deadline);
            if (path && keepsRotation(start, path->landing, zone, window)) {
                const auto label = laser ? "laser-hop" : blocked ? "wall-hop" : "cover-hop";
                hop_ = ActiveHop{*path, zone.now, label};
                nextHopAt_ = zone.now + 2.4;
                retreat_.reset();
                shelteredAt_.reset();
                peek_.reset();
                return Decision{{path->steerAfter > 0 ? Vec2{} : path->launch, false, true}, label};
            }
        }
        if (!laser) return {};

        // Low ceilings or the ring can rule out a jump. Break the beam's line sideways on foot.
        const auto velocity = me.at("vel").get<Vec3>();
        std::optional<Decision> best;
        double bestScore = -infinity;
        for (const double side : {-1.0, 1.0}) {
            const Vec2 point{start[0] + 1.4 * away[0] - side * 6.0 * away[1],
                             start[1] + 1.4 * away[1] + side * 6.0 * away[0]};
            const auto target = navigator_.support(point, motion, start[2] + motion.step);
            if (!target || !avoidsApproach(start, point, enemies) || !keepsRotation(start, *target, zone, window) ||
                !navigator_.direct(start, *target, zone, motion)) continue;
            const auto move = direction(start, point);
            const bool sprint = me.value("energy", 0.0) > 0.4 * me.value("max_energy", 100.0) &&
                navigator_.guard(move, true, me, zone, motion);
            if (!sprint && !navigator_.guard(move, false, me, zone, motion)) continue;
            const double score = 4 * wallCover(*target, enemy, motion.height, arena_) +
                0.16 * (velocity[0] * move[0] + velocity[1] * move[1]);
            if (score > bestScore) {
                bestScore = score;
                best = Decision{{move, sprint}, "laser-strafe"};
            }
        }
        if (best) {
            retreat_.reset();
            shelteredAt_.reset();
            peek_.reset();
        }
        return best;
    }

    std::optional<RetreatPlan> retreatTarget(const Vec3& start, const Threat& threat, const std::vector<Vec3>& enemies,
                                            const Zone& zone, const Motion& motion, Clock::time_point deadline) {
        if (retreat_ && retreat_->opponent == threat.actor_id && zone.now < retreat_->expires &&
            distanceXY(start, retreat_->target) > 0.9 && avoidsApproach(start, horizontal(retreat_->target), enemies) &&
            navigator_.direct(start, retreat_->target, zone, motion)) {
            return RetreatPlan{retreat_->target, false};
        }
        auto plan = planRetreat(start, threat, enemies, zone, motion, arena_, navigator_, deadline);
        retreat_.reset();
        if (plan && distanceXY(start, plan->target) > 0.9) {
            const double hold = std::clamp(distanceXY(start, plan->target) / motion.walk + 0.45, 0.65, 2.3);
            retreat_ = RetreatIntent{plan->target, threat.actor_id, zone.now + hold};
        }
        return plan;
    }

    std::optional<Decision> peekCommand(const Json& obs, const Vec3& start, const Zone& zone,
                                        const Motion& motion, const Threat& threat,
                                        const std::vector<Vec3>& enemies, const RotationWindow& window) {
        const auto& me = obs.at("self");
        if (peek_ && (!threat.actor_id || peek_->opponent != *threat.actor_id || enemies.size() > 1 ||
                      zone.now - peek_->started > 3.0 || !navigator_.direct(start, peek_->cover, zone, motion))) {
            peek_.reset();
        }
        if (!peek_) {
            const bool gunReady = std::any_of(me.at("weapons").begin(), me.at("weapons").end(), [](const Json& weapon) {
                return weapon.is_object() && weapon.value("kind", "") == std::string_view("gun") &&
                    weapon.value("ammo", 0.0) > 0 && weapon.value("reload", 0.0) <= 0 && weapon.value("cooldown", 0.0) <= 0.1;
            });
            if (!shelteredAt_ || zone.now - *shelteredAt_ < 0.55 || zone.now < nextPeekAt_ || !gunReady ||
                !me.value("grounded", false) || me.value("hp", 0.0) < 0.65 * me.value("max_hp", 1400.0) ||
                window.secondsUntilRisk < window.estimatedTravel + 10 || enemies.size() != 1 || !threat.actor_id ||
                !threat.position || !threat.enemy_aim || distanceXY(start, *threat.position) < 15 ||
                wallCover(start, *threat.position, motion.height, arena_) != 3) {
                return {};
            }
            const Vec2 toward = direction(start, *threat.position);
            for (const double side : {3.5, -3.5, 5.5, -5.5}) {
                const Vec2 xy{start[0] - toward[1] * side, start[1] + toward[0] * side};
                if (!avoidsApproach(start, xy, enemies)) continue;
                const auto edge = navigator_.support(xy, motion, start[2] + motion.step);
                if (edge && zone.clearance(*edge, zone.now + 2) > motion.radius + 1 &&
                    navigator_.direct(start, *edge, zone, motion) && navigator_.direct(*edge, start, zone, motion) &&
                    enemy_aims_at(threat, *edge, motion.radius, motion.height, arena_)) {
                    peek_ = Peek{start, *edge, *threat.actor_id, zone.now};
                    nextPeekAt_ = zone.now + 6.0;
                    shelteredAt_.reset();
                    break;
                }
            }
            if (!peek_) return {};
        }
        auto& peek = *peek_;
        peek.returning |= threat.facing_us || distanceXY(start, peek.edge) < 0.95 || zone.now - peek.started > 0.9;
        if (peek.returning && distanceXY(start, peek.cover) < 0.8) {
            peek_.reset();
            return {};
        }
        const auto target = peek.returning ? peek.cover : peek.edge;
        const auto move = direction(start, target, std::min(1.0, distanceXY(start, target) / 1.3));
        if (!navigator_.guard(move, false, me, zone, motion)) {
            peek_.reset();
            return {};
        }
        return Decision{{move}, peek.returning ? "peek-return" : "peek-edge", peek.returning && !peek.fired, peek.returning};
    }

    Decision rotate(const Json& obs, const Vec3& start, const Zone& zone, const Motion& motion,
                    const std::optional<Threat>& threat, const std::vector<Vec3>& enemies,
                    const RotationWindow& window, Clock::time_point started) {
        const auto& me = obs.at("self");
        Goals route;
        const bool insideNext = zone.inNext(start, motion.radius + 5);
        bool complete = insideNext;
        if (!insideNext) {
            const bool reachedPrevious = goals_.size() == 1 && distanceXY(start, goals_.front()) < 0.85;
            if (zone.now >= goalDeadline_ || reachedPrevious) {
                auto desired = rotationGoals(start, zone, motion);
                if (goals_.empty() || desired.empty() || reachedPrevious || distanceXY(goals_.front(), desired.front()) > 10) {
                    goals_ = std::move(desired);
                    ++routeId_;
                }
                goalDeadline_ = zone.now + 1.1;
            }
            auto plan = navigator_.plan(start, goals_, zone, motion, routeId_, started + 26ms);
            route = std::move(plan.route);
            complete = plan.complete;
        }
        const double eta = navigator_.pathTime(start, route, motion.walk);
        if (complete) routeEstimate_ = eta;
        const double energy = me.value("energy", 0.0), maximum = me.value("max_energy", 100.0);
        const bool grounded = me.value("grounded", true);
        const bool outside = !me.value("in_zone", true);
        const double remaining = std::max(0.0, zone.end - zone.now);
        if (grounded && zone.now >= considerFlightAt_ && !insideNext && energy > maximum * 0.35 &&
            (goals_.empty() || navigator_.searchExhausted() || (complete && eta > remaining * 1.2))) {
            considerFlightAt_ = zone.now + 0.7;
            flight_ = navigator_.flightPlan(start, goals_, zone, motion, energy, maximum, started + 34ms);
            if (const auto airborne = fly(me, start, zone, motion)) return {*airborne, "rotate-air"};
        }
        auto target = pickup(obs, start, threat, zone, motion, route, complete);
        if (!target && !route.empty()) target = route.front();
        if (!target && !insideNext) target = recover(start, zone, motion, started + 34ms, enemies);
        if (target && !enemies.empty() && !avoidsApproach(start, horizontal(*target), enemies)) {
            const auto escape = recover(start, zone, motion, started + 34ms, enemies);
            if (escape) target = escape;
        }
        monitorProgress(start, target, zone.now);
        const bool boost = grounded && target && !insideNext && (outside || eta + 3 > window.secondsUntilRisk) &&
            energy > 0.2 * maximum + motion.boostDrain * 0.2;
        auto command = groundCommand(start, target, me, zone, motion, boost, enemies);
        // A zone emergency can require passing a threat when every safe detour is blocked.
        if (window.urgent && target && !enemies.empty() && (outside || window.secondsUntilRisk < 1.5) &&
            std::hypot(command.move[0], command.move[1]) < 0.12) {
            command = groundCommand(start, target, me, zone, motion, boost);
        }
        if (window.urgent && (outside || zone.clearance(start, zone.now + 0.55) < 0) &&
            std::hypot(command.move[0], command.move[1]) < 0.12) {
            // If the boundary is already faster than the mech, keep moving toward recovery.
            // Freeze the current circle for collision checks and score movement against the live one.
            auto current = zone;
            current.nextCenter = current.center;
            current.nextRadius = current.radius;
            const auto safeStep = recover(start, current, motion, started + 36ms, enemies, zone.nextCenter);
            const auto forcedStep = safeStep ? safeStep : recover(start, current, motion, started + 37ms, {}, zone.nextCenter);
            if (forcedStep) {
                const auto lead = groundCommand(start, forcedStep, me, current, motion, boost, safeStep ? enemies : std::vector<Vec3>{});
                const Vec2 projected{start[0] + lead.move[0] * 4, start[1] + lead.move[1] * 4};
                if (zone.clearance(projected, zone.now + 0.55) > zone.clearance(start, zone.now + 0.55)) {
                    command = lead;
                }
            }
        }
        return {command, outside ? "recover-zone" : "rotate"};
    }

    std::optional<Vec3> pickup(const Json& obs, const Vec3& start, const std::optional<Threat>& threat,
                               const Zone& zone, const Motion& motion, const Goals& route, bool complete) {
        if (threat || !threats_.can_collect_cubes(obs) || !obs.at("self").at("in_zone").get<bool>() ||
            (!complete && !zone.inNext(start, motion.radius + 4))) {
            return {};
        }
        const auto cubes = safe_cubes(obs, threat, pickupRadius_, [&](const Vec2& xy) {
            return zone.clearance(xy, zone.now) > motion.radius;
        });
        const double base = navigator_.pathTime(start, route, motion.walk);
        for (std::size_t i = 0; i < std::min<std::size_t>(4, cubes.size()); ++i) {
            const auto& cube = cubes[i].pos;
            if (distanceXY(start, cube) <= pickupRadius_ && std::abs(start[2] - cube[2]) <= 4) {
                continue;
            }
            const auto point = navigator_.support(horizontal(cube), motion, cube[2] + 4);
            if (!point || std::abs((*point)[2] - cube[2]) > 4 || !navigator_.direct(start, *point, zone, motion) ||
                (!route.empty() && !navigator_.walkable(*point, route.front(), motion))) {
                continue;
            }
            Goals path{*point};
            path.insert(path.end(), route.begin(), route.end());
            const double total = navigator_.pathTime(start, path, motion.walk);
            if (total - base <= 3 && navigator_.pathSafe(start, path, zone, motion.walk) &&
                zone.inNext(path.back(), motion.radius) &&
                (zone.inNext(start, motion.radius) || zone.now + total + 2 <= zone.end)) {
                return point;
            }
        }
        return {};
    }

    std::optional<Vec3> recover(const Vec3& start, const Zone& zone, const Motion& motion,
                                Clock::time_point deadline, const std::vector<Vec3>& enemies,
                                std::optional<Vec2> destination = {}) {
        const Vec2 center = destination.value_or(zone.nextRadius < zone.radius ? zone.nextCenter : zone.circle(zone.now + 0.8).first);
        const double yaw = std::atan2(center[1] - start[1], center[0] - start[0]);
        double bestDistance = distanceXY(start, center) - 0.1;
        std::optional<Vec3> best;
        for (const double length : {7.0, 3.0}) {
            for (const int offset : angles) {
                if (Clock::now() >= deadline) {
                    return best;
                }
                const double angle = yaw + radians(offset);
                const Vec2 xy{start[0] + length * std::cos(angle), start[1] + length * std::sin(angle)};
                const double distance = distanceXY(xy, center);
                if (distance >= bestDistance || !avoidsApproach(start, xy, enemies)) {
                    continue;
                }
                const auto point = navigator_.support(xy, motion, start[2] + motion.step);
                if (point && navigator_.direct(start, *point, zone, motion)) {
                    best = point;
                    bestDistance = distance;
                }
            }
        }
        return best;
    }

    void monitorProgress(const Vec3& start, const std::optional<Vec3>& target, double now) {
        if (!target || distanceXY(start, *target) < 1) {
            progress_.reset();
            return;
        }
        if (!progress_ || distanceXY(progress_->target, *target) > 2) {
            progress_ = Progress{*target, start, now};
            return;
        }
        if (now - progress_->time >= 1.0) {
            if (distanceXY(progress_->origin, *target) - distanceXY(start, *target) < 0.75) {
                navigator_.reset();
                goalDeadline_ = -infinity;
            }
            progress_ = Progress{*target, start, now};
        }
    }

    Command groundCommand(const Vec3& start, const std::optional<Vec3>& target, const Json& me,
                          const Zone& zone, const Motion& motion, bool boost, const std::vector<Vec3>& enemies = {}) {
        const auto acceptable = [&](const Vec2& move, bool sprint) {
            const Vec2 ahead{start[0] + 4 * move[0], start[1] + 4 * move[1]};
            return avoidsApproach(start, ahead, enemies) && navigator_.guard(move, sprint, me, zone, motion);
        };
        const Vec2 move = target ? direction(start, *target, std::min(1.0, distanceXY(start, *target) / 1.3)) : Vec2{};
        if (acceptable(move, boost)) {
            return {move, boost};
        }
        if (boost && acceptable(move, false)) {
            return {move};
        }
        if (navigator_.guard({}, false, me, zone, motion)) {
            if (!target || distanceXY(start, *target) < 1.2) {
                return {};
            }
            const double yaw = std::atan2(move[1], move[0]);
            for (const int offset : {30, -30, 60, -60, 90, -90}) {
                const double angle = yaw + radians(offset);
                const Vec2 adjusted{0.5 * std::cos(angle), 0.5 * std::sin(angle)};
                const Vec2 ahead{start[0] + adjusted[0] * 3, start[1] + adjusted[1] * 3};
                if (distanceXY(ahead, *target) < distanceXY(start, *target) + 0.1 &&
                    acceptable(adjusted, false)) {
                    return {adjusted};
                }
            }
            return {};
        }
        const Vec2 center = zone.circle(zone.now + 0.4).first;
        for (const double speed : {0.66, 1.0}) {
            const auto inward = direction(start, center, speed);
            if (acceptable(inward, false)) {
                return {inward};
            }
        }
        return {};
    }

    std::optional<Command> fly(const Json& me, const Vec3& start, const Zone& zone, const Motion& motion) {
        if (!flight_) {
            return {};
        }
        auto& flight = *flight_;
        const double energy = me.at("energy").get<double>();
        const double maximum = me.at("max_energy").get<double>();
        const auto velocity = me.at("vel").get<Vec3>();
        if (me.at("grounded").get<bool>()) {
            if (flight.airborne || energy <= 0.2 * maximum) {
                flight_.reset();
                return {};
            }
            if (distanceXY(start, flight.takeoff) > 1.0 || std::hypot(velocity[0], velocity[1]) > 2) {
                return groundCommand(start, flight.takeoff, me, zone, motion, false);
            }
            const Vec3 vertical{start[0], start[1], flight.apex};
            if (!arena_.flightSegment(start, vertical, motion.radius, motion.height) ||
                !zone.segment(start, start, zone.now, flight.climb) ||
                !zone.segment(start, flight.landing, zone.now + flight.climb, flight.seconds - flight.climb)) {
                flight_.reset();
                return {};
            }
            flight.airborne = true;
            return Command{{}, false, true};
        }
        if (!zone.segment(start, flight.landing, zone.now, distanceXY(start, flight.landing) / motion.walk + 0.3)) {
            for (const auto& xy : {zone.circle(zone.now + 0.6).first, horizontal(start)}) {
                const auto point = navigator_.support(xy, motion, start[2] + 0.1);
                if (point && zone.segment(start, *point, zone.now, distanceXY(start, *point) / motion.walk)) {
                    flight.landing = *point;
                    break;
                }
            }
        }
        const double offset = distanceXY(start, flight.landing);
        const double speed = std::min(1.0, std::sqrt(2 * motion.airAcceleration * std::max(0.0, offset - 0.35)) / motion.walk);
        const auto move = start[2] >= flight.apex - 0.4 ? direction(start, flight.landing, speed) : Vec2{};
        const bool jet = offset > 1.1 && energy > 0.2 * maximum + 0.2 * motion.jetDrain &&
            start[2] + velocity[2] * 0.16 < flight.apex + 0.4;
        return navigator_.guard(move, jet, me, zone, motion, true) ? Command{move, jet} : Command{};
    }

    Json config_;
    Arena arena_;
    Navigator navigator_;
    ThreatTracker threats_;
    double pickupRadius_;
    std::optional<Flight> flight_;
    double considerFlightAt_ = -infinity;
    Goals goals_;
    std::optional<std::pair<double, double>> rotationPhase_;
    std::optional<double> routeEstimate_;
    std::optional<RetreatIntent> retreat_;
    std::optional<Peek> peek_;
    std::optional<ActiveHop> hop_;
    std::optional<double> shelteredAt_;
    std::optional<double> scanHeading_;
    double nextPeekAt_ = -infinity;
    double nextHopAt_ = -infinity;
    double goalDeadline_ = -infinity;
    std::size_t routeId_ = 0;
    std::optional<Progress> progress_;
};

void send(const Json& message) {
    std::cout << message.dump() << '\n' << std::flush;
}

// Read every complete line already available on the Linux pipe before deciding.
// That keeps a single expensive decision from causing stale responses to queue up.
class Inbox {
public:
    bool readBatch() {
        if (!readSome()) {
            return false;
        }
        for (;;) {
            pollfd descriptor{STDIN_FILENO, POLLIN, 0};
            const int ready = ::poll(&descriptor, 1, 0);
            if (ready < 0 && errno == EINTR) {
                continue;
            }
            if (ready < 0) {
                throw std::system_error(errno, std::generic_category(), "poll stdin");
            }
            if (ready == 0 || !(descriptor.revents & (POLLIN | POLLHUP)) || !readSome()) {
                return true;
            }
        }
    }

    template <typename Handler>
    bool consume(Handler handle) {
        std::size_t start = 0;
        for (std::size_t end; (end = bytes_.find('\n', start)) != std::string::npos; start = end + 1) {
            auto message = Json::parse(bytes_.begin() + start, bytes_.begin() + end, nullptr, false);
            if (message.is_object() && !handle(std::move(message))) {
                return false;
            }
        }
        bytes_.erase(0, start);
        return true;
    }

private:
    bool readSome() {
        char block[32768];
        ssize_t size;
        do {
            size = ::read(STDIN_FILENO, block, sizeof(block));
        } while (size < 0 && errno == EINTR);
        if (size < 0) {
            throw std::system_error(errno, std::generic_category(), "read stdin");
        }
        bytes_.append(block, static_cast<std::size_t>(size));
        return size != 0;
    }

    std::string bytes_;
};

int run() {
    Inbox inbox;
    std::unique_ptr<Bot> bot;
    Json lastMove = Json::object();
    while (inbox.readBatch()) {
        std::optional<Json> latest;
        if (!inbox.consume([&](Json message) {
            const auto type = message.value("type", std::string{});
            if (type == "end") {
                return false;
            }
            if (type == "hello") {
                bot = std::make_unique<Bot>(message);
                lastMove = Json::object();
                send({{"type", "ready"}, {"name", "circle-guard"}});
            } else if (type == "obs") {
                latest = std::move(message);
            }
            return true;
        })) {
            return 0;
        }
        if (!bot || !latest) {
            continue;
        }
        Json action;
        try {
            action = bot->act(*latest);
            lastMove = Json::object();
            for (const auto* field : {"move", "aim", "boost"}) {
                if (action.contains(field)) {
                    lastMove[field] = action[field];
                }
            }
        } catch (const std::exception& error) {
            std::cerr << "Decision failed: " << error.what() << '\n';
            action = lastMove;
        }
        action["tick"] = latest->at("tick");
        send(action);
    }
    return 0;
}

} // namespace
} // namespace mr

int main() {
    std::ios::sync_with_stdio(false);
    try {
        return mr::run();
    } catch (const std::exception& error) {
        std::cerr << "Bot stopped: " << error.what() << '\n';
        return 1;
    }
}
