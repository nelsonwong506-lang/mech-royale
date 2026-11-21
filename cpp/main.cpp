#include "arena.hpp"
#include "navigation.hpp"
#include "tactics.hpp"

#include <algorithm>
#include <cerrno>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <tuple>
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
        const bool grounded = me.at("grounded").get<bool>();
        const double energy = me.at("energy").get<double>();
        const double maximum = me.at("max_energy").get<double>();

        auto command = fly(me, start, zone, motion);
        if (!command) {
            updateGoals(start, threat, zone, motion, started + 9ms);
            auto [route, complete] = navigator_.plan(start, goals_, zone, motion, routeId_, started + 25ms);
            if (!alternatives_.empty() && navigator_.searchExhausted()) {
                goals_ = std::exchange(alternatives_, {});
                auto fallback = navigator_.plan(start, goals_, zone, motion, ++routeId_, started + 26ms);
                route = std::move(fallback.route);
                complete = fallback.complete;
            }
            const double remaining = std::max(0.0, zone.end - zone.now);
            const double walkEta = navigator_.pathTime(start, route, motion.walk);
            const double length = walkEta * motion.walk;
            const double usable = std::max(0.0, energy - 0.2 * maximum);
            const double boostDistance = std::min(length, motion.boost * usable / std::max(1.0, motion.boostDrain));
            const double bestGroundEta = boostDistance / motion.boost + (length - boostDistance) / motion.walk;
            const bool tooLate = complete && bestGroundEta > remaining;
            const bool noRoute = goals_.empty() || navigator_.searchExhausted();
            if (grounded && zone.now >= considerFlightAt_ && !zone.inNext(start) &&
                energy > maximum * 0.35 && (noRoute || tooLate)) {
                considerFlightAt_ = zone.now + 0.7;
                flight_ = navigator_.flightPlan(start, goals_, zone, motion, energy, maximum, started + 35ms);
                command = fly(me, start, zone, motion);
            }
            if (!command) {
                auto target = pickup(obs, start, threat, zone, motion, route, complete);
                if (!target && !route.empty()) {
                    target = route.front();
                }
                if (!target && !zone.inNext(start, motion.radius)) {
                    target = recover(start, zone, motion, started + 33ms);
                }
                monitorProgress(start, target, zone.now);
                const bool boost = grounded && target && !zone.inNext(start) &&
                    (!me.at("in_zone").get<bool>() || walkEta + 2 > remaining) &&
                    energy > 0.2 * maximum + motion.boostDrain * 0.2;
                command = groundCommand(start, target, me, zone, motion, boost);
            }
        }

        Json action = {{"move", command->move}, {"boost", command->boost}, {"jump", command->jump}};
        if (const auto aim = retreat_aim(obs, threat)) {
            action["aim"] = *aim;
        } else if (std::hypot(command->move[0], command->move[1]) > 0.1) {
            action["aim"] = Vec2{std::atan2(command->move[1], command->move[0]), 0.0};
        }
        std::array fire{false, false};
        const auto& weapons = me.at("weapons");
        if (return_fire(obs, threat, command->move, arena_)) {
            for (std::size_t i = 0; i < std::min(fire.size(), weapons.size()); ++i) {
                if (weapons[i].is_object() && weapons[i].value("kind", "") == std::string_view("gun")) {
                    fire[i] = true;
                    break;
                }
            }
        }
        action["fire"] = fire;
        if (!threat) {
            action["reload"] = std::any_of(weapons.begin(), weapons.end(), [](const Json& weapon) {
                return weapon.is_object() && weapon.value("kind", "") == std::string_view("gun") &&
                    weapon.value("ammo", 0.0) <= 3;
            });
        }
        if (!obs.at("offer").is_null()) {
            action["pick"] = 0;
        }
        return action;
    }

private:
    using Goals = std::vector<Vec3>;
    using GoalKey = std::tuple<double, double, bool, std::optional<int>, bool>;

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

    std::pair<Goals, Goals> retreatGoals(const Vec3& start, const std::optional<Threat>& threat,
                                               const Zone& zone, const Motion& motion, Goals rotation,
                                               Clock::time_point deadline) const {
        if (!threat->position) {
            return {std::move(rotation), {}};
        }
        if (zone.inNext(start, motion.radius + 4) && cover_quality(start, threat, arena_, motion.height) == 3) {
            return {{start}, {}};
        }
        const auto& position = *threat->position;
        const double away = std::atan2(start[1] - position[1], start[0] - position[0]);
        const double initialDistance = distanceXY(start, position);
        std::vector<std::tuple<int, double, Vec3>> candidates;
        for (const double radius : {13.0, 23.0, 6.0}) {
            for (const int offset : angles) {
                if (Clock::now() >= deadline) {
                    break;
                }
                const double angle = away + radians(offset);
                const Vec2 xy{start[0] + radius * std::cos(angle), start[1] + radius * std::sin(angle)};
                if (!zone.inNext(xy, std::min(motion.radius, 3.0))) {
                    continue;
                }
                const auto point = navigator_.support(xy, motion, start[2] + motion.step);
                if (!point || !zone.segment(start, *point, zone.now, radius / motion.walk)) {
                    continue;
                }
                const int cover = cover_quality(*point, threat, arena_, motion.height);
                const double gain = distanceXY(*point, position) - initialDistance;
                if (gain >= 1 || cover >= 2) {
                    candidates.emplace_back(cover, gain - 0.5 * radius, *point);
                }
            }
        }
        if (candidates.empty()) {
            return {std::move(rotation), {}};
        }
        std::sort(candidates.begin(), candidates.end(), std::greater<>{});
        const int bestCover = std::get<0>(candidates.front());
        Goals preferred, alternatives;
        for (const auto& candidate : candidates) {
            auto& list = std::get<0>(candidate) == bestCover ? preferred : alternatives;
            if (list.size() < 5) {
                list.push_back(std::get<2>(candidate));
            }
        }
        if (!zone.inNext(start, motion.radius + 4)) {
            alternatives.insert(alternatives.end(), rotation.begin(), rotation.begin() + std::min<std::size_t>(3, rotation.size()));
        }
        return {std::move(preferred), std::move(alternatives)};
    }

    void updateGoals(const Vec3& start, const std::optional<Threat>& threat,
                     const Zone& zone, const Motion& motion, Clock::time_point deadline) {
        const GoalKey key{zone.start, zone.end, threat.has_value(), threat ? threat->actor_id : std::nullopt,
                          threat && threat->hostile};
        const bool live = std::any_of(goals_.begin(), goals_.end(), [&](const Vec3& point) { return zone.inNext(point); });
        if (goalKey_ == key && zone.now < goalDeadline_ && (live || goals_.empty())) {
            return;
        }
        auto rotation = rotationGoals(start, zone, motion);
        auto [goals, alternatives] = threat
            ? retreatGoals(start, threat, zone, motion, std::move(rotation), deadline)
            : std::pair<Goals, Goals>{std::move(rotation), {}};
        const auto overlap = [&](const Vec3& old) {
            return std::any_of(goals.begin(), goals.end(), [&](const Vec3& next) { return distanceXY(old, next) <= 8; });
        };
        if ((goals.size() == 1 && goals.front() == start) || goals_.empty() || goalKey_ != key || !live ||
            (!goals.empty() && std::none_of(goals_.begin(), goals_.end(), overlap))) {
            goals_ = std::move(goals);
            alternatives_ = std::move(alternatives);
            ++routeId_;
        }
        goalKey_ = key;
        goalDeadline_ = zone.now + 0.7;
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
                                Clock::time_point deadline) {
        const Vec2 center = zone.nextRadius < zone.radius ? zone.nextCenter : zone.circle(zone.now + 0.8).first;
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
                if (distance >= bestDistance) {
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
                          const Zone& zone, const Motion& motion, bool boost) {
        const Vec2 move = target ? direction(start, *target, std::min(1.0, distanceXY(start, *target) / 1.3)) : Vec2{};
        if (navigator_.guard(move, boost, me, zone, motion)) {
            return {move, boost};
        }
        if (boost && navigator_.guard(move, false, me, zone, motion)) {
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
                    navigator_.guard(adjusted, false, me, zone, motion)) {
                    return {adjusted};
                }
            }
            return {};
        }
        const Vec2 center = zone.circle(zone.now + 0.4).first;
        for (const double speed : {0.66, 1.0}) {
            const auto inward = direction(start, center, speed);
            if (navigator_.guard(inward, false, me, zone, motion)) {
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
    Goals alternatives_;
    std::optional<GoalKey> goalKey_;
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
