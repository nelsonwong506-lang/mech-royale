#include "tactics.hpp"

#include "arena.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>
#include <tuple>

namespace mr {
namespace {

constexpr double RetaliationSeconds = 3.0;
constexpr double CubeDamageQuietSeconds = 5.0;
constexpr double ProjectileLookahead = 0.4;
constexpr double CubeDetourMeters = 20.0;
constexpr double Tau = 6.283185307179586;

const Json& field(const Json& value, const char* key) {
    static const Json missing;
    const auto item = value.find(key);
    return item == value.end() ? missing : *item;
}

double number(const Json& value, const char* key, double fallback) {
    const auto& item = field(value, key);
    return item.is_number() ? item.get<double>() : fallback;
}

std::string_view text(const Json& value) {
    const auto* string = value.get_ptr<const Json::string_t*>();
    return string ? std::string_view(*string) : std::string_view{};
}

Vec3 point(const Json& value) {
    return {value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
}

Vec2 pair(const Json& value) {
    return {value.at(0).get<double>(), value.at(1).get<double>()};
}

Vec3 above(Vec3 position, double height) {
    position[2] += height;
    return position;
}

double distance_3d(const Vec3& first, const Vec3& second) {
    return std::hypot(first[0] - second[0], first[1] - second[1], first[2] - second[2]);
}

Vec2 angles(const Vec3& origin, const Vec3& target) {
    return {std::atan2(target[1] - origin[1], target[0] - origin[0]),
            std::atan2(target[2] - origin[2], distanceXY(origin, target))};
}

double match_time(const Json& obs) {
    return number(obs, "t", number(obs, "tick", 0.0) / 20.0);
}

std::optional<int> actor_id(const Json& object, const char* key) {
    const double id = number(object, key, -1.0);
    if (id >= 0 && id < 16 && std::trunc(id) == id) {
        return static_cast<int>(id);
    }
    return std::nullopt;
}

struct Projectile {
    Vec3 position;
    Vec3 velocity;
    bool mortar;
    double gravity;
};

std::optional<double> collision_time(const Vec3& position, const Vec3& velocity,
                                      double radius, double height, const Projectile& projectile,
                                      double best_time, const Arena& arena) {
    const auto [px, py, pz] = projectile.position;
    const auto [vx, vy, vz] = projectile.velocity;
    const double dx = px - position[0];
    const double dy = py - position[1];
    const double rvx = vx - velocity[0];
    const double rvy = vy - velocity[1];
    const double speed_sq = rvx * rvx + rvy * rvy;
    if (speed_sq < 1) {
        return std::nullopt;
    }
    const double time = -(dx * rvx + dy * rvy) / speed_sq;
    if (time <= 0 || time > ProjectileLookahead || time >= best_time) {
        return std::nullopt;
    }
    const double closest_x = dx + rvx * time;
    const double closest_y = dy + rvy * time;
    if (closest_x * closest_x + closest_y * closest_y > radius * radius) {
        return std::nullopt;
    }
    const Vec3 shell{px + vx * time, py + vy * time, pz + vz * time - projectile.gravity * time * time / 2};
    const double feet = position[2] + velocity[2] * time;
    if (shell[2] < feet - 0.5 || shell[2] > feet + height + 0.5 || arena.blocksLine(projectile.position, shell)) {
        return std::nullopt;
    }
    return time;
}

bool could_have_fired(const Json& opponent, const Projectile& projectile, double size_per_cube,
                       double mech_radius, double muzzle_height, double turn_rate,
                       double pitch_rate, const Arena& arena) {
    const auto& aim = field(opponent, "aim");
    const auto& weapons = field(opponent, "weapons");
    const std::string_view kind = projectile.mortar ? "mortar" : "gun";
    if (aim.is_null() || std::none_of(weapons.begin(), weapons.end(), [kind](const auto& weapon) {
            return text(weapon) == kind;
        })) {
        return false;
    }
    const auto [px, py, pz] = projectile.position;
    const auto [vx, vy, vz] = projectile.velocity;
    const auto [ax, ay, az] = point(opponent.at("pos"));
    const auto [avx, avy, avz] = point(opponent.at("vel"));
    const double dx = px - ax;
    const double dy = py - ay;
    const double rvx = vx - avx;
    const double rvy = vy - avy;
    const double speed_sq = rvx * rvx + rvy * rvy;
    if (speed_sq < 1) {
        return false;
    }
    const double age = (dx * rvx + dy * rvy) / speed_sq;
    if (age < 0 || age > (projectile.mortar ? 1.5 : 0.9)) {
        return false;
    }
    const double scale = 1 + size_per_cube * number(opponent, "cubes", 0.0);
    const double height_at_launch = pz - (vz - avz) * age - projectile.gravity * age * age / 2;
    const double error = std::hypot(dx - age * rvx, dy - age * rvy, height_at_launch - az - muzzle_height * scale);
    if (error > mech_radius * scale + 1.5) {
        return false;
    }

    const auto [yaw, pitch] = pair(aim);
    const double launch_yaw = std::atan2(vy, vx);
    const double launch_pitch = std::atan2(vz + projectile.gravity * age, std::hypot(vx, vy));
    if (std::abs(std::remainder(launch_yaw - yaw, Tau)) > std::min(0.8, 0.15 + turn_rate * age) ||
        std::abs(launch_pitch - pitch) > std::min(0.7, 0.15 + pitch_rate * age)) {
        return false;
    }
    const Vec3 origin{ax - avx * age, ay - avy * age, az - avz * age + muzzle_height * scale};
    return projectile.mortar || !arena.blocksLine(origin, projectile.position);
}

const Json& gun(const Json& me) {
    static const Json unarmed;
    for (const auto& weapon : field(me, "weapons")) {
        if (text(field(weapon, "kind")) == "gun") {
            return weapon;
        }
    }
    return unarmed;
}

Vec3 muzzle(const Json& me) {
    return above(point(me.at("pos")), number(field(me, "stats"), "muzzle_height", 7.0));
}

struct Shot {
    Vec3 origin;
    Vec3 target;
    Vec3 intercept;
};

Shot shot_points(const Json& me, const Threat& threat, const Json& weapon_stats) {
    const Vec3 origin = muzzle(me);
    const Vec3 target = above(*threat.attack_position, threat.attack_height / 2);
    const double speed = std::max(1.0, number(weapon_stats, "speed", 252.0));
    const Vec3 velocity = threat.attack_velocity.value_or(Vec3{});
    Vec3 intercept = target;
    for (int step = 0; step < 2; ++step) {
        const double flight_time = distance_3d(origin, intercept) / speed;
        for (int axis = 0; axis < 3; ++axis) {
            intercept[axis] = target[axis] + velocity[axis] * flight_time;
        }
    }
    return {origin, target, intercept};
}

}  // namespace

ThreatTracker::ThreatTracker(const Json& config)
    : mech_radius_(number(field(config, "mech"), "radius", 2.5)),
      mech_height_(number(field(config, "mech"), "height", 9.0)),
      muzzle_height_(number(field(config, "mech"), "muzzle_height", 7.0)),
      turn_rate_(number(field(config, "mech"), "turn_rate", 4.0)),
      pitch_rate_(number(field(config, "mech"), "pitch_rate", 3.0)),
      size_per_cube_(number(field(config, "cubes"), "size_per_cube", 0.05)),
      mortar_gravity_(number(field(field(config, "weapons"), "mortar"), "gravity", 22.0)) {}

bool ThreatTracker::can_collect_cubes(const Json& obs) const {
    return match_time(obs) - last_damage_at_ >= CubeDamageQuietSeconds;
}

std::optional<Threat> ThreatTracker::update(const Json& obs, const Arena& arena) {
    const auto& me = obs.at("self");
    const auto self_id = actor_id(me, "id");
    const Vec3 position = point(me.at("pos"));
    const auto& self_stats = field(me, "stats");
    const double now = match_time(obs);
    std::array<const Json*, 16> enemies{};
    for (const auto& actor : field(obs, "mechs")) {
        if (text(field(actor, "kind")) == "mech") {
            const auto id = actor_id(actor, "id");
            if (id) {
                enemies[*id] = &actor;
            }
        }
    }
    for (auto& attack : attacks_) {
        if (now - attack.time >= RetaliationSeconds) {
            attack = {};
        }
    }
    for (const auto& event : field(obs, "events")) {
        if (text(field(event, "type")) != "damaged") {
            continue;
        }
        last_damage_at_ = now;
        const auto id = actor_id(event, "by");
        const auto weapon = text(field(event, "weapon"));
        if (!id || id == self_id || weapon == "zone" || weapon == "npc") {
            continue;
        }
        attacks_[*id].time = now;
        const auto& bearing = field(event, "bearing");
        if (!enemies[*id] && bearing.is_number()) {
            const double angle = bearing.get<double>();
            attacks_[*id].position = Vec3{position[0] + 40 * std::cos(angle),
                                          position[1] + 40 * std::sin(angle), position[2]};
        }
    }

    const auto& projectiles = field(obs, "projectiles");
    if (!projectiles.empty()) {
        const Vec3 velocity = point(me.at("vel"));
        const double radius = number(self_stats, "radius", 2.5) + 0.5;
        const double height = number(self_stats, "height", 9.0);
        double soonest = std::numeric_limits<double>::infinity();
        std::optional<Projectile> imminent;
        for (const auto& object : projectiles) {
            const auto kind = text(field(object, "kind"));
            if (kind != "gun" && kind != "mortar") {
                continue;
            }
            const Projectile projectile{point(object.at("pos")), point(object.at("vel")),
                                        kind == "mortar", kind == "mortar" ? mortar_gravity_ : 0.0};
            const auto time = collision_time(position, velocity, radius, height, projectile, soonest, arena);
            if (time) {
                soonest = *time;
                imminent = projectile;
            }
        }
        if (imminent) {
            std::optional<int> shooter;
            for (int id = 0; id < static_cast<int>(enemies.size()); ++id) {
                if (!enemies[id] || !could_have_fired(*enemies[id], *imminent, size_per_cube_, mech_radius_,
                                                       imminent->mortar ? mech_height_ + 0.5 : muzzle_height_,
                                                       turn_rate_, pitch_rate_, arena)) {
                    continue;
                }
                // Ownerless projectiles only justify return fire under a unique match.
                if (shooter) {
                    shooter.reset();
                    break;
                }
                shooter = id;
            }
            if (shooter) {
                attacks_[*shooter].time = now;
            } else {
                const auto [x, y, z] = imminent->position;
                const double vx = imminent->velocity[0];
                const double vy = imminent->velocity[1];
                const double speed = std::max(1.0, std::hypot(vx, vy));
                unknown_position_ = Vec3{x - 40 * vx / speed, y - 40 * vy / speed, z};
                unknown_until_ = now + 1.0;
            }
        }
    }

    int attacker = -1;
    for (int id = 0; id < static_cast<int>(attacks_.size()); ++id) {
        auto& attack = attacks_[id];
        if (now - attack.time >= RetaliationSeconds) {
            continue;
        }
        if (enemies[id]) {
            attack.position = point(enemies[id]->at("pos"));
        }
        if (attacker < 0 || std::tuple{!enemies[id], -attack.time} <
                                std::tuple{!enemies[attacker], -attacks_[attacker].time}) {
            attacker = id;
        }
    }
    if (attacker >= 0) {
        const auto* enemy = enemies[attacker];
        const double scale = 1 + size_per_cube_ * (enemy ? number(*enemy, "cubes", 0.0) : 0.0);
        Threat threat;
        threat.position = attacks_[attacker].position;
        threat.actor_id = attacker;
        threat.hostile = true;
        threat.attack_id = attacker;
        if (enemy) {
            threat.attack_position = threat.position;
            threat.attack_velocity = point(enemy->at("vel"));
        }
        threat.attack_radius = mech_radius_ * scale;
        threat.attack_height = mech_height_ * scale;
        return threat;
    }
    if (now < unknown_until_) {
        Threat threat;
        threat.position = unknown_position_;
        threat.hostile = true;
        return threat;
    }
    std::optional<Threat> nearest;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (int id = 0; id < static_cast<int>(enemies.size()); ++id) {
        if (!enemies[id]) {
            continue;
        }
        const Vec3 enemy_position = point(enemies[id]->at("pos"));
        const double distance = distanceXY(position, enemy_position);
        if (distance < nearest_distance) {
            Threat threat;
            threat.position = enemy_position;
            threat.actor_id = id;
            nearest = threat;
            nearest_distance = distance;
        }
    }
    return nearest;
}

std::optional<Vec2> retreat_aim(const Json& obs, const std::optional<Threat>& threat) {
    if (!threat || !threat->position) {
        return std::nullopt;
    }
    const auto& me = obs.at("self");
    if (threat->attack_position) {
        const Shot shot = shot_points(me, *threat, field(gun(me), "stats"));
        return angles(shot.origin, shot.intercept);
    }
    return angles(muzzle(me), above(*threat->position, 4.5));
}

bool return_fire(const Json& obs, const std::optional<Threat>& threat, const Vec2& move, const Arena& arena) {
    if (!threat || !threat->hostile || !threat->attack_id || !threat->attack_position) {
        return false;
    }
    const auto& me = obs.at("self");
    const auto& weapon = gun(me);
    if (weapon.is_null() || number(weapon, "cooldown", 0.0) > 0.1 ||
        number(weapon, "reload", 0.0) > 0 || number(weapon, "ammo", 1.0) <= 0) {
        return false;
    }
    const auto position = point(me.at("pos"));
    const double dx = (*threat->attack_position)[0] - position[0];
    const double dy = (*threat->attack_position)[1] - position[1];
    // Lateral movement can reach cover, but closing on the attacker is not withdrawal.
    if (std::hypot(move[0], move[1]) < 0.15 || dx * move[0] + dy * move[1] > 0.05 * std::hypot(dx, dy)) {
        return false;
    }
    const auto& weapon_stats = field(weapon, "stats");
    const Shot shot = shot_points(me, *threat, weapon_stats);
    const double distance = distance_3d(shot.origin, shot.intercept);
    if (distance > number(weapon_stats, "range", 230.0)) {
        return false;
    }
    const auto [yaw, pitch] = angles(shot.origin, shot.intercept);
    const auto [actual_yaw, actual_pitch] = pair(me.at("aim"));
    const double yaw_error = std::remainder(yaw - actual_yaw, Tau) * std::cos(pitch);
    const double pitch_error = pitch - actual_pitch;
    const double tolerance = std::min(0.18, std::atan2(0.75 * threat->attack_radius, std::max(distance, 1.0)));
    if (yaw_error * yaw_error + pitch_error * pitch_error > tolerance * tolerance) {
        return false;
    }
    const double horizontal = distance * std::cos(actual_pitch);
    const Vec3 actual_end{shot.origin[0] + horizontal * std::cos(actual_yaw),
                          shot.origin[1] + horizontal * std::sin(actual_yaw),
                          shot.origin[2] + distance * std::sin(actual_pitch)};
    return !arena.blocksLine(shot.origin, shot.target) && !arena.blocksLine(shot.origin, shot.intercept) &&
           !arena.blocksLine(shot.origin, actual_end);
}

std::vector<Cube> safe_cubes(const Json& obs, const std::optional<Threat>& threat,
                             double pickup_radius, const std::function<bool(const Vec2&)>& safe_point) {
    const auto& events = field(obs, "events");
    if (threat || std::any_of(events.begin(), events.end(), [](const auto& event) {
            return text(field(event, "type")) == "damaged";
        })) {
        return {};
    }
    const auto position = point(obs.at("self").at("pos"));
    for (const auto& actor : field(obs, "mechs")) {
        if (text(field(actor, "kind")) == "npc" && distanceXY(point(actor.at("pos")), position) < 45) {
            return {};
        }
    }
    std::vector<std::pair<double, Cube>> candidates;
    for (const auto& cube : field(obs, "cubes")) {
        const Vec3 cube_pos = point(cube.at("pos"));
        const double distance = distanceXY(cube_pos, position);
        if (distance <= CubeDetourMeters + pickup_radius && safe_point({cube_pos[0], cube_pos[1]})) {
            candidates.push_back({distance, Cube{cube.at("id").get<double>(), cube_pos}});
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const auto& first, const auto& second) {
        return std::tuple{first.first, first.second.id} < std::tuple{second.first, second.second.id};
    });
    std::vector<Cube> cubes;
    cubes.reserve(candidates.size());
    for (const auto& candidate : candidates) {
        cubes.push_back(candidate.second);
    }
    return cubes;
}

int cover_quality(const Vec3& position, const std::optional<Threat>& threat, const Arena& arena, double height) {
    if (!threat || !threat->position) {
        return 0;
    }
    const Vec3 source = above(*threat->position, threat->attack_height * 0.8);
    int protected_points = 0;
    for (const double fraction : {0.2, 0.5, 0.9}) {
        protected_points += arena.blocksLine(source, above(position, height * fraction));
    }
    return protected_points;
}

}  // namespace mr
