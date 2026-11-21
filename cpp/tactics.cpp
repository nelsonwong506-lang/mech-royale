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
constexpr double FreshHostileDamageSeconds = 0.3;
constexpr double ProjectileLookahead = 0.4;
constexpr double CubeDetourMeters = 20.0;
constexpr double DuelEvidenceSeconds = 0.25;
constexpr double DuelMemorySeconds = 1.2;
constexpr double Tau = 6.283185307179586;
constexpr std::array<std::string_view, 4> WeaponKinds{"gun", "mortar", "laser", "lightning"};
constexpr std::size_t MortarIndex = 1;
constexpr std::size_t LaserIndex = 2;

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

bool segment_hits_mech(const Vec3& start, const Vec3& end, const Threat& mech) {
    constexpr double contact_slack = 0.15;
    const Vec3& feet = *mech.position;
    const double bottom = feet[2] - contact_slack;
    const double top = feet[2] + mech.attack_height + contact_slack;
    const double delta_z = end[2] - start[2];
    double begin = 0;
    double finish = 1;
    if (std::abs(delta_z) > 1e-8) {
        const auto [first, last] = std::minmax({(bottom - start[2]) / delta_z, (top - start[2]) / delta_z});
        begin = std::max(begin, first);
        finish = std::min(finish, last);
    } else if (start[2] < bottom || start[2] > top) {
        return false;
    }
    if (begin > finish) {
        return false;
    }
    const double dx = end[0] - start[0];
    const double dy = end[1] - start[1];
    const double length_squared = dx * dx + dy * dy;
    const double projection = length_squared > 1e-8
        ? ((feet[0] - start[0]) * dx + (feet[1] - start[1]) * dy) / length_squared : begin;
    const double nearest = std::clamp(projection, begin, finish);
    const double gap_x = start[0] + dx * nearest - feet[0];
    const double gap_y = start[1] + dy * nearest - feet[1];
    const double radius = mech.attack_radius + contact_slack;
    return gap_x * gap_x + gap_y * gap_y <= radius * radius;
}

double mortar_range(const Json& mortar) {
    const double speed = number(mortar, "speed", 72.0);
    const double gravity = std::max(1.0, number(mortar, "gravity", 22.0));
    return number(mortar, "range", speed * speed / gravity);
}

}  // namespace

ThreatTracker::ThreatTracker(const Json& config)
    : mech_radius_(number(field(config, "mech"), "radius", 2.5)),
      mech_height_(number(field(config, "mech"), "height", 9.0)),
      muzzle_height_(number(field(config, "mech"), "muzzle_height", 7.0)),
      turn_rate_(number(field(config, "mech"), "turn_rate", 4.0)),
      pitch_rate_(number(field(config, "mech"), "pitch_rate", 3.0)),
      size_per_cube_(number(field(config, "cubes"), "size_per_cube", 0.05)),
      mortar_gravity_(number(field(field(config, "weapons"), "mortar"), "gravity", 22.0)) {
    const auto& weapons = field(config, "weapons");
    weapon_ranges_ = {number(field(weapons, "gun"), "range", 230.0), mortar_range(field(weapons, "mortar")),
                      number(field(weapons, "laser"), "range", 280.0), number(field(weapons, "lightning"), "range", 45.0)};
}

Threat ThreatTracker::describe_enemy(const Json& actor, int id) const {
    const double scale = 1 + size_per_cube_ * number(actor, "cubes", 0.0);
    Threat threat;
    threat.position = point(actor.at("pos"));
    threat.actor_id = id;
    threat.attack_radius = mech_radius_ * scale;
    threat.attack_height = mech_height_ * scale;
    const auto& aim = field(actor, "aim");
    if (aim.is_array() && aim.size() >= 2) {
        threat.enemy_aim = pair(aim);
    }

    const auto& weapons = field(actor, "weapons");
    threat.enemy_muzzle_height = muzzle_height_ * scale;
    if (!weapons.is_array()) {
        // The protocol may omit loadout data. Every mech starts with a gun.
        threat.enemy_range = weapon_ranges_.front();
        return threat;
    }
    bool has_straight_weapon = false;
    for (const auto& weapon : weapons) {
        const auto kind = std::find(WeaponKinds.begin(), WeaponKinds.end(), text(weapon));
        if (kind == WeaponKinds.end()) {
            continue;
        }
        const auto index = static_cast<std::size_t>(kind - WeaponKinds.begin());
        threat.enemy_range = std::max(threat.enemy_range, weapon_ranges_[index]);
        has_straight_weapon = has_straight_weapon || index != MortarIndex;
    }
    if (threat.enemy_range > 0 && !has_straight_weapon) {
        threat.enemy_muzzle_height = (mech_height_ + 0.5) * scale;
    }
    return threat;
}

bool enemy_aims_at(const Threat& threat, const Vec3& feet, double radius, double height, const Arena& arena) {
    if (!threat.position || !threat.enemy_aim || threat.enemy_range <= 0) {
        return false;
    }
    const auto [enemy_yaw, enemy_pitch] = *threat.enemy_aim;
    const Vec3 source = above(*threat.position, threat.enemy_muzzle_height);
    const double horizontal_distance = distanceXY(source, feet);
    if (horizontal_distance > threat.enemy_range + radius) {
        return false;
    }
    const double yaw = std::atan2(feet[1] - source[1], feet[0] - source[0]);
    const double yaw_tolerance = std::clamp(std::atan2(radius, horizontal_distance) + radians(3),
                                            radians(5), radians(20));
    if (horizontal_distance > radius && std::abs(std::remainder(yaw - enemy_yaw, Tau)) > yaw_tolerance) {
        return false;
    }
    // Check the height hit by the aim first. Sampling the chassis lets a partially
    // exposed target count only when its exposed part is also inside the aim cone.
    const double aimed_height = std::clamp(source[2] - feet[2] + horizontal_distance * std::tan(enemy_pitch),
                                            height * 0.1, height * 0.95);
    for (const double target_height : {aimed_height, height * 0.5, height * 0.2, height * 0.9}) {
        const Vec3 target = above(feet, target_height);
        const double pitch = std::atan2(target[2] - source[2], horizontal_distance);
        if (std::abs(pitch - enemy_pitch) <= radians(5) &&
            distance_3d(source, target) <= threat.enemy_range + radius && !arena.blocksLine(source, target)) {
            return true;
        }
    }
    return false;
}

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
            if (id && id != self_id) {
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
        last_hostile_damage_at_ = now;
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

    std::optional<Threat> nearest;
    std::optional<Threat> facing;
    double nearest_distance = std::numeric_limits<double>::infinity();
    double facing_distance = std::numeric_limits<double>::infinity();
    const bool damaged_recently = now <= last_hostile_damage_at_ + FreshHostileDamageSeconds;
    const double radius = number(self_stats, "radius", mech_radius_);
    const double height = number(self_stats, "height", mech_height_);
    for (int id = 0; id < static_cast<int>(enemies.size()); ++id) {
        if (!enemies[id]) {
            continue;
        }
        Threat observed = describe_enemy(*enemies[id], id);
        const double distance = distanceXY(position, *observed.position);
        if (distance < nearest_distance) {
            nearest = observed;
            nearest_distance = distance;
        }
        const auto& charging = field(*enemies[id], "charging");
        if (charging.is_boolean() && charging.get<bool>() &&
            (!facing || !facing->laser_charging || distance < facing_distance)) {
            auto laser = observed;
            laser.enemy_range = weapon_ranges_[LaserIndex];
            laser.enemy_muzzle_height = muzzle_height_ * (1 + size_per_cube_ * number(*enemies[id], "cubes", 0.0));
            if (enemy_aims_at(laser, position, radius, height, arena)) {
                observed = laser;
                observed.laser_charging = true;
            }
        }
        const bool preferred = !facing || (observed.laser_charging && !facing->laser_charging) ||
                               (observed.laser_charging == facing->laser_charging && distance < facing_distance);
        if (preferred && (observed.laser_charging || enemy_aims_at(observed, position, radius, height, arena))) {
            observed.facing_us = true;
            observed.hostile = true;
            observed.damaged_recently = damaged_recently;
            observed.attack_id = id;
            observed.attack_position = observed.position;
            observed.attack_velocity = point(enemies[id]->at("vel"));
            facing = observed;
            facing_distance = distance;
        }
    }
    if (facing) {
        return facing;
    }
    if (attacker >= 0) {
        const auto* enemy = enemies[attacker];
        Threat threat = enemy ? describe_enemy(*enemy, attacker) : Threat{};
        threat.position = attacks_[attacker].position;
        threat.actor_id = attacker;
        threat.hostile = true;
        threat.damaged_recently = damaged_recently;
        threat.attack_id = attacker;
        if (enemy) {
            threat.attack_position = threat.position;
            threat.attack_velocity = point(enemy->at("vel"));
        }
        return threat;
    }
    if (now < unknown_until_) {
        Threat threat;
        threat.position = unknown_position_;
        threat.hostile = true;
        threat.damaged_recently = damaged_recently;
        return threat;
    }
    return nearest;
}

std::optional<Threat> ThreatTracker::duel_target(const Json& obs, const Arena& arena) {
    const auto& me = obs.at("self");
    const auto own_id = actor_id(me, "id");
    const auto& actors = field(obs, "mechs");
    const auto& beams = field(obs, "beams");
    const double now = match_time(obs);
    const auto& zone = field(obs, "zone");
    const auto& center = field(zone, "center");
    const Vec2 circle_center = center.is_array() && center.size() >= 2 ? pair(center) : Vec2{};
    const double circle_radius = number(zone, "radius", -1.0);
    std::array<const Json*, 16> opponents{};
    std::array<bool, 16> hurt_by_us{};
    std::array<bool, 16> lost_health{};
    for (const auto& event : field(obs, "events")) {
        if (text(field(event, "type")) == "hit") {
            if (const auto victim = actor_id(event, "target")) {
                hurt_by_us[*victim] = true;
            }
        }
    }
    std::size_t opponent_count = 0;
    for (const auto& actor : actors) {
        const auto id = actor_id(actor, "id");
        if (!id || id == own_id || text(field(actor, "kind")) != "mech" || opponents[*id]) {
            continue;
        }
        opponents[*id] = &actor;
        ++opponent_count;
        const double hp = number(actor, "hp", -1.0);
        const Vec3 position = point(actor.at("pos"));
        const bool in_zone = distanceXY(position, circle_center) <= circle_radius;
        const auto previous = health_samples_[*id];
        if (!hurt_by_us[*id] && in_zone && previous.in_zone && now > previous.time &&
            now <= previous.time + DuelEvidenceSeconds && hp >= 0 && previous.hp - hp > 0.5) {
            lost_health[*id] = true;
        }
        health_samples_[*id] = {now, hp, in_zone};
    }
    if (opponent_count < 2) {
        if (now <= last_duel_at_ + DuelMemorySeconds) {
            duel_times_ = {};
            last_duel_at_ = -std::numeric_limits<double>::infinity();
        }
        return std::nullopt;
    }
    if (now > last_duel_at_ + DuelMemorySeconds && beams.empty() &&
        std::none_of(lost_health.begin(), lost_health.end(), [](bool lost) {
            return lost;
        })) {
        return std::nullopt;
    }

    const auto& stats = field(me, "stats");
    const Vec3 own_position = point(me.at("pos"));
    const Vec3 eye = above(own_position, number(stats, "eye_height", 8.0));
    std::array<std::optional<Threat>, 16> seen;
    for (int id = 0; id < static_cast<int>(opponents.size()); ++id) {
        if (!opponents[id]) {
            continue;
        }
        auto actor = describe_enemy(*opponents[id], id);
        const auto body = above(*actor.position, actor.attack_height / 2);
        // The server continues reporting actors recently seen behind solid walls.
        if (!arena.blocksLine(eye, body)) {
            seen[id] = std::move(actor);
        }
    }

    const auto remember_duel = [&](int first, int second) {
        duel_times_[std::min(first, second)][std::max(first, second)] = now;
        last_duel_at_ = now;
    };
    for (const auto& beam : beams) {
        const auto kind = text(field(beam, "kind"));
        const auto shooter = actor_id(beam, "by");
        const auto& path = field(beam, "path");
        if ((kind != "laser" && kind != "lightning") || !shooter || !seen[*shooter] ||
            number(beam, "age", DuelEvidenceSeconds + 1) > DuelEvidenceSeconds || !path.is_array() || path.size() < 2) {
            continue;
        }
        const auto& source = *seen[*shooter];
        for (int id = 0; id < static_cast<int>(seen.size()); ++id) {
            if (id == *shooter || !seen[id]) {
                continue;
            }
            const auto& target = *seen[id];
            for (std::size_t index = 1; index < path.size(); ++index) {
                if (segment_hits_mech(point(path[index - 1]), point(path[index]), target) &&
                    enemy_aims_at(target, *source.position, source.attack_radius, source.attack_height, arena)) {
                    remember_duel(id, *shooter);
                    break;
                }
            }
        }
    }
    for (int id = 0; id < static_cast<int>(seen.size()); ++id) {
        if (!seen[id] || !lost_health[id]) {
            continue;
        }
        const auto& victim = *seen[id];
        for (int other = 0; other < static_cast<int>(seen.size()); ++other) {
            if (other == id || !seen[other]) {
                continue;
            }
            const auto& rival = *seen[other];
            if (enemy_aims_at(rival, *victim.position, victim.attack_radius, victim.attack_height, arena) &&
                enemy_aims_at(victim, *rival.position, rival.attack_radius, rival.attack_height, arena)) {
                remember_duel(id, other);
            }
        }
    }

    std::array<bool, 16> fighting{};
    for (std::size_t first = 0; first < seen.size(); ++first) {
        for (std::size_t second = first + 1; second < seen.size(); ++second) {
            auto& last_evidence = duel_times_[first][second];
            if (last_evidence && seen[first] && seen[second] && now <= *last_evidence + DuelMemorySeconds) {
                fighting[first] = fighting[second] = true;
            } else {
                last_evidence.reset();
            }
        }
    }

    const auto& weapon = gun(me);
    const auto& weapon_stats = field(weapon, "stats");
    const double range = number(weapon_stats, "range", 230.0);
    const double radius = number(stats, "radius", mech_radius_);
    const double height = number(stats, "height", mech_height_);
    std::optional<Threat> preferred;
    double nearest = std::numeric_limits<double>::infinity();
    for (int id = 0; id < static_cast<int>(seen.size()); ++id) {
        if (weapon.is_null() || !fighting[id] || !seen[id]->enemy_aim) {
            continue;
        }
        auto target = *seen[id];
        if (enemy_aims_at(target, own_position, radius, height, arena)) {
            continue;
        }
        target.attack_position = target.position;
        target.attack_id = id;
        target.attack_velocity = point(opponents[id]->at("vel"));
        const auto shot = shot_points(me, target, weapon_stats);
        const double distance = distance_3d(shot.origin, shot.intercept);
        if (distance < nearest && distance <= range && !arena.blocksLine(shot.origin, shot.target) &&
            !arena.blocksLine(shot.origin, shot.intercept)) {
            preferred = std::move(target);
            nearest = distance;
        }
    }
    return preferred;
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

namespace {

bool gun_ready(const Json& weapon) {
    return !weapon.is_null() && number(weapon, "cooldown", 0.0) <= 0.1 &&
           number(weapon, "reload", 0.0) <= 0 && number(weapon, "ammo", 1.0) > 0;
}

bool aim_aligned(const Vec2& desired, const Vec2& actual, double distance, double radius) {
    const auto [yaw, pitch] = desired;
    const auto [actual_yaw, actual_pitch] = actual;
    const double yaw_error = std::remainder(yaw - actual_yaw, Tau) * std::cos(pitch);
    const double pitch_error = pitch - actual_pitch;
    const double tolerance = std::min(0.18, std::atan2(0.75 * radius, std::max(distance, 1.0)));
    return yaw_error * yaw_error + pitch_error * pitch_error <= tolerance * tolerance;
}

bool clear_target_line(const Shot& shot, const Arena& arena) {
    return !arena.blocksLine(shot.origin, shot.target) && !arena.blocksLine(shot.origin, shot.intercept);
}

bool clear_aim_line(const Shot& shot, const Vec2& actual, double distance, const Arena& arena) {
    const auto [actual_yaw, actual_pitch] = actual;
    const double horizontal = distance * std::cos(actual_pitch);
    const Vec3 actual_end{shot.origin[0] + horizontal * std::cos(actual_yaw),
                          shot.origin[1] + horizontal * std::sin(actual_yaw),
                          shot.origin[2] + distance * std::sin(actual_pitch)};
    return !arena.blocksLine(shot.origin, actual_end);
}

bool ready_aligned_shot(const Json& me, const Threat& target, const Arena& arena) {
    const auto& weapon = gun(me);
    if (!gun_ready(weapon)) {
        return false;
    }
    const auto& weapon_stats = field(weapon, "stats");
    const Shot shot = shot_points(me, target, weapon_stats);
    const double distance = distance_3d(shot.origin, shot.intercept);
    const auto actual = pair(me.at("aim"));
    return distance <= number(weapon_stats, "range", 230.0) &&
           aim_aligned(angles(shot.origin, shot.intercept), actual, distance, target.attack_radius) &&
           clear_target_line(shot, arena) && clear_aim_line(shot, actual, distance, arena);
}

}  // namespace

std::optional<GunShot> ThreatTracker::visible_shot(const Json& obs, const Arena& arena) const {
    const auto& me = obs.at("self");
    const auto& weapon = gun(me);
    if (weapon.is_null()) {
        return std::nullopt;
    }
    const auto& stats = field(weapon, "stats");
    const double range = number(stats, "range", 230.0);
    const bool ready = gun_ready(weapon);
    const auto actual = pair(me.at("aim"));
    const auto self_id = actor_id(me, "id");
    double best_distance = std::numeric_limits<double>::infinity();
    std::optional<GunShot> best;
    for (const auto& actor : field(obs, "mechs")) {
        const auto id = actor_id(actor, "id");
        if (!id || id == self_id || text(field(actor, "kind")) != "mech") {
            continue;
        }
        auto target = describe_enemy(actor, *id);
        target.attack_position = target.position;
        target.attack_velocity = point(actor.at("vel"));
        const Shot shot = shot_points(me, target, stats);
        const double distance = distance_3d(shot.origin, shot.intercept);
        if (distance > range || !clear_target_line(shot, arena)) {
            continue;
        }
        const auto aim = angles(shot.origin, shot.intercept);
        const bool fire = ready && aim_aligned(aim, actual, distance, target.attack_radius) &&
                          clear_aim_line(shot, actual, distance, arena);
        // Prefer a shot that can leave now, then the nearest clear target to aim at.
        if (!best || (fire && !best->fire) || (fire == best->fire && distance < best_distance)) {
            best = GunShot{aim, fire};
            best_distance = distance;
        }
    }
    return best;
}

bool return_fire(const Json& obs, const std::optional<Threat>& threat, const Vec2& move, const Arena& arena,
                 bool returning_to_cover) {
    if (!threat || !threat->facing_us || !threat->attack_id || !threat->attack_position) {
        return false;
    }
    const auto& me = obs.at("self");
    const auto position = point(me.at("pos"));
    const double dx = (*threat->attack_position)[0] - position[0];
    const double dy = (*threat->attack_position)[1] - position[1];
    // The caller may validate a short return route to cover that points towards the attacker.
    if (std::hypot(move[0], move[1]) < 0.15 ||
        (!returning_to_cover && dx * move[0] + dy * move[1] > 0.05 * std::hypot(dx, dy))) {
        return false;
    }
    return ready_aligned_shot(me, *threat, arena);
}

bool opportunity_fire(const Json& obs, const std::optional<Threat>& target, const Arena& arena) {
    return target && !target->hostile && !target->facing_us && target->attack_id && target->attack_position &&
           ready_aligned_shot(obs.at("self"), *target, arena);
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
