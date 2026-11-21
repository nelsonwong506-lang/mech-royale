#include "tactics.hpp"

#include "arena.hpp"

#include <algorithm>
#include <cmath>
#include <string_view>
#include <tuple>

namespace mr {
namespace {

constexpr double RetaliationSeconds = 3.0;
constexpr double FreshHostileDamageSeconds = 0.35;
constexpr double ProjectileLookahead = 0.45;
constexpr double CubeDetourMeters = 24.0;
constexpr double DuelEvidenceSeconds = 0.25;
constexpr double DuelMemorySeconds = 1.2;
constexpr double SoundMemorySeconds = 1.8;
constexpr double Tau = 6.283185307179586;
constexpr std::array<std::string_view, 4> WeaponKinds{"gun", "mortar", "laser", "lightning"};
constexpr std::size_t MortarIndex = 1;
constexpr std::size_t LaserIndex = 2;
constexpr std::size_t LightningIndex = 3;

const Json& field(const Json& value, const char* key) {
    static const Json missing;
    if (!value.is_object()) {
        return missing;
    }
    const auto item = value.find(key);
    return item == value.end() ? missing : *item;
}

double number(const Json& value, const char* key, double fallback) {
    const auto& item = field(value, key);
    return item.is_number() ? finiteOr(item.get<double>(), fallback) : fallback;
}

std::string_view text(const Json& value) {
    const auto* string = value.get_ptr<const Json::string_t*>();
    return string ? std::string_view(*string) : std::string_view{};
}

std::optional<Vec3> try_point(const Json& value) {
    if (!value.is_array() || value.size() < 3
        || !value[0].is_number() || !value[1].is_number() || !value[2].is_number()) {
        return std::nullopt;
    }
    const Vec3 p{value[0].get<double>(), value[1].get<double>(), value[2].get<double>()};
    if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {
        return std::nullopt;
    }
    return p;
}

std::optional<Vec2> try_pair(const Json& value) {
    if (!value.is_array() || value.size() < 2
        || !value[0].is_number() || !value[1].is_number()) {
        return std::nullopt;
    }
    const Vec2 p{value[0].get<double>(), value[1].get<double>()};
    if (!std::isfinite(p[0]) || !std::isfinite(p[1])) {
        return std::nullopt;
    }
    return p;
}

Vec3 point_or(const Json& object, const char* key, Vec3 fallback = {}) {
    const auto p = try_point(field(object, key));
    return p.value_or(fallback);
}

Vec2 pair_or(const Json& object, const char* key, Vec2 fallback = {}) {
    const auto p = try_pair(field(object, key));
    return p.value_or(fallback);
}

Vec3 above(Vec3 position, double height) {
    position[2] += height;
    return position;
}

double distance_3d(const Vec3& first, const Vec3& second) {
    return std::hypot(first[0] - second[0], first[1] - second[1], first[2] - second[2]);
}

Vec2 angles(const Vec3& origin, const Vec3& target) {
    const double horiz = distanceXY(origin, target);
    const double yaw = horiz > 1e-9 ? std::atan2(target[1] - origin[1], target[0] - origin[0]) : 0.0;
    const double pitch = std::clamp(std::atan2(target[2] - origin[2], std::max(horiz, 1e-6)), -1.50, 1.50);
    return {finiteOr(yaw), finiteOr(pitch)};
}

double match_time(const Json& obs) {
    return number(obs, "t", number(obs, "tick", 0.0) / 20.0);
}

std::optional<int> actor_id(const Json& object, const char* key) {
    const double id = number(object, key, -1.0);
    if (id >= 0 && id <= 1000000 && std::trunc(id) == id) {
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
    const auto aim_opt = try_pair(field(opponent, "aim"));
    const auto& weapons = field(opponent, "weapons");
    const std::string_view kind = projectile.mortar ? "mortar" : "gun";
    if (!aim_opt || !weapons.is_array() || std::none_of(weapons.begin(), weapons.end(), [kind](const auto& weapon) {
            return text(weapon) == kind;
        })) {
        return false;
    }
    const auto opp_pos = try_point(field(opponent, "pos"));
    if (!opp_pos) {
        return false;
    }
    const auto [px, py, pz] = projectile.position;
    const auto [vx, vy, vz] = projectile.velocity;
    const auto [ax, ay, az] = *opp_pos;
    const auto [avx, avy, avz] = point_or(opponent, "vel");
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

    const auto [yaw, pitch] = *aim_opt;
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
    const auto& weapons = field(me, "weapons");
    if (!weapons.is_array()) {
        return unarmed;
    }
    for (const auto& weapon : weapons) {
        if (text(field(weapon, "kind")) == "gun") {
            return weapon;
        }
    }
    return unarmed;
}

Vec3 muzzle(const Json& me) {
    return above(point_or(me, "pos"), number(field(me, "stats"), "muzzle_height", 7.0));
}

struct Shot {
    Vec3 origin;       // Current muzzle position (for LOS & clear_aim_line).
    Vec3 lead_origin;  // +50 ms muzzle position (for own-velocity lead angle).
    Vec3 target;
    Vec3 intercept;
};

Shot shot_points(const Json& me, const Threat& threat, const Json& weapon_stats,
                 const Arena& arena, bool hitscan = false) {
    const Vec3 self_vel = point_or(me, "vel");
    const Vec3 base_origin = muzzle(me);
    const Vec3 lead_origin{base_origin[0] + self_vel[0] * 0.05,
                           base_origin[1] + self_vel[1] * 0.05,
                           base_origin[2]};
    const Vec3 target = above(*threat.attack_position, threat.attack_height / 2);
    const Vec3 velocity = threat.attack_velocity.value_or(Vec3{});
    if (hitscan) {
        // Tiny 25 ms motion lead for hitscan beams (laser/lightning) so fast movers stay centered.
        Vec3 hit_pt{target[0] + velocity[0] * 0.025,
                    target[1] + velocity[1] * 0.025,
                    target[2] + velocity[2] * 0.025};
        const double floor_z = arena.ground(hit_pt[0], hit_pt[1]) + threat.attack_height * 0.45;
        hit_pt[2] = std::max(hit_pt[2], floor_z);
        return {base_origin, base_origin, target, hit_pt};
    }
    const double speed = std::max(1.0, number(weapon_stats, "speed", 252.0));
    Vec3 intercept = target;
    for (int step = 0; step < 2; ++step) {
        const double flight_time = distance_3d(lead_origin, intercept) / speed;
        for (int axis = 0; axis < 3; ++axis) {
            intercept[axis] = target[axis] + velocity[axis] * flight_time;
        }
        // Prevent falling/jumping enemies from projecting underground and tripping terrain LOS.
        const double floor_z = arena.ground(intercept[0], intercept[1]) + threat.attack_height * 0.45;
        intercept[2] = std::max(intercept[2], floor_z);
    }
    return {base_origin, lead_origin, target, intercept};
}

bool segment_near_cylinder(const Vec3& start, const Vec3& end, const Vec3& feet,
                           double radius, double height, double slack) {
    const double bottom = feet[2] - slack;
    const double top = feet[2] + height + slack;
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
    const double total_radius = radius + slack;
    return gap_x * gap_x + gap_y * gap_y <= total_radius * total_radius;
}

bool segment_hits_mech(const Vec3& start, const Vec3& end, const Threat& mech) {
    return mech.position && segment_near_cylinder(start, end, *mech.position, mech.attack_radius, mech.attack_height, 0.15);
}

double mortar_range(const Json& mortar) {
    const double speed = number(mortar, "speed", 72.0);
    const double gravity = std::max(1.0, number(mortar, "gravity", 22.0));
    return number(mortar, "range", speed * speed / gravity);
}

// Simulate an actual mortar shell along (yaw, pitch) to verify zero self-splash and accurate landing.
std::optional<Vec3> mortar_impact(const Vec3& origin, double yaw, double pitch,
                                  double speed, double gravity, const Arena& arena) {
    const double cos_p = std::cos(pitch);
    const double sin_p = std::sin(pitch);
    if (cos_p <= 1e-3 || speed <= 1.0 || gravity <= 0.1) {
        return std::nullopt;
    }
    const double ux = std::cos(yaw), uy = std::sin(yaw);
    Vec3 prev = origin;
    for (int step = 1; step <= 45; ++step) {
        const double t = step * 0.12;
        const Vec3 next{origin[0] + speed * cos_p * ux * t,
                        origin[1] + speed * cos_p * uy * t,
                        origin[2] + speed * sin_p * t - 0.5 * gravity * t * t};
        if (arena.blocksPrismLine(prev, next)) {
            return prev;
        }
        const double floor_z = arena.ground(next[0], next[1]);
        if (next[2] <= floor_z) {
            return Vec3{next[0], next[1], floor_z};
        }
        prev = next;
    }
    return std::nullopt;
}

std::optional<Vec2> mortar_solution(const Vec3& origin, const Vec3& raw_target, const Vec3& vel,
                                    double speed, double gravity, const Arena& arena) {
    const double base_horiz = distanceXY(origin, raw_target);
    if (base_horiz < 14.0 || speed <= 1.0 || gravity <= 0.1) {
        return std::nullopt;
    }
    const double est_flight = std::min(2.2, base_horiz / (speed * 0.78));
    const Vec3 target{raw_target[0] + vel[0] * est_flight * 0.55,
                      raw_target[1] + vel[1] * est_flight * 0.55,
                      raw_target[2]};
    const double horiz = distanceXY(origin, target);
    if (horiz < 14.0) {
        return std::nullopt;
    }
    const double dz = target[2] - origin[2];
    const double v2 = speed * speed;
    const double disc = v2 * v2 - gravity * (gravity * horiz * horiz + 2.0 * dz * v2);
    if (disc < 0.0) {
        return std::nullopt;
    }
    const double root = std::sqrt(disc);
    const double yaw = std::atan2(target[1] - origin[1], target[0] - origin[0]);
    const double ux = std::cos(yaw), uy = std::sin(yaw);
    for (const double sign : {-1.0, 1.0}) {
        const double pitch = std::atan((v2 + sign * root) / (gravity * horiz));
        if (pitch < -1.2 || pitch > 1.50) {
            continue;
        }
        const double cos_p = std::cos(pitch);
        const double sin_p = std::sin(pitch);
        if (cos_p <= 1e-3) {
            continue;
        }
        const double flight_time = horiz / (speed * cos_p);
        Vec3 prev = origin;
        bool blocked = false;
        constexpr int samples = 10;
        for (int i = 1; i <= samples; ++i) {
            const double t = flight_time * (0.93 * i / samples);
            const Vec3 next{origin[0] + speed * cos_p * ux * t,
                            origin[1] + speed * cos_p * uy * t,
                            origin[2] + speed * sin_p * t - 0.5 * gravity * t * t};
            if (arena.blocksPrismLine(prev, next) || next[2] <= arena.ground(next[0], next[1])) {
                blocked = true;
                break;
            }
            prev = next;
        }
        if (!blocked) {
            return Vec2{finiteOr(yaw), finiteOr(pitch)};
        }
    }
    return std::nullopt;
}

bool weapon_ready(const Json& weapon) {
    return weapon.is_object() && number(weapon, "cooldown", 0.0) <= 0.1 &&
           number(weapon, "reload", 0.0) <= 0.0 && number(weapon, "ammo", 1.0) > 0.0;
}

bool aim_aligned(const Vec2& desired, const Vec2& actual, double distance, double radius, double extra_slack = 0.025) {
    const auto [yaw, pitch] = desired;
    const auto [actual_yaw, actual_pitch] = actual;
    const double yaw_error = std::remainder(yaw - actual_yaw, Tau) * std::cos(pitch);
    const double pitch_error = pitch - actual_pitch;
    const double tolerance = std::min(0.18, std::atan2(radius, std::max(distance, 1.0)) + extra_slack);
    return yaw_error * yaw_error + pitch_error * pitch_error <= tolerance * tolerance;
}

bool clear_aim_line(const Vec3& origin, const Vec2& actual, double distance, const Arena& arena) {
    const auto [actual_yaw, actual_pitch] = actual;
    const double horizontal = distance * std::cos(actual_pitch);
    const Vec3 actual_end{origin[0] + horizontal * std::cos(actual_yaw),
                          origin[1] + horizontal * std::sin(actual_yaw),
                          origin[2] + distance * std::sin(actual_pitch)};
    return !arena.blocksLine(origin, actual_end);
}

std::optional<Vec3> clear_aim_point(const Shot& shot, const Arena& arena) {
    if (!arena.blocksLine(shot.origin, shot.target)) {
        if (!arena.blocksLine(shot.origin, shot.intercept)) {
            return shot.intercept;
        }
        return shot.target;
    }
    return std::nullopt;
}

bool ready_aligned_shot(const Json& me, const Threat& target, const Arena& arena) {
    const auto& weapon = gun(me);
    if (!weapon_ready(weapon) || !target.attack_position) {
        return false;
    }
    const auto& weapon_stats = field(weapon, "stats");
    const Shot shot = shot_points(me, target, weapon_stats, arena);
    const auto aim_point = clear_aim_point(shot, arena);
    if (!aim_point) {
        return false;
    }
    const double distance = distance_3d(shot.origin, *aim_point);
    const auto actual = pair_or(me, "aim");
    const double slack = distance > 90.0 ? 0.012 : 0.025;
    return distance <= number(weapon_stats, "range", 230.0) &&
           aim_aligned(angles(shot.lead_origin, *aim_point), actual, distance, target.attack_radius, slack) &&
           clear_aim_line(shot.origin, actual, distance, arena);
}

}  // namespace

ThreatTracker::ThreatTracker(const Json& config)
    : mech_radius_(number(field(config, "mech"), "radius", 2.5)),
      mech_height_(number(field(config, "mech"), "height", 9.0)),
      muzzle_height_(number(field(config, "mech"), "muzzle_height", 7.0)),
      turn_rate_(number(field(config, "mech"), "turn_rate", 4.0)),
      pitch_rate_(number(field(config, "mech"), "pitch_rate", 3.0)),
      size_per_cube_(number(field(config, "cubes"), "size_per_cube", 0.05)),
      damage_per_cube_(number(field(config, "cubes"), "damage_per_cube", 0.04)),
      cube_stat_cap_(number(field(config, "cubes"), "stat_cap", 10.0)),
      mortar_gravity_(number(field(field(config, "weapons"), "mortar"), "gravity", 22.0)) {
    const auto& weapons = field(config, "weapons");
    weapon_ranges_ = {number(field(weapons, "gun"), "range", 230.0), mortar_range(field(weapons, "mortar")),
                      number(field(weapons, "laser"), "range", 280.0), number(field(weapons, "lightning"), "range", 45.0)};
}

Threat ThreatTracker::describe_enemy(const Json& actor, int id) const {
    const bool is_npc = text(field(actor, "kind")) == "npc";
    const double scale = is_npc ? 1.0 : 1 + size_per_cube_ * number(actor, "cubes", 0.0);
    Threat threat;
    threat.position = point_or(actor, "pos");
    threat.actor_id = id;
    threat.is_npc = is_npc;
    threat.hp = number(actor, "hp", is_npc ? 350.0 : 1400.0);
    threat.attack_radius = (is_npc ? 1.8 : mech_radius_) * scale;
    threat.attack_height = (is_npc ? 3.5 : mech_height_) * scale;
    if (const auto aim = try_pair(field(actor, "aim"))) {
        threat.enemy_aim = *aim;
    }

    const auto& weapons = field(actor, "weapons");
    threat.enemy_muzzle_height = (is_npc ? 2.5 : muzzle_height_) * scale;
    if (!weapons.is_array()) {
        threat.enemy_range = is_npc ? 60.0 : weapon_ranges_.front();
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
        threat.has_mortar = threat.has_mortar || index == MortarIndex;
        threat.has_laser = threat.has_laser || index == LaserIndex;
        threat.has_lightning = threat.has_lightning || index == LightningIndex;
    }
    if (threat.enemy_range > 0 && !has_straight_weapon) {
        threat.enemy_muzzle_height = (mech_height_ + 0.5) * scale;
        threat.mortar_only = threat.has_mortar;
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
    if (threat.mortar_only) {
        return enemy_pitch > radians(5) && enemy_pitch < radians(82);
    }
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

std::optional<ProjectileDodge> ThreatTracker::projectile_dodge(const Json& obs, const Arena& arena) const {
    const auto& projectiles = field(obs, "projectiles");
    if (!projectiles.is_array() || projectiles.empty()) {
        return std::nullopt;
    }
    const auto& me = field(obs, "self");
    const Vec3 self_pos = point_or(me, "pos");
    const Vec3 self_vel = point_or(me, "vel");
    const auto& stats = field(me, "stats");
    const double self_radius = number(stats, "radius", mech_radius_);
    const double self_height = number(stats, "height", mech_height_);

    // 1. Check incoming mortar shells first (360 dmg + knockback + energy drain!).
    double soonest_mortar = std::numeric_limits<double>::infinity();
    std::optional<Vec2> mortar_escape;
    for (const auto& object : projectiles) {
        if (text(field(object, "kind")) != "mortar") continue;
        const auto pos = try_point(field(object, "pos"));
        const auto vel = try_point(field(object, "vel"));
        if (!pos || !vel) continue;
        Vec3 prev = *pos;
        for (int step = 1; step <= 30; ++step) {
            const double t = step * 0.1;
            const Vec3 next{(*pos)[0] + (*vel)[0] * t,
                            (*pos)[1] + (*vel)[1] * t,
                            (*pos)[2] + (*vel)[2] * t - 0.5 * mortar_gravity_ * t * t};
            const double surf = arena.surface(next[0], next[1], std::max(prev[2], next[2]) + 1.0);
            const bool hit = next[2] <= surf || arena.blocksPrismLine(prev, next);
            if (hit) {
                const double dist = distanceXY(self_pos, next);
                if (dist <= self_radius + 8.8 && t < soonest_mortar) {
                    soonest_mortar = t;
                    const double dx = self_pos[0] - next[0];
                    const double dy = self_pos[1] - next[1];
                    const double norm = std::hypot(dx, dy);
                    if (norm > 0.4) {
                        mortar_escape = Vec2{dx / norm, dy / norm};
                    } else {
                        // Dead center on landing point: sprint perpendicular to shell velocity.
                        const double vnorm = std::max(1.0, std::hypot((*vel)[0], (*vel)[1]));
                        mortar_escape = Vec2{-(*vel)[1] / vnorm, (*vel)[0] / vnorm};
                    }
                }
                break;
            }
            prev = next;
        }
    }
    if (mortar_escape) {
        return ProjectileDodge{*mortar_escape, true};
    }

    // 2. Accumulate perpendicular sidestep away from imminent gun / NPC bullets.
    double dodge_x = 0.0, dodge_y = 0.0;
    for (const auto& object : projectiles) {
        const auto kind = text(field(object, "kind"));
        if (kind != "gun" && kind != "npc") continue;
        const auto pos = try_point(field(object, "pos"));
        const auto vel = try_point(field(object, "vel"));
        if (!pos || !vel) continue;
        const double dx = (*pos)[0] - self_pos[0];
        const double dy = (*pos)[1] - self_pos[1];
        const double rvx = (*vel)[0] - self_vel[0];
        const double rvy = (*vel)[1] - self_vel[1];
        const double speed_sq = rvx * rvx + rvy * rvy;
        if (speed_sq < 4.0) continue;
        const double t = -(dx * rvx + dy * rvy) / speed_sq;
        if (t <= 0.02 || t > 0.45) continue;
        const double miss_x = dx + rvx * t;
        const double miss_y = dy + rvy * t;
        const double miss_dist = std::hypot(miss_x, miss_y);
        if (miss_dist > self_radius + 1.3) continue;
        const double bullet_z = (*pos)[2] + (*vel)[2] * t;
        if (bullet_z < self_pos[2] - 0.6 || bullet_z > self_pos[2] + self_height + 0.6) continue;
        const Vec3 impact{(*pos)[0] + (*vel)[0] * t, (*pos)[1] + (*vel)[1] * t, bullet_z};
        if (arena.blocksLine(*pos, impact)) continue;
        // Sidestep away from the bullet path; bias toward the side we're already slightly on.
        const double vnorm = std::sqrt(speed_sq);
        const double perp_x = -rvy / vnorm;
        const double perp_y = rvx / vnorm;
        const double side = (-miss_x * perp_x - miss_y * perp_y) >= 0.0 ? 1.0 : -1.0;
        const double weight = 1.0 / std::max(0.08, t);
        dodge_x += side * perp_x * weight;
        dodge_y += side * perp_y * weight;
    }
    const double dodge_norm = std::hypot(dodge_x, dodge_y);
    if (dodge_norm > 1e-3) {
        return ProjectileDodge{{dodge_x / dodge_norm, dodge_y / dodge_norm}, false};
    }
    return std::nullopt;
}

bool ThreatTracker::can_collect_cubes(const Json& obs) const {
    return match_time(obs) - last_damage_at_ >= 0.8;
}

double ThreatTracker::last_combat_time() const {
    return std::max(last_damage_at_, last_hit_dealt_at_);
}

double ThreatTracker::last_damage_time() const {
    return last_damage_at_;
}

std::optional<double> ThreatTracker::sound_heading(double now) const {
    if (sound_heading_ && now <= sound_until_) {
        return sound_heading_;
    }
    return std::nullopt;
}

std::vector<EnemyInfo> ThreatTracker::visible_enemies(const Json& obs, const Arena& arena) const {
    std::vector<EnemyInfo> list;
    const auto& me = field(obs, "self");
    const auto self_id = actor_id(me, "id");
    const Vec3 self_pos = point_or(me, "pos");
    const auto& self_stats = field(me, "stats");
    const double self_radius = number(self_stats, "radius", mech_radius_);
    const double self_height = number(self_stats, "height", mech_height_);
    const auto& mechs = field(obs, "mechs");
    if (!mechs.is_array()) {
        return list;
    }
    for (const auto& actor : mechs) {
        const auto id = actor_id(actor, "id");
        if (!id || id == self_id || !try_point(field(actor, "pos"))) {
            continue;
        }
        const auto kind = text(field(actor, "kind"));
        if (kind != "mech") {
            continue;
        }
        Threat desc = describe_enemy(actor, *id);
        EnemyInfo info;
        info.id = *id;
        info.pos = *desc.position;
        info.vel = point_or(actor, "vel");
        info.hp = desc.hp;
        info.radius = desc.attack_radius;
        info.height = desc.attack_height;
        info.is_npc = false;
        info.has_lightning = desc.has_lightning;
        info.has_laser = desc.has_laser;
        info.has_mortar = desc.has_mortar;
        info.aim = desc.enemy_aim;
        const auto& charging = field(actor, "charging");
        info.charging = charging.is_boolean() && charging.get<bool>();
        info.facing_us = distanceXY(self_pos, info.pos) <= desc.enemy_range + self_radius &&
            enemy_aims_at(desc, self_pos, self_radius, self_height, arena);
        list.push_back(info);
    }
    return list;
}

std::optional<Threat> ThreatTracker::update(const Json& obs, const Arena& arena, Clock::time_point deadline) {
    const auto& me = field(obs, "self");
    const auto self_id = actor_id(me, "id");
    const Vec3 position = point_or(me, "pos");
    const auto& self_stats = field(me, "stats");
    const double now = match_time(obs);
    const double radius = number(self_stats, "radius", mech_radius_);
    const double height = number(self_stats, "height", mech_height_);

    std::unordered_map<int, const Json*> enemies;
    std::unordered_map<int, const Json*> npcs;
    const auto& mechs = field(obs, "mechs");
    if (mechs.is_array()) {
        for (const auto& actor : mechs) {
            const auto id = actor_id(actor, "id");
            if (!id || id == self_id || !try_point(field(actor, "pos"))) {
                continue;
            }
            const auto kind = text(field(actor, "kind"));
            if (kind == "mech") {
                enemies[*id] = &actor;
            } else if (kind == "npc") {
                npcs[*id] = &actor;
            }
        }
    }

    for (auto it = attacks_.begin(); it != attacks_.end();) {
        if (now - it->second.time >= RetaliationSeconds) {
            it = attacks_.erase(it);
        } else {
            ++it;
        }
    }

    const auto& events = field(obs, "events");
    if (events.is_array()) {
        for (const auto& event : events) {
            const auto etype = text(field(event, "type"));
            if (etype == "hit") {
                last_hit_dealt_at_ = now;
                continue;
            }
            if (etype != "damaged") {
                continue;
            }
            last_damage_at_ = now;
            const auto weapon = text(field(event, "weapon"));
            if (weapon == "zone") {
                continue;
            }
            const auto id = actor_id(event, "by");
            if (id && id == self_id) {
                continue;
            }
            last_hostile_damage_at_ = now;
            const auto& bearing = field(event, "bearing");
            const bool is_npc = weapon == "npc" || (id && npcs.count(*id) > 0);
            if (id) {
                auto& mem = attacks_[*id];
                mem.time = now;
                mem.is_npc = is_npc;
                if (const auto it = enemies.find(*id); it != enemies.end()) {
                    mem.position = point_or(*it->second, "pos");
                } else if (const auto it = npcs.find(*id); it != npcs.end()) {
                    mem.position = point_or(*it->second, "pos");
                } else if (bearing.is_number()) {
                    const double angle = finiteOr(bearing.get<double>());
                    mem.position = Vec3{position[0] + 40 * std::cos(angle),
                                        position[1] + 40 * std::sin(angle), position[2]};
                }
            } else if (bearing.is_number()) {
                const double angle = finiteOr(bearing.get<double>());
                unknown_position_ = Vec3{position[0] + 40 * std::cos(angle),
                                         position[1] + 40 * std::sin(angle), position[2]};
                unknown_until_ = now + 2.0;
            }
        }
    }

    const auto& beams = field(obs, "beams");
    if (beams.is_array()) {
        for (const auto& beam : beams) {
            if (Clock::now() >= deadline) {
                break;
            }
            const auto shooter = actor_id(beam, "by");
            if (shooter && shooter == self_id) {
                continue;
            }
            const auto& path = field(beam, "path");
            if (!path.is_array() || path.size() < 2) {
                continue;
            }
            bool near_us = false;
            for (std::size_t i = 1; i < path.size(); ++i) {
                const auto a = try_point(path[i - 1]);
                const auto b = try_point(path[i]);
                if (a && b && segment_near_cylinder(*a, *b, position, radius, height, 4.0)) {
                    near_us = true;
                    break;
                }
            }
            if (near_us) {
                if (shooter) {
                    auto& mem = attacks_[*shooter];
                    mem.time = now;
                    mem.is_npc = npcs.count(*shooter) > 0;
                    if (const auto a = try_point(path[0])) {
                        mem.position = *a;
                    }
                } else if (const auto a = try_point(path[0])) {
                    unknown_position_ = *a;
                    unknown_until_ = now + 1.5;
                }
            }
        }
    }

    const auto& sounds = field(obs, "sounds");
    if (sounds.is_array()) {
        int best_priority = -1;
        for (const auto& sound : sounds) {
            const auto skind = text(field(sound, "kind"));
            if (skind == "death" || skind == "drop") continue;
            const auto dist_str = text(field(sound, "dist"));
            int prio = (dist_str == "near" ? 3 : dist_str == "mid" ? 1 : 0);
            if (skind == "laser_charge" || skind == "lightning") prio += 3;
            else if (skind == "gun" || skind == "laser" || skind == "mortar") prio += 1;
            if (prio <= best_priority) continue;
            if (const auto p = try_point(field(sound, "pos"))) {
                if (distanceXY(position, *p) > 3.0) {
                    best_priority = prio;
                    sound_heading_ = std::atan2((*p)[1] - position[1], (*p)[0] - position[0]);
                    sound_until_ = now + SoundMemorySeconds;
                }
            } else if (const auto& b = field(sound, "bearing"); b.is_number()) {
                best_priority = prio;
                sound_heading_ = finiteOr(b.get<double>());
                sound_until_ = now + SoundMemorySeconds;
            }
        }
    }

    const auto& projectiles = field(obs, "projectiles");
    if (projectiles.is_array() && !projectiles.empty()) {
        const Vec3 velocity = point_or(me, "vel");
        const double hit_radius = radius + 0.5;
        double soonest = std::numeric_limits<double>::infinity();
        std::optional<Projectile> imminent;
        for (const auto& object : projectiles) {
            if (Clock::now() >= deadline) {
                break;
            }
            const auto kind = text(field(object, "kind"));
            if (kind != "gun" && kind != "mortar") {
                continue;
            }
            const auto pos = try_point(field(object, "pos"));
            const auto vel = try_point(field(object, "vel"));
            if (!pos || !vel) {
                continue;
            }
            const Projectile projectile{*pos, *vel, kind == "mortar", kind == "mortar" ? mortar_gravity_ : 0.0};
            const auto time = collision_time(position, velocity, hit_radius, height, projectile, soonest, arena);
            if (time) {
                soonest = *time;
                imminent = projectile;
            }
        }
        if (imminent) {
            std::optional<int> shooter;
            for (const auto& [id, actor] : enemies) {
                if (Clock::now() >= deadline) {
                    break;
                }
                if (!could_have_fired(*actor, *imminent, size_per_cube_, mech_radius_,
                                      imminent->mortar ? mech_height_ + 0.5 : muzzle_height_,
                                      turn_rate_, pitch_rate_, arena)) {
                    continue;
                }
                if (shooter) {
                    shooter.reset();
                    break;
                }
                shooter = id;
            }
            if (shooter) {
                attacks_[*shooter].time = now;
                attacks_[*shooter].is_npc = false;
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

    int attacker_id = -1;
    double attacker_dist = std::numeric_limits<double>::infinity();
    bool attacker_visible = false;
    double attacker_time = -std::numeric_limits<double>::infinity();
    for (auto& [id, attack] : attacks_) {
        if (now - attack.time >= RetaliationSeconds) {
            continue;
        }
        const Json* actor = nullptr;
        if (const auto it = enemies.find(id); it != enemies.end()) {
            actor = it->second;
            attack.is_npc = false;
        } else if (const auto it = npcs.find(id); it != npcs.end()) {
            actor = it->second;
            attack.is_npc = true;
        }
        if (actor) {
            attack.position = point_or(*actor, "pos");
        }
        const bool visible = actor != nullptr;
        const double dist = attack.position ? distanceXY(position, *attack.position) : 1000.0;
        if (attacker_id < 0 || std::tuple{!visible, dist > 90.0, -attack.time, dist} <
                                   std::tuple{!attacker_visible, attacker_dist > 90.0, -attacker_time, attacker_dist}) {
            attacker_id = id;
            attacker_visible = visible;
            attacker_dist = dist;
            attacker_time = attack.time;
        }
    }

    std::optional<Threat> nearest;
    std::optional<Threat> facing;
    double nearest_distance = std::numeric_limits<double>::infinity();
    double facing_distance = std::numeric_limits<double>::infinity();
    const bool damaged_recently = now <= last_hostile_damage_at_ + FreshHostileDamageSeconds;
    for (const auto& [id, actor] : enemies) {
        if (Clock::now() >= deadline) {
            break;
        }
        Threat observed = describe_enemy(*actor, id);
        const double distance = distanceXY(position, *observed.position);
        if (distance < nearest_distance) {
            nearest = observed;
            nearest_distance = distance;
        }
        const auto& charging = field(*actor, "charging");
        if (charging.is_boolean() && charging.get<bool>() &&
            (!facing || !facing->laser_charging || distance < facing_distance)) {
            auto laser = observed;
            laser.enemy_range = weapon_ranges_[LaserIndex];
            laser.enemy_muzzle_height = muzzle_height_ * (1 + size_per_cube_ * number(*actor, "cubes", 0.0));
            const Vec3 src = above(*laser.position, laser.enemy_muzzle_height);
            const Vec3 torso = above(position, height * 0.5);
            const double yaw_to_us = std::atan2(position[1] - src[1], position[0] - src[0]);
            const bool sweeping_toward_us = laser.enemy_aim && distance <= laser.enemy_range + radius &&
                std::abs(std::remainder(yaw_to_us - (*laser.enemy_aim)[0], Tau)) <= radians(25) &&
                !arena.blocksLine(src, torso);
            if (sweeping_toward_us || enemy_aims_at(laser, position, radius, height, arena)) {
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
            observed.attack_velocity = point_or(*actor, "vel");
            facing = observed;
            facing_distance = distance;
        }
    }

    auto build_attacker = [&]() -> Threat {
        const Json* actor = nullptr;
        if (const auto it = enemies.find(attacker_id); it != enemies.end()) {
            actor = it->second;
        } else if (const auto it = npcs.find(attacker_id); it != npcs.end()) {
            actor = it->second;
        }
        Threat threat = actor ? describe_enemy(*actor, attacker_id) : Threat{};
        threat.position = attacks_[attacker_id].position;
        threat.actor_id = attacker_id;
        threat.is_npc = attacks_[attacker_id].is_npc;
        threat.hostile = true;
        threat.damaged_recently = damaged_recently;
        threat.attack_id = attacker_id;
        if (actor) {
            threat.attack_position = threat.position;
            threat.attack_velocity = point_or(*actor, "vel");
        }
        return threat;
    };

    if (facing && facing->laser_charging) {
        return facing;
    }
    if (attacker_id >= 0 && (!facing || (attacker_visible && (attacker_dist < facing_distance || attacker_dist <= 80.0)))) {
        return build_attacker();
    }
    if (facing) {
        return facing;
    }
    if (attacker_id >= 0) {
        return build_attacker();
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

std::optional<Threat> ThreatTracker::duel_target(const Json& obs, const Arena& arena, Clock::time_point deadline) {
    if (Clock::now() >= deadline) {
        return std::nullopt;
    }
    const auto& me = field(obs, "self");
    const auto own_id = actor_id(me, "id");
    const auto& actors = field(obs, "mechs");
    const auto& beams = field(obs, "beams");
    const double now = match_time(obs);
    const auto& zone = field(obs, "zone");
    const Vec2 circle_center = pair_or(zone, "center");
    const double circle_radius = number(zone, "radius", -1.0);
    std::array<const Json*, 16> opponents{};
    std::array<bool, 16> hurt_by_us{};
    std::array<bool, 16> lost_health{};
    const auto& events = field(obs, "events");
    if (events.is_array()) {
        for (const auto& event : events) {
            if (text(field(event, "type")) == "hit") {
                if (const auto victim = actor_id(event, "target"); victim && *victim < 16) {
                    hurt_by_us[*victim] = true;
                }
            }
        }
    }
    std::size_t opponent_count = 0;
    if (actors.is_array()) {
        for (const auto& actor : actors) {
            const auto id = actor_id(actor, "id");
            if (!id || *id >= 16 || id == own_id || text(field(actor, "kind")) != "mech" || opponents[*id]) {
                continue;
            }
            const auto pos = try_point(field(actor, "pos"));
            if (!pos) {
                continue;
            }
            opponents[*id] = &actor;
            ++opponent_count;
            const double hp = number(actor, "hp", -1.0);
            const bool in_zone = distanceXY(*pos, circle_center) <= circle_radius;
            const auto previous = health_samples_[*id];
            if (!hurt_by_us[*id] && in_zone && previous.in_zone && now > previous.time &&
                now <= previous.time + DuelEvidenceSeconds && hp >= 0 && previous.hp - hp > 0.5) {
                lost_health[*id] = true;
            }
            health_samples_[*id] = {now, hp, in_zone};
        }
    }
    if (opponent_count < 2) {
        if (now <= last_duel_at_ + DuelMemorySeconds) {
            duel_times_ = {};
            last_duel_at_ = -std::numeric_limits<double>::infinity();
        }
        return std::nullopt;
    }
    if (now > last_duel_at_ + DuelMemorySeconds && (!beams.is_array() || beams.empty()) &&
        std::none_of(lost_health.begin(), lost_health.end(), [](bool lost) {
            return lost;
        })) {
        return std::nullopt;
    }

    const auto& stats = field(me, "stats");
    const Vec3 own_position = point_or(me, "pos");
    const Vec3 eye = above(own_position, number(stats, "eye_height", 8.0));
    std::array<std::optional<Threat>, 16> seen;
    for (int id = 0; id < static_cast<int>(opponents.size()); ++id) {
        if (!opponents[id] || Clock::now() >= deadline) {
            continue;
        }
        auto actor = describe_enemy(*opponents[id], id);
        const auto body = above(*actor.position, actor.attack_height / 2);
        if (!arena.blocksLine(eye, body)) {
            seen[id] = std::move(actor);
        }
    }

    const auto remember_duel = [&](int first, int second) {
        duel_times_[std::min(first, second)][std::max(first, second)] = now;
        last_duel_at_ = now;
    };
    if (beams.is_array()) {
        for (const auto& beam : beams) {
            if (Clock::now() >= deadline) {
                break;
            }
            const auto kind = text(field(beam, "kind"));
            const auto shooter = actor_id(beam, "by");
            const auto& path = field(beam, "path");
            if ((kind != "laser" && kind != "lightning") || !shooter || *shooter >= 16 || !seen[*shooter] ||
                number(beam, "age", DuelEvidenceSeconds + 1) > DuelEvidenceSeconds || !path.is_array() || path.size() < 2) {
                continue;
            }
            const auto& source = *seen[*shooter];
            for (int id = 0; id < static_cast<int>(seen.size()) && Clock::now() < deadline; ++id) {
                if (id == *shooter || !seen[id]) {
                    continue;
                }
                const auto& target = *seen[id];
                for (std::size_t index = 1; index < path.size(); ++index) {
                    const auto p0 = try_point(path[index - 1]);
                    const auto p1 = try_point(path[index]);
                    if (p0 && p1 && segment_hits_mech(*p0, *p1, target) &&
                        enemy_aims_at(target, *source.position, source.attack_radius, source.attack_height, arena)) {
                        remember_duel(id, *shooter);
                        break;
                    }
                }
            }
        }
    }
    for (int id = 0; id < static_cast<int>(seen.size()) && Clock::now() < deadline; ++id) {
        if (!seen[id] || !lost_health[id]) {
            continue;
        }
        const auto& victim = *seen[id];
        for (int other = 0; other < static_cast<int>(seen.size()) && Clock::now() < deadline; ++other) {
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
    double best_score = -std::numeric_limits<double>::infinity();
    for (int id = 0; id < static_cast<int>(seen.size()) && Clock::now() < deadline; ++id) {
        if (weapon.is_null() || !fighting[id] || !seen[id]->enemy_aim) {
            continue;
        }
        auto target = *seen[id];
        if (enemy_aims_at(target, own_position, radius, height, arena)) {
            continue;
        }
        target.attack_position = target.position;
        target.attack_id = id;
        target.attack_velocity = point_or(*opponents[id], "vel");
        const auto shot = shot_points(me, target, weapon_stats, arena);
        const auto aim_point = clear_aim_point(shot, arena);
        if (!aim_point) {
            continue;
        }
        const double distance = distance_3d(shot.origin, *aim_point);
        const double score = (1400.0 - std::min(1400.0, target.hp)) * 0.2 - distance;
        if (distance <= range && score > best_score) {
            preferred = std::move(target);
            best_score = score;
        }
    }
    return preferred;
}

std::optional<Vec2> retreat_aim(const Json& obs, const std::optional<Threat>& threat) {
    if (!threat || !threat->position) {
        return std::nullopt;
    }
    const auto& me = field(obs, "self");
    const Vec3 origin = muzzle(me);
    if (threat->attack_position) {
        const Vec3 target = above(*threat->attack_position, threat->attack_height * 0.5);
        const double speed = std::max(1.0, number(field(gun(me), "stats"), "speed", 252.0));
        const Vec3 vel = threat->attack_velocity.value_or(Vec3{});
        const double ft = distance_3d(origin, target) / speed;
        const Vec3 intercept{target[0] + vel[0] * ft, target[1] + vel[1] * ft, target[2] + vel[2] * ft};
        return angles(origin, intercept);
    }
    return angles(origin, above(*threat->position, threat->attack_height * 0.5));
}

std::optional<GunShot> ThreatTracker::visible_shot(const Json& obs, const Arena& arena,
                                                   const std::optional<Threat>& primary_threat,
                                                   Clock::time_point deadline,
                                                   bool allow_crates,
                                                   bool allow_npcs,
                                                   bool allow_laser_charge) const {
    const auto& me = field(obs, "self");
    const auto& weapons = field(me, "weapons");
    if (!weapons.is_array() || weapons.empty()) {
        return std::nullopt;
    }

    struct SlotInfo {
        int index = -1;
        std::string_view kind;
        const Json* weapon = nullptr;
        const Json* stats = nullptr;
        double range = 0.0;
        bool ready = false;
        double charge = 0.0;
    };
    std::array<SlotInfo, 2> slots{};
    std::size_t slot_count = 0;
    for (std::size_t i = 0; i < std::min<std::size_t>(2, weapons.size()); ++i) {
        if (!weapons[i].is_object()) {
            continue;
        }
        const auto kind = text(field(weapons[i], "kind"));
        const auto& wstats = field(weapons[i], "stats");
        double default_range = 230.0;
        if (kind == "mortar") default_range = mortar_range(wstats);
        else if (kind == "laser") default_range = weapon_ranges_[LaserIndex];
        else if (kind == "lightning") default_range = weapon_ranges_[LightningIndex];
        slots[slot_count++] = SlotInfo{
            static_cast<int>(i), kind, &weapons[i], &wstats,
            number(wstats, "range", default_range),
            weapon_ready(weapons[i]),
            number(weapons[i], "charge", 0.0)
        };
    }
    if (slot_count == 0) {
        return std::nullopt;
    }

    const auto actual = pair_or(me, "aim");
    const auto self_id = actor_id(me, "id");
    const Vec3 self_pos = point_or(me, "pos");
    const double self_height = number(field(me, "stats"), "height", mech_height_);
    const double my_energy = number(me, "energy", 0.0);
    const double my_cubes = number(me, "cubes", 0.0);
    const double cube_mult = 1.0 + damage_per_cube_ * std::min(my_cubes, cube_stat_cap_);
    const Vec3 mortar_origin = above(self_pos, self_height + 0.5);

    // Pre-collect visible enemy mech positions for lightning chain & mortar splash cluster scoring.
    std::vector<Vec3> mech_positions;
    const auto& mechs = field(obs, "mechs");
    if (mechs.is_array()) {
        mech_positions.reserve(mechs.size());
        for (const auto& actor : mechs) {
            const auto id = actor_id(actor, "id");
            if (!id || id == self_id || text(field(actor, "kind")) != "mech") continue;
            if (const auto p = try_point(field(actor, "pos"))) {
                mech_positions.push_back(*p);
            }
        }
    }

    auto count_neighbors = [&](const Vec3& center, double radius) {
        int count = 0;
        for (const auto& p : mech_positions) {
            const double d = distanceXY(center, p);
            if (d > 0.5 && d <= radius) {
                ++count;
            }
        }
        return count;
    };

    double best_score = -std::numeric_limits<double>::infinity();
    std::optional<GunShot> best;

    if (mechs.is_array()) {
        for (const auto& actor : mechs) {
            if (Clock::now() >= deadline) {
                break;
            }
            const auto id = actor_id(actor, "id");
            if (!id || id == self_id || !try_point(field(actor, "pos"))) {
                continue;
            }
            const auto kind = text(field(actor, "kind"));
            const bool is_npc = kind == "npc";
            const bool attacking_npc = is_npc && primary_threat && primary_threat->is_npc
                && primary_threat->actor_id && *primary_threat->actor_id == *id;
            const bool farm_npc = is_npc && allow_npcs && distanceXY(self_pos, point_or(actor, "pos")) <= 65.0;
            if (kind != "mech" && !attacking_npc && !farm_npc) {
                continue;
            }

            auto target = describe_enemy(actor, *id);
            target.attack_position = target.position;
            target.attack_velocity = point_or(actor, "vel");

            const bool is_primary = primary_threat && primary_threat->actor_id && *primary_threat->actor_id == *id;
            const double hp = std::clamp(target.hp, 1.0, 2200.0);
            const double hp_bonus = (is_npc && !attacking_npc)
                ? -450.0  // Only farm passive NPCs when no real enemy mech can be shot.
                : ((hp <= 350.0 ? 240.0 : 0.0) + std::max(0.0, 1400.0 - hp) * 0.15);
            const double threat_bonus = is_primary && primary_threat->hostile
                ? (primary_threat->laser_charging ? 200.0 : 95.0) : 0.0;

            std::optional<Vec2> gun_aim;
            bool gun_can_fire = false;
            bool gun_is_ready = false;
            int gun_slot = -1;
            double gun_dist = std::numeric_limits<double>::infinity();

            std::optional<Vec2> light_aim;
            bool light_can_fire = false;
            int light_slot = -1;
            double light_dist = std::numeric_limits<double>::infinity();
            double light_bonus = 0.0;

            std::optional<Vec2> laser_aim;
            bool laser_trigger = false;
            bool laser_releasing = false;
            int laser_slot = -1;
            double laser_dist = std::numeric_limits<double>::infinity();
            double laser_bonus = 0.0;

            std::optional<Vec2> mortar_aim;
            bool mortar_can_fire = false;
            int mortar_slot = -1;
            double mortar_dist = std::numeric_limits<double>::infinity();
            double mortar_bonus = 0.0;

            const double self_radius = number(field(me, "stats"), "radius", mech_radius_);
            bool enemy_holds_lightning_close = target.has_lightning && distanceXY(self_pos, *target.position) <= 50.0;
            for (std::size_t s = 0; s < slot_count; ++s) {
                const auto& slot = slots[s];
                if (slot.kind == "gun") {
                    gun_is_ready = slot.ready;
                    const Shot shot = shot_points(me, target, *slot.stats, arena, false);
                    const auto aim_pt = clear_aim_point(shot, arena);
                    if (!aim_pt) continue;
                    const double dist = distance_3d(shot.origin, *aim_pt);
                    if (dist > slot.range) continue;
                    const Vec2 aim = angles(shot.lead_origin, *aim_pt);
                    const double slack = dist > 90.0 ? 0.012 : 0.025;
                    const bool fire = slot.ready && aim_aligned(aim, actual, dist, target.attack_radius, slack) &&
                                      clear_aim_line(shot.origin, actual, dist, arena);
                    gun_aim = aim;
                    gun_can_fire = fire;
                    gun_slot = slot.index;
                    gun_dist = dist;
                } else if (slot.kind == "lightning") {
                    const double e_cost = number(*slot.stats, "energy", 3.0);
                    const double cone_deg = number(*slot.stats, "cone_deg", 30.0);
                    const double chain_r = number(*slot.stats, "chain_range", 14.0);
                    const Shot shot = shot_points(me, target, *slot.stats, arena, true);
                    if (arena.blocksLine(shot.origin, shot.target)) continue;
                    const double dist = distance_3d(shot.origin, shot.target);
                    if (dist > slot.range) continue;
                    const Vec2 aim = angles(shot.origin, shot.intercept);
                    const bool in_cone = std::abs(std::remainder(aim[0] - actual[0], Tau)) <= radians(cone_deg * 0.45);
                    const double energy_floor = (is_npc && !attacking_npc) ? 35.0 : (hp <= 90.0 ? 0.0 : 1.5);
                    const bool has_energy = my_energy >= e_cost + energy_floor;
                    const bool fire = slot.ready && has_energy && in_cone;
                    light_aim = aim;
                    light_can_fire = fire;
                    light_slot = slot.index;
                    light_dist = dist;
                    const int chains = is_npc ? 0 : count_neighbors(*target.position, chain_r);
                    light_bonus = 60.0 + 115.0 * chains +
                        (target.has_laser ? (field(actor, "charging").is_boolean() && field(actor, "charging").get<bool>() ? 160.0 : 75.0) : 0.0);
                } else if (slot.kind == "laser") {
                    const Shot shot = shot_points(me, target, *slot.stats, arena, true);
                    if (arena.blocksLine(shot.origin, shot.target)) continue;
                    const double dist = distance_3d(shot.origin, shot.target);
                    if (dist > slot.range) continue;
                    const Vec2 aim = angles(shot.origin, shot.intercept);
                    const double full_charge = std::max(0.15, number(*slot.stats, "charge", 0.75));
                    const double min_charge = number(*slot.stats, "min_charge", 0.15);
                    const double base_dmg = number(*slot.stats, "damage", 315.0);
                    const double cookoff = target.has_mortar ? 1.6 : 1.0;
                    // Strict hitscan accuracy (0.80 * radius, 0 extra slack) so 100-280m beams never whiff!
                    const bool aligned = aim_aligned(aim, actual, dist, target.attack_radius * 0.80, 0.0) &&
                                         clear_aim_line(shot.origin, actual, dist, arena);

                    if (slot.charge > 0.0) {
                        const double frac = std::clamp(slot.charge / full_charge, 0.0, 1.0);
                        const double est_dmg = base_dmg * cube_mult * (0.35 + 0.65 * frac) * cookoff;
                        const bool ready_to_release = (slot.charge >= full_charge - 0.03) ||
                            (slot.charge >= min_charge && est_dmg >= hp * 1.02);
                        laser_aim = aim;
                        laser_slot = slot.index;
                        laser_dist = dist;
                        if (ready_to_release && aligned) {
                            laser_trigger = false;
                            laser_releasing = true;
                            laser_bonus = 260.0 + (target.has_mortar ? 180.0 : 0.0);
                        } else if (!allow_laser_charge && slot.charge >= min_charge && aligned) {
                            laser_trigger = false;
                            laser_releasing = true;
                            laser_bonus = 200.0;
                        } else {
                            // Keep holding charge (up to hold_s = 2.0s after full charge) until aim locks dead-center.
                            laser_trigger = slot.charge < full_charge + 1.5;
                            laser_bonus = 140.0 + (target.has_mortar ? 140.0 : 0.0);
                        }
                    } else if (slot.ready && allow_laser_charge && !is_npc && !enemy_holds_lightning_close &&
                               (target.has_mortar || dist > 62.0 || !gun_is_ready || !target.facing_us)) {
                        laser_aim = aim;
                        laser_slot = slot.index;
                        laser_dist = dist;
                        laser_trigger = true;
                        laser_bonus = (target.has_mortar ? 170.0 : 45.0);
                    }
                } else if (slot.kind == "mortar" && slot.ready && !is_npc) {
                    const double speed = number(*slot.stats, "speed", 72.0);
                    const double grav = number(*slot.stats, "gravity", mortar_gravity_);
                    const double splash_r = number(*slot.stats, "splash_radius", 7.0);
                    // Aim near target feet/lower torso so even slight misses explode right at their feet!
                    const Vec3 aim_target = above(*target.attack_position, 1.5);
                    const double dist = distanceXY(mortar_origin, aim_target);
                    if (dist < 14.0 || dist > slot.range) continue;
                    const int cluster = count_neighbors(*target.position, splash_r + 4.0);
                    const double target_speed = std::hypot((*target.attack_velocity)[0], (*target.attack_velocity)[1]);
                    const bool high_value_mortar = !gun_aim || !gun_is_ready || cluster >= 1 ||
                        (target.has_lightning && dist >= 18.0 && dist <= 58.0) ||
                        (target_speed < 4.5 && dist >= 22.0 && dist <= 130.0);
                    if (!high_value_mortar) continue;
                    if (const auto m_aim = mortar_solution(mortar_origin, aim_target, *target.attack_velocity,
                                                           speed, grav, arena)) {
                        // Compute exact ballistic pitch tolerance from range sensitivity dR/dpitch!
                        const double dR_dpitch = std::abs(2.0 * (speed * speed / std::max(1.0, grav)) * std::cos(2.0 * (*m_aim)[1]));
                        const double pitch_tol = std::clamp(0.45 * splash_r / std::max(35.0, dR_dpitch), 0.012, 0.045);
                        const double yaw_tol = std::clamp(std::atan2(0.75 * splash_r, dist), 0.02, 0.14);
                        const bool mortar_aligned = std::abs(std::remainder((*m_aim)[0] - actual[0], Tau)) <= yaw_tol &&
                                                    std::abs((*m_aim)[1] - actual[1]) <= pitch_tol;
                        bool safe_actual_shot = false;
                        if (mortar_aligned) {
                            // Verify the actual shell won't clip a near wall and blow ourselves up!
                            if (const auto impact = mortar_impact(mortar_origin, actual[0], actual[1], speed, grav, arena)) {
                                const double self_dist = distanceXY(self_pos, *impact);
                                const double target_err = distanceXY(aim_target, *impact);
                                safe_actual_shot = self_dist > splash_r + self_radius + 3.0 && target_err <= splash_r + 3.5;
                            }
                        }
                        mortar_aim = *m_aim;
                        mortar_can_fire = safe_actual_shot;
                        mortar_slot = slot.index;
                        mortar_dist = dist;
                        mortar_bonus = 110.0 * cluster + (target.has_lightning ? 95.0 : 0.0) +
                            (!gun_aim ? 80.0 : (target_speed < 4.5 ? 55.0 : 0.0));
                    }
                }
            }

            std::optional<GunShot> candidate;
            double candidate_dist = std::numeric_limits<double>::infinity();
            double weapon_bonus = 0.0;

            const bool mortar_priority = mortar_aim && (
                !gun_aim || !gun_is_ready || mortar_bonus >= 95.0);

            // Combine weapons intelligently on the chosen target.
            if (laser_aim && (laser_releasing || (laser_slot >= 0 && slots[static_cast<std::size_t>(laser_slot)].charge > 0.0))) {
                GunShot gs;
                gs.aim = *laser_aim;
                gs.fire[static_cast<std::size_t>(laser_slot)] = laser_trigger;
                if (gun_can_fire && gun_slot >= 0) gs.fire[static_cast<std::size_t>(gun_slot)] = true;
                gs.npc = is_npc;
                gs.target_id = *id;
                candidate = gs;
                candidate_dist = laser_dist;
                weapon_bonus = laser_bonus;
            } else if (light_aim && (light_can_fire || !gun_aim || light_dist <= 38.0)) {
                GunShot gs;
                // Aim with gun lead when gun is ready and close to lightning cone so BOTH weapons hit!
                const bool lead_in_cone = gun_aim && gun_is_ready &&
                    std::abs(std::remainder((*gun_aim)[0] - (*light_aim)[0], Tau)) <= radians(11.0);
                gs.aim = lead_in_cone ? *gun_aim : *light_aim;
                gs.fire[static_cast<std::size_t>(light_slot)] = light_can_fire;
                if (gun_can_fire && gun_slot >= 0) gs.fire[static_cast<std::size_t>(gun_slot)] = true;
                gs.npc = is_npc;
                gs.target_id = *id;
                candidate = gs;
                candidate_dist = light_dist;
                weapon_bonus = light_bonus;
            } else if (mortar_priority) {
                GunShot gs;
                gs.aim = *mortar_aim;
                gs.fire[static_cast<std::size_t>(mortar_slot)] = mortar_can_fire;
                gs.npc = is_npc;
                gs.target_id = *id;
                candidate = gs;
                candidate_dist = mortar_dist;
                weapon_bonus = mortar_bonus;
            } else if (gun_aim) {
                GunShot gs;
                gs.aim = *gun_aim;
                gs.fire[static_cast<std::size_t>(gun_slot)] = gun_can_fire;
                if (light_can_fire && light_slot >= 0) {
                    gs.fire[static_cast<std::size_t>(light_slot)] = true;
                    weapon_bonus += light_bonus;
                } else if (laser_aim && laser_trigger && laser_slot >= 0) {
                    gs.fire[static_cast<std::size_t>(laser_slot)] = true;
                    weapon_bonus += laser_bonus;
                }
                gs.npc = is_npc;
                gs.target_id = *id;
                candidate = gs;
                candidate_dist = gun_dist;
            } else if (laser_aim) {
                GunShot gs;
                gs.aim = *laser_aim;
                gs.fire[static_cast<std::size_t>(laser_slot)] = laser_trigger;
                gs.npc = is_npc;
                gs.target_id = *id;
                candidate = gs;
                candidate_dist = laser_dist;
                weapon_bonus = laser_bonus;
            } else if (mortar_aim) {
                GunShot gs;
                gs.aim = *mortar_aim;
                gs.fire[static_cast<std::size_t>(mortar_slot)] = mortar_can_fire;
                gs.npc = is_npc;
                gs.target_id = *id;
                candidate = gs;
                candidate_dist = mortar_dist;
                weapon_bonus = mortar_bonus;
            }

            if (!candidate) {
                continue;
            }
            const double fire_bonus = (candidate->firing() || laser_releasing) ? 85.0 : 0.0;
            const double score = fire_bonus + hp_bonus + threat_bonus + weapon_bonus - candidate_dist;
            if (score > best_score) {
                best_score = score;
                best = candidate;
            }
        }
    }

    // Free opportunistic lightning pass: if we picked a distant gun target while ANOTHER enemy is right
    // inside our current 30-deg lightning cone, fire lightning simultaneously for free 240+ DPS!
    if (best && mechs.is_array()) {
        for (std::size_t s = 0; s < slot_count; ++s) {
            const auto& slot = slots[s];
            if (slot.kind != "lightning" || !slot.ready || best->fire[static_cast<std::size_t>(slot.index)]) {
                continue;
            }
            const double e_cost = number(*slot.stats, "energy", 3.0);
            const double cone_deg = number(*slot.stats, "cone_deg", 30.0);
            if (my_energy < e_cost + 2.0) continue;
            const Vec3 origin = muzzle(me);
            for (const auto& actor : mechs) {
                const auto id = actor_id(actor, "id");
                if (!id || id == self_id || text(field(actor, "kind")) != "mech") continue;
                const auto p = try_point(field(actor, "pos"));
                if (!p) continue;
                const Vec3 torso = above(*p, mech_height_ * 0.5);
                if (distance_3d(origin, torso) > slot.range) continue;
                const double yaw = std::atan2(torso[1] - origin[1], torso[0] - origin[0]);
                if (std::abs(std::remainder(yaw - actual[0], Tau)) <= radians(cone_deg * 0.45) &&
                    !arena.blocksLine(origin, torso)) {
                    best->fire[static_cast<std::size_t>(slot.index)] = true;
                    break;
                }
            }
        }
    }

    // If a laser is currently charging and enemies ducked behind cover briefly, HOLD the charge
    // (up to full_charge + 1.0s) while pre-aiming the threat instead of wasting the beam into a wall!
    if (!best) {
        for (std::size_t s = 0; s < slot_count; ++s) {
            if (slots[s].kind == "laser" && slots[s].charge > 0.0) {
                const double full_charge = std::max(0.15, number(*slots[s].stats, "charge", 0.75));
                const bool hold = allow_laser_charge && slots[s].charge < full_charge + 1.0 &&
                    primary_threat && primary_threat->position;
                GunShot gs;
                gs.aim = (primary_threat && primary_threat->position)
                    ? angles(muzzle(me), above(*primary_threat->position, primary_threat->attack_height * 0.5))
                    : actual;
                gs.fire[static_cast<std::size_t>(slots[s].index)] = hold;
                return gs;
            }
        }
    }

    if (best || !allow_crates || Clock::now() >= deadline) {
        return best;
    }

    int gun_slot = -1;
    double gun_range = 65.0;
    bool gun_spare_ready = false;
    int light_slot = -1;
    double light_range = 42.0;
    bool light_spare_ready = false;
    for (std::size_t s = 0; s < slot_count; ++s) {
        if (slots[s].kind == "gun") {
            gun_slot = slots[s].index;
            gun_range = std::min(65.0, slots[s].range);
            gun_spare_ready = slots[s].ready && number(*slots[s].weapon, "ammo", 0.0) > 3.0;
        } else if (slots[s].kind == "lightning") {
            light_slot = slots[s].index;
            light_range = slots[s].range;
            light_spare_ready = slots[s].ready && my_energy > 45.0;
        }
    }
    const auto& crates = field(obs, "crates");
    if ((gun_spare_ready || light_spare_ready) && crates.is_array()) {
        const Vec3 origin = muzzle(me);
        double nearest_crate = std::numeric_limits<double>::infinity();
        for (const auto& crate : crates) {
            if (Clock::now() >= deadline) {
                break;
            }
            const auto pos = try_point(field(crate, "pos"));
            if (!pos) continue;
            const Vec3 center = above(*pos, 1.5);
            const double dist = distance_3d(origin, center);
            if (dist > gun_range || dist >= nearest_crate || arena.blocksLine(origin, center)) {
                continue;
            }
            const Vec2 aim = angles(origin, center);
            GunShot gs;
            gs.aim = aim;
            if (gun_slot >= 0 && gun_spare_ready) {
                gs.fire[static_cast<std::size_t>(gun_slot)] =
                    aim_aligned(aim, actual, dist, 1.8) && clear_aim_line(origin, actual, dist, arena);
            }
            if (light_slot >= 0 && light_spare_ready && dist <= light_range &&
                std::abs(std::remainder(aim[0] - actual[0], Tau)) <= radians(13.0)) {
                gs.fire[static_cast<std::size_t>(light_slot)] = true;
            }
            gs.crate = true;
            best = gs;
            nearest_crate = dist;
        }
    }
    return best;
}

bool return_fire(const Json& obs, const std::optional<Threat>& threat, const Vec2& move, const Arena& arena,
                 bool returning_to_cover) {
    if (!threat || !threat->facing_us || !threat->attack_id || !threat->attack_position) {
        return false;
    }
    const auto& me = field(obs, "self");
    const auto position = point_or(me, "pos");
    const double dx = (*threat->attack_position)[0] - position[0];
    const double dy = (*threat->attack_position)[1] - position[1];
    if (std::hypot(move[0], move[1]) < 0.15 ||
        (!returning_to_cover && dx * move[0] + dy * move[1] > 0.05 * std::hypot(dx, dy))) {
        return false;
    }
    return ready_aligned_shot(me, *threat, arena);
}

bool opportunity_fire(const Json& obs, const std::optional<Threat>& target, const Arena& arena) {
    return target && !target->hostile && !target->facing_us && target->attack_id && target->attack_position &&
           ready_aligned_shot(field(obs, "self"), *target, arena);
}

std::vector<Cube> safe_cubes(const Json& obs, const std::optional<Threat>& threat,
                             double pickup_radius, const std::function<bool(const Vec2&)>& safe_point) {
    const auto& events = field(obs, "events");
    const bool hostile_threat = threat && (threat->hostile || threat->damaged_recently || threat->laser_charging);
    if (hostile_threat || (events.is_array() && std::any_of(events.begin(), events.end(), [](const auto& event) {
            return text(field(event, "type")) == "damaged";
        }))) {
        return {};
    }
    const auto position = point_or(field(obs, "self"), "pos");
    std::vector<std::pair<double, Cube>> candidates;
    const auto& raw_cubes = field(obs, "cubes");
    if (raw_cubes.is_array()) {
        for (const auto& cube : raw_cubes) {
            const auto cube_pos = try_point(field(cube, "pos"));
            if (!cube_pos) continue;
            const double distance = distanceXY(*cube_pos, position);
            if (distance <= CubeDetourMeters + pickup_radius && safe_point({(*cube_pos)[0], (*cube_pos)[1]})) {
                candidates.push_back({distance, Cube{number(cube, "id", 0.0), *cube_pos}});
            }
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
