#pragma once

#include "common.hpp"

#include <array>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace mr {

class Arena;

struct Threat {
    std::optional<Vec3> position;
    std::optional<int> actor_id;
    bool hostile = false;
    std::optional<Vec3> attack_position;
    std::optional<int> attack_id;
    std::optional<Vec3> attack_velocity;
    double attack_radius = 2.5;
    double attack_height = 9.0;
    bool facing_us = false;
    std::optional<Vec2> enemy_aim;
    double enemy_range = 0.0;
    double enemy_muzzle_height = 7.0;
};

struct Cube {
    double id;
    Vec3 pos;
};

class ThreatTracker {
public:
    explicit ThreatTracker(const Json& config);

    std::optional<Threat> update(const Json& obs, const Arena& arena);
    bool can_collect_cubes(const Json& obs) const;

private:
    Threat describe_enemy(const Json& actor, int id) const;

    struct AttackMemory {
        double time = -std::numeric_limits<double>::infinity();
        std::optional<Vec3> position;
    };

    std::array<AttackMemory, 16> attacks_{};
    std::optional<Vec3> unknown_position_;
    double unknown_until_ = -std::numeric_limits<double>::infinity();
    double last_damage_at_ = -std::numeric_limits<double>::infinity();
    double mech_radius_;
    double mech_height_;
    double muzzle_height_;
    double turn_rate_;
    double pitch_rate_;
    double size_per_cube_;
    double mortar_gravity_;
    std::array<double, 4> weapon_ranges_;
};

bool enemy_aims_at(const Threat& threat, const Vec3& feet, double radius, double height, const Arena& arena);
std::optional<Vec2> retreat_aim(const Json& obs, const std::optional<Threat>& threat);
bool return_fire(const Json& obs, const std::optional<Threat>& threat, const Vec2& move, const Arena& arena,
                 bool returning_to_cover = false);
std::vector<Cube> safe_cubes(const Json& obs, const std::optional<Threat>& threat,
                             double pickup_radius, const std::function<bool(const Vec2&)>& safe_point);
int cover_quality(const Vec3& position, const std::optional<Threat>& threat, const Arena& arena,
                  double height = 9.0);

}  // namespace mr
