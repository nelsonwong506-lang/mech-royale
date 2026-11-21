#pragma once

#include "common.hpp"

#include <array>
#include <functional>
#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

namespace mr {

class Arena;

struct Threat {
    std::optional<Vec3> position;
    std::optional<int> actor_id;
    bool hostile = false;
    bool is_npc = false;
    bool mortar_only = false;
    bool has_mortar = false;
    bool has_laser = false;
    bool has_lightning = false;
    double hp = 1400.0;
    std::optional<Vec3> attack_position;
    std::optional<int> attack_id;
    std::optional<Vec3> attack_velocity;
    double attack_radius = 2.5;
    double attack_height = 9.0;
    bool facing_us = false;
    bool laser_charging = false;
    bool damaged_recently = false;
    std::optional<Vec2> enemy_aim;
    double enemy_range = 0.0;
    double enemy_muzzle_height = 7.0;
};

struct EnemyInfo {
    int id = -1;
    Vec3 pos{};
    Vec3 vel{};
    double hp = 1400.0;
    double radius = 2.5;
    double height = 9.0;
    bool is_npc = false;
    bool has_lightning = false;
    bool has_laser = false;
    bool has_mortar = false;
    bool charging = false;
    bool facing_us = false;
    std::optional<Vec2> aim;
};

struct Cube {
    double id;
    Vec3 pos;
};

struct GunShot {
    Vec2 aim{};
    std::array<bool, 2> fire{false, false};
    bool crate = false;
    bool npc = false;
    int target_id = -1;

    bool firing() const { return fire[0] || fire[1]; }
};

struct ProjectileDodge {
    Vec2 direction{};
    bool urgent_mortar = false;
};

struct SoundAlert {
    double bearing = 0.0;
    bool laser_charge = false;
    bool near_hostile = false;
    bool near_jet = false;
    bool near_reload = false;
};

struct SoundContact {
    double bearing = 0.0;
    double est_dist = 45.0;
    std::string weapon;
    bool near = false;
    bool laser_charging = false;
    bool jet = false;
    double expires = -std::numeric_limits<double>::infinity();
};

class ThreatTracker {
public:
    explicit ThreatTracker(const Json& config);

    std::optional<Threat> update(const Json& obs, const Arena& arena,
                                 Clock::time_point deadline = Clock::time_point::max());
    std::optional<Threat> duel_target(const Json& obs, const Arena& arena,
                                      Clock::time_point deadline = Clock::time_point::max());
    std::optional<GunShot> visible_shot(const Json& obs, const Arena& arena,
                                        const std::optional<Threat>& primary_threat = std::nullopt,
                                        Clock::time_point deadline = Clock::time_point::max(),
                                        bool allow_crates = false,
                                        bool allow_npcs = false,
                                        bool allow_laser_charge = true,
                                        std::optional<int> sticky_target_id = std::nullopt) const;
    std::vector<EnemyInfo> visible_enemies(const Json& obs, const Arena& arena) const;
    std::optional<ProjectileDodge> projectile_dodge(const Json& obs, const Arena& arena) const;
    std::optional<SoundAlert> sound_alert(double now) const;
    std::vector<SoundContact> unseen_sound_contacts(const Json& obs,
                                                    const std::vector<EnemyInfo>& visible) const;
    bool can_collect_cubes(const Json& obs) const;
    double last_combat_time() const;
    double last_damage_time() const;
    bool laser_shorted(double now) const;
    std::optional<double> rear_hit_bearing(double now) const;
    std::optional<double> sound_heading(double now) const;

private:
    Threat describe_enemy(const Json& actor, int id) const;

    struct AttackMemory {
        double time = -std::numeric_limits<double>::infinity();
        std::optional<Vec3> position;
        bool is_npc = false;
    };

    struct HealthSample {
        double time = -std::numeric_limits<double>::infinity();
        double hp = -1.0;
        bool in_zone = false;
    };

    std::unordered_map<int, AttackMemory> attacks_{};
    std::unordered_map<int, HealthSample> health_samples_{};
    std::array<std::array<std::optional<double>, 16>, 16> duel_times_{};
    std::optional<Vec3> unknown_position_;
    double unknown_until_ = -std::numeric_limits<double>::infinity();
    std::optional<double> sound_heading_;
    double sound_until_ = -std::numeric_limits<double>::infinity();
    std::optional<SoundAlert> sound_alert_;
    double sound_alert_until_ = -std::numeric_limits<double>::infinity();
    std::vector<SoundContact> sound_contacts_{};
    double last_damage_at_ = -std::numeric_limits<double>::infinity();
    double last_hit_dealt_at_ = -std::numeric_limits<double>::infinity();
    double last_hostile_damage_at_ = -std::numeric_limits<double>::infinity();
    double last_duel_at_ = -std::numeric_limits<double>::infinity();
    double laser_shorted_until_ = -std::numeric_limits<double>::infinity();
    std::optional<double> rear_hit_bearing_;
    double rear_hit_until_ = -std::numeric_limits<double>::infinity();
    double mech_radius_;
    double mech_height_;
    double muzzle_height_;
    double turn_rate_;
    double pitch_rate_;
    double min_pitch_;
    double max_pitch_;
    double size_per_cube_;
    double damage_per_cube_;
    double cube_stat_cap_;
    double mortar_gravity_;
    std::array<double, 4> weapon_ranges_;
};

bool enemy_aims_at(const Threat& threat, const Vec3& feet, double radius, double height, const Arena& arena);
std::optional<Vec2> retreat_aim(const Json& obs, const std::optional<Threat>& threat);
bool return_fire(const Json& obs, const std::optional<Threat>& threat, const Vec2& move, const Arena& arena,
                 bool returning_to_cover = false);
bool opportunity_fire(const Json& obs, const std::optional<Threat>& target, const Arena& arena);
std::vector<Cube> safe_cubes(const Json& obs, const std::optional<Threat>& threat,
                             double pickup_radius, const std::function<bool(const Vec2&)>& safe_point);
int cover_quality(const Vec3& position, const std::optional<Threat>& threat, const Arena& arena,
                  double height = 9.0);

}  // namespace mr
