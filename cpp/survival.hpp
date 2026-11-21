#pragma once

#include "arena.hpp"
#include "common.hpp"
#include "tactics.hpp"

#include <optional>
#include <vector>

namespace mr {

class Navigator;
struct Motion;
struct Zone;

struct RotationWindow {
    bool urgent;
    double secondsUntilRisk;
    double estimatedTravel;
};

struct RetreatPlan {
    Vec3 target;
    bool hardCover;
};

struct CombatContext {
    double hp = 1400.0;
    double maxHp = 1400.0;
    double energy = 100.0;
    double maxEnergy = 100.0;
    double cubes = 0.0;
    double muzzleHeight = 7.0;
    double strafeSign = 1.0;
    double weaveInOut = 1.0;
    bool hasLightning = false;
    bool hasLaser = false;
    bool hasMortar = false;
    bool hasReadyMortar = false;
    bool laserCharging = false;
    bool reloading = false;
    bool wantRegen = false;
    bool needReposition = false;
    bool onRoof = false;
    const RooftopNode* currentRoof = nullptr;
    Vec3 anchor{};
    Vec2 velocity{};
    std::optional<Vec2> bulletDodge;
    std::optional<Vec2> coverPull;
    std::optional<Vec2> treeCover;
    std::vector<Vec3> nearbyCubes;
};

struct CombatMove {
    Vec3 target;
    bool boost = false;
    bool hardCover = false;
    int exposedCount = 0;
    const char* label = "engage";
};

struct DefensiveHop {
    Vec3 landing;
    Vec2 launch;
    double duration;
    double steerAfter;
};

RotationWindow rotationWindow(const Vec3& from, const Zone& zone, const Motion& motion,
                              std::optional<double> routeSeconds = std::nullopt);
bool avoidsApproach(const Vec3& from, const Vec2& toward, const std::vector<Vec3>& enemies);
double approachPenalty(const Vec3& from, const Vec2& toward, const std::vector<Vec3>& enemies);
int wallCover(const Vec3& feet, const Vec3& threat, double ownHeight, const Arena& arena);
std::optional<double> frontArmorYaw(const Vec3& from, const std::vector<EnemyInfo>& enemies,
                                    double ownHeight, const Arena& arena);
std::optional<RetreatPlan> planRetreat(const Vec3& from, const Threat& threat, const std::vector<Vec3>& enemies,
                                       const Zone& zone, const Motion& motion, const Arena& arena,
                                       Navigator& navigator, Clock::time_point deadline);
std::optional<CombatMove> planCombatPosition(const Vec3& from, const Threat& primary,
                                             const std::vector<EnemyInfo>& enemies,
                                             const CombatContext& ctx, const Zone& zone,
                                             const Motion& motion, const Arena& arena,
                                             Navigator& navigator, Clock::time_point deadline);
std::optional<DefensiveHop> planDefensiveHop(const Json& me, const Threat& threat,
                                             const std::vector<Vec3>& enemies, const Zone& zone,
                                             const Motion& motion, const Arena& arena, Navigator& navigator,
                                             bool preferLateral, bool liftFirst, Clock::time_point deadline);

}  // namespace mr
