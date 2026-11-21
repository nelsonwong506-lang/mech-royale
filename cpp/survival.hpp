#pragma once

#include "common.hpp"
#include "tactics.hpp"

#include <optional>
#include <vector>

namespace mr {

class Arena;
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
    const Json* weapons = nullptr;
    const Json* config = nullptr;
    const Json* stats = nullptr;
    std::optional<Vec3> returnCover;
    double cubes = 0.0;
    bool laserShorted = false;
    double regenWait = 5.0;
    double hp = 1400.0;
    double maxHp = 1400.0;
    double energy = 100.0;
    double maxEnergy = 100.0;
    double muzzleHeight = 7.0;
    double strafeSign = 1.0;
    bool hasLightning = false;
    bool hasLaser = false;
    bool hasMortar = false;
    bool laserCharging = false;
    bool reloading = false;
    bool wantRegen = false;
    bool onRoof = false;
    Vec3 anchor{};
    Vec2 velocity{};
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
int wallCover(const Vec3& feet, const EnemyInfo& threat, double ownHeight, const Arena& arena);
double exchangeValue(const Vec3& from, const Vec3& candidate, const Threat& primary,
                     const std::vector<EnemyInfo>& enemies, const CombatContext& ctx,
                     const Motion& motion, const Arena& arena);
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
