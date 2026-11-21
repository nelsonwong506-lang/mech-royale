#pragma once

#include "common.hpp"

#include <optional>
#include <vector>

namespace mr {

class Arena;
class Navigator;
struct Motion;
struct Threat;
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
std::optional<RetreatPlan> planRetreat(const Vec3& from, const Threat& threat, const std::vector<Vec3>& enemies,
                                       const Zone& zone, const Motion& motion, const Arena& arena,
                                       Navigator& navigator, Clock::time_point deadline);
std::optional<Vec3> planEngageStep(const Vec3& from, const Threat& threat, const std::vector<Vec3>& enemies,
                                   const Zone& zone, const Motion& motion, const Arena& arena,
                                   Navigator& navigator, double desiredRange, double strafeSign,
                                   double muzzleHeight, Clock::time_point deadline);
std::optional<DefensiveHop> planDefensiveHop(const Json& me, const Threat& threat,
                                             const std::vector<Vec3>& enemies, const Zone& zone,
                                             const Motion& motion, const Arena& arena, Navigator& navigator,
                                             bool preferLateral, bool liftFirst, Clock::time_point deadline);

}  // namespace mr
