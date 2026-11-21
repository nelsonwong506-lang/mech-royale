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

RotationWindow rotationWindow(const Vec3& from, const Zone& zone, const Motion& motion,
                              std::optional<double> routeSeconds = std::nullopt);
bool avoidsApproach(const Vec3& from, const Vec2& toward, const std::vector<Vec3>& enemies);
int wallCover(const Vec3& feet, const Vec3& threat, double ownHeight, const Arena& arena);
std::optional<RetreatPlan> planRetreat(const Vec3& from, const Threat& threat, const std::vector<Vec3>& enemies,
                                       const Zone& zone, const Motion& motion, const Arena& arena,
                                       Navigator& navigator, Clock::time_point deadline);

}  // namespace mr
