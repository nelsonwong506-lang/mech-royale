#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <numbers>
#include <nlohmann/json.hpp>

namespace mr {

using Json = nlohmann::json;
using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;
using Clock = std::chrono::steady_clock;

template <std::size_t A, std::size_t B>
double distanceXY(const std::array<double, A>& a, const std::array<double, B>& b) {
    return std::hypot(a[0] - b[0], a[1] - b[1]);
}

template <std::size_t A, std::size_t B>
Vec2 direction(const std::array<double, A>& from, const std::array<double, B>& to,
               double magnitude = 1.0) {
    const double dx = to[0] - from[0];
    const double dy = to[1] - from[1];
    const double norm = std::hypot(dx, dy);
    const double scale = norm > 1e-6 ? magnitude / norm : 0.0;
    return {dx * scale, dy * scale};
}

inline Vec2 horizontal(const Vec3& point) {
    return {point[0], point[1]};
}

inline double radians(double degrees) {
    return degrees * (std::numbers::pi / 180.0);
}

} // namespace mr
