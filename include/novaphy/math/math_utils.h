#pragma once

#include "novaphy/math/math_types.h"

namespace novaphy {

/// Skew-symmetric matrix from a vector: [v]_x
/// Such that skew(a) * b = a.cross(b)
inline Mat3f skew(const Vec3f& v) {
    Mat3f m;
    m << 0, -v.z(), v.y(),
         v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;
    return m;
}

/// Create quaternion from axis and angle (radians)
inline Quatf quat_from_axis_angle(const Vec3f& axis, float angle) {
    return Quatf(Eigen::AngleAxisf(angle, axis.normalized()));
}

/// Clamp a value between min and max
inline float clampf(float val, float lo, float hi) {
    return std::max(lo, std::min(val, hi));
}

/// Linear interpolation
inline float lerpf(float a, float b, float t) { return a + t * (b - a); }

/// Constant: pi
constexpr float PI = 3.14159265358979323846f;

/// Degrees to radians
inline float deg2rad(float deg) { return deg * PI / 180.0f; }

/// Radians to degrees
inline float rad2deg(float rad) { return rad * 180.0f / PI; }

}  // namespace novaphy
