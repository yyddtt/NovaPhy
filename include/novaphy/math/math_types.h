#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

#include "novaphy/novaphy_types.h"

namespace novaphy {

// Vector types (float32)
using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Vec4f = Eigen::Vector4f;
using VecXf = Eigen::VectorXf;

// Matrix types (float32)
using Mat3f = Eigen::Matrix3f;
using Mat4f = Eigen::Matrix4f;
using MatXf = Eigen::MatrixXf;
using Mat6f = Eigen::Matrix<float, 6, 6>;

// Quaternion (float32)
using Quatf = Eigen::Quaternionf;

/// Rigid body transform: position + orientation.
struct Transform {
    Vec3f position = Vec3f::Zero();
    Quatf rotation = Quatf::Identity();

    Transform() = default;
    Transform(const Vec3f& p, const Quatf& r) : position(p), rotation(r.normalized()) {}

    /// Compose transforms: this * other
    Transform operator*(const Transform& other) const {
        return Transform(position + rotation * other.position,
                         rotation * other.rotation);
    }

    /// Inverse transform
    Transform inverse() const {
        Quatf inv_rot = rotation.conjugate();
        return Transform(inv_rot * (-position), inv_rot);
    }

    /// Transform a point from local to world space
    Vec3f transform_point(const Vec3f& p) const { return position + rotation * p; }

    /// Transform a direction vector (rotation only, no translation)
    Vec3f transform_vector(const Vec3f& v) const { return rotation * v; }

    /// Get the 3x3 rotation matrix
    Mat3f rotation_matrix() const { return rotation.toRotationMatrix(); }

    /// Identity transform
    static Transform identity() { return Transform(); }

    /// Translation-only transform
    static Transform from_translation(const Vec3f& t) {
        return Transform(t, Quatf::Identity());
    }

    /// Rotation-only transform
    static Transform from_rotation(const Quatf& q) {
        return Transform(Vec3f::Zero(), q);
    }

    /// From axis-angle rotation
    static Transform from_axis_angle(const Vec3f& axis, float angle) {
        return Transform(Vec3f::Zero(),
                         Quatf(Eigen::AngleAxisf(angle, axis.normalized())));
    }
};

}  // namespace novaphy
