#pragma once

#include "novaphy/math/math_types.h"
#include "novaphy/math/math_utils.h"

namespace novaphy {

/// 6D spatial vector: [angular(3); linear(3)] (Featherstone convention)
using SpatialVector = Eigen::Matrix<float, 6, 1>;
/// 6x6 spatial matrix (inertia, transforms)
using SpatialMatrix = Eigen::Matrix<float, 6, 6>;

/// Helper: get angular part of spatial vector (top 3)
inline Vec3f spatial_angular(const SpatialVector& v) { return v.head<3>(); }
/// Helper: get linear part of spatial vector (bottom 3)
inline Vec3f spatial_linear(const SpatialVector& v) { return v.tail<3>(); }

/// Construct spatial vector from angular and linear parts
inline SpatialVector make_spatial(const Vec3f& angular, const Vec3f& linear) {
    SpatialVector v;
    v.head<3>() = angular;
    v.tail<3>() = linear;
    return v;
}

/// Spatial cross product for motion vectors: v x_m u
/// [w; v] x_m [w'; v'] = [w x w'; w x v' + v x w']
inline SpatialVector spatial_cross_motion(const SpatialVector& v, const SpatialVector& u) {
    Vec3f w = spatial_angular(v);
    Vec3f vl = spatial_linear(v);
    Vec3f wp = spatial_angular(u);
    Vec3f vp = spatial_linear(u);
    return make_spatial(w.cross(wp), w.cross(vp) + vl.cross(wp));
}

/// Spatial cross product for force vectors: v x* f
/// [w; v] x* [n; f] = [w x n + v x f; w x f]
inline SpatialVector spatial_cross_force(const SpatialVector& v, const SpatialVector& f) {
    Vec3f w = spatial_angular(v);
    Vec3f vl = spatial_linear(v);
    Vec3f n = spatial_angular(f);
    Vec3f fl = spatial_linear(f);
    return make_spatial(w.cross(n) + vl.cross(fl), w.cross(fl));
}

/// Spatial transform: rotation E and translation r.
/// Transforms spatial vectors from frame B to frame A.
/// Convention: X_{A<-B} = (E, r) where E rotates B to A, and r is the origin of B in A.
struct SpatialTransform {
    Mat3f E = Mat3f::Identity();  // rotation (B to A)
    Vec3f r = Vec3f::Zero();     // translation (origin of B expressed in A)

    SpatialTransform() = default;
    SpatialTransform(const Mat3f& rot, const Vec3f& trans) : E(rot), r(trans) {}

    /// Apply this transform to a motion vector
    SpatialVector apply_motion(const SpatialVector& v) const {
        Vec3f w = spatial_angular(v);
        Vec3f vl = spatial_linear(v);
        Vec3f w_new = E * w;
        Vec3f v_new = E * vl + r.cross(E * w);
        return make_spatial(w_new, v_new);
    }

    /// Apply transpose of this transform to a force vector
    SpatialVector apply_force(const SpatialVector& f) const {
        Vec3f n = spatial_angular(f);
        Vec3f fl = spatial_linear(f);
        Vec3f fl_new = E.transpose() * fl;
        Vec3f n_new = E.transpose() * (n - r.cross(fl));
        return make_spatial(n_new, fl_new);
    }

    /// Convert to 6x6 matrix representation
    SpatialMatrix to_matrix() const {
        Mat3f rx = skew(r);
        SpatialMatrix X;
        X.topLeftCorner<3, 3>() = E;
        X.topRightCorner<3, 3>() = Mat3f::Zero();
        X.bottomLeftCorner<3, 3>() = rx * E;
        X.bottomRightCorner<3, 3>() = E;
        return X;
    }

    /// Inverse transform
    SpatialTransform inverse() const {
        Mat3f Et = E.transpose();
        return SpatialTransform(Et, -Et * r);
    }

    /// Compose transforms: this * other (A<-B * B<-C = A<-C)
    SpatialTransform operator*(const SpatialTransform& other) const {
        return SpatialTransform(E * other.E, r + E * other.r);
    }

    /// Create from a Transform
    static SpatialTransform from_transform(const Transform& t) {
        return SpatialTransform(t.rotation_matrix(), t.position);
    }

    /// Identity transform
    static SpatialTransform identity() { return SpatialTransform(); }
};

/// Build a 6x6 spatial inertia matrix from mass, center of mass (in body frame),
/// and rotational inertia (about center of mass, in body frame).
///
/// I_spatial = [ I_rot + m * [c]_x^T * [c]_x,  m * [c]_x^T ]
///             [ m * [c]_x,                      m * I_3     ]
inline SpatialMatrix spatial_inertia_matrix(float mass, const Vec3f& com,
                                            const Mat3f& I_rot) {
    Mat3f cx = skew(com);
    SpatialMatrix I;
    I.topLeftCorner<3, 3>() = I_rot + mass * cx.transpose() * cx;
    I.topRightCorner<3, 3>() = mass * cx.transpose();
    I.bottomLeftCorner<3, 3>() = mass * cx;
    I.bottomRightCorner<3, 3>() = mass * Mat3f::Identity();
    return I;
}

/// Transform a spatial inertia matrix by a spatial transform X.
/// I_new = X * I * X^T  (in matrix form)
inline SpatialMatrix transform_spatial_inertia(const SpatialTransform& X,
                                               const SpatialMatrix& I) {
    SpatialMatrix Xm = X.to_matrix();
    return Xm * I * Xm.transpose();
}

}  // namespace novaphy
