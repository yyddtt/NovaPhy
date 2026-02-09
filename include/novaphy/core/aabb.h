#pragma once

#include "novaphy/math/math_types.h"

namespace novaphy {

/// Axis-Aligned Bounding Box
struct AABB {
    Vec3f min = Vec3f::Constant(std::numeric_limits<float>::max());
    Vec3f max = Vec3f::Constant(-std::numeric_limits<float>::max());

    AABB() = default;
    AABB(const Vec3f& min_pt, const Vec3f& max_pt) : min(min_pt), max(max_pt) {}

    /// Check if this AABB overlaps with another
    bool overlaps(const AABB& other) const {
        return (min.x() <= other.max.x() && max.x() >= other.min.x()) &&
               (min.y() <= other.max.y() && max.y() >= other.min.y()) &&
               (min.z() <= other.max.z() && max.z() >= other.min.z());
    }

    /// Expand this AABB to include a point
    void expand(const Vec3f& point) {
        min = min.cwiseMin(point);
        max = max.cwiseMax(point);
    }

    /// Merge this AABB with another
    AABB merged(const AABB& other) const {
        return AABB(min.cwiseMin(other.min), max.cwiseMax(other.max));
    }

    /// Expand the AABB by a margin on all sides
    AABB expanded(float margin) const {
        Vec3f m(margin, margin, margin);
        return AABB(min - m, max + m);
    }

    /// Center of the AABB
    Vec3f center() const { return 0.5f * (min + max); }

    /// Half-extents (half-size along each axis)
    Vec3f half_extents() const { return 0.5f * (max - min); }

    /// Surface area (used for BVH cost heuristics)
    float surface_area() const {
        Vec3f d = max - min;
        return 2.0f * (d.x() * d.y() + d.y() * d.z() + d.z() * d.x());
    }

    /// Check if the AABB is valid (min <= max on all axes)
    bool is_valid() const {
        return min.x() <= max.x() && min.y() <= max.y() && min.z() <= max.z();
    }

    /// Create AABB from center and half-extents
    static AABB from_center_half_extents(const Vec3f& center, const Vec3f& half) {
        return AABB(center - half, center + half);
    }

    /// Create AABB for a sphere
    static AABB from_sphere(const Vec3f& center, float radius) {
        Vec3f r(radius, radius, radius);
        return AABB(center - r, center + r);
    }

    /// Create AABB for an oriented box (8 corners transformed)
    static AABB from_oriented_box(const Vec3f& half_extents, const Transform& t) {
        AABB result;
        // Transform each of the 8 corners
        for (int i = 0; i < 8; ++i) {
            Vec3f corner(
                (i & 1) ? half_extents.x() : -half_extents.x(),
                (i & 2) ? half_extents.y() : -half_extents.y(),
                (i & 4) ? half_extents.z() : -half_extents.z());
            result.expand(t.transform_point(corner));
        }
        return result;
    }
};

}  // namespace novaphy
