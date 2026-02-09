"""NovaPhy: A 3D physics engine for embodied intelligence."""

from novaphy._core import (
    version,
    # Math types
    Transform,
    SpatialTransform,
    # Math functions
    skew,
    spatial_cross_motion,
    spatial_cross_force,
    spatial_inertia_matrix,
    deg2rad,
    rad2deg,
)

__version__ = version()

__all__ = [
    "version",
    "Transform",
    "SpatialTransform",
    "skew",
    "spatial_cross_motion",
    "spatial_cross_force",
    "spatial_inertia_matrix",
    "deg2rad",
    "rad2deg",
]
