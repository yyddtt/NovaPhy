"""Tests for NovaPhy math types and spatial algebra."""

import numpy as np
import numpy.testing as npt
import novaphy


def test_transform_identity():
    t = novaphy.Transform.identity()
    npt.assert_allclose(t.position, [0, 0, 0], atol=1e-6)
    # rotation is [x, y, z, w] - identity quaternion is [0, 0, 0, 1]
    npt.assert_allclose(t.rotation, [0, 0, 0, 1], atol=1e-6)


def test_transform_translation():
    t = novaphy.Transform.from_translation(np.array([1, 2, 3], dtype=np.float32))
    npt.assert_allclose(t.position, [1, 2, 3], atol=1e-6)
    p = t.transform_point(np.array([0, 0, 0], dtype=np.float32))
    npt.assert_allclose(p, [1, 2, 3], atol=1e-6)


def test_transform_composition():
    t1 = novaphy.Transform.from_translation(np.array([1, 0, 0], dtype=np.float32))
    t2 = novaphy.Transform.from_translation(np.array([0, 2, 0], dtype=np.float32))
    t3 = t1 * t2
    npt.assert_allclose(t3.position, [1, 2, 0], atol=1e-6)


def test_transform_inverse():
    t = novaphy.Transform.from_translation(np.array([3, 4, 5], dtype=np.float32))
    inv = t.inverse()
    identity = t * inv
    npt.assert_allclose(identity.position, [0, 0, 0], atol=1e-5)


def test_transform_rotation():
    import math
    t = novaphy.Transform.from_axis_angle(
        np.array([0, 0, 1], dtype=np.float32),
        math.pi / 2
    )
    p = t.transform_point(np.array([1, 0, 0], dtype=np.float32))
    npt.assert_allclose(p, [0, 1, 0], atol=1e-5)


def test_skew_matrix():
    v = np.array([1, 2, 3], dtype=np.float32)
    S = novaphy.skew(v)
    assert S.shape == (3, 3)
    # skew(v) * u = v x u
    u = np.array([4, 5, 6], dtype=np.float32)
    cross = S @ u
    expected = np.cross(v, u)
    npt.assert_allclose(cross, expected, atol=1e-5)


def test_deg_rad_conversion():
    assert abs(novaphy.deg2rad(180.0) - np.pi) < 1e-5
    assert abs(novaphy.rad2deg(np.pi) - 180.0) < 1e-3


def test_spatial_transform_identity():
    st = novaphy.SpatialTransform.identity()
    v = np.array([1, 2, 3, 4, 5, 6], dtype=np.float32)
    result = st.apply_motion(v)
    npt.assert_allclose(result, v, atol=1e-6)


def test_spatial_cross_motion():
    v = np.array([0, 0, 1, 0, 0, 0], dtype=np.float32)  # rotation about z
    u = np.array([1, 0, 0, 0, 0, 0], dtype=np.float32)  # rotation about x
    result = novaphy.spatial_cross_motion(v, u)
    # [0,0,1] x [1,0,0] = [0,1,0] for angular part
    npt.assert_allclose(result[:3], [0, 1, 0], atol=1e-6)


def test_spatial_inertia_matrix():
    mass = 2.0
    com = np.array([0, 0, 0], dtype=np.float32)
    I_rot = np.eye(3, dtype=np.float32) * 0.5
    I_spatial = novaphy.spatial_inertia_matrix(mass, com, I_rot)
    assert I_spatial.shape == (6, 6)
    # When com=0, the spatial inertia is block diagonal
    npt.assert_allclose(I_spatial[:3, :3], I_rot, atol=1e-6)
    npt.assert_allclose(I_spatial[3:, 3:], mass * np.eye(3), atol=1e-6)
    npt.assert_allclose(I_spatial[:3, 3:], np.zeros((3, 3)), atol=1e-6)
