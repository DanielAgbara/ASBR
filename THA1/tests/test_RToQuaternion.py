import numpy as np
import pytest

from THA1.rotation import RToQuaternion


def hat(w):
    wx, wy, wz = w
    return np.array([[0.0, -wz, wy], [wz, 0.0, -wx], [-wy, wx, 0.0]], dtype=float)


def exp_so3(axis, theta):
    """Rodrigues formula: axis-angle -> SO(3) rotation matrix"""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    W = hat(axis)
    I = np.eye(3)
    return I + np.sin(theta) * W + (1.0 - np.cos(theta)) * (W @ W)


def quat_to_R(q):
    """Quaternion q=[q0,q1,q2,q3] -> SO(3) rotation matrix"""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)

    q0, q1, q2, q3 = q
    return np.array(
        [
            [
                q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3,
                2 * (q1 * q2 - q0 * q3),
                2 * (q0 * q2 + q1 * q3),
            ],
            [
                2 * (q0 * q3 + q1 * q2),
                q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3,
                2 * (q2 * q3 - q0 * q1),
            ],
            [
                2 * (q1 * q3 - q0 * q2),
                2 * (q0 * q1 + q2 * q3),
                q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3,
            ],
        ],
        dtype=float,
    )


def is_unit_quat(q, atol=1e-6):
    return abs(np.linalg.norm(q) - 1.0) < atol


@pytest.mark.parametrize(
    "axis,theta",
    [
        ([1, 0, 0], 0.0),
        ([0, 1, 0], np.pi / 6),
        ([0, 0, 1], np.pi / 2),
        ([1, 2, 3], np.pi / 3),
        ([1, 1, 0], -np.pi / 4),
        ([1, 0, 1], np.pi),  # pi-case (important)
    ],
)
def test_RToQuaternion_fixed(axis, theta):
    R = exp_so3(axis, theta)
    q = RToQuaternion(R)

    assert q.shape == (4,)
    assert np.all(np.isfinite(q))
    assert is_unit_quat(q)

    R2 = quat_to_R(q)
    err = np.linalg.norm(R - R2, ord="fro")

    # Allow small numerical error
    assert err < 1e-6


def test_RToQuaternion_random():
    rng = np.random.default_rng(42)
    N = 200

    for _ in range(N):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)

        theta = (rng.random() * 2 - 1) * np.pi  # uniform in [-pi, pi]

        R = exp_so3(axis, theta)
        q = RToQuaternion(R)

        assert is_unit_quat(q)

        R2 = quat_to_R(q)
        err = np.linalg.norm(R - R2, ord="fro")
        assert err < 1e-6
