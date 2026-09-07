import numpy as np
import pytest

from THA1.rotation import AxisAngleToR, RToAxisAngle


def hat(w):
    wx, wy, wz = w
    return np.array([[0.0, -wz, wy], [wz, 0.0, -wx], [-wy, wx, 0.0]], dtype=float)


def exp_so3(axis, theta):
    """Rodrigues: axis-angle -> R"""
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    W = hat(axis)
    I = np.eye(3)
    return I + np.sin(theta) * W + (1.0 - np.cos(theta)) * (W @ W)


def is_SO3(R, atol=1e-6):
    return np.allclose(R.T @ R, np.eye(3), atol=atol) and np.allclose(
        np.linalg.det(R), 1.0, atol=atol
    )


@pytest.mark.parametrize(
    "axis,theta",
    [
        ([0, 0, 1], np.pi / 2),
        ([1, 0, 0], np.pi / 3),
        ([1, 2, 3], -1.1),
        ([1, -1, 2], 2.2),
    ],
)
def test_roundtrip_fixed(axis, theta):
    # Make a clean SO(3) matrix R
    R = exp_so3(axis, theta)
    assert is_SO3(R)

    axis_hat, theta_hat = RToAxisAngle(R)
    R2 = AxisAngleToR(axis_hat, theta_hat)

    err = np.linalg.norm(R - R2, ord="fro")
    assert err < 1e-6


def test_roundtrip_random_general():
    rng = np.random.default_rng(123)
    N = 400
    # Avoid theta extremely close to pi to reduce ambiguity unless you want that case
    margin = 1e-3

    for k in range(N):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)

        theta = (rng.random() * 2.0 - 1.0) * (np.pi - margin)  # (-pi+margin, pi-margin)

        R = exp_so3(axis, theta)
        axis_hat, theta_hat = RToAxisAngle(R)
        R2 = AxisAngleToR(axis_hat, theta_hat)

        err = np.linalg.norm(R - R2, ord="fro")
        assert err < 1e-6, f"k={k}, err={err}"
