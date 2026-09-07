import numpy as np
import pytest

from THA1.rotation import RToQuaternion, QuaternionToR


def hat(w):
    wx, wy, wz = w
    return np.array([[0.0, -wz, wy], [wz, 0.0, -wx], [-wy, wx, 0.0]], dtype=float)


def exp_so3(axis, theta):
    """Rodrigues formula"""
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
        ([1, 0, 0], 0.0),
        ([0, 0, 1], np.pi / 2),
        ([1, 2, 3], np.pi / 3),
        ([1, -1, 2], -1.7),
        # ([0.2, 0.3, 0.9], np.pi),  # pi-case included
    ],
)
def test_R_to_q_to_R_fixed(axis, theta):
    R = exp_so3(axis, theta)
    assert is_SO3(R)

    q = RToQuaternion(R)
    assert q.shape == (4,)
    assert np.all(np.isfinite(q))
    assert abs(np.linalg.norm(q) - 1.0) < 1e-6

    R2 = QuaternionToR(q)
    err = np.linalg.norm(R - R2, ord="fro")
    assert err < 1e-6


def test_R_to_q_to_R_random():
    rng = np.random.default_rng(7)
    N = 500
    margin = 1e-4  # avoid extremely ill-conditioned edge; you can set 0 if you want

    for k in range(N):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)

        theta = (rng.random() * 2.0 - 1.0) * (np.pi - margin)

        R = exp_so3(axis, theta)
        q = RToQuaternion(R)
        R2 = QuaternionToR(q)

        err = np.linalg.norm(R - R2, ord="fro")
        assert err < 1e-6, f"k={k}, err={err}"
