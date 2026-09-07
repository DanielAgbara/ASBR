import numpy as np
import pytest

from THA1.rotation import AxisAngleToR


def is_SO3(R, atol=1e-6):
    R = np.asarray(R, dtype=float)
    return np.allclose(R.T @ R, np.eye(3), atol=atol) and np.allclose(
        np.linalg.det(R), 1.0, atol=atol
    )


@pytest.mark.parametrize(
    "axis,theta",
    [
        ([1, 0, 0], 0.0),
        ([0, 1, 0], np.pi / 6),
        ([0, 0, 1], -np.pi / 2),
        ([1, 2, 3], np.pi / 3),
        ([1, -1, 0.5], np.pi),
    ],
)
def test_AxisAngleToR_fixed(axis, theta):
    R = AxisAngleToR(axis, theta)
    assert R.shape == (3, 3)
    assert np.all(np.isfinite(R))
    assert is_SO3(R)


def test_AxisAngleToR_random():
    rng = np.random.default_rng(42)
    N = 300

    for k in range(N):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        theta = (rng.random() * 2.0 - 1.0) * np.pi  # [-pi, pi]

        R = AxisAngleToR(axis, theta)
        assert is_SO3(R), f"Not SO(3) at k={k}"
