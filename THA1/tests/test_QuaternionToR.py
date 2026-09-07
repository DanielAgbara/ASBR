import numpy as np
import pytest

from THA1.rotation import QuaternionToR


def is_SO3(R, atol=1e-6):
    R = np.asarray(R, dtype=float)
    return np.allclose(R.T @ R, np.eye(3), atol=atol) and np.allclose(
        np.linalg.det(R), 1.0, atol=atol
    )


@pytest.mark.parametrize(
    "q",
    [
        [1.0, 0.0, 0.0, 0.0],  # identity
        [0.0, 1.0, 0.0, 0.0],  # 180 deg about x (if unit)
        [0.0, 0.0, 1.0, 0.0],  # 180 deg about y
        [0.0, 0.0, 0.0, 1.0],  # 180 deg about z
        [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)],  # 90 deg about z
    ],
)
def test_QuaternionToR_fixed(q):
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)

    R = QuaternionToR(q)
    assert R.shape == (3, 3)
    assert np.all(np.isfinite(R))
    assert is_SO3(R)


def test_QuaternionToR_random():
    rng = np.random.default_rng(42)
    N = 500

    for k in range(N):
        q = rng.normal(size=4)
        q = q / np.linalg.norm(q)

        R = QuaternionToR(q)
        assert is_SO3(R), f"Not SO(3) at k={k}"
