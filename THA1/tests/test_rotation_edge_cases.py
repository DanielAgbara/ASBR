"""Regressions for exact half-turns and reusable quaternion inputs."""
import numpy as np
import pytest
from THA1.rotation import RToQuaternion, QuaternionToR


@pytest.mark.parametrize("axis", [[1,0,0], [0,1,0], [0,0,1], [1,-1,0]])
def test_exact_symmetric_half_turn(axis):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    rotation = 2 * np.outer(axis, axis) - np.eye(3)
    np.testing.assert_allclose(QuaternionToR(RToQuaternion(rotation)), rotation, atol=1e-12)


def test_nonunit_quaternion_is_normalized():
    np.testing.assert_allclose(QuaternionToR([2,0,0,0]), np.eye(3))


@pytest.mark.parametrize("q", [[0,0,0,0], [1,2,3], [np.nan,0,0,0]])
def test_invalid_quaternion(q):
    with pytest.raises(ValueError):
        QuaternionToR(q)
