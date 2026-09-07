"""Physical invariants of the screw exponential and translation recovery."""
import numpy as np
from THA1.pa3 import get_twist, screw_to_T, T_to_screw


def test_translation_recovery_round_trip():
    transform = np.eye(4)
    transform[:3,3] = [1,-2,3]
    parameters, distance = T_to_screw(transform)
    np.testing.assert_allclose(screw_to_T(get_twist(*parameters),distance), transform,atol=1e-12)


def test_scaled_twist_equals_scaled_motion_parameter():
    twist = get_twist([1,0,0],[0,0,1],0.2)
    np.testing.assert_allclose(screw_to_T(2*twist,0.3),screw_to_T(twist,0.6),atol=1e-12)


def test_screw_composition():
    twist = get_twist([1,2,3],[2,1,-1],0.4)
    half = screw_to_T(twist,0.5)
    np.testing.assert_allclose(half@half,screw_to_T(twist,1.0),atol=1e-12)
