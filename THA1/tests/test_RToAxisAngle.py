import numpy as np
import pytest

from THA1.rotation import RToAxisAngle


def hat(w):
    """w in R^3 -> w^ in so(3)"""
    wx, wy, wz = w
    return np.array([[0.0, -wz, wy], [wz, 0.0, -wx], [-wy, wx, 0.0]], dtype=float)


def exp_so3(axis, theta):
    """Axis-angle -> R in SO(3) using Rodrigues' formula"""
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    axis = axis / n

    W = hat(axis)
    I = np.eye(3)
    return I + np.sin(theta) * W + (1.0 - np.cos(theta)) * (W @ W)


def is_SO3(R, atol=1e-6):
    """Check if R is approximately in SO(3)"""
    R = np.asarray(R, dtype=float)
    return np.allclose(R.T @ R, np.eye(3), atol=atol) and np.allclose(
        np.linalg.det(R), 1.0, atol=atol
    )


def recon_error_fro(R, axis, theta):
    """Frobenius norm of reconstruction error"""
    R_rec = exp_so3(axis, theta)
    return np.linalg.norm(R - R_rec, ord="fro")


@pytest.mark.parametrize(
    "axis,theta,name",
    [
        ([0, 0, 1], 0.0, "identity-via-0"),
        ([0, 0, 1], np.pi / 2, "z-90deg"),
        ([1, 0, 0], np.pi / 4, "x-45deg"),
        ([1, 0, 0], np.pi, "x-180deg"),
        ([1, 2, 3], np.pi, "arb-180deg"),
        ([1, 2, 3], np.pi - 1e-7, "arb-near-pi"),
        ([1, 2, 3], 1e-7, "arb-near-0"),
    ],
)
def test_fixed_cases(axis, theta, name):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)

    R = exp_so3(axis, theta)
    assert is_SO3(R), f"Generated R not in SO(3) for case {name}"

    axis_hat, theta_hat = RToAxisAngle(R)

    # Basic sanity
    axis_hat = np.asarray(axis_hat, dtype=float)
    assert axis_hat.shape == (3,), "axis must be shape (3,)"
    assert np.isfinite(theta_hat), "theta must be finite"
    assert np.all(np.isfinite(axis_hat)), "axis must be finite"

    # axis should be unit unless theta ~ 0
    if abs(theta_hat) > 1e-12:
        assert abs(np.linalg.norm(axis_hat) - 1.0) < 1e-6, "axis not unit"

    # Validate by reconstruction (NOT by comparing axis/theta directly)
    err = recon_error_fro(R, axis_hat, theta_hat)
    assert err < 1e-5, f"{name}: reconstruction error too large: {err}"


def test_random_cases():
    rng = np.random.default_rng(42)
    N = 200

    for k in range(N):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        theta = rng.random() * np.pi  # [0, pi]

        R = exp_so3(axis, theta)
        assert is_SO3(R), f"Generated R not in SO(3) at k={k}"

        axis_hat, theta_hat = RToAxisAngle(R)

        err = recon_error_fro(R, axis_hat, theta_hat)
        assert err < 1e-5, f"Random k={k}: reconstruction error too large: {err}"


def test_axis_angle_output_range():
    # Optional: enforce theta in [0, pi]
    rng = np.random.default_rng(0)
    axis = rng.normal(size=3)
    axis /= np.linalg.norm(axis)

    R = exp_so3(axis, 2.3)  # some angle < pi
    axis_hat, theta_hat = RToAxisAngle(R)

    assert 0.0 <= theta_hat <= np.pi + 1e-6, "theta not in [0, pi] (within tolerance)"
