import numpy as np
import pytest

from THA1.rotation import RToZYZ


def Rz(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def Ry(b):
    cb, sb = np.cos(b), np.sin(b)
    return np.array([[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]], dtype=float)


def zyz_to_R(phi, theta, psi):
    # ZYZ convention: R = Rz(phi) * Ry(theta) * Rz(psi)
    return Rz(phi) @ Ry(theta) @ Rz(psi)


def is_SO3(R, atol=1e-6):
    return np.allclose(R.T @ R, np.eye(3), atol=atol) and np.allclose(
        np.linalg.det(R), 1.0, atol=atol
    )


@pytest.mark.parametrize(
    "phi,theta,psi",
    [
        (0.3, 0.7, -1.2),
        (-2.0, 1.0, 2.2),
        (1.5, 2.0, -0.4),
        (-0.9, 0.4, 0.8),
    ],
)
def test_RToZYZ_fixed_general(phi, theta, psi):
    # enforce general case: 0 < theta < pi
    assert 1e-3 < theta < np.pi - 1e-3

    R = zyz_to_R(phi, theta, psi)
    assert is_SO3(R)

    phi_hat, theta_hat, psi_hat = RToZYZ(R)

    R2 = zyz_to_R(phi_hat, theta_hat, psi_hat)
    err = np.linalg.norm(R - R2, ord="fro")

    assert err < 1e-6


def test_RToZYZ_random_general():
    rng = np.random.default_rng(42)
    N = 300

    for k in range(N):
        # random angles
        phi = (rng.random() * 2.0 - 1.0) * np.pi
        psi = (rng.random() * 2.0 - 1.0) * np.pi

        # general case: theta strictly inside (0, pi)
        theta = rng.random() * (np.pi - 2e-2) + 1e-2  # in [1e-2, pi-1e-2]

        R = zyz_to_R(phi, theta, psi)
        assert is_SO3(R)

        phi_hat, theta_hat, psi_hat = RToZYZ(R)

        # Check we're still in general case (numerically)
        assert 0.0 < theta_hat < np.pi

        R2 = zyz_to_R(phi_hat, theta_hat, psi_hat)
        err = np.linalg.norm(R - R2, ord="fro")

        assert err < 1e-6, f"k={k}, err={err}"
