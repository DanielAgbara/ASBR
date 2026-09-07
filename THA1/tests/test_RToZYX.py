import numpy as np
import pytest

from THA1.rotation import RToZYX


def Rz(a):
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def Ry(b):
    cb, sb = np.cos(b), np.sin(b)
    return np.array([[cb, 0.0, sb], [0.0, 1.0, 0.0], [-sb, 0.0, cb]], dtype=float)


def Rx(c):
    cc, sc = np.cos(c), np.sin(c)
    return np.array([[1.0, 0.0, 0.0], [0.0, cc, -sc], [0.0, sc, cc]], dtype=float)


def zyx_to_R(phi, theta, psi):
    # Slide convention: R = Rz(phi) Ry(theta) Rx(psi)
    return Rz(phi) @ Ry(theta) @ Rx(psi)


def is_SO3(R, atol=1e-6):
    return np.allclose(R.T @ R, np.eye(3), atol=atol) and np.allclose(
        np.linalg.det(R), 1.0, atol=atol
    )


@pytest.mark.parametrize(
    "phi,theta,psi,theta_range",
    [
        (0.3, 0.7, -1.2, 0),
        (-2.0, 1.0, 2.2, 0),
        (1.5, 0.4, -0.4, 0),
        # theta_range=1 케이스는 theta를 (pi/2, 3pi/2) 쪽으로 뽑아줌
        (0.2, 2.2, 0.5, 1),
        (-1.1, 2.8, -2.0, 1),
    ],
)
def test_RToZYX_fixed_general(phi, theta, psi, theta_range):
    # avoid singularity: theta not near +/- pi/2 (mod pi)
    # 여기서는 그냥 입력을 그렇게 골랐다고 가정
    R = zyx_to_R(phi, theta, psi)
    assert is_SO3(R)

    phi_hat, theta_hat, psi_hat = RToZYX(R, theta_range=theta_range)
    R2 = zyx_to_R(phi_hat, theta_hat, psi_hat)

    err = np.linalg.norm(R - R2, ord="fro")
    assert err < 1e-6


def test_RToZYX_random_general_theta_range0():
    rng = np.random.default_rng(42)
    N = 300
    margin = 5e-2  # radians, avoid singularity neighborhood

    for k in range(N):
        phi = (rng.random() * 2 - 1) * np.pi
        psi = (rng.random() * 2 - 1) * np.pi

        # theta_range 0: theta in (-pi/2, pi/2), avoid edges
        theta = (rng.random() * 2 - 1) * (np.pi / 2 - margin)

        R = zyx_to_R(phi, theta, psi)
        assert is_SO3(R)

        phi_hat, theta_hat, psi_hat = RToZYX(R, theta_range=0)
        R2 = zyx_to_R(phi_hat, theta_hat, psi_hat)

        err = np.linalg.norm(R - R2, ord="fro")
        assert err < 1e-6, f"k={k}, err={err}"


def test_RToZYX_random_general_theta_range1():
    rng = np.random.default_rng(7)
    N = 300
    margin = 5e-2

    for k in range(N):
        phi = (rng.random() * 2 - 1) * np.pi
        psi = (rng.random() * 2 - 1) * np.pi

        # theta_range 1: theta in (pi/2, 3pi/2), avoid edges
        theta = (np.pi / 2 + margin) + rng.random() * (np.pi - 2 * margin)

        R = zyx_to_R(phi, theta, psi)
        assert is_SO3(R)

        phi_hat, theta_hat, psi_hat = RToZYX(R, theta_range=1)
        R2 = zyx_to_R(phi_hat, theta_hat, psi_hat)

        err = np.linalg.norm(R - R2, ord="fro")
        assert err < 1e-6, f"k={k}, err={err}"
