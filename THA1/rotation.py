"""Rotation conversions: radians and scalar-first quaternions [w, x, y, z]."""

import numpy as np
import warnings


def RToAxisAngle(R, eps=1e-6):
    R = np.asarray(R, dtype=float)
    tr_R = np.trace(R)
    I = np.eye(3)
    # Case I: R = I
    if np.linalg.norm(R - I) < eps:
        warnings.warn("Rotation axis is undefined", UserWarning)
        return np.array([0.0, 0.0, 1.0]), 0.0

    # Case II: tr(R) = -1
    if abs(np.trace(R) + 1) < eps:
        diag = 1 + np.diag(R)
        i = int(np.argmax(diag))

        theta = np.pi

        w_hat = R[:, i].copy()
        w_hat[i] += 1.0
        w_hat = w_hat / np.sqrt(2 * (1 + R[i, i]))

    else:  # Case III: Otherwise
        theta = np.arccos(np.clip((tr_R - 1) / 2, -1.0, 1.0))

        W = (R - np.transpose(R)) / (2 * np.sin(theta))
        w_hat = np.array([W[2, 1], W[0, 2], W[1, 0]])

    w_hat /= np.linalg.norm(w_hat)
    return w_hat, theta


def RToQuaternion(R, eps=1e-6):
    R = np.asarray(R, dtype=float)

    if R.shape != (3, 3) or not np.all(np.isfinite(R)):
        raise ValueError("R must be a finite 3x3 rotation matrix.")
    if not np.allclose(R.T @ R, np.eye(3), atol=eps, rtol=0) or not np.isclose(np.linalg.det(R), 1, atol=eps, rtol=0):
        raise ValueError("R must be orthogonal with determinant +1.")
    # Select a large component; sign-based formulas fail at exact half-turns.
    if np.trace(R) > 0:
        scale = 2 * np.sqrt(1 + np.trace(R))
        q = np.array([scale / 4, (R[2, 1]-R[1, 2])/scale,
                      (R[0, 2]-R[2, 0])/scale, (R[1, 0]-R[0, 1])/scale])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        scale = 2 * np.sqrt(max(0.0, 1 + R[i, i] - R[j, j] - R[k, k]))
        q = np.zeros(4)
        q[0] = (R[k, j] - R[j, k]) / scale
        q[i + 1] = scale / 4
        q[j + 1] = (R[j, i] + R[i, j]) / scale
        q[k + 1] = (R[k, i] + R[i, k]) / scale
    q /= np.linalg.norm(q)
    return -q if q[0] < 0 else q


def RToZYZ(R, theta_range=0, eps=1e-6):
    # 0 for theta \in (0, \pi), 1 for theta \in (-pi, 0)
    R = np.asarray(R, dtype=float)
    I = np.eye(3)

    if np.linalg.norm(R - I) < eps:
        warnings.warn("Identity Rotation is detected", UserWarning)
        return np.array([0.0, 0.0, 0.0])

    r11, r12, r13 = R[0, :]
    r21, r22, r23 = R[1, :]
    r31, r32, r33 = R[2, :]

    s = np.sqrt(r13 * r13 + r23 * r23)

    if s < eps:
        theta = 0.0 if r33 > 0 else np.pi
        raise ValueError(
            f"Singularity detected: sin(theta) ≈ 0 (theta ≈ {theta}). "
            "ZYZ Euler angles are not uniquely defined."
        )

    if not theta_range:
        phi = np.atan2(r23, r13)
        theta = np.atan2(s, r33)
        psi = np.atan2(r32, -r31)
    else:
        phi = np.atan2(-r23, -r13)
        theta = np.atan2(-s, r33)
        psi = np.atan2(-r32, r31)

    return np.array([phi, theta, psi])


def RToZYX(R, theta_range=0, eps=1e-6):
    # 0 for theta \in (-\pi/2, \pi/2), 1 for theta \in (\pi/2, 3\pi/2)
    R = np.asarray(R, dtype=float)
    r11, r12, r13 = R[0, :]
    r21, r22, r23 = R[1, :]
    r31, r32, r33 = R[2, :]

    c = np.sqrt(r32 * r32 + r33 * r33)

    if c < eps:
        theta = np.pi / 2 if r31 < 0 else -np.pi / 2
        raise ValueError(
            f"Singularity detected: cos(theta) ≈ 0 (theta ≈ {theta}). "
            "ZYX Euler angles are not uniquely defined."
        )

    if not theta_range:
        # -pi/2 < theta < pi/2
        phi = np.atan2(r21, r11)
        theta = np.atan2(-r31, c)
        psi = np.atan2(r32, r33)
    else:
        # pi/2 < theta < 3pi/2
        phi = np.atan2(-r21, -r11)
        theta = np.atan2(-r31, -c)
        psi = np.atan2(-r32, -r33)

    return np.array([phi, theta, psi])


def AxisAngleToR(w_hat, theta, eps=1e-6):
    w_hat = np.asarray(w_hat, dtype=float)
    w_hat = w_hat / np.linalg.norm(w_hat)
    x1, x2, x3 = w_hat
    W = np.array([[0.0, -x3, x2], [x3, 0.0, -x1], [-x2, x1, 0.0]])
    R = np.eye(3) + W * np.sin(theta) + (W @ W) * (1 - np.cos(theta))
    return R


def QuaternionToR(Q):
    """Normalize a nonzero scalar-first quaternion and return its rotation."""
    Q = np.asarray(Q, dtype=float)
    if Q.shape != (4,) or not np.all(np.isfinite(Q)) or np.linalg.norm(Q) == 0:
        raise ValueError("Quaternion must be a finite nonzero 4-vector.")
    Q = Q / np.linalg.norm(Q)
    q0, q1, q2, q3 = Q
    R = np.array(
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
        ]
    )

    return R
