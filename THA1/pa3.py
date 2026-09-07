import numpy as np
import matplotlib.pyplot as plt
if __package__:
    from . import rotation as rot
else:
    import rotation as rot

# ============================================================
# Basic Lie algebra helpers (so(3), se(3))
# ============================================================

def skew(w: np.ndarray) -> np.ndarray:
    """
    Convert a 3-vector w into a 3x3 skew-symmetric matrix [w]x such that:
        [w]x v = w x v
    """
    w1, w2, w3 = w
    return np.array([[0,  -w3,  w2],
                     [w3,   0,  -w1],
                     [-w2,  w1,  0]], dtype=float)


def unskew(W: np.ndarray) -> np.ndarray:
    """
    Inverse of skew(): convert a 3x3 skew-symmetric matrix into the 3-vector.
    """
    W = np.asarray(W, dtype=float)
    if W.shape != (3, 3):
        raise ValueError("Input must be a 3x3 matrix.")
    if not np.allclose(W + W.T, 0):
        raise ValueError("Matrix is not skew-symmetric.")
    return np.array([W[2, 1], W[0, 2], W[1, 0]], dtype=float)


def normalize(v: np.ndarray, eps: float = 1e-12):
    """
    Return (v_hat, ||v||). If ||v|| is tiny, returns (v, 0).
    """
    n = np.linalg.norm(v)
    if n < eps:
        return v, 0.0
    return v / n, n


# ============================================================
# Screw theory: twist <-> transform
# ============================================================

def get_angles(theta: float) -> np.ndarray:
    """Convenience angles: theta/4, theta/2, 3theta/4, theta."""
    return np.array([theta/4, theta/2, 3*theta/4, theta], dtype=float)


def get_twist(q, s, h) -> np.ndarray:
    """
    Build a 6x1 twist S = [w; v] from screw axis parameters {q, s, h}.

    q: point on axis in R^3
    s: axis direction (will be normalized)
    h: pitch

    For unit w = s_hat:
      v = -w x q + h*w
    """
    q = np.asarray(q, dtype=float).reshape(3,)
    s = np.asarray(s, dtype=float).reshape(3,)

    w, wn = normalize(s)
    if wn == 0:
        raise ValueError("s must be non-zero.")
    # Infinite pitch denotes pure translation; theta is then travel distance.
    if np.isinf(h):
        return np.hstack([np.zeros(3), w]).reshape(6, 1)
    v = -np.cross(w, q) + h * w
    return np.hstack([w, v]).reshape(6, 1)


def screw_to_T(twist: np.ndarray, theta: float, eps: float = 1e-12) -> np.ndarray:
    """
    Exponential map: T = exp([S]*theta) in SE(3).

    twist: 6x1 [w; v]
      - if ||w|| ~ 0 => pure translation
      - else => rotation + translation
    """
    w = twist[0:3, 0]
    v = twist[3:6, 0]

    w_hat, w_norm = normalize(w, eps=eps)

    if w_norm < eps:
        R = np.eye(3)
        p = v * theta
    else:
        W = skew(w_hat)
        angle = w_norm * theta
        R = np.eye(3) + np.sin(angle) * W + (1 - np.cos(angle)) * (W @ W)
        G = (np.eye(3) * angle) + (1 - np.cos(angle)) * W + (angle - np.sin(angle)) * (W @ W)
        p = G @ (v / w_norm)

    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T


def T_to_screw(T: np.ndarray, eps: float = 1e-9):
    """
    Given T in SE(3), recover screw parameters {q, s, h} and theta.

    Returns:
      (q, s, h), theta

    Note: for theta ~ 0, the screw axis is not unique (near pure translation).
    """
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4):
        raise ValueError("T must be 4x4.")

    R = T[0:3, 0:3]
    p = T[0:3, 3]

    s, theta = rot.RToAxisAngle(R)  # s is unit axis from your rotation.py

    # Near-identity rotation case (translation / tiny rotation)
    if abs(theta) < eps:
        pnorm = np.linalg.norm(p)
        if pnorm < eps:
            return (np.zeros(3), np.array([0.0, 0.0, 1.0]), 0.0), 0.0

        s = p / pnorm
        q = np.zeros(3)
        h = np.inf
        return (q, s, h), pnorm

    # Use the inverse of the left Jacobian (G_inv) to recover v from p
    W = skew(s)
    cot_half = 1.0 / np.tan(theta / 2.0)
    G_inv = (1.0/theta) * np.eye(3) - 0.5 * W + (1.0/theta - 0.5 * cot_half) * (W @ W)

    v = G_inv @ p

    # Screw parameters
    q = np.cross(s, v) / (np.linalg.norm(s) ** 2)   # norm(s)=1 usually
    h = (s @ v) / (np.linalg.norm(s) ** 2)

    return (q, s, h), theta


def forward_configs(T_initial, q, s, h, theta):
    """
    Compute intermediate configurations:
      T(ang) = exp([S]*ang) @ T_initial
    for ang in [theta/4, theta/2, 3theta/4, theta].
    """
    twist = get_twist(q, s, h)
    configs = []
    for ang in get_angles(theta):
        configs.append(screw_to_T(twist, ang) @ T_initial)
    return configs


# ============================================================
# Plotting helpers
# ============================================================

def draw_frame(ax, origin, R=np.eye(3), L=0.5, name=None):
    """
    Draw a coordinate frame at 'origin' with orientation R.
    Columns of R are the axis directions in world coordinates.
    """
    origin = np.asarray(origin, dtype=float).reshape(3,)
    colors = ["r", "g", "b"]  # x,y,z

    for i in range(3):
        v = R[:, i] * L
        ax.quiver(origin[0], origin[1], origin[2],
                  v[0], v[1], v[2],
                  color=colors[i], linewidth=2, arrow_length_ratio=0.15)

    if name:
        ax.text(origin[0], origin[1], origin[2], name)


def set_equal_3d(ax, pts: np.ndarray):
    """
    Set 3D limits so x/y/z appear with roughly equal scale.
    pts: (N,3) array of points that must be visible.
    """
    pts = np.asarray(pts, dtype=float)
    X, Y, Z = pts[:, 0], pts[:, 1], pts[:, 2]

    xmid = 0.5 * (np.min(X) + np.max(X))
    ymid = 0.5 * (np.min(Y) + np.max(Y))
    zmid = 0.5 * (np.min(Z) + np.max(Z))

    r = max(np.max(X) - np.min(X),
            np.max(Y) - np.min(Y),
            np.max(Z) - np.min(Z)) * 0.55
    if r == 0:
        r = 1.0

    ax.set_xlim(xmid - r, xmid + r)
    ax.set_ylim(ymid - r, ymid + r)
    ax.set_zlim(zmid - r, zmid + r)


def screw_axis_line_points(q, s, span=5.0, n=200):
    """
    Points on the screw axis line: q + alpha*s for alpha in [-span, span].
    """
    q = np.asarray(q, dtype=float).reshape(3,)
    s = np.asarray(s, dtype=float).reshape(3,)
    s, sn = normalize(s)
    if sn == 0:
        raise ValueError("s must be non-zero.")

    alphas = np.linspace(-span, span, n)
    return q[None, :] + alphas[:, None] * s[None, :]

def screw_trajectory_points(S, theta, p0_world, n=200):
    """
    Trajectory of a world point p0_world under the screw motion exp([S] t),
    for t in [0, theta].

    Returns: (n,3) array of points.
    """
    q, s, h = S
    q = np.asarray(q, dtype=float).reshape(3,)
    s = np.asarray(s, dtype=float).reshape(3,)
    p0_world = np.asarray(p0_world, dtype=float).reshape(3,)

    # Ensure s is unit
    s, sn = normalize(s)
    if sn == 0:
        raise ValueError("s must be non-zero.")

    twist = get_twist(q, s, h)

    ts = np.linspace(0.0, theta, n)
    p0_h = np.hstack([p0_world, 1.0])

    traj = []
    for t in ts:
        Tt = screw_to_T(twist, t)
        pt = (Tt @ p0_h)[0:3]
        traj.append(pt)

    return np.array(traj)


# ============================================================
# Input helpers
# ============================================================

def read_matrix_4x4():
    print("Enter initial configuration T_initial (4 rows, 4 numbers each):")
    rows = []
    for i in range(4):
        row = input(f"Row {i+1}: ").strip().split()
        if len(row) != 4:
            raise ValueError("Each row must have exactly 4 numbers.")
        rows.append([float(x) for x in row])
    T = np.array(rows, dtype=float)
    if T.shape != (4, 4):
        raise ValueError("T_initial must be 4x4.")
    return T


def read_vec3(name):
    vals = input(f"Enter {name} (3 numbers): ").strip().split()
    if len(vals) != 3:
        raise ValueError(f"{name} must have 3 numbers.")
    return np.array([float(x) for x in vals], dtype=float)


# ============================================================
# Main
# ============================================================

def main():
    # ----- user input -----
    T_initial = read_matrix_4x4()
    q = read_vec3("q")
    s = read_vec3("s")
    h = float(input("Enter h (pitch): ").strip())
    theta = float(input("Enter theta (radians): ").strip())

    # ----- forward motion configs -----
    configs = forward_configs(T_initial, q, s, h, theta)
    T_final = configs[-1]

    # ------------------------------------------------------------
    # Print Part 1: Intermediate + Final Configurations
    # ------------------------------------------------------------
    print("\n================ Part 1: Configurations ================")

    for i, T in enumerate(configs, start=1):
        print(f"\nT{i} =")
        print(np.round(T, 6))

    print("\nFinal Configuration T_final =")
    print(np.round(T_final, 6))

    # ========================================================
    # Plot 1: World + initial + intermediate configurations
    # ========================================================
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, projection="3d")

    draw_frame(ax1, origin=[0, 0, 0], R=np.eye(3), L=0.7, name="World")

    R0 = T_initial[0:3, 0:3]
    p0 = T_initial[0:3, 3]
    draw_frame(ax1, origin=p0, R=R0, L=0.5, name="T0")

    points1 = [np.zeros(3), p0]
    for k, T in enumerate(configs, start=1):
        Rk = T[0:3, 0:3]
        pk = T[0:3, 3]
        points1.append(pk)
        draw_frame(ax1, origin=pk, R=Rk, L=0.4, name=f"T{k}")

    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.set_zlabel("Z")
    ax1.set_title("Rigid Body Screw Motion")
    set_equal_3d(ax1, np.array(points1))

    # ========================================================
    # Plot 2: Screw axis for motion from Final -> World
    # ========================================================
    T_rel = np.linalg.inv(T_final)          # final -> world
    (S_rel, theta_rel) = T_to_screw(T_rel)  # S_rel = (q_rel, s_rel, h_rel)
    q_rel, s_rel, h_rel = S_rel

    # ------------------------------------------------------------
    # Print Part 2: Screw Parameters
    # ------------------------------------------------------------
    print("\n================ Part 2: Screw Parameters ================")

    print("\nq =")
    print(np.round(q_rel, 6))

    print("\ns =")
    print(np.round(s_rel, 6))

    print("\nh =")
    print(np.round(h_rel, 6))

    print("\ntheta =")
    print(np.round(theta_rel, 6))

    # Compute twist (w, v)
    twist_rel = get_twist(q_rel, s_rel, h_rel)
    w_rel = twist_rel[0:3, 0]
    v_rel = twist_rel[3:6, 0]

    print("\nw =")
    print(np.round(w_rel, 6))

    print("\nv =")
    print(np.round(v_rel, 6))

    axis_line = screw_axis_line_points(q_rel, s_rel, span=5.0, n=200)

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111, projection="3d")

    # World frame at origin
    draw_frame(ax2, origin=[0, 0, 0], R=np.eye(3), L=0.7, name="World")

    # Final frame
    Rf = T_final[0:3, 0:3]
    pf = T_final[0:3, 3]
    draw_frame(ax2, origin=pf, R=Rf, L=0.6, name="Final")

    #Trajectory Plot
    traj = screw_trajectory_points(S_rel, theta_rel, pf, n=250)
    ax2.plot(traj[:, 0], traj[:, 1], traj[:, 2])

    # Screw axis line
    ax2.plot(axis_line[:, 0], axis_line[:, 1], axis_line[:, 2])

    # Dotted lines from q to world origin and to final origin
    qpt = np.asarray(q_rel, dtype=float).reshape(3,)
    origin_world = np.zeros(3)

    ax2.plot([qpt[0], origin_world[0]],
             [qpt[1], origin_world[1]],
             [qpt[2], origin_world[2]],
             linestyle=":")

    ax2.plot([qpt[0], pf[0]],
             [qpt[1], pf[1]],
             [qpt[2], pf[2]],
             linestyle=":")

    # Label q on the axis
    ax2.text(qpt[0], qpt[1], qpt[2], "q")

    ax2.set_xlabel("X")
    ax2.set_ylabel("Y")
    ax2.set_zlabel("Z")
    ax2.set_title("Screw Axis (Final → World)")

    # Use points relevant to plot 2 for scaling
    pts2 = np.vstack([axis_line, traj, origin_world[None, :], pf[None, :], qpt[None, :]])
    set_equal_3d(ax2, pts2)

    plt.show()




if __name__ == "__main__":
    main()
