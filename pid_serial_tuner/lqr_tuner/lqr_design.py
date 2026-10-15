import math
import numpy as np

TS = 0.002

AD = np.array([
    [1.0, 0.0, 0.002, 0.0],
    [0.0, 1.00007202, 0.0, 0.00200005],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.07201861, 0.0, 1.00007202],
], dtype=float)

BD = np.array([
    [0.00000200],
    [0.0000009173],
    [0.00200000],
    [0.0009173012],
], dtype=float)


def _solve_dare_iterative(Ad, Bd, Q, R, max_iter=10000, tol=1e-11):
    P = Q.copy()

    for _ in range(max_iter):
        S = R + Bd.T @ P @ Bd
        K = np.linalg.solve(S, Bd.T @ P @ Ad)
        P_next = Ad.T @ P @ Ad - Ad.T @ P @ Bd @ K + Q

        if np.linalg.norm(P_next - P, ord="fro") < tol:
            return P_next

        P = P_next

    raise RuntimeError("DARE não convergiu.")


def compute_lqr_gain(
    q_theta: float,
    q_alpha: float,
    q_theta_dot: float,
    q_alpha_dot: float,
    r_value: float,
):
    values = [q_theta, q_alpha, q_theta_dot, q_alpha_dot, r_value]

    if not all(math.isfinite(v) for v in values):
        raise ValueError("Q/R possuem valores inválidos.")

    if q_theta < 0 or q_alpha < 0 or q_theta_dot < 0 or q_alpha_dot < 0:
        raise ValueError("Os pesos de Q devem ser maiores ou iguais a zero.")

    if r_value <= 0:
        raise ValueError("R deve ser maior que zero.")

    Q = np.diag([q_theta, q_alpha, q_theta_dot, q_alpha_dot]).astype(float)
    R = np.array([[r_value]], dtype=float)

    try:
        from scipy.linalg import solve_discrete_are
        P = solve_discrete_are(AD, BD, Q, R)
    except Exception:
        P = _solve_dare_iterative(AD, BD, Q, R)

    K = np.linalg.solve(R + BD.T @ P @ BD, BD.T @ P @ AD)

    return K.flatten()
