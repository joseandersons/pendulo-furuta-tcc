from dataclasses import dataclass
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
    0.00000200,
    0.0000009173,
    0.00200000,
    0.0009173012,
], dtype=float)


@dataclass
class ObserverMatrices:
    p1: float
    p2: float
    Lr: np.ndarray
    F: np.ndarray
    G: np.ndarray
    H: np.ndarray

    @property
    def label(self) -> str:
        return f"p=({self.p1:.3f},{self.p2:.3f})"

    def cmd_olr(self) -> str:
        return (
            f"OLR {self.Lr[0, 0]:.8f} {self.Lr[0, 1]:.8f} "
            f"{self.Lr[1, 0]:.8f} {self.Lr[1, 1]:.8f}"
        )

    def cmd_of(self) -> str:
        return (
            f"OF {self.F[0, 0]:.8f} {self.F[0, 1]:.8f} "
            f"{self.F[1, 0]:.8f} {self.F[1, 1]:.8f}"
        )

    def cmd_og(self) -> str:
        return (
            f"OG {self.G[0, 0]:.8f} {self.G[0, 1]:.8f} "
            f"{self.G[1, 0]:.8f} {self.G[1, 1]:.8f}"
        )

    def cmd_oh(self) -> str:
        return f"OH {self.H[0]:.8f} {self.H[1]:.8f}"


def observer_matrices_from_poles(p1: float, p2: float) -> ObserverMatrices:
    if not math.isfinite(p1) or not math.isfinite(p2):
        raise ValueError("Polos inválidos.")

    if p1 >= 0.0 or p2 >= 0.0:
        raise ValueError("Os polos do observador devem ser negativos.")

    A11 = AD[0:2, 0:2]
    A12 = AD[0:2, 2:4]
    A21 = AD[2:4, 0:2]
    A22 = AD[2:4, 2:4]

    B1 = BD[0:2]
    B2 = BD[2:4]

    if abs(A12[0, 0]) < 1e-12 or abs(A12[1, 1]) < 1e-12:
        raise ValueError("A matriz A12 não permite o cálculo direto de Lr.")

    z = np.exp(np.array([p1, p2], dtype=float) * TS)

    Lr = np.zeros((2, 2), dtype=float)

    Lr[0, 0] = (A22[0, 0] - z[0]) / A12[0, 0]
    Lr[1, 1] = (A22[1, 1] - z[1]) / A12[1, 1]

    F = A22 - Lr @ A12
    G = A21 + A22 @ Lr - Lr @ A11 - Lr @ A12 @ Lr
    H = B2 - Lr @ B1

    return ObserverMatrices(
        p1=float(p1),
        p2=float(p2),
        Lr=Lr,
        F=F,
        G=G,
        H=H,
    )
