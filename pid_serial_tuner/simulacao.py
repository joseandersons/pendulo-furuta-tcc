from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete, place_poles
from scipy.linalg import solve_discrete_are


COMPRIMENTO_HASTE_PENDULAR_M = 0.4086542
COMPRIMENTO_BRACO_M = 0.12495
GRAVIDADE = 9.81

USAR_CENTRO_MASSA_COMO_METADE_DA_HASTE = True
DISTANCIA_CENTRO_MASSA_M = 0.2043271

Ts = 0.002
TEMPO_FINAL_S = 120.0
TELEMETRIA_DT_S = 0.020

ALPHA_INICIAL_GRAUS = 5.0
THETA_INICIAL_GRAUS = 0.0

RUIDO_ALPHA_SENSOR_GRAUS = 0.03
RUIDO_THETA_SENSOR_GRAUS = 0.03

CPR_PENDULO = 1200.0
CPR_BRACO_AS5600 = 4096.0

ALPHA_DOT_WINDOW = 10
THETA_DOT_WINDOW = 10

ACCEL_MAX_RAD_S2 = 80.0
VEL_MAX_RAD_S = 1.5

POLOS_OBSERVADOR_CONTINUO = np.array([-25, -30, -35, -40], dtype=float)

Q = np.diag([5, 300, 1, 20])
R = np.array([[1.0]])

SEMENTE_RUIDO = 10

GERAR_CSV = True
GERAR_GRAFICO_SAIDA_PENDULO = True

INTERVALO_GRAFICO_INI_S = 0.0
INTERVALO_GRAFICO_FIM_S = 10.0

DATA_DIR = Path("dados_simulados")


def wrap_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def rad2deg(x: float | np.ndarray) -> float | np.ndarray:
    return np.asarray(x) * 180.0 / math.pi


def deg2rad(x: float) -> float:
    return x * math.pi / 180.0


def quantizar_angulo(angle_rad: float, cpr: float, wrap: bool) -> float:
    if wrap:
        angle_rad = wrap_pi(angle_rad)

    counts = int(round(angle_rad * cpr / (2.0 * math.pi)))
    angle_q = counts * (2.0 * math.pi) / cpr

    if wrap:
        angle_q = wrap_pi(angle_q)

    return float(angle_q)


def montar_modelo():
    if USAR_CENTRO_MASSA_COMO_METADE_DA_HASTE:
        L_cm = COMPRIMENTO_HASTE_PENDULAR_M / 2.0
    else:
        L_cm = DISTANCIA_CENTRO_MASSA_M

    r = COMPRIMENTO_BRACO_M

    A = np.array([
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        [0, 0, 0, 0],
        [0, 3 * GRAVIDADE / (4 * L_cm), 0, 0],
    ], dtype=float)

    B = np.array([
        [0],
        [0],
        [1],
        [3 * r / (4 * L_cm)],
    ], dtype=float)

    C = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
    ], dtype=float)

    return A, B, C, L_cm


A, B, C, L_cm = montar_modelo()

Ad, Bd, _, _, _ = cont2discrete(
    (A, B, np.eye(4), np.zeros((4, 1))),
    Ts,
    method="zoh",
)

Pd = solve_discrete_are(Ad, Bd, Q, R)
Kd = np.linalg.inv(R + Bd.T @ Pd @ Bd) @ (Bd.T @ Pd @ Ad)

polos_observador_discreto = np.exp(POLOS_OBSERVADOR_CONTINUO * Ts)

Ld = place_poles(
    Ad.T,
    C.T,
    polos_observador_discreto,
).gain_matrix.T


class DerivadorJanela:
    def __init__(self, tamanho: int, wrap: bool):
        self.tamanho = tamanho
        self.wrap = wrap
        self.valores: list[float] = []
        self.tempos: list[float] = []

    def update(self, valor: float, tempo_s: float) -> float:
        self.valores.append(valor)
        self.tempos.append(tempo_s)

        if len(self.valores) > self.tamanho:
            self.valores.pop(0)
            self.tempos.pop(0)

        if len(self.valores) < self.tamanho:
            return 0.0

        dv = self.valores[-1] - self.valores[0]

        if self.wrap:
            dv = wrap_pi(dv)

        dt = self.tempos[-1] - self.tempos[0]

        if dt <= 0.0:
            return 0.0

        return dv / dt


def medir_sensores(x: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    theta_real = float(x[0])
    alpha_real = float(x[1])

    theta_ruidoso = theta_real + deg2rad(float(rng.normal(0.0, RUIDO_THETA_SENSOR_GRAUS)))
    alpha_ruidoso = alpha_real + deg2rad(float(rng.normal(0.0, RUIDO_ALPHA_SENSOR_GRAUS)))

    theta_meas = quantizar_angulo(theta_ruidoso, CPR_BRACO_AS5600, wrap=False)
    alpha_meas = quantizar_angulo(alpha_ruidoso, CPR_PENDULO, wrap=True)

    return np.array([theta_meas, alpha_meas], dtype=float)


def observer_update(xhat: np.ndarray, y: np.ndarray, u_prev_rad_s2: float) -> np.ndarray:
    erro = y.reshape(2, 1) - C @ xhat.reshape(4, 1)
    erro[1, 0] = wrap_pi(float(erro[1, 0]))

    xhat_next = (
        Ad @ xhat.reshape(4, 1)
        + Bd * u_prev_rad_s2
        + Ld @ erro
    ).ravel()

    xhat_next[1] = wrap_pi(float(xhat_next[1]))

    if not np.all(np.isfinite(xhat_next)):
        return np.array([y[0], y[1], 0.0, 0.0], dtype=float)

    return xhat_next


def lqr_compute_accel(state: np.ndarray) -> float:
    u = float(-(Kd @ state.reshape(4, 1))[0, 0])
    return float(np.clip(u, -ACCEL_MAX_RAD_S2, ACCEL_MAX_RAD_S2))


def aplicar_comando_velocidade(u_desejado: float, vel_cmd_anterior: float) -> tuple[float, float]:
    vel_cmd = vel_cmd_anterior + u_desejado * Ts
    vel_cmd = float(np.clip(vel_cmd, -VEL_MAX_RAD_S, VEL_MAX_RAD_S))

    u_aplicado = (vel_cmd - vel_cmd_anterior) / Ts

    return u_aplicado, vel_cmd


def simular(usar_observador: bool) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(SEMENTE_RUIDO)

    n = int(TEMPO_FINAL_S / Ts)
    telemetria_every = max(1, int(round(TELEMETRIA_DT_S / Ts)))

    x = np.array([
        deg2rad(THETA_INICIAL_GRAUS),
        deg2rad(ALPHA_INICIAL_GRAUS),
        0.0,
        0.0,
    ], dtype=float)

    y0 = medir_sensores(x, rng)

    xhat = np.array([
        y0[0],
        y0[1],
        0.0,
        0.0,
    ], dtype=float)

    deriv_alpha = DerivadorJanela(ALPHA_DOT_WINDOW, wrap=True)
    deriv_theta = DerivadorJanela(THETA_DOT_WINDOW, wrap=False)

    vel_cmd_rad_s = 0.0
    u_prev_rad_s2 = 0.0

    dados = {
        "t_s": [],
        "alpha_real_deg": [],
        "alpha_saida_deg": [],
        "theta_saida_deg": [],
        "alpha_dot_saida_deg_s": [],
        "theta_dot_saida_deg_s": [],
        "u_rad_s2": [],
        "vel_rad_s": [],
    }

    for k in range(n + 1):
        t = k * Ts

        y = medir_sensores(x, rng)

        alpha_dot_num = deriv_alpha.update(float(y[1]), t)
        theta_dot_num = deriv_theta.update(float(y[0]), t)

        if usar_observador:
            xhat = observer_update(xhat, y, u_prev_rad_s2)
            state_lqr = xhat.copy()
            state_saida = xhat.copy()
        else:
            state_lqr = np.array([
                y[0],
                y[1],
                theta_dot_num,
                alpha_dot_num,
            ], dtype=float)

            state_saida = state_lqr.copy()

        u_desejado = lqr_compute_accel(state_lqr)
        u_aplicado, vel_cmd_rad_s = aplicar_comando_velocidade(u_desejado, vel_cmd_rad_s)

        if k % telemetria_every == 0:
            dados["t_s"].append(t)
            dados["alpha_real_deg"].append(float(rad2deg(x[1])))
            dados["alpha_saida_deg"].append(float(rad2deg(state_saida[1])))
            dados["theta_saida_deg"].append(float(rad2deg(state_saida[0])))
            dados["alpha_dot_saida_deg_s"].append(float(rad2deg(state_saida[3])))
            dados["theta_dot_saida_deg_s"].append(float(rad2deg(state_saida[2])))
            dados["u_rad_s2"].append(u_aplicado)
            dados["vel_rad_s"].append(vel_cmd_rad_s)

        x = (Ad @ x.reshape(4, 1) + Bd * u_aplicado).ravel()
        x[1] = wrap_pi(float(x[1]))

        u_prev_rad_s2 = u_aplicado

    return {k: np.array(v, dtype=float) for k, v in dados.items()}


def salvar_csv(caminho: Path, dados: dict[str, np.ndarray]) -> None:
    caminho.parent.mkdir(parents=True, exist_ok=True)

    with caminho.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)

        writer.writerow([
            "t_s",
            "alpha_deg",
            "theta_deg",
            "alpha_dot_deg_s",
            "theta_dot_deg_s",
            "u_rad_s2",
            "vel_rad_s",
            "enabled",
            "k_theta",
            "k_alpha",
            "k_theta_dot",
            "k_alpha_dot",
            "accel_max_rad_s2",
        ])

        for i in range(len(dados["t_s"])):
            writer.writerow([
                dados["t_s"][i],
                dados["alpha_saida_deg"][i],
                dados["theta_saida_deg"][i],
                dados["alpha_dot_saida_deg_s"][i],
                dados["theta_dot_saida_deg_s"][i],
                dados["u_rad_s2"][i],
                dados["vel_rad_s"][i],
                1,
                float(Kd[0, 0]),
                float(Kd[0, 1]),
                float(Kd[0, 2]),
                float(Kd[0, 3]),
                ACCEL_MAX_RAD_S2,
            ])


def gerar_grafico_saida_pendulo(dados_com: dict[str, np.ndarray],
                                dados_sem: dict[str, np.ndarray],
                                caminho: Path) -> None:
    caminho.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(12, 5))

    mask_com = (
        (dados_com["t_s"] >= INTERVALO_GRAFICO_INI_S)
        & (dados_com["t_s"] <= INTERVALO_GRAFICO_FIM_S)
    )

    mask_sem = (
        (dados_sem["t_s"] >= INTERVALO_GRAFICO_INI_S)
        & (dados_sem["t_s"] <= INTERVALO_GRAFICO_FIM_S)
    )

    ax.plot(
        dados_com["t_s"][mask_com],
        dados_com["alpha_saida_deg"][mask_com],
        linewidth=1.8,
        label="saída α com observador",
    )

    ax.plot(
        dados_sem["t_s"][mask_sem],
        dados_sem["alpha_saida_deg"][mask_sem],
        linewidth=1.3,
        linestyle="--",
        label="saída α sem observador",
    )

    ax.set_xlim(INTERVALO_GRAFICO_INI_S, INTERVALO_GRAFICO_FIM_S)
    ax.set_xlabel("Tempo [s]")
    ax.set_ylabel("Ângulo do pêndulo α [graus]")
    ax.set_title("Saída simulada do pêndulo")
    ax.grid(True, alpha=0.3)
    ax.legend()

    fig.tight_layout()
    fig.savefig(caminho, dpi=220)
    plt.close(fig)


def imprimir_resumo() -> None:
    O = np.vstack([
        C,
        C @ Ad,
        C @ np.linalg.matrix_power(Ad, 2),
        C @ np.linalg.matrix_power(Ad, 3),
    ])

    print("Resumo do modelo")
    print("L_cm usado no modelo:", L_cm)
    print("Ts:", Ts)
    print("rank observabilidade:", np.linalg.matrix_rank(O))
    print("Kd:", Kd)
    print("Ld:")
    print(Ld)
    print()


def main() -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)

    imprimir_resumo()

    dados_com = simular(usar_observador=True)
    dados_sem = simular(usar_observador=False)

    if GERAR_CSV:
        csv_com = DATA_DIR / "coleta_simulada_com_observador.csv"
        csv_sem = DATA_DIR / "coleta_simulada_sem_observador.csv"

        salvar_csv(csv_com, dados_com)
        salvar_csv(csv_sem, dados_sem)

        print("CSVs gerados:")
        print(csv_com)
        print(csv_sem)

    if GERAR_GRAFICO_SAIDA_PENDULO:
        grafico = DATA_DIR / "saida_simulada_pendulo_alpha.png"

        gerar_grafico_saida_pendulo(
            dados_com=dados_com,
            dados_sem=dados_sem,
            caminho=grafico,
        )

        print()
        print("Gráfico simples gerado:")
        print(grafico)


if __name__ == "__main__":
    main()
