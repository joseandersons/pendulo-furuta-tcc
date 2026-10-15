#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Identificação de sistema para pêndulo rotacional usando mínimos quadrados.

Coloque este arquivo dentro da pasta `lqr_tuner`, no mesmo nível da pasta `data`.

Estrutura esperada:
lqr_tuner/
├── data/
│   └── coleta_lqr_2026....csv
└── identificacao_sistema.py

Execute:
    python identificacao_sistema.py

O script:
1. Procura automaticamente a coleta mais recente em ./data;
2. Converte graus para radianos;
3. Monta o estado:
       x = [theta_erro, alpha, theta_dot, alpha_dot]
4. Identifica:
       x[k+1] = A x[k] + B u[k]
5. Testa `u_rad_s2` e `vel_rad_s`, se existirem;
6. Salva matrizes, métricas e gráficos dentro de ./data/modelos_identificados.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, Tuple, Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ============================================================
# CONFIGURAÇÕES PRINCIPAIS
# ============================================================

DATA_DIR = Path(__file__).resolve().parent / "data"

# Deixe None para escolher pelo menu interativo.
# Exemplo manual:
# CSV_PATH = DATA_DIR / "coleta_lqr_20260621.csv"
CSV_PATH: Optional[Path] = None

# Se True, lista os arquivos de DATA_DIR e permite escolher pelo número.
# Se False e CSV_PATH=None, usa automaticamente o arquivo mais recente.
SELECT_FILE_INTERACTIVELY = True

# Entrada principal para exportar o modelo.
# Para seu firmware LQR atual, normalmente faz mais sentido usar u_rad_s2.
PREFERRED_INPUT = "u_rad_s2"

# O script também testa outras entradas, se existirem no CSV.
INPUT_COLUMNS_TO_TEST = ["u_rad_s2", "vel_rad_s"]

# Usa theta em relação à referência.
# Recomendado para LQR em torno de theta_target.
USE_THETA_ERROR = True

# Percentual usado para identificação.
# O restante é usado para validação.
TRAIN_FRAC = 0.70

# Adicionar constante c no modelo:
# x[k+1] = A x[k] + B u[k] + c
#
# Para LQR, normalmente usamos sem constante.
# Aqui a constante fica apenas para diagnóstico.
FIT_WITH_BIAS_FOR_DIAGNOSTIC = True

# Geração de gráficos.
SAVE_PLOTS = True

# Tentar converter o modelo identificado para outro Ts.
# Atenção: isso é aproximação. O melhor é coletar no mesmo Ts do controle.
TRY_EXPORT_TS_CONTROL = True
TS_CONTROL = 0.002  # 2 ms


# ============================================================
# FUNÇÕES AUXILIARES
# ============================================================

def wrap_pi(angle_rad: np.ndarray) -> np.ndarray:
    """Limita ângulo para o intervalo [-pi, pi]."""
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def list_data_files(data_dir: Path) -> list[Path]:
    """Lista arquivos de coleta dentro de data/."""
    if not data_dir.exists():
        raise FileNotFoundError(f"Pasta data não encontrada: {data_dir}")

    candidates = []
    for ext in ("*.csv", "*.txt"):
        candidates.extend(data_dir.glob(ext))

    candidates = [p for p in candidates if p.is_file()]
    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)

    if not candidates:
        raise FileNotFoundError(
            f"Nenhum arquivo .csv ou .txt encontrado em: {data_dir}"
        )

    return candidates


def find_latest_data_file(data_dir: Path) -> Path:
    """Procura a coleta mais recente dentro de data/."""
    return list_data_files(data_dir)[0]


def select_data_file(data_dir: Path) -> Path:
    """
    Permite escolher manualmente a coleta.

    Os arquivos aparecem ordenados do mais recente para o mais antigo.
    """
    files = list_data_files(data_dir)

    print("\nArquivos encontrados em data/:")
    for i, path in enumerate(files, start=1):
        size_kb = path.stat().st_size / 1024.0
        print(f"  [{i}] {path.name}  ({size_kb:.1f} KB)")

    print("\nDigite o número do arquivo que deseja usar.")
    print("Pressione ENTER para usar o mais recente.")

    while True:
        choice = input("Arquivo: ").strip()

        if choice == "":
            return files[0]

        if not choice.isdigit():
            print("Digite apenas o número do arquivo.")
            continue

        idx = int(choice)
        if 1 <= idx <= len(files):
            return files[idx - 1]

        print(f"Escolha inválida. Use um número entre 1 e {len(files)}.")


def load_data() -> pd.DataFrame:
    """Carrega a coleta."""
    if CSV_PATH is not None:
        path = CSV_PATH
    elif SELECT_FILE_INTERACTIVELY:
        path = select_data_file(DATA_DIR)
    else:
        path = find_latest_data_file(DATA_DIR)

    print("=" * 72)
    print(f"Arquivo usado: {path}")
    print("=" * 72)

    df = pd.read_csv(path)

    required = [
        "t_s",
        "alpha_deg",
        "theta_deg",
        "alpha_dot_deg_s",
        "theta_dot_deg_s",
    ]

    missing = [col for col in required if col not in df.columns]
    if missing:
        raise ValueError(f"Colunas obrigatórias ausentes no CSV: {missing}")

    # Mantém somente amostras com controle habilitado, se a coluna existir.
    if "enabled" in df.columns:
        df = df[df["enabled"] == 1].copy()

    # Ordena por tempo e remove linhas inválidas.
    df = df.sort_values("t_s").reset_index(drop=True)
    df = df.replace([np.inf, -np.inf], np.nan).dropna().reset_index(drop=True)

    if len(df) < 100:
        raise ValueError("Poucas amostras válidas para identificação.")

    return df


def build_state_and_input(df: pd.DataFrame, input_col: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Monta:
        X = [theta_erro, alpha, theta_dot, alpha_dot]
        u = entrada escolhida
        t = tempo
        Ts = período médio da coleta
    """
    if input_col not in df.columns:
        raise ValueError(f"Coluna de entrada não encontrada: {input_col}")

    t = df["t_s"].to_numpy(dtype=float)
    dt = np.diff(t)
    Ts = float(np.median(dt))

    if Ts <= 0:
        raise ValueError("Tempo inválido. Verifique a coluna t_s.")

    theta = np.deg2rad(df["theta_deg"].to_numpy(dtype=float))
    alpha = np.deg2rad(df["alpha_deg"].to_numpy(dtype=float))

    theta_dot = np.deg2rad(df["theta_dot_deg_s"].to_numpy(dtype=float))
    alpha_dot = np.deg2rad(df["alpha_dot_deg_s"].to_numpy(dtype=float))

    if USE_THETA_ERROR and "theta_target_deg" in df.columns:
        theta_target = np.deg2rad(df["theta_target_deg"].to_numpy(dtype=float))
        theta_state = wrap_pi(theta - theta_target)
    else:
        theta_state = wrap_pi(theta)

    # Se seu alpha tiver wrap em +/- pi, use wrap_pi(alpha).
    # Como sua coleta está próxima do equilíbrio, alpha direto é melhor.
    X = np.column_stack([
        theta_state,
        alpha,
        theta_dot,
        alpha_dot,
    ])

    u = df[input_col].to_numpy(dtype=float)

    return X, u, t, Ts


def least_squares_identification(
    X: np.ndarray,
    u: np.ndarray,
    train_frac: float,
    add_bias: bool = False,
) -> Dict[str, np.ndarray | float]:
    """
    Identifica:
        x[k+1] = A x[k] + B u[k]
    ou:
        x[k+1] = A x[k] + B u[k] + c
    """

    Xk = X[:-1, :]
    Y = X[1:, :]
    Uk = u[:-1].reshape(-1, 1)

    if add_bias:
        Z = np.hstack([Xk, Uk, np.ones((len(Xk), 1))])
    else:
        Z = np.hstack([Xk, Uk])

    n_train = int(train_frac * len(Z))
    if n_train < 10 or n_train >= len(Z):
        raise ValueError("TRAIN_FRAC inválido para o número de amostras.")

    Z_train = Z[:n_train, :]
    Y_train = Y[:n_train, :]

    Z_val = Z[n_train:, :]
    Y_val = Y[n_train:, :]

    # Resolve Y = Z W por mínimos quadrados.
    W, residuals, rank, singular_values = np.linalg.lstsq(Z_train, Y_train, rcond=None)

    Y_pred_train = Z_train @ W
    Y_pred_val = Z_val @ W

    err_train = Y_train - Y_pred_train
    err_val = Y_val - Y_pred_val

    rmse_train = np.sqrt(np.mean(err_train ** 2, axis=0))
    rmse_val = np.sqrt(np.mean(err_val ** 2, axis=0))

    A = W[:4, :].T
    B = W[4, :].reshape(4, 1)

    if add_bias:
        c = W[5, :].reshape(4, 1)
    else:
        c = np.zeros((4, 1))

    return {
        "A": A,
        "B": B,
        "c": c,
        "W": W,
        "rank": rank,
        "singular_values": singular_values,
        "rmse_train": rmse_train,
        "rmse_val": rmse_val,
        "Y_val": Y_val,
        "Y_pred_val": Y_pred_val,
        "n_train": n_train,
        "n_val": len(Y_val),
    }


def simulate_one_step(A: np.ndarray, B: np.ndarray, X: np.ndarray, u: np.ndarray) -> np.ndarray:
    """Predição de um passo."""
    Xk = X[:-1, :]
    Uk = u[:-1].reshape(-1, 1)
    return Xk @ A.T + Uk @ B.T


def model_score_deg(rmse_rad: np.ndarray) -> np.ndarray:
    """Converte RMSE dos estados para unidade mais fácil de ler."""
    return np.array([
        math.degrees(rmse_rad[0]),
        math.degrees(rmse_rad[1]),
        math.degrees(rmse_rad[2]),
        math.degrees(rmse_rad[3]),
    ])


def print_matrix(name: str, M: np.ndarray) -> None:
    print(f"\n{name} =")
    with np.printoptions(precision=10, suppress=False):
        print(M)


def make_cpp_array_matrix(name: str, M: np.ndarray) -> str:
    lines = [f"static const float {name}[{M.shape[0]}][{M.shape[1]}] = {{"]
    for row in M:
        values = ", ".join(f"{v:.9g}f" for v in row)
        lines.append(f"    {{ {values} }},")
    lines.append("};")
    return "\n".join(lines)


def make_cpp_array_vector(name: str, v: np.ndarray) -> str:
    v = v.reshape(-1)
    values = ", ".join(f"{x:.9g}f" for x in v)
    return f"static const float {name}[{len(v)}] = {{ {values} }};"


def save_outputs(
    out_dir: Path,
    input_col: str,
    Ts: float,
    result: Dict[str, np.ndarray | float],
    result_bias: Optional[Dict[str, np.ndarray | float]],
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    A = result["A"]
    B = result["B"]
    c = result["c"]

    eig_A = np.linalg.eigvals(A)

    payload = {
        "input_col": input_col,
        "Ts_identificado_s": Ts,
        "state_order": ["theta_error_rad", "alpha_rad", "theta_dot_rad_s", "alpha_dot_rad_s"],
        "model": "x[k+1] = A x[k] + B u[k]",
        "A": A.tolist(),
        "B": B.tolist(),
        "c": c.tolist(),
        "eig_A": [
            {"real": float(np.real(z)), "imag": float(np.imag(z))}
            for z in eig_A
        ],
        "rmse_train_rad": result["rmse_train"].tolist(),
        "rmse_val_rad": result["rmse_val"].tolist(),
        "rmse_train_deg": model_score_deg(result["rmse_train"]).tolist(),
        "rmse_val_deg": model_score_deg(result["rmse_val"]).tolist(),
        "n_train": int(result["n_train"]),
        "n_val": int(result["n_val"]),
        "use_theta_error": USE_THETA_ERROR,
        "fit_bias_used_for_export": False,
    }

    if result_bias is not None:
        payload["diagnostic_with_bias"] = {
            "A": result_bias["A"].tolist(),
            "B": result_bias["B"].tolist(),
            "c": result_bias["c"].tolist(),
            "rmse_train_rad": result_bias["rmse_train"].tolist(),
            "rmse_val_rad": result_bias["rmse_val"].tolist(),
            "rmse_train_deg": model_score_deg(result_bias["rmse_train"]).tolist(),
            "rmse_val_deg": model_score_deg(result_bias["rmse_val"]).tolist(),
        }

    json_path = out_dir / f"modelo_identificado_{input_col}_Ts_{Ts:.6f}.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=4, ensure_ascii=False)

    h_path = out_dir / f"modelo_identificado_{input_col}_Ts_{Ts:.6f}.h"
    cpp_text = []
    cpp_text.append("#pragma once")
    cpp_text.append("")
    cpp_text.append("// Modelo identificado por mínimos quadrados.")
    cpp_text.append("// Estado: [theta_error_rad, alpha_rad, theta_dot_rad_s, alpha_dot_rad_s]")
    cpp_text.append(f"// Entrada: {input_col}")
    cpp_text.append(f"// Ts identificado: {Ts:.9f} s")
    cpp_text.append("")
    cpp_text.append(make_cpp_array_matrix("AD_ID", A))
    cpp_text.append("")
    cpp_text.append(make_cpp_array_vector("BD_ID", B))
    cpp_text.append("")
    cpp_text.append(make_cpp_array_vector("C_BIAS_ID", c))
    cpp_text.append("")
    h_path.write_text("\n".join(cpp_text), encoding="utf-8")

    print(f"\nArquivos salvos:")
    print(f"  {json_path}")
    print(f"  {h_path}")


def save_validation_plots(
    out_dir: Path,
    input_col: str,
    Ts: float,
    result: Dict[str, np.ndarray | float],
) -> None:
    if not SAVE_PLOTS:
        return

    out_dir.mkdir(parents=True, exist_ok=True)

    Y_val = result["Y_val"]
    Y_pred = result["Y_pred_val"]

    labels = [
        ("theta_error", "rad"),
        ("alpha", "rad"),
        ("theta_dot", "rad/s"),
        ("alpha_dot", "rad/s"),
    ]

    n = len(Y_val)
    t_val = np.arange(n) * Ts

    for i, (name, unit) in enumerate(labels):
        plt.figure(figsize=(10, 4))
        plt.plot(t_val, Y_val[:, i], label="medido")
        plt.plot(t_val, Y_pred[:, i], label="predito 1 passo")
        plt.xlabel("tempo de validação (s)")
        plt.ylabel(f"{name} ({unit})")
        plt.title(f"Validação - {name} - entrada {input_col}")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()

        path = out_dir / f"validacao_{input_col}_{name}.png"
        plt.savefig(path, dpi=150)
        plt.close()


def try_convert_model_to_ts(
    A: np.ndarray,
    B: np.ndarray,
    Ts_from: float,
    Ts_to: float,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    """
    Converte modelo discreto de Ts_from para Ts_to usando logm/expm.

    Atenção:
    - Requer scipy.
    - É uma aproximação baseada em ZOH.
    - O melhor é identificar diretamente no mesmo Ts usado no firmware.
    """
    try:
        from scipy.linalg import logm, expm
    except Exception:
        print("\nSciPy não encontrado. Conversão de Ts ignorada.")
        print("Para habilitar: pip install scipy")
        return None

    n = A.shape[0]
    m = B.shape[1]

    M = np.eye(n + m)
    M[:n, :n] = A
    M[:n, n:n + m] = B

    G = logm(M) / Ts_from
    M_to = expm(G * Ts_to)

    A_to = np.real_if_close(M_to[:n, :n]).astype(float)
    B_to = np.real_if_close(M_to[:n, n:n + m]).astype(float)

    return A_to, B_to


def save_converted_model(
    out_dir: Path,
    input_col: str,
    Ts_from: float,
    Ts_to: float,
    A_to: np.ndarray,
    B_to: np.ndarray,
) -> None:
    payload = {
        "input_col": input_col,
        "Ts_original_s": Ts_from,
        "Ts_convertido_s": Ts_to,
        "state_order": ["theta_error_rad", "alpha_rad", "theta_dot_rad_s", "alpha_dot_rad_s"],
        "model": "x[k+1] = A x[k] + B u[k]",
        "A": A_to.tolist(),
        "B": B_to.tolist(),
        "observacao": (
            "Modelo convertido por logm/expm. Use com cuidado. "
            "O melhor é coletar dados diretamente no Ts do controle."
        ),
    }

    json_path = out_dir / f"modelo_convertido_{input_col}_Ts_{Ts_to:.6f}.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=4, ensure_ascii=False)

    h_path = out_dir / f"modelo_convertido_{input_col}_Ts_{Ts_to:.6f}.h"
    cpp_text = []
    cpp_text.append("#pragma once")
    cpp_text.append("")
    cpp_text.append("// Modelo convertido para outro Ts.")
    cpp_text.append("// Use com cuidado. O ideal é identificar diretamente neste Ts.")
    cpp_text.append("// Estado: [theta_error_rad, alpha_rad, theta_dot_rad_s, alpha_dot_rad_s]")
    cpp_text.append(f"// Entrada: {input_col}")
    cpp_text.append(f"// Ts original: {Ts_from:.9f} s")
    cpp_text.append(f"// Ts convertido: {Ts_to:.9f} s")
    cpp_text.append("")
    cpp_text.append(make_cpp_array_matrix("AD_ID_CONVERTED", A_to))
    cpp_text.append("")
    cpp_text.append(make_cpp_array_vector("BD_ID_CONVERTED", B_to))
    cpp_text.append("")
    h_path.write_text("\n".join(cpp_text), encoding="utf-8")

    print(f"\nModelo convertido salvo:")
    print(f"  {json_path}")
    print(f"  {h_path}")


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

def main() -> None:
    df = load_data()

    print(f"Amostras válidas: {len(df)}")
    print(f"Tempo inicial: {df['t_s'].iloc[0]:.6f} s")
    print(f"Tempo final:   {df['t_s'].iloc[-1]:.6f} s")

    dt = np.diff(df["t_s"].to_numpy(dtype=float))
    Ts_med = float(np.median(dt))
    print(f"Ts mediano da coleta: {Ts_med:.9f} s")
    print(f"Frequência aproximada: {1.0 / Ts_med:.2f} Hz")

    out_dir = DATA_DIR / "modelos_identificados"
    out_dir.mkdir(parents=True, exist_ok=True)

    results = {}

    for input_col in INPUT_COLUMNS_TO_TEST:
        if input_col not in df.columns:
            print(f"\nPulando {input_col}: coluna não existe no CSV.")
            continue

        print("\n" + "=" * 72)
        print(f"Identificando com entrada: {input_col}")
        print("=" * 72)

        X, u, t, Ts = build_state_and_input(df, input_col)

        result = least_squares_identification(
            X=X,
            u=u,
            train_frac=TRAIN_FRAC,
            add_bias=False,
        )

        result_bias = None
        if FIT_WITH_BIAS_FOR_DIAGNOSTIC:
            result_bias = least_squares_identification(
                X=X,
                u=u,
                train_frac=TRAIN_FRAC,
                add_bias=True,
            )

        A = result["A"]
        B = result["B"]

        print_matrix("A identificado", A)
        print_matrix("B identificado", B)

        eig_A = np.linalg.eigvals(A)
        print_matrix("autovalores de A", eig_A.reshape(-1, 1))

        rmse_train_deg = model_score_deg(result["rmse_train"])
        rmse_val_deg = model_score_deg(result["rmse_val"])

        print("\nRMSE treino:")
        print(f"  theta_error:     {rmse_train_deg[0]:.6f} deg")
        print(f"  alpha:           {rmse_train_deg[1]:.6f} deg")
        print(f"  theta_dot:       {rmse_train_deg[2]:.6f} deg/s")
        print(f"  alpha_dot:       {rmse_train_deg[3]:.6f} deg/s")

        print("\nRMSE validação:")
        print(f"  theta_error:     {rmse_val_deg[0]:.6f} deg")
        print(f"  alpha:           {rmse_val_deg[1]:.6f} deg")
        print(f"  theta_dot:       {rmse_val_deg[2]:.6f} deg/s")
        print(f"  alpha_dot:       {rmse_val_deg[3]:.6f} deg/s")

        if result_bias is not None:
            rmse_bias_deg = model_score_deg(result_bias["rmse_val"])
            c_bias = result_bias["c"]
            print("\nDiagnóstico com constante c:")
            print_matrix("c", c_bias)
            print("RMSE validação com c:")
            print(f"  theta_error:     {rmse_bias_deg[0]:.6f} deg")
            print(f"  alpha:           {rmse_bias_deg[1]:.6f} deg")
            print(f"  theta_dot:       {rmse_bias_deg[2]:.6f} deg/s")
            print(f"  alpha_dot:       {rmse_bias_deg[3]:.6f} deg/s")

        save_outputs(out_dir, input_col, Ts, result, result_bias)
        save_validation_plots(out_dir, input_col, Ts, result)

        results[input_col] = {
            "X": X,
            "u": u,
            "Ts": Ts,
            "result": result,
        }

    if PREFERRED_INPUT not in results:
        available = list(results.keys())
        raise ValueError(
            f"A entrada preferida {PREFERRED_INPUT!r} não foi identificada. "
            f"Entradas disponíveis: {available}"
        )

    preferred = results[PREFERRED_INPUT]
    result = preferred["result"]
    Ts = preferred["Ts"]
    A = result["A"]
    B = result["B"]

    print("\n" + "=" * 72)
    print(f"Modelo principal escolhido: {PREFERRED_INPUT}")
    print("=" * 72)
    print(f"Ts identificado: {Ts:.9f} s")

    if abs(Ts - TS_CONTROL) > 1e-9:
        print("\nATENÇÃO:")
        print(f"  Sua coleta está com Ts = {Ts:.6f} s.")
        print(f"  Seu controle pode estar rodando com Ts = {TS_CONTROL:.6f} s.")
        print("  Não use diretamente uma matriz identificada em 20 ms num LQR de 2 ms.")
        print("  O ideal é coletar dados no mesmo período do controle.")

    if TRY_EXPORT_TS_CONTROL and abs(Ts - TS_CONTROL) > 1e-9:
        converted = try_convert_model_to_ts(A, B, Ts_from=Ts, Ts_to=TS_CONTROL)
        if converted is not None:
            A_to, B_to = converted
            save_converted_model(
                out_dir=out_dir,
                input_col=PREFERRED_INPUT,
                Ts_from=Ts,
                Ts_to=TS_CONTROL,
                A_to=A_to,
                B_to=B_to,
            )

    print("\nFinalizado.")
    print(f"Veja os arquivos em: {out_dir}")


if __name__ == "__main__":
    main()
