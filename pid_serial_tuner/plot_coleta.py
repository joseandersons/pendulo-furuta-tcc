
from pathlib import Path
import argparse
import math

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


FILES = {
    "20": "coleta_20.csv",
    "30": "coleta_30.csv",
    "60": "coleta_60.csv",
    "180 (sem fuzzy)": "coleta_semfuzzy.csv",
}


def load_last_window(csv_path: Path, window_s: float) -> dict:
    df = pd.read_csv(csv_path)

    required = ["t_s", "TH_deg", "SP_deg", "THM_deg", "E_deg", "DUTY"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"{csv_path.name}: colunas ausentes: {missing}")

    df = df.sort_values("t_s").reset_index(drop=True).copy()

    for col in required + [c for c in ["inicio_acao_deg", "acao_maxima_deg"] if c in df.columns]:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    t0 = float(df["t_s"].dropna().iloc[0])
    t_end = float(df["t_s"].dropna().iloc[-1])
    t_start = max(t0, t_end - window_s)

    df = df[df["t_s"] >= t_start].copy().reset_index(drop=True)

    if "inicio_acao_deg" in df.columns and df["inicio_acao_deg"].notna().any():
        region = int(round(float(df["inicio_acao_deg"].dropna().iloc[0])))
    else:
        region = 180 if "sem fuzzy" in csv_path.name.lower() else None

    return {
        "path": csv_path,
        "label": csv_path.stem,
        "region": region,
        "t_start": t_start,
        "t_end": t_end,
        "df": df,
    }


def compute_metrics(df: pd.DataFrame, t_start: float) -> dict:
    t = df["t_s"].to_numpy(dtype=float)
    e = df["E_deg"].to_numpy(dtype=float)
    thm = df["THM_deg"].to_numpy(dtype=float)

    if len(t) < 2:
        return {
            "MSE": float("nan"),
            "IAE": float("nan"),
            "ISE": float("nan"),
            "ITAE": float("nan"),
            "ErroMaxAbs_deg": float("nan"),
            "BracoMaxAbs_deg": float("nan"),
        }

    dt = np.diff(t)
    dt = np.append(dt, np.median(dt))

    return {
        "MSE": float(np.mean(e**2)),
        "IAE": float(np.sum(np.abs(e) * dt)),
        "ISE": float(np.sum((e**2) * dt)),
        "ITAE": float(np.sum((t - t_start) * np.abs(e) * dt)),
        "ErroMaxAbs_deg": float(np.max(np.abs(e))),
        "BracoMaxAbs_deg": float(np.max(np.abs(thm))),
    }


def save_fig(fig, outdir: Path, name: str) -> None:
    fig.savefig(outdir / name, dpi=300, bbox_inches="tight")
    plt.close(fig)


def plot_braco_por_regiao(cases: list[dict], outdir: Path, window_s: float) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharey=True)
    axes = axes.ravel()

    ymax = max(float(np.nanmax(np.abs(case["df"]["THM_deg"]))) for case in cases)
    ymax = max(40.0, math.ceil((ymax + 10) / 10) * 10)

    for ax, case in zip(axes, cases):
        df = case["df"]
        region = case["region"]
        title = f"Região {region}" if region != 180 else "Região 180 (sem fuzzy)"

        if region != 180:
            ax.axhspan(-region, region, alpha=0.15, label="Faixa útil")
            ax.axhline(region, linestyle="--", linewidth=1.2)
            ax.axhline(-region, linestyle="--", linewidth=1.2)

        ax.plot(df["t_s"], df["THM_deg"], linewidth=2, label=title.replace("Região ", ""))
        ax.set_title(title)
        ax.set_xlabel("Tempo (s)")
        ax.set_ylabel("THM (deg)")
        ax.set_xlim(case["t_start"], case["t_end"])
        ax.set_ylim(-ymax, ymax)
        ax.grid(True, alpha=0.6)
        ax.legend()

    fig.suptitle(f"Braço em relação à região definida — últimos {window_s:g} s", fontsize=15)
    fig.tight_layout()
    save_fig(fig, outdir, "comparacao_braco_regiao_60s.png")


def plot_pendulo_por_regiao(cases: list[dict], outdir: Path, window_s: float) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharey=True)
    axes = axes.ravel()

    ymin = min(float(np.nanmin(case["df"]["TH_deg"])) for case in cases)
    ymax = max(float(np.nanmax(case["df"]["TH_deg"])) for case in cases)
    ymin = math.floor((ymin - 1) / 1) * 1
    ymax = math.ceil((ymax + 1) / 1) * 1

    for ax, case in zip(axes, cases):
        df = case["df"]
        region = case["region"]
        title = f"Região {region}" if region != 180 else "Região 180 (sem fuzzy)"

        sp = np.zeros(len(df)) if region == 180 else df["SP_deg"].to_numpy(dtype=float)

        ax.plot(df["t_s"], df["TH_deg"], linewidth=1.7, label="TH")
        ax.plot(df["t_s"], sp, linewidth=1.7, linestyle="--", label="SP")
        ax.set_title(title)
        ax.set_xlabel("Tempo (s)")
        ax.set_ylabel("TH (deg)")
        ax.set_xlim(case["t_start"], case["t_end"])
        ax.set_ylim(ymin, ymax)
        ax.grid(True, alpha=0.6)
        ax.legend()

    fig.suptitle(f"Ângulo do pêndulo por região — últimos {window_s:g} s", fontsize=15)
    fig.tight_layout()
    save_fig(fig, outdir, "comparacao_pendulo_regiao_60s.png")


def plot_overlay(cases: list[dict], ycol: str, ylabel: str, title: str, filename: str, outdir: Path) -> None:
    fig = plt.figure(figsize=(12, 5))
    ax = fig.add_subplot(111)

    for case in cases:
        df = case["df"]
        region = case["region"]
        label = f"{region}" if region != 180 else "180 (sem fuzzy)"
        y = df[ycol].to_numpy(dtype=float)
        ax.plot(df["t_s"], y, linewidth=1.8, label=label)

    ax.set_title(title)
    ax.set_xlabel("Tempo (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.6)
    ax.legend(title="Região")
    fig.tight_layout()
    save_fig(fig, outdir, filename)


def save_metrics(cases: list[dict], outdir: Path) -> None:
    rows = []
    for case in cases:
        m = compute_metrics(case["df"], case["t_start"])
        rows.append({
            "Regiao": case["region"] if case["region"] != 180 else "180 (sem fuzzy)",
            "Arquivo": case["path"].name,
            "t_inicio_janela_s": round(case["t_start"], 4),
            "t_fim_janela_s": round(case["t_end"], 4),
            "duracao_janela_s": round(case["t_end"] - case["t_start"], 4),
            **m,
        })

    pd.DataFrame(rows).to_csv(outdir / "metricas_ultimos_60s.csv", index=False, encoding="utf-8-sig")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--folder", type=str, default="coletas")
    parser.add_argument("--window", type=float, default=60.0)
    parser.add_argument("--output", type=str, default="graficos_ultimos_60s")
    args = parser.parse_args()

    folder = Path(args.folder)
    outdir = Path(args.output)
    outdir.mkdir(parents=True, exist_ok=True)

    cases = []
    for label, filename in FILES.items():
        path = folder / filename
        if not path.exists():
            raise FileNotFoundError(f"Arquivo não encontrado: {path}")
        cases.append(load_last_window(path, args.window))

    cases = sorted(cases, key=lambda c: (999 if c["region"] == 180 else c["region"]))

    plot_braco_por_regiao(cases, outdir, args.window)
    plot_pendulo_por_regiao(cases, outdir, args.window)
    plot_overlay(
        cases,
        ycol="THM_deg",
        ylabel="THM (deg)",
        title=f"Ângulo do braço: comparação entre regiões — últimos {args.window:g} s",
        filename="comparacao_braco_60s.png",
        outdir=outdir,
    )
    plot_overlay(
        cases,
        ycol="E_deg",
        ylabel="Erro (deg)",
        title=f"Erro angular do pêndulo: comparação entre regiões — últimos {args.window:g} s",
        filename="comparacao_erro_60s.png",
        outdir=outdir,
    )
    save_metrics(cases, outdir)

    print(f"Gráficos salvos em: {outdir.resolve()}")


if __name__ == "__main__":
    main()
