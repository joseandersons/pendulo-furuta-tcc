"""Gera as figuras e a tabela do artigo ICIT 2027 a partir da coleta de 25/10/2026.

Uso: python make_figures.py
Saida: ../figures/*.pdf e metricas impressas no terminal (usadas na Tabela III).
"""
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "..", "..", "pid_serial_tuner", "lqr_tuner", "data",
                   "coleta_lqr_20261025_004214.csv")
OUT = os.path.join(HERE, "..", "figures")
os.makedirs(OUT, exist_ok=True)

F_MAX = 23000.0      # STEP_FREQ_MAX (config.h)
N_STEPS = 200.0      # MOTOR_STEPS_PER_REV (config.h)
WINDOW_S = 8.0       # janela de regime permanente, igual para todos os modos


def cap(mu):
    return 2 * math.pi * F_MAX / (N_STEPS * mu)


rows = list(csv.DictReader(open(CSV)))
T = [float(r["t_s"]) for r in rows]
A = [float(r["alpha_deg"]) for r in rows]
TH = [float(r["theta_deg"]) for r in rows]
V = [float(r["vel_rad_s"]) for r in rows]
U = [float(r["u_rad_s2"]) for r in rows]
MS = [int(float(r["microsteps_active"])) for r in rows]
LQR = [r["enabled"] == "1" and r["controller_mode"] == "2.0" for r in rows]
AUTO = [r["microstep_auto_actual"] == "1.0" for r in rows]

# (rotulo, mu fixo ou None, intervalo da coleta em s, inicio da janela de regime
# permanente relativo ao inicio do trecho LQR). 64 e 32: segunda rodada.
MODES_SPEC = [
    ("Fuzzy adaptive", None, (0, 36), 26.3),
    ("Fixed 1/256", 256, (35, 70), 10.0),
    ("Fixed 1/128", 128, (84, 120), 10.0),
    ("Fixed 1/64", 64, (198, 237), 10.0),
    ("Fixed 1/32", 32, (283, 328), 10.0),
    ("Fixed 1/16", 16, (350, 400), 4.0),
]
MODES = []
for label, mu, (t0, t1), ss in MODES_SPEC:
    idx = [k for k in range(len(rows))
           if t0 <= T[k] <= t1 and LQR[k] and AUTO[k] == (mu is None)]
    MODES.append(dict(label=label, mu=mu, i0=idx[0], i1=idx[-1], ss=ss))

# Golpes: mesma deteccao usada nos graficos de analise (plots.py).
pushes = []
for m in MODES:
    i0, i1 = m["i0"], m["i1"]
    k = i0
    while k <= i1:
        if abs(A[k]) >= 3.5 and T[k] - T[i0] > 2.0:
            on = k
            while on > max(i0, k - 15) and abs(A[on]) > 0.6:
                on -= 1
            w = [j for j in range(on, min(len(rows), on + 60)) if LQR[j]]
            pk = max(abs(A[j]) for j in w)
            if pk >= 4.5:
                fell = any(not LQR[j] for j in range(on, min(len(rows), on + 75)))
                rate = max(abs(A[j] - A[j - 1]) / (T[j] - T[j - 1]) for j in range(on + 1, on + 5))
                pushes.append(dict(mode=m["label"], on=on, t=T[on], peak=pk, rate=rate,
                                   vel=max(abs(V[j]) for j in w),
                                   ms_min=min(MS[j] for j in w), fell=fell))
                k = on + 60
                continue
        k += 1

# ---------------------------------------------------------------- estilo IEEE
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8,
    "axes.labelsize": 8,
    "axes.titlesize": 8,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "legend.fontsize": 7,
    "axes.linewidth": 0.6,
    "xtick.major.width": 0.5,
    "ytick.major.width": 0.5,
    "xtick.major.size": 2.5,
    "ytick.major.size": 2.5,
    "xtick.direction": "in",
    "ytick.direction": "in",
    "axes.grid": True,
    "grid.linewidth": 0.3,
    "grid.color": "0.85",
    "lines.linewidth": 0.8,
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
})
COL_W, PAGE_W = 3.5, 7.16
LINE = "#1f4e9c"
STYLE = {
    "Fuzzy adaptive": dict(color="#000000", linestyle="-", linewidth=1.1),
    "Fixed 1/256": dict(color="#d62728", linestyle="--", linewidth=0.9),
    "Fixed 1/128": dict(color="#ff7f0e", linestyle="-.", linewidth=0.9),
    "Fixed 1/64": dict(color="#9467bd", linestyle=(0, (1, 1)), linewidth=0.9),
    "Fixed 1/32": dict(color="#2ca02c", linestyle=(0, (5, 1, 1, 1, 1, 1)), linewidth=0.9),
    "Fixed 1/16": dict(color="#1f77b4", linestyle=":", linewidth=1.1),
}


def ss_window(m):
    t_start = T[m["i0"]] + m["ss"]
    ks = [k for k in range(m["i0"], m["i1"] + 1) if t_start <= T[k] < t_start + WINDOW_S]
    assert all(MS[k] == (m["mu"] or 256) for k in ks), m["label"]
    return ks


def save(fig, name):
    fig.savefig(os.path.join(OUT, name + ".pdf"))
    if os.environ.get("PREVIEW_DIR"):
        fig.savefig(os.path.join(os.environ["PREVIEW_DIR"], name + ".png"), dpi=300)
    plt.close(fig)


def rms(x):
    return math.sqrt(sum(v * v for v in x) / len(x))


# ------------------------------------------- Fig. 3: regime permanente (figure*)
wins = [(m, ss_window(m)) for m in MODES]
th_dev = {m["label"]: [TH[k] - sum(TH[j] for j in ks) / len(ks) for k in ks] for m, ks in wins}
a_lim = 3.0
th_lim = math.ceil(max(abs(v) for d in th_dev.values() for v in d) * 1.1)
u_lim = 10 * math.ceil(max(abs(U[k]) for _, ks in wins for k in ks) * 1.05 / 10)

fig, axs = plt.subplots(3, len(wins), figsize=(PAGE_W, 3.5), sharex=True, sharey="row")
for c, (m, ks) in enumerate(wins):
    tt = [T[k] - T[ks[0]] for k in ks]
    axs[0][c].plot(tt, [A[k] for k in ks], color=LINE)
    axs[1][c].plot(tt, th_dev[m["label"]], color=LINE)
    axs[2][c].plot(tt, [U[k] for k in ks], color=LINE, linewidth=0.6)
    axs[0][c].set_title(f"({chr(97 + c)}) {m['label']}")
    axs[2][c].set_xlabel("Time (s)")
    for r in range(3):
        axs[r][c].set_xlim(0, WINDOW_S)
        axs[r][c].set_xticks([0, 2, 4, 6, 8])
axs[0][0].set_ylim(-a_lim, a_lim)
axs[1][0].set_ylim(-th_lim, th_lim)
axs[2][0].set_ylim(-u_lim, u_lim)
axs[0][0].set_ylabel(r"$\alpha$ (deg)")
axs[1][0].set_ylabel(r"$\theta-\bar{\theta}$ (deg)")
axs[2][0].set_ylabel(r"$u$ (rad/s$^2$)")
fig.align_ylabels(axs[:, 0])
fig.subplots_adjust(wspace=0.12, hspace=0.12)
save(fig, "fig_steady_state")

# ------------------- Fig. 3 (alternativa): ultimos 30 s de cada modo, com golpes e quedas
LONG_S = 30.0
A_LIM, TH_LIM, U_LIM = 5.0, 10.0, 40.0
RED = "#c62828"
fig, axs = plt.subplots(4, len(MODES), figsize=(PAGE_W, 4.3), sharex=True, sharey="row",
                        gridspec_kw={"height_ratios": [1, 1, 1, 0.7]})
AFTER_S = 2.5  # quanto mostrar depois da queda (LQR desligado)
for c, m in enumerate(MODES):
    t_end = T[m["i1"]]
    fell = any(p["fell"] for p in pushes if p["mode"] == m["label"])
    t1 = t_end + AFTER_S if fell else t_end
    t0 = t1 - LONG_S
    ks = [k for k in range(m["i0"], m["i1"] + 1) if T[k] >= t0]
    tt = [T[k] - t0 for k in ks]
    th_ref = sorted(TH[k] for k in ks)[len(ks) // 2]
    col = [axs[r][c] for r in range(4)]
    col[0].plot(tt, [A[k] for k in ks], color=LINE, linewidth=0.5)
    col[1].plot(tt, [TH[k] - th_ref for k in ks], color=LINE, linewidth=0.5)
    col[2].plot(tt, [U[k] for k in ks], color=LINE, linewidth=0.4)
    col[3].step(tt, [MS[k] for k in ks], where="post", color=LINE, linewidth=0.8)
    if fell:
        # depois da queda o LQR esta desligado: mostra o pendulo caindo
        kf = [k for k in range(m["i1"], len(rows)) if T[k] <= t1]
        tf = [T[k] - t0 for k in kf]
        col[0].plot(tf, [A[k] for k in kf], color=RED, linewidth=0.9)
        col[1].plot(tf, [TH[k] - th_ref for k in kf], color=RED, linewidth=0.9)
        for a in col:
            a.axvspan(t_end - t0, LONG_S, color=RED, alpha=0.18, linewidth=0, zorder=0)
            a.axvline(t_end - t0, color=RED, linewidth=0.9, zorder=3)
        col[0].text(t_end - t0 - 0.6, A_LIM * 0.62, "lost\nbalance", color=RED, fontsize=6.5,
                    ha="right", va="center", linespacing=0.9,
                    bbox=dict(facecolor="white", edgecolor="none", pad=0.6))
    for p in pushes:
        if p["mode"] != m["label"] or not t0 <= p["t"] <= t1:
            continue
        for a in col:
            a.axvline(p["t"] - t0, color="0.45", linewidth=0.5, linestyle="--", zorder=0)
        col[0].plot(p["t"] - t0, A_LIM, marker="v", color="0.3", markersize=3.5,
                    clip_on=False, zorder=4)
    col[0].set_title(f"({chr(97 + c)}) {m['label']}")
    col[3].set_xlabel("Time (s)")
    col[0].set_xlim(0, LONG_S)
    col[0].set_xticks([0, 10, 20, 30])
axs[0][0].set_ylim(-A_LIM, A_LIM)
axs[1][0].set_ylim(-TH_LIM, TH_LIM)
axs[2][0].set_ylim(-U_LIM, U_LIM)
axs[3][0].set_yscale("log", base=2)
axs[3][0].set_yticks([16, 32, 64, 128, 256])
axs[3][0].set_yticklabels(["16", "32", "64", "128", "256"], fontsize=6)
axs[3][0].minorticks_off()
axs[3][0].set_ylim(12, 340)
axs[0][0].set_ylabel(r"$\alpha$ (deg)")
axs[1][0].set_ylabel(r"$\theta-\tilde{\theta}$ (deg)")
axs[2][0].set_ylabel(r"$u$ (rad/s$^2$)")
axs[3][0].set_ylabel(r"$\mu$")
fig.align_ylabels(axs[:, 0])
fig.subplots_adjust(wspace=0.12, hspace=0.12)
save(fig, "fig_last30s")

# ------------------------------- Fig. 4: resposta a golpe, eventos representativos
REPR = ["Fuzzy adaptive", "Fixed 1/256", "Fixed 1/128", "Fixed 1/16"]
chosen = []
for lab in REPR:
    evs = [p for p in pushes if p["mode"] == lab]
    # primeiro golpe que derrubou; se nenhum, o de maior pico
    ev = next((p for p in evs if p["fell"]), None) or max(evs, key=lambda p: p["peak"])
    chosen.append(ev)

T_PRE, T_POST = 0.2, 2.5
fig, axs = plt.subplots(3, 1, figsize=(COL_W, 3.6), sharex=True,
                        gridspec_kw={"height_ratios": [1.2, 1.2, 0.8]})
for p in chosen:
    st = STYLE[p["mode"]]
    ks = [k for k in range(p["on"] - 20, min(len(rows), p["on"] + 200))
          if -T_PRE <= T[k] - p["t"] <= T_POST]
    live = []
    for k in ks:
        if not LQR[k] and T[k] > p["t"]:
            break
        live.append(k)
    tt = [T[k] - p["t"] for k in live]
    lab = p["mode"] + (" (lost)" if p["fell"] else "")
    axs[0].plot(tt, [A[k] for k in live], label=lab, **st)
    axs[1].plot(tt, [abs(V[k]) for k in live], **st)
    axs[2].step(tt, [MS[k] for k in live], where="post", **st)
    if p["fell"]:
        axs[0].plot(tt[-1], A[live[-1]], marker="x", color=st["color"], markersize=5,
                    markeredgewidth=1.0, linestyle="none")
for mu in (256, 128):
    axs[1].axhline(cap(mu), color="0.35", linewidth=0.5, linestyle="-")
    axs[1].text(T_POST - 0.03, cap(mu) + 0.15, rf"$\omega_{{\max}}$ at 1/{mu}",
                ha="right", va="bottom", fontsize=6.5, color="0.25")
# faixa em que o LQR desliga (ALPHA_DISABLE_RAD = 20 graus, control.cpp)
axs[0].axhspan(20, 40, color="0.9", linewidth=0)
axs[0].axhspan(-40, -20, color="0.9", linewidth=0)
axs[0].set_ylim(-24, 24)
axs[0].set_yticks([-20, -10, 0, 10, 20])
axs[0].set_ylabel(r"$\alpha$ (deg)")
axs[1].set_ylim(0, 10)
axs[1].set_ylabel(r"$|\omega_{\mathrm{cmd}}|$ (rad/s)")
axs[2].set_yscale("log", base=2)
axs[2].set_yticks([16, 32, 64, 128, 256])
axs[2].set_yticklabels(["16", "32", "64", "128", "256"])
axs[2].minorticks_off()
axs[2].set_ylim(12, 340)
axs[2].set_ylabel(r"$\mu$")
axs[2].set_xlabel("Time after impact (s)")
axs[2].set_xlim(-T_PRE, T_POST)
axs[0].legend(loc="upper right", ncol=2, frameon=True, framealpha=1, edgecolor="0.8",
              handlelength=2.6, columnspacing=1.0, borderpad=0.3)
fig.align_ylabels(axs)
fig.subplots_adjust(hspace=0.08)
save(fig, "fig_disturbance")

# ------------------------------- Fig. 5: pico de velocidade x teto do micropasso
fig, ax = plt.subplots(figsize=(COL_W, 1.9))
labels = [m["label"] for m in MODES]
short = ["Fuzzy", "1/256", "1/128", "1/64", "1/32", "1/16"]
for x, m in enumerate(MODES):
    if m["mu"]:
        c = cap(m["mu"])
        if c < 24:
            ax.plot([x - 0.3, x + 0.3], [c, c], color="0.3", linewidth=1.2)
        else:
            ax.annotate(f"{c:.1f}", xy=(x, 24.5), xytext=(x, 21.3), ha="center", fontsize=6.5,
                        color="0.25", arrowprops=dict(arrowstyle="->", color="0.3", lw=0.6))
    evs = [p for p in pushes if p["mode"] == m["label"]]
    n = len(evs)
    for i, p in enumerate(evs):
        dx = (i - (n - 1) / 2) * 0.12
        if p["fell"]:
            ax.plot(x + dx, p["vel"], marker="x", color="#d62728", markersize=5.5,
                    markeredgewidth=1.2, linestyle="none", zorder=3)
        else:
            ax.plot(x + dx, p["vel"], marker="o", markerfacecolor="white", color="#1f4e9c",
                    markersize=4.5, markeredgewidth=1.0, linestyle="none", zorder=3)
ax.set_xticks(range(len(MODES)))
ax.set_xticklabels(short)
ax.set_xlim(-0.5, len(MODES) - 0.5)
ax.set_ylim(0, 25)
ax.set_ylabel(r"Peak $|\omega_{\mathrm{cmd}}|$ (rad/s)")
ax.grid(axis="x", visible=False)
ax.legend(handles=[
    Line2D([], [], marker="o", markerfacecolor="white", color="#1f4e9c", linestyle="none",
           markersize=4.5, label="Recovered"),
    Line2D([], [], marker="x", color="#d62728", linestyle="none", markersize=5.5,
           markeredgewidth=1.2, label="Lost balance"),
    Line2D([], [], color="0.3", linewidth=1.2, label=r"$\omega_{\max}=2\pi f_{\max}/(N\mu)$"),
], loc="upper left", frameon=True, framealpha=1, edgecolor="0.8", ncol=1)
save(fig, "fig_speed_ceiling")

# ------------------------------------------------------------- metricas (tabela)
print(f"Janela de regime permanente: {WINDOW_S:.0f} s por modo (amostras a 50 Hz)")
print(f"{'mode':15s} {'win(s)':>13s} {'aRMS':>6s} {'a_p95':>6s} {'sdTh':>6s} {'uRMS':>6s} "
      f"{'wRMS':>6s} {'held':>6s} {'cap':>6s}")
for m, ks in wins:
    a = [A[k] for k in ks]
    ab = sorted(abs(v) for v in a)
    th = th_dev[m["label"]]
    evs = [p for p in pushes if p["mode"] == m["label"]]
    held = sum(not p["fell"] for p in evs)
    t0 = T[m["i0"]]
    print(f"{m['label']:15s} {T[ks[0]] - t0:5.1f}-{T[ks[-1]] - t0:5.1f} s "
          f"{rms(a):6.3f} {ab[int(0.95 * len(ab))]:6.2f} {rms(th):6.2f} "
          f"{rms([U[k] for k in ks]):6.2f} {rms([V[k] for k in ks]):6.3f} "
          f"{held:>3d}/{len(evs):<2d} {cap(m['mu'] or 16):6.2f}")
print("\nGolpes:")
for p in pushes:
    print(f"{p['mode']:15s} t={p['t']:6.1f}s  dalpha/dt0={p['rate']:5.0f} deg/s  "
          f"pico={p['peak']:5.1f} deg  |w|max={p['vel']:5.2f}  mu_min={p['ms_min']:3d}  "
          f"{'LOST' if p['fell'] else 'held'}")
print("Escolhidos p/ Fig. disturbance:", [(p["mode"], round(p["t"], 1)) for p in chosen])
print("lims:", a_lim, th_lim, u_lim)
