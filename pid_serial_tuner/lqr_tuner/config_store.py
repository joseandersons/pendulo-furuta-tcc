import json
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
DATA_DIR = BASE_DIR / "data"
DATA_DIR.mkdir(exist_ok=True)

CONFIG_PATH = BASE_DIR / "config.json"


def load_config():
    if not CONFIG_PATH.exists():
        return {}

    try:
        return json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    except Exception:
        return {}


def save_config(vals):
    cfg = load_config()

    cfg["lqr_qr"] = {
        "q_theta": float(vals.get("q_theta", 5.0)),
        "q_alpha": float(vals.get("q_alpha", 300.0)),
        "q_theta_dot": float(vals.get("q_theta_dot", 1.0)),
        "q_alpha_dot": float(vals.get("q_alpha_dot", 20.0)),
        "r": float(vals.get("r", 1.0)),
    }

    cfg["lqr"] = {
        "k_theta": float(vals.get("k_theta", -2.2038)),
        "k_alpha": float(vals.get("k_alpha", 228.1777)),
        "k_theta_dot": float(vals.get("k_theta_dot", -3.0616)),
        "k_alpha_dot": float(vals.get("k_alpha_dot", 38.1739)),
    }

    cfg["theta_ref"] = {
        "theta_target_deg": float(vals.get("theta_target_deg", 0.0)),
        "rate_rad_s": float(vals.get("theta_ref_rate_rad_s", 0.6)),
    }

    cfg["observer_poles"] = {
        "p1": float(vals.get("observer_p1", -18.0)),
        "p2": float(vals.get("observer_p2", -28.0)),
    }

    microsteps = {
        "auto_enabled": bool(vals.get("microstep_auto_enabled", True)),
        "manual": int(vals.get("microstep_manual", 256)),
    }

    for key in ("ramp_down", "ramp_up"):
        ramp = vals.get(f"microstep_ramp_{key[5:]}")

        if ramp is not None:
            microsteps[key] = {
                "enabled": [bool(en) for en in ramp["enabled"]],
                "ms": [int(ms) for ms in ramp["ms"]],
            }
        elif key in cfg.get("microsteps", {}):
            microsteps[key] = cfg["microsteps"][key]

    cfg["microsteps"] = microsteps

    cfg["limits"] = {
        "accel_max_rad_s2": float(vals.get("accel_max_rad_s2", 30.0)),
    }

    cfg["disturbance"] = {
        "magnitude_rad_s2": float(vals.get("dist_magnitude", 3.0)),
        "duration_ms": int(vals.get("dist_duration_ms", 100)),
        "negative": bool(vals.get("dist_negative", False)),
        "kick_alpha_deg": float(vals.get("kick_alpha_deg", 5.0)),
        "kick_accel_rad_s2": float(vals.get("kick_accel_rad_s2", 40.0)),
    }

    _write_config(cfg)


def save_ui_sections(sections):
    """Guarda quais seções da barra lateral estão abertas."""
    cfg = load_config()
    cfg.setdefault("ui", {})["sections"] = {
        str(k): bool(v) for k, v in sections.items()
    }
    _write_config(cfg)


def _write_config(cfg):
    CONFIG_PATH.write_text(
        json.dumps(cfg, indent=2, ensure_ascii=False),
        encoding="utf-8"
    )
