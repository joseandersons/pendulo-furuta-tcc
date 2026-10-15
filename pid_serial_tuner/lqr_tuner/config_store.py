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
    }

    cfg["observer_poles"] = {
        "p1": float(vals.get("observer_p1", -18.0)),
        "p2": float(vals.get("observer_p2", -28.0)),
    }

    cfg["microsteps"] = {
        "auto_enabled": bool(vals.get("microstep_auto_enabled", True)),
        "manual": int(vals.get("microstep_manual", 256)),
    }

    cfg["limits"] = {
        "accel_max_rad_s2": float(vals.get("accel_max_rad_s2", 30.0)),
    }

    CONFIG_PATH.write_text(
        json.dumps(cfg, indent=2, ensure_ascii=False),
        encoding="utf-8"
    )
