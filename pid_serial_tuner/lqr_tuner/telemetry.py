import math


def parse_kv(line: str) -> dict:
    try:
        parts = line.strip().split(",")
        d = {}
        i = 0
        while i < len(parts) - 1:
            d[parts[i].strip()] = parts[i + 1].strip()
            i += 2
        return d
    except Exception:
        return {}


def to_float(d: dict, key: str, default=math.nan):
    try:
        return float(d.get(key, default))
    except Exception:
        return default


def wrap_deg_180(angle_deg: float) -> float:
    """Mantem o angulo visual no intervalo [-180, 180]."""
    if not math.isfinite(angle_deg):
        return angle_deg

    wrapped = (angle_deg + 180.0) % 360.0 - 180.0

    # Preserva +180 quando o valor original chega pelo lado positivo.
    if wrapped == -180.0 and angle_deg > 0.0:
        return 180.0

    return wrapped


def decode_telemetry(line: str):
    d = parse_kv(line)
    if not d or "T" not in d:
        return None
    t_ms = to_float(d, "T", 0.0)
    theta_deg = to_float(d, "TH") / 100.0
    return {
        "t_s": t_ms / 1000.0,
        "alpha_deg": to_float(d, "A") / 100.0,
        "theta_deg": wrap_deg_180(theta_deg),
        "alpha_dot_deg_s": to_float(d, "AD") / 100.0,
        "theta_dot_deg_s": to_float(d, "TD") / 100.0,
        "u_rad_s2": to_float(d, "U") / 100.0,
        "vel_rad_s": to_float(d, "VEL") / 100.0,
        "enabled": int(to_float(d, "EN", 0)),
    }
