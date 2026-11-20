"""Strict DIST v1 protocol. Times are ESP32 monotonic microseconds."""
import math
import re

AMPLITUDE_MAX = 1000.0  # rad/s², igual a DIST_AMPLITUDE_MAX_RAD_S2 do firmware
DURATION_MIN_MS = 20
DURATION_MAX_MS = 3000  # igual a DIST_DURATION_MAX_MS do firmware
KICK_ALPHA_MIN_DEG = 0.5   # igual a KICK_ALPHA_MIN_DEG do firmware
KICK_ALPHA_MAX_DEG = 15.0  # igual a KICK_ALPHA_MAX_DEG do firmware
KICK_ACCEL_MIN = 10        # rad/s², igual a KICK_ACCEL_MIN_RAD_S2

STATES = {"IDLE", "PENDING", "ACTIVE", "ENDED", "CANCELLED"}
CODES = {
    "OK": {"PULSE", "KICK", "CANCEL"},
    "ERR": {"SYNTAX", "RANGE", "BUSY", "MODE", "ANGLE", "BACKPRESSURE"},
    "STATUS": {"QUERY"},
    "EVENT": {"START", "END", "CANCEL", "STOP", "START_RESET", "MODE",
              "ANGLE", "RESET", "CALIBRATION", "STALE_CONTROL", "TIMEOUT"},
}


def pulse_command(amplitude, duration_ms):
    if not math.isfinite(amplitude) or abs(amplitude) > AMPLITUDE_MAX:
        raise ValueError(f"Amplitude deve estar entre -{AMPLITUDE_MAX:g} e {AMPLITUDE_MAX:g} rad/s².")
    if isinstance(duration_ms, bool) or int(duration_ms) != duration_ms or not DURATION_MIN_MS <= duration_ms <= DURATION_MAX_MS:
        raise ValueError(f"Duração deve ser um inteiro entre {DURATION_MIN_MS} e {DURATION_MAX_MS} ms.")
    return f"DIST PULSE {amplitude:.6f} {int(duration_ms)}"


def kick_command(alpha0_deg, accel_rad_s2):
    """Toque em malha aberta: inclina o pendulo ~alpha0 (sinal = sentido) com
    um tranco do braco de |a| = accel_rad_s2, com o LQR suspenso."""
    if not math.isfinite(alpha0_deg) or not KICK_ALPHA_MIN_DEG <= abs(alpha0_deg) <= KICK_ALPHA_MAX_DEG:
        raise ValueError(f"α0 do toque deve estar entre {KICK_ALPHA_MIN_DEG:g} e {KICK_ALPHA_MAX_DEG:g} graus.")
    accel = int(round(accel_rad_s2))
    if not KICK_ACCEL_MIN <= accel <= AMPLITUDE_MAX:
        raise ValueError(f"Aceleração do tranco deve estar entre {KICK_ACCEL_MIN} e {AMPLITUDE_MAX:g} rad/s².")
    return f"DIST KICK {alpha0_deg:.3f} {accel}"


def decode_disturbance(line):
    tokens = line.split()
    if len(tokens) not in (9, 10) or tokens[0] != "DIST":
        return None
    kind, code = tokens[1:3]
    if code not in CODES.get(kind, set()):
        return None
    fields = {}
    for item in tokens[3:]:
        parts = item.split("=")
        if len(parts) != 2 or parts[0] in fields:
            return None
        fields[parts[0]] = parts[1]
    fields.setdefault("kick", "0")  # firmware antigo nao manda o campo
    if set(fields) != {"id", "t_us", "state", "a", "ms", "start_us", "kick"}:
        return None
    try:
        for key in ("id", "t_us", "ms", "start_us", "kick"):
            if not re.fullmatch(r"[0-9]+", fields[key]):
                return None
            fields[key] = int(fields[key])
        fields["a"] = float(fields["a"])
    except (ValueError, OverflowError):
        return None
    if (fields["state"] not in STATES or not math.isfinite(fields["a"])
            or abs(fields["a"]) > AMPLITUDE_MAX or not 0 <= fields["ms"] <= DURATION_MAX_MS
            or fields["id"] > 0xFFFFFFFF or fields["start_us"] > fields["t_us"]):
        return None
    if fields["state"] in {"ACTIVE", "PENDING"} and fields["ms"] < DURATION_MIN_MS:
        return None
    if kind == "EVENT" and ((code == "START" and fields["state"] != "ACTIVE")
                            or (code == "END" and fields["state"] != "ENDED")
                            or (code not in {"START", "END"} and fields["state"] != "CANCELLED")):
        return None
    return {"kind": kind, "code": code, **fields}
