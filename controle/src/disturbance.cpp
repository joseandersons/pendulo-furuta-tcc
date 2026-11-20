#include "disturbance.h"
#include "config.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static DistSnapshot pulse;
static bool allowed = false;
static float alpha = 0;
static int64_t last_control_us = 0;
static uint32_t generation = 0;
static int64_t pending_since_us = 0;
static float kick_alpha_start = 0;
static int64_t kick_phase1_us = 0;
static int64_t kick_phase2_start_us = 0;
// Reserve space for START + terminal event before accepting a pulse.
struct Message { const char* kind; const char* code; DistSnapshot p; int64_t time; };
static Message messages[32];
static unsigned head = 0, count = 0;

static const char* state_name(DistState s) {
  switch (s) {
    case DistState::PENDING: return "PENDING";
    case DistState::ACTIVE: return "ACTIVE";
    case DistState::ENDED: return "ENDED";
    case DistState::CANCELLED: return "CANCELLED";
    default: return "IDLE";
  }
}
static bool busy() {
  return pulse.state == DistState::PENDING || pulse.state == DistState::ACTIVE;
}
static void emit(const char* kind, const char* code, int64_t now) {
  if (count < 32) {
    messages[(head + count++) % 32] = {kind, code, pulse, now};
  }
}
static void cancel(int64_t now, const char* reason) {
  if (busy()) {
    pulse.state = DistState::CANCELLED;
    pulse.value = 0;
    emit("EVENT", reason, now);
  }
}
static void expire(int64_t now) {
  if (pulse.state == DistState::ACTIVE &&
      now - pulse.start_us >= (int64_t)pulse.duration_ms * 1000) {
    pulse.state = DistState::ENDED;
    pulse.value = 0;
    emit("EVENT", "END", now);
  }
  if (busy() && now - last_control_us > DIST_CONTROL_FRESH_US) {
    cancel(now, "STALE_CONTROL");
    allowed = false;
  }
}
void disturbance_reset(int64_t now, const char* reason) {
  portENTER_CRITICAL(&mux);
  generation++;
  cancel(now, reason);
  allowed = false;
  portEXIT_CRITICAL(&mux);
}
uint32_t disturbance_generation() {
  portENTER_CRITICAL(&mux);
  const auto result = generation;
  portEXIT_CRITICAL(&mux);
  return result;
}
DistSnapshot disturbance_update(int64_t now, bool lqr_allowed, float alpha_rad, uint32_t tick_generation,
                                float alpha_dot_rad_s) {
  portENTER_CRITICAL(&mux);
  expire(now);
  if (tick_generation != generation) {
    DistSnapshot result = pulse;
    result.value = 0;
    portEXIT_CRITICAL(&mux);
    return result;
  }
  allowed = lqr_allowed;
  alpha = alpha_rad;
  last_control_us = now;
  if (!allowed) cancel(now, "MODE");
  if (pulse.state == DistState::PENDING) {
    const bool calm = fabsf(alpha) <= DIST_START_CALM_ALPHA_RAD &&
                      isfinite(alpha_dot_rad_s) &&
                      fabsf(alpha_dot_rad_s) <= DIST_START_CALM_ALPHA_DOT_RAD_S;
    if (!isfinite(alpha) || fabsf(alpha) > DIST_SAFE_ALPHA_RAD) {
      cancel(now, "ANGLE");
    } else if (pulse.kick && now - pending_since_us > (int64_t)DIST_START_TIMEOUT_MS * 1000) {
      cancel(now, "TIMEOUT");
    } else if (!pulse.kick || calm) {
      // O toque espera o pendulo calmo; o pulso antigo comeca no proximo ciclo.
      pulse.state = DistState::ACTIVE;
      pulse.start_us = now;
      if (pulse.kick) {
        pulse.kick_phase = 1;
        kick_alpha_start = alpha;
        pulse.value = (pulse.amplitude >= 0 ? 1.0f : -1.0f) * pulse.kick_accel;
      } else {
        pulse.value = pulse.amplitude;
      }
      emit("EVENT", "START", now);
    }
  } else if (pulse.state == DistState::ACTIVE && pulse.kick) {
    const float sign = pulse.amplitude >= 0 ? 1.0f : -1.0f;
    const float half_rad = 0.5f * fabsf(pulse.amplitude) * 0.017453292519943295f;
    if (pulse.kick_phase == 1) {
      // Fase 1: acelera ate o pendulo inclinar alpha0/2 (meio deslocamento).
      if (sign * (alpha - kick_alpha_start) >= half_rad ||
          now - pulse.start_us >= (int64_t)KICK_PHASE_MAX_MS * 1000) {
        pulse.kick_phase = 2;
        kick_phase1_us = now - pulse.start_us;
        kick_phase2_start_us = now;
        pulse.value = -sign * pulse.kick_accel;
      }
    } else if (now - kick_phase2_start_us >= kick_phase1_us) {
      // Fase 2 terminou (mesma duracao): braco ~parado, LQR reassume.
      pulse.state = DistState::ENDED;
      pulse.value = 0;
      pulse.kick_phase = 0;
      emit("EVENT", "END", now);
    }
  }
  DistSnapshot result = pulse;
  portEXIT_CRITICAL(&mux);
  return result;
}
DistSnapshot disturbance_snapshot(int64_t now) {
  portENTER_CRITICAL(&mux);
  expire(now);
  DistSnapshot result = pulse;
  portEXIT_CRITICAL(&mux);
  return result;
}
bool disturbance_motor_active(uint32_t id, int64_t now) {
  const auto p = disturbance_snapshot(now);
  return id != 0 && p.id == id && p.state == DistState::ACTIVE;
}

bool disturbance_command(const char* line, int64_t now) {
  if (strncmp(line, "DIST", 4) != 0) return false;
  // Tokenize a private copy; reject overflow, extra tokens and partial numbers.
  char copy[200];
  char* tokens[5]{};
  unsigned n = 0;
  bool valid_length = strlen(line) < sizeof(copy);
  if (valid_length) {
    strcpy(copy, line);
    char* save = nullptr;
    for (char* t = strtok_r(copy, " \t", &save); t && n < 5;
         t = strtok_r(nullptr, " \t", &save)) tokens[n++] = t;
  }
  bool query = n == 1 && strcmp(tokens[0], "DIST?") == 0;
  bool stop = n == 2 && strcmp(tokens[0], "DIST") == 0 && strcmp(tokens[1], "CANCEL") == 0;
  const bool is_kick = n == 4 && strcmp(tokens[0], "DIST") == 0 && strcmp(tokens[1], "KICK") == 0;
  bool request = n == 4 && strcmp(tokens[0], "DIST") == 0 &&
                 (strcmp(tokens[1], "PULSE") == 0 || is_kick);
  float amplitude = 0;
  unsigned long duration = 0;
  const char* error = "SYNTAX";
  if (request) {
    char* end = nullptr;
    errno = 0;
    amplitude = strtof(tokens[2], &end);
    request = end != tokens[2] && *end == 0 && errno != ERANGE && isfinite(amplitude);
    if (request) {
      // Duration is an unsigned decimal integer, not 100ms, 100.5 or -1.
      for (const char* c = tokens[3]; *c; ++c) if (*c < '0' || *c > '9') request = false;
      errno = 0;
      duration = strtoul(tokens[3], &end, 10);
      request = request && *end == 0 && errno != ERANGE;
    }
    // PULSE: amplitude em rad/s^2 e duracao em ms.
    // KICK: amplitude = alpha0 em graus (sinal = sentido) e o segundo
    // numero = |a| do tranco em rad/s^2 (inteiro).
    const bool out_of_range = is_kick
      ? (fabsf(amplitude) < KICK_ALPHA_MIN_DEG || fabsf(amplitude) > KICK_ALPHA_MAX_DEG ||
         duration < KICK_ACCEL_MIN_RAD_S2 || duration > (unsigned long)DIST_AMPLITUDE_MAX_RAD_S2)
      : (fabsf(amplitude) > DIST_AMPLITUDE_MAX_RAD_S2 ||
         duration < DIST_DURATION_MIN_MS || duration > DIST_DURATION_MAX_MS);
    if (request && out_of_range) {
      request = false;
      error = "RANGE";
    }
  }
  portENTER_CRITICAL(&mux);
  expire(now);
  // Cancellation always takes effect, including under serial backpressure.
  if (stop) {
    cancel(now, "CANCEL");
    if (count < 30) emit("OK", "CANCEL", now);
    portEXIT_CRITICAL(&mux);
    return true;
  }
  // Keep two slots reserved for an accepted pulse's lifecycle events.
  if (count >= 29) {
    if (count == 29) emit("ERR", "BACKPRESSURE", now);
    portEXIT_CRITICAL(&mux);
    return true; // Backpressure: no acceptance, caller may query after draining.
  }
  if (query) emit("STATUS", "QUERY", now);
  else if (!request) emit("ERR", error, now);
  else if (busy()) emit("ERR", "BUSY", now);
  else if (!allowed || now - last_control_us > DIST_CONTROL_FRESH_US) emit("ERR", "MODE", now);
  else if (!isfinite(alpha) || fabsf(alpha) > DIST_SAFE_ALPHA_RAD) emit("ERR", "ANGLE", now);
  else {
    pulse.id++;
    if (!pulse.id) pulse.id++;
    pulse.state = DistState::PENDING;
    pending_since_us = now;
    pulse.amplitude = amplitude;
    pulse.kick = is_kick;
    pulse.kick_phase = 0;
    pulse.kick_accel = is_kick ? (float)duration : 0.0f;
    // KICK: a duracao e so um teto de seguranca (as fases terminam antes).
    pulse.duration_ms = is_kick ? 2u * KICK_PHASE_MAX_MS + 20u : (uint32_t)duration;
    pulse.start_us = 0;
    pulse.value = 0;
    emit("OK", is_kick ? "KICK" : "PULSE", now);
  }
  portEXIT_CRITICAL(&mux);
  return true;
}
bool disturbance_pop_message(char* out, size_t size) {
  Message m;
  portENTER_CRITICAL(&mux);
  if (!count) { portEXIT_CRITICAL(&mux); return false; }
  m = messages[head];
  head = (head + 1) % 32;
  count--;
  portEXIT_CRITICAL(&mux);
  snprintf(out, size,
    "DIST %s %s id=%lu t_us=%lld state=%s a=%.6f ms=%lu start_us=%lld kick=%d\n",
    m.kind, m.code, (unsigned long)m.p.id, (long long)m.time,
    state_name(m.p.state), m.p.amplitude, (unsigned long)m.p.duration_ms,
    (long long)m.p.start_us, m.p.kick ? (int)m.p.kick_accel : 0);
  return true;
}
