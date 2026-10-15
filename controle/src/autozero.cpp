#include "autozero.h"
#include "config.h"
#include "calibration.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

#define AZ_NAMESPACE "furuta-az"

#define AZ_COUNTS_PER_DEG (CPR_PENDULO / 360.0f)

enum AzState {
  AZ_IDLE = 0,   // fora do LQR
  AZ_WAIT,       // no LQR, esperando o sistema assentar
  AZ_LEARN,      // ajustando d
  AZ_DONE        // ajuste concluido nesta posicao
};

struct AzBin {
  float theta_rad;  // angulo absoluto do braco onde o ponto foi medido
  float zero_counts; // zero correto do pendulo naquela posicao (0..CPR)
  uint8_t valid;
  uint8_t pad[3];
};

static portMUX_TYPE g_az_mux = portMUX_INITIALIZER_UNLOCKED;

static bool g_enabled = AZ_ENABLED_DEFAULT != 0;
static AzState g_state = AZ_IDLE;

// Curva da inclinacao (contagens)
static float g_tilt_p = 0.0f;
static float g_tilt_q = 0.0f;
static bool g_fit_valid = false;

// Ajuste local (contagens)
static float g_d = 0.0f;
static bool g_d_decay = false;

static AzBin g_bins[AZ_BINS];
static bool g_dirty = false;

// Estado do aprendizado
static bool g_have_target = false;
static float g_last_target_rad = 0.0f;
static float g_last_arm_abs_rad = 0.0f;
static float g_settle_t = 0.0f;
static float g_learn_t = 0.0f;
static float g_done_t = 0.0f;
static float g_alpha_lp = 0.0f;
static float g_alpha_slow = 0.0f;
static float g_theta_dot_lp = 0.0f;
static float g_d_at_learn_start = 0.0f;

// Caixa de mensagens (laco de controle -> tarefa serial)
#define AZ_MSG_SLOTS 4
#define AZ_MSG_LEN 168
static char g_msgs[AZ_MSG_SLOTS][AZ_MSG_LEN];
static int g_msg_head = 0;
static int g_msg_count = 0;

static const char* state_name(AzState s) {
  switch (s) {
    case AZ_IDLE:  return "PARADO";
    case AZ_WAIT:  return "ESPERANDO";
    case AZ_LEARN: return "APRENDENDO";
    case AZ_DONE:  return "CONCLUIDO";
  }
  return "?";
}

static float wrap_half_counts(float x) {
  const float half = 0.5f * CPR_PENDULO;
  while (x > half) x -= CPR_PENDULO;
  while (x < -half) x += CPR_PENDULO;
  return x;
}

static float normalize_counts(float x) {
  x = fmodf(x, CPR_PENDULO);
  if (x < 0.0f) x += CPR_PENDULO;
  return x;
}

static float wrap_pi_local(float x) {
  while (x > (float)PI) x -= 2.0f * (float)PI;
  while (x < -(float)PI) x += 2.0f * (float)PI;
  return x;
}

static float lpf(float y, float x, float dt, float tau) {
  float k = (tau > 0.0f) ? dt / tau : 1.0f;
  if (k > 1.0f) k = 1.0f;
  return y + (x - y) * k;
}

static float clampf_az(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static float tilt_locked(float arm_abs_rad) {
  return g_tilt_p * sinf(arm_abs_rad) + g_tilt_q * cosf(arm_abs_rad);
}

static int bin_index(float arm_abs_rad) {
  float deg = arm_abs_rad * 57.29577951308232f;
  deg = fmodf(deg, 360.0f);
  if (deg < 0.0f) deg += 360.0f;
  const float width = 360.0f / (float)AZ_BINS;
  int idx = (int)floorf((deg + 0.5f * width) / width);
  return idx % AZ_BINS;
}

/*
  Eventos para mensagens. Os numeros sao copiados dentro da secao critica e o
  texto e formatado fora dela (snprintf com float nao deve rodar com a
  secao critica ativa).
*/
enum AzEventType {
  AZ_EV_NONE = 0,
  AZ_EV_LEARN_START,
  AZ_EV_DONE_POINT,
  AZ_EV_DONE_CURVE,
  AZ_EV_TIMEOUT
};

struct AzEvent {
  AzEventType type;
  float arm_deg;
  float alpha_counts;
  float err_deg;
  float learned_counts;
  float p_deg;
  float q_deg;
  float rms;
  int n;
};

// Mensagem: chamar com g_az_mux travado
static void push_msg_locked(const char* text) {
  int slot;
  if (g_msg_count < AZ_MSG_SLOTS) {
    slot = (g_msg_head + g_msg_count) % AZ_MSG_SLOTS;
    g_msg_count++;
  } else {
    // descarta a mais antiga
    slot = g_msg_head;
    g_msg_head = (g_msg_head + 1) % AZ_MSG_SLOTS;
  }
  strncpy(g_msgs[slot], text, AZ_MSG_LEN - 1);
  g_msgs[slot][AZ_MSG_LEN - 1] = 0;
}

/*
  Ajusta z = c + p*sen(th) + q*cos(th) aos pontos validos.
  y_k e o zero de cada ponto relativo a c_ref (sem saltos de 4096).
*/
static bool fit_curve_locked(
  float c_ref,
  float* c_new,
  float* p_new,
  float* q_new,
  float* rms_out,
  int* n_out
) {
  double M[3][3] = {{0}};
  double b[3] = {0};
  int n = 0;

  for (int i = 0; i < AZ_BINS; i++) {
    if (!g_bins[i].valid) continue;

    const double x[3] = {
      1.0,
      sin(g_bins[i].theta_rad),
      cos(g_bins[i].theta_rad)
    };
    const double y = wrap_half_counts(g_bins[i].zero_counts - c_ref);

    for (int r = 0; r < 3; r++) {
      for (int k = 0; k < 3; k++) M[r][k] += x[r] * x[k];
      b[r] += x[r] * y;
    }
    n++;
  }

  *n_out = n;

  if (n < AZ_FIT_MIN_BINS) return false;

  // Espalhamento: determinante da matriz normalizada por n
  double A[3][3];
  for (int r = 0; r < 3; r++)
    for (int k = 0; k < 3; k++) A[r][k] = M[r][k] / (double)n;

  const double det =
      A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
    - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
    + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

  if (!(det > (double)AZ_FIT_MIN_DET)) return false;

  // Cramer em M x = b
  const double detM = det * (double)n * (double)n * (double)n;
  double sol[3];
  for (int col = 0; col < 3; col++) {
    double T[3][3];
    for (int r = 0; r < 3; r++)
      for (int k = 0; k < 3; k++) T[r][k] = (k == col) ? b[r] : M[r][k];
    const double d =
        T[0][0] * (T[1][1] * T[2][2] - T[1][2] * T[2][1])
      - T[0][1] * (T[1][0] * T[2][2] - T[1][2] * T[2][0])
      + T[0][2] * (T[1][0] * T[2][1] - T[1][1] * T[2][0]);
    sol[col] = d / detM;
  }

  const float max_tilt = AZ_MAX_TILT_DEG * AZ_COUNTS_PER_DEG;
  if (!isfinite(sol[0]) || !isfinite(sol[1]) || !isfinite(sol[2])) return false;
  if (fabs(sol[1]) > max_tilt || fabs(sol[2]) > max_tilt) return false;

  double ss = 0.0;
  for (int i = 0; i < AZ_BINS; i++) {
    if (!g_bins[i].valid) continue;
    const double y = wrap_half_counts(g_bins[i].zero_counts - c_ref);
    const double yh = sol[0] + sol[1] * sin(g_bins[i].theta_rad)
                             + sol[2] * cos(g_bins[i].theta_rad);
    ss += (y - yh) * (y - yh);
  }
  const float rms = (float)sqrt(ss / (double)n);
  *rms_out = rms;

  if (n > 3 && rms > AZ_MAX_FIT_RMS_COUNTS) return false;

  *c_new = normalize_counts(c_ref + (float)sol[0]);
  *p_new = (float)sol[1];
  *q_new = (float)sol[2];
  return true;
}

/*
  Conclui o ajuste na posicao atual. Retorna em *shift_c quanto somar a c
  (aplicado fora da secao critica).
*/
static void conclude_locked(
  const AutozeroInput& in,
  bool success,
  float* shift_c,
  AzEvent* ev
) {
  const float arm_deg = in.arm_abs_rad * 57.29577951308232f;
  const float learned = g_d - g_d_at_learn_start;

  ev->arm_deg = arm_deg;
  ev->learned_counts = learned;
  ev->alpha_counts = g_alpha_slow;
  ev->err_deg = in.theta_err_rad * 57.29577951308232f;

  if (!success) {
    if (!g_fit_valid) {
      *shift_c += g_d;
      g_d = 0.0f;
      g_dirty = true;
    }
    ev->type = AZ_EV_TIMEOUT;
    g_state = AZ_DONE;
    return;
  }

  // Zero correto nesta posicao, em contagens absolutas
  const float c_now = in.base_offset_counts;
  const float z = normalize_counts(c_now + tilt_locked(in.arm_abs_rad) + g_d);

  const int idx = bin_index(in.arm_abs_rad);
  g_bins[idx].theta_rad = in.arm_abs_rad;
  g_bins[idx].zero_counts = z;
  g_bins[idx].valid = 1;
  g_dirty = true;

  float c_fit = 0.0f, p_fit = 0.0f, q_fit = 0.0f, rms = 0.0f;
  int n = 0;
  const bool ok = fit_curve_locked(c_now, &c_fit, &p_fit, &q_fit, &rms, &n);

  if (ok) {
    // Troca para a nova curva mantendo o zero total continuo aqui
    const float delta_c = wrap_half_counts(c_fit - c_now);
    const float tilt_old = tilt_locked(in.arm_abs_rad);
    g_tilt_p = p_fit;
    g_tilt_q = q_fit;
    g_fit_valid = true;
    const float tilt_new = tilt_locked(in.arm_abs_rad);
    g_d = tilt_old + g_d - delta_c - tilt_new;
    *shift_c += delta_c;

    ev->type = AZ_EV_DONE_CURVE;
    ev->p_deg = p_fit / AZ_COUNTS_PER_DEG;
    ev->q_deg = q_fit / AZ_COUNTS_PER_DEG;
    ev->rms = rms;
    ev->n = n;
  } else {
    // Sem curva confiavel: transfere o ajuste para c (igual ao ZTOP)
    *shift_c += g_d;
    g_d = 0.0f;

    ev->type = AZ_EV_DONE_POINT;
    ev->n = n;
  }

  g_state = AZ_DONE;
}

static void publish_event(const AzEvent& ev) {
  if (ev.type == AZ_EV_NONE) return;

  char msg[AZ_MSG_LEN];

  switch (ev.type) {
    case AZ_EV_LEARN_START:
      snprintf(msg, sizeof(msg),
        "[autozero] aprendendo em th_abs=%.1f deg (alpha=%+.2f cont, erro_braco=%+.2f deg)",
        ev.arm_deg, ev.alpha_counts, ev.err_deg);
      break;
    case AZ_EV_DONE_POINT:
      snprintf(msg, sizeof(msg),
        "[autozero] OK th_abs=%.1f deg ajuste=%+.2f cont (%+.3f deg) erro_braco=%+.2f deg | pontos=%d (curva precisa de %d+ espalhados)",
        ev.arm_deg, ev.learned_counts, ev.learned_counts / AZ_COUNTS_PER_DEG,
        ev.err_deg, ev.n, AZ_FIT_MIN_BINS);
      break;
    case AZ_EV_DONE_CURVE:
      snprintf(msg, sizeof(msg),
        "[autozero] OK th_abs=%.1f deg ajuste=%+.2f cont erro_braco=%+.2f deg | curva p=%+.3f q=%+.3f deg pontos=%d rms=%.2f cont",
        ev.arm_deg, ev.learned_counts, ev.err_deg, ev.p_deg, ev.q_deg, ev.n, ev.rms);
      break;
    case AZ_EV_TIMEOUT:
      snprintf(msg, sizeof(msg),
        "[autozero] sem convergencia em th_abs=%.1f deg (ajuste %+.2f cont, alpha medio %+.2f cont)",
        ev.arm_deg, ev.learned_counts, ev.alpha_counts);
      break;
    default:
      return;
  }

  portENTER_CRITICAL(&g_az_mux);
  push_msg_locked(msg);
  portEXIT_CRITICAL(&g_az_mux);
}

// ---------------------------------------------------------------------------

void autozero_begin() {
  Preferences prefs;

  memset(g_bins, 0, sizeof(g_bins));

  if (prefs.begin(AZ_NAMESPACE, true)) {
    g_enabled = prefs.getUChar("en", AZ_ENABLED_DEFAULT ? 1 : 0) != 0;
    g_tilt_p = prefs.getFloat("p", 0.0f);
    g_tilt_q = prefs.getFloat("q", 0.0f);
    g_fit_valid = prefs.getBool("fit", false);

    if (prefs.getBytesLength("bins") == sizeof(g_bins)) {
      prefs.getBytes("bins", g_bins, sizeof(g_bins));
    }

    prefs.end();
  }

  if (!isfinite(g_tilt_p) || !isfinite(g_tilt_q) || !g_fit_valid) {
    g_tilt_p = 0.0f;
    g_tilt_q = 0.0f;
    g_fit_valid = false;
  }

  int n = 0;
  for (int i = 0; i < AZ_BINS; i++) {
    if (g_bins[i].valid &&
        (!isfinite(g_bins[i].theta_rad) || !isfinite(g_bins[i].zero_counts))) {
      g_bins[i].valid = 0;
    }
    if (g_bins[i].valid) n++;
  }

  Serial.printf(
    "[autozero] %s curva=%d p=%+.3f q=%+.3f deg pontos=%d\n",
    g_enabled ? "ligado" : "desligado",
    g_fit_valid ? 1 : 0,
    g_tilt_p / AZ_COUNTS_PER_DEG,
    g_tilt_q / AZ_COUNTS_PER_DEG,
    n
  );
}

float autozero_tilt_counts(float arm_abs_rad) {
  portENTER_CRITICAL(&g_az_mux);
  const float t = tilt_locked(arm_abs_rad);
  portEXIT_CRITICAL(&g_az_mux);
  return t;
}

float autozero_extra_counts(float arm_abs_rad) {
  portENTER_CRITICAL(&g_az_mux);
  const float v = tilt_locked(arm_abs_rad) + g_d;
  g_last_arm_abs_rad = arm_abs_rad;
  portEXIT_CRITICAL(&g_az_mux);
  return v;
}

void autozero_on_lqr_enter() {
  portENTER_CRITICAL(&g_az_mux);
  g_state = AZ_WAIT;
  g_have_target = false;
  g_settle_t = 0.0f;
  g_learn_t = 0.0f;
  g_done_t = 0.0f;
  g_theta_dot_lp = 0.0f;
  g_d_decay = false;
  portEXIT_CRITICAL(&g_az_mux);
}

void autozero_on_lqr_exit() {
  portENTER_CRITICAL(&g_az_mux);
  g_state = AZ_IDLE;
  portEXIT_CRITICAL(&g_az_mux);
}

void autozero_on_stop() {
  float shift = 0.0f;

  portENTER_CRITICAL(&g_az_mux);
  if (!g_fit_valid && g_d != 0.0f) {
    shift = g_d;
    g_dirty = true;
  }
  g_d = 0.0f;
  g_d_decay = false;
  g_state = AZ_IDLE;
  portEXIT_CRITICAL(&g_az_mux);

  calibration_shift_offset_immediate(shift);
}

void autozero_update(const AutozeroInput& in) {
  float shift_c = 0.0f;
  AzEvent ev{};

  portENTER_CRITICAL(&g_az_mux);

  if (!g_enabled || g_state == AZ_IDLE) {
    portEXIT_CRITICAL(&g_az_mux);
    return;
  }

  float dt = in.dt;
  if (!(dt > 0.0f) || dt > 0.05f) dt = 0.0f;

  // Mudanca de alvo: recomeca nesta nova posicao
  if (!g_have_target ||
      fabsf(wrap_pi_local(in.target_rad - g_last_target_rad)) > 1e-4f) {
    if (g_have_target) {
      if (!g_fit_valid) {
        // sem curva, o melhor palpite e manter o ajuste como termo constante
        shift_c += g_d;
        g_d = 0.0f;
        g_dirty = g_dirty || (shift_c != 0.0f);
      } else {
        // com curva, o residuo local vai a zero durante o movimento
        g_d_decay = true;
      }
    }
    g_last_target_rad = in.target_rad;
    g_have_target = true;
    g_state = AZ_WAIT;
    g_settle_t = 0.0f;
    g_learn_t = 0.0f;
    g_done_t = 0.0f;
  }

  g_theta_dot_lp = lpf(g_theta_dot_lp, in.theta_dot_rad_s, dt, 0.3f);

  const float alpha_deg = in.alpha_raw_counts / AZ_COUNTS_PER_DEG;
  const bool stable =
    fabsf(alpha_deg) < AZ_STABLE_ALPHA_DEG &&
    fabsf(in.alpha_dot_rad_s) * 57.29577951308232f < AZ_STABLE_ALPHA_DOT_DEG_S &&
    fabsf(g_theta_dot_lp) * 57.29577951308232f < AZ_STABLE_THETA_DOT_DEG_S;

  const float max_step = AZ_MAX_RATE_COUNTS_S * dt;
  const float max_trim = AZ_MAX_TRIM_DEG * AZ_COUNTS_PER_DEG;

  switch (g_state) {
    case AZ_WAIT: {
      if (g_d_decay) {
        g_d -= clampf_az(g_d, -max_step, max_step);
      }

      if (in.ref_settled && stable) {
        g_settle_t += dt;
      } else {
        g_settle_t = 0.0f;
      }

      if (g_settle_t >= AZ_SETTLE_S) {
        g_state = AZ_LEARN;
        g_d_decay = false;
        g_done_t = 0.0f;
        g_alpha_lp = in.alpha_raw_counts;
        g_alpha_slow = in.alpha_raw_counts;
        if (g_learn_t == 0.0f) {
          g_d_at_learn_start = g_d;
          ev.type = AZ_EV_LEARN_START;
          ev.arm_deg = in.arm_abs_rad * 57.29577951308232f;
          ev.alpha_counts = in.alpha_raw_counts;
          ev.err_deg = in.theta_err_rad * 57.29577951308232f;
        }
      }
      break;
    }

    case AZ_LEARN: {
      if (!stable || !in.ref_settled) {
        // disturbio: pausa sem perder o que ja aprendeu
        g_state = AZ_WAIT;
        g_settle_t = 0.0f;
        g_done_t = 0.0f;
        break;
      }

      g_alpha_lp = lpf(g_alpha_lp, in.alpha_raw_counts, dt, AZ_LPF_TAU_S);
      g_alpha_slow = lpf(g_alpha_slow, in.alpha_raw_counts, dt, AZ_DONE_LPF_TAU_S);

      // Mesmo sinal do ZTOP: alpha medio positivo -> soma ao zero
      float step = g_alpha_lp * dt / AZ_TAU_LEARN_S;
      step = clampf_az(step, -max_step, max_step);
      g_d = clampf_az(g_d + step, -max_trim, max_trim);

      g_learn_t += dt;

      if (g_learn_t >= AZ_MIN_LEARN_S &&
          fabsf(g_alpha_slow) < AZ_DONE_ALPHA_COUNTS) {
        g_done_t += dt;
      } else {
        g_done_t = 0.0f;
      }

      if (g_done_t >= AZ_DONE_HOLD_S) {
        conclude_locked(in, true, &shift_c, &ev);
      } else if (g_learn_t >= AZ_TIMEOUT_S) {
        conclude_locked(in, false, &shift_c, &ev);
      }
      break;
    }

    case AZ_DONE:
    case AZ_IDLE:
    default:
      break;
  }

  portEXIT_CRITICAL(&g_az_mux);

  // c e d trocam no mesmo ciclo: o zero total continua igual
  calibration_shift_offset_immediate(shift_c);
  publish_event(ev);
}

bool autozero_save_if_dirty() {
  bool dirty;
  bool en, fit;
  float p, q;
  AzBin bins[AZ_BINS];

  portENTER_CRITICAL(&g_az_mux);
  dirty = g_dirty;
  en = g_enabled;
  fit = g_fit_valid;
  p = g_tilt_p;
  q = g_tilt_q;
  memcpy(bins, g_bins, sizeof(bins));
  portEXIT_CRITICAL(&g_az_mux);

  if (!dirty) return true;

  Preferences prefs;
  if (!prefs.begin(AZ_NAMESPACE, false)) {
    Serial.println("[autozero] erro ao abrir a flash");
    return false;
  }

  prefs.putUChar("en", en ? 1 : 0);
  prefs.putFloat("p", p);
  prefs.putFloat("q", q);
  prefs.putBool("fit", fit);
  const size_t w = prefs.putBytes("bins", bins, sizeof(bins));
  prefs.end();

  if (w != sizeof(bins)) {
    Serial.println("[autozero] erro ao salvar");
    return false;
  }

  portENTER_CRITICAL(&g_az_mux);
  g_dirty = false;
  portEXIT_CRITICAL(&g_az_mux);

  Serial.printf("[autozero] salvo curva=%d p=%+.3f q=%+.3f deg\n",
    fit ? 1 : 0, p / AZ_COUNTS_PER_DEG, q / AZ_COUNTS_PER_DEG);
  return true;
}

void autozero_set_enabled(bool enabled) {
  portENTER_CRITICAL(&g_az_mux);
  if (g_enabled != enabled) {
    g_enabled = enabled;
    g_dirty = true;
    if (enabled && g_state != AZ_IDLE) {
      g_state = AZ_WAIT;
      g_settle_t = 0.0f;
      g_learn_t = 0.0f;
      g_done_t = 0.0f;
    }
  }
  portEXIT_CRITICAL(&g_az_mux);
}

bool autozero_get_enabled() {
  portENTER_CRITICAL(&g_az_mux);
  const bool v = g_enabled;
  portEXIT_CRITICAL(&g_az_mux);
  return v;
}

void autozero_reset_curve() {
  portENTER_CRITICAL(&g_az_mux);
  // Se estiver equilibrando, preserva o zero total na posicao atual
  if (g_state != AZ_IDLE) {
    g_d += tilt_locked(g_last_arm_abs_rad);
  }
  g_tilt_p = 0.0f;
  g_tilt_q = 0.0f;
  g_fit_valid = false;
  memset(g_bins, 0, sizeof(g_bins));
  g_dirty = true;
  portEXIT_CRITICAL(&g_az_mux);
}

void autozero_format_status(char* buf, size_t len) {
  int n = 0;

  portENTER_CRITICAL(&g_az_mux);
  for (int i = 0; i < AZ_BINS; i++) if (g_bins[i].valid) n++;
  const bool en = g_enabled;
  const AzState st = g_state;
  const float d = g_d, p = g_tilt_p, q = g_tilt_q;
  const bool fit = g_fit_valid;
  const float a = g_alpha_slow, lt = g_learn_t;
  portEXIT_CRITICAL(&g_az_mux);

  snprintf(buf, len,
    "AZ en=%d estado=%s d=%+.2f cont alpha_medio=%+.2f cont t=%.1fs curva=%d p=%+.3f q=%+.3f deg pontos=%d",
    en ? 1 : 0, state_name(st), d, a, lt, fit ? 1 : 0,
    p / AZ_COUNTS_PER_DEG, q / AZ_COUNTS_PER_DEG, n);
}

bool autozero_pop_message(char* buf, size_t len) {
  bool got = false;

  portENTER_CRITICAL(&g_az_mux);
  if (g_msg_count > 0) {
    strncpy(buf, g_msgs[g_msg_head], len - 1);
    buf[len - 1] = 0;
    g_msg_head = (g_msg_head + 1) % AZ_MSG_SLOTS;
    g_msg_count--;
    got = true;
  }
  portEXIT_CRITICAL(&g_az_mux);

  return got;
}
