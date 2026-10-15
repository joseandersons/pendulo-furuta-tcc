#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include "calibration.h"
#include "encoder.h"
#include "config.h"
#include "autozero.h"
#include "tipos.h"

#define CALIBRATION_NAMESPACE "furuta-cal"
#define CALIBRATION_KEY "alpha-zero"        // legado: contagens inteiras
#define CALIBRATION_KEY_F "alpha-zero-f"    // atual: contagens com fracao
#define CALIBRATION_SAMPLES 200

/*
  Zero do pendulo em contagens do AS5600, com parte fracionaria.

  g_offset_target_counts:  valor desejado (definido por CALIB ou ZTOP)
  g_offset_current_counts: valor usado pelo controle; segue o alvo com
                           taxa limitada para nao dar tranco quando o zero
                           e ajustado com o pendulo equilibrado.
*/
static float g_offset_target_counts = 0.0f;
static float g_offset_current_counts = 0.0f;
static bool g_offset_dirty = false;

static portMUX_TYPE mux_calibration =
  portMUX_INITIALIZER_UNLOCKED;

static float normalize_counts_f(float counts) {
  counts = fmodf(counts, CPR_PENDULO);

  if (counts < 0.0f) {
    counts += CPR_PENDULO;
  }

  return counts;
}

static float wrap_counts_diff_f(float diff) {
  const float half = 0.5f * CPR_PENDULO;

  while (diff > half) diff -= CPR_PENDULO;
  while (diff < -half) diff += CPR_PENDULO;

  return diff;
}

static void set_zero_offset_immediate(float counts) {
  counts = normalize_counts_f(counts);

  portENTER_CRITICAL(&mux_calibration);
  g_offset_target_counts = counts;
  g_offset_current_counts = counts;
  portEXIT_CRITICAL(&mux_calibration);
}

static bool save_zero_offset(float counts) {
  Preferences preferences;

  if (!preferences.begin(CALIBRATION_NAMESPACE, false)) {
    return false;
  }

  size_t bytes_f = preferences.putFloat(CALIBRATION_KEY_F, counts);
  preferences.putInt(CALIBRATION_KEY, (int32_t)lroundf(counts));
  preferences.end();

  return bytes_f > 0;
}

void calibration_begin() {
  Preferences preferences;

  float stored_offset = 0.0f;

  if (preferences.begin(CALIBRATION_NAMESPACE, true)) {
    if (preferences.isKey(CALIBRATION_KEY_F)) {
      stored_offset = preferences.getFloat(CALIBRATION_KEY_F, 0.0f);
    } else {
      stored_offset = (float)preferences.getInt(CALIBRATION_KEY, 0);
    }

    preferences.end();
  }

  if (!isfinite(stored_offset)) {
    stored_offset = 0.0f;
  }

  set_zero_offset_immediate(stored_offset);

  Serial.printf(
    "[calib] offset absoluto carregado=%.3f\n",
    calibration_get_target_offset_counts()
  );

  autozero_begin();
}

bool calibrar_zero_por_fundo() {
  double sum_sin = 0.0;
  double sum_cos = 0.0;
  int valid_samples = 0;

  const int32_t full_rev_counts =
    (int32_t)lroundf(CPR_PENDULO);

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int32_t counts = 0;

    if (encoder_pendulo_get_absolute_count(&counts)) {
      double angle =
        ((double)counts * TWO_PI)
        / (double)full_rev_counts;

      sum_sin += sin(angle);
      sum_cos += cos(angle);
      valid_samples++;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }

  if (valid_samples < CALIBRATION_SAMPLES / 2) {
    return false;
  }

  double mean_angle = atan2(sum_sin, sum_cos);

  if (mean_angle < 0.0) {
    mean_angle += TWO_PI;
  }

  // Mantem a fracao: a media de muitas leituras e mais precisa que 1 contagem
  const float fundo_counts = normalize_counts_f(
    (float)(mean_angle * (double)full_rev_counts / TWO_PI)
  );

  // Zero do topo na posicao atual do braco. A curva de inclinacao (se ja
  // aprendida) e descontada para que c continue sendo o termo constante.
  float tilt_here_counts = 0.0f;
  {
    Obs o{};
    if (encoder_get_obs(&o)) {
      const float arm_abs_rad =
        ((float)o.braco_abs_counts * TWO_PI) / CPR_BRACO;
      tilt_here_counts = autozero_tilt_counts(arm_abs_rad);
    }
  }

  const float topo_offset_counts = normalize_counts_f(
    fundo_counts + 0.5f * CPR_PENDULO - tilt_here_counts
  );

  set_zero_offset_immediate(topo_offset_counts);

  portENTER_CRITICAL(&mux_calibration);
  g_offset_dirty = false;
  portEXIT_CRITICAL(&mux_calibration);

  if (!save_zero_offset(topo_offset_counts)) {
    return false;
  }

  Serial.printf(
    "[calib] fundo=%.3f topo_offset=%.3f amostras=%d\n",
    fundo_counts,
    topo_offset_counts,
    valid_samples
  );

  return true;
}

bool calibrar_zero_ajuste_topo(float delta_deg) {
  /*
    delta_deg = media de alpha medida com o pendulo equilibrado em cima.
    Como alpha = contagens - offset, somar delta ao offset leva essa media
    para zero. O controle passa ao novo valor de forma gradual.
  */
  if (!isfinite(delta_deg) || fabsf(delta_deg) > ZERO_TOP_MAX_ADJUST_DEG) {
    return false;
  }

  const float delta_counts = delta_deg * CPR_PENDULO / 360.0f;

  portENTER_CRITICAL(&mux_calibration);
  g_offset_target_counts = normalize_counts_f(
    g_offset_target_counts + delta_counts
  );
  g_offset_dirty = true;
  portEXIT_CRITICAL(&mux_calibration);

  return true;
}

bool calibration_save_if_dirty() {
  bool dirty;
  float target;

  portENTER_CRITICAL(&mux_calibration);
  dirty = g_offset_dirty;
  target = g_offset_target_counts;
  portEXIT_CRITICAL(&mux_calibration);

  if (!dirty) {
    return true;
  }

  if (!save_zero_offset(target)) {
    Serial.println("[calib] erro ao salvar zero do topo");
    return false;
  }

  portENTER_CRITICAL(&mux_calibration);
  g_offset_dirty = false;
  portEXIT_CRITICAL(&mux_calibration);

  Serial.printf("[calib] zero do topo salvo offset=%.3f\n", target);
  return true;
}

float calibration_get_alpha_offset_counts(float dt) {
  if (!(dt > 0.0f) || dt > 0.05f) {
    dt = 0.0f;
  }

  const float max_step = ZERO_SLEW_COUNTS_PER_S * dt;

  portENTER_CRITICAL(&mux_calibration);

  const float diff = wrap_counts_diff_f(
    g_offset_target_counts - g_offset_current_counts
  );

  if (fabsf(diff) <= max_step) {
    g_offset_current_counts = g_offset_target_counts;
  } else {
    g_offset_current_counts = normalize_counts_f(
      g_offset_current_counts + (diff > 0.0f ? max_step : -max_step)
    );
  }

  const float current = g_offset_current_counts;

  portEXIT_CRITICAL(&mux_calibration);

  return current;
}

void calibration_shift_offset_immediate(float delta_counts) {
  if (!isfinite(delta_counts) || delta_counts == 0.0f) {
    return;
  }

  portENTER_CRITICAL(&mux_calibration);
  g_offset_target_counts =
    normalize_counts_f(g_offset_target_counts + delta_counts);
  g_offset_current_counts =
    normalize_counts_f(g_offset_current_counts + delta_counts);
  g_offset_dirty = true;
  portEXIT_CRITICAL(&mux_calibration);
}

float calibration_get_target_offset_counts() {
  portENTER_CRITICAL(&mux_calibration);
  const float v = g_offset_target_counts;
  portEXIT_CRITICAL(&mux_calibration);
  return v;
}

float calibration_get_current_offset_counts() {
  portENTER_CRITICAL(&mux_calibration);
  const float v = g_offset_current_counts;
  portEXIT_CRITICAL(&mux_calibration);
  return v;
}

bool calibration_is_dirty() {
  portENTER_CRITICAL(&mux_calibration);
  const bool v = g_offset_dirty;
  portEXIT_CRITICAL(&mux_calibration);
  return v;
}

int32_t get_zero_offset() {
  return (int32_t)lroundf(calibration_get_target_offset_counts());
}
