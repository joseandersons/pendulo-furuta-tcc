#include <Arduino.h>
#include <math.h>

#include "control.h"
#include "config.h"
#include "encoder.h"
#include "tipos.h"
#include "filas.h"
#include "ang.h"
#include "calibration.h"
#include "motor.h"
#include "lqr.h"
#include "swing_up.h"
#include "observer.h"
#include "driver_tmc.h"
#include "microstep_control.h"
#include "autozero.h"

#ifndef ALPHA_DISABLE_RAD
#define ALPHA_DISABLE_RAD 0.34906585040f   // 20 graus
#endif

static uint32_t prev_us = 0;
static uint32_t g_last_processed_sample_id = 0;

static volatile bool g_control_enabled = false;
static volatile bool g_start_requested = false;

/*
  O encoder do braço permanece contínuo para que a velocidade seja calculada
  sem saltos: 350, 360, 370, 720... graus.

  A referência representa uma orientação circular:
    0 graus = 360 graus = 720 graus
    -1 grau = 359 graus

  O LQR usa sin(theta - theta_ref) para evitar o ponto de corte do wrapPi().
  Isso remove trancos em +/-180 graus relativos à referência.
*/
static volatile float g_theta_target_rad = 0.0f;
static volatile float g_theta_ref_rad = 0.0f;

static portMUX_TYPE g_theta_reference_mux = portMUX_INITIALIZER_UNLOCKED;

static float alpha_hist[ALPHA_DOT_WINDOW];
static uint32_t alpha_time_hist_us[ALPHA_DOT_WINDOW];
static int alpha_hist_index = 0;
static bool alpha_hist_ready = false;

static float g_last_lqr_accel_rad_s2 = 0.0f;

enum ControlMode {
  CONTROL_MODE_STOPPED = 0,
  CONTROL_MODE_SWING_UP,
  CONTROL_MODE_LQR
};

static volatile ControlMode g_mode = CONTROL_MODE_STOPPED;

static float clampf_local(float x, float min_v, float max_v) {
  if (x < min_v) return min_v;
  if (x > max_v) return max_v;
  return x;
}

static float shortest_angular_difference(
  float target_rad,
  float current_rad
) {
  /*
    Retorna quanto current_rad precisa caminhar para chegar a target_rad pelo
    menor caminho. O resultado fica em [-pi, +pi].
  */
  return wrapPi(target_rad - current_rad);
}

void control_set_theta_target_rad(float theta_target_rad) {
  /*
    O alvo é uma orientação circular, não uma quantidade de voltas.

    Exemplos:
      360 graus -> 0 grau
      359 graus -> -1 grau
      720 graus -> 0 grau
  */
  const float normalized_target_rad = wrapPi(theta_target_rad);

  portENTER_CRITICAL(&g_theta_reference_mux);
  g_theta_target_rad = normalized_target_rad;
  portEXIT_CRITICAL(&g_theta_reference_mux);
}

float control_get_theta_target_rad() {
  portENTER_CRITICAL(&g_theta_reference_mux);
  const float target_rad = g_theta_target_rad;
  portEXIT_CRITICAL(&g_theta_reference_mux);

  return target_rad;
}

float control_get_theta_ref_rad() {
  portENTER_CRITICAL(&g_theta_reference_mux);
  const float ref_rad = g_theta_ref_rad;
  portEXIT_CRITICAL(&g_theta_reference_mux);

  return ref_rad;
}

void control_reset_theta_reference() {
  portENTER_CRITICAL(&g_theta_reference_mux);
  g_theta_target_rad = 0.0f;
  g_theta_ref_rad = 0.0f;
  portEXIT_CRITICAL(&g_theta_reference_mux);
}

static float theta_reference_update(float dt) {
  portENTER_CRITICAL(&g_theta_reference_mux);
  const float target_rad = g_theta_target_rad;
  float ref_rad = g_theta_ref_rad;
  portEXIT_CRITICAL(&g_theta_reference_mux);

  const float error_rad = shortest_angular_difference(
    target_rad,
    ref_rad
  );

  const float max_step_rad = THETA_REF_RATE_LIMIT_RAD_S * dt;

  const float step_rad = clampf_local(
    error_rad,
    -max_step_rad,
    max_step_rad
  );

  ref_rad = wrapPi(ref_rad + step_rad);

  portENTER_CRITICAL(&g_theta_reference_mux);
  g_theta_ref_rad = ref_rad;
  portEXIT_CRITICAL(&g_theta_reference_mux);

  return ref_rad;
}

static float compute_dt(uint32_t stamp_us) {
  if (prev_us == 0) {
    prev_us = stamp_us;
    return PERIOD_CONTROL_MS * 1e-3f;
  }

  uint32_t diff_us = stamp_us - prev_us;
  prev_us = stamp_us;

  float dt = diff_us * 1e-6f;

  if (dt <= 0.0f) dt = PERIOD_CONTROL_MS * 1e-3f;
  if (dt > 0.05f) dt = 0.05f;

  return dt;
}

static void alpha_velocity_reset() {
  for (int i = 0; i < ALPHA_DOT_WINDOW; i++) {
    alpha_hist[i] = 0.0f;
    alpha_time_hist_us[i] = 0;
  }

  alpha_hist_index = 0;
  alpha_hist_ready = false;
}

static float alpha_velocity_update(float alpha_rad, uint32_t stamp_us) {
  int old_index = alpha_hist_index;

  float old_alpha = alpha_hist[old_index];
  uint32_t old_time_us = alpha_time_hist_us[old_index];

  alpha_hist[old_index] = alpha_rad;
  alpha_time_hist_us[old_index] = stamp_us;

  alpha_hist_index = (alpha_hist_index + 1) % ALPHA_DOT_WINDOW;

  if (!alpha_hist_ready) {
    if (alpha_hist_index == 0) {
      alpha_hist_ready = true;
    }

    return 0.0f;
  }

  uint32_t dt_us = stamp_us - old_time_us;

  if (dt_us == 0) return 0.0f;

  float d_alpha = wrapPi(alpha_rad - old_alpha);
  float dt = dt_us * 1e-6f;

  if (dt <= 0.0f) return 0.0f;

  return d_alpha / dt;
}

static void reset_state_on_start() {
  prev_us = 0;
  g_last_processed_sample_id = 0;

  alpha_velocity_reset();

  lqr_reset();
  swing_up_reset();
  observer_reset();

  g_last_lqr_accel_rad_s2 = 0.0f;

  encoder_reset_braco_zero();

  motor_reset_theta_est();
  motor_stop_immediate();

  control_reset_theta_reference();
  microstep_control_reset_runtime();

  g_mode = CONTROL_MODE_SWING_UP;
}

void control_request_start() {
  tmc_init();
  microstep_control_apply_startup(0.0f);
  g_start_requested = true;
  g_mode = CONTROL_MODE_SWING_UP;
}

void control_stop() {
  g_control_enabled = false;
  g_start_requested = false;
  g_mode = CONTROL_MODE_STOPPED;

  lqr_reset();
  swing_up_reset();
  observer_reset();

  g_last_lqr_accel_rad_s2 = 0.0f;

  motor_stop_immediate();

  microstep_control_reset_runtime();
  microstep_control_force_fine(0.0f);

  // Transfere o ajuste do autozero para o zero salvo
  autozero_on_stop();

  // Grava na flash o zero ajustado no topo (fora do equilibrio, sem afetar o controle)
  calibration_save_if_dirty();
  autozero_save_if_dirty();
}

static void send_telemetry(
  const Obs& o,
  float alpha_rad,
  float theta_rad,
  float alpha_dot_rad_s,
  float theta_dot_rad_s,
  float u_rad_s2,
  float vel_cmd_rad_s,
  bool enabled
) {
  static uint32_t last_tx = 0;

  if (millis() - last_tx < 20) {
    return;
  }

  last_tx = millis();

  TelePkt pkt{};

  pkt.t_ms = o.stamp_us / 1000u;
  pkt.alpha_c2 = (int16_t)lroundf(rad2deg(alpha_rad) * 100.0f);

  /*
    Somente a telemetria visual é limitada a [-180, +180] graus.
    Internamente, o encoder do braço continua acumulado.
  */
  const float theta_visual_rad = wrapPi(theta_rad);
  pkt.theta_c2 = (int16_t)lroundf(rad2deg(theta_visual_rad) * 100.0f);

  pkt.alpha_dot_c2 = (int16_t)lroundf(rad2deg(alpha_dot_rad_s) * 100.0f);
  pkt.theta_dot_c2 = (int16_t)lroundf(rad2deg(theta_dot_rad_s) * 100.0f);
  pkt.u_c2 = (int16_t)lroundf(u_rad_s2 * 100.0f);
  pkt.vel_c2 = (int16_t)lroundf(vel_cmd_rad_s * 100.0f);
  pkt.enabled = enabled ? 1 : 0;

  xQueueSend(qTel, &pkt, 0);
}

static void control_step(const Obs& o) {
  float dt = compute_dt(o.stamp_us);

  // Zero com parte fracionaria (calibracao no topo), em contagens
  const float zero_offset_counts = calibration_get_alpha_offset_counts(dt);

  // Angulo absoluto do braco (para a curva de inclinacao do autozero)
  const float arm_abs_rad =
    ((float)o.braco_abs_counts * TAU_ANG) / CPR_BRACO;

  // Zero total = c + curva(theta_abs) + ajuste local do autozero
  const float zero_total_counts =
    zero_offset_counts + autozero_extra_counts(arm_abs_rad);

  float alpha_counts = (float)o.pendulo_counts - zero_total_counts;
  const float half_cpr = 0.5f * CPR_PENDULO;

  while (alpha_counts > half_cpr) alpha_counts -= CPR_PENDULO;
  while (alpha_counts < -half_cpr) alpha_counts += CPR_PENDULO;

  // Mesma conta com a leitura bruta (sem histerese): usada so pelo autozero
  float alpha_raw_counts = (float)o.pendulo_raw_counts - zero_total_counts;
  while (alpha_raw_counts > half_cpr) alpha_raw_counts -= CPR_PENDULO;
  while (alpha_raw_counts < -half_cpr) alpha_raw_counts += CPR_PENDULO;

  float alpha_rad = (alpha_counts * TAU_ANG) / CPR_PENDULO;
  alpha_rad = wrapPi(alpha_rad);

  float alpha_dot_rad_s = alpha_velocity_update(alpha_rad, o.stamp_us);

  float theta_rad = o.braco_rad;
  float theta_dot_rad_s = o.braco_dot_rad_s;

  LQRState state{};
  state.theta = theta_rad;
  state.alpha = alpha_rad;
  state.theta_dot = theta_dot_rad_s;
  state.alpha_dot = alpha_dot_rad_s;

  if (g_mode == CONTROL_MODE_SWING_UP) {
    SwingUpOutput sw = swing_up_update(state, dt); //VERIFICAE

    motor_set_velocity_adaptive_rad_s(
      sw.velocity_cmd_rad_s,
      state.alpha,
      state.alpha_dot
    );

    if (!sw.active) {
      swing_up_reset();
      lqr_reset();

      observer_init(
        state.theta,
        state.alpha,
        state.theta_dot,
        state.alpha_dot
      );

      g_last_lqr_accel_rad_s2 = 0.0f;

      g_mode = CONTROL_MODE_LQR;

      // Comeca a ajustar o zero assim que o equilibrio assentar
      autozero_on_lqr_enter();

      microstep_control_reset_runtime();
      microstep_control_update_lqr(state.alpha, state.alpha_dot);
    }

    send_telemetry(
      o,
      alpha_rad,
      theta_rad,
      alpha_dot_rad_s,
      theta_dot_rad_s,
      0.0f,
      sw.velocity_cmd_rad_s,
      false
    );

    return;
  }

  if (g_mode == CONTROL_MODE_LQR) {
    
    if (fabsf(state.alpha) > ALPHA_DISABLE_RAD) {
      lqr_reset();
      swing_up_reset();
      observer_reset();

      g_last_lqr_accel_rad_s2 = 0.0f;

      motor_stop_immediate();

      microstep_control_reset_runtime();
      microstep_control_force_fine(state.alpha);

      autozero_on_lqr_exit();

      g_mode = CONTROL_MODE_SWING_UP;

      SwingUpOutput sw = swing_up_update(state, dt);

      motor_set_velocity_adaptive_rad_s(
        sw.velocity_cmd_rad_s,
        state.alpha,
        state.alpha_dot
      );

      send_telemetry(
        o,
        alpha_rad,
        theta_rad,
        alpha_dot_rad_s,
        theta_dot_rad_s,
        0.0f,
        sw.velocity_cmd_rad_s,
        false
      );

      return;
    }

    LQRState state_est = observer_update(
      theta_rad,
      alpha_rad,
      g_last_lqr_accel_rad_s2
    );

    microstep_control_update_lqr(state_est.alpha, state_est.alpha_dot); //VERIFICAE

    float theta_ref_rad = theta_reference_update(dt);

    LQRState state_lqr = state_est;

    state_lqr.theta = sinf(state_est.theta - theta_ref_rad);

    LQROutput lqr = lqr_update(state_lqr, dt);

    motor_set_acceleration_adaptive_rad_s2(
      lqr.accel_cmd_rad_s2,
      state_est.alpha,
      state_est.alpha_dot
    );

    g_last_lqr_accel_rad_s2 = lqr.accel_cmd_rad_s2;

    {
      const float target_rad = control_get_theta_target_rad();

      AutozeroInput az{};
      az.dt = dt;
      az.alpha_raw_counts = alpha_raw_counts;
      az.alpha_dot_rad_s = alpha_dot_rad_s;
      az.theta_dot_rad_s = theta_dot_rad_s;
      az.theta_err_rad = wrapPi(state_est.theta - theta_ref_rad);
      az.target_rad = target_rad;
      az.ref_settled =
        fabsf(shortest_angular_difference(target_rad, theta_ref_rad)) < 1e-4f;
      az.arm_abs_rad = arm_abs_rad;
      az.base_offset_counts = zero_offset_counts;

      autozero_update(az);
    }

    send_telemetry(
      o,
      state_est.alpha,
      state_est.theta,
      state_est.alpha_dot,
      state_est.theta_dot,
      lqr.accel_cmd_rad_s2,
      motor_get_velocity_cmd_rad_s(),
      true
    );

    return;
  }
}

void control_task_start() {
  xTaskCreatePinnedToCore([](void*) {
    Serial.println("[control] start");

    encoder_register_control_task(
      xTaskGetCurrentTaskHandle()
    );

    for (;;) {
      if (!g_control_enabled) {
        if (g_start_requested) {
          Serial.println("[control] iniciando sem calibração");

          ulTaskNotifyTake(pdTRUE, 0);
          reset_state_on_start();

          Serial.println("[control] encoder absoluto pronto, controle habilitado");

          g_control_enabled = true;
          g_start_requested = false;
        } else {
          vTaskDelay(pdMS_TO_TICKS(10));
        }

        continue;
      }

      if (
        ulTaskNotifyTake(
          pdTRUE,
          pdMS_TO_TICKS(20)
        )
        == 0
      ) {
        continue;
      }

      Obs o{};

      if (!encoder_get_obs(&o)) {
        continue;
      }

      if (
        o.sample_id == 0
        || o.sample_id == g_last_processed_sample_id
      ) {
        continue;
      }

      g_last_processed_sample_id = o.sample_id;
      control_step(o);
    }
  }, "thCONTROL", 4096, nullptr, PRIO_CONTROL, nullptr, CORE_CTRL);
}
