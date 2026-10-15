#include "swing_up.h"
#include "config.h"
#include "ang.h"
#include "motor.h"
#include "driver_tmc.h"
#include <math.h>

enum SwingUpPhase {
  SWING_PHASE_INIT = 0,
  SWING_PHASE_LEFT,
  SWING_PHASE_BACK,
  SWING_PHASE_NORMAL
};

static SwingUpPhase g_phase = SWING_PHASE_INIT;

static float g_direction = 1.0f;
static float g_theta_start = 0.0f;
static float g_theta_left_target = 0.0f;

static uint32_t g_phase_start_ms = 0;
static uint32_t g_last_switch_ms = 0;

static bool g_swing_microsteps_applied = false;

static constexpr float LEFT_OFFSET_RAD = 0.25f;
static constexpr float THETA_TOL_RAD = 0.03f;

static constexpr float LEFT_VEL_RAD_S = 0.6f;
static constexpr float BACK_VEL_RAD_S = 1.2f;

static constexpr uint32_t LEFT_TIMEOUT_MS = 1200;
static constexpr uint32_t BACK_TIMEOUT_MS = 1600;

static constexpr uint32_t NORMAL_SWITCH_MS = 250;
static constexpr float NORMAL_VEL_RAD_S = 0.6f;

static void apply_swing_up_microsteps() {
  if (g_swing_microsteps_applied) {
    return;
  }

  if (!tmc_is_ready()) {
    return;
  }

  if (tmc_get_microsteps() != SWING_UP_MICROSTEPS) {
    if (!tmc_set_microsteps(SWING_UP_MICROSTEPS)) {
      Serial.println("[swing-up] ERRO: nao conseguiu aplicar 16 microsteps");
      return;
    }
  }

  motor_set_microsteps_runtime(SWING_UP_MICROSTEPS);

  g_swing_microsteps_applied = true;

  Serial.print("[swing-up] microsteps = ");
  Serial.println(SWING_UP_MICROSTEPS);
}

static void restore_lqr_microsteps() {
  if (!g_swing_microsteps_applied) {
    return;
  }

  if (!tmc_is_ready()) {
    return;
  }

  if (tmc_get_microsteps() != LQR_FINE_MICROSTEPS) {
    if (!tmc_set_microsteps(LQR_FINE_MICROSTEPS)) {
      Serial.println("[swing-up] ERRO: nao conseguiu restaurar 256 microsteps");
      return;
    }
  }

  motor_set_microsteps_runtime(LQR_FINE_MICROSTEPS);

  g_swing_microsteps_applied = false;

  Serial.print("[lqr] microsteps = ");
  Serial.println(LQR_FINE_MICROSTEPS);
}

void swing_up_reset() {
  g_phase = SWING_PHASE_INIT;
  g_direction = 1.0f;
  g_theta_start = 0.0f;
  g_theta_left_target = 0.0f;
  g_phase_start_ms = millis();
  g_last_switch_ms = millis();

  /*
    Não restaura 256 aqui, porque swing_up_reset() normalmente é chamado
    antes de iniciar outro swing-up. Quem restaura 256 é a saída para LQR.
  */
}

SwingUpOutput swing_up_update(const LQRState& state, float dt) {
  (void)dt;

  SwingUpOutput out{};
  uint32_t now = millis();

  apply_swing_up_microsteps();

  out.active = true;
  out.velocity_cmd_rad_s = 0.0f;

  float vel_limit = motor_get_effective_velocity_max_rad_s();

  /*
    Se o pendulo ja chegou perto da vertical, sai do swing-up
    e restaura o controle fino em 256 microsteps.
  */
  if (fabsf(state.alpha) < ALPHA_ENABLE_RAD) {
    restore_lqr_microsteps();

    out.active = false;
    out.velocity_cmd_rad_s = 0.0f;
    return out;
  }

  if (g_phase == SWING_PHASE_INIT) {
    g_theta_start = state.theta;
    g_theta_left_target = g_theta_start - LEFT_OFFSET_RAD;
    g_phase = SWING_PHASE_LEFT;
    g_phase_start_ms = now;
  }

  if (g_phase == SWING_PHASE_LEFT) {
    bool chegou_na_esquerda = state.theta <= g_theta_left_target;
    bool timeout = (now - g_phase_start_ms) > LEFT_TIMEOUT_MS;

    if (!chegou_na_esquerda && !timeout) {
      out.velocity_cmd_rad_s = constrain(
        -LEFT_VEL_RAD_S,
        -vel_limit,
        vel_limit
      );
      return out;
    }

    g_phase = SWING_PHASE_BACK;
    g_phase_start_ms = now;
  }

  if (g_phase == SWING_PHASE_BACK) {
    bool voltou_para_inicio = state.theta >= (g_theta_start - THETA_TOL_RAD);
    bool timeout = (now - g_phase_start_ms) > BACK_TIMEOUT_MS;

    if (!voltou_para_inicio && !timeout) {
      out.velocity_cmd_rad_s = constrain(
        BACK_VEL_RAD_S,
        -vel_limit,
        vel_limit
      );
      return out;
    }

    g_phase = SWING_PHASE_NORMAL;
    g_direction = 1.0f;
    g_last_switch_ms = now;
  }

  float alpha_from_bottom = fabsf(fabsf(state.alpha) - PI);

  if (alpha_from_bottom < 0.25f) {
    if (now - g_last_switch_ms > NORMAL_SWITCH_MS) {
      g_direction = -g_direction;
      g_last_switch_ms = now;
    }
  }

  out.velocity_cmd_rad_s = constrain(
    g_direction * NORMAL_VEL_RAD_S,
    -vel_limit,
    vel_limit
  );

  out.active = true;

  return out;
}

