#include "microstep_control.h"

#include <Arduino.h>
#include <math.h>

#include "ang.h"
#include "config.h"
#include "driver_tmc.h"
#include "fuzzy.h"
#include "lqr.h"
#include "motor.h"

static portMUX_TYPE g_microstep_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool g_auto_enabled = true;
static volatile uint16_t g_manual_microsteps = LQR_MANUAL_MICROSTEPS_DEFAULT;

enum class MicrostepState {
  NORMAL,
  HOLD_DISTURBANCE,
  RAMP_UP,
  RECOVERY_BLOCKED
};

static MicrostepState g_microstep_state = MicrostepState::NORMAL;

static uint32_t g_lqr_disturbance_start_ms = 0;
static uint32_t g_last_ramp_step_ms = 0;
static uint32_t g_recovery_block_until_ms = 0;
static uint32_t g_stable_since_ms = 0;

static uint8_t g_ramp_index = 0;
static uint16_t g_disturbance_microsteps = LQR_DISTURBANCE_MICROSTEPS;
static uint16_t g_lqr_current_microsteps = 0;
static uint32_t g_lqr_status_last_ms = 0;

static constexpr uint16_t RAMP_UP_STEPS[] = {16, 32, 64, 128, 256};
static constexpr uint8_t RAMP_UP_STEPS_COUNT = 5;

bool microstep_control_is_valid_microsteps(uint16_t microsteps) {
  return microsteps == 8   ||
         microsteps == 16  ||
         microsteps == 32  ||
         microsteps == 64  ||
         microsteps == 128 ||
         microsteps == 256;
}

static bool microstep_control_get_settings(
  bool* auto_enabled,
  uint16_t* manual_microsteps
) {
  if (auto_enabled == nullptr || manual_microsteps == nullptr) {
    return false;
  }

  portENTER_CRITICAL(&g_microstep_mux);
  *auto_enabled = g_auto_enabled;
  *manual_microsteps = g_manual_microsteps;
  portEXIT_CRITICAL(&g_microstep_mux);

  return true;
}

bool microstep_control_get_auto_enabled() {
  bool auto_enabled;
  uint16_t manual_microsteps;

  microstep_control_get_settings(&auto_enabled, &manual_microsteps);

  return auto_enabled;
}

uint16_t microstep_control_get_manual_microsteps() {
  bool auto_enabled;
  uint16_t manual_microsteps;

  microstep_control_get_settings(&auto_enabled, &manual_microsteps);

  return manual_microsteps;
}

uint16_t microstep_control_get_current_microsteps() {
  return g_lqr_current_microsteps;
}

static uint8_t ramp_index_for_microsteps(uint16_t microsteps) {
  for (uint8_t i = 0; i < RAMP_UP_STEPS_COUNT; i++) {
    if (RAMP_UP_STEPS[i] >= microsteps) {
      return i;
    }
  }

  return RAMP_UP_STEPS_COUNT - 1;
}

static bool use_disturbance_profile_for(uint16_t microsteps) {
  /*
    O perfil de ganhos de disturbio e mais agressivo. Por isso, ele so e usado
    quando o supervisor fuzzy pede uma reducao forte de micropassos.
  */
  return microsteps <= 64;
}

static bool is_rearm_stable(float alpha_rad, float alpha_dot_rad_s) {
  const float alpha_deg = fabsf(rad2deg(wrapPi(alpha_rad)));
  const float alpha_dot_deg_s = fabsf(rad2deg(alpha_dot_rad_s));

  return alpha_deg <= LQR_DISTURBANCE_REARM_ALPHA_DEG &&
         alpha_dot_deg_s <= LQR_DISTURBANCE_REARM_ALPHA_DOT_DEG_S;
}

static void microstep_control_print_status_line(
  const char* reason,
  float alpha_rad,
  float alpha_dot_rad_s,
  bool disturbance_profile
) {
  bool auto_enabled;
  uint16_t manual_microsteps;

  microstep_control_get_settings(&auto_enabled, &manual_microsteps);

  FuzzySupervisorOutput fuzzy = fuzzy_evaluate_microsteps(
    alpha_rad,
    alpha_dot_rad_s
  );

  Serial.print("[lqr-ms] ");
  Serial.print(reason);

  Serial.print(" | alpha_deg=");
  Serial.print(rad2deg(alpha_rad), 2);

  Serial.print(" | alpha_dot_deg_s=");
  Serial.print(rad2deg(alpha_dot_rad_s), 1);

  Serial.print(" | fuzzy_ms=");
  Serial.print(fuzzy.microsteps);

  Serial.print(" | fuzzy_crisp=");
  Serial.print(fuzzy.crisp_microsteps, 1);

  Serial.print(" | auto=");
  Serial.print(auto_enabled ? 1 : 0);

  Serial.print(" | manual=");
  Serial.print(manual_microsteps);

  Serial.print(" | modo=");

  if (!auto_enabled) {
    Serial.print("MANUAL");
  } else {
    switch (g_microstep_state) {
      case MicrostepState::NORMAL:
        Serial.print("NORMAL");
        break;

      case MicrostepState::HOLD_DISTURBANCE:
        Serial.print("SEGURANDO_DISTURBIO");
        break;

      case MicrostepState::RAMP_UP:
        Serial.print("RAMPA_SUBINDO");
        break;

      case MicrostepState::RECOVERY_BLOCKED:
        Serial.print("RECUPERACAO_BLOQUEADA");
        break;
    }
  }

  Serial.print(" | tmc=");
  Serial.print(tmc_get_microsteps());

  Serial.print(" | motor=");
  Serial.print(motor_get_microsteps_runtime());

  Serial.print(" | ganhos=");
  Serial.println(disturbance_profile ? "disturbio" : "interface");
}

void microstep_control_print_status(float alpha_rad) {
  microstep_control_print_status_line(
    "status",
    alpha_rad,
    0.0f,
    lqr_is_disturbance_profile()
  );
}

static void microstep_control_print_status_throttled(
  float alpha_rad,
  float alpha_dot_rad_s
) {
  uint32_t now = millis();

  if ((now - g_lqr_status_last_ms) < LQR_MICROSTEP_STATUS_INTERVAL_MS) {
    return;
  }

  g_lqr_status_last_ms = now;

  microstep_control_print_status_line(
    "status",
    alpha_rad,
    alpha_dot_rad_s,
    lqr_is_disturbance_profile()
  );
}

static bool microstep_control_apply_profile(
  uint16_t microsteps,
  bool disturbance_profile,
  float alpha_rad,
  float alpha_dot_rad_s,
  const char* reason
) {
  if (!microstep_control_is_valid_microsteps(microsteps)) {
    Serial.print("[lqr-ms] microsteps invalido: ");
    Serial.println(microsteps);
    return false;
  }

  bool same_microsteps = g_lqr_current_microsteps == microsteps;
  bool same_tmc = tmc_get_microsteps() == microsteps;
  bool same_motor = motor_get_microsteps_runtime() == microsteps;

  lqr_set_disturbance_profile(disturbance_profile);

  if (same_microsteps && same_tmc && same_motor) {
    return true;
  }

  if (!tmc_is_ready()) {
    g_lqr_current_microsteps = microsteps;

    Serial.print("[lqr-ms] configuracao salva; TMC ainda nao inicializado | microsteps=");
    Serial.println(microsteps);

    return true;
  }

  bool ok = tmc_set_microsteps(microsteps);

  if (!ok) {
    Serial.println("[lqr-ms] ERRO: falha ao trocar microsteps no TMC");
    return false;
  }

  motor_set_microsteps_runtime(microsteps);

  g_lqr_current_microsteps = microsteps;

  microstep_control_print_status_line(
    reason,
    alpha_rad,
    alpha_dot_rad_s,
    disturbance_profile
  );

  return true;
}

static bool microstep_control_apply_manual(
  float alpha_rad,
  float alpha_dot_rad_s,
  const char* reason
) {
  bool auto_enabled;
  uint16_t manual_microsteps;

  microstep_control_get_settings(&auto_enabled, &manual_microsteps);

  g_microstep_state = MicrostepState::NORMAL;
  g_lqr_disturbance_start_ms = 0;
  g_last_ramp_step_ms = 0;
  g_recovery_block_until_ms = 0;
  g_stable_since_ms = 0;
  g_ramp_index = 0;
  g_disturbance_microsteps = LQR_DISTURBANCE_MICROSTEPS;

  return microstep_control_apply_profile(
    manual_microsteps,
    false,
    alpha_rad,
    alpha_dot_rad_s,
    reason
  );
}

static void microstep_control_enter_disturbance_hold(
  uint16_t microsteps,
  float alpha_rad,
  float alpha_dot_rad_s,
  const char* reason
) {
  g_microstep_state = MicrostepState::HOLD_DISTURBANCE;
  g_lqr_disturbance_start_ms = millis();
  g_last_ramp_step_ms = 0;
  g_recovery_block_until_ms = 0;
  g_stable_since_ms = 0;
  g_ramp_index = 0;
  g_disturbance_microsteps = microsteps;

  microstep_control_apply_profile(
    g_disturbance_microsteps,
    use_disturbance_profile_for(g_disturbance_microsteps),
    alpha_rad,
    alpha_dot_rad_s,
    reason
  );
}

static void microstep_control_enter_ramp_up(
  float alpha_rad,
  float alpha_dot_rad_s
) {
  g_microstep_state = MicrostepState::RAMP_UP;
  g_ramp_index = ramp_index_for_microsteps(g_lqr_current_microsteps);
  g_last_ramp_step_ms = millis();
  g_stable_since_ms = 0;

  microstep_control_apply_profile(
    RAMP_UP_STEPS[g_ramp_index],
    false,
    alpha_rad,
    alpha_dot_rad_s,
    "iniciou rampa subindo"
  );
}

static void microstep_control_enter_recovery_blocked(
  float alpha_rad,
  float alpha_dot_rad_s
) {
  g_microstep_state = MicrostepState::RECOVERY_BLOCKED;
  g_recovery_block_until_ms = millis() + LQR_DISTURBANCE_REARM_BLOCK_MS;
  g_stable_since_ms = 0;
  g_ramp_index = 0;

  microstep_control_apply_profile(
    LQR_FINE_MICROSTEPS,
    false,
    alpha_rad,
    alpha_dot_rad_s,
    "recuperacao: voltou para 256 e bloqueou nova deteccao"
  );
}

void microstep_control_force_fine(float alpha_rad) {
  g_microstep_state = MicrostepState::NORMAL;
  g_lqr_disturbance_start_ms = 0;
  g_last_ramp_step_ms = 0;
  g_recovery_block_until_ms = 0;
  g_stable_since_ms = 0;
  g_ramp_index = 0;
  g_disturbance_microsteps = LQR_DISTURBANCE_MICROSTEPS;

  microstep_control_apply_profile(
    LQR_FINE_MICROSTEPS,
    false,
    alpha_rad,
    0.0f,
    "voltou para fino/padrao"
  );
}

void microstep_control_reset_runtime() {
  g_microstep_state = MicrostepState::NORMAL;
  g_lqr_disturbance_start_ms = 0;
  g_last_ramp_step_ms = 0;
  g_recovery_block_until_ms = 0;
  g_stable_since_ms = 0;
  g_ramp_index = 0;
  g_disturbance_microsteps = LQR_DISTURBANCE_MICROSTEPS;
  g_lqr_current_microsteps = 0;
  g_lqr_status_last_ms = 0;
}

void microstep_control_apply_startup(float alpha_rad) {
  bool auto_enabled;
  uint16_t manual_microsteps;

  microstep_control_get_settings(&auto_enabled, &manual_microsteps);

  microstep_control_reset_runtime();

  if (auto_enabled) {
    microstep_control_force_fine(alpha_rad);
  } else {
    microstep_control_apply_manual(alpha_rad, 0.0f, "manual/startup");
  }
}

void microstep_control_update_lqr(float alpha_rad, float alpha_dot_rad_s) {
  bool auto_enabled;
  uint16_t manual_microsteps;

  microstep_control_get_settings(&auto_enabled, &manual_microsteps);

  if (!auto_enabled) {
    microstep_control_apply_manual(alpha_rad, alpha_dot_rad_s, "manual/lqr");
    microstep_control_print_status_throttled(alpha_rad, alpha_dot_rad_s);
    return;
  }

  const uint32_t now = millis();
  FuzzySupervisorOutput fuzzy = fuzzy_evaluate_microsteps(
    alpha_rad,
    alpha_dot_rad_s
  );

  switch (g_microstep_state) {
    case MicrostepState::NORMAL: {
      microstep_control_apply_profile(
        LQR_FINE_MICROSTEPS,
        false,
        alpha_rad,
        alpha_dot_rad_s,
        "normal"
      );

      if (fuzzy.microsteps < LQR_FINE_MICROSTEPS) {
        microstep_control_enter_disturbance_hold(
          fuzzy.microsteps,
          alpha_rad,
          alpha_dot_rad_s,
          "fuzzy: disturbio detectado"
        );
      }

      break;
    }

    case MicrostepState::HOLD_DISTURBANCE: {
      if (fuzzy.microsteps < g_disturbance_microsteps) {
        g_disturbance_microsteps = fuzzy.microsteps;
        g_lqr_disturbance_start_ms = now;
      }

      microstep_control_apply_profile(
        g_disturbance_microsteps,
        use_disturbance_profile_for(g_disturbance_microsteps),
        alpha_rad,
        alpha_dot_rad_s,
        "segurando disturbio fuzzy"
      );

      const uint32_t elapsed = now - g_lqr_disturbance_start_ms;

      if (elapsed >= LQR_DISTURBANCE_HOLD_MS) {
        if (fuzzy.microsteps < LQR_FINE_MICROSTEPS) {
          g_disturbance_microsteps = fuzzy.microsteps;
          g_lqr_disturbance_start_ms = now;
        } else {
          microstep_control_enter_ramp_up(alpha_rad, alpha_dot_rad_s);
        }
      }

      break;
    }

    case MicrostepState::RAMP_UP: {
      /*
        Durante a rampa, so aborta se o fuzzy indicar uma condicao realmente
        forte. Isso evita novo falso disturbio quando o pendulo passa por 2 ou
        3 graus no caminho de volta para o zero.
      */
      if (fuzzy.microsteps <= 32) {
        microstep_control_enter_disturbance_hold(
          fuzzy.microsteps,
          alpha_rad,
          alpha_dot_rad_s,
          "fuzzy: abortou rampa e voltou para recuperacao"
        );
        break;
      }

      if ((now - g_last_ramp_step_ms) >= LQR_MICROSTEP_RAMP_INTERVAL_MS) {
        g_last_ramp_step_ms = now;

        if (g_ramp_index < (RAMP_UP_STEPS_COUNT - 1)) {
          g_ramp_index++;
        }

        microstep_control_apply_profile(
          RAMP_UP_STEPS[g_ramp_index],
          false,
          alpha_rad,
          alpha_dot_rad_s,
          "rampa subindo"
        );

        if (g_ramp_index >= (RAMP_UP_STEPS_COUNT - 1)) {
          microstep_control_enter_recovery_blocked(alpha_rad, alpha_dot_rad_s);
        }
      }

      break;
    }

    case MicrostepState::RECOVERY_BLOCKED: {
      microstep_control_apply_profile(
        LQR_FINE_MICROSTEPS,
        false,
        alpha_rad,
        alpha_dot_rad_s,
        "recuperacao bloqueada"
      );

      /* Emergencia real: permite voltar para robusto mesmo durante bloqueio. */
      if (fuzzy.microsteps <= LQR_DISTURBANCE_MICROSTEPS) {
        microstep_control_enter_disturbance_hold(
          fuzzy.microsteps,
          alpha_rad,
          alpha_dot_rad_s,
          "fuzzy: emergencia durante bloqueio"
        );
        break;
      }

      if (now < g_recovery_block_until_ms) {
        break;
      }

      if (is_rearm_stable(alpha_rad, alpha_dot_rad_s)) {
        if (g_stable_since_ms == 0) {
          g_stable_since_ms = now;
        }

        if ((now - g_stable_since_ms) >= LQR_DISTURBANCE_REARM_STABLE_MS) {
          g_microstep_state = MicrostepState::NORMAL;
          g_stable_since_ms = 0;

          microstep_control_apply_profile(
            LQR_FINE_MICROSTEPS,
            false,
            alpha_rad,
            alpha_dot_rad_s,
            "fuzzy: rearmado"
          );
        }
      } else {
        g_stable_since_ms = 0;
      }

      break;
    }
  }

  microstep_control_print_status_throttled(alpha_rad, alpha_dot_rad_s);
}

bool microstep_control_set_auto_enabled(bool enabled) {
  portENTER_CRITICAL(&g_microstep_mux);
  g_auto_enabled = enabled;
  portEXIT_CRITICAL(&g_microstep_mux);

  microstep_control_reset_runtime();

  if (enabled) {
    microstep_control_force_fine(0.0f);
  } else {
    microstep_control_apply_manual(0.0f, 0.0f, "manual/config");
  }

  Serial.print("MSAUTO ");
  Serial.print(enabled ? 1 : 0);
  Serial.print(" auto=");
  Serial.print(enabled ? 1 : 0);
  Serial.print(" manual=");
  Serial.print(microstep_control_get_manual_microsteps());
  Serial.print(" current=");
  Serial.println(motor_get_microsteps_runtime());

  return true;
}

bool microstep_control_set_manual_microsteps(uint16_t microsteps) {
  if (!microstep_control_is_valid_microsteps(microsteps)) {
    Serial.print("ERR MSMAN microsteps invalido: ");
    Serial.println(microsteps);
    return false;
  }

  portENTER_CRITICAL(&g_microstep_mux);
  g_manual_microsteps = microsteps;
  portEXIT_CRITICAL(&g_microstep_mux);

  if (!microstep_control_get_auto_enabled()) {
    microstep_control_apply_manual(0.0f, 0.0f, "manual/config");
  }

  Serial.print("MSMAN ");
  Serial.print(microsteps);
  Serial.print(" auto=");
  Serial.print(microstep_control_get_auto_enabled() ? 1 : 0);
  Serial.print(" manual=");
  Serial.print(microsteps);
  Serial.print(" current=");
  Serial.println(motor_get_microsteps_runtime());

  return true;
}
