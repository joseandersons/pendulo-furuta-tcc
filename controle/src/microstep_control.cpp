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
static uint32_t g_recovery_block_until_ms = 0;
static uint32_t g_stable_since_ms = 0;

/*
  g_ramp_level: nivel aplicado pela maquina de estados (descendo ou subindo).
  g_disturbance_microsteps: alvo da descida (nivel pedido pelo fuzzy, ja
  ajustado aos niveis habilitados na rampa de descida).
*/
static uint16_t g_ramp_level = LQR_FINE_MICROSTEPS;
static uint32_t g_ramp_level_since_ms = 0;
static uint16_t g_disturbance_microsteps = LQR_DISTURBANCE_MICROSTEPS;
static uint16_t g_lqr_current_microsteps = 0;
static uint32_t g_lqr_status_last_ms = 0;

// Indice 0 = 256 (fino). Os demais sao os niveis configuraveis da rampa.
static constexpr uint16_t RAMP_LEVELS[] = {256, 128, 64, 32, 16};
static constexpr uint8_t RAMP_LEVELS_COUNT = 5;

static_assert(
  MICROSTEP_RAMP_LEVELS_COUNT == RAMP_LEVELS_COUNT - 1,
  "niveis configuraveis = RAMP_LEVELS sem o 256"
);

static MicrostepRampConfig g_ramp_cfg = {
  {true, true, true, true},
  {
    LQR_MICROSTEP_RAMP_DOWN_DEFAULT_MS,
    LQR_MICROSTEP_RAMP_DOWN_DEFAULT_MS,
    LQR_MICROSTEP_RAMP_DOWN_DEFAULT_MS,
    LQR_MICROSTEP_RAMP_DOWN_DEFAULT_MS
  },
  {true, true, true, true},
  {
    LQR_MICROSTEP_RAMP_UP_DEFAULT_MS,
    LQR_MICROSTEP_RAMP_UP_DEFAULT_MS,
    LQR_MICROSTEP_RAMP_UP_DEFAULT_MS,
    LQR_MICROSTEP_RAMP_UP_DEFAULT_MS
  }
};

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

MicrostepRampConfig microstep_control_get_ramp() {
  MicrostepRampConfig cfg;

  portENTER_CRITICAL(&g_microstep_mux);
  cfg = g_ramp_cfg;
  portEXIT_CRITICAL(&g_microstep_mux);

  return cfg;
}

bool microstep_control_set_ramp(
  bool down,
  const bool enabled[MICROSTEP_RAMP_LEVELS_COUNT],
  const uint32_t ms[MICROSTEP_RAMP_LEVELS_COUNT]
) {
  if (enabled == nullptr || ms == nullptr) {
    return false;
  }

  for (uint8_t i = 0; i < MICROSTEP_RAMP_LEVELS_COUNT; i++) {
    if (ms[i] > LQR_MICROSTEP_RAMP_MAX_MS) {
      return false;
    }
  }

  portENTER_CRITICAL(&g_microstep_mux);

  for (uint8_t i = 0; i < MICROSTEP_RAMP_LEVELS_COUNT; i++) {
    if (down) {
      g_ramp_cfg.down_enabled[i] = enabled[i];
      g_ramp_cfg.down_ms[i] = ms[i];
    } else {
      g_ramp_cfg.up_enabled[i] = enabled[i];
      g_ramp_cfg.up_ms[i] = ms[i];
    }
  }

  portEXIT_CRITICAL(&g_microstep_mux);

  return true;
}

void microstep_control_print_ramp(bool down) {
  MicrostepRampConfig cfg = microstep_control_get_ramp();

  const bool* en = down ? cfg.down_enabled : cfg.up_enabled;
  const uint32_t* ms = down ? cfg.down_ms : cfg.up_ms;

  // Mesmo formato do comando: en128 en64 en32 en16 t128 t64 t32 t16
  Serial.printf(
    "%s %d %d %d %d %lu %lu %lu %lu\n",
    down ? "MSRD" : "MSRU",
    en[0] ? 1 : 0, en[1] ? 1 : 0, en[2] ? 1 : 0, en[3] ? 1 : 0,
    (unsigned long)ms[0], (unsigned long)ms[1],
    (unsigned long)ms[2], (unsigned long)ms[3]
  );
}

static int8_t ramp_level_index(uint16_t microsteps) {
  for (uint8_t i = 0; i < RAMP_LEVELS_COUNT; i++) {
    if (RAMP_LEVELS[i] == microsteps) {
      return (int8_t)i;
    }
  }

  return -1;
}

/*
  Tempo que a rampa fica no nivel antes de seguir. Nivel desmarcado (ou 256)
  nao participa: tempo zero, a rampa passa direto por ele.
*/
static uint32_t ramp_dwell_ms(
  const bool* enabled,
  const uint32_t* ms,
  uint16_t level
) {
  const int8_t i = ramp_level_index(level);

  if (i <= 0) {
    return 0;
  }

  return enabled[i - 1] ? ms[i - 1] : 0;
}

/*
  Alvo da descida: o maior nivel marcado na descida que nao passe do pedido do
  fuzzy. Se nenhum marcado chega ate o pedido, usa o menor nivel marcado.
*/
static uint16_t ramp_down_target_for(
  const MicrostepRampConfig& cfg,
  uint16_t fuzzy_microsteps
) {
  for (uint8_t i = 1; i < RAMP_LEVELS_COUNT; i++) {
    if (cfg.down_enabled[i - 1] && RAMP_LEVELS[i] <= fuzzy_microsteps) {
      return RAMP_LEVELS[i];
    }
  }

  for (int8_t i = RAMP_LEVELS_COUNT - 1; i >= 1; i--) {
    if (cfg.down_enabled[i - 1]) {
      return RAMP_LEVELS[i];
    }
  }

  return LQR_DISTURBANCE_MICROSTEPS;
}

static uint16_t ramp_next_down_level(
  const MicrostepRampConfig& cfg,
  uint16_t current,
  uint16_t target
) {
  const int8_t i = ramp_level_index(current);

  if (i < 0) {
    return target;
  }

  for (uint8_t j = (uint8_t)i + 1; j < RAMP_LEVELS_COUNT; j++) {
    const uint16_t level = RAMP_LEVELS[j];

    if (level <= target) {
      return target;
    }

    if (cfg.down_enabled[j - 1]) {
      return level;
    }
  }

  return target;
}

static uint16_t ramp_next_up_level(
  const MicrostepRampConfig& cfg,
  uint16_t current
) {
  const int8_t i = ramp_level_index(current);

  for (int8_t j = i - 1; j >= 1; j--) {
    if (cfg.up_enabled[j - 1]) {
      return RAMP_LEVELS[j];
    }
  }

  return LQR_FINE_MICROSTEPS;
}

/*
  Avanca a descida: enquanto o tempo no nivel atual ja venceu, passa para o
  proximo nivel marcado, sem passar do alvo.
*/
static void ramp_advance_down(const MicrostepRampConfig& cfg, uint32_t now) {
  while (g_ramp_level > g_disturbance_microsteps) {
    const uint32_t dwell = ramp_dwell_ms(
      cfg.down_enabled,
      cfg.down_ms,
      g_ramp_level
    );

    if (g_ramp_level != LQR_FINE_MICROSTEPS &&
        (now - g_ramp_level_since_ms) < dwell) {
      break;
    }

    g_ramp_level = ramp_next_down_level(
      cfg,
      g_ramp_level,
      g_disturbance_microsteps
    );
    g_ramp_level_since_ms = now;
  }
}

/* Avanca a subida pelo mesmo criterio; para ao chegar em 256. */
static void ramp_advance_up(const MicrostepRampConfig& cfg, uint32_t now) {
  while (g_ramp_level < LQR_FINE_MICROSTEPS) {
    const uint32_t dwell = ramp_dwell_ms(
      cfg.up_enabled,
      cfg.up_ms,
      g_ramp_level
    );

    if ((now - g_ramp_level_since_ms) < dwell) {
      break;
    }

    g_ramp_level = ramp_next_up_level(cfg, g_ramp_level);
    g_ramp_level_since_ms = now;
  }
}

static bool use_disturbance_profile_for(uint16_t microsteps) {
  /*
    Perfil de ganhos alternativo (LQR_GAINS_DISTURBANCE_16), so usado com
    LQR_USE_DISTURBANCE_GAINS = 1 e nivel <= 64. Desligado por padrao: assim
    o automatico em 16 e o mesmo controlador do manual em 16.
  */
#if LQR_USE_DISTURBANCE_GAINS
  return microsteps <= 64;
#else
  (void)microsteps;
  return false;
#endif
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

static void microstep_control_reset_ramp_state() {
  g_microstep_state = MicrostepState::NORMAL;
  g_lqr_disturbance_start_ms = 0;
  g_recovery_block_until_ms = 0;
  g_stable_since_ms = 0;
  g_ramp_level = LQR_FINE_MICROSTEPS;
  g_ramp_level_since_ms = 0;
  g_disturbance_microsteps = LQR_DISTURBANCE_MICROSTEPS;
}

static bool microstep_control_apply_manual(
  float alpha_rad,
  float alpha_dot_rad_s,
  const char* reason
) {
  bool auto_enabled;
  uint16_t manual_microsteps;

  microstep_control_get_settings(&auto_enabled, &manual_microsteps);

  microstep_control_reset_ramp_state();

  return microstep_control_apply_profile(
    manual_microsteps,
    false,
    alpha_rad,
    alpha_dot_rad_s,
    reason
  );
}

static void microstep_control_apply_ramp_level(
  float alpha_rad,
  float alpha_dot_rad_s,
  const char* reason
) {
  microstep_control_apply_profile(
    g_ramp_level,
    use_disturbance_profile_for(g_ramp_level),
    alpha_rad,
    alpha_dot_rad_s,
    reason
  );
}

/*
  Entra em disturbio a partir do nivel atual (256 no NORMAL, ou o nivel em que
  a subida estava) e desce pela rampa de descida ate o alvo.
*/
static void microstep_control_enter_disturbance_hold(
  const MicrostepRampConfig& cfg,
  uint16_t fuzzy_microsteps,
  float alpha_rad,
  float alpha_dot_rad_s,
  const char* reason
) {
  const uint32_t now = millis();

  uint16_t target = ramp_down_target_for(cfg, fuzzy_microsteps);

  if (target > g_ramp_level) {
    target = g_ramp_level;
  }

  g_microstep_state = MicrostepState::HOLD_DISTURBANCE;
  g_lqr_disturbance_start_ms = now;
  g_recovery_block_until_ms = 0;
  g_stable_since_ms = 0;
  g_disturbance_microsteps = target;
  g_ramp_level_since_ms = now;

  ramp_advance_down(cfg, now);
  microstep_control_apply_ramp_level(alpha_rad, alpha_dot_rad_s, reason);
}

static void microstep_control_enter_ramp_up(
  const MicrostepRampConfig& cfg,
  float alpha_rad,
  float alpha_dot_rad_s
) {
  const uint32_t now = millis();

  g_microstep_state = MicrostepState::RAMP_UP;
  g_ramp_level_since_ms = now;
  g_stable_since_ms = 0;

  // Nivel atual desmarcado na subida tem tempo zero: ja sobe neste ciclo.
  ramp_advance_up(cfg, now);
  microstep_control_apply_ramp_level(
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
  g_ramp_level = LQR_FINE_MICROSTEPS;

  microstep_control_apply_profile(
    LQR_FINE_MICROSTEPS,
    false,
    alpha_rad,
    alpha_dot_rad_s,
    "recuperacao: voltou para 256 e bloqueou nova deteccao"
  );
}

void microstep_control_force_fine(float alpha_rad) {
  microstep_control_reset_ramp_state();

  microstep_control_apply_profile(
    LQR_FINE_MICROSTEPS,
    false,
    alpha_rad,
    0.0f,
    "voltou para fino/padrao"
  );
}

void microstep_control_reset_runtime() {
  microstep_control_reset_ramp_state();
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
  const MicrostepRampConfig cfg = microstep_control_get_ramp();
  FuzzySupervisorOutput fuzzy = fuzzy_evaluate_microsteps(
    alpha_rad,
    alpha_dot_rad_s
  );

  switch (g_microstep_state) {
    case MicrostepState::NORMAL: {
      g_ramp_level = LQR_FINE_MICROSTEPS;

      microstep_control_apply_profile(
        LQR_FINE_MICROSTEPS,
        false,
        alpha_rad,
        alpha_dot_rad_s,
        "normal"
      );

      if (fuzzy.microsteps < LQR_FINE_MICROSTEPS) {
        microstep_control_enter_disturbance_hold(
          cfg,
          fuzzy.microsteps,
          alpha_rad,
          alpha_dot_rad_s,
          "fuzzy: disturbio detectado"
        );
      }

      break;
    }

    case MicrostepState::HOLD_DISTURBANCE: {
      if (fuzzy.microsteps < LQR_FINE_MICROSTEPS) {
        const uint16_t target = ramp_down_target_for(cfg, fuzzy.microsteps);

        if (target < g_disturbance_microsteps) {
          g_disturbance_microsteps = target;
          g_lqr_disturbance_start_ms = now;
        }
      }

      ramp_advance_down(cfg, now);
      microstep_control_apply_ramp_level(
        alpha_rad,
        alpha_dot_rad_s,
        "segurando disturbio fuzzy"
      );

      const uint32_t elapsed = now - g_lqr_disturbance_start_ms;

      if (elapsed >= LQR_DISTURBANCE_HOLD_MS) {
        if (fuzzy.microsteps < LQR_FINE_MICROSTEPS) {
          /*
            Disturbio continua: renova a retencao com o pedido atual do fuzzy,
            mas nunca sobe por aqui (subir so pela rampa de subida).
          */
          uint16_t target = ramp_down_target_for(cfg, fuzzy.microsteps);

          if (target > g_ramp_level) {
            target = g_ramp_level;
          }

          g_disturbance_microsteps = target;
          g_lqr_disturbance_start_ms = now;
        } else {
          microstep_control_enter_ramp_up(cfg, alpha_rad, alpha_dot_rad_s);
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
          cfg,
          fuzzy.microsteps,
          alpha_rad,
          alpha_dot_rad_s,
          "fuzzy: abortou rampa e voltou para recuperacao"
        );
        break;
      }

      ramp_advance_up(cfg, now);

      if (g_ramp_level >= LQR_FINE_MICROSTEPS) {
        microstep_control_enter_recovery_blocked(alpha_rad, alpha_dot_rad_s);
      } else {
        microstep_control_apply_ramp_level(
          alpha_rad,
          alpha_dot_rad_s,
          "rampa subindo"
        );
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
        g_ramp_level = LQR_FINE_MICROSTEPS;

        microstep_control_enter_disturbance_hold(
          cfg,
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
