#include <Arduino.h>
#include <math.h>

#include "motor.h"
#include "config.h"

static volatile uint16_t g_motor_microsteps_runtime = MOTOR_MICROSTEPS;
static portMUX_TYPE g_motor_mux = portMUX_INITIALIZER_UNLOCKED;

enum MotionMode {
  MOTION_MODE_VELOCITY = 0,
  MOTION_MODE_ACCELERATION = 1
};

static volatile MotionMode g_motion_mode = MOTION_MODE_VELOCITY;

static volatile float g_omega_alvo_rad_s = 0.0f;
static volatile float g_accel_alvo_rad_s2 = 0.0f;

static float g_omega_aplicada_rad_s = 0.0f;
static float g_accel_aplicada_rad_s2 = 0.0f;

/*
  Velocidade comandada interna (antes de virar frequencia de passos).
  No modo aceleracao ela integra o comando do LQR mesmo quando ainda e
  pequena demais para gerar passos (< ~0,001 rad/s). Antes, a velocidade
  realizada (zero) era usada como ponto de partida a cada ciclo e as
  aceleracoes pequenas eram descartadas para sempre.
*/
static float g_omega_interna_rad_s = 0.0f;

static float g_theta_est_rad = 0.0f;
static float g_step_count_est = 0.0f;

static volatile float g_arm_accel_max_rad_s2 = ARM_ACCEL_MAX_RAD_S2;
static volatile float g_arm_jerk_max_rad_s3 = ARM_JERK_MAX_RAD_S3;

static float g_alpha_rad = 0.0f;
static float g_alpha_dot_rad_s = 0.0f;

static uint32_t g_last_step_freq_hz = 0;
static int g_last_dir = 0;

static bool motor_is_valid_microsteps(uint16_t microsteps) {
  return microsteps == 1   ||
         microsteps == 2   ||
         microsteps == 4   ||
         microsteps == 8   ||
         microsteps == 16  ||
         microsteps == 32  ||
         microsteps == 64  ||
         microsteps == 128 ||
         microsteps == 256;
}

static float rad_per_step_for_microsteps(uint16_t microsteps) {
  return (2.0f * PI) /
         ((float)MOTOR_STEPS_PER_REV *
          (float)microsteps *
          (float)MOTOR_GEAR_RATIO);
}

uint16_t motor_get_microsteps_runtime() {
  uint16_t microsteps;

  portENTER_CRITICAL(&g_motor_mux);
  microsteps = g_motor_microsteps_runtime;
  portEXIT_CRITICAL(&g_motor_mux);

  return microsteps;
}

uint16_t motor_get_current_microsteps() {
  return motor_get_microsteps_runtime();
}

void motor_set_microsteps_runtime(uint16_t microsteps) {
  if (!motor_is_valid_microsteps(microsteps)) {
    return;
  }

  portENTER_CRITICAL(&g_motor_mux);

  if (g_motor_microsteps_runtime != microsteps) {
    g_motor_microsteps_runtime = microsteps;

    float new_rad_per_step = rad_per_step_for_microsteps(microsteps);

    if (new_rad_per_step > 0.0f) {
      g_step_count_est = g_theta_est_rad / new_rad_per_step;
    }
  }

  portEXIT_CRITICAL(&g_motor_mux);
}

static float rad_per_step() {
  return rad_per_step_for_microsteps(motor_get_microsteps_runtime());
}

static float omega_to_step_frequency(float omega) {
  return fabsf(omega) / rad_per_step();
}

static float step_frequency_to_omega(float freq) {
  return freq * rad_per_step();
}

static float max_omega_by_frequency() {
  return step_frequency_to_omega((float)STEP_FREQ_MAX);
}

float motor_get_effective_velocity_max_rad_s() {
  return max_omega_by_frequency();
}

void motor_set_accel_max_rad_s2(float max_accel_rad_s2) {
  if (!isfinite(max_accel_rad_s2)) {
    max_accel_rad_s2 = 0.0f;
  }

  if (max_accel_rad_s2 < 0.0f) {
    max_accel_rad_s2 = 0.0f;
  }

  portENTER_CRITICAL(&g_motor_mux);
  g_arm_accel_max_rad_s2 = max_accel_rad_s2;
  portEXIT_CRITICAL(&g_motor_mux);
}

float motor_get_accel_max_rad_s2() {
  float accel;

  portENTER_CRITICAL(&g_motor_mux);
  accel = g_arm_accel_max_rad_s2;
  portEXIT_CRITICAL(&g_motor_mux);

  return accel;
}

void motor_set_jerk_max_rad_s3(float max_jerk_rad_s3) {
  if (!isfinite(max_jerk_rad_s3)) {
    max_jerk_rad_s3 = 0.0f;
  }

  if (max_jerk_rad_s3 < 0.0f) {
    max_jerk_rad_s3 = 0.0f;
  }

  portENTER_CRITICAL(&g_motor_mux);
  g_arm_jerk_max_rad_s3 = max_jerk_rad_s3;
  portEXIT_CRITICAL(&g_motor_mux);
}

float motor_get_jerk_max_rad_s3() {
  float jerk;

  portENTER_CRITICAL(&g_motor_mux);
  jerk = g_arm_jerk_max_rad_s3;
  portEXIT_CRITICAL(&g_motor_mux);

  return jerk;
}

static void motor_stop() {
  ledcWrite(STEP_CHANNEL, 0);
  g_last_step_freq_hz = 0;
}

static float motor_write_velocity_rad_s_internal(float omega) {
  float vel_limit = motor_get_effective_velocity_max_rad_s();

  if (vel_limit <= 0.0f) {
    motor_stop();
    return 0.0f;
  }

  if (!isfinite(omega)) {
    omega = 0.0f;
  }

  omega = constrain(omega, -vel_limit, vel_limit);

  if (fabsf(omega) < 0.001f) {
    motor_stop();
    return 0.0f;
  }

  float freq = omega_to_step_frequency(omega);

  if (freq < 1.0f) {
    motor_stop();
    return 0.0f;
  }

  if (STEP_FREQ_MIN > 0 && freq < STEP_FREQ_MIN) {
    freq = STEP_FREQ_MIN;
  }

  if (freq > STEP_FREQ_MAX) {
    freq = STEP_FREQ_MAX;
  }

  uint32_t freq_hz = (uint32_t)(freq + 0.5f);

  if (freq_hz < 1) {
    motor_stop();
    return 0.0f;
  }

  int dir = omega > 0.0f ? 1 : -1;

  if (dir != g_last_dir) {
    digitalWrite(DIR_PIN, dir > 0 ? HIGH : LOW);
    g_last_dir = dir;
  }

  uint32_t duty = (1UL << STEP_PWM_RESOLUTION) / 2;

  if (freq_hz != g_last_step_freq_hz) {
    ledcChangeFrequency(STEP_CHANNEL, freq_hz, STEP_PWM_RESOLUTION);
    g_last_step_freq_hz = freq_hz;
  }

  ledcWrite(STEP_CHANNEL, duty);

  float omega_real = step_frequency_to_omega((float)freq_hz);

  return dir > 0 ? omega_real : -omega_real;
}

void motor_set_balance_state(float alpha_rad, float alpha_dot_rad_s) {
  if (!isfinite(alpha_rad)) {
    alpha_rad = 0.0f;
  }

  if (!isfinite(alpha_dot_rad_s)) {
    alpha_dot_rad_s = 0.0f;
  }

  portENTER_CRITICAL(&g_motor_mux);
  g_alpha_rad = alpha_rad;
  g_alpha_dot_rad_s = alpha_dot_rad_s;
  portEXIT_CRITICAL(&g_motor_mux);
}

void motor_set_velocity_rad_s(float omega) {
  if (!isfinite(omega)) {
    omega = 0.0f;
  }

  float vel_limit = motor_get_effective_velocity_max_rad_s();
  omega = constrain(omega, -vel_limit, vel_limit);

  portENTER_CRITICAL(&g_motor_mux);

  if (g_motion_mode != MOTION_MODE_VELOCITY) {
    g_accel_aplicada_rad_s2 = 0.0f;
  }

  g_motion_mode = MOTION_MODE_VELOCITY;
  g_omega_alvo_rad_s = omega;
  g_accel_alvo_rad_s2 = 0.0f;

  portEXIT_CRITICAL(&g_motor_mux);
}

void motor_set_velocity_adaptive_rad_s(
  float omega,
  float alpha_rad,
  float alpha_dot_rad_s
) {
  motor_set_balance_state(alpha_rad, alpha_dot_rad_s);
  motor_set_velocity_rad_s(omega);
}

void motor_set_acceleration_rad_s2(float accel_rad_s2) {
  if (!isfinite(accel_rad_s2)) {
    accel_rad_s2 = 0.0f;
  }

  float accel_limit;

  portENTER_CRITICAL(&g_motor_mux);
  accel_limit = g_arm_accel_max_rad_s2;
  portEXIT_CRITICAL(&g_motor_mux);

  accel_rad_s2 = constrain(accel_rad_s2, -accel_limit, accel_limit);

  portENTER_CRITICAL(&g_motor_mux);

  if (g_motion_mode != MOTION_MODE_ACCELERATION) {
    g_accel_aplicada_rad_s2 = 0.0f;
    g_omega_interna_rad_s = g_omega_aplicada_rad_s;
  }

  g_motion_mode = MOTION_MODE_ACCELERATION;
  g_accel_alvo_rad_s2 = accel_rad_s2;

  portEXIT_CRITICAL(&g_motor_mux);
}

void motor_set_acceleration_adaptive_rad_s2(
  float accel_rad_s2,
  float alpha_rad,
  float alpha_dot_rad_s
) {
  motor_set_balance_state(alpha_rad, alpha_dot_rad_s);
  motor_set_acceleration_rad_s2(accel_rad_s2);
}

float motor_get_theta_est_rad() {
  float theta;

  portENTER_CRITICAL(&g_motor_mux);
  theta = g_theta_est_rad;
  portEXIT_CRITICAL(&g_motor_mux);

  return theta;
}

float motor_get_velocity_cmd_rad_s() {
  float omega;

  portENTER_CRITICAL(&g_motor_mux);
  omega = g_omega_aplicada_rad_s;
  portEXIT_CRITICAL(&g_motor_mux);

  return omega;
}

void motor_reset_theta_est() {
  portENTER_CRITICAL(&g_motor_mux);
  g_theta_est_rad = 0.0f;
  g_step_count_est = 0.0f;
  portEXIT_CRITICAL(&g_motor_mux);
}

static float aplicar_limite_jerk(
  float accel_atual,
  float accel_alvo,
  float dt
) {
  if (dt <= 0.0f) {
    return accel_atual;
  }

  float jerk_limit;

  portENTER_CRITICAL(&g_motor_mux);
  jerk_limit = g_arm_jerk_max_rad_s3;
  portEXIT_CRITICAL(&g_motor_mux);

  if (jerk_limit <= 0.0f) {
    return accel_alvo;
  }

  float max_delta_accel = jerk_limit * dt;
  float erro = accel_alvo - accel_atual;

  if (erro > max_delta_accel) {
    return accel_atual + max_delta_accel;
  }

  if (erro < -max_delta_accel) {
    return accel_atual - max_delta_accel;
  }

  return accel_alvo;
}

static float aplicar_curva_s_velocidade(
  float atual,
  float alvo,
  float dt,
  float accel_atual,
  float* accel_saida
) {
  if (dt <= 0.0f) {
    *accel_saida = accel_atual;
    return atual;
  }

  float accel_limit;

  portENTER_CRITICAL(&g_motor_mux);
  accel_limit = g_arm_accel_max_rad_s2;
  portEXIT_CRITICAL(&g_motor_mux);

  float vel_limit = motor_get_effective_velocity_max_rad_s();

  if (vel_limit <= 0.0f) {
    *accel_saida = 0.0f;
    return 0.0f;
  }

  if (!isfinite(alvo)) {
    alvo = 0.0f;
  }

  if (!isfinite(atual)) {
    atual = 0.0f;
  }

  alvo = constrain(alvo, -vel_limit, vel_limit);

  if ((atual > 0.0f && alvo < 0.0f) || (atual < 0.0f && alvo > 0.0f)) {
    alvo = 0.0f;
  }

  if (accel_limit <= 0.0f) {
    *accel_saida = 0.0f;
    return alvo;
  }

  float erro_vel = alvo - atual;

  if (fabsf(erro_vel) < 0.0005f && fabsf(accel_atual) < 0.5f) {
    *accel_saida = 0.0f;
    return alvo;
  }

  float accel_alvo = erro_vel / dt;
  accel_alvo = constrain(accel_alvo, -accel_limit, accel_limit);

  float accel_suave = aplicar_limite_jerk(
    accel_atual,
    accel_alvo,
    dt
  );

  float omega = atual + accel_suave * dt;

  if ((alvo - atual) * (alvo - omega) <= 0.0f) {
    omega = alvo;
    accel_suave = 0.0f;
  }

  omega = constrain(omega, -vel_limit, vel_limit);

  *accel_saida = accel_suave;

  return omega;
}

static float aplicar_curva_s_aceleracao(
  float atual,
  float accel_cmd,
  float dt,
  float accel_atual,
  float* accel_saida
) {
  if (dt <= 0.0f) {
    *accel_saida = accel_atual;
    return atual;
  }

  float accel_limit;

  portENTER_CRITICAL(&g_motor_mux);
  accel_limit = g_arm_accel_max_rad_s2;
  portEXIT_CRITICAL(&g_motor_mux);

  float vel_limit = motor_get_effective_velocity_max_rad_s();

  if (vel_limit <= 0.0f) {
    *accel_saida = 0.0f;
    return 0.0f;
  }

  if (!isfinite(accel_cmd)) {
    accel_cmd = 0.0f;
  }

  if (accel_limit <= 0.0f) {
    accel_cmd = 0.0f;
  } else {
    accel_cmd = constrain(accel_cmd, -accel_limit, accel_limit);
  }

  if (atual >= vel_limit && accel_cmd > 0.0f) {
    accel_cmd = 0.0f;
  }

  if (atual <= -vel_limit && accel_cmd < 0.0f) {
    accel_cmd = 0.0f;
  }

  float accel_suave = aplicar_limite_jerk(
    accel_atual,
    accel_cmd,
    dt
  );

  float omega = atual + accel_suave * dt;
  omega = constrain(omega, -vel_limit, vel_limit);

  // So zera quando o comando tambem e praticamente nulo. Antes bastava
  // |accel| < 0,5 rad/s^2: partindo do repouso, qualquer |u| < 0,25 rad/s^2
  // era descartado a cada ciclo de 2 ms e o braco ficava parado.
  if (fabsf(omega) < 0.0005f && fabsf(accel_cmd) < 1e-3f) {
    omega = 0.0f;
    accel_suave = 0.0f;
  }

  *accel_saida = accel_suave;

  return omega;
}

void motor_stop_immediate() {
  portENTER_CRITICAL(&g_motor_mux);

  g_motion_mode = MOTION_MODE_VELOCITY;
  g_omega_alvo_rad_s = 0.0f;
  g_accel_alvo_rad_s2 = 0.0f;
  g_omega_aplicada_rad_s = 0.0f;
  g_accel_aplicada_rad_s2 = 0.0f;
  g_omega_interna_rad_s = 0.0f;

  portEXIT_CRITICAL(&g_motor_mux);

  ledcWrite(STEP_CHANNEL, 0);
  g_last_step_freq_hz = 0;
}

static void motor_task(void*) {
  TickType_t last = xTaskGetTickCount();
  TickType_t last_motor_update = xTaskGetTickCount();

  for (;;) {
    TickType_t now = xTaskGetTickCount();

    if ((now - last_motor_update) >= pdMS_TO_TICKS(MOTOR_UPDATE_MS)) {
      TickType_t elapsed_ticks = now - last_motor_update;
      last_motor_update = now;

      float dt = ((float)elapsed_ticks * (float)portTICK_PERIOD_MS) / 1000.0f;

      float alvo;
      float atual;
      float accel_cmd;
      float accel_atual;
      MotionMode motion_mode;

      portENTER_CRITICAL(&g_motor_mux);

      alvo = g_omega_alvo_rad_s;
      motion_mode = g_motion_mode;
      // No modo aceleracao parte da velocidade interna (nao quantizada).
      atual = (motion_mode == MOTION_MODE_ACCELERATION)
                ? g_omega_interna_rad_s
                : g_omega_aplicada_rad_s;
      accel_cmd = g_accel_alvo_rad_s2;
      accel_atual = g_accel_aplicada_rad_s2;
      motion_mode = g_motion_mode;

      portEXIT_CRITICAL(&g_motor_mux);

      float omega_cmd = 0.0f;
      float accel_nova = 0.0f;

      if (motion_mode == MOTION_MODE_ACCELERATION) {
        omega_cmd = aplicar_curva_s_aceleracao(
          atual,
          accel_cmd,
          dt,
          accel_atual,
          &accel_nova
        );
      } else {
        omega_cmd = aplicar_curva_s_velocidade(
          atual,
          alvo,
          dt,
          accel_atual,
          &accel_nova
        );
      }

      float omega_real = motor_write_velocity_rad_s_internal(omega_cmd);

      float steps_delta = 0.0f;

      if (fabsf(omega_real) > 0.0f) {
        float freq_real = omega_to_step_frequency(omega_real);
        steps_delta = (omega_real > 0.0f ? 1.0f : -1.0f) * freq_real * dt;
      }

      portENTER_CRITICAL(&g_motor_mux);

      g_omega_aplicada_rad_s = omega_real;
      g_omega_interna_rad_s =
        (motion_mode == MOTION_MODE_ACCELERATION) ? omega_cmd : omega_real;
      g_accel_aplicada_rad_s2 = accel_nova;

      g_step_count_est += steps_delta;
      g_theta_est_rad = g_step_count_est * rad_per_step_for_microsteps(
        g_motor_microsteps_runtime
      );

      portEXIT_CRITICAL(&g_motor_mux);
    }

    vTaskDelayUntil(&last, pdMS_TO_TICKS(2));
  }
}

void motor_init() {
  pinMode(DIR_PIN, OUTPUT);

  ledcSetup(STEP_CHANNEL, 1000, STEP_PWM_RESOLUTION);
  ledcAttachPin(STEP_PIN, STEP_CHANNEL);

  portENTER_CRITICAL(&g_motor_mux);

  g_motor_microsteps_runtime = MOTOR_MICROSTEPS;

  g_motion_mode = MOTION_MODE_VELOCITY;
  g_omega_alvo_rad_s = 0.0f;
  g_accel_alvo_rad_s2 = 0.0f;
  g_omega_aplicada_rad_s = 0.0f;
  g_accel_aplicada_rad_s2 = 0.0f;
  g_omega_interna_rad_s = 0.0f;

  g_theta_est_rad = 0.0f;
  g_step_count_est = 0.0f;

  g_arm_accel_max_rad_s2 = ARM_ACCEL_MAX_RAD_S2;
  g_arm_jerk_max_rad_s3 = ARM_JERK_MAX_RAD_S3;

  g_alpha_rad = 0.0f;
  g_alpha_dot_rad_s = 0.0f;

  portEXIT_CRITICAL(&g_motor_mux);

  g_last_step_freq_hz = 0;
  g_last_dir = 0;

  motor_stop();
}

void motor_task_start() {
  xTaskCreatePinnedToCore(
    motor_task,
    "thMOTOR",
    4096,
    nullptr,
    PRIO_MOTOR,
    nullptr,
    CORE_IO
  );
}