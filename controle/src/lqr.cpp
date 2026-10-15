#include "lqr.h"
#include "config.h"
#include "params.h"
#include "motor.h"

#include <Arduino.h>
#include <math.h>

static float g_velocity_cmd_rad_s = 0.0f;
static volatile bool g_lqr_disturbance_profile = false;

static const LQRGains LQR_GAINS_DISTURBANCE_16 = {
  -13.454442f,   // k_theta
   535.967467f, // k_alpha
  -15.876477f,  // k_theta_dot
   86.634851f   // k_alpha_dot
};

void lqr_set_disturbance_profile(bool active) {
  static bool first = true;
  static bool last_active = false;

  g_lqr_disturbance_profile = active;

  if (first || last_active != active) {
    Serial.print("[lqr] perfil = ");
    Serial.println(active ? "DISTURBIO_16" : "PADRAO_INTERFACE");

    last_active = active;
    first = false;
  }
}

bool lqr_is_disturbance_profile() {
  return g_lqr_disturbance_profile;
}

static LQRGains lqr_get_active_gains() {
  if (g_lqr_disturbance_profile) {
    return LQR_GAINS_DISTURBANCE_16;
  }

  return params_lqr_get();
}

void lqr_reset() {
  g_velocity_cmd_rad_s = 0.0f;
}

float lqr_compute_accel_rad_s2(const LQRState& state) {
  LQRGains k = lqr_get_active_gains();

  float u = -(k.k_theta * state.theta
            + k.k_alpha * state.alpha
            + k.k_theta_dot * state.theta_dot
            + k.k_alpha_dot * state.alpha_dot);

  float accel_limit = motor_get_accel_max_rad_s2();

  u = constrain(u, -accel_limit, accel_limit);

  return u;
}

LQROutput lqr_update(const LQRState& state, float dt) {
  LQROutput out{};

  if (dt <= 0.0f) {
    dt = 1e-3f;
  }

  if (fabsf(state.alpha) > ALPHA_ENABLE_RAD) {
    g_velocity_cmd_rad_s = 0.0f;
    out.accel_cmd_rad_s2 = 0.0f;
    out.velocity_cmd_rad_s = 0.0f;
    out.active = false;
    return out;
  }

  /*
    theta e um erro continuo do braco. Nao existe corte artificial em
    +/-180 graus; a atuacao continua sendo limitada por aceleracao e
    velocidade do motor.
  */

  float u = lqr_compute_accel_rad_s2(state);

  g_velocity_cmd_rad_s += u * dt;

  float vel_limit = motor_get_effective_velocity_max_rad_s();

  g_velocity_cmd_rad_s = constrain(
    g_velocity_cmd_rad_s,
    -vel_limit,
    vel_limit
  );

  out.accel_cmd_rad_s2 = u;
  out.velocity_cmd_rad_s = g_velocity_cmd_rad_s;
  out.active = false;

  return out;
}

float lqr_get_velocity_cmd_rad_s() {
  return g_velocity_cmd_rad_s;
}
