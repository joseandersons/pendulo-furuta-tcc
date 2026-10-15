#include "params.h"
#include <Arduino.h>

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static volatile float g_lqr_k_theta;
static volatile float g_lqr_k_alpha;
static volatile float g_lqr_k_theta_dot;
static volatile float g_lqr_k_alpha_dot;

void params_begin() {
  portENTER_CRITICAL(&mux);

  g_lqr_k_theta = -2.2038f;
  g_lqr_k_alpha = 228.1777f;
  g_lqr_k_theta_dot = -3.0616f;
  g_lqr_k_alpha_dot = 38.1739f;

  portEXIT_CRITICAL(&mux);
}

void params_lqr_set(float k_theta,
                    float k_alpha,
                    float k_theta_dot,
                    float k_alpha_dot) {
  portENTER_CRITICAL(&mux);

  g_lqr_k_theta = k_theta;
  g_lqr_k_alpha = k_alpha;
  g_lqr_k_theta_dot = k_theta_dot;
  g_lqr_k_alpha_dot = k_alpha_dot;

  portEXIT_CRITICAL(&mux);
}

LQRGains params_lqr_get() {
  LQRGains r;

  portENTER_CRITICAL(&mux);

  r.k_theta = g_lqr_k_theta;
  r.k_alpha = g_lqr_k_alpha;
  r.k_theta_dot = g_lqr_k_theta_dot;
  r.k_alpha_dot = g_lqr_k_alpha_dot;

  portEXIT_CRITICAL(&mux);

  return r;
}