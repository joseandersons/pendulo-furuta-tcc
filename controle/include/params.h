#pragma once
#include <Arduino.h>

struct LQRGains {
  float k_theta;
  float k_alpha;
  float k_theta_dot;
  float k_alpha_dot;
};

void params_begin();

void params_lqr_set(float k_theta,
                    float k_alpha,
                    float k_theta_dot,
                    float k_alpha_dot);

LQRGains params_lqr_get();