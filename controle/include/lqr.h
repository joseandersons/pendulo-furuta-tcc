#pragma once

#include <Arduino.h>

struct LQRState {
  float theta;
  float alpha;
  float theta_dot;
  float alpha_dot;
};

struct LQROutput {
  float accel_cmd_rad_s2;
  float velocity_cmd_rad_s;
  bool active;
};

void lqr_reset();
float lqr_compute_accel_rad_s2(const LQRState& state);
LQROutput lqr_update(const LQRState& state, float dt);
float lqr_get_velocity_cmd_rad_s();


void lqr_set_disturbance_profile(bool active);
bool lqr_is_disturbance_profile();
