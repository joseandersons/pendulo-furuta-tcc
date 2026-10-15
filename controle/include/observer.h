#pragma once

#include "lqr.h"

void observer_reset();

void observer_init(
  float theta,
  float alpha,
  float theta_dot,
  float alpha_dot
);

bool observer_ready();

LQRState observer_get_state();

LQRState observer_update(
  float theta_meas,
  float alpha_meas,
  float u_prev_rad_s2
);

bool observer_stage_Lr(
  float m00,
  float m01,
  float m10,
  float m11
);

bool observer_stage_F(
  float m00,
  float m01,
  float m10,
  float m11
);

bool observer_stage_G(
  float m00,
  float m01,
  float m10,
  float m11
);

bool observer_stage_H(
  float h0,
  float h1
);

bool observer_apply_staged();
