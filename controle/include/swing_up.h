#pragma once

#include <Arduino.h>
#include "lqr.h"

struct SwingUpOutput {
  float velocity_cmd_rad_s;
  bool active;
};

void swing_up_reset();
SwingUpOutput swing_up_update(const LQRState& state, float dt);