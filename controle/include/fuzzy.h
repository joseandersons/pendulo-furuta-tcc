#pragma once

#include <Arduino.h>
#include <stdint.h>

struct FuzzySupervisorOutput {
  uint16_t microsteps;
  float crisp_microsteps;

  float alpha_small;
  float alpha_medium;
  float alpha_large;

  float alpha_dot_low;
  float alpha_dot_medium;
  float alpha_dot_high;
};

FuzzySupervisorOutput fuzzy_evaluate_microsteps(
  float alpha_rad,
  float alpha_dot_rad_s
);

uint16_t fuzzy_select_microsteps(
  float alpha_rad,
  float alpha_dot_rad_s
);
