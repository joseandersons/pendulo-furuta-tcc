#pragma once

#include <Arduino.h>
#include <stdint.h>

void microstep_control_reset_runtime();
void microstep_control_apply_startup(float alpha_rad);
void microstep_control_force_fine(float alpha_rad);
void microstep_control_update_lqr(float alpha_rad, float alpha_dot_rad_s = 0.0f);

bool microstep_control_set_auto_enabled(bool enabled);
bool microstep_control_get_auto_enabled();

bool microstep_control_set_manual_microsteps(uint16_t microsteps);
uint16_t microstep_control_get_manual_microsteps();
uint16_t microstep_control_get_current_microsteps();

bool microstep_control_is_valid_microsteps(uint16_t microsteps);
void microstep_control_print_status(float alpha_rad);

// Niveis configuraveis da rampa, nesta ordem: 128, 64, 32, 16.
// O 256 e sempre o inicio da descida e o fim da subida.
static constexpr uint8_t MICROSTEP_RAMP_LEVELS_COUNT = 4;

struct MicrostepRampConfig {
  bool down_enabled[MICROSTEP_RAMP_LEVELS_COUNT];
  uint32_t down_ms[MICROSTEP_RAMP_LEVELS_COUNT];
  bool up_enabled[MICROSTEP_RAMP_LEVELS_COUNT];
  uint32_t up_ms[MICROSTEP_RAMP_LEVELS_COUNT];
};

// down = true: rampa de descida (MSRD); false: subida (MSRU).
bool microstep_control_set_ramp(
  bool down,
  const bool enabled[MICROSTEP_RAMP_LEVELS_COUNT],
  const uint32_t ms[MICROSTEP_RAMP_LEVELS_COUNT]
);
MicrostepRampConfig microstep_control_get_ramp();
void microstep_control_print_ramp(bool down);
