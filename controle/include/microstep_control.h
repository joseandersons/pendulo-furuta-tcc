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
