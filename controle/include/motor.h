#pragma once

#include <Arduino.h>
#include <stdint.h>

void motor_init();
void motor_task_start();

void motor_set_velocity_rad_s(float omega);
void motor_set_velocity_adaptive_rad_s(
  float omega,
  float alpha_rad,
  float alpha_dot_rad_s
);

void motor_set_acceleration_rad_s2(float accel_rad_s2);
void motor_set_acceleration_adaptive_rad_s2(
  float accel_rad_s2,
  float alpha_rad,
  float alpha_dot_rad_s
);

void motor_set_balance_state(float alpha_rad, float alpha_dot_rad_s);

void motor_set_accel_max_rad_s2(float max_accel_rad_s2);
float motor_get_accel_max_rad_s2();

float motor_get_effective_velocity_max_rad_s();
float motor_get_velocity_cmd_rad_s();
float motor_get_theta_est_rad();

void motor_reset_theta_est();
void motor_stop_immediate();

uint16_t motor_get_current_microsteps();

void motor_set_jerk_max_rad_s3(float max_jerk_rad_s3);
float motor_get_jerk_max_rad_s3();

void motor_set_microsteps_runtime(uint16_t microsteps);
uint16_t motor_get_microsteps_runtime();