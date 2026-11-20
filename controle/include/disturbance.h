#pragma once
#include <stdint.h>
#include <stddef.h>

enum class DistState : uint8_t { IDLE, PENDING, ACTIVE, ENDED, CANCELLED };
struct DistSnapshot {
  DistState state = DistState::IDLE;
  uint32_t id = 0;
  float amplitude = 0;
  uint32_t duration_ms = 0;
  int64_t start_us = 0;
  float value = 0;          // aceleracao extra (PULSE) ou do braco (KICK), rad/s^2
  bool kick = false;        // true: toque em malha aberta, LQR suspenso enquanto ACTIVE
  float kick_accel = 0;     // |a| do tranco, rad/s^2 (amplitude = alpha0 em graus)
  uint8_t kick_phase = 0;   // 1 = acelera, 2 = freia
};

// All entry points are synchronized; no UART, heap or waits in control/motor.
uint32_t disturbance_generation();
DistSnapshot disturbance_update(int64_t now_us, bool lqr_allowed, float alpha_rad, uint32_t generation,
                                float alpha_dot_rad_s = 0.0f);
void disturbance_reset(int64_t now_us, const char* reason);
DistSnapshot disturbance_snapshot(int64_t now_us);
bool disturbance_motor_active(uint32_t id, int64_t now_us);
// Handles the complete DIST namespace, including malformed commands.
bool disturbance_command(const char* line, int64_t now_us);
bool disturbance_pop_message(char* out, size_t size);
