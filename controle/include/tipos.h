#pragma once

#include <Arduino.h>

struct Obs {
  uint32_t sample_id;
  int32_t pendulo_counts;      // com histerese (usado pelo controle)
  int32_t pendulo_raw_counts;  // absoluto bruto, sem histerese (autozero)
  int32_t braco_abs_counts;    // absoluto do braco 0..4095 (curva de inclinacao)
  int32_t braco_counts;
  uint16_t braco_raw;
  float braco_rad;
  float braco_dot_rad_s;
  uint32_t stamp_us;
};

struct TelePkt {
  uint32_t t_ms;
  int16_t alpha_c2;
  int16_t theta_c2;
  int16_t alpha_dot_c2;
  int16_t theta_dot_c2;
  int16_t u_c2;
  int16_t vel_c2;
  int8_t enabled;
  uint64_t control_us;
  int32_t d_c2;        // ate 1000 rad/s^2 * 100 nao cabe em int16
  int32_t total_c2;
  uint16_t microsteps;
  uint8_t mode;
  uint8_t ms_auto;
  uint8_t gain_profile;
  uint32_t pulse_id;
  int32_t pulse_a_c2;
  uint16_t pulse_ms;
  int16_t theta_ref_c2;   // theta_ref em graus * 100
  uint32_t ref_rate_c2;   // velocidade da rampa em rad/s * 100
};
