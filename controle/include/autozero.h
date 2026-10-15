#pragma once

#include <Arduino.h>

/*
  Autozero do pendulo.

  O zero de alpha usado pelo controle passa a ser:

    zero_total(theta_abs) = c + p*sen(theta_abs) + q*cos(theta_abs) + d

    c    -> offset que ja existia (CALIB por baixo / ZTOP), em calibration.cpp
    p, q -> curva da inclinacao da base, aprendida e salva na flash
    d    -> ajuste local que o autozero integra enquanto o sistema esta assentado

  Ao concluir um ajuste, o zero correto naquela posicao vira um ponto. Com
  pontos suficientes, c/p/q sao reajustados; caso contrario d e transferido
  para c (o mesmo que o ZTOP faz).
*/

struct AutozeroInput {
  float dt;
  float alpha_raw_counts;   // alpha bruto (sem histerese), ja descontado o zero total
  float alpha_dot_rad_s;    // derivada medida de alpha
  float theta_dot_rad_s;    // velocidade medida do braco
  float theta_err_rad;      // theta - theta_ref (so para informacao)
  float target_rad;         // alvo atual do braco
  bool ref_settled;         // rampa da referencia terminou
  float arm_abs_rad;        // angulo absoluto do braco (encoder)
  float base_offset_counts; // c atual (calibration_get_alpha_offset_counts)
};

void autozero_begin();

// Parte do zero alem de c: curva + ajuste local d (contagens).
float autozero_extra_counts(float arm_abs_rad);

// So a curva p*sen + q*cos (contagens).
float autozero_tilt_counts(float arm_abs_rad);

void autozero_on_lqr_enter();
void autozero_on_lqr_exit();
void autozero_on_stop();
void autozero_update(const AutozeroInput& in);

bool autozero_save_if_dirty();

void autozero_set_enabled(bool enabled);
bool autozero_get_enabled();
void autozero_reset_curve();

// Texto de estado para o comando AZ?
void autozero_format_status(char* buf, size_t len);

// Mensagens geradas no laco de controle, impressas pela tarefa serial.
bool autozero_pop_message(char* buf, size_t len);
