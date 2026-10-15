#pragma once

#include <Arduino.h>

void calibration_begin();
bool calibrar_zero_por_fundo();

// Calibracao no topo: soma delta_deg (media de alpha com o pendulo
// equilibrado) ao zero. Aplica gradualmente; salva na flash no STOP.
bool calibrar_zero_ajuste_topo(float delta_deg);
bool calibration_save_if_dirty();
bool calibration_is_dirty();

// Offset usado pelo controle (segue o alvo com taxa limitada). Chamar 1x por ciclo.
float calibration_get_alpha_offset_counts(float dt);
float calibration_get_target_offset_counts();
float calibration_get_current_offset_counts();

// Soma delta ao zero (alvo e atual juntos, sem rampa). Usado pelo autozero,
// que ja aplica a mudanca de forma gradual e so transfere o resultado.
void calibration_shift_offset_immediate(float delta_counts);

int32_t get_zero_offset();
