#pragma once

#include <Arduino.h>
#include "tipos.h"

void encoder_pendulo_isr_init();
void encoder_task_start();
void encoder_register_control_task(TaskHandle_t task_handle);

bool encoder_pendulo_get_absolute_count(int32_t* counts);
void encoder_reset_braco_zero();

bool encoder_get_obs(Obs* out);
