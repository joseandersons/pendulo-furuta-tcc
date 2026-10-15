#pragma once

#include <Arduino.h>
#include <stdint.h>

void tmc_init();
bool tmc_is_ready();

bool tmc_set_current_ma(uint16_t current_ma);
bool tmc_set_microsteps(uint16_t microsteps);

uint16_t tmc_get_microsteps();

void tmc_set_spreadcycle(bool enabled);