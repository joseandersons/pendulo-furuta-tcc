#include "filas.h"

QueueHandle_t qTel = nullptr;

void filas_init() {
  qTel = xQueueCreate(32, sizeof(TelePkt));
}