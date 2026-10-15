#include <Arduino.h>
#include "filas.h"
#include "tipos.h"
#include "config.h"

#define TEL Serial
#define TEL_BEGIN() do { Serial.begin(115200); Serial.setTxBufferSize(2048); } while(0)

static inline void tx_line(const TelePkt& p) {
  if (TEL.availableForWrite() < 120) return;

  char buf[220];

  int n = snprintf(
    buf,
    sizeof(buf),
    "T,%u,A,%d,TH,%d,AD,%d,TD,%d,U,%d,VEL,%d,EN,%d\n",
    p.t_ms,
    p.alpha_c2,
    p.theta_c2,
    p.alpha_dot_c2,
    p.theta_dot_c2,
    p.u_c2,
    p.vel_c2,
    p.enabled
  );

  if (n > 0) {
    TEL.write((const uint8_t*)buf, (size_t)n);
  }
}

static void serial_tx_task(void*) {
  TEL_BEGIN();

  TelePkt pkt{};

  for (;;) {
    if (xQueueReceive(qTel, &pkt, pdMS_TO_TICKS(10)) == pdTRUE) {
      tx_line(pkt);
    }
  }
}

void serial_tx_task_start() {
  xTaskCreatePinnedToCore(
    serial_tx_task,
    "thTEL",
    3072,
    nullptr,
    tskIDLE_PRIORITY + 1,
    nullptr,
    CORE_IO
  );
}