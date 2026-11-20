#include <Arduino.h>
#include "filas.h"
#include "tipos.h"
#include "config.h"
#include "disturbance.h"
#include <esp_timer.h>

#define TEL Serial
// Serial ja foi iniciado em setup() com buffer de TX; nao reiniciar aqui
// (begin() de novo descartaria o buffer configurado).
#define TEL_BEGIN() do { } while(0)

static inline void tx_line(const TelePkt& p) {
  char buf[300];

  int n = snprintf(
    buf,
    sizeof(buf),
    "\nT,%u,A,%d,TH,%d,AD,%d,TD,%d,U,%d,VEL,%d,EN,%d,TS,%llu,D,%ld,UT,%ld,MS,%u,CM,%u,MA,%u,GP,%u,DI,%lu,DA,%ld,DP,%u,TR,%d,RR,%lu\n",
    p.t_ms,
    p.alpha_c2,
    p.theta_c2,
    p.alpha_dot_c2,
    p.theta_dot_c2,
    p.u_c2,
    p.vel_c2,
    p.enabled,
    (unsigned long long)p.control_us, (long)p.d_c2, (long)p.total_c2,
    p.microsteps, p.mode, p.ms_auto, p.gain_profile,
    (unsigned long)p.pulse_id, (long)p.pulse_a_c2, p.pulse_ms,
    (int)p.theta_ref_c2, (unsigned long)p.ref_rate_c2
  );

  // Nao usar TEL.availableForWrite() como filtro: neste core ele retorna so o
  // espaco livre do FIFO de hardware (max. 128 bytes), nao o buffer de 2048.
  // Linhas > 128 bytes (ex.: durante um pulso, D/UT com 5-6 digitos) eram
  // descartadas sempre. Esta task tem prioridade baixa; bloquear aqui so
  // atrasa a telemetria (qTel descarta pacotes se encher), nao o controle.
  if (n > 0 && n < (int)sizeof(buf)) {
    TEL.write((const uint8_t*)buf, (size_t)n);
  }
}

static void serial_tx_task(void*) {
  TEL_BEGIN();

  TelePkt pkt{};

  for (;;) {
    disturbance_snapshot(esp_timer_get_time()); // Expire even without encoder samples.
    char event[220];
    // Mesmo motivo acima: exigir 220 bytes livres no FIFO (max. 128) fazia
    // nenhuma mensagem DIST sair; a fila enchia e tudo virava BACKPRESSURE.
    if (disturbance_pop_message(event + 1, sizeof(event) - 1)) {
      event[0] = '\n'; // Isolate from existing multipart diagnostic logs.
      TEL.write((const uint8_t*)event, strlen(event));
    }
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
