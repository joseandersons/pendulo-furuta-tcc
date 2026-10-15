#include <Arduino.h>
#include <string.h>

#include "serial_cmd.h"
#include "params.h"
#include "config.h"
#include "control.h"
#include "motor.h"
#include "encoder.h"
#include "observer.h"
#include "driver_tmc.h"
#include "microstep_control.h"
#include "calibration.h"
#include "autozero.h"

static float cmd_deg_to_rad(float deg) {
  return deg * 0.017453292519943295f;
}

static float cmd_rad_to_deg(float rad) {
  return rad * 57.29577951308232f;
}

static void serial_cmd_task(void*) {
  Serial.println("[serial] cmd task start");

  static char buf[200];
  size_t n = 0;

  for (;;) {
    {
      // Mensagens do autozero geradas no laco de controle
      char az_msg[176];
      while (autozero_pop_message(az_msg, sizeof(az_msg))) {
        Serial.println(az_msg);
      }
    }

    while (Serial.available()) {
      char c = (char)Serial.read();

      if (c == '\n' || n >= sizeof(buf) - 1) {
        buf[n] = 0;

        if (n) {
          float k_theta;
          float k_alpha;
          float k_theta_dot;
          float k_alpha_dot;
          float accel_max;
          float theta_deg;
          float theta_rad;

          float m00;
          float m01;
          float m10;
          float m11;
          float h0;
          float h1;
          int ms_auto;
          int ms_manual;
          float ztop_deg;

          if (sscanf(
                buf,
                "LQR %f %f %f %f",
                &k_theta,
                &k_alpha,
                &k_theta_dot,
                &k_alpha_dot
              ) == 4) {
            params_lqr_set(k_theta, k_alpha, k_theta_dot, k_alpha_dot);

            auto k = params_lqr_get();

            Serial.printf(
              "LQR %.6f %.6f %.6f %.6f\n",
              k.k_theta,
              k.k_alpha,
              k.k_theta_dot,
              k.k_alpha_dot
            );

          } else if (sscanf(buf, "OLR %f %f %f %f", &m00, &m01, &m10, &m11) == 4) {
            if (observer_stage_Lr(m00, m01, m10, m11)) {
              Serial.printf("OLR %.8f %.8f %.8f %.8f\n", m00, m01, m10, m11);
            } else {
              Serial.println("ERR OLR");
            }

          } else if (sscanf(buf, "OF %f %f %f %f", &m00, &m01, &m10, &m11) == 4) {
            if (observer_stage_F(m00, m01, m10, m11)) {
              Serial.printf("OF %.8f %.8f %.8f %.8f\n", m00, m01, m10, m11);
            } else {
              Serial.println("ERR OF");
            }

          } else if (sscanf(buf, "OG %f %f %f %f", &m00, &m01, &m10, &m11) == 4) {
            if (observer_stage_G(m00, m01, m10, m11)) {
              Serial.printf("OG %.8f %.8f %.8f %.8f\n", m00, m01, m10, m11);
            } else {
              Serial.println("ERR OG");
            }

          } else if (sscanf(buf, "OH %f %f", &h0, &h1) == 2) {
            if (observer_stage_H(h0, h1)) {
              Serial.printf("OH %.8f %.8f\n", h0, h1);
            } else {
              Serial.println("ERR OH");
            }

          } else if (strcmp(buf, "OAPPLY") == 0) {
            if (observer_apply_staged()) {
              Serial.println("OAPPLY OK");
            } else {
              Serial.println("ERR OAPPLY");
            }

          } else if (sscanf(buf, "AL %f", &accel_max) == 1) {
            motor_set_accel_max_rad_s2(accel_max);

            Serial.printf(
              "AL %.6f\n",
              motor_get_accel_max_rad_s2()
            );

          } else if (sscanf(buf, "TH %f", &theta_deg) == 1) {
            theta_rad = cmd_deg_to_rad(theta_deg);
            control_set_theta_target_rad(theta_rad);

            Serial.printf(
              "TH target=%.3f deg ref=%.3f deg\n",
              cmd_rad_to_deg(control_get_theta_target_rad()),
              cmd_rad_to_deg(control_get_theta_ref_rad())
            );

          } else if (sscanf(buf, "THR %f", &theta_rad) == 1) {
            control_set_theta_target_rad(theta_rad);

            Serial.printf(
              "THR target=%.6f rad ref=%.6f rad\n",
              control_get_theta_target_rad(),
              control_get_theta_ref_rad()
            );

          } else if (strcmp(buf, "TH?") == 0) {
            Serial.printf(
              "TH target=%.3f deg ref=%.3f deg\n",
              cmd_rad_to_deg(control_get_theta_target_rad()),
              cmd_rad_to_deg(control_get_theta_ref_rad())
            );

          } else if (strcmp(buf, "RTH") == 0) {
            encoder_reset_braco_zero();
            motor_reset_theta_est();
            control_reset_theta_reference();
            Serial.println("RTH OK");

          } else if (sscanf(buf, "MSAUTO %d", &ms_auto) == 1) {
            if (ms_auto == 0 || ms_auto == 1) {
              microstep_control_set_auto_enabled(ms_auto != 0);
            } else {
              Serial.println("ERR MSAUTO");
            }

          } else if (sscanf(buf, "MSMAN %d", &ms_manual) == 1) {
            if (!microstep_control_set_manual_microsteps((uint16_t)ms_manual)) {
              Serial.println("ERR MSMAN");
            }

          } else if (strcmp(buf, "MS?") == 0) {
            Serial.printf(
              "MS auto=%d manual=%u current=%u tmc=%u motor=%u\n",
              microstep_control_get_auto_enabled() ? 1 : 0,
              (unsigned)microstep_control_get_manual_microsteps(),
              (unsigned)microstep_control_get_current_microsteps(),
              (unsigned)tmc_get_microsteps(),
              (unsigned)motor_get_microsteps_runtime()
            );

          } else if (strcmp(buf, "AZ?") == 0) {
            char st[200];
            autozero_format_status(st, sizeof(st));
            Serial.println(st);

          } else if (strcmp(buf, "AZRESET") == 0) {
            autozero_reset_curve();
            Serial.println("AZRESET OK (curva e pontos apagados; salva no STOP)");

          } else if (strcmp(buf, "AZ 0") == 0 || strcmp(buf, "AZ 1") == 0) {
            autozero_set_enabled(buf[3] == '1');
            Serial.printf("AZ en=%d\n", autozero_get_enabled() ? 1 : 0);

          } else if (strcmp(buf, "ZERO?") == 0) {
            Serial.printf(
              "ZERO alvo=%.3f atual=%.3f pendente=%d\n",
              calibration_get_target_offset_counts(),
              calibration_get_current_offset_counts(),
              calibration_is_dirty() ? 1 : 0
            );

          } else if (sscanf(buf, "ZTOP %f", &ztop_deg) == 1) {
            if (calibrar_zero_ajuste_topo(ztop_deg)) {
              Serial.printf(
                "ZTOP OK delta=%.4f alvo=%.3f\n",
                ztop_deg,
                calibration_get_target_offset_counts()
              );
            } else {
              Serial.println("ZTOP ERR");
            }

          } else if (strcmp(buf, "ZSAVE") == 0) {
            if (calibration_save_if_dirty()) {
              Serial.printf(
                "ZSAVE OK offset=%.3f\n",
                calibration_get_target_offset_counts()
              );
            } else {
              Serial.println("ZSAVE ERR");
            }

          } else if (strcmp(buf, "CALIB") == 0) {
            control_stop();

            if (calibrar_zero_por_fundo()) {
              Serial.printf(
                "CALIB OK offset=%.3f\n",
                calibration_get_target_offset_counts()
              );
            } else {
              Serial.println("CALIB ERR");
            }

          } else if (strcmp(buf, "START") == 0) {
            control_request_start();
            Serial.println("START");

          } else if (strcmp(buf, "STOP") == 0) {
            control_stop();
            Serial.println("STOP");

          } else {
            Serial.println("ERR");
          }
        }

        n = 0;

      } else if (c != '\r') {
        buf[n++] = c;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void serial_cmd_start() {
  xTaskCreatePinnedToCore(
    serial_cmd_task,
    "thSER",
    4096,
    nullptr,
    PRIO_ENCODER,
    nullptr,
    CORE_IO
  );
}
