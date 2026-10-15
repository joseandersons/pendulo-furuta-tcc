#include "driver_tmc.h"
#include "config.h"

#include <Arduino.h>
#include <TMCStepper.h>

#define TMC_SERIAL Serial2

static TMC2209Stepper tmc_driver(
  &TMC_SERIAL,
  TMC2209_R_SENSE,
  TMC2209_ADDR
);

static uint16_t g_current_microsteps = TMC2209_MICROSTEPS;
static bool g_tmc_ready = false;

static bool is_valid_microsteps(uint16_t microsteps) {
  return microsteps == 1   ||
         microsteps == 2   ||
         microsteps == 4   ||
         microsteps == 8   ||
         microsteps == 16  ||
         microsteps == 32  ||
         microsteps == 64  ||
         microsteps == 128 ||
         microsteps == 256;
}

static void tmc_print_status() {
  Serial.print("[tmc] test_connection = ");
  Serial.println(tmc_driver.test_connection());

  Serial.print("[tmc] IFCNT = ");
  Serial.println(tmc_driver.IFCNT());

  Serial.print("[tmc] microsteps lido = ");
  Serial.println(tmc_driver.microsteps());

  Serial.print("[tmc] cs_actual = ");
  Serial.println(tmc_driver.cs_actual());

  Serial.print("[tmc] IOIN = 0x");
  Serial.println(tmc_driver.IOIN(), HEX);

  Serial.print("[tmc] GCONF = 0x");
  Serial.println(tmc_driver.GCONF(), HEX);

  Serial.print("[tmc] DRV_STATUS = 0x");
  Serial.println(tmc_driver.DRV_STATUS(), HEX);
}

void tmc_init() {
  g_tmc_ready = false;
  TMC_SERIAL.begin(
    115200,
    SERIAL_8N1,
    TMC2209_UART_RX_PIN,
    TMC2209_UART_TX_PIN
  );

  delay(500);

  tmc_driver.begin();

  uint8_t result = tmc_driver.test_connection();

  Serial.print("[tmc] test_connection inicial = ");
  Serial.println(result);

  if (result != 0) {
    Serial.println("[tmc] ERRO: TMC2209 nao respondeu via UART");
    Serial.println("[tmc] A configuracao por UART NAO foi aplicada");
    Serial.println("[tmc] Verifique PDN_UART, RX/TX, GND, VIO, VMOT e EN");
    return;
  }

  Serial.println("[tmc] UART OK");

  /*
    GCONF:
    bit 0 = I_scale_analog
      0 = corrente controlada por registrador/UART
      1 = referencia analogica VREF

    bit 6 = pdn_disable
    bit 7 = mstep_reg_select
    bit 8 = multistep_filt

    0x1C0 = PDN desabilitado + microstep por registrador + filtro
    Importante: deixei bit 0 = 0 para a corrente por UART funcionar direito.
  */
  tmc_driver.GCONF(0x000001C0);
  delay(50);

  tmc_driver.toff(5);
  tmc_driver.blank_time(24);

  tmc_driver.rms_current(TMC2209_CURRENT_MA);

  tmc_driver.mstep_reg_select(true);
  tmc_driver.microsteps(TMC2209_MICROSTEPS);
  g_current_microsteps = TMC2209_MICROSTEPS;
  g_tmc_ready = true;

  /*
    Para pêndulo invertido, SpreadCycle costuma responder melhor
    a variações rápidas de carga do que StealthChop.
  */
  tmc_driver.en_spreadCycle(true);

  tmc_driver.intpol(true);
  tmc_driver.pwm_autoscale(true);

  delay(300);

  Serial.print("[tmc] corrente configurada = ");
  Serial.print(TMC2209_CURRENT_MA);
  Serial.println(" mA");

  Serial.print("[tmc] microsteps configurado = ");
  Serial.println(TMC2209_MICROSTEPS);

  Serial.println("[tmc] modo = SpreadCycle");

  tmc_print_status();
}

bool tmc_is_ready() {
  return g_tmc_ready;
}

bool tmc_set_current_ma(uint16_t current_ma) {
  if (current_ma < 100 || current_ma > 2000) {
    Serial.println("[tmc] corrente fora da faixa segura");
    return false;
  }

  tmc_driver.rms_current(current_ma);

  Serial.print("[tmc] corrente = ");
  Serial.print(current_ma);
  Serial.println(" mA");

  return true;
}

bool tmc_set_microsteps(uint16_t microsteps) {
  if (!is_valid_microsteps(microsteps)) {
    Serial.println("[tmc] microsteps invalido");
    return false;
  }

  if (!g_tmc_ready) {
    Serial.println("[tmc] microsteps solicitado antes do tmc_init");
    return false;
  }

  tmc_driver.mstep_reg_select(true);
  tmc_driver.microsteps(microsteps);

  g_current_microsteps = microsteps;

  Serial.print("[tmc] microsteps = ");
  Serial.println(microsteps);

  return true;
}

uint16_t tmc_get_microsteps() {
  return g_current_microsteps;
}

void tmc_set_spreadcycle(bool enabled) {
  tmc_driver.en_spreadCycle(enabled);

  Serial.print("[tmc] modo = ");
  Serial.println(enabled ? "SpreadCycle" : "StealthChop");
}