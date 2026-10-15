#include <Arduino.h>
#include "config.h"
#include "filas.h"
#include "encoder.h"
#include "motor.h"
#include "control.h"
#include "params.h"
#include "serial_cmd.h"
#include "serial_tx.h"
#include "driver_tmc.h"
#include "calibration.h"

void setup(){
  Serial.begin(115200);
  Serial.setTxBufferSize(2048);   

  filas_init();

  encoder_pendulo_isr_init();
  tmc_init();
  motor_init();
  vTaskDelay(pdMS_TO_TICKS(300));

  encoder_task_start();
  calibration_begin();
  motor_task_start();

  params_begin();          
  serial_cmd_start();      
  serial_tx_task_start(); 

  control_task_start();
}


void loop(){
  vTaskDelay(pdMS_TO_TICKS(1000));
}
