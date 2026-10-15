#pragma once
#include <Arduino.h>

#define CORE_CTRL 0
#define CORE_IO   1

#define PRIO_CONTROL     (configMAX_PRIORITIES-2)
#define PRIO_ENCODER     (configMAX_PRIORITIES-3)
#define PRIO_MOTOR       (configMAX_PRIORITIES-3)



#define PERIOD_CONTROL_MS 2
#define PERIOD_ENCODER_MS 2
#define MOTOR_UPDATE_MS 2

#define DIR_PIN 33  
#define STEP_PIN 32
#define SERIAL_PORT Serial
#define TMC2209_UART_RX_PIN 16
#define TMC2209_UART_TX_PIN 17

#define TMC2209_R_SENSE 0.11f
#define TMC2209_ADDR 0b00

#define TMC2209_CURRENT_MA 1200
#define TMC2209_MICROSTEPS 256


// #define ENCA_PENDULO 35
// #define ENCB_PENDULO 34


#define STEP_CHANNEL 0
#define STEP_PWM_RESOLUTION 8

#define STEP_FREQ_MIN 10
#define STEP_FREQ_MAX 23000


#define SETPOINT_DEG 0.0f
#define MOTOR_STEPS_PER_REV 200.0f
#define MOTOR_MICROSTEPS TMC2209_MICROSTEPS
#define MOTOR_GEAR_RATIO 1.0f

#define ARM_ACCEL_MAX_RAD_S2 120.0f

#define ALPHA_ENABLE_RAD 0.261799f // 15 degrees
//#define ALPHA_ENABLE_RAD 0.174533f // 10 degrees
//#define ALPHA_ENABLE_RAD 0.523599f // 30 degrees

#define ALPHA_DOT_WINDOW 10

#define CPR_PENDULO 4096.0f
#define AS5600_BRACO_SDA_PIN 21
#define AS5600_BRACO_SCL_PIN 22
#define AS5600_PENDULO_SDA_PIN 18
#define AS5600_PENDULO_SCL_PIN 19
#define AS5600_I2C_CLOCK 400000L

// Configuracao volatil dos AS5600 aplicada em toda inicializacao.
// SF=2: filtro lento 4x. FTH=1: filtro rapido a partir de 6 LSB.
#define AS5600_PENDULO_POWER_MODE 0u
#define AS5600_PENDULO_HYSTERESIS 0u
#define AS5600_PENDULO_SLOW_FILTER 2u
#define AS5600_PENDULO_FAST_FILTER_THRESHOLD 1u
#define AS5600_PENDULO_WATCHDOG 0u

// O braco recebe os mesmos ajustes internos do pendulo.
#define AS5600_BRACO_POWER_MODE AS5600_PENDULO_POWER_MODE
#define AS5600_BRACO_HYSTERESIS AS5600_PENDULO_HYSTERESIS
#define AS5600_BRACO_SLOW_FILTER AS5600_PENDULO_SLOW_FILTER
#define AS5600_BRACO_FAST_FILTER_THRESHOLD AS5600_PENDULO_FAST_FILTER_THRESHOLD
#define AS5600_BRACO_WATCHDOG AS5600_PENDULO_WATCHDOG

// Histerese digital aplicada aos angulos enviados ao controle.
// A leitura bruta continua disponivel para calibracao e diagnostico.
#define AS5600_PENDULO_SOFTWARE_HYSTERESIS_COUNTS 2
#define AS5600_BRACO_SOFTWARE_HYSTERESIS_COUNTS AS5600_PENDULO_SOFTWARE_HYSTERESIS_COUNTS

// #define AS5600_PENDULO_SLOW_FILTER 0u  // 16x: mais filtrado e mais atrasado
// #define AS5600_PENDULO_SLOW_FILTER 1u  // 8x
// #define AS5600_PENDULO_SLOW_FILTER 2u  // 4x
// #define AS5600_PENDULO_SLOW_FILTER 3u  // 2x: mais rápido e menos atrasado


#define ARM_JERK_MAX_RAD_S3 1200.0f

#define LQR_FINE_MICROSTEPS 256
#define LQR_DISTURBANCE_MICROSTEPS 16
#define LQR_MANUAL_MICROSTEPS_DEFAULT 256

// Supervisor fuzzy de micropassos durante o LQR.
// e_alpha pequeno: ate 2 graus; medio: 3 a 4 graus; grande: 5+ graus.
#define FUZZY_ALPHA_SMALL_FULL_DEG 2.0f
#define FUZZY_ALPHA_MEDIUM_FULL_START_DEG 3.0f
#define FUZZY_ALPHA_MEDIUM_FULL_END_DEG 4.0f
#define FUZZY_ALPHA_LARGE_FULL_DEG 5.0f
#define FUZZY_ALPHA_MAX_DEG 6.0f

// e_alpha_dot em graus/s. Ajuste estes valores se o fuzzy trocar microsteps cedo/tarde demais.
#define FUZZY_ALPHA_DOT_LOW_FULL_DEG_S 15.0f
#define FUZZY_ALPHA_DOT_LOW_END_DEG_S 40.0f
#define FUZZY_ALPHA_DOT_MED_START_DEG_S 25.0f
#define FUZZY_ALPHA_DOT_MED_FULL_START_DEG_S 50.0f
#define FUZZY_ALPHA_DOT_MED_FULL_END_DEG_S 70.0f
#define FUZZY_ALPHA_DOT_MED_END_DEG_S 95.0f
#define FUZZY_ALPHA_DOT_HIGH_START_DEG_S 80.0f
#define FUZZY_ALPHA_DOT_HIGH_FULL_DEG_S 110.0f
#define FUZZY_ALPHA_DOT_MAX_DEG_S 150.0f

// Retencao e rearme para evitar falso disturbio durante o retorno para o zero.
#define LQR_DISTURBANCE_HOLD_MS 1000
#define LQR_MICROSTEP_RAMP_INTERVAL_MS 200
#define LQR_DISTURBANCE_REARM_BLOCK_MS 1200
#define LQR_DISTURBANCE_REARM_STABLE_MS 350
#define LQR_DISTURBANCE_REARM_ALPHA_DEG 1.0f
#define LQR_DISTURBANCE_REARM_ALPHA_DOT_DEG_S 20.0f
#define LQR_MICROSTEP_STATUS_INTERVAL_MS 1000

#define SWING_UP_MICROSTEPS 16

#define THETA_REF_RATE_LIMIT_RAD_S 0.60f

// Calibracao do zero do pendulo no topo (comando ZTOP, botao na interface).
// Taxa com que o controle passa ao novo zero: 2 contagens/s ~ 0.18 graus/s.
#define ZERO_SLEW_COUNTS_PER_S 2.0f
// Maior ajuste aceito por comando ZTOP, em graus.
#define ZERO_TOP_MAX_ADJUST_DEG 5.0f

// ---------------------------------------------------------------------------
// Autozero: ajuste automatico do zero do pendulo durante o equilibrio.
// Faz automaticamente o que o "Calibrar topo" faz a mao: quando o sistema
// assenta, desloca o zero ate o alpha medio (leitura bruta) ir a zero.
// Liga sozinho ao entrar no LQR e ao fim de cada mudanca de posicao.
// Cada ajuste concluido vira um ponto (theta absoluto do braco, zero);
// com 3+ pontos ajusta a curva da inclinacao z = c + p*sen(th) + q*cos(th).
#define AZ_ENABLED_DEFAULT 1
#define AZ_SETTLE_S 1.0f               // tempo assentado antes de aprender
#define AZ_TAU_LEARN_S 3.0f            // constante de tempo do ajuste
#define AZ_LPF_TAU_S 0.5f              // media de alpha usada no ajuste
#define AZ_DONE_LPF_TAU_S 2.0f         // media lenta usada para declarar concluido
#define AZ_DONE_ALPHA_COUNTS 0.4f      // |alpha medio| para considerar zerado
#define AZ_DONE_HOLD_S 2.0f            // tempo dentro do criterio
#define AZ_MIN_LEARN_S 3.0f            // tempo minimo aprendendo
#define AZ_TIMEOUT_S 45.0f             // desiste se nao convergir
#define AZ_MAX_RATE_COUNTS_S ZERO_SLEW_COUNTS_PER_S
#define AZ_MAX_TRIM_DEG ZERO_TOP_MAX_ADJUST_DEG
#define AZ_STABLE_ALPHA_DEG 3.0f       // fora disso: disturbio, pausa
#define AZ_STABLE_ALPHA_DOT_DEG_S 40.0f
#define AZ_STABLE_THETA_DOT_DEG_S 25.0f
#define AZ_BINS 12                     // pontos da curva: 1 a cada 30 graus
#define AZ_FIT_MIN_BINS 3
#define AZ_FIT_MIN_DET 0.02f           // espalhamento minimo dos pontos
#define AZ_MAX_TILT_DEG 3.0f           // |p|,|q| maximos aceitos
#define AZ_MAX_FIT_RMS_COUNTS 1.5f     // residuo maximo do ajuste
#define CPR_BRACO 4096.0f
