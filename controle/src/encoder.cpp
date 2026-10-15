#include "encoder.h"
#include "config.h"

#include <Arduino.h>
#include <Wire.h>

#define AS5600_ADDR 0x36
#define AS5600_REG_CONF_H 0x07
#define AS5600_REG_RAW_ANGLE_H 0x0C
#define AS5600_CPR_COUNTS 4096
#define AS5600_CPR 4096.0f
#define AS5600_HALF_CPR 2048

#define AS5600_CONF_PM_MASK 0x0003u
#define AS5600_CONF_HYST_MASK 0x000Cu
#define AS5600_CONF_SF_MASK 0x0300u
#define AS5600_CONF_FTH_MASK 0x1C00u
#define AS5600_CONF_WD_MASK 0x2000u

#define SENTIDO_BRACO 1
#define SENTIDO_PENDULO 1

TwoWire I2C_BRACO = TwoWire(0);
TwoWire I2C_PENDULO = TwoWire(1);

struct AS5600State {
  uint16_t raw;
  uint16_t last_raw;

  int32_t accumulated_counts;
  int32_t zero_counts;
  int32_t control_relative_counts;
  int32_t last_relative_counts;

  uint32_t last_stamp_us;

  float rad;
  float dot_rad_s;

  bool initialized;
  bool reset_kinematics_requested;
};

static AS5600State braco{};
static AS5600State pendulo{};

static portMUX_TYPE mux_encoders =
  portMUX_INITIALIZER_UNLOCKED;

static SemaphoreHandle_t mtx = nullptr;
static Obs g_obs{};

static TaskHandle_t g_control_task_handle = nullptr;
static uint32_t g_sample_id = 0;

static bool g_pendulo_control_counts_initialized = false;
static int32_t g_pendulo_control_counts = 0;

static bool as5600_read_bytes(
  TwoWire& i2c,
  uint8_t first_register,
  uint8_t* data,
  uint8_t length
) {
  if (data == nullptr || length == 0) {
    return false;
  }

  i2c.beginTransmission(
    (uint8_t)AS5600_ADDR
  );

  i2c.write(first_register);

  if (i2c.endTransmission(false) != 0) {
    return false;
  }

  uint8_t received = i2c.requestFrom(
    (uint8_t)AS5600_ADDR,
    length
  );

  if (
    received != length
    || i2c.available() < length
  ) {
    while (i2c.available()) {
      i2c.read();
    }

    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    data[i] = (uint8_t)i2c.read();
  }

  return true;
}

static bool as5600_write_bytes(
  TwoWire& i2c,
  uint8_t first_register,
  const uint8_t* data,
  uint8_t length
) {
  if (data == nullptr || length == 0) {
    return false;
  }

  i2c.beginTransmission(
    (uint8_t)AS5600_ADDR
  );

  i2c.write(first_register);

  for (uint8_t i = 0; i < length; i++) {
    i2c.write(data[i]);
  }

  return i2c.endTransmission(true) == 0;
}

static bool as5600_read_u16(
  TwoWire& i2c,
  uint8_t high_register,
  uint16_t& value
) {
  uint8_t data[2] = {0, 0};

  if (!as5600_read_bytes(i2c, high_register, data, 2)) {
    return false;
  }

  value =
    ((uint16_t)data[0] << 8)
    | (uint16_t)data[1];

  return true;
}

static bool as5600_write_u16(
  TwoWire& i2c,
  uint8_t high_register,
  uint16_t value
) {
  uint8_t data[2] = {
    (uint8_t)((value >> 8) & 0xFFu),
    (uint8_t)(value & 0xFFu)
  };

  return as5600_write_bytes(
    i2c,
    high_register,
    data,
    2
  );
}

static uint16_t as5600_desired_conf(
  uint16_t power_mode,
  uint16_t hysteresis,
  uint16_t slow_filter,
  uint16_t fast_filter_threshold,
  uint16_t watchdog
) {
  uint16_t value = 0;

  value |=
    (power_mode & 0x03u);

  value |=
    ((hysteresis & 0x03u) << 2);

  value |=
    ((slow_filter & 0x03u) << 8);

  value |=
    (
      (fast_filter_threshold & 0x07u)
      << 10
    );

  value |=
    ((watchdog & 0x01u) << 13);

  return value;
}

static bool as5600_configure_and_verify(
  TwoWire& i2c,
  const char* name,
  uint16_t desired_bits
) {
  const uint16_t controlled_mask =
    AS5600_CONF_PM_MASK
    | AS5600_CONF_HYST_MASK
    | AS5600_CONF_SF_MASK
    | AS5600_CONF_FTH_MASK
    | AS5600_CONF_WD_MASK;

  uint16_t current_conf = 0;

  if (!as5600_read_u16(
    i2c,
    AS5600_REG_CONF_H,
    current_conf
  )) {
    Serial.printf(
      "[encoder] erro ao ler CONF do AS5600 do %s\n", name
    );

    return false;
  }

  const uint16_t requested_conf =
    (current_conf & ~controlled_mask)
    | (desired_bits & controlled_mask);

  if (requested_conf != current_conf) {
    if (!as5600_write_u16(
      i2c,
      AS5600_REG_CONF_H,
      requested_conf
    )) {
      Serial.printf(
        "[encoder] erro ao escrever CONF do AS5600 do %s\n", name
      );

      return false;
    }
  }

  delay(2);

  uint16_t readback_conf = 0;

  if (!as5600_read_u16(
    i2c,
    AS5600_REG_CONF_H,
    readback_conf
  )) {
    Serial.printf(
      "[encoder] erro ao verificar CONF do AS5600 do %s\n", name
    );

    return false;
  }

  const bool verified =
    (readback_conf & controlled_mask)
    == (desired_bits & controlled_mask);

  Serial.printf(
    "[encoder] %s CONF anterior=0x%04X solicitado=0x%04X lido=0x%04X %s\n",
    name,
    current_conf,
    requested_conf,
    readback_conf,
    verified ? "OK" : "FALHOU"
  );

  return verified;
}

static bool as5600_read_raw(
  TwoWire& i2c,
  uint16_t& raw
) {
  uint16_t value = 0;

  if (!as5600_read_u16(
    i2c,
    AS5600_REG_RAW_ANGLE_H,
    value
  )) {
    return false;
  }

  raw = value & 0x0FFFu;

  return true;
}

static int32_t as5600_calculate_delta(
  uint16_t raw,
  uint16_t last_raw
) {
  int32_t delta =
    (int32_t)raw
    - (int32_t)last_raw;

  if (delta > AS5600_HALF_CPR) {
    delta -= AS5600_CPR_COUNTS;
  }

  if (delta < -AS5600_HALF_CPR) {
    delta += AS5600_CPR_COUNTS;
  }

  return delta;
}

static int32_t as5600_oriented_absolute_counts(
  uint16_t raw,
  int32_t direction
) {
  int32_t counts = (int32_t)raw;

  if (direction < 0) {
    counts =
      (AS5600_CPR_COUNTS - counts)
      % AS5600_CPR_COUNTS;
  }

  return counts;
}

static int32_t normalize_absolute_counts(int32_t counts) {
  counts %= AS5600_CPR_COUNTS;

  if (counts < 0) {
    counts += AS5600_CPR_COUNTS;
  }

  return counts;
}

static int32_t shortest_count_difference(
  int32_t target_counts,
  int32_t reference_counts
) {
  int32_t delta =
    normalize_absolute_counts(target_counts)
    - normalize_absolute_counts(reference_counts);

  if (delta > AS5600_HALF_CPR) {
    delta -= AS5600_CPR_COUNTS;
  }

  if (delta < -AS5600_HALF_CPR) {
    delta += AS5600_CPR_COUNTS;
  }

  return delta;
}

static int32_t pendulo_counts_for_control(
  int32_t absolute_counts
) {
  const int32_t normalized_counts =
    normalize_absolute_counts(absolute_counts);

  if (!g_pendulo_control_counts_initialized) {
    g_pendulo_control_counts = normalized_counts;
    g_pendulo_control_counts_initialized = true;

    return g_pendulo_control_counts;
  }

  const int32_t delta = shortest_count_difference(
    normalized_counts,
    g_pendulo_control_counts
  );

  const int32_t hysteresis_counts =
    AS5600_PENDULO_SOFTWARE_HYSTERESIS_COUNTS > 0
      ? AS5600_PENDULO_SOFTWARE_HYSTERESIS_COUNTS
      : 0;

  if (abs(delta) > hysteresis_counts) {
    g_pendulo_control_counts = normalized_counts;
  }

  return g_pendulo_control_counts;
}

static bool as5600_update(
  TwoWire& i2c,
  AS5600State& state,
  int32_t direction,
  uint32_t* sample_stamp_us,
  int32_t hysteresis_counts = 0
) {
  uint16_t raw = 0;

  if (!as5600_read_raw(i2c, raw)) {
    return false;
  }

  const uint32_t now_us = micros();

  if (sample_stamp_us != nullptr) {
    *sample_stamp_us = now_us;
  }

  bool was_initialized = false;

  portENTER_CRITICAL(&mux_encoders);

  state.raw = raw;
  was_initialized = state.initialized;

  if (!state.initialized) {
    state.last_raw = raw;
    state.last_stamp_us = now_us;
    state.initialized = true;
  }

  portEXIT_CRITICAL(&mux_encoders);

  if (!was_initialized) {
    return true;
  }

  int32_t delta =
    as5600_calculate_delta(
      raw,
      state.last_raw
    );

  delta *= direction;

  state.last_raw = raw;

  int32_t relative_counts = 0;
  bool reset_kinematics = false;

  portENTER_CRITICAL(&mux_encoders);

  state.accumulated_counts += delta;

  relative_counts =
    state.accumulated_counts
    - state.zero_counts;

  reset_kinematics =
    state.reset_kinematics_requested;

  state.reset_kinematics_requested = false;

  portEXIT_CRITICAL(&mux_encoders);

  float dt =
    (float)(now_us - state.last_stamp_us)
    * 1e-6f;

  // Filtra depois de acumular as voltas: nao perde movimentos lentos nem
  // cria descontinuidade ao cruzar 4095/0. O zero reinicia a histerese.
  const int32_t threshold = hysteresis_counts > 0 ? hysteresis_counts : 0;
  if (
    reset_kinematics
    || abs(relative_counts - state.control_relative_counts) > threshold
  ) {
    state.control_relative_counts = relative_counts;
  }

  relative_counts = state.control_relative_counts;

  state.rad =
    (float)relative_counts
    * TWO_PI
    / AS5600_CPR;

  if (reset_kinematics) {
    state.last_relative_counts =
      relative_counts;

    state.dot_rad_s = 0.0f;
    state.last_stamp_us = now_us;

    return true;
  }

  if (dt > 0.0f) {
    int32_t delta_counts =
      relative_counts
      - state.last_relative_counts;

    state.dot_rad_s =
      (
        (float)delta_counts
        * TWO_PI
        / AS5600_CPR
      )
      / dt;
  }

  state.last_relative_counts =
    relative_counts;

  state.last_stamp_us = now_us;

  return true;
}

static void encoder_reset_zero(
  AS5600State& state
) {
  portENTER_CRITICAL(&mux_encoders);

  state.zero_counts =
    state.accumulated_counts;

  state.reset_kinematics_requested = true;

  portEXIT_CRITICAL(&mux_encoders);
}

void encoder_pendulo_isr_init() {
}

void encoder_register_control_task(
  TaskHandle_t task_handle
) {
  portENTER_CRITICAL(&mux_encoders);
  g_control_task_handle = task_handle;
  portEXIT_CRITICAL(&mux_encoders);
}

bool encoder_pendulo_get_absolute_count(
  int32_t* counts
) {
  if (counts == nullptr) {
    return false;
  }

  uint16_t raw = 0;
  bool initialized = false;

  portENTER_CRITICAL(&mux_encoders);

  raw = pendulo.raw;
  initialized = pendulo.initialized;

  portEXIT_CRITICAL(&mux_encoders);

  if (!initialized) {
    return false;
  }

  *counts =
    as5600_oriented_absolute_counts(
      raw,
      SENTIDO_PENDULO
    );

  return true;
}

void encoder_reset_braco_zero() {
  encoder_reset_zero(
    braco
  );
}

bool encoder_get_obs(Obs* out) {
  if (
    mtx == nullptr
    || out == nullptr
  ) {
    return false;
  }

  if (
    xSemaphoreTake(
      mtx,
      pdMS_TO_TICKS(5)
    )
    != pdTRUE
  ) {
    return false;
  }

  *out = g_obs;

  xSemaphoreGive(mtx);

  return true;
}

static void encoder_task(void*) {
  TickType_t last =
    xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(
      &last,
      pdMS_TO_TICKS(PERIOD_ENCODER_MS)
    );

    uint32_t pendulo_stamp_us = 0;

    const bool pendulo_ok = as5600_update(
      I2C_PENDULO,
      pendulo,
      SENTIDO_PENDULO,
      &pendulo_stamp_us
    );

    as5600_update(
      I2C_BRACO,
      braco,
      SENTIDO_BRACO,
      nullptr,
      AS5600_BRACO_SOFTWARE_HYSTERESIS_COUNTS
    );

    if (!pendulo_ok) {
      continue;
    }

    Obs o{};

    int32_t pendulo_absolute_counts = 0;

    if (!encoder_pendulo_get_absolute_count(
      &pendulo_absolute_counts
    )) {
      continue;
    }

    o.pendulo_counts =
      pendulo_counts_for_control(
        pendulo_absolute_counts
      );

    o.pendulo_raw_counts =
      normalize_absolute_counts(pendulo_absolute_counts);

    o.braco_abs_counts =
      as5600_oriented_absolute_counts(
        braco.raw,
        SENTIDO_BRACO
      );

    o.braco_counts =
      braco.control_relative_counts;

    o.braco_raw =
      braco.raw;

    o.braco_rad =
      braco.rad;

    o.braco_dot_rad_s =
      braco.dot_rad_s;

    o.stamp_us =
      pendulo_stamp_us;

    bool published = false;

    if (
      xSemaphoreTake(
        mtx,
        0
      )
      == pdTRUE
    ) {
      g_sample_id++;

      if (g_sample_id == 0) {
        g_sample_id++;
      }

      o.sample_id = g_sample_id;
      g_obs = o;
      published = true;

      xSemaphoreGive(mtx);
    }

    if (published) {
      TaskHandle_t control_task_handle = nullptr;

      portENTER_CRITICAL(&mux_encoders);
      control_task_handle = g_control_task_handle;
      portEXIT_CRITICAL(&mux_encoders);

      if (control_task_handle != nullptr) {
        xTaskNotifyGive(control_task_handle);
      }
    }
  }
}

void encoder_task_start() {
  if (mtx == nullptr) {
    mtx =
      xSemaphoreCreateMutex();
  }

  I2C_BRACO.begin(
    AS5600_BRACO_SDA_PIN,
    AS5600_BRACO_SCL_PIN,
    AS5600_I2C_CLOCK
  );

  I2C_PENDULO.begin(
    AS5600_PENDULO_SDA_PIN,
    AS5600_PENDULO_SCL_PIN,
    AS5600_I2C_CLOCK
  );

  delay(20);

  as5600_configure_and_verify(
    I2C_PENDULO,
    "pendulo",
    as5600_desired_conf(
      AS5600_PENDULO_POWER_MODE,
      AS5600_PENDULO_HYSTERESIS,
      AS5600_PENDULO_SLOW_FILTER,
      AS5600_PENDULO_FAST_FILTER_THRESHOLD,
      AS5600_PENDULO_WATCHDOG
    )
  );

  as5600_configure_and_verify(
    I2C_BRACO,
    "braco",
    as5600_desired_conf(
      AS5600_BRACO_POWER_MODE,
      AS5600_BRACO_HYSTERESIS,
      AS5600_BRACO_SLOW_FILTER,
      AS5600_BRACO_FAST_FILTER_THRESHOLD,
      AS5600_BRACO_WATCHDOG
    )
  );

  uint32_t pendulo_stamp_us = 0;

  as5600_update(
    I2C_PENDULO,
    pendulo,
    SENTIDO_PENDULO,
    &pendulo_stamp_us
  );

  as5600_update(
    I2C_BRACO,
    braco,
    SENTIDO_BRACO,
    nullptr,
    AS5600_BRACO_SOFTWARE_HYSTERESIS_COUNTS
  );

  xTaskCreatePinnedToCore(
    encoder_task,
    "thENCODER",
    4096,
    nullptr,
    PRIO_ENCODER,
    nullptr,
    CORE_CTRL
  );
}
