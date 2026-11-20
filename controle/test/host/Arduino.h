#pragma once
// Host-only FreeRTOS spinlock substitute; production uses ESP32 portMUX_TYPE.
#include <mutex>
#define portMUX_TYPE std::mutex
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(m) (m)->lock()
#define portEXIT_CRITICAL(m) (m)->unlock()
