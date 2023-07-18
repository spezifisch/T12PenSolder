#ifndef CONFIG_H
#define CONFIG_H

#include <pins_arduino.h>

// i2c bitbanged
static constexpr int SDA_PIN = PA7;
static constexpr int SCL_PIN = PA6;
// buttons (high active)
static constexpr int BUT_MINUS = PA4;
static constexpr int BUT_PLUS = PA5;
static constexpr int BUT_SET = PB1;
// heating output
static constexpr int TIPHEAT_DRV = PA3;
// adc inputs
static constexpr int VIN_MEAS = PA1;
static constexpr int TIPTEMP_MEAS = PA2;

// default settings
static constexpr uint32_t DEFAULT_TEMPERATURE_degC = 320;
static constexpr uint32_t DEFAULT_STANDBY_TIME_ms = 60000;

#endif /* CONFIG_H */
