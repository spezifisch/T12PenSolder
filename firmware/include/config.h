/*
 * Copyright 2023 spezifisch <spezifisch23@proton.me> https://github.com/spezifisch
 *
 * This file is part of T12PenSolder.
 *
 * T12PenSolder is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, version 3 of the
 * License.
 *
 * T12PenSolder is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with Foobar. If not, see
 * <https://www.gnu.org/licenses/>.
 */

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

// NOTE there are also some soldering tip related constants in ./solderingtip.h

#endif /* CONFIG_H */
