/*
 *******************************************************************************
 * Copyright (c) 2021, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#if defined(ARDUINO_T12F030)
#include "pins_arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pin number
// This array allows to wrap Arduino pin number(Dx or x)
// to STM32 PinName (PX_n)
const PinName digitalPin[] = {
  // USB connector on the top, MCU side
  // Left Side
  PA_0,  //D0
  PA_1,  //D1
  PA_2,  //D2  - TX
  PA_3,  //D3  - RX
  PA_4,  //D4  - LED
  // Right side
  PA_5,  //D5  - SCK
  PA_6,  //D6  - MISO
  PA_7,  //D7  - MOSI
  PB_1,  //D8  - SS
  PA_9,  //D9  - SCL (TX UART Header)
  PA_10, //D10 - SDA (RX UART Header)
  PA_13, //D11 - SWDIO
  PA_14, //D12 - SWCLK
  // These two are only available on boards without a crystal:
  PF_0,
  PF_1,
};

// Analog (Ax) pin number array
const uint32_t analogInputPin[] = {
  0, //A0
  1, //A1
  2, //A2
  3, //A3
  4, //A4
  5, //A5
  6, //A6
  7, //A7
  8  //A8
};

#ifdef __cplusplus
}
#endif
#endif /* ARDUINO_* */
