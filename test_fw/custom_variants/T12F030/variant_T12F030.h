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
#pragma once

/*----------------------------------------------------------------------------
 *        STM32 pins number
 *----------------------------------------------------------------------------*/
// USB connector on the top, MCU side
// Left Side
#define PA0                     PIN_A0 //D0/A0
#define PA1                     PIN_A1 //D1/A1
#define PA2                     PIN_A2 //D2/A2 - TX
#define PA3                     PIN_A3 //D3/A3 - RX
#define PA4                     PIN_A4 //D4/A4 - LED
// Right side
#define PA5                     PIN_A5 //D5/A5 - SCK
#define PA6                     PIN_A6 //D6/A6 - MISO
#define PA7                     PIN_A7 //D7/A7 - MOSI
#define PB1                     PIN_A8 //D8/A8 - SS
#define PA9                     9      //D9    - SCL (TX UART header)
#define PA10                    10     //D10   - SDA (RX UART header)
#define PA13                    11     //D11   - SWDIO
#define PA14                    12     //D12   - SWCLK
// Boards without a crystal can use these pins as well:
#define PF0                     13
#define PF1                     14

// Alternate pins number
#define PA6_ALT1                (PA6 | ALT1)
#define PA7_ALT1                (PA7 | ALT1)
#define PA7_ALT2                (PA7 | ALT2)
#define PA7_ALT3                (PA7 | ALT3)
#define PB1_ALT1                (PB1 | ALT1)
#define PB1_ALT2                (PB1 | ALT2)

#define NUM_DIGITAL_PINS        15
#define NUM_ANALOG_INPUTS       9

// On-board user button
#ifndef USER_BTN
  #define USER_BTN              PNUM_NOT_DEFINED
#endif

// I2C Definitions
#define PIN_WIRE_SDA            PA10
#define PIN_WIRE_SCL            PA9

// Timer Definitions
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM14
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM16
#endif
