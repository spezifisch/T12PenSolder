/**
 * Copyright (c) 2011-2023 Bill Greiman
 * This file is part of the Arduino SSD1306Ascii Library
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
/**
 * @file SSD1306AsciiAvrI2c.h
 * @brief Class for I2C displays using AvrI2c.
 */

#ifndef SSD1306ASCIIBBI2C_H
#define SSD1306ASCIIBBI2C_H
#include "SSD1306Ascii.h"
#include <BitBang_I2C.h>

/**
 * @class SSD1306AsciiBBI2c
 * @brief Class for I2C displays with BBi2c.
 *
 * Uses the AvrI2c class that is smaller and faster than the
 * Wire library.
 */
class SSD1306AsciiBBI2c : public SSD1306Ascii {
 public:
  /**
   * @brief Initialize the display controller.
   *
   * @param[in] dev A device initialization structure.
   * @param[in] i2cAddr The I2C address of the display controller.
   */
  void begin(const DevType* dev, BBI2C *_bbi2c, uint8_t i2cAddr) {
    bbi2c = _bbi2c;
    m_nData = 0;
    m_i2cAddr = i2cAddr;

    init(dev);
  }

 protected:
  void writeDisplay(uint8_t b, uint8_t mode) {
    uint8_t buf[2] = {
      0,
      b,
    };
    if (mode != SSD1306_MODE_CMD) {
      buf[0] = 0x40;
    }
    I2CWrite(bbi2c, m_i2cAddr, buf, 2);

    if (mode == SSD1306_MODE_RAM_BUF) {
      m_nData++;
    } else {
      m_nData = 0;
    }
  }

 protected:
  BBI2C *bbi2c;
  uint8_t m_i2cAddr;
  uint8_t m_nData;
};

#endif /* SSD1306ASCIIBBI2C_H */
