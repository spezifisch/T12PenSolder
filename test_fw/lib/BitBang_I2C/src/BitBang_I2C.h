//
// Bit Bang I2C library
// Copyright (c) 2018 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
// Project started 10/12/2018
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
#ifndef __BITBANG_I2C__
#define __BITBANG_I2C__

typedef struct mybbi2c
{
    // pin numbers used for SDA and SCL
    // Note that you need to configure the pins before calling I2CInit()
    uint8_t iSDA, iSCL;

    // set the desired clock period / 2,
    // e.g. 100 kHz -> 10 us / 2 -> iDelay_us = 5
    uint32_t iDelay;
} BBI2C;

//
// Initialize the I2C BitBang library
// Note that you need to configure the pins before calling this (output, open-drain)
//
void I2CInit(BBI2C *pI2C);

//
// Transaction mode: Read N bytes
//
int I2CRead(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen);

//
// Transaction mode: Read N bytes starting at a specific I2C internal register
//
int I2CReadRegister(BBI2C *pI2C, uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen);

//
// Transaction mode: Write I2C data
// quits if a NACK is received and returns 0
// otherwise returns the number of bytes written
//
int I2CWrite(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen);

//
// Manual mode: Send start condition and address byte
//
void I2CStart(BBI2C *pI2C, uint8_t iAddr, uint8_t bRead);

//
// Manual mode: Send single byte
//
void I2CSendByte(BBI2C *pI2C, uint8_t iByte);

//
// Manual mode: Send stop condition
//
void I2CStop(BBI2C *pI2C);

#endif //__BITBANG_I2C__
