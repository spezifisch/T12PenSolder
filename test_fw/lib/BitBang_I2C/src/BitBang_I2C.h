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

// On Linux, use it as C code, not C++
#if !defined(ARDUINO) && defined(__cplusplus)
extern "C" {
#endif

typedef struct mybbi2c
{
uint8_t iSDA, iSCL; // pin numbers (0xff = disabled)
uint8_t bWire, bAlign; // use the Wire library
uint8_t iSDABit, iSCLBit; // bit numbers of the ports
uint32_t iDelay;
#ifdef _LINUX_
int file_i2c;
int iBus;
#else
volatile uint32_t *pSDADDR, *pSDAPORT; // data direction and port register addr
volatile uint32_t *pSCLDDR, *pSCLPORT;
#endif
} BBI2C;
//
// Read N bytes
//
int I2CRead(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen);
//
// Read N bytes starting at a specific I2C internal register
//
int I2CReadRegister(BBI2C *pI2C, uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen);
//
// Write I2C data
// quits if a NACK is received and returns 0
// otherwise returns the number of bytes written
//
int I2CWrite(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen);
//
// Initialize the I2C BitBang library
// Pass the pin numbers used for SDA and SCL
// as well as the clock rate in Hz
//
void I2CInit(BBI2C *pI2C, uint32_t iClock);

void I2CStop(BBI2C *pI2C);
void I2CStart(BBI2C *pI2C, uint8_t iAddr, uint8_t bRead);
void I2CSendByte(BBI2C *pI2C, uint8_t iByte);

#if !defined(ARDUINO) && defined(__cplusplus)
}
#endif

#endif //__BITBANG_I2C__

