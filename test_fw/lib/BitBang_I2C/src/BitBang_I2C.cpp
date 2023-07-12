//
// Bit Bang I2C library
// Copyright (c) 2018-2019 BitBank Software, Inc.
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

#include <Arduino.h>
#include "BitBang_I2C.h"

inline uint8_t SDA_READ(uint8_t iSDA)
{
    // use stm32 lowlevel functions which are much faster than arduino's digitalWrite/Read wrappers
    return digital_io_read(GPIOA, STM_LL_GPIO_PIN(iSDA));
}

inline void SCL_HIGH(uint8_t iSCL)
{
    digital_io_write(GPIOA, STM_LL_GPIO_PIN(iSCL), 1);
}

inline void SCL_LOW(uint8_t iSCL)
{
    digital_io_write(GPIOA, STM_LL_GPIO_PIN(iSCL), 0);
}

inline void SDA_HIGH(uint8_t iSDA)
{
    digital_io_write(GPIOA, STM_LL_GPIO_PIN(iSDA), 1);
}

inline void SDA_LOW(uint8_t iSDA)
{
    digital_io_write(GPIOA, STM_LL_GPIO_PIN(iSDA), 0);
}

inline void my_sleep_us(int iDelay)
{
    if (iDelay > 0)
    {
        delayMicroseconds(iDelay);
    }
}

// Transmit a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//
static int i2cByteOut(BBI2C *pI2C, uint8_t b)
{
    uint8_t i, ack;
    uint8_t iSDA = pI2C->iSDA;
    uint8_t iSCL = pI2C->iSCL; // in case of bad C compiler
    int iDelay = pI2C->iDelay;

    for (i = 0; i < 8; i++)
    {
        if (b & 0x80)
        {
            SDA_HIGH(iSDA); // set data line to 1
        }
        else
        {
            SDA_LOW(iSDA); // set data line to 0
        }
        SCL_HIGH(iSCL); // clock high (slave latches data)
        my_sleep_us(iDelay);
        SCL_LOW(iSCL); // clock low
        b <<= 1;
        my_sleep_us(iDelay);
    }

    // read ack bit
    SDA_HIGH(iSDA);      // set data line for reading
    SCL_HIGH(iSCL);      // clock line high
    my_sleep_us(iDelay); // DEBUG - delay/2
    ack = SDA_READ(iSDA);
    SCL_LOW(iSCL);             // clock low
    my_sleep_us(iDelay);       // DEBUG - delay/2
    SDA_LOW(iSDA);             // data low
    return (ack == 0) ? 1 : 0; // a low ACK bit means success
}

//
// Receive a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//
static uint8_t i2cByteIn(BBI2C *pI2C, uint8_t bLast)
{
    uint8_t i;
    uint8_t b = 0;

    SDA_HIGH(pI2C->iSDA); // set data line as input
    for (i = 0; i < 8; i++)
    {
        my_sleep_us(pI2C->iDelay); // wait for data to settle
        SCL_HIGH(pI2C->iSCL);      // clock high (slave latches data)
        b <<= 1;
        if (SDA_READ(pI2C->iSDA) != 0) // read the data bit
            b |= 1;                    // set data bit
        SCL_LOW(pI2C->iSCL);           // cloc low
    }

    if (bLast)
    {
        SDA_HIGH(pI2C->iSDA); // last byte sends a NACK
    }
    else
    {
        SDA_LOW(pI2C->iSDA);
    }
    SCL_HIGH(pI2C->iSCL); // clock high
    my_sleep_us(pI2C->iDelay);
    SCL_LOW(pI2C->iSCL); // clock low to send ack
    my_sleep_us(pI2C->iDelay);
    SDA_LOW(pI2C->iSDA); // data low
    return b;
}

//
// Send I2C STOP condition
//
static void i2cEnd(BBI2C *pI2C)
{
    SDA_LOW(pI2C->iSDA); // data line low
    my_sleep_us(pI2C->iDelay);
    SCL_HIGH(pI2C->iSCL); // clock high
    my_sleep_us(pI2C->iDelay);
    SDA_HIGH(pI2C->iSDA); // data high
    my_sleep_us(pI2C->iDelay);
}

static int i2cBegin(BBI2C *pI2C, uint8_t addr, uint8_t bRead)
{
    int rc;
    SDA_LOW(pI2C->iSDA); // data line low first
    my_sleep_us(pI2C->iDelay);
    SCL_LOW(pI2C->iSCL); // then clock line low is a START signal
    addr <<= 1;
    if (bRead)
    {
        addr++; // set read bit
    }

    rc = i2cByteOut(pI2C, addr); // send the slave address and R/W bit

    return rc;
}

static inline int i2cWrite(BBI2C *pI2C, uint8_t *pData, int iLen)
{
    uint8_t b;
    int rc, iOldLen = iLen;

    rc = 1;
    while (iLen && rc == 1)
    {
        b = *pData++;
        rc = i2cByteOut(pI2C, b);

        if (rc == 1) // success
        {
            iLen--;
        }
    }                                        // for each byte
    return (rc == 1) ? (iOldLen - iLen) : 0; // 0 indicates bad ack from sending a byte
}

static inline void i2cRead(BBI2C *pI2C, uint8_t *pData, int iLen)
{
    while (iLen--)
    {
        *pData++ = i2cByteIn(pI2C, iLen == 0);
    } // for each byte
}

//
// Initialize the I2C BitBang library
//
void I2CInit(BBI2C *pI2C)
{
    if (pI2C == NULL)
        return;

    digital_io_write(GPIOA, STM_LL_GPIO_PIN(pI2C->iSDA), 1);
    digital_io_write(GPIOA, STM_LL_GPIO_PIN(pI2C->iSCL), 1);
}

void I2CStop(BBI2C *pI2C)
{
    i2cEnd(pI2C);
}

void I2CStart(BBI2C *pI2C, uint8_t iAddr, uint8_t bRead)
{
    i2cBegin(pI2C, iAddr, bRead);
}

void I2CSendByte(BBI2C *pI2C, uint8_t iByte)
{
    i2cByteOut(pI2C, iByte);
}

//
// Write I2C data
// quits if a NACK is received and returns 0
// otherwise returns the number of bytes written
//
int I2CWrite(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen)
{
    int rc = 0;

    rc = i2cBegin(pI2C, iAddr, 0);
    if (rc == 1) // slave sent ACK for its address
    {
        rc = i2cWrite(pI2C, pData, iLen);
    }
    i2cEnd(pI2C);
    return rc; // returns the number of bytes sent or 0 for error
}

//
// Read N bytes starting at a specific I2C internal register
// returns 1 for success, 0 for error
//
int I2CReadRegister(BBI2C *pI2C, uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen)
{
    int rc;

    rc = i2cBegin(pI2C, iAddr, 0); // start a write operation
    if (rc == 1)                   // slave sent ACK for its address
    {
        rc = i2cWrite(pI2C, &u8Register, 1); // write the register we want to read from
        if (rc == 1)
        {
            i2cEnd(pI2C);
            rc = i2cBegin(pI2C, iAddr, 1); // start a read operation
            if (rc == 1)
            {
                i2cRead(pI2C, pData, iLen);
            }
        }
    }
    i2cEnd(pI2C);
    return rc; // returns 1 for success, 0 for error
}
//
// Read N bytes
//
int I2CRead(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen)
{
    int rc;

    rc = i2cBegin(pI2C, iAddr, 1);
    if (rc == 1) // slave sent ACK for its address
    {
        i2cRead(pI2C, pData, iLen);
    }
    i2cEnd(pI2C);
    return rc; // returns 1 for success, 0 for error
}
