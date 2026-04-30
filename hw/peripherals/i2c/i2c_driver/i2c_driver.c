/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Eikya Lagisetti <eikya.lagisetti1@tha.de>
                     2025 Johannes Fleiner <johannes.fleiner1@tha.de>
                     2025 Sebastian Ebenhöh <sebastian.moritz.ebenhöh@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg


  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/
#include "i2c_driver.h"
#include "../i2c_defs.h"

#ifndef REG32
#define REG32(addr) (*(volatile uint32_t*)(addr))
#endif

#ifndef I2C_CR1
#define I2C_CR1(self) REG32((self)->base_addr + I2C_REG_CR1)
#endif
#ifndef I2C_CR2
#define I2C_CR2(self) REG32((self)->base_addr + I2C_REG_CR2)
#endif
#ifndef I2C_DR
#define I2C_DR(self) REG32((self)->base_addr + I2C_REG_DR)
#endif
#ifndef I2C_SR1
#define I2C_SR1(self) REG32((self)->base_addr + I2C_REG_SR1)
#endif
#ifndef I2C_SR2
#define I2C_SR2(self) REG32((self)->base_addr + I2C_REG_SR2)
#endif
#ifndef I2C_CCR
#define I2C_CCR(self) REG32((self)->base_addr + I2C_REG_CCR)
#endif
#ifndef I2C_TRISE
#define I2C_TRISE(self) REG32((self)->base_addr + I2C_REG_TRISE)
#endif

// Static functions prototypes--------------------------------------
static void i2c_set_speed_mode(i2c_t* self,
    uint32_t system_clk_hz);

static i2c_status_t i2c_wait_for_flag(i2c_t* self, volatile uint32_t* reg, uint32_t mask);

// Driver functions--------------------------------------

i2c_status_t i2c_init(i2c_t* self,
    uint32_t system_clk_hz)
{
    if(!self)
    {
        return I2C_ERR_INVALID;
    }

    if(system_clk_hz == 0)
    {
        system_clk_hz = 25000000;
    }

    if(!self->scl_speed_hz)
    {
        return I2C_ERR_INVALID;
    }

    if(self->scl_speed_hz > 400000)
    {
        self->scl_speed_hz = 400000;
    }

    I2C_CR1(self) &= ~I2C_CR1_PE;

    uint32_t freqMHz = system_clk_hz / 1000000u;
    I2C_CR2(self) &= ~0x3Fu;
    I2C_CR2(self) |= (freqMHz & 0x3Fu);

    i2c_set_speed_mode(self, system_clk_hz);

    I2C_CR1(self) |= I2C_CR1_PE;

    return I2C_OK;
}

i2c_status_t i2c_de_init(i2c_t* self)
{
    if(!self)
    {
        return I2C_ERR_INVALID;
    }

    I2C_CR1(self) &= ~I2C_CR1_PE;
    return I2C_OK;
}

i2c_status_t i2c_start(i2c_t* self)
{
    if(!self)
    {
        return I2C_ERR_INVALID;
    }

    I2C_CR1(self) |= I2C_CR1_START;

    return i2c_wait_for_flag(self, &I2C_SR1(self), I2C_SR1_SB);
}

i2c_status_t i2c_stop(i2c_t* self)
{
    if(!self)
    {
        return I2C_ERR_INVALID;
    }

    I2C_CR1(self) |= I2C_CR1_STOP;

    while(I2C_CR1(self) & I2C_CR1_STOP)
        ;

    return I2C_OK;
}

i2c_status_t i2c_send_addr(i2c_t* self, uint8_t addr, bool read)
{
    i2c_status_t st = I2C_OK;

    if(!self)
    {
        return I2C_ERR_INVALID;
    }

    uint32_t addr_rw = ((uint32_t)addr << 1) | (read ? 1u : 0u);
    I2C_DR(self) = addr_rw;

    st = i2c_wait_for_flag(self, &I2C_SR1(self), I2C_SR1_ADDR);
    if(st != I2C_OK)
        return st;

    uint32_t temp = I2C_SR1(self);
    temp = I2C_SR2(self);

    return I2C_OK;
}

i2c_status_t i2c_send_data(i2c_t* self, uint8_t byte)
{
    if(!self)
    {
        return I2C_ERR_INVALID;
    }

    I2C_DR(self) = byte;

    return i2c_wait_for_flag(self, &I2C_SR1(self), I2C_SR1_TXE);
}

i2c_status_t i2c_recv_data(i2c_t* self,
    uint8_t* byte,
    uint8_t size)
{
    i2c_status_t st = I2C_OK;

    if(!self || !byte || size == 0)
    {
        return I2C_ERR_INVALID;
    }

    for(uint8_t i = 0; i < size; i++)
    {
        if(i == (size - 1))
        {
            I2C_CR1(self) &= ~I2C_CR1_ACK;
        }
        else
        {
            I2C_CR1(self) |= I2C_CR1_ACK;
        }

        st = i2c_wait_for_flag(self, &I2C_SR1(self), I2C_SR1_RXNE);
        if(st != I2C_OK)
            return st;

        byte[i] = (uint8_t)(I2C_DR(self) & 0xFFu);
    }

    return I2C_OK;
}

i2c_status_t i2c_master_send(i2c_t* self,
    uint8_t slave_addr,
    const uint8_t* data,
    uint32_t size)
{
    i2c_status_t st = I2C_OK;

    if(!self || (!data && size > 0u))
{
    return I2C_ERR_INVALID;
}

    if(I2C_SR2(self) & I2C_SR2_BUSY)
    {
        return I2C_ERR_BUSY;
    }

    st = i2c_start(self);
    if(st != I2C_OK)
        return st;

    st = i2c_send_addr(self, slave_addr, false);
    if(st != I2C_OK)
        return st;

    for(uint32_t i = 0; i < size; i++)
    {
        st = i2c_send_data(self, data[i]);
        if(st != I2C_OK)
            return st;
    }

    st = i2c_wait_for_flag(self, &I2C_SR1(self), I2C_SR1_BTF);
    if(st != I2C_OK)
        return st;

    return i2c_stop(self);
}

i2c_status_t i2c_master_recv(i2c_t* self,
    uint8_t slave_addr,
    uint8_t* data,
    uint32_t size)
{
    i2c_status_t st = I2C_OK;

    if(!self || (!data && size > 0u))
{
    return I2C_ERR_INVALID;
}

    if(I2C_SR2(self) & I2C_SR2_BUSY)
    {
        return I2C_ERR_BUSY;
    }

    st = i2c_start(self);
    if(st != I2C_OK)
        return st;

    st = i2c_send_addr(self, slave_addr, true);
    if(st != I2C_OK)
        return st;

    st = i2c_recv_data(self, data, size);
    if(st != I2C_OK)
        return st;

    return i2c_stop(self);
}

static void i2c_set_speed_mode(i2c_t* self,
    uint32_t system_clk_hz)
{
    uint16_t ccr_val = 0;

    if(self->scl_speed_hz <= 100000u)
    {

        ccr_val = (uint16_t)(system_clk_hz / (2u * self->scl_speed_hz));
        I2C_CCR(self) = ccr_val;
        uint32_t system_clk_mhz = system_clk_hz / 1000000u;
        I2C_TRISE(self) = system_clk_mhz + 1u;
    }
    else
    {
        ccr_val = (uint16_t)(system_clk_hz / (3u * self->scl_speed_hz));
        I2C_CCR(self) = (uint32_t)((1u << 15) | ccr_val);
        uint32_t system_clk_mhz = system_clk_hz / 1000000u;
        I2C_TRISE(self) = ((system_clk_mhz * 300u) / 1000u) + 1u;
    }
}

static i2c_status_t i2c_wait_for_flag(i2c_t* self,
    volatile uint32_t* reg,
    uint32_t mask)
{
    while(((*reg) & mask) == 0u)
    {
        if(I2C_SR1(self) & I2C_SR1_AF)
        {
            I2C_CR1(self) |= I2C_CR1_STOP;
            return I2C_ERR_NACK;
        }
    }
    return I2C_OK;
}
