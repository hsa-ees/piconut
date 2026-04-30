/*******************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)     2025 Eikya Lagisetti <eikya.lagisetti1@tha.de>
                    2025 Sebastian Ebenhöh <sebastian.moritz.ebenhoeh@tha.de>
                    2025 Johannes Fleiner <Johannes.fleiner1@tha.de>

      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    Common definitions for the UART peripheral module shared between hardware
    and software.

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

 ******************************************************************************/

#pragma once

/**
 * @addtogroup i2c_defs
 *
 * I²C register overview
 * 
 * | Offset | Register     | Name                 | Description |
 * |--------|--------------|----------------------|-------------|
 * | 0x00   | I2C_CR1      | Control Register 1   | Core control of the I²C peripheral (enable, START/STOP, ACK). |
 * | 0x04   | I2C_CR2      | Control Register 2   | Peripheral clock configuration and interrupt control. |
 * | 0x08   | I2C_OAR1     | Own Address Register | Primary own address configuration (slave mode). |
 * | 0x0C   | I2C_OAR2     | Own Address Register | Secondary own address configuration (slave mode). |
 * | 0x10   | I2C_DR       | Data Register        | 8-bit data register for transmit and receive operations. |
 * | 0x14   | I2C_SR1      | Status Register 1    | Event and error flags indicating I²C bus status. |
 * | 0x18   | I2C_SR2      | Status Register 2    | Bus state and master/slave mode indication. |
 * | 0x1C   | I2C_CCR      | Clock Control Reg.   | SCL clock prescaler and speed mode configuration. |
 * | 0x20   | I2C_TRISE    | TRISE Register       | Maximum allowed SDA/SCL rise time (Standard Mode). |
 *
 * All registers are memory-mapped and accessed via the Wishbone bus.
 * 
 * Control Register 1 (I2C_CR1)
 *
 * | Bit | Name   | Description |
 * |-----|--------|-------------|
 * | 0   | PE     | Peripheral enable |
 * | 8   | START  | Generate START condition (master mode) |
 * | 9   | STOP   | Generate STOP condition (master mode) |
 * | 10  | ACK    | Enable acknowledge generation |
 * 
 * Control Register 2 (I2C_CR2)
 *
 * | Bits  | Name        | Description |
 * |-------|-------------|-------------|
 * | 5:0   | FREQ[5:0]   | Peripheral clock frequency in MHz (basis for timing generation) |
 * 
 * Data Register (I2C_DR)
 *
 * | Bits | Name | Description |
 * |------|------|-------------|
 * | 7:0  | DR   | Transmit address and transmit/receive data byte |
 * 
 * Status Register 1 (I2C_SR1)
 *
 * | Bit | Name | Description |
 * |-----|------|-------------|
 * | 0   | SB   | START condition generated |
 * | 1   | ADDR | Address sent and ACK/NACK received |
 * | 2   | BTF  | Byte transfer finished |
 * | 6   | RXNE | Receive buffer not empty |
 * | 7   | TXE  | Transmit data register empty |
 * | 10  | AF   | Acknowledge failure (NACK received) |
 *
 * Status Register 2 (I2C_SR2)
 *
 * | Bit | Name | Description |
 * |-----|------|-------------|
 * | 0   | MSL  | Master/slave mode |
 * | 1   | BUSY | Bus busy indication |
 * | 2   | TRA  | Transmitter/receiver mode |
 *
 * Clock Control Register (I2C_CCR)
 *
 * | Bit  | Name | Description |
 * |------|------|-------------|
 * | 15   | FS   | I²C speed selection (0 = Standard Mode) |
 * | 11:0 | CCR  | Clock control prescaler value |
 *
 * TRISE Register (I2C_TRISE)
 *
 * | Bits | Name  | Description |
 * |------|-------|-------------|
 * | 5:0  | TRISE | Maximum allowed SDA/SCL rise time |
 */


#ifndef PN_CFG_I2C_BASE_ADDRESS
/** @brief Base address of the I2C peripheral in the system
 *
 *  This defines the start address of the I2C registers in the
 *  Wishbone address space.
 */
#define PN_CFG_I2C_BASE_ADDRESS 0x70000000U
#endif

#ifndef PN_CFG_WB_DATA_WIDTH
#define PN_CFG_WB_DATA_WIDTH 32
#endif

#ifndef PN_CFG_WB_ADDR_WIDTH
#define PN_CFG_WB_ADDR_WIDTH 32
#endif

#ifndef PN_I2C_REG_COUNT
#define PN_I2C_REG_COUNT 10
#endif

#ifndef PN_I2C_REG_STRIDE
#define PN_I2C_REG_STRIDE 4
#endif

#ifndef PN_I2C_REG_SIZE_BYTES
#define PN_I2C_REG_SIZE_BYTES (PN_I2C_REG_COUNT * PN_I2C_REG_STRIDE)
#endif

#ifndef REGS_DAT_WIDTH
#define REGS_DAT_WIDTH 32
#endif

/************************ Register interface **********************************/

typedef enum
{
    I2C_REG_CR1 = 0x00,
    I2C_REG_CR2 = 0x04,
    I2C_REG_OAR1 = 0x08,
    I2C_REG_OAR2 = 0x0C,
    I2C_REG_DR = 0x10,
    I2C_REG_SR1 = 0x14,
    I2C_REG_SR2 = 0x18,
    I2C_REG_CCR = 0x1C,
    I2C_REG_TRISE = 0x20,
    I2C_REG_FLTR = 0x24
} e_i2c_registers;

// -----------------------------------------------------------------------------
// I²C control/status bit masks
// -----------------------------------------------------------------------------

enum
{
    I2C_CR1_PE = (1u << 0),
    I2C_CR1_START = (1u << 8),
    I2C_CR1_STOP = (1u << 9),
    I2C_CR1_ACK = (1u << 10),
    I2C_CR1_SWRST = (1u << 15)
};
enum
{
    I2C_CR2_FREQ_MASK = 0x3Fu
};
enum
{
    I2C_SR1_SB = (1u << 0),
    I2C_SR1_ADDR = (1u << 1),
    I2C_SR1_BTF = (1u << 2),
    I2C_SR1_RXNE = (1u << 6),
    I2C_SR1_TXE = (1u << 7),
    I2C_SR1_BERR = (1u << 8),
    I2C_SR1_AF = (1u << 10)
};
enum
{
    I2C_SR2_MSL = (1u << 0),
    I2C_SR2_BUSY = (1u << 1),
    I2C_SR2_TRA = (1u << 2)
};
enum
{
    I2C_CCR_FS = (1u << 15)
};
