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

 /** @addtogroup i2c_driver
  * @brief I²C driver interface
 *  @{
 */
#ifndef __I2C_DRIVER_H__
#define __I2C_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief I2C driver instance structure.
 * @param[in] base_addr   Base address of the I2C peripheral.
 * @param[in] scl_speed_hz Desired SCL clock speed in Hz.
 * @ingroup i2c_driver
 */
typedef struct
{
    uint32_t base_addr; /**< I2C base address */
    uint32_t scl_speed_hz; /**< Desired SCL speed in Hz */
} i2c_t;

/**
 * @brief Status codes returned by I2C driver functions.
 * @ingroup i2c_driver
 */
typedef enum
{
    I2C_OK = 0,           /**< Operation successful. */
    I2C_ERR_INVALID = -1, /**< Invalid argument (e.g. null pointer). */
    I2C_ERR_BUSY = -3,    /**< I2C bus is currently busy. */
    I2C_ERR_NACK = -4,    /**< NACK received from slave device. */
    I2C_ERR_BUS = -5      /**< General bus error. */
} i2c_status_t;

/**
 * @brief Initializes the I2C peripheral.
 *
 * Configures frequency, CCR, TRISE and enables the I2C peripheral.
 * The driver instance must contain a valid base address and SCL speed.
 *
 * @param[in,out] self           Driver instance.
 * @param[in]     system_clk_hz  System clock frequency in Hz.
 *
 * @return I2C_OK on success, otherwise error code.
 */
i2c_status_t i2c_init(i2c_t* self,
    uint32_t system_clk_hz);

/**
 * @brief Deinitializes the I2C peripheral.
 *
 * Disables the I2C module (PE = 0). The driver instance remains valid.
 *
 * @param[in,out] self   Driver instance.
 *
 * @return I2C_OK on success or I2C_ERR_INVALID.
 */
i2c_status_t i2c_de_init(i2c_t* self);

/**
 * @brief Generates an I2C START condition.
 *
 * Waits until the START condition is acknowledged (SB flag).
 *
 * @param[in,out] self   Driver instance.
 *
 * @return I2C_OK if START succeeded, otherwise error code.
 */
i2c_status_t i2c_start(i2c_t* self);

/**
 * @brief Sends a 7-bit slave address with R/W bit.
 *
 * Sets and clears the ADDR flag.
 *
 * @param[in,out] self        Driver instance.
 * @param[in]     slave_addr  7-bit slave address.
 * @param[in]     read        true = read operation, false = write.
 *
 * @return I2C_OK or an error code.
 */
i2c_status_t i2c_send_addr(i2c_t* self,
    uint8_t slave_addr,
    bool read);

/**
 * @brief Generates an I2C STOP condition.
 *
 * Releases the bus by setting STOP=1.
 *
 * @param[in,out] self   Driver instance.
 *
 * @return I2C_OK or I2C_ERR_INVALID.
 */
i2c_status_t i2c_stop(i2c_t* self);

/**
 * @brief Sends a single data byte.
 *
 * Writes to DR and waits until TXE=1.
 *
 * @param[in,out] self   Driver instance.
 * @param[in]     byte   Byte to transmit.
 *
 * @return I2C_OK or error code.
 */
i2c_status_t i2c_send_data(i2c_t* self,
    uint8_t byte);

/**
 * @brief Receives one or more bytes from an I2C slave.
 *
 * Handles ACK/NACK depending on the remaining number of bytes.
 *
 * @param[in,out] self   Driver instance.
 * @param[out]    byte   Pointer to receive buffer.
 * @param[in]     size   Number of bytes to read.
 *
 * @return I2C_OK or error code.
 */
i2c_status_t i2c_recv_data(i2c_t* self,
    uint8_t* byte,
    uint8_t size);

/**
 * @brief Performs a complete I2C write transaction.
 *
 * Sequence: START -> address (write) -> data bytes -> STOP.
 *
 * @param[in,out] self        Driver instance.
 * @param[in]     slave_addr  7-bit slave address.
 * @param[in]     data        Pointer to transmit buffer.
 * @param[in]     size        Number of bytes to send.
 *
 * @return I2C_OK or an error code.
 */
i2c_status_t i2c_master_send(i2c_t* self,
    uint8_t slave_addr,
    const uint8_t* data,
    uint32_t size);

/**
 * @brief Performs a complete  I2C read transaction.
 *
 * Sequence: START -> address (read) -> receive bytes → STOP.
 *
 * @param[in,out] self        Driver instance.
 * @param[in]     slave_addr  7-bit slave address.
 * @param[out]    data        Pointer to receive buffer.
 * @param[in]     size        Number of bytes to read.
 *
 * @return I2C_OK or an error code.
 */
i2c_status_t i2c_master_recv(i2c_t* self,
    uint8_t slave_addr,
    uint8_t* data,
    uint32_t size);

#endif // __I2C_DRIVER_H__

/** @} */
