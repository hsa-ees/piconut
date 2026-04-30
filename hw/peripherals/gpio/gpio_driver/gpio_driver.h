/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2026 Johannes Hofmann <johannes.hofmann1@tha.de>
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

/** @addtogroup gpio_driver
 * @brief GPIO driver interface
 *  @{
 */
#ifndef __GPIO_DRIVER_H__
#define __GPIO_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief GPIO driver instance structure.
 */
typedef struct
{
    uintptr_t base_addr; /**< GPIO base address */
} gpio_t;

/**
 * @brief Status codes returned by GPIO driver functions.
 */
typedef enum
{
    GPIO_OK = 0,           /**< Operation successful. */
    GPIO_ERR_INVALID = -1, /**< Invalid argument (e.g. null pointer). */
} gpio_status_t;

/**
 * @brief Initializes the GPIO peripheral.
 *
 * @param[in,out] self Driver instance.
 * @param[in] base_addr Base address of the GPIO peripheral.
 * @return GPIO_OK on success, otherwise error code.
 */
gpio_status_t gpio_init(gpio_t* self, uintptr_t base_addr);

/**
 * @brief Deinitializes the GPIO peripheral.
 *
 * Disables the GPIO module by disableing all pins. The driver instance remains valid.
 *
 * @param[in,out] self Driver instance.
 * @return GPIO_OK on success, otherwise error code.
 */
gpio_status_t gpio_de_init(gpio_t* self);

/**
 * @brief Read the input bank of GPIO peripheral.
 *
 * @param[in,out] self Driver instance.
 * @param[out] value Value of the input bank.
 * @return GPIO_OK on success, otherwise error code.
 */
gpio_status_t gpio_read_input(gpio_t* self, uint32_t* value);

/**
 * @brief Read one input pin of GPIO peripheral.
 *
 * @param[in,out] self Driver instance.
 * @param[in] pin Number of the pin to read.
 * @param[out] value Value of the input pin (e.g. 0, 1).
 * @return GPIO_OK on success, otherwise error code.
 */
gpio_status_t gpio_read_input_pin(gpio_t* self, uint8_t pin, bool* value);

/**
 * @brief Write the output bank of GPIO peripheral.
 *
 * @param[in,out] self Driver instance.
 * @param[in] value Value to write to the output bank.
 * @return GPIO_OK on success, otherwise error code.
 */
gpio_status_t gpio_write_output(gpio_t* self, uint32_t value);

/**
 * @brief Write one output pin of GPIO peripheral.
 *
 * @param[in,out] self Driver instance.
 * @param[in] pin Number of the pin to write.
 * @param[in] value Value to write to the pin(e.g. 0, 1).
 * @return GPIO_OK on success, otherwise error code.
 */
gpio_status_t gpio_write_output_pin(gpio_t* self, uint8_t pin, bool value);

#endif // __GPIO_DRIVER_H__

/** @} */
