/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
                2026 Johannes Hofmann <johannes.hofmann1@tha.de>
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

/**
 * @file gpio_defs.h
 * @brief GPIO module definitions.
 *
 * @defgroup gpio_defs GPIO Definitions
 * @{
 */

#ifndef __GPIO_DEFS_H__
#define __GPIO_DEFS_H__

/**
 * @brief Base address of the GPIO module.
 *
 * This macro defines the memory-mapped base address where the GPIO registers
 * are located. It can be overridden by defining @c PN_CFG_GPIO_BASE_ADDRESS
 * before including this header.
 *
 * Default value: 0x40000000
 */
#ifndef PN_CFG_GPIO_BASE_ADDRESS
#define PN_CFG_GPIO_BASE_ADDRESS 0x40000000U
#endif

/**
 * @brief GPIO register offsets.
 *
 * The GPIO module follows the register layout of a SiFive GPIO module.
 * See [SiFive FE310-G000 Manual](https://sifive.cdn.prismic.io/sifive/4faf3e34-4a42-4c2f-be9e-c77baa4928c7_fe310-g000-manual-v3p2.pdf)
 * for detailed documentation.
 *
 * Enumeration of all GPIO register offsets relative to the GPIO module base address.
 *
 * | **Offset** | **Register**         | **Reset** | **Access** | **Description**    |
 * |------------|----------------------|-----------|------------|--------------------|
 * | 0x00       | GPIO_REG_INPUT_VAL   | 0         | R          | Input bank value   |
 * | 0x04       | GPIO_REG_INPUT_EN    | 0         | R/W        | Input bank enable  |
 * | 0x08       | GPIO_REG_OUTPUT_EN   | 0         | R/W        | Output bank enable |
 * | 0x0C       | GPIO_REG_OUTPUT_VAL  | 0         | R/W        | Output bank value  |
 */
typedef enum
{
    GPIO_REG_INPUT_VAL = 0x00,
    GPIO_REG_INPUT_EN = 0x04,
    GPIO_REG_OUTPUT_EN = 0x08,
    GPIO_REG_OUTPUT_VAL = 0x0C,
} e_gpio_regs;

/**
 * @brief Total addressable size of the GPIO module.
 *
 * Defines the memory span occupied by all GPIO registers (16 bytes).
 */
#define GPIO_SIZE 0x10U

/**
 * @brief Maximum number of GPIO pins per bank.
 *
 * Specifies the maximum number of individual GPIO pins that can be controlled
 * in a single GPIO bank. Also determins the max width of the GPIO registers
 * interfaced by the wishbone bus.
 */
#define GPIO_MAX_PINS 32U

/**
 * @brief Wishbone bus address width in bits.
 *
 * Specifies the width of the address bus used by the Wishbone interface
 * to communicate with the GPIO module.
 */
#define GPIO_WB_ADR_WIDTH 32U

#endif // __GPIO_DEFS_H__

/** @} */ // end of gpio_defs
