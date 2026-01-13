/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Lukas Bauer <lukas.bauer1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_soft_uart for simulation ONLY

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
 * @addtogroup c_soft_uart
 * @brief soft peripheral implementation of the sifive UART for simulation
 * @author Lukas Bauer
 *
 * This module is used to simulate the UART peripheral of the SiFive core.
 * The module is implemented as a soft peripheral and can be used in the
 * simulation environment. The module has the same registers as the SiFive
 * UART. They are:
 * - `txdata`: 32-bit register for the transmit data
 * - `rxdata`: 32-bit register for the receive data
 * - `txctrl`: 32-bit register for the transmit control
 * - `rxctrl`: 32-bit register for the receive control
 * - `ie`: 32-bit register for the interrupt enable
 * - `ip`: 32-bit register for the interrupt pending
 * - `div`: 32-bit register for the baud rate divider
 *
 * The module has a 32-bit memory interface and can be accessed by the
 * soft peripheral interface.
 *
 * Note: At the moment, the module only implements transmit functionality,
 * meaning it can only display data on `stdout`, but cannot receive data on `stdin`.
 *
 * Note: The module is not synthesizable and is only used for simulation purposes.
 *
 * Note: The module does not simulate the baud rate because it is not needed for the simulation,
 * meaning the `div` register has no effect on the module.
 *
 */

#ifndef __SOFTUART_H__
#define __SOFTUART_H__

#include "c_soft_peripheral.h"
#include <piconut.h>

#include <vector>
#include <cstring>  // For memcpy, if needed
#include <stdio.h>  // For snprintf
#include <cstdint>  // For fixed width integer types
#include <unistd.h> // FILE_NO
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

#include <fstream>
#include <iomanip>
#include <iostream>

// definitions of the different registers as structs
typedef struct
{
    bool txen = 0x0;
    bool nstop = 0x0;
    uint8_t txcnt = 0x0;
} txctrl_t;

typedef struct
{
    bool rxen = 0x0;
    uint8_t rxcnt = 0x0;
} rxctrl_t;

typedef struct
{
    bool txwm = 0x0;
    bool rxwm = 0x0;
} ie_t;

typedef struct
{
    bool txwm = 0x0;
    bool rxwm = 0x0;
} ip_t;

typedef struct
{
    uint16_t div = 0x0;
} bauddiv_t;

// struct of simple registers of the uart (simple as in not fifo based)
typedef struct
{
    txctrl_t txctrl;
    rxctrl_t rxctrl;
    ie_t ie;
    ip_t ip;
    bauddiv_t div;
} uart_regs_t;

// register address offsets
typedef enum
{
    SOFT_UART_TXDATA = 0x00,
    SOFT_UART_RXDATA = 0x04,
    SOFT_UART_TXCTRL = 0x08,
    SOFT_UART_RXCTRL = 0x0C,
    SOFT_UART_IE = 0x10,
    SOFT_UART_IP = 0x14,
    SOFT_UART_DIV = 0x18
} e_soft_uart_regs;

class c_soft_uart : public c_soft_peripheral
{
public:
    char name[32] = "UART";
    uint64_t size;
    uint64_t base_address; // Starting address of memory

    /**
     * @brief Construct a new Soft Uart object
     *
     * @param size address space of the peripheral
     * @param base_address base address of the peripheral in address space of the simulation
     */
    c_soft_uart(uint64_t size, uint64_t base_address);

    ~c_soft_uart();

    const char* get_info() override;

    uint8_t read8(uint64_t adr) override;
    void write8(uint64_t adr, uint8_t data) override;
    uint32_t read32(uint64_t adr) override;
    void write32(uint64_t adr, uint32_t data) override;

    bool is_addressed(uint64_t adr) override;

private:
    /**
     * @brief handles the transmit of the uart
     *
     * Handles the transmission of the UART by checking if txen is enabled,
     * and then sending the data from the tx_fifo if the UART is enabled and the
     * tx_fifo is not empty. The data is sent to stdout.
     */
    void handel_transmit();

    /**
     * @brief Returns true if the TX FIFO is full.
     */
    bool get_txfull_value();

    /**
     * @brief sets the values in the ip register according to the conditions
     *
     * Checks if the interrupts are enabled and then sets the interrupt flag
     * if the cnt register has the correct value.
     */
    void handel_interrupt();

    /**
     * @brief runs handel_interrupt and handel_transmit to update the uart
     *
     */
    void update_uart();
    uart_regs_t uart_registers;
    std::vector<uint8_t> tx_fifo;
    std::vector<uint8_t> rx_fifo;
    bool tty_available = false; // whether stdin supports termios/ioctl
};

#endif
