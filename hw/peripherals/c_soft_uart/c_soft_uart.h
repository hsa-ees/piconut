/**
 * @file softuart.h
 */

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
#include <base.h>

#include <vector>
#include <cstring> // For memcpy, if needed
#include <stdio.h> // For snprintf
#include <cstdint> // For fixed width integer types

#include <fstream>
#include <iomanip>
#include <iostream>

// #define debug

// definitions of the different registers as structs
typedef struct
{

    uint8_t data = 0x0;
    bool full = 0x0;
} txdata_t;

typedef struct
{

    uint8_t data = 0x0;
    bool empty = 0x0;
} rxdata_t;

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

// struct of all registers of the uart
typedef struct
{

    txdata_t txdata;
    rxdata_t rxdata;
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
    uart_regs_t uart_registers;
    std::vector<uint8_t> tx_fifo;
    std::vector<uint8_t> rx_fifo;
    uint64_t size;
    uint64_t base_address; // Starting address of memory

    /**
     * @brief Construct a new Soft Uart object
     *
     * @param size address space of the peripheral
     * @param base_address base address of the peripheral in address space of the simulation
     */
    c_soft_uart(uint64_t size, uint64_t base_address) : size(size), uart_registers(uart_registers), base_address(base_address), tx_fifo(0), rx_fifo(0)
    {
    }
    ~c_soft_uart() {}

    const char *get_info() override
    {
        static char info[128]; // Static to avoid memory management issues
        snprintf(info, sizeof(info), "Name: %s,\nBase Address: 0x%llX ,\nSize: %d B\n", name, base_address, size);
        return info;
    }

    //TODO Implement the read and write functions for 8 bit access
    uint8_t read8(uint64_t adr) override
    {
        uint32_t data_u32 = this->read32(adr);
        uint8_t internal_address = adr - this->base_address;

        // select the correct byte from the 32 bit data

        return (data_u32 >> (internal_address % 4) * 8) & 0xFF;

    }

    void write8(uint64_t adr, uint8_t data) override
    {

        uint32_t data_u32 = this->read32(adr);
        uint8_t internal_address = adr - this->base_address;

        // clear the byte in the 32 bit data
        data_u32 &= ~(0xFF << (internal_address % 4) * 8);

        // set the byte in the 32 bit data
        data_u32 |= data << (internal_address % 4) * 8;

        this->write32(adr, data_u32);

    }

    uint32_t read32(uint64_t adr) override
    {

        uint8_t internal_address = adr - this->base_address;
        uint32_t data_value = 0;

        switch (internal_address)
        {
        case SOFT_UART_TXDATA:
            data_value |= (this->uart_registers.txdata.full << 31) & 0x1;
            data_value |= this->uart_registers.txdata.data;
            break;

        case SOFT_UART_RXDATA:
            data_value |= (this->uart_registers.rxdata.empty << 31) & 0x1;
            data_value |= this->uart_registers.rxdata.data;
            break;

        case SOFT_UART_TXCTRL:
            data_value |= (this->uart_registers.txctrl.txen & 0x1);
            data_value |= ((this->uart_registers.txctrl.nstop << 1) & 0x1);
            data_value |= (this->uart_registers.txctrl.txcnt << 16) & 0x7;
            break;

        case SOFT_UART_RXCTRL:

            data_value |= (this->uart_registers.rxctrl.rxen & 0x1);
            data_value |= (this->uart_registers.rxctrl.rxcnt << 16) & 0x7;
            break;

        case SOFT_UART_IE:

            data_value |= this->uart_registers.ie.txwm & 0x1;
            data_value |= (this->uart_registers.ie.rxwm << 1) & 0x1;
            break;

        case SOFT_UART_IP:

            data_value |= (this->uart_registers.ip.txwm) & 0x1;
            data_value |= (this->uart_registers.ip.rxwm << 1) & 0x1;
            break;

        case SOFT_UART_DIV:

            data_value |= this->uart_registers.div.div & 0xFFFF;
            break;
        }

        return data_value;
    }

    void write32(uint64_t adr, uint32_t data) override
    {
        uint8_t internal_address = adr - this->base_address;
        switch (internal_address)
        {
        case SOFT_UART_TXDATA:
            this->uart_registers.txdata.data = data & 0xFF;
            tx_fifo.insert(tx_fifo.begin(), this->uart_registers.txdata.data);
            break;

        case SOFT_UART_RXDATA:
            this->uart_registers.rxdata.data = data & 0xFF;
            break;

        case SOFT_UART_TXCTRL:
            this->uart_registers.txctrl.txen = data & 0x1;
            this->uart_registers.txctrl.nstop = (data >> 1) & 0x1;
            this->uart_registers.txctrl.txcnt = (data >> 16) & 0x7;
            break;

        case SOFT_UART_RXCTRL:

            this->uart_registers.rxctrl.rxen = data & 0x1;
            this->uart_registers.rxctrl.rxcnt = (data >> 16) & 0x7;
            break;

        case SOFT_UART_IE:

            this->uart_registers.ie.txwm = data & 0x1;
            this->uart_registers.ie.rxwm = (data >> 1) & 0x1;
            break;

        case SOFT_UART_IP:

            break;

        case SOFT_UART_DIV:

            this->uart_registers.div.div = data & 0xFFFF;
            break;
        }

        // update uart after write access
        this->update_uart();
    }

    bool is_addressed(uint64_t adr) override
    {
        return adr >= base_address && adr < size + base_address;
    }

    /**
     * @brief handels the transmit of the uart
     *
     * Handles the transmission of the UART by checking if txen is enabled,
     * and then sending the data from the tx_fifo if the UART is enabled and the
     * tx_fifo is not empty. The data is sent to stdout.
     */
    void handel_transmit();

    /**
     * @brief updates the txfull flag in the txdata register
     *
     * if the fifo is not full its set to false else to true
     */
    void update_txfull();

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
};

#endif