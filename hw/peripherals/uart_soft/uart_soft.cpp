/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Lukas Bauer <lukas.bauer1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_soft_uart  simulation ONLY

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

#include "uart_soft.h"

c_soft_uart::c_soft_uart(uint64_t size, uint64_t base_address)
    : size(size)
    , base_address(base_address)
    , uart_registers()
    , tx_fifo()
    , rx_fifo()
    , tty_available(false)
{
    // Only perform termios/fcntl operations if stdin is a TTY.
    tty_available = isatty(STDIN_FILENO);
    if(tty_available)
    {
        struct termios attr;
        tcgetattr(STDIN_FILENO, &attr);
        attr.c_lflag &= ~ICANON; // disable line-by-line input
        tcsetattr(STDIN_FILENO, TCSANOW, &attr);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }
}

c_soft_uart::~c_soft_uart()
{
    if(tty_available)
    {
        struct termios attr;
        tcgetattr(STDIN_FILENO, &attr);
        attr.c_lflag |= (ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &attr);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        if(flags != -1)
        {
            fcntl(STDIN_FILENO, F_SETFL, flags & ~O_NONBLOCK);
        }
    }
}

const char* c_soft_uart::get_info()
{
    static char info[128]; // Static to avoid memory management issues
    snprintf(info, sizeof(info), "Name: %s, Size: %d B\n", name, size);
    return info;
}

uint8_t c_soft_uart::read8(uint64_t adr)
{
    uint8_t internal_address = adr - this->base_address;
    uint32_t aligned_adr = adr - (adr % 4);
    uint32_t data_u32 = this->read32(aligned_adr);

    // select the correct byte from the 32 bit data
    uint8_t selected_data = (data_u32 >> (internal_address % 4) * 8) & 0xFF;

    return selected_data;
}

void c_soft_uart::write8(uint64_t adr, uint8_t data)
{
    uint32_t data_u32 = this->read32(adr);
    uint8_t internal_address = adr - this->base_address;

    // clear the byte in the 32 bit data
    data_u32 &= ~(0xFF << (internal_address % 4) * 8);

    // set the byte in the 32 bit data
    data_u32 |= data << (internal_address % 4) * 8;

    this->write32(adr, data_u32);
}

uint32_t c_soft_uart::read32(uint64_t adr)
{
    uint8_t internal_address = adr - this->base_address;
    uint32_t data_value = 0;

    char ch;
    if(this->tty_available)
    {
        errno = 0; // clear errno to avoid reporting stale errors
        ssize_t n = ::read(STDIN_FILENO, &ch, 1);

        if(n > 0)
        {
            this->rx_fifo.insert(this->rx_fifo.begin(), ch);
            // printf("read32 adr 0x%llX data 0x%02X\n", internal_address, ch);
        }
        else if(errno != EAGAIN && errno != 0)
        {
            PN_ERRORF(("Unexpected error in uart read: %s\n", strerror(errno)));
        }
    }

    switch(internal_address)
    {
        case SOFT_UART_TXDATA:
            // [7:0]  = data (RW)
            // [30:8] = Reserved (reads as 0)
            // [31]   = full (RO)
            if(!this->tx_fifo.empty())
            {
                data_value |= (this->tx_fifo.front() & 0xFF);
            }
            data_value |= (static_cast<uint32_t>(this->get_txfull_value()) << 31);
            break;

        case SOFT_UART_RXDATA:
            if(rx_fifo.size() > 0)
            {
                data_value = this->rx_fifo.back();
                this->rx_fifo.pop_back();
            }
            else
            {
                data_value = (1 << 31);
            }
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

void c_soft_uart::write32(uint64_t adr, uint32_t data)
{
    uint8_t internal_address = adr - this->base_address;
    switch(internal_address)
    {
        case SOFT_UART_TXDATA:
            tx_fifo.insert(tx_fifo.begin(), data & 0xFF);
            break;

        case SOFT_UART_RXDATA:
            // "Writes to rxdata are ignored."
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

bool c_soft_uart::is_addressed(uint64_t adr)
{
    return adr >= base_address && adr < size + base_address;
}

void c_soft_uart::handel_transmit()
{

    // if the txen disable sending data
    if(!this->uart_registers.txctrl.txen)
    {
        return;
    }

    // sends all the data in the fifo to the console
    while(this->tx_fifo.size() > 0)
    {
        std::cout << this->tx_fifo.back();
        this->tx_fifo.pop_back();
    }
}

void c_soft_uart::handel_interrupt()
{

    // check if the number of bytes in fifo is less then txcnt if so set txwm else clear txwm
    // only if the txwm interrupt is enabled
    if(this->tx_fifo.size() > this->uart_registers.txctrl.txcnt && this->uart_registers.ie.txwm)
    {
        this->uart_registers.ip.txwm = true;
    }
    else
    {
        this->uart_registers.ip.txwm = false;
    }

    // check if the number of bytes in fifo is greater then rxcnt if so set rxwm else clear rxwm
    // only if the rxwm interrupt is enabled
    if(this->rx_fifo.size() > this->uart_registers.rxctrl.rxcnt && this->uart_registers.ie.rxwm)
    {
        this->uart_registers.ip.rxwm = true;
    }
    else
    {
        this->uart_registers.ip.rxwm = false;
    }
}

bool c_soft_uart::get_txfull_value()
{

    return this->tx_fifo.size() >= 8;
}

void c_soft_uart::update_uart()
{

    this->handel_transmit();
    this->handel_interrupt();
}
