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

#include "c_soft_uart.h"

void c_soft_uart::handel_transmit()
{

    // if the txen disable sending data
    if (!this->uart_registers.txctrl.txen)
    {
        return;
    }

    // sends all the data in the fifo to the console
    while (this->tx_fifo.size() > 0)
    {
        std::cout << this->tx_fifo.back();
        this->tx_fifo.pop_back();
        this->update_txfull();
    }
}


void c_soft_uart::handel_interrupt()
{


    // check if the number of bytes in fifo is less then txcnt if so set txwm else clear txwm
    // only if the txwm interrupt is enabled
    if (this->tx_fifo.size() > this->uart_registers.txctrl.txcnt && this->uart_registers.ie.txwm)
    {
        this->uart_registers.ip.txwm = true;
    }
    else
    {
        this->uart_registers.ip.txwm = false;
    }

    // check if the number of bytes in fifo is greater then rxcnt if so set rxwm else clear rxwm
    // only if the rxwm interrupt is enabled
    if (this->rx_fifo.size() > this->uart_registers.rxctrl.rxcnt && this->uart_registers.ie.rxwm)
    {
        this->uart_registers.ip.rxwm = true;
    }
    else
    {
        this->uart_registers.ip.rxwm = false;
    }

}


void c_soft_uart::update_txfull()
{

    this->uart_registers.txdata.full = this->tx_fifo.size() >= 8 ? true : false;
}


void c_soft_uart::update_uart()
{

    this->handel_transmit();
    this->handel_interrupt();
}