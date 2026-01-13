/*******************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
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





/************************ Configuration Options *******************************/


#ifndef PN_CFG_UART_BASE_ADDRESS
  /** @brief Base address of the UART instance in the system
   */
#define PN_CFG_UART_BASE_ADDRESS 0x30000000U
#endif


#ifndef PN_CFG_UART_DISABLE_FIFO
#define PN_CFG_UART_DISABLE_FIFO 0
  /** @brief Disables the RX and TX FIFO of the UART
   */
#endif


#ifndef PN_CFG_UART_FIFO_WIDTH
#define PN_CFG_UART_FIFO_WIDTH 8
  /** @brief Width of the UART data interface (data_i/data_o) in bits
   */
#endif


#ifndef PN_CFG_UART_FIFO_SIZE_2E
#define PN_CFG_UART_FIFO_SIZE_2E 3
  /** @brief log2 of the size of the UART FIFO buffer
   */
#endif





/************************ Register interface **********************************/


typedef enum
{
        WB_UART_REG_TXDATA = 0,
        WB_UART_REG_RXDATA = 0x4,
        WB_UART_REG_TXCTRL = 0x8,
        WB_UART_REG_RXCTRL = 0xC,
        WB_UART_REG_IE = 0x10,
        WB_UART_REG_IP = 0x14,
        WB_UART_REG_DIV = 0x18
} e_wb_uart_registers;
