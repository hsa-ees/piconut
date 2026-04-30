/*******************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    State definitions for UART communication

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





/************************ UART states *******************************/


typedef enum
{
    WB_UART_WB_IDLE = 0,
    WB_UART_WB_READ,
    WB_UART_WB_READ_WAIT,
    WB_UART_WB_WRITE,
    WB_UART_WB_WRITE_WAIT,
} e_uart_wb_states;

typedef enum
{
    WB_UART_TOP_FIFO_IDLE = 0,
    WB_UART_TOP_FIFO_RX_ACK,
    WB_UART_TOP_FIFO_RX_READ,
    WB_UART_TOP_FIFO_TX_WRITE,
    WB_UART_TOP_FIFO_TX_ACK,
} e_uart_wb_fifo_states;

typedef enum
{
    WB_UART_TOP_TX_IDLE = 0,
    WB_UART_TOP_TX_START,
    WB_UART_TOP_TX_WAIT,
    WB_UART_TOP_TX_RUN,
} e_uart_tx_states;

typedef enum
{
    WB_UART_TOP_RX_IDLE = 0,
    WB_UART_TOP_RX_WAIT1,
    WB_UART_TOP_RX_SAVE,
    WB_UART_TOP_RX_WAIT2
} e_uart_rx_states;