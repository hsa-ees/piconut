/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
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

#include "uart_tx.h"

/**
 * @brief this function is used to generate a tracefile
 * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
 * use PN_TRACE(tf, signalname) here to add whatever signal u need to trace.
 * @param tf this is the tracefile object
 * @param level is used as a selector when to enable a trace*/
void m_uart_tx::pn_trace(sc_trace_file *tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    PN_TRACE(tf, baudtick_i);
    PN_TRACE(tf, tx_start_i);
    PN_TRACE(tf, stopbit_cnt_i);
    PN_TRACE(tf, tx_data_i);

    PN_TRACE(tf, tx_finish_o);
    PN_TRACE(tf, tx_o);

    // Internal Signals
    PN_TRACE(tf, state);
    PN_TRACE(tf, next_state);
}

// **************** Helpers *********************

// **************** m_uart_tx ******************

void m_uart_tx::proc_comb_module()
{

    // variables
    sc_uint<8> tx_data_var = tx_data_i.read();

    // defaults
    next_state = state;
    tx_finish_o = 0;
    tx_o = 1;

    switch (state.read())
    {
    case WB_UART_TX_IDLE:

        // wait for the start signal then start transmitting
        if (tx_start_i.read())
        {
            next_state = WB_UART_TX_START;
        }
        break;

    case WB_UART_TX_START:

        // Send the start bit
        tx_o = 0;

        next_state = WB_UART_TX_BIT0;
        break;

    case WB_UART_TX_BIT0:

        // Send the first bit
        tx_o = tx_data_var[0];
        next_state = WB_UART_TX_BIT1;
        break;

    case WB_UART_TX_BIT1:

        // Send the second bit
        tx_o = tx_data_var[1];
        next_state = WB_UART_TX_BIT2;
        break;

    case WB_UART_TX_BIT2:

        // Send the third bit
        tx_o = tx_data_var[2];
        next_state = WB_UART_TX_BIT3;
        break;

    case WB_UART_TX_BIT3:

        // Send the fourth bit
        tx_o = tx_data_var[3];
        next_state = WB_UART_TX_BIT4;
        break;

    case WB_UART_TX_BIT4:

        // Send the fifth bit
        tx_o = tx_data_var[4];
        next_state = WB_UART_TX_BIT5;
        break;

    case WB_UART_TX_BIT5:

        // Send the sixth bit
        tx_o = tx_data_var[5];
        next_state = WB_UART_TX_BIT6;
        break;

    case WB_UART_TX_BIT6:

        // Send the seventh bit
        tx_o = tx_data_var[6];
        next_state = WB_UART_TX_BIT7;
        break;

    case WB_UART_TX_BIT7:

        // Send the eighth bit
        tx_o = tx_data_var[7];

        // check if we need to send 1 or 2 stop bits
        // and change the state accordingly
        if (stopbit_cnt_i.read())
        {
            next_state = WB_UART_TX_STOP_9_1;
        }
        else
        {
            next_state = WB_UART_TX_STOP_8;
        }

        break;

    case WB_UART_TX_STOP_8:

        // only send 1 stop bit -> set tx_finished and send the stop bit
        tx_o = 1;
        tx_finish_o = 1;

        // check if we need to start a new transmission immediately
        // or go back to idle
        if (tx_start_i.read())
        {
            next_state = WB_UART_TX_START;
        }
        else
        {
            next_state = WB_UART_TX_IDLE;
        }
        break;

    case WB_UART_TX_STOP_9_1:

        // send the first stop bit
        tx_o = 1;
        next_state = WB_UART_TX_STOP_9_2;
        break;

    case WB_UART_TX_STOP_9_2:

        // send the second stop bit -> set tx_finished and send stopbit
        tx_o = 1;
        tx_finish_o = 1;

        // check if we need to start a new transmission immediately
        // or go back to idle
        if (tx_start_i.read())
        {
            next_state = WB_UART_TX_START;
        }
        else
        {
            next_state = WB_UART_TX_IDLE;
        }
        break;

    default:

        break;
    }
}

void m_uart_tx::proc_clk_module()
{
    // default state
    state = WB_UART_TX_IDLE;

    while (true)
    {
        wait();

        // only do a state transistion if the baudtick is high
        // to send the data with the baudfequency
        if (baudtick_i.read() == 1)
        {
            state = next_state;
        }
    }
}