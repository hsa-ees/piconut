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

#include "uart_rx.h"

/**
 * @brief this function is used to generate a tracefile
 * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
 * use PN_TRACE(tf, signalname) here to add whatever signal u need to trace.
 * @param tf this is the tracefile object
 * @param level is used as a selector when to enable a trace*/
void m_uart_rx::Trace(sc_trace_file *tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    // Trace Submodules
    if (level >= 2)
    {
        majority_filter->Trace(tf, level);
    }

    // Trace own signals

    PN_TRACE(tf, baudtick_i);
    PN_TRACE(tf, baudtick_x16_i);
    PN_TRACE(tf, stopbit_8_9_i);
    PN_TRACE(tf, rx_i);
    PN_TRACE(tf, data_o);
    PN_TRACE(tf, rx_data);
    PN_TRACE(tf, rx_finished_o);
    PN_TRACE(tf, baudtick_disable_o)

    PN_TRACE(tf, state);
    PN_TRACE(tf, next_state);
    PN_TRACE(tf, mjv_capture);
    PN_TRACE(tf, mjv_clear);
    PN_TRACE(tf, mjv_filter_out);
    PN_TRACE_BUS(tf, rx_sync, 3);
}

// **************** Helpers *********************

void m_uart_rx::init_submodules()
{
    majority_filter = sc_new<m_majority_filter>("majority_filter");
    majority_filter->clk(clk);
    majority_filter->reset(reset);
    majority_filter->filter_i(rx_sync[2]);
    majority_filter->capture_i(baudtick_x16_i);
    majority_filter->clear_i(mjv_clear);
    majority_filter->filter_o(mjv_filter_out);
}

// **************** m_uart_rx ******************

void m_uart_rx::proc_comb_module()
{

    // defaults of the statmachine
    next_state = state;
    rx_finished_o = 0;
    rx_bit = 0;
    c_enable_shift = 0;
    baudtick_disable_o = 0;

    switch (state.read())
    {

    case WB_UART_RX_IDLE:

        // keep the baudtick generation disabled
        // this is needed to sync the baudtick generation with the rx line
        baudtick_disable_o = 1;

        // check startbit if it is present start receiving the data
        if (rx_sync[2] == 0)
        {
            next_state = WB_UART_RX_START;
        }

        break;

    case WB_UART_RX_START:

        // Wait until the first baudtick is over and the startbit is still low
        if (mjv_filter_out.read() == 0 && baudtick_i.read())
        {
            next_state = WB_UART_RX_BIT0;
        }

        break;

    case WB_UART_RX_BIT0:

        // Read bit 0 by reading the majority filter output
        // and enable the shift register to capture the data
        rx_bit = mjv_filter_out.read();
        c_enable_shift = 1;

        // if the baudtick is high go to the next bit
        if (baudtick_i.read())
        {
            next_state = WB_UART_RX_BIT1;
        }
        break;

    case WB_UART_RX_BIT1:

        // Read bit 1 by reading the majority filter output
        // and enable the shift register to capture the data
        rx_bit = mjv_filter_out.read();
        c_enable_shift = 1;

        // if the baudtick is high go to the next bit
        if (baudtick_i.read())
        {
            next_state = WB_UART_RX_BIT2;
        }
        break;

    case WB_UART_RX_BIT2:

        // Read bit 2 by reading the majority filter output
        // and enable the shift register to capture the data
        rx_bit = mjv_filter_out.read();
        c_enable_shift = 1;

        // if the baudtick is high go to the next bit
        if (baudtick_i.read())
        {
            next_state = WB_UART_RX_BIT3;
        }
        break;

    case WB_UART_RX_BIT3:

        // Read bit 3 by reading the majority filter output
        // and enable the shift register to capture the data
        rx_bit = mjv_filter_out.read();
        c_enable_shift = 1;

        // if the baudtick is high go to the next bit
        if (baudtick_i.read())
        {
            next_state = WB_UART_RX_BIT4;
        }
        break;

    case WB_UART_RX_BIT4:

        // Read bit 4 by reading the majority filter output
        // and enable the shift register to capture the data
        rx_bit = mjv_filter_out.read();
        c_enable_shift = 1;

        // if the baudtick is high go to the next bit
        if (baudtick_i.read())
        {
            next_state = WB_UART_RX_BIT5;
        }
        break;

    case WB_UART_RX_BIT5:

        // Read bit 5 by reading the majority filter output
        // and enable the shift register to capture the data
        rx_bit = mjv_filter_out.read();
        c_enable_shift = 1;

        // if the baudtick is high go to the next bit
        if (baudtick_i.read())
        {
            next_state = WB_UART_RX_BIT6;
        }
        break;

    case WB_UART_RX_BIT6:

        // Read bit 6 by reading the majority filter output
        // and enable the shift register to capture the data
        rx_bit = mjv_filter_out.read();
        c_enable_shift = 1;

        // if the baudtick is high go to the next bit
        if (baudtick_i.read())
        {
            next_state = WB_UART_RX_BIT7;
        }
        break;

    case WB_UART_RX_BIT7:

        // Read bit 7 by reading the majority filter output
        rx_bit = mjv_filter_out.read();
        c_enable_shift = 1;

        // go to the state handling either 1 or 2 stop bits
        // if the baudtick is high
        if (stopbit_8_9_i.read() && baudtick_i.read())
        {
            next_state = WB_UART_RX_STOP_9_1;
        }
        else if (!stopbit_8_9_i.read() && baudtick_i.read())
        {
            next_state = WB_UART_RX_STOP_8;
        }

        break;

    case WB_UART_RX_STOP_8:

        // only 1 stop bit present -> finish the reception
        // and set the rx finished signal
        rx_finished_o = 1;

        // if the baudtick is high go to the idle state
        if (baudtick_i.read())
        {
            next_state = WB_UART_RX_IDLE;
        }
        break;

    case WB_UART_RX_STOP_9_1:

        // 2 stop bits present -> wait for the second stop bit
        if (baudtick_i.read())
        {
            next_state = WB_UART_RX_STOP_9_2;
        }
        break;

    case WB_UART_RX_STOP_9_2:

        // 2 stop bits present -> finish the reception
        // and set the rx finished signal
        rx_finished_o = 1;

        // if the baudtick is high go to the idle state
        if (baudtick_i.read())
        {
            next_state = WB_UART_RX_IDLE;
        }
        break;

    default:

        break;
    }
}

void m_uart_rx::proc_clk_module()
{
    // Internal Variables
    sc_uint<8> rx_data_var;

    // default state of the state machine at reset
    state = WB_UART_RX_IDLE;

    // reseting the clockdomain syncronization signals
    for (size_t i = 0; i < 3; i++)
    {
        rx_sync[i] = 0;
    }

    while (true)
    {
        wait();

        // reading in the shift register value
        rx_data_var = rx_data.read();

        // state transition
        state = next_state;

        // clockdomain syncronization of the rx line
        rx_sync[2] = rx_sync[1];
        rx_sync[1] = rx_sync[0];
        rx_sync[0] = rx_i.read();


        // only clear the mayority filter and enable shifting if the baudtick is high
        // because the data is sampled form the rx line at the baudfrequency
        if (baudtick_i.read())
        {
            // Reset Majority Filter
            mjv_clear = 1;

            // Shift in new bit
            if (c_enable_shift.read())
            {
                rx_data_var = (rx_bit, rx_data_var.range(7, 1));
            }
        }
        else
        {
            mjv_clear = 0;
        }

        // Write back the internal variable and output the data on the port
        rx_data = rx_data_var;
        data_o = rx_data_var;
    }
}