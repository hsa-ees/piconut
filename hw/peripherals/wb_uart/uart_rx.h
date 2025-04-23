/**
 * @file uart_rx.h
 */

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

/**
 * @fn SC_MODULE(m_uart_rx)
 * @brief Implementation of the UART receiver
 * @author Lukas Bauer
 *
 * The UART receiver is a state machine that waits for the start bit of a UART frame.
 * It contains the following submodule:
 * - Majority Filter
 * Once the receiver detects the start bit, it starts the baudtick generation and waits for the next
 * baudtick to sample the RX line. The received bits are stored in a shift register.
 * They are then filtered by a majority filter to reduce noise.
 * When the stop bit is received, the data is stored in the data register, and the
 *`rx_finished` signal is set. The number of stop bits can be configured to 1 or 2.
 *
 * @par Ports:
 * @param[in] clk clock of the module
 * @param[in] reset reset of the module
 * @param[in] baudtick_i baudtick input
 * @param[in] baudtick_x16_i baudtick x16 input
 * @param[in] stopbit_8_9_i unset for 1 stop bit, set for 2 stop bits
 * @param[in] rx_i UART rx line
 * @param[out] data_o <16> Data Output
 * @param[out] rx_finished_o RX finished output
 * @param[out] baudtick_disable_o disable the baudtick generation if needed
 *
 */

#ifndef __UART_RX_H__
#define __UART_RX_H__

#include <systemc.h>
#include <base.h> // contains all PN_<> Macros and is part of the PicoNut
#include <elab_alloc.h>
#include "majority_filter.h"

typedef enum{
    WB_UART_RX_IDLE = 0,
    WB_UART_RX_START,
    WB_UART_RX_BIT0,
    WB_UART_RX_BIT1,
    WB_UART_RX_BIT2,
    WB_UART_RX_BIT3,
    WB_UART_RX_BIT4,
    WB_UART_RX_BIT5,
    WB_UART_RX_BIT6,
    WB_UART_RX_BIT7,
    WB_UART_RX_STOP_8,
    WB_UART_RX_STOP_9_1,
    WB_UART_RX_STOP_9_2
} e_wb_uart_rx_state;


SC_MODULE(m_uart_rx) {
public:
    /** Ports ...
     * this are the two necessary signals
     * you may add your own signals here */
    sc_in_clk   PN_NAME(clk);                   // clock signal of the module
    sc_in<bool> PN_NAME(reset);                 // reset for the module

    sc_in<bool> PN_NAME(baudtick_i);            // baudtick input
    sc_in<bool> PN_NAME(baudtick_x16_i);        // baudtick x16 input
    sc_in<bool> PN_NAME(stopbit_8_9_i);         // unset for 1 stop bit, set for 2 stop bits
    sc_in<bool> PN_NAME(rx_i);                  // UART rx line

    sc_out<sc_uint<8>> PN_NAME(data_o);         // Data Output
    sc_out<bool> PN_NAME(rx_finished_o);        // RX finished output
    sc_out<bool> PN_NAME(baudtick_disable_o);   // disable the baudtick generation if needed



    /* Constructor... */
    SC_CTOR(m_uart_rx){
        SC_CTHREAD (proc_clk_module, clk.pos ());
        reset_signal_is(reset, true);
        SC_METHOD (proc_comb_module);
        sensitive << state << rx_i << mjv_filter_out << rx_data;
        sensitive << stopbit_8_9_i << rx_sync[2] << baudtick_i;

        init_submodules();
    }

    /* Functions...*/
    /**
     * @brief this function is used to generate a tracefile
     * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
     * use PN_TRACE(tf, signalname) inside the cpp file to add whatever Signal u need to trace.
     * @param tf this is the tracefile object
     * @param level is used as a selector when to enable a trace*/
    void Trace (sc_trace_file * tf, int level = 1);

    /** Processes...
    */

    void proc_clk_module();
    void proc_comb_module();

    // Submodules...
    m_majority_filter* majority_filter;

    // Methods...
    void init_submodules();

protected:
    /** Registers...
     * This are examples of Module Registers
     * you may add your own Registers here*/

    sc_signal<sc_uint<4>> PN_NAME(state);               // state for the UART receiver
    sc_signal<sc_uint<4>> PN_NAME(next_state);          // next state for the UART receiver
    sc_signal<bool> PN_NAME(mjv_capture);               // majority filter capture enable
    sc_signal<bool> PN_NAME(mjv_clear);                 // majority filter clear (sync reset)
    sc_signal<bool> PN_NAME(mjv_filter_out);            // majority filter output of the filter
    sc_signal<sc_uint<8>> PN_NAME(rx_data);             // shift register for the received data
    sc_signal<bool> PN_NAME(rx_bit);                    // internal rx line signal
    sc_signal<bool> PN_NAME(c_enable_shift);            // state machine control signal: enable shift register

    sc_vector<sc_signal<bool>> PN_NAME_VEC(rx_sync, 3); // clockdomain syncronization signals for rx line


};
#endif // __UART_RX_H__