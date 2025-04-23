/**
 * @file uart_tx.h
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
 * @fn SC_MODULE(m_uart_tx)
 * @brief Implementation of the UART transmitter
 * @author Lukas Bauer
 *
 * This module is used to transmit data over the UART interface.
 * The data is provided to the module as an 8-bit word, and the module
 * will transmit the data over the `tx_o` line. Transmission is initiated
 * by setting the `tx_start_i` signal. The module will then transmit the data
 * and set the `tx_finish_o` signal when the transmission is complete.
 * The module also handles the stop bits. If the `stopbit_cnt_i` signal is set,
 * the module will transmit 2 stop bits. If the signal is unset, the module
 * will transmit 1 stop bit.
 *
 *
 * @par Ports:
 * @param[in] clk clock of the module
 * @param[in] reset reset of the module
 * @param[in] baudtick_i baudtick input
 * @param[in] tx_start_i start signal for transmission
 * @param[in] stopbit_cnt_i unset: 1 stopbit, set: 2 stopbits
 * @param[in] tx_data_i <8> 8-bit word to transmit
 * @param[out] tx_finish_o transmission finished signal
 * @param[out] tx_o tx line signal
 *
 */

#ifndef __UART_TX_H__
#define __UART_TX_H__

#include <systemc.h>
#include <base.h> // contains all PN_<> Macros and is part of the PicoNut

typedef enum{
    WB_UART_TX_IDLE = 0,
    WB_UART_TX_START,
    WB_UART_TX_BIT0,
    WB_UART_TX_BIT1,
    WB_UART_TX_BIT2,
    WB_UART_TX_BIT3,
    WB_UART_TX_BIT4,
    WB_UART_TX_BIT5,
    WB_UART_TX_BIT6,
    WB_UART_TX_BIT7,
    WB_UART_TX_STOP_8,
    WB_UART_TX_STOP_9_1,
    WB_UART_TX_STOP_9_2
} e_wb_uart_tx_state;

SC_MODULE(m_uart_tx) {
public:
    /** Ports ...
     * this are the two necessary signals
     * you may add your own signals here */
    sc_in_clk   PN_NAME(clk);               // clock signal of the module
    sc_in<bool> PN_NAME(reset);             // reset signal of the module

    sc_in<bool> PN_NAME(baudtick_i);        // baudtick to handel transmission
    sc_in<bool> PN_NAME(tx_start_i);        // start signal for transmission
    sc_in<bool> PN_NAME(stopbit_cnt_i);     // unset: 1 stopbit, set: 2 stopbits
    sc_in<sc_uint<8>> PN_NAME(tx_data_i);   // 8-bit word to transmit

    sc_out<bool> PN_NAME(tx_finish_o);      // transmission finished signal
    sc_out<bool> PN_NAME(tx_o);             // tx line signal




    /* Constructor... */
    SC_CTOR(m_uart_tx){
        SC_CTHREAD (proc_clk_module, clk.pos ());
        reset_signal_is(reset, true);
        SC_METHOD (proc_comb_module);
        sensitive << state << tx_data_i << tx_start_i;
        sensitive << stopbit_cnt_i;
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

protected:
    /** Registers...
     * This are examples of Module Registers
     * you may add your own Registers here*/

    sc_signal<sc_uint<4>> PN_NAME(state);       // state of the module
    sc_signal<sc_uint<4>> PN_NAME(next_state);  // next state of the module



};
#endif // __UART_TX_H__