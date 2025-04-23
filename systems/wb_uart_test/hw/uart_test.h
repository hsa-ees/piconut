/**
 * @file uart_test.h
 *
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Lukas Bauer <lukas.bauer1@tha.de>
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

#ifndef __UART_TEST_H__
#define __UART_TEST_H__

#include <systemc.h>
#include "elab_alloc.h"

// Include the header files of the submodules
#include "wb_uart.h"

typedef enum{
   TOP_WB_IDLE = 0,
   TOP_WB_WRITE,
   TOP_WB_WRITE_ACK,
   TOP_WB_READ,
   TOP_WB_READ_ACK,
   TOP_WB_DONE

} e_top_wb_states;

typedef enum{
   TOP_INIT_1 = 0,
   TOP_INIT_2,
   TOP_INIT_3,
   TOP_INIT_4,
   TOP_INIT_5,
   TOP_INIT_6,
   TOP_IDLE,
   TOP_READ_WB,
   TOP_READ_WB_WAIT,
   TOP_READ_WB_SAVE,
   TOP_WRITE_WB_INIT,
   TOP_WRITE_WB,
   TOP_WRITE_WB_ACK

} e_top_states;


/**
 * @fn SC_MODULE(m_uart_test)
 * @brief a hardware module to test the wb_uart
 * @author Lukas Bauer
 *
 * This module initializes the wb_uart module via its Wishbone interface by setting the baud rate
 * and enabling RX and TX. It then periodically checks the rxdata register to see if the
 * RX FIFO is no longer empty. If that is the case, it saves the received data in a register and
 * sends it to the txdata register to be sent back. This effectively echoes the received data
 * back to the sender to check the basic functionality of the wb_uart module in hardware.
 *
 * @par Ports:
 * @param[in] clk the clock signal of the module
 * @param[in] reset the reset signal of the module
 * @param[in] rx the receive signal of the uart
 * @param[out] tx the transmit signal of the uart
 *
 *
 */
SC_MODULE(m_uart_test)
{
public:
   // Ports
   sc_in_clk                        PN_NAME(clk);           // clock of the module
   sc_in<bool>                      PN_NAME(reset);         // clock of the module
   sc_in<bool>                      PN_NAME(rx);            // uart receive signal
   sc_out<bool>                     PN_NAME(tx);            // uart transmit signal



   // Constructor/Destructors
   SC_CTOR(m_uart_test)
   {
      SC_METHOD(proc_comb_wb);
      sensitive << top_wb_state << c_start_read << c_start_write << ack_o;
      SC_CTHREAD(proc_clk_wb, clk.pos());
      reset_signal_is(reset, true);

      SC_METHOD(proc_comb_top);
      sensitive << top_state << c_finish << dat_i << ack_o << cyc_i << stb_i << c_save_finished
                   << dat_o << data_reg;
      SC_CTHREAD(proc_clk_top, clk.pos());
      reset_signal_is(reset, true);

      init_submodules();
   }

   // Functions
   void Trace(sc_trace_file * tf, int level = 1);

   // Processes
   void proc_comb_wb();
   void proc_clk_wb();
   void proc_comb_top();
   void proc_clk_top();

   void proc_clk_count();
   void proc_comb_state();

   // Submodules
   m_wb_uart *wb_uart;

protected:
   // Methods
   void init_submodules();


   // WB_State_Registers
   sc_signal<sc_uint<3>>         PN_NAME(top_wb_state);                 // state of the wb master interface
   sc_signal<sc_uint<3>>         PN_NAME(top_wb_state_next);            // next state of the wb master interface

   // Control Signals
   sc_signal<bool>               PN_NAME(c_start_write);                // starts a write transaction at the wb master
   sc_signal<bool>               PN_NAME(c_start_read);                 // starts a read transaction at the wb master
   sc_signal<bool>               PN_NAME(c_finish);                     // signals the end of a transaction at the wb master
   sc_signal<bool>               PN_NAME(c_save_finished);              // signals that the data from the wb_data_in should be saved into register
   sc_signal<bool>               PN_NAME(c_save_start);                 // signals that saving the data is finished

   // Top State
   sc_signal<sc_uint<4>>         PN_NAME(top_state);                    // state of the top module
   sc_signal<sc_uint<4>>         PN_NAME(top_state_next);               // next state of the top module

   // Register
   sc_signal<sc_uint<32>>        PN_NAME(data_reg);                     // Data Register

   // Internal Signals
   sc_signal<bool>               PN_NAME(stb_i);                        // Strobe
   sc_signal<bool>               PN_NAME(cyc_i);                        // Cycle
   sc_signal<bool>               PN_NAME(we_i);                         // Write Enable
   sc_signal<sc_uint<4>>         PN_NAME(sel_i);                        // Byte Select
   sc_signal<bool>               PN_NAME(ack_o);                        // Acknowledge
   sc_signal<bool>               PN_NAME(err_o);                        // Error
   sc_signal<bool>               PN_NAME(rty_o);                        // Retry

   sc_signal<sc_uint<32>>        PN_NAME(addr_i);                       // Address
   sc_signal<sc_uint<32>>        PN_NAME(dat_i);                        // Data in
   sc_signal<sc_uint<32>>        PN_NAME(dat_o);                        // Data out



};

#endif // __UART_TEST_H__