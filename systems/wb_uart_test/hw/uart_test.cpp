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

#include "top.h"

void m_uart_test::Trace(sc_trace_file *tf, int level)
{

   PN_TRACE(tf, clk);
   PN_TRACE(tf, reset);
   PN_TRACE(tf, rx);
   PN_TRACE(tf, tx);

   PN_TRACE(tf, stb_i);
   PN_TRACE(tf, cyc_i);
   PN_TRACE(tf, we_i);
   PN_TRACE(tf, sel_i);
   PN_TRACE(tf, ack_o);
   PN_TRACE(tf, err_o);
   PN_TRACE(tf, rty_o);
   PN_TRACE(tf, addr_i);
   PN_TRACE(tf, dat_i);
   PN_TRACE(tf, dat_o);

   PN_TRACE(tf, top_wb_state);
   PN_TRACE(tf, top_wb_state_next);

   PN_TRACE(tf, c_start_write);
   PN_TRACE(tf, c_start_read);
   PN_TRACE(tf, c_finish);

   PN_TRACE(tf, top_state);
   PN_TRACE(tf, top_state_next);

   PN_TRACE(tf, c_save_start);
   PN_TRACE(tf, c_save_finished);

   PN_TRACE(tf, data_reg);

   // calling trace of submodules
   if (level >= 2)
   {
      wb_uart->Trace(tf, level);
   }
   // Internal traces
}

void m_uart_test::init_submodules()
{

   wb_uart = sc_new<m_wb_uart>("wb_uart");
   wb_uart->clk(clk);
   wb_uart->reset(reset);
   wb_uart->rx(rx);
   wb_uart->tx(tx);
   wb_uart->stb_i(stb_i);
   wb_uart->cyc_i(cyc_i);
   wb_uart->we_i(we_i);
   wb_uart->sel_i(sel_i);
   wb_uart->ack_o(ack_o);
   wb_uart->err_o(err_o);
   wb_uart->rty_o(rty_o);
   wb_uart->addr_i(addr_i);
   wb_uart->dat_i(dat_i);
   wb_uart->dat_o(dat_o);
}

void m_uart_test::proc_comb_wb()
{

   // Defaults
   top_wb_state_next = top_wb_state;

   stb_i = 0;
   cyc_i = 0;
   we_i = 0;
   c_finish = 0;

   // State Machine
   switch (top_wb_state.read())
   {

   case TOP_WB_IDLE:

      // checks of the transaction started is read or write and sets the state accordingly
      if (c_start_write.read())
      {
         top_wb_state_next = TOP_WB_WRITE;
      }
      else if (c_start_read.read())
      {
         top_wb_state_next = TOP_WB_READ;
      }
      break;

   case TOP_WB_WRITE:

      // set the signals of the wb_master interface for a write transaction

      stb_i = 1;
      cyc_i = 1;
      we_i = 1;

      top_wb_state_next = TOP_WB_WRITE_ACK;

      break;

   case TOP_WB_WRITE_ACK:

      // keep the signals set until the ack from the wb_slave is received
      // then end the transaction by changing state but only if
      // the c_start_write is not low this allows the instantiator
      // of the transaction to keep the transaction alive until it
      // is done processing the data

      stb_i = 1;
      cyc_i = 1;
      we_i = 1;

      if (ack_o.read() && !c_start_write.read())
      {
         top_wb_state_next = TOP_WB_DONE;
      }

      break;

   case TOP_WB_READ:

      // set the signals of the wb_master interface for a read transaction

      stb_i = 1;
      cyc_i = 1;

      top_wb_state_next = TOP_WB_READ_ACK;

      break;

   case TOP_WB_READ_ACK:

      // keep the signals set until the ack from the wb_slave is received
      // then end the transaction by changing state but only if
      // the c_start_read is not low this allows the instantiator
      // of the transaction to keep the transaction alive until it
      // is done processing the data

      stb_i = 1;
      cyc_i = 1;

      if (ack_o.read() && !c_start_read.read())
      {
         top_wb_state_next = TOP_WB_DONE;
      }

      break;

   case TOP_WB_DONE:
      // indicate to the instanciator of the transaction that the transaction is done
      // then go back to idle

      c_finish = 1;

      top_wb_state_next = TOP_WB_IDLE;
      break;

   default:
      break;
   }
}

void m_uart_test::proc_clk_wb()
{

   // default state of state machine on reset
   top_wb_state = TOP_WB_IDLE;

   while (true)
   {
      wait();

      // state transistion
      top_wb_state = top_wb_state_next;
   }
}

void m_uart_test::proc_comb_top()
{

   // Defaults
   top_state_next = top_state;

   c_start_read = 0;
   c_start_write = 0;
   c_save_start = 0;

   dat_i = 0;
   addr_i = 0;
   sel_i = 0x0;

   // State Machine
   switch (top_state.read())
   {

   case TOP_INIT_1:

      // set the divider register to 216 (115200 baud for a 25MHz clock)
      // start the write transaction
      c_start_write = 1;
      dat_i = 216;
      sel_i = 0x3;
      addr_i = CFG_WB_UART_BASE_ADDRESS + WB_UART_REG_DIV;

      top_state_next = TOP_INIT_2;

      break;

   case TOP_INIT_2:

      // keep the data set for the write transaction
      // and wait for the finish signal to go high meaning the transaction is done
      dat_i = 216;
      sel_i = 0x3;
      addr_i = CFG_WB_UART_BASE_ADDRESS + WB_UART_REG_DIV;

      if (c_finish.read())
      {
         top_state_next = TOP_INIT_3;
      }

      break;

   case TOP_INIT_3:

      // enable the transmitter of the wb_uart by setting the TXCTRL registers
      // bit 0 to 1 and start the write transaction
      // enshure that finished is not high to prevent a new transaction
      // before the last one is done
      dat_i = 0x1;
      sel_i = 0x1;
      addr_i = CFG_WB_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL;
      c_start_write = 1;

      if (!c_finish.read())
      {
         top_state_next = TOP_INIT_4;
      }
      break;

   case TOP_INIT_4:

      // keep the data set for the write transaction
      // and wait for the finish signal to go high meaning the transaction is done
      dat_i = 0x1;
      sel_i = 0x1;
      addr_i = CFG_WB_UART_BASE_ADDRESS + WB_UART_REG_TXCTRL;

      if (c_finish.read())
      {
         top_state_next = TOP_INIT_5;
      }

      break;

   case TOP_INIT_5:

      // enable the receiver of the wb_uart by setting the RXCTRL registers
      // bit 0 to 1 and start the write transaction
      // enshure that finished is not high to prevent a new transaction
      // before the last one is done
      dat_i = 0x1;
      sel_i = 0x1;
      addr_i = CFG_WB_UART_BASE_ADDRESS + WB_UART_REG_RXCTRL;
      c_start_write = 1;

      if (!c_finish.read())
      {
         top_state_next = TOP_INIT_6;
      }
      break;

   case TOP_INIT_6:

      // keep the data set for the write transaction
      // and wait for the finish signal to go high meaning the transaction is done
      dat_i = 0x1;
      sel_i = 0x1;
      addr_i = CFG_WB_UART_BASE_ADDRESS + WB_UART_REG_RXCTRL;

      if (c_finish.read())
      {
         top_state_next = TOP_IDLE;
      }

      break;

   case TOP_IDLE:

      // now the uart is initialized and ready for use
      // if the last transaction is done and the counter spacing out the
      // checking of the rxdata fifo has counted to the specified value

      if (!c_finish.read())
      {
         top_state_next = TOP_READ_WB;
      }

      break;

      // change after here

   case TOP_READ_WB:

      // read the rxdata register and check if the fifo empty flag is set or not
      // and set the next state accordingly

      sel_i = 0xF;
      c_start_read = 1;
      addr_i = CFG_WB_UART_BASE_ADDRESS + WB_UART_REG_RXDATA;

      if (ack_o.read() && dat_o.read()[31])
      {
         top_state_next = TOP_READ_WB_WAIT;
      }
      else if (ack_o.read() && !dat_o.read()[31])
      {
         top_state_next = TOP_READ_WB_SAVE;
      }

      break;

   case TOP_READ_WB_WAIT:

      // the fifo is empty so the transaction is ended
      // and the the state machine goes back to idle
      sel_i = 0xF;
      addr_i = CFG_WB_UART_BASE_ADDRESS + WB_UART_REG_RXDATA;

      if (c_finish.read())
      {
         top_state_next = TOP_IDLE;
      }

      break;

   case TOP_READ_WB_SAVE:

      // the fifo is not empty so the data is saved in a register
      // to do that the read transaction is kept alive until the data is saved
      // when the data is save the statemachine can continue
      sel_i = 0xF;
      addr_i = CFG_WB_UART_BASE_ADDRESS + WB_UART_REG_RXDATA;
      c_start_read = 1;
      c_save_start = 1;

      if (c_save_finished.read())
      {
         top_state_next = TOP_WRITE_WB_INIT;
      }

      break;

   case TOP_WRITE_WB_INIT:

      // check if the last transaction is done continue to the next state

      if (!cyc_i.read() && !stb_i.read())
      {

         top_state_next = TOP_WRITE_WB;
      }

      break;

   case TOP_WRITE_WB:

      // start seting the data to write to the txdata register and start
      // the write transaction

      c_start_write = 1;
      sel_i = 0x1;
      addr_i = CFG_WB_UART_BASE_ADDRESS + WB_UART_REG_TXDATA;
      dat_i = data_reg;

      top_state_next = TOP_WRITE_WB_ACK;

      break;

   case TOP_WRITE_WB_ACK:

      // wait until the transaction to txdata is done if it is done go back to idle

      sel_i = 0x1;
      addr_i = CFG_WB_UART_BASE_ADDRESS + WB_UART_REG_TXDATA;
      dat_i = data_reg;

      if (c_finish.read())
      {
         top_state_next = TOP_IDLE;
      }

      break;

   default:

      top_state_next = top_state;
      break;
   }
}

void m_uart_test::proc_clk_top()
{

   // default state of state machine on reset
   top_state = TOP_INIT_1;

   while (true)
   {
      wait();

      // state transistion
      top_state = top_state_next.read();

      // default value of the finish signal
      c_save_finished = 0;

      // if data should be saved in register do it and set the finished signal
      if (c_save_start.read())
      {
         data_reg = dat_o.read().range(7, 0);
         c_save_finished = 1;
      }
   }
}

