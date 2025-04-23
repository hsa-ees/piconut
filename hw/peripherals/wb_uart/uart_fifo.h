/**
 * @file uart_fifo.h
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
 * @fn SC_MODULE(m_uart_fifo)
 * @author Lukas Bauer
 * @brief first in first out buffer for UART
 *
 * This module is a simple first-in, first-out (FIFO) buffer for the UART RX and TX.
 * If the read and write pointers are equal, the buffer is empty. If the write pointer
 * and the addresses are as far apart as possible, the buffer is full.
 * The `usage` signal is a counter for the number of elements in the buffer.
 * The `clear` signal resets the read and write pointers, as well as the usage counter.
 * If `write` is set and the buffer is not full, the data is written to the buffer.
 * If `read` is set and the buffer is not empty, the data is read from the buffer.
 * The width of the data can be set using the `UART_FIFO_WIDTH` parameter.
 * The size of the buffer can be set using the `UART_FIFO_SIZE_2E` parameter,
 * which is the log2 of the size of the buffer.
 * The buffer is implemented as a circular buffer.
 *
 * @par Ports:
 * @param[in] clk clock of the module
 * @param[in] reset reset of the module
 * @param[in] clear_i clear the content of the FIFO
 * @param[in] write_i write enable for the FIFO
 * @param[in] read_i read enable for the FIFO
 * @param[in] data_i <8> data to be written to the FIFO
 * @param[out] data_o <8> data to be read from the FIFO
 * @param[out] full_o FIFO is full
 * @param[out] empty_o FIFO is empty
 * @param[out] usage_o <4> usage of the FIFO
 *
 *
 */

#ifndef __UART_FIFO_H__
#define __UART_FIFO_H__

#include <systemc.h>
#include <base.h> // contains all PN_<> Macros and is part of the PicoNut


#define UART_FIFO_WIDTH 8
#define UART_FIFO_SIZE_2E 3



SC_MODULE(m_uart_fifo) {
public:
    /** Ports ...
     * this are the two necessary signals
     * you may add your own signals here */
    sc_in_clk                               PN_NAME(clk);     // Clock of the module
    sc_in<bool>                             PN_NAME(reset);   // Reset of the module

    sc_in<bool>                             PN_NAME(clear_i); // Clear the content of the FIFO
    sc_in<bool>                             PN_NAME(write_i); // Write enable for the FIFO
    sc_in<bool>                             PN_NAME(read_i);  // Read enable for the FIFO
    sc_in<sc_uint<UART_FIFO_WIDTH>>         PN_NAME(data_i);  // Data to be written to the FIFO
    sc_out<sc_uint<UART_FIFO_WIDTH>>        PN_NAME(data_o);  // Data to be read from the FIFO
    sc_out<bool>                            PN_NAME(full_o);  // FIFO is full
    sc_out<bool>                            PN_NAME(empty_o); // FIFO is empty
    sc_out<sc_uint<UART_FIFO_SIZE_2E + 1>>  PN_NAME(usage_o); // Usage of the FIFO



    /* Constructor... */
    SC_CTOR(m_uart_fifo){
        SC_CTHREAD (proc_clk_module, clk.pos ());
        reset_signal_is(reset, true);
        SC_METHOD (proc_comb_module);
        sensitive << read_addr << write_addr << empty << empty << usage;
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

    sc_vector<sc_signal<sc_uint<UART_FIFO_WIDTH>>> PN_NAME_VEC(fifo_mem, 1 << UART_FIFO_SIZE_2E); // FIFO Data

    sc_signal<sc_uint<UART_FIFO_SIZE_2E + 1>> PN_NAME(read_addr); // read address
    sc_signal<sc_uint<UART_FIFO_SIZE_2E + 1>> PN_NAME(write_addr); // write address
    sc_signal<sc_uint<UART_FIFO_SIZE_2E + 1>> PN_NAME(usage); // usage of the FIFO
    sc_signal<bool> PN_NAME(full); // FIFO is full
    sc_signal<bool> PN_NAME(empty); // FIFO is empty

};
#endif // __UART_FIFO_H__