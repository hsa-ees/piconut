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

#include "uart_fifo.h"

/**
 * @brief this function is used to generate a tracefile
 * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
 * use PN_TRACE(tf, signalname) here to add whatever signal u need to trace.
 * @param tf this is the tracefile object
 * @param level is used as a selector when to enable a trace*/
void m_uart_fifo::Trace(sc_trace_file *tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, clear_i);
    PN_TRACE(tf, write_i);
    PN_TRACE(tf, read_i);
    PN_TRACE(tf, data_i);
    PN_TRACE(tf, data_o);
    PN_TRACE(tf, full_o);
    PN_TRACE(tf, empty_o);
    PN_TRACE(tf, usage_o);

    PN_TRACE_BUS(tf, fifo_mem, 1 << UART_FIFO_SIZE_2E);

    PN_TRACE(tf, read_addr);
    PN_TRACE(tf, write_addr);
    PN_TRACE(tf, full);
    PN_TRACE(tf, empty);
    PN_TRACE(tf, usage);

}

// **************** Helpers *********************

// **************** m_majority_filter ******************

void m_uart_fifo::proc_comb_module()
{

    // creating internal variables
    sc_uint<UART_FIFO_SIZE_2E + 1> read_addr_var = read_addr.read();
    sc_uint<UART_FIFO_SIZE_2E + 1> write_addr_var = write_addr.read();

    // if difference between read and write address is the greatest possible set the full flag
    // otherwise clear the full flag
    if (read_addr_var(UART_FIFO_SIZE_2E - 1, 0) == write_addr_var(UART_FIFO_SIZE_2E - 1, 0) &&
        read_addr_var[UART_FIFO_SIZE_2E] != write_addr_var[UART_FIFO_SIZE_2E])
    {
        full = 1;
        full_o = 1;
    }
    else
    {
        full = 0;
        full_o = 0;
    }

    // output empty and full from the module
    empty_o = empty.read();
    usage_o = usage.read();
}

void m_uart_fifo::proc_clk_module()
{

    // internal variables
    sc_uint<UART_FIFO_SIZE_2E + 1> read_addr_var;
    sc_uint<UART_FIFO_SIZE_2E + 1> write_addr_var;

    // Reset if reset signal is high
    read_addr = 0;
    write_addr = 0;
    usage = 0;
    empty = 0;

    while (true)
    {
        wait();

        // read addresses into internal variables
        read_addr_var = read_addr.read();
        write_addr_var = write_addr.read();

        // write new data to fifo if write enabled and fifo is not full
        if (write_i.read() && !full_o.read())
        {
            fifo_mem[write_addr_var(UART_FIFO_SIZE_2E - 1, 0)] = data_i.read();
        }

        // read the data from the fifo's read address
        data_o = fifo_mem[read_addr_var(UART_FIFO_SIZE_2E - 1, 0)];


        // set usage according to the content of the FIFO but only if fifo is not
        // beeing cleared
        if (clear_i.read())
        {
            read_addr_var = 0;
            write_addr_var = 0;
            usage = 0;
        }
        else
        {

            // increment usage if the fifo is written to and not full
            if (!read_i.read() && write_i.read() && !full.read())
            {
                usage = usage.read() + 1;
            }

            // increment read address if read is enabled and fifo is not empty
            if (read_i.read() && !write_i.read() && !empty.read())
            {
                usage = usage.read() - 1;
            }

            // increment write address if write is enabled and fifo is not full
            if (write_i.read() && !read_i.read() && !full.read())
            {
                write_addr_var = write_addr_var + 1;
            }

            // increment read address if read is enabled and fifo is not empty
            if (read_i.read() && !write_i.read() && !empty.read())
            {
                read_addr_var = read_addr_var + 1;
            }
        }

        // if read and write address are the same the fifo is empty => set the empty flag
        if (read_addr_var == write_addr_var)
        {
            empty = 1;
        }
        else
        {
            empty = 0;
        }


        // Writeback of the addresses
        read_addr = read_addr_var;
        write_addr = write_addr_var;
    }
}