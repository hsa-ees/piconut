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


#include "baudgen.h"

/**
 * @brief this function is used to generate a tracefile
 * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
 * use PN_TRACE(tf, signalname) here to add whatever signal u need to trace.
 * @param tf this is the tracefile object
 * @param level is used as a selector when to enable a trace*/
void m_baudgen::pn_trace(sc_trace_file *tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, en_i);
    PN_TRACE(tf, clear_i);
    PN_TRACE(tf, div_i);
    PN_TRACE(tf, baudtick_o);

    // Internal Registers
    PN_TRACE(tf, baud_cnt);
}

// **************** Helpers *********************

// **************** m_baudgen ******************


void m_baudgen::proc_clk_module()
{
    // reset the counter register and the output
    baud_cnt = 0;
    baudtick_o = 0;

    while (true)
    {
        wait();

        // if the counter is beeing cleared reset the counter
        // this is a synchronous reset and prevents the counter from counting up
        if (clear_i.read() == 1)
        {
            baud_cnt = 0;
        }
        // if the module is not cleared and the enable signal is high
        // start counting up
        else if (en_i.read() == 1)
        {
            baud_cnt = baud_cnt.read() + 1;
        }

        // if the counter reaches the divider value set the baudtick signal
        // else leave it at 0
        if (baud_cnt.read() == div_i.read())
        {
            baud_cnt = 0;
            baudtick_o = 1;
        }
        else
        {
            baudtick_o = 0;
        }
    }
}