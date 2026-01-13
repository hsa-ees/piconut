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


#include "majority_filter.h"

/**
 * @brief this function is used to generate a tracefile
 * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
 * use PN_TRACE(tf, signalname) here to add whatever signal u need to trace.
 * @param tf this is the tracefile object
 * @param level is used as a selector when to enable a trace*/
void m_majority_filter::pn_trace(sc_trace_file *tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, filter_i);
    PN_TRACE(tf, capture_i);
    PN_TRACE(tf, clear_i);
    PN_TRACE(tf, filter_o);

    // Internal Registers
    PN_TRACE(tf, sample_cnt);
}

// **************** Helpers *********************

// **************** m_majority_filter ******************


void m_majority_filter::proc_clk_module()
{

    // reseting the values
    // the filter value is reset to high because the default value of uart is high
    sample_cnt = 0;
    filter_o = 1;

    while (true)
    {
        wait();

        // Reset the majority filter if clear_i is high
        if (clear_i.read() == 1)
        {
            sample_cnt = 0;
        }

        // If the majority threshold is reached, set the output to low and stop incrementing the sample counter
        if (sample_cnt.read() >= MAJORITY_FILTER_THRESHOLD)
        {
            filter_o = 0;
        }
        else
        {
            // Increment the sample counter if capturing is enabled and the input is low
            if (capture_i.read() == 1 && filter_i.read() == 0)
            {
                sample_cnt = sample_cnt.read() + 1;
            }

            filter_o = 1;
        }
    }
}