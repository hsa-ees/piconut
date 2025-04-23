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


#include "clock_div.h"

/**
 * @brief this function is used to generate a tracefile
 * you may use this function to add Signals and Ports or even Members of Structs to your .vcd tracefile
 * use PN_TRACE(tf, signalname) here to add whatever signal u need to trace.
 * @param tf this is the tracefile object
 * @param level is used as a selector when to enable a trace*/
void m_clock_div::Trace(sc_trace_file *tf, int level){

    if (level >= 1){
    /* Ports ... */
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, div_clk);
    /* Registers... */
    PN_TRACE(tf, counter_reg);
    }
}


// **************** Helpers *********************

/* This is an example Helper function
it checks if the address is smaller than the CFG_NUT_MEM_SIZE */
// static inline bool AdrIsCacheable(sc_uint<32> adr) { return ((adr ^ CFG_NUT_RESET_ADDR) < CFG_NUT_MEM_SIZE); }

// **************** m_clock_div ******************

void m_clock_div::proc_clk_module()
{
    // put all Declarations here
    // Defaults
    // Reset Area...
    counter_reg = 1000000;

    while (true) // necessary for SC_CTHREAD/SC_THREAD
    {
        wait();

        if (counter_reg.read() == 0){
            counter_reg = 1000000;
        }else{
            counter_reg = counter_reg.read() - 1;
        }
        div_clk = div_clk_next.read();
    }
}


void m_clock_div::proc_comb_module(){

    // Default Values
    div_clk_next = 0;

    if(counter_reg.read() == 0){
        div_clk_next = 1;
    }
}