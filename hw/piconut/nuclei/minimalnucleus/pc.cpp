/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
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

#include "pc.h"



void m_pc::Trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, inc_in);
    PN_TRACE(tf, en_load_in);
    PN_TRACE(tf, pc_in);
    PN_TRACE(tf, pc_out);
    PN_TRACE(tf, debug_level_enter_in);
    PN_TRACE(tf, debug_level_leave_in);
    PN_TRACE(tf, csr_dpc_in);
    PN_TRACE(tf, pc_reg);
}

void m_pc::proc_cmb_pc()
{

    // Read register and set lowest two bits to 0
    sc_uint<32> pc_temp = 0x0;

    pc_temp.range(31, 2) = pc_reg.read();
    pc_temp.range(1, 0) = 0x0;

    // Write value to output port.
    pc_out = pc_temp;
}

void m_pc::proc_clk_pc()
{
    // Reset section
    // Set PC to start address with reset
    // Shift left by two because internal register is 30 bits wide
    // Upper 30 bits are stored in internal registers.
    // Lowest two bits are static '00' at output.
    pc_reg = CFG_START_ADDRESS >> 2;

    while(true)
    {
        wait();

        // If debug level is left, load value of the csr dpc into internal register.
        if(debug_level_leave_in.read())
        {
            pc_reg = csr_dpc_in.read().range(31, 2);
        }
        // If debug level is entered, load debug rom start address
        // into internal register.
        else if(debug_level_enter_in.read())
        {
            pc_reg = CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS >> 2;
        }
        // If read signal is high, load input value into internal register.
        else if(en_load_in.read())
        {
            pc_reg = pc_in.read().range(31, 2);
        }
        // If inc signal is high, increment pc_reg by 4
        else if(inc_in.read())
        {
            // +0x1 because of 30 bit internal size
            pc_reg = pc_reg.read() + 0x1;
        }
    }
}