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

#include "regfile.h"


void m_regfile::Trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, en_load_in);
    PN_TRACE(tf, data_in);
    PN_TRACE(tf, select_in);
    PN_TRACE(tf, rs1_select_in);
    PN_TRACE(tf, rs2_select_in);
    PN_TRACE(tf, rs1_out);
    PN_TRACE(tf, rs2_out);
    PN_TRACE_BUS(tf, regfile, CFG_REGFILE_SIZE);
}

void m_regfile::proc_cmb_regfile()
{
    // Set permanent output by regfile indices selected via rs1_select_in and rs2_select_in
    rs1_out = regfile[rs1_select_in.read()].read();
    rs2_out = regfile[rs2_select_in.read()].read();
}

void m_regfile::proc_clk_regfile()
{
    // Reset section
    // Set all registers to 0 when reset is high.
    for(uint32_t i = 0; i < CFG_REGFILE_SIZE; i++)
    {
        regfile[i] = 0x0;
    }

    while(true)
    {
        wait();

        if(en_load_in.read())
        {
            /* When the control signal c_reg_ld_en is high, load the value from the input port into the register
               selected by select_in. */
            regfile[select_in.read()] = data_in.read();
        }

        // x0 is always 0
        regfile[0] = 0;
    }
}