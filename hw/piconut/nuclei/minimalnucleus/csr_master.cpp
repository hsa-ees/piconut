/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Johannes Hofmann <johannes.hofmann1@tha.de>
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

#include "csr_master.h"

void m_csr_master::Trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, csr_bus_rdata_in);
    PN_TRACE(tf, source_reg_in);
    PN_TRACE(tf, imm_en_in);
    PN_TRACE(tf, imm_in);
    PN_TRACE(tf, write_mode_in);
    PN_TRACE(tf, csr_bus_wdata_out);
}

void m_csr_master::proc_cmb()
{
    csr_bus_wdata_out = 0x0;

    sc_uint<32> source_reg_var = 0x0;

    if(imm_en_in == 1)
    {
        // Zero extend immediate value
        source_reg_var = (sc_uint<27>(0x0), imm_in.read());
    }
    else
    {
        source_reg_var = source_reg_in.read();
    }

    switch(write_mode_in.read())
    {
        case e_csr_write_mode::CSR_WRITE_ALL:
            csr_bus_wdata_out = source_reg_var;
            break;
        case e_csr_write_mode::CSR_WRITE_SET:
            csr_bus_wdata_out = source_reg_var | csr_bus_rdata_in.read();
            break;
        case e_csr_write_mode::CSR_WRITE_CLEAR:
            csr_bus_wdata_out = source_reg_var & csr_bus_rdata_in.read();
            break;
        default:
            PN_ERROR("csr master: Invalid write mode.");
            break;
    }
}
