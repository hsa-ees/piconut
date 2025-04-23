/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
                     2024 Marco Milenkovic <Marco.Milenkovic@tha.de>
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

#include "piconut.h"

void m_piconut::Trace(sc_trace_file* tf, int level)
{

    // calling trace of submodules
    if(level >= 2)
    {

#ifdef __SIMONLYMEMU__
        simmemu->Trace(tf, level);
#endif

#ifdef __MINIMALNUCLEUS__
        nucleus->Trace(tf, level);
#endif

#ifdef __HW_MEMU__
        hw_memu->Trace(tf, level);
#endif
    }
    // Internal traces
}

void m_piconut::init_submodules()
{

#ifdef __SIMONLYMEMU__
    simmemu->clk(clk);
    simmemu->reset(reset);

    simmemu->stb_iport(stb_iport);
    simmemu->adr_iport(adr_iport);
    simmemu->bsel_iport(bsel_iport);
    simmemu->rdata_iport(rdata_iport);
    simmemu->ack_iport(ack_iport);

    simmemu->stb_dport(stb_dport);
    simmemu->we_dport(we_dport);
    simmemu->adr_dport(adr_dport);
    simmemu->wdata_dport(wdata_dport);
    simmemu->bsel_dport(bsel_dport);
    simmemu->rdata_dport(rdata_dport);
    simmemu->ack_dport(ack_dport);
#endif

#ifdef __MINIMALNUCLEUS__
    nucleus->clk(clk);
    nucleus->reset(reset);
    nucleus->debug_haltrequest_in(debug_haltrequest_in);
    nucleus->debug_haltrequest_ack_out(debug_haltrequest_ack_out);

    nucleus->iport_stb_out(stb_iport);
    nucleus->iport_adr_out(adr_iport);
    nucleus->iport_bsel_out(bsel_iport);
    nucleus->iport_rdata_in(rdata_iport);
    nucleus->iport_ack_in(ack_iport);

    nucleus->dport_stb_out(stb_dport);
    nucleus->dport_we_out(we_dport);
    nucleus->dport_adr_out(adr_dport);
    nucleus->dport_wdata_out(wdata_dport);
    nucleus->dport_bsel_out(bsel_dport);
    nucleus->dport_rdata_in(rdata_dport);
    nucleus->dport_ack_in(ack_dport);

#endif

#ifdef __HW_MEMU__

    hw_memu->clk(clk);
    hw_memu->reset(reset);
    // ------------ IPort Signals ------------
    hw_memu->ip_stb(stb_iport);
    hw_memu->ip_adr(adr_iport);
    hw_memu->ip_bsel(bsel_iport);
    hw_memu->ip_rdata(rdata_iport);
    hw_memu->ip_ack(ack_iport);

    // ------------ DPort Signals ------------
    hw_memu->dp_stb(stb_dport);
    hw_memu->dp_we(we_dport);
    hw_memu->dp_adr(adr_dport);
    hw_memu->dp_wdata(wdata_dport);
    hw_memu->dp_bsel(bsel_dport);
    hw_memu->dp_rdata(rdata_dport);
    hw_memu->dp_ack(ack_dport);

    // ------------ Wishbone Interface ------------
    hw_memu->wb_ack_i(wb_ack_i);
    hw_memu->wb_dat_i(wb_dat_i);

    hw_memu->wb_adr_o(wb_adr_o);
    hw_memu->wb_dat_o(wb_dat_o);
    hw_memu->wb_we_o(wb_we_o);
    hw_memu->wb_stb_o(wb_stb_o);
    hw_memu->wb_cyc_o(wb_cyc_o);
    hw_memu->wb_sel_o(wb_sel_o);

#endif
}

#ifdef __MINIMALNUCLEUS__
bool m_piconut::state_is_not_halt()
{
    if(nucleus->nucleus_get_controller_state() == STATE_HALT)
    {
        return false;
    }
    return true;
}
#endif
