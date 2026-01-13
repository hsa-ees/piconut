/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the definition of the m_debugger module.

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

#include "debugger.h"

#include "dtm.h"
#include "dm.h"


void m_debugger::pn_trace(sc_trace_file* tf, int level)
{
    // Ports...
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    PN_TRACE(tf, tck_i);
    PN_TRACE(tf, tms_i);
    PN_TRACE(tf, tdi_i);
    PN_TRACE(tf, tdo_o);

    // Submodules
    dtm->pn_trace(tf, level);
    dm->pn_trace(tf, level);
}

void m_debugger::init_submodules()
{
    dtm = sc_new<m_dtm>("dtm");
    dtm->clk(clk);
    dtm->reset(reset);
    dtm->tck_i(tck_i);
    dtm->tms_i(tms_i);
    dtm->tdi_i(tdi_i);
    dtm->trst_n_i(trst_n_i);
    dtm->tdo_o(tdo_o);
    dtm->dmi_adr_o(dmi_adr);
    dtm->dmi_dat_o(dmi_dat_w);
    dtm->dmi_dat_i(dmi_dat_r);
    dtm->dmi_re_o(dmi_re);
    dtm->dmi_we_o(dmi_we);

    dm = sc_new<m_dm>("dm");
    dm->clk(clk);
    dm->reset(reset);
    dm->dmi_adr_i(dmi_adr);
    dm->dmi_dat_i(dmi_dat_w);
    dm->dmi_dat_o(dmi_dat_r);
    dm->dmi_re_i(dmi_re);
    dm->dmi_we_i(dmi_we);

    dm->wb_stb_i(wb_stb_i);
    dm->wb_cyc_i(wb_cyc_i);
    dm->wb_we_i(wb_we_i);
    dm->wb_cti_i(wb_cti_i);
    dm->wb_bte_i(wb_bte_i);
    dm->wb_sel_i(wb_sel_i);
    dm->wb_ack_o(wb_ack_o);
    dm->wb_err_o(wb_err_o);
    dm->wb_rty_o(wb_rty_o);
    dm->wb_adr_i(wb_adr_i);
    dm->wb_dat_i(wb_dat_i);
    dm->wb_dat_o(wb_dat_o);

    dm->debug_haltrequest_o(debug_haltrequest_o);
    dm->debug_haltrequest_ack_i(debug_haltrequest_ack_i);
}
