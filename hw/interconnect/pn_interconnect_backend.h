/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This file contains the pn_interconnect_backend module.

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

#ifndef __PN_INTERCONNECT_BACKEND_H__
#define __PN_INTERCONNECT_BACKEND_H__

#include <systemc.h>
#include <piconut.h>

#include <vector>

SC_MODULE(m_pn_interconnect_backend)
{
public:
    SC_HAS_PROCESS(m_pn_interconnect_backend);
    m_pn_interconnect_backend(
        sc_module_name name,
        std::vector<pn_module_if*> & modules);

    void pn_trace(sc_trace_file * tf, int level = 1);

    void proc_cmb();

protected:
    const uint64_t num_slaves_wishbone;
    const std::vector<pn_wb_adr_t> base_addresses_wishbone;
    const std::vector<pn_wb_adr_t> sizes_wishbone;

    sc_signal<bool> PN_NAME(wb_stb);
    sc_signal<bool> PN_NAME(wb_cyc);
    sc_signal<bool> PN_NAME(wb_we);
    sc_signal<pn_wb_sel_t> PN_NAME(wb_sel);
    sc_vector<sc_signal<bool>> wb_ack;
    sc_signal<pn_wb_adr_t> PN_NAME(wb_adr);
    sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_o);
    sc_vector<sc_signal<pn_wb_dat_t>> wb_dat_i;
    sc_vector<sc_signal<bool>> wb_rty;
    sc_vector<sc_signal<bool>> wb_err;

    sc_signal<bool> PN_NAME(wb_ack_master);
    sc_signal<pn_wb_dat_t> PN_NAME(wb_dat_i_master);
    sc_signal<bool> PN_NAME(wb_rty_master);
    sc_signal<bool> PN_NAME(wb_err_master);
};

#endif
