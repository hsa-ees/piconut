/**
 * @file dm.h
 * @brief This file contains the declaration of the m_dm module.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the declaration of the m_dm module.

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

/**
 * @addtogroup m_dm
 * TODO
 */

#ifndef __DM_H__
#define __DM_H__

#include <systemc.h>

#include <piconut.h>

#include "dm_defs.h"

SC_MODULE(m_dm)
{
public:
    // --------------- System ---------------
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // --------------- Dmi ---------------
    sc_in<sc_uint<PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH>> PN_NAME(dmi_adr_i);
    sc_in<sc_uint<32>> PN_NAME(dmi_dat_i);
    sc_out<sc_uint<32>> PN_NAME(dmi_dat_o);
    sc_in<bool> PN_NAME(dmi_re_i);
    sc_in<bool> PN_NAME(dmi_we_i);

    // --------------- Wishbone ---------------
    sc_in<bool> PN_NAME(wb_stb_i);            // strobe input
    sc_in<bool> PN_NAME(wb_cyc_i);            // cycle valid input
    sc_in<bool> PN_NAME(wb_we_i);             // indicates write transfer
    sc_in<sc_uint<3>> PN_NAME(wb_cti_i);      // cycle type identifier (optional, for registered feedback)
    sc_in<sc_uint<2>> PN_NAME(wb_bte_i);      // burst type extension (optional, for registered feedback)
    sc_in<sc_uint<32 / 8>> PN_NAME(wb_sel_i); // byte select inputs
    sc_out<bool> PN_NAME(wb_ack_o);           // normal termination
    sc_out<bool> PN_NAME(wb_err_o);           // termination w/ error (optional)
    sc_out<bool> PN_NAME(wb_rty_o);           // termination w/ retry (optional)
    sc_in<sc_uint<32>> PN_NAME(wb_adr_i);     // address
    sc_in<sc_uint<32>> PN_NAME(wb_dat_i);     // data in
    sc_out<sc_uint<32>> PN_NAME(wb_dat_o);    // data out

    // --------------- Debug Interrupt ---------------
    sc_out<bool> PN_NAME(debug_haltrequest_o);
    sc_in<bool> PN_NAME(debug_haltrequest_ack_i);

#if !PN_PRESYNTHESIZED_H_ONLY(DM)

    // Constructor...
    SC_CTOR(m_dm)
    {
        init_submodules();
    }

    // Functions...
    void pn_trace(sc_trace_file * tf, int levels = 1);

    // Submodules
    void init_submodules();

    // Processes...

#else // !PN_PRESYNTHESIZED_H_ONLY(NUCLEUS_REF)

    SC_CTOR(m_dm) {}
    void pn_trace(sc_trace_file * tf, int level = 1) {}

#endif

protected:
#if !PN_PRESYNTHESIZED_H_ONLY(DM)
    // Registers ...

    // Data flow signals
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(abstract_regs, NUM_ABSTRACT_REGS);
    sc_signal<sc_uint<32>> PN_NAME(command_reg);

    // Control signals
    sc_signal<bool> PN_NAME(c_hart_halted);
    sc_signal<bool> PN_NAME(c_hart_running);
    sc_signal<bool> PN_NAME(c_hart_run_acmds);

    sc_signal<bool> PN_NAME(c_hart_resumereq);
    sc_signal<bool> PN_NAME(c_hart_resumeack);
    sc_signal<bool> PN_NAME(c_acmds_generate);
    sc_signal<bool> PN_NAME(c_acmds_runreq);
    sc_signal<bool> PN_NAME(c_acmds_busy);
    sc_signal<bool> PN_NAME(c_acmds_error_not_supported);
    sc_signal<bool> PN_NAME(c_acmds_increment_data1);

    // Status signals
    sc_signal<sc_uint<3>> PN_NAME(s_abstractcs_cmderr);
    sc_signal<bool> PN_NAME(s_dmcontrol_haltreq);
    sc_signal<bool> PN_NAME(s_dmcontrol_resumereq);
    sc_signal<bool> PN_NAME(s_hartstatus_halted);
    sc_signal<bool> PN_NAME(s_hartstatus_running);
    sc_signal<bool> PN_NAME(s_hartstatus_acmds_running);

    sc_signal<bool> PN_NAME(s_new_acmds);
    sc_signal<bool> PN_NAME(s_acmds_error_not_supported);

    // Submodules
    class m_dm_controller* dm_controller;
    class m_dm_registers* dm_registers;
    class m_dm_reg_abstract* dm_reg_abstract;

#else
    PN_PRESYNTHESIZED;
#endif
};

#endif
