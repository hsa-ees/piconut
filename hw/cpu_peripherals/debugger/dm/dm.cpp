/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the definition of the m_dm module.

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

#include "dm.h"
#include "dm_controller.h"
#include "dm_registers.h"
#include "dm_reg_abstract.h"


void m_dm::pn_trace(sc_trace_file* tf, int level)
{
    // Ports...
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    PN_TRACE(tf, dmi_adr_i);
    PN_TRACE(tf, dmi_dat_i);
    PN_TRACE(tf, dmi_dat_o);
    PN_TRACE(tf, dmi_re_i);
    PN_TRACE(tf, dmi_we_i);

    PN_TRACE(tf, wb_ack_o);
    PN_TRACE(tf, wb_adr_i);
    PN_TRACE(tf, wb_cyc_i);
    PN_TRACE(tf, wb_dat_i);
    PN_TRACE(tf, wb_dat_o);
    PN_TRACE(tf, wb_sel_i);
    PN_TRACE(tf, wb_we_i);
    PN_TRACE(tf, wb_err_o);

    PN_TRACE(tf, debug_haltrequest_o);
    PN_TRACE(tf, debug_haltrequest_ack_i);

    // Internal
    PN_TRACE(tf, c_hart_halted);
    PN_TRACE(tf, c_hart_running);
    PN_TRACE(tf, c_hart_run_acmds);
    PN_TRACE(tf, c_hart_resumereq);
    PN_TRACE(tf, c_hart_resumeack);
    PN_TRACE(tf, c_acmds_generate);
    PN_TRACE(tf, c_acmds_runreq);

    PN_TRACE(tf, s_abstractcs_cmderr);
    PN_TRACE(tf, s_dmcontrol_haltreq);
    PN_TRACE(tf, s_dmcontrol_resumereq);
    PN_TRACE(tf, s_hartstatus_halted);
    PN_TRACE(tf, s_hartstatus_running);
    PN_TRACE(tf, s_hartstatus_acmds_running);
    PN_TRACE(tf, s_new_acmds);

    // Submodules
    dm_controller->pn_trace(tf, level);
    dm_registers->pn_trace(tf, level);
    dm_reg_abstract->pn_trace(tf, level);
}

void m_dm::init_submodules()
{

    //
    // dm_controller
    //
    dm_controller = sc_new<m_dm_controller>("dm_controller");
    dm_controller->clk(clk);
    dm_controller->reset(reset);

    dm_controller->c_haltreq_o(debug_haltrequest_o);
    dm_controller->c_hart_halted_o(c_hart_halted);
    dm_controller->c_hart_running_o(c_hart_running);
    dm_controller->c_hart_run_acmds_o(c_hart_run_acmds);
    dm_controller->c_hart_resumereq_o(c_hart_resumereq);
    dm_controller->c_hart_resumeack_o(c_hart_resumeack);
    dm_controller->c_acmds_generate_o(c_acmds_generate);
    dm_controller->c_acmds_runreq_o(c_acmds_runreq);
    dm_controller->c_acmds_busy_o(c_acmds_busy);
    dm_controller->c_acmds_error_not_supported_o(c_acmds_error_not_supported);
    dm_controller->c_acmds_increment_data1_o(c_acmds_increment_data1);

    dm_controller->s_haltreq_ack_i(debug_haltrequest_ack_i);
    dm_controller->s_new_acmds_i(s_new_acmds);
    dm_controller->s_abstractcs_cmderr_i(s_abstractcs_cmderr);
    dm_controller->s_dmcontrol_haltreq_i(s_dmcontrol_haltreq);
    dm_controller->s_dmcontrol_resumereq_i(s_dmcontrol_resumereq);
    dm_controller->s_hartstatus_halted_i(s_hartstatus_halted);
    dm_controller->s_hartstatus_running_i(s_hartstatus_running);
    dm_controller->s_hartstatus_acmds_running_i(s_hartstatus_acmds_running);
    dm_controller->s_acmds_error_not_supported_i(s_acmds_error_not_supported);
    dm_controller->s_command_reg_i(command_reg);

    //
    // dm_registers
    //
    dm_registers = sc_new<m_dm_registers>("dm_registers");
    dm_registers->clk(clk);
    dm_registers->reset(reset);

    // DMI
    dm_registers->dmi_adr_i(dmi_adr_i);
    dm_registers->dmi_dat_i(dmi_dat_i);
    dm_registers->dmi_dat_o(dmi_dat_o);
    dm_registers->dmi_re_i(dmi_re_i);
    dm_registers->dmi_we_i(dmi_we_i);

    // Wishbone
    dm_registers->wb_stb_i(wb_stb_i);
    dm_registers->wb_cyc_i(wb_cyc_i);
    dm_registers->wb_we_i(wb_we_i);
    dm_registers->wb_cti_i(wb_cti_i);
    dm_registers->wb_bte_i(wb_bte_i);
    dm_registers->wb_sel_i(wb_sel_i);
    dm_registers->wb_ack_o(wb_ack_o);
    dm_registers->wb_err_o(wb_err_o);
    dm_registers->wb_rty_o(wb_rty_o);
    dm_registers->wb_adr_i(wb_adr_i);
    dm_registers->wb_dat_i(wb_dat_i);
    dm_registers->wb_dat_o(wb_dat_o);

    // Data flow signals
    dm_registers->abstract_regs_i(abstract_regs);
    dm_registers->command_reg_o(command_reg);

    // Control signals
    dm_registers->c_hart_halted_i(c_hart_halted);
    dm_registers->c_hart_running_i(c_hart_running);
    dm_registers->c_hart_run_acmds_i(c_hart_run_acmds);
    dm_registers->c_hart_resumereq_i(c_hart_resumereq);
    dm_registers->c_hart_resumeack_i(c_hart_resumeack);
    dm_registers->c_acmds_generate_i(c_acmds_generate);
    dm_registers->c_acmds_runreq_i(c_acmds_runreq);
    dm_registers->c_acmds_busy_i(c_acmds_busy);
    dm_registers->c_acmds_error_not_supported_i(c_acmds_error_not_supported);
    dm_registers->c_acmds_increment_data1_i(c_acmds_increment_data1);

    // Status signals
    dm_registers->s_new_acmds_o(s_new_acmds);
    dm_registers->s_abstractcs_cmderr_o(s_abstractcs_cmderr);
    dm_registers->s_dmcontrol_haltreq_o(s_dmcontrol_haltreq);
    dm_registers->s_dmcontrol_resumereq_o(s_dmcontrol_resumereq);
    dm_registers->s_hartstatus_halted_o(s_hartstatus_halted);
    dm_registers->s_hartstatus_running_o(s_hartstatus_running);
    dm_registers->s_hartstatus_acmds_running_o(s_hartstatus_acmds_running);

    //
    // dm_reg_abstract
    //
    dm_reg_abstract = sc_new<m_dm_reg_abstract>("dm_reg_abstract");
    dm_reg_abstract->clk(clk);
    dm_reg_abstract->reset(reset);

    // Data flow signals
    dm_reg_abstract->abstract_regs_o(abstract_regs);
    dm_reg_abstract->command_reg_i(command_reg);

    // Control signals
    dm_reg_abstract->c_acmds_generate_i(c_acmds_generate);

    // Status signals
    dm_reg_abstract->s_error_not_supported(s_acmds_error_not_supported);
}
