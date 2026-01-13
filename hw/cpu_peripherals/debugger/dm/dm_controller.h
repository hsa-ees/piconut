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

#ifndef __DM_CONTROLLER_H__
#define __DM_CONTROLLER_H__

#include <systemc.h>

#include <piconut.h>

SC_MODULE(m_dm_controller)
{
public:
    // --------------- System ---------------
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // --------------- Control signals ---------------
    sc_out<bool> PN_NAME(c_haltreq_o);

    sc_out<bool> PN_NAME(c_hart_halted_o);
    sc_out<bool> PN_NAME(c_hart_running_o);
    sc_out<bool> PN_NAME(c_hart_run_acmds_o);

    sc_out<bool> PN_NAME(c_hart_resumereq_o);
    sc_out<bool> PN_NAME(c_hart_resumeack_o);
    sc_out<bool> PN_NAME(c_acmds_generate_o);
    sc_out<bool> PN_NAME(c_acmds_runreq_o);
    sc_out<bool> PN_NAME(c_acmds_busy_o);
    sc_out<bool> PN_NAME(c_acmds_error_not_supported_o);
    sc_out<bool> PN_NAME(c_acmds_increment_data1_o);

    // --------------- Status signals ---------------
    sc_in<bool> PN_NAME(s_haltreq_ack_i);
    sc_in<bool> PN_NAME(s_new_acmds_i);
    sc_in<sc_uint<3>> PN_NAME(s_abstractcs_cmderr_i);
    sc_in<bool> PN_NAME(s_dmcontrol_haltreq_i);
    sc_in<bool> PN_NAME(s_dmcontrol_resumereq_i);
    sc_in<bool> PN_NAME(s_hartstatus_halted_i);
    sc_in<bool> PN_NAME(s_hartstatus_running_i);
    sc_in<bool> PN_NAME(s_hartstatus_acmds_running_i);
    sc_in<sc_uint<32>> PN_NAME(s_command_reg_i);

    sc_in<bool> PN_NAME(s_acmds_error_not_supported_i);

    // Constructor...
    SC_CTOR(m_dm_controller)
    {
        SC_METHOD(proc_cmb);
        sensitive << dm_current_state
                  << s_haltreq_ack_i
                  << s_new_acmds_i
                  << s_abstractcs_cmderr_i
                  << s_dmcontrol_haltreq_i
                  << s_dmcontrol_resumereq_i
                  << s_hartstatus_halted_i
                  << s_hartstatus_running_i
                  << s_hartstatus_acmds_running_i
                  << s_acmds_error_not_supported_i
                  << s_command_reg_i;

        SC_CTHREAD(proc_clk, clk.pos());
    }

    // Functions...
    void pn_trace(sc_trace_file * tf, int levels = 1);

    // Processes...

    void proc_cmb();
    void proc_clk();

protected:
    enum e_dm_state
    {
        DM_RUNNING = 0,
        DM_HALTREQ,
        DM_HALTED,
        DM_RESUMEREQ,
        DM_RESUMEACK,
        DM_ACMDS_GENERATE,
        DM_ACMDS_GEN_ERROR_CHECK,
        DM_ACMDS_GEN_ERROR,
        DM_ACMDS_RUNREQ,
        DM_ACMDS_RUNNING,
        DM_ACMDS_INCREMENT_DATA1,
    };

protected:
    sc_signal<sc_uint<4>> PN_NAME(dm_current_state);
    sc_signal<sc_uint<4>> PN_NAME(dm_next_state);

    // Registers ...

    // Control signals

    // Status signals
};

#endif
