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

#include "dm_controller.h"
#include "dm_defs.h"


void m_dm_controller::pn_trace(sc_trace_file* tf, int level)
{
    // Ports...
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    // Control signals
    PN_TRACE(tf, c_haltreq_o);
    PN_TRACE(tf, c_hart_halted_o);
    PN_TRACE(tf, c_hart_running_o);
    PN_TRACE(tf, c_hart_run_acmds_o);
    PN_TRACE(tf, c_hart_resumereq_o);
    PN_TRACE(tf, c_hart_resumeack_o);
    PN_TRACE(tf, c_acmds_generate_o);
    PN_TRACE(tf, c_acmds_runreq_o);
    PN_TRACE(tf, c_acmds_busy_o);
    PN_TRACE(tf, c_acmds_error_not_supported_o);
    PN_TRACE(tf, c_acmds_increment_data1_o);

    // Status signals
    PN_TRACE(tf, s_haltreq_ack_i);
    PN_TRACE(tf, s_new_acmds_i);
    PN_TRACE(tf, s_abstractcs_cmderr_i);
    PN_TRACE(tf, s_dmcontrol_haltreq_i);
    PN_TRACE(tf, s_dmcontrol_resumereq_i);
    PN_TRACE(tf, s_hartstatus_halted_i);
    PN_TRACE(tf, s_hartstatus_running_i);
    PN_TRACE(tf, s_hartstatus_acmds_running_i);
    PN_TRACE(tf, s_command_reg_i);

    // Internal
    PN_TRACE(tf, dm_current_state);
}

void m_dm_controller::proc_cmb()
{
    c_haltreq_o = 0;
    c_hart_halted_o = 0;
    c_hart_running_o = 0;
    c_hart_run_acmds_o = 0;
    c_hart_resumereq_o = 0;
    c_hart_resumeack_o = 0;
    c_acmds_generate_o = 0;
    c_acmds_runreq_o = 0;
    c_acmds_busy_o = 0;
    c_acmds_error_not_supported_o = 0;
    c_acmds_increment_data1_o = 0;

    dm_next_state = dm_current_state.read();

    switch(dm_current_state.read())
    {
        case e_dm_state::DM_RUNNING:
            c_hart_running_o = 1;

            if(s_hartstatus_halted_i.read() == 1)
            {
                //
                // This case applies when ebreak was hit or single step completed.
                //
                dm_next_state = e_dm_state::DM_HALTED;
            }
            else if(s_dmcontrol_haltreq_i.read() == 1)
            {
                //
                // This case applies when async halt was triggered.
                //
                dm_next_state = e_dm_state::DM_HALTREQ;
            }
            break;

        case e_dm_state::DM_HALTREQ:
            c_hart_running_o = 1;
            c_haltreq_o = 1;

            if(s_haltreq_ack_i.read() == 1 ||
                s_hartstatus_halted_i.read() == 1)
            {
                dm_next_state = e_dm_state::DM_HALTED;
            }
            break;

        case e_dm_state::DM_HALTED:
            c_hart_halted_o = 1;

            if(s_dmcontrol_resumereq_i.read() == 1)
            {
                dm_next_state = e_dm_state::DM_RESUMEREQ;
            }
            else if(s_new_acmds_i.read() == 1 /* &&
                    s_abstractcs_cmderr_i.read() ==
                        e_abstractcs_cmderr::ABSTRACTCS_CMDERR_NONE*/
            )
            {
                dm_next_state = e_dm_state::DM_ACMDS_GENERATE;
            }
            break;

        case e_dm_state::DM_RESUMEREQ:
            c_hart_halted_o = 1;
            c_hart_resumereq_o = 1;

            if(s_hartstatus_running_i.read() == 1)
            {
                dm_next_state = e_dm_state::DM_RESUMEACK;
            }
            break;

        case e_dm_state::DM_RESUMEACK:
            c_hart_resumeack_o = 1;

            dm_next_state = e_dm_state::DM_RUNNING;
            break;

        case e_dm_state::DM_ACMDS_GENERATE:
            c_acmds_busy_o = 1;
            c_acmds_generate_o = 1;

            dm_next_state = e_dm_state::DM_ACMDS_GEN_ERROR_CHECK;
            break;

        case e_dm_state::DM_ACMDS_GEN_ERROR_CHECK:
            c_acmds_busy_o = 1;

            if(s_acmds_error_not_supported_i.read() == 1)
            {
                dm_next_state = e_dm_state::DM_ACMDS_GEN_ERROR;
            }
            else
            {
                dm_next_state = e_dm_state::DM_ACMDS_RUNREQ;
            }
            break;

        case e_dm_state::DM_ACMDS_GEN_ERROR:
            c_acmds_busy_o = 1;
            c_acmds_error_not_supported_o = 1;

            dm_next_state = e_dm_state::DM_HALTED;
            break;

        case e_dm_state::DM_ACMDS_RUNREQ:
            c_acmds_busy_o = 1;
            c_acmds_runreq_o = 1;

            if(s_hartstatus_acmds_running_i.read() == 1)
            {
                dm_next_state = e_dm_state::DM_ACMDS_RUNNING;
            }
            break;

        case e_dm_state::DM_ACMDS_RUNNING:
            c_acmds_busy_o = 1;

            if(s_hartstatus_acmds_running_i.read() == 0)
            {
                if(s_command_reg_i.read().range(COMMAND_CMDTYPE) == COMMAND_CMDTYPE_MEMORY &&
                    s_command_reg_i.read()[COMMAND_MEM_AAMPOSTINCREMENT] == 1)
                {
                    dm_next_state = e_dm_state::DM_ACMDS_INCREMENT_DATA1;
                }
                else
                {
                    dm_next_state = e_dm_state::DM_HALTED;
                }
            }
            break;

        case e_dm_state::DM_ACMDS_INCREMENT_DATA1:
            c_acmds_increment_data1_o = 1;

            dm_next_state = e_dm_state::DM_HALTED;
            break;

        default:
            PN_ERRORF(("Invalid state: 0x%02x", dm_current_state.read().to_int()));
            break;
    }
}

void m_dm_controller::proc_clk()
{
    while(true)
    {
        if(reset.read() == 1)
        {
            dm_current_state = e_dm_state::DM_RUNNING;
        }
        else
        {
            dm_current_state = dm_next_state.read();
        }

        wait();
    }
}
