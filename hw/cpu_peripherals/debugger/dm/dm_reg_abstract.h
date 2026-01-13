/**
 * @file dm.h
 * @brief This file contains the declaration of the m_dm_reg_abstract_commands module.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the declaration of the m_dm_abstract_commands module.

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

#ifndef __DM_REG_ABSTRACT_H__
#define __DM_REG_ABSTRACT_H__

#include "dm_defs.h"

#include <systemc.h>

#include <piconut.h>
#include <piconut-config.h>

class m_dm_controller;
class m_dm_registers;

SC_MODULE(m_dm_reg_abstract)
{
public:
    // --------------- System ---------------
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // --------------- Data flow signals ---------------
    sc_vector<sc_out<sc_uint<32>>> PN_NAME_VEC(abstract_regs_o, NUM_ABSTRACT_REGS);
    sc_in<sc_uint<32>> PN_NAME(command_reg_i);

    // --------------- Control signals ---------------
    sc_in<bool> PN_NAME(c_acmds_generate_i);

    // --------------- Status signals ---------------
    sc_out<bool> PN_NAME(s_error_not_supported);

    // Constructor...
    SC_CTOR(m_dm_reg_abstract)
    {
        SC_METHOD(proc_cmb_out);
        for(size_t i = 0; i < NUM_ABSTRACT_REGS; ++i)
        {
            sensitive << abstract_regs[i];
        }

        SC_CTHREAD(proc_clk, clk.pos());
    }

    // Functions...
    void pn_trace(sc_trace_file * tf, int levels = 1);

    // Submodules

    // Processes...
    void proc_clk();
    void proc_cmb_out();

private:
    void _generate_acmds();
    void _generate_acmds_cmdtype_reg();
    void _generate_acmds_cmdtype_reg_csr();
    void _generate_acmds_cmdtype_reg_gpr();
    void _generate_acmds_cmdtype_mem();

protected:
    // Registers ...
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(abstract_regs, NUM_ABSTRACT_REGS);

    // Control signals

    // Status signals

    // Submodules
};

#endif
