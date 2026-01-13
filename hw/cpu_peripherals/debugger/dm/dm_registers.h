/**
 * @file dm_registers.h
 * @brief This file contains the declaration of the m_dm_registers module.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the declaration of the m_dm_registers module.

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

#ifndef __DM_REGISTERS_H__
#define __DM_REGISTERS_H__

#include <systemc.h>
#include <piconut.h>

#include "dm_defs.h"

#ifndef WB_DAT_WIDTH
#define WB_DAT_WIDTH 32
#endif

#ifndef WB_ADR_WIDTH
#define WB_ADR_WIDTH 32
#endif

SC_MODULE(m_dm_registers)
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
    sc_in<bool> PN_NAME(wb_stb_i);                      // strobe input
    sc_in<bool> PN_NAME(wb_cyc_i);                      // cycle valid input
    sc_in<bool> PN_NAME(wb_we_i);                       // indicates write transfer
    sc_in<sc_uint<3>> PN_NAME(wb_cti_i);                // cycle type identifier (optional, for registered feedback)
    sc_in<sc_uint<2>> PN_NAME(wb_bte_i);                // burst type extension (optional, for registered feedback)
    sc_in<sc_uint<WB_DAT_WIDTH / 8>> PN_NAME(wb_sel_i); // byte select inputs
    sc_out<bool> PN_NAME(wb_ack_o);                     // normal termination
    sc_out<bool> PN_NAME(wb_err_o);                     // termination w/ error (optional)
    sc_out<bool> PN_NAME(wb_rty_o);                     // termination w/ retry (optional)
    sc_in<sc_uint<WB_ADR_WIDTH>> PN_NAME(wb_adr_i);     // address
    sc_in<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_i);     // data in
    sc_out<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_o);    // data out

    // --------------- Data flow signals ---------------
    sc_vector<sc_in<sc_uint<32>>> PN_NAME_VEC(abstract_regs_i, NUM_ABSTRACT_REGS);
    sc_out<sc_uint<32>> PN_NAME(command_reg_o);

    // --------------- Control signals ---------------
    sc_in<bool> PN_NAME(c_hart_halted_i);
    sc_in<bool> PN_NAME(c_hart_running_i);
    sc_in<bool> PN_NAME(c_hart_run_acmds_i);

    sc_in<bool> PN_NAME(c_hart_resumereq_i);
    sc_in<bool> PN_NAME(c_hart_resumeack_i);
    sc_in<bool> PN_NAME(c_acmds_generate_i);
    sc_in<bool> PN_NAME(c_acmds_runreq_i);
    sc_in<bool> PN_NAME(c_acmds_busy_i);
    sc_in<bool> PN_NAME(c_acmds_error_not_supported_i);
    sc_in<bool> PN_NAME(c_acmds_increment_data1_i);

    // --------------- Status signals ---------------
    sc_out<bool> PN_NAME(s_new_acmds_o);
    sc_out<sc_uint<3>> PN_NAME(s_abstractcs_cmderr_o);
    sc_out<bool> PN_NAME(s_dmcontrol_haltreq_o);
    sc_out<bool> PN_NAME(s_dmcontrol_resumereq_o);
    sc_out<bool> PN_NAME(s_hartstatus_halted_o);
    sc_out<bool> PN_NAME(s_hartstatus_running_o);
    sc_out<bool> PN_NAME(s_hartstatus_acmds_running_o);

    // Constructor...
    SC_CTOR(m_dm_registers)
    {
        SC_METHOD(proc_cmb_read_dmi);
        sensitive << dmi_re_i
                  << dmi_adr_i
                  << dmcontrol_reg
                  << dmstatus_reg
                  << command_reg
                  << abstractcs_reg
                  << abstractauto_reg;
        for(size_t i = 0; i < NUM_DATA_REGS; ++i)
        {
            sensitive << data_regs[i];
        }
        for(size_t i = 0; i < NUM_PROGBUF_REGS; ++i)
        {
            sensitive << progbuf_regs[i];
        }

        SC_METHOD(proc_cmb_wb);
        sensitive << wb_stb_i
                  << wb_cyc_i
                  << wb_we_i
                  << wb_adr_i
                  << wb_dat_i
                  << wb_sel_i
                  << wb_current_state
                  << hartcontrol_reg
                  << hartstatus_reg;
        for(size_t i = 0; i < NUM_DATA_REGS; ++i)
        {
            sensitive << data_regs[i];
        }
        for(size_t i = 0; i < NUM_PROGBUF_REGS; ++i)
        {
            sensitive << progbuf_regs[i];
        }
        for(size_t i = 0; i < NUM_ABSTRACT_REGS; ++i)
        {
            sensitive << abstract_regs_i[i];
        }

        SC_METHOD(proc_cmb_dmcontrol_out);
        sensitive << dmcontrol_reg
                  << c_hart_halted_i
                  << c_acmds_generate_i;

        SC_METHOD(proc_cmb_hartstatus_out);
        sensitive << hartstatus_reg;

        SC_METHOD(proc_cmb_abstractcs_out);
        sensitive << abstractcs_reg;

        SC_METHOD(proc_cmb_command_out);
        sensitive << command_reg;

        SC_METHOD(proc_cmb_new_acmds_out);
        sensitive << abstractauto_reg
                  << command_reg_written
                  << data_regs_written;

        SC_CTHREAD(proc_clk_wb, clk.pos());
        SC_CTHREAD(proc_clk_data, clk.pos());
        SC_CTHREAD(proc_clk_dmcontrol, clk.pos());
        SC_CTHREAD(proc_clk_dmstatus, clk.pos());
        SC_CTHREAD(proc_clk_abstractcs, clk.pos());
        SC_CTHREAD(proc_clk_command, clk.pos());
        SC_CTHREAD(proc_clk_abstractcauto, clk.pos());
        SC_CTHREAD(proc_clk_progbuf, clk.pos());

        SC_CTHREAD(proc_clk_hartcontrol, clk.pos());
        SC_CTHREAD(proc_clk_hartstatus, clk.pos());
    }

    // Functions...
    void pn_trace(sc_trace_file * tf, int levels = 1);

    // Processes...
    void proc_cmb_read_dmi();
    void proc_cmb_wb();

    void proc_cmb_dmcontrol_out();

    void proc_clk_wb();

    void proc_cmb_new_acmds_out();

    void proc_clk_data();
    void proc_clk_dmcontrol();
    void proc_clk_dmstatus();
    void proc_clk_abstractcs();
    void proc_cmb_abstractcs_out();
    void proc_clk_command();
    void proc_cmb_command_out();
    void proc_clk_abstractcauto();
    void proc_clk_progbuf();

    void proc_clk_hartcontrol();
    void proc_clk_hartstatus();
    void proc_cmb_hartstatus_out();

protected:
    enum e_wb_state
    {
        WB_IDLE = 0,
        WB_READ,
        WB_WRITE1,
        WB_WRITE2
    };

private:
    sc_uint<32> write_with_byte_select(sc_uint<32> input_word);
    sc_uint<32> read_with_byte_select(sc_uint<32> input_word);

protected:
    const uint64_t wb_base_address = 0; // According to the debug spec. the base address of the debug module is zero 0x0
    const uint64_t wb_size = 0x3C;

    // Registers ...
    // Interfaced by DMI bus only
    sc_signal<sc_uint<32>> PN_NAME(dmcontrol_reg);
    sc_signal<sc_uint<32>> PN_NAME(dmstatus_reg);
    sc_signal<sc_uint<32>> PN_NAME(abstractauto_reg);

    // Debug register: ABSTRACTCS
    sc_signal<sc_uint<32>> PN_NAME(abstractcs_reg);
    sc_signal<sc_uint<3>> PN_NAME(s_abstractcs_cmderr);
    sc_signal<bool> PN_NAME(s_abstractcs_busy);

    // Debug register: COMMAND
    sc_signal<sc_uint<32>> PN_NAME(command_reg);
    sc_signal<bool> PN_NAME(command_reg_written);

    // Interfaced by system bus only
    sc_signal<sc_uint<32>> PN_NAME(hartcontrol_reg);

    // System bus register: HARTSTATUS
    sc_signal<sc_uint<32>> PN_NAME(hartstatus_reg);

    // Interfaced by DMI bus and system bus
    static const size_t NUM_DATA_REGS = 2;
    static const size_t NUM_PROGBUF_REGS = 3;

    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(data_regs, NUM_DATA_REGS);
    sc_signal<sc_uint<NUM_DATA_REGS>> PN_NAME(data_regs_written);
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(progbuf_regs, NUM_PROGBUF_REGS);

    // Wishbone fsm
    sc_signal<sc_uint<3>> PN_NAME(wb_current_state);
    sc_signal<sc_uint<3>> PN_NAME(wb_next_state);

    // Control signals
    sc_signal<bool> PN_NAME(c_wb_write_en);

    // Status signals

    // Submodules
};

#endif
