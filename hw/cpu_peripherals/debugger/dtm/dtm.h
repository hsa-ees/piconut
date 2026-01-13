/**
 * @file dtm.h
 * @brief This file contains the declaration of the m_dtm module.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the declaration of the m_dtm module.

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
 * @addtogroup dtm
 *
 * The `Debug Transport Module (DTM)` provide access to the `Debug Module (DM)`
 * over JTAG. In the "RISC-V Debug Specification" standard, the specification for
 * a `DTM` with a JTAG interface is defined. This is based on the definition of
 * a `TAP` in the JTAG standard, which enables access to custom-defined JTAG registers.
 * There are four registers implemented to accomplish this.
 *
 * | Name   | Address |  Description                                                                                                   |
 * | ------ | ------- | -------------------------------------------------------------------------------------------------------------- |
 * | BYPASS | 0x00    | This register has 1-bit length and has no effect on. Used for bypass data. (JTAG standard)                     |
 * | IDCODE | 0x01    | This register is read-only. It holds the value `0xdeadbeef`. Used for identification purposes. (JTAG standard) |
 * | DTMCS  | 0x10    | This register is used to control the `DMI-bus` and to reset the `DMI-bus`.                                     |
 * | DMI    | 0x11    | This register is used to access the `DM` via the `DMI-bus`.                                                    |
 *
 * The `DMI-bus` is used to read or write to registers inside the `DM`.
 *
 * TODO: interface description.
 *
 */

#ifndef __DTM_H__
#define __DTM_H__

#include <systemc.h>

#include <piconut.h>

SC_MODULE(m_dtm)
{
public:
    // --------------- System ---------------
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    // --------------- Jtag ---------------
    sc_in<bool> PN_NAME(tck_i);
    sc_in<bool> PN_NAME(tms_i);
    sc_in<bool> PN_NAME(tdi_i);
    sc_in<bool> PN_NAME(trst_n_i);
    sc_out<bool> PN_NAME(tdo_o);

    // --------------- Dmi ---------------
    sc_out<sc_uint<PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH>> PN_NAME(dmi_adr_o); // Dmi address
    sc_out<sc_uint<32>> PN_NAME(dmi_dat_o);                             // Dmi output data
    sc_in<sc_uint<32>> PN_NAME(dmi_dat_i);                              // Dmi input data
    sc_out<bool> PN_NAME(dmi_re_o);                                     // Dmi read enable
    sc_out<bool> PN_NAME(dmi_we_o);                                     // Dmi write enable

#if !PN_PRESYNTHESIZED_H_ONLY(DTM)
    // Constructor...
    SC_CTOR(m_dtm)
    {
        init_submodules();

        SC_METHOD(proc_cmb_trans_tap);
        sensitive << jtag_sync_tms
                  << s_dtmcs_dtmhardreset
                  << state
                  << ir_reg;

        SC_METHOD(proc_cmb_trans_dmi);
        sensitive << c_tap_update_dr
                  << ir_reg
                  << dmi_reg
                  << state_dmi;

        SC_CTHREAD(proc_clk_tap, clk.pos());
        SC_CTHREAD(proc_clk_dmi, clk.pos());

        SC_CTHREAD(proc_clk_ir, clk.pos());
        SC_CTHREAD(proc_clk_dr_bypass, clk.pos());
        SC_CTHREAD(proc_clk_dr_idcode, clk.pos());
        SC_CTHREAD(proc_clk_dr_dtmcs, clk.pos());
        SC_CTHREAD(proc_clk_dr_dmi, clk.pos());

        SC_METHOD(proc_cmb_out_dtmcs);
        sensitive << dtmcs_reg;

        SC_METHOD(proc_cmb_out_tdo);
        sensitive << ir_reg
                  << bypass_reg
                  << idcode_reg
                  << dtmcs_reg
                  << dmi_reg
                  << c_tap_shift_ir
                  << c_tap_shift_dr;
    }

    // Functions...
    void pn_trace(sc_trace_file * tf, int levels = 1);

    // Submodules
    void init_submodules();

    class m_jtag_sync* jtag_sync;

    // Processes...
    void proc_cmb_trans_tap();
    void proc_clk_tap();

    void proc_cmb_trans_dmi();
    void proc_clk_dmi();

    void proc_clk_ir();
    void proc_clk_dr_bypass();
    void proc_clk_dr_idcode();
    void proc_clk_dr_dtmcs();
    void proc_clk_dr_dmi();

    void proc_cmb_out_dtmcs();
    void proc_cmb_out_tdo();

#else

    SC_CTOR(m_dtm) {}
    void pn_trace(sc_trace_file * tf, int level = 1) {}

#endif

protected:
#if !PN_PRESYNTHESIZED_H_ONLY(DTM)

    static constexpr size_t instruction_reg_width = 5;
    static constexpr size_t data_reg_width_bypass = 1;
    static constexpr size_t data_reg_width_idcode = 32;
    static constexpr size_t data_reg_width_dtmcs = 32;
    static constexpr size_t data_reg_width_dmi = PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH + 34;

protected:
    enum e_tap_state
    {
        TEST_LOGIC_RESET,
        RUN_TEST_IDLE,
        SELECT_DR_SCAN,
        CAPTURE_DR,
        SHIFT_DR,
        EXIT1_DR,
        PAUSE_DR,
        EXIT2_DR,
        UPDATE_DR,
        SELECT_IR_SCAN,
        CAPTURE_IR,
        SHIFT_IR,
        EXIT1_IR,
        PAUSE_IR,
        EXIT2_IR,
        UPDATE_IR
    };

    enum e_dmi_state
    {
        IDLE,
        READ,
        WRITE,
        WAIT,
    };

    enum e_regs
    {
        BYPASS = 0x0,
        IDCODE = 0x1,
        DTMCS = 0x10,
        DMI = 0x11,
    };

protected:
    // Registers ...
    // TAP registers
    sc_signal<sc_uint<instruction_reg_width>> ir_reg;
    sc_signal<sc_uint<data_reg_width_bypass>> bypass_reg;
    sc_signal<sc_uint<data_reg_width_idcode>> idcode_reg;
    sc_signal<sc_uint<data_reg_width_dtmcs>> dtmcs_reg;
    sc_signal<sc_uint<data_reg_width_dmi>> dmi_reg;

    sc_signal<sc_uint<4>> state;
    sc_signal<sc_uint<4>> next_state;

    sc_signal<sc_uint<2>> state_dmi;
    sc_signal<sc_uint<2>> next_state_dmi;

    // Control signals
    sc_signal<bool> c_tap_test_logic_reset;
    sc_signal<bool> c_tap_capture_dr;
    sc_signal<bool> c_tap_shift_dr;
    sc_signal<bool> c_tap_update_dr;
    sc_signal<bool> c_tap_shift_ir;

    sc_signal<bool> c_dmi_read;
    sc_signal<bool> c_dmi_write;

    // Status signals
    sc_signal<bool> s_dtmcs_dtmhardreset;

    // Submodules
    sc_signal<bool> PN_NAME(jtag_sync_tck_en);
    sc_signal<bool> PN_NAME(jtag_sync_tms);
    sc_signal<bool> PN_NAME(jtag_sync_tdi);
    sc_signal<bool> PN_NAME(jtag_sync_trst_n_en);

#else
    PN_PRESYNTHESIZED;
#endif
    //
};

#endif
