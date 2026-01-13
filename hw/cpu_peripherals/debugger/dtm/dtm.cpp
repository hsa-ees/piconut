/**
 * @file dtm.cpp
 * @brief This file contains the definition of the m_dtm module.
 * @author Johannes Hofmann
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the definition of the m_dtm module.

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

#include "dtm.h"

#include "jtag_sync.h"

enum e_dmi_op_code
{
    DMI_OP_NOP = 0b00,
    DMI_OP_READ = 0b01,
    DMI_OP_WRITE = 0b10,
    DMI_OP_RESERVED = 0b11
};

void m_dtm::pn_trace(sc_trace_file* tf, int level)
{
    // Ports...
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    PN_TRACE(tf, tck_i);
    PN_TRACE(tf, tms_i);
    PN_TRACE(tf, tdi_i);
    PN_TRACE(tf, tdo_o);
    PN_TRACE(tf, trst_n_i);

    PN_TRACE(tf, dmi_adr_o);
    PN_TRACE(tf, dmi_dat_o);
    PN_TRACE(tf, dmi_dat_i);
    PN_TRACE(tf, dmi_re_o);
    PN_TRACE(tf, dmi_we_o);

    // Signals
    PN_TRACE(tf, ir_reg);
    PN_TRACE(tf, bypass_reg);
    PN_TRACE(tf, idcode_reg);
    PN_TRACE(tf, dtmcs_reg);
    PN_TRACE(tf, dmi_reg);

    PN_TRACE(tf, state);
    PN_TRACE(tf, next_state);

    PN_TRACE(tf, state_dmi);
    PN_TRACE(tf, next_state_dmi);

    PN_TRACE(tf, c_tap_test_logic_reset);
    PN_TRACE(tf, c_tap_capture_dr);
    PN_TRACE(tf, c_tap_shift_dr);
    PN_TRACE(tf, c_tap_update_dr);
    PN_TRACE(tf, c_tap_shift_ir);

    PN_TRACE(tf, c_dmi_read);

    PN_TRACE(tf, jtag_sync_tck_en);
    PN_TRACE(tf, jtag_sync_tms);
    PN_TRACE(tf, jtag_sync_tdi);
    PN_TRACE(tf, jtag_sync_trst_n_en);

    jtag_sync->pn_trace(tf, level);
}

void m_dtm::init_submodules()
{
    jtag_sync = sc_new<m_jtag_sync>("jtag_sync");

    jtag_sync->clk(clk);
    jtag_sync->reset(reset);
    jtag_sync->tck_i(tck_i);
    jtag_sync->tms_i(tms_i);
    jtag_sync->tdi_i(tdi_i);
    jtag_sync->trst_n_i(trst_n_i);
    jtag_sync->tck_sync_en_o(jtag_sync_tck_en);
    jtag_sync->tms_sync_o(jtag_sync_tms);
    jtag_sync->tdi_sync_o(jtag_sync_tdi);
    jtag_sync->trst_n_sync_en_o(jtag_sync_trst_n_en);
}

void m_dtm::proc_cmb_trans_dmi()
{
    dmi_adr_o = 0;
    dmi_dat_o = 0;
    dmi_re_o = 0;
    dmi_we_o = 0;
    c_dmi_read = 0;
    c_dmi_write = 0;

    sc_uint<2> op_var = dmi_reg.read()(1, 0);
    sc_uint<32> dat_var =
        dmi_reg.read()(data_reg_width_dmi - PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH - 1, 2);
    sc_uint<PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH> adr_var =
        dmi_reg.read()(data_reg_width_dmi - 1, 34);

    next_state_dmi = state_dmi.read();

    switch(state_dmi.read())
    {
        case e_dmi_state::IDLE:
            if(ir_reg.read() == e_regs::DMI &&
                c_tap_update_dr.read() == 1 &&
                op_var == e_dmi_op_code::DMI_OP_READ)
            {
                next_state_dmi = e_dmi_state::READ;
            }
            else if(ir_reg.read() == e_regs::DMI &&
                    c_tap_update_dr.read() == 1 &&
                    op_var == e_dmi_op_code::DMI_OP_WRITE)
            {
                next_state_dmi = e_dmi_state::WRITE;
            }
            break;

        case e_dmi_state::READ:
            dmi_adr_o = adr_var;
            dmi_re_o = 1;
            c_dmi_read = 1;
            next_state_dmi = e_dmi_state::WAIT;
            break;

        case e_dmi_state::WRITE:
            dmi_adr_o = adr_var;
            dmi_dat_o = dat_var;
            dmi_we_o = 1;
            c_dmi_write = 1;
            next_state_dmi = e_dmi_state::WAIT;
            break;

        case e_dmi_state::WAIT:
            if(c_tap_update_dr.read() == 0)
            {
                next_state_dmi = e_dmi_state::IDLE;
            }
            break;

        default:
            PN_ERRORF(("Unknown DMI state: %d", (uint8_t)state_dmi.read()));
            break;
    }
}

void m_dtm::proc_clk_dmi()
{
    while(true)
    {
        wait();

        if(reset.read() == 1 ||
            jtag_sync_trst_n_en.read() == 0)
        {
            state_dmi = e_dmi_state::IDLE;
        }
        else
        {
            state_dmi = next_state_dmi.read();
        }
    }
}

void m_dtm::proc_cmb_trans_tap()
{
    c_tap_test_logic_reset = 0;
    c_tap_capture_dr = 0;
    c_tap_shift_dr = 0;
    c_tap_update_dr = 0;
    c_tap_shift_ir = 0;

    next_state = state.read();

    switch(state.read())
    {
        case TEST_LOGIC_RESET:
            c_tap_test_logic_reset = 1;
            next_state = jtag_sync_tms ? TEST_LOGIC_RESET : RUN_TEST_IDLE;
            break;
        case RUN_TEST_IDLE:
            next_state = jtag_sync_tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
            break;
        case SELECT_DR_SCAN:
            next_state = jtag_sync_tms ? SELECT_IR_SCAN : CAPTURE_DR;
            break;
        case CAPTURE_DR:
            c_tap_capture_dr = 1;
            next_state = jtag_sync_tms ? EXIT1_DR : SHIFT_DR;
            break;
        case SHIFT_DR:
            c_tap_shift_dr = 1;
            next_state = jtag_sync_tms ? EXIT1_DR : SHIFT_DR;
            break;
        case EXIT1_DR:
            next_state = jtag_sync_tms ? UPDATE_DR : PAUSE_DR;
            break;
        case PAUSE_DR:
            next_state = jtag_sync_tms ? EXIT2_DR : PAUSE_DR;
            break;
        case EXIT2_DR:
            next_state = jtag_sync_tms ? UPDATE_DR : SHIFT_DR;
            break;
        case UPDATE_DR:
            c_tap_update_dr = 1;

            if(ir_reg.read() == e_regs::DTMCS &&
                s_dtmcs_dtmhardreset.read() == 1)
            {
                next_state = TEST_LOGIC_RESET;
            }
            else
            {
                next_state = jtag_sync_tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
            }
            break;
        case SELECT_IR_SCAN:
            next_state = jtag_sync_tms ? TEST_LOGIC_RESET : CAPTURE_IR;
            break;
        case CAPTURE_IR:
            next_state = jtag_sync_tms ? EXIT1_IR : SHIFT_IR;
            break;
        case SHIFT_IR:
            c_tap_shift_ir = 1;
            next_state = jtag_sync_tms ? EXIT1_IR : SHIFT_IR;
            break;
        case EXIT1_IR:
            next_state = jtag_sync_tms ? UPDATE_IR : PAUSE_IR;
            break;
        case PAUSE_IR:
            next_state = jtag_sync_tms ? EXIT2_IR : PAUSE_IR;
            break;
        case EXIT2_IR:
            next_state = jtag_sync_tms ? UPDATE_IR : SHIFT_IR;
            break;
        case UPDATE_IR:
            next_state = jtag_sync_tms ? SELECT_DR_SCAN : RUN_TEST_IDLE;
            break;
        default:
            PN_ERRORF(("DTM is in unknown state %d!", (uint8_t)state.read()));
            break;
    }
}

void m_dtm::proc_clk_tap()
{
    while(true)
    {
        wait();

        if(reset.read() == 1 ||
            jtag_sync_trst_n_en.read() == 0)
        {
            state = e_tap_state::TEST_LOGIC_RESET;
        }
        // We use tck as an enable signal for the TAP controller state transition.
        else if(jtag_sync_tck_en.read() == 1)
        {
            state = next_state.read();
        }
    }
}

void m_dtm::proc_clk_ir()
{
    while(true)
    {
        wait();

        if(jtag_sync_tck_en.read() == 1 &&
            c_tap_test_logic_reset.read() == 1)
        {
            ir_reg = e_regs::IDCODE;
        }
        else if(jtag_sync_tck_en.read() == 1 &&
                c_tap_shift_ir.read() == 1)
        {
            ir_reg = (jtag_sync_tdi.read(), ir_reg.read()(instruction_reg_width - 1, 1));
        }
    }
}

void m_dtm::proc_clk_dr_bypass()
{
    while(true)
    {
        wait();

        if(jtag_sync_tck_en.read() == 1 &&
            c_tap_test_logic_reset.read() == 1)
        {
            bypass_reg = 0;
        }
        else if(jtag_sync_tck_en.read() == 1 &&
                ir_reg.read() == e_regs::BYPASS &&
                c_tap_shift_dr.read() == 1)
        {
            bypass_reg = jtag_sync_tdi.read(); // Bypass register is only 1 bit wide
        }
    }
}

void m_dtm::proc_clk_dr_idcode()
{
    while(true)
    {
        wait();

        if(jtag_sync_tck_en.read() == 1 &&
            c_tap_test_logic_reset.read() == 1)
        {
            idcode_reg = 0;
        }
        else if(jtag_sync_tck_en.read() == 1 &&
                ir_reg.read() == e_regs::IDCODE &&
                c_tap_capture_dr.read() == 1)
        {
            idcode_reg = 0xdeadbeef;
        }
        else if(jtag_sync_tck_en.read() == 1 &&
                ir_reg.read() == e_regs::IDCODE &&
                c_tap_shift_dr.read() == 1)
        {
            idcode_reg = (jtag_sync_tdi.read(), idcode_reg.read()(data_reg_width_idcode - 1, 1));
        }
    }
}

void m_dtm::proc_clk_dr_dtmcs()
{
    while(true)
    {
        wait();

        if(jtag_sync_tck_en.read() == 1 &&
            c_tap_test_logic_reset.read() == 1)
        {
            dtmcs_reg = 0;
        }
        else if(jtag_sync_tck_en.read() == 1 &&
                ir_reg.read() == e_regs::DTMCS &&
                c_tap_capture_dr.read() == 1)
        {
            dtmcs_reg = (4U << 12U) | (PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH << 4U) | 1U;
        }
        else if(jtag_sync_tck_en.read() == 1 &&
                ir_reg.read() == e_regs::DTMCS &&
                c_tap_shift_dr.read() == 1)
        {
            dtmcs_reg = (jtag_sync_tdi.read(), dtmcs_reg.read()(data_reg_width_dtmcs - 1, 1));
        }
    }
}

void m_dtm::proc_clk_dr_dmi()
{
    while(true)
    {
        wait();

        if(jtag_sync_tck_en.read() == 1 &&
            c_tap_test_logic_reset.read() == 1)
        {
            dmi_reg = 0;
        }
        else if(jtag_sync_tck_en.read() == 1 &&
                ir_reg.read() == e_regs::DMI &&
                c_tap_capture_dr.read() == 1)
        {
            // Nothing to do here, capturing of the dmi_reg happens in update_dr
            // state
        }
        else if(jtag_sync_tck_en.read() == 1 &&
                ir_reg.read() == e_regs::DMI &&
                c_tap_shift_dr.read() == 1)
        {
            dmi_reg = (jtag_sync_tdi.read(), dmi_reg.read()(data_reg_width_dmi - 1, 1));
        }
        else if(c_dmi_read.read() == 1)
        {
            dmi_reg = dmi_dat_i.read() << 2U;
        }
        else if(c_dmi_write.read() == 1)
        {
            // Clear op field after write
            // Note: It is assumed that all write operations succeed.
            dmi_reg = dmi_reg.read() & ~0x3U;
        }
    }
}

void m_dtm::proc_cmb_out_dtmcs()
{
    s_dtmcs_dtmhardreset = dtmcs_reg.read()[16];
}

void m_dtm::proc_cmb_out_tdo()
{
    tdo_o = 0;

    if(c_tap_shift_ir.read() == 1)
    {
        tdo_o = ir_reg.read()[0];
    }
    else if(c_tap_shift_dr.read() == 1)
    {
        switch(ir_reg.read())
        {
            case e_regs::BYPASS:
                tdo_o = bypass_reg.read()[0];
                break;
            case e_regs::IDCODE:
                tdo_o = idcode_reg.read()[0];
                break;
            case e_regs::DTMCS:
                tdo_o = dtmcs_reg.read()[0];
                break;
            case e_regs::DMI:
                tdo_o = dmi_reg.read()[0];
                break;
            default:
                PN_ERRORF(("Unsupported IR: 0x%02x\n", (uint8_t)ir_reg.read()));
                break;
        }
    }
}
