/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the definition of the m_dm_registers module.

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

#include "dm_registers.h"
#include "dm_defs.h"

void m_dm_registers::pn_trace(sc_trace_file* tf, int level)
{
    // Ports...
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    // DMI
    PN_TRACE(tf, dmi_adr_i);
    PN_TRACE(tf, dmi_dat_i);
    PN_TRACE(tf, dmi_dat_o);
    PN_TRACE(tf, dmi_re_i);
    PN_TRACE(tf, dmi_we_i);

    // Wishbone
    PN_TRACE(tf, wb_ack_o);
    PN_TRACE(tf, wb_adr_i);
    PN_TRACE(tf, wb_cyc_i);
    PN_TRACE(tf, wb_stb_i);
    PN_TRACE(tf, wb_dat_i);
    PN_TRACE(tf, wb_dat_o);
    PN_TRACE(tf, wb_sel_i);
    PN_TRACE(tf, wb_we_i);
    PN_TRACE(tf, wb_err_o);
    PN_TRACE(tf, c_wb_write_en);

    // Data flow signals
    PN_TRACE_BUS(tf, abstract_regs_i, NUM_ABSTRACT_REGS);
    PN_TRACE(tf, command_reg_o);

    // Control signals
    PN_TRACE(tf, c_hart_halted_i);
    PN_TRACE(tf, c_hart_running_i);
    PN_TRACE(tf, c_hart_run_acmds_i);
    PN_TRACE(tf, c_hart_resumereq_i);
    PN_TRACE(tf, c_hart_resumeack_i);
    PN_TRACE(tf, c_acmds_generate_i);
    PN_TRACE(tf, c_acmds_runreq_i);
    PN_TRACE(tf, c_acmds_increment_data1_i);

    // Status signals
    PN_TRACE(tf, s_new_acmds_o);
    PN_TRACE(tf, s_abstractcs_cmderr_o);
    PN_TRACE(tf, s_dmcontrol_haltreq_o);
    PN_TRACE(tf, s_dmcontrol_resumereq_o);
    PN_TRACE(tf, s_hartstatus_halted_o);
    PN_TRACE(tf, s_hartstatus_running_o);
    PN_TRACE(tf, s_hartstatus_acmds_running_o);

    // Internal
    PN_TRACE(tf, wb_current_state);
    PN_TRACE(tf, wb_next_state);

    PN_TRACE(tf, command_reg_written);
    PN_TRACE(tf, data_regs_written);

    PN_TRACE_BUS(tf, data_regs, NUM_DATA_REGS);
    PN_TRACE(tf, dmcontrol_reg);
    PN_TRACE(tf, dmstatus_reg);
    PN_TRACE(tf, abstractcs_reg);
    PN_TRACE(tf, command_reg);
    PN_TRACE_BUS(tf, progbuf_regs, NUM_PROGBUF_REGS);
    PN_TRACE(tf, hartcontrol_reg);
    PN_TRACE(tf, hartstatus_reg);
    PN_TRACE(tf, abstractauto_reg);
}

void m_dm_registers::proc_cmb_read_dmi()
{
    dmi_dat_o = 0;

    if(dmi_re_i.read() == 1)
    {
        switch(dmi_adr_i.read())
        {
            case e_dmi_reg_adrs::DMI_DATA0:
                dmi_dat_o = data_regs[0].read();
                break;
            case e_dmi_reg_adrs::DMI_DATA1:
                dmi_dat_o = data_regs[1].read();
                break;
            case e_dmi_reg_adrs::DMI_DMCONTROL:
                dmi_dat_o = dmcontrol_reg.read();
                break;
            case e_dmi_reg_adrs::DMI_DMSTATUS:
                dmi_dat_o = dmstatus_reg.read();
                break;
            case e_dmi_reg_adrs::DMI_ABSTRACTCS:
                dmi_dat_o = abstractcs_reg.read();
                break;
            case e_dmi_reg_adrs::DMI_COMMAND:
                dmi_dat_o = command_reg.read();
                break;
            case e_dmi_reg_adrs::DMI_ABSTRACTAUTO:
                dmi_dat_o = abstractauto_reg.read();
                break;
            case e_dmi_reg_adrs::DMI_PROGBUF0:
                dmi_dat_o = progbuf_regs[0].read();
                break;
            case e_dmi_reg_adrs::DMI_PROGBUF1:
                dmi_dat_o = progbuf_regs[1].read();
                break;
            case e_dmi_reg_adrs::DMI_PROGBUF2:
                dmi_dat_o = progbuf_regs[2].read();
                break;
            default:
                dmi_dat_o = 0;
                break;
        }
    }
}

void m_dm_registers::proc_cmb_wb()
{
    wb_ack_o = 0;
    wb_dat_o = 0;
    wb_err_o = 0;
    wb_rty_o = 0;

    c_wb_write_en = 0;

    wb_next_state = wb_current_state.read();

    switch(wb_current_state.read())
    {
        case WB_IDLE:
            if(wb_stb_i.read() == 1 && wb_cyc_i.read() == 1)
            {
                // if((wb_adr_i.read() >= wb_base_address) &&
                //     (wb_adr_i.read() < (wb_base_address + wb_size)))
                if((wb_adr_i.read() >= 0) &&
                    (wb_adr_i.read() < 0x3C))
                {
                    if(wb_we_i.read() == 1)
                    {
                        wb_next_state = WB_WRITE1;
                    }
                    else
                    {
                        wb_next_state = WB_READ;
                    }
                }
            }
            break;

        case WB_WRITE1:
            c_wb_write_en = 1;
            wb_next_state = WB_WRITE2;
            break;

        case WB_WRITE2:
            wb_ack_o = 1;
            if(wb_stb_i.read() == 0 /* && wb_cyc_i.read() == 0*/)
            {
                wb_next_state = WB_IDLE;
            }
            break;

        case WB_READ:

            switch(wb_adr_i.read() - wb_base_address)
            {
                case e_wb_reg_adrs::WB_DATA0:
                    wb_dat_o = read_with_byte_select(data_regs[0].read());
                    break;
                case e_wb_reg_adrs::WB_DATA1:
                    wb_dat_o = read_with_byte_select(data_regs[1].read());
                    break;
                case e_wb_reg_adrs::WB_PROGBUG0:
                    wb_dat_o = read_with_byte_select(progbuf_regs[0].read());
                    break;
                case e_wb_reg_adrs::WB_PROGBUG1:
                    wb_dat_o = read_with_byte_select(progbuf_regs[1].read());
                    break;
                case e_wb_reg_adrs::WB_PROGBUG2:
                    wb_dat_o = read_with_byte_select(progbuf_regs[2].read());
                    break;
                case e_wb_reg_adrs::WB_ABSTRACT0:
                    wb_dat_o = read_with_byte_select(abstract_regs_i[0].read());
                    break;
                case e_wb_reg_adrs::WB_ABSTRACT1:
                    wb_dat_o = read_with_byte_select(abstract_regs_i[1].read());
                    break;
                case e_wb_reg_adrs::WB_ABSTRACT2:
                    wb_dat_o = read_with_byte_select(abstract_regs_i[2].read());
                    break;
                case e_wb_reg_adrs::WB_ABSTRACT3:
                    wb_dat_o = read_with_byte_select(abstract_regs_i[3].read());
                    break;
                case e_wb_reg_adrs::WB_ABSTRACT4:
                    wb_dat_o = read_with_byte_select(abstract_regs_i[4].read());
                    break;
                case e_wb_reg_adrs::WB_ABSTRACT5:
                    wb_dat_o = read_with_byte_select(abstract_regs_i[5].read());
                    break;
                case e_wb_reg_adrs::WB_ABSTRACT6:
                    wb_dat_o = read_with_byte_select(abstract_regs_i[6].read());
                    break;
                case e_wb_reg_adrs::WB_ABSTRACT7:
                    wb_dat_o = read_with_byte_select(abstract_regs_i[7].read());
                    break;
                case e_wb_reg_adrs::WB_HARTCONTROL:
                    wb_dat_o = read_with_byte_select(hartcontrol_reg.read());
                    break;
                case e_wb_reg_adrs::WB_HARTSTATUS:
                    // This register is write-only from wishbone bus
                    break;
                default:
                    wb_dat_o = 0;
                    break;
            }

            wb_ack_o = 1;

            if(wb_stb_i.read() == 0 /* && wb_cyc_i.read() == 0*/)
            {
                wb_next_state = WB_IDLE;
            }
            break;
    }
}

void m_dm_registers::proc_clk_wb()
{
    while(true)
    {

        if(reset.read() == 1)
        {
            wb_current_state = WB_IDLE;
        }
        else
        {
            wb_current_state = wb_next_state;
        }

        wait();
    }
}

void m_dm_registers::proc_clk_data()
{
    while(true)
    {
        sc_uint<NUM_DATA_REGS> data_regs_written_var = 0;

        if(reset.read() == 1)
        {
            for(size_t i = 0; i < NUM_DATA_REGS; ++i)
            {
                data_regs[i] = 0;
            }
        }
        else if(c_wb_write_en.read() == 1)
        {
            switch(wb_adr_i.read() - wb_base_address)
            {
                case e_wb_reg_adrs::WB_DATA0:
                    data_regs[0] = write_with_byte_select(wb_dat_i.read());
                    break;
                case e_wb_reg_adrs::WB_DATA1:
                    data_regs[1] = write_with_byte_select(wb_dat_i.read());
                    break;
                default:
                    break;
            }
        }
        else if(dmi_we_i.read() == 1)
        {
            switch(dmi_adr_i.read())
            {
                case e_dmi_reg_adrs::DMI_DATA0:
                    data_regs[0] = dmi_dat_i.read();
                    data_regs_written_var[0] = 1;
                    break;
                case e_dmi_reg_adrs::DMI_DATA1:
                    data_regs[1] = dmi_dat_i.read();
                    data_regs_written_var[1] = 1;
                    break;
                default:
                    break;
            }
        }
        else if(c_acmds_increment_data1_i.read() == 1)
        {
            data_regs[1] = data_regs[1].read() + sc_uint<32>(0x4);
        }

        data_regs_written = data_regs_written_var;

        wait();
    }
}

void m_dm_registers::proc_clk_dmcontrol()
{
    while(true)
    {
        sc_uint<32> dmcontrol_reg_var = dmcontrol_reg.read();
        if(reset.read() == 1)
        {
            dmcontrol_reg_var = 0;
        }
        else
        {
            if(dmi_we_i.read() == 1 &&
                dmi_adr_i.read() == e_dmi_reg_adrs::DMI_DMCONTROL)
            {
                dmcontrol_reg_var = dmi_dat_i.read();

                // Check for hartsellen detection
                if(dmcontrol_reg_var.range(25, 6) == 0xFFFFF)
                {
                    dmcontrol_reg_var.range(25, 6) = 0; // 1 hart available
                }
            }
            else
            {
                //
                // The debugger (OpenOCD) doesnt clear the resumereq bit fast
                // enough by himself. A single step would fall trough
                // the debug handler because resumereq is still set.
                //
                if(c_hart_resumeack_i.read() == 1)
                {
                    dmcontrol_reg_var[DMCONTROL_RESUMEREQ] = 0;
                }
            }
        }
        dmcontrol_reg = dmcontrol_reg_var;

        wait();
    }
}

void m_dm_registers::proc_cmb_dmcontrol_out()
{
    s_dmcontrol_haltreq_o = dmcontrol_reg.read()[DMCONTROL_HALTREQ];
    s_dmcontrol_resumereq_o = dmcontrol_reg.read()[DMCONTROL_RESUMEREQ];
}

void m_dm_registers::proc_clk_dmstatus()
{
    while(true)
    {
        sc_uint<32> dmstatus_reg_var = dmstatus_reg.read();
        if(reset.read() == 1)
        {
            dmstatus_reg_var = 0;
            dmstatus_reg_var.range(DMSTATUS_VERSION) =
                e_dmstatus_version::DMSTATUS_VERSION_1_0;
            dmstatus_reg_var[DMSTATUS_AUTHENTICATED] = 1;
        }
        else
        {
            // halt / running
            if(c_hart_halted_i.read() == 1)
            {
                dmstatus_reg_var[DMSTATUS_ANYHALTED] = 1;
                dmstatus_reg_var[DMSTATUS_ALLHALTED] = 1;
                dmstatus_reg_var[DMSTATUS_ANYRUNNING] = 0;
                dmstatus_reg_var[DMSTATUS_ALLRUNNING] = 0;
            }
            else if(c_hart_running_i.read() == 1)
            {
                dmstatus_reg_var[DMSTATUS_ANYHALTED] = 0;
                dmstatus_reg_var[DMSTATUS_ALLHALTED] = 0;
                dmstatus_reg_var[DMSTATUS_ANYRUNNING] = 1;
                dmstatus_reg_var[DMSTATUS_ALLRUNNING] = 1;
            }

            // resumeack
            if(c_hart_resumeack_i.read() == 1)
            {
                dmstatus_reg_var[DMSTATUS_ANYRESUMEACK] = 1;
                dmstatus_reg_var[DMSTATUS_ALLRESUMEACK] = 1;
            }
            else if(c_hart_resumereq_i.read() == 1)
            {
                dmstatus_reg_var[DMSTATUS_ANYRESUMEACK] = 0;
                dmstatus_reg_var[DMSTATUS_ALLRESUMEACK] = 0;
            }
        }

        dmstatus_reg = dmstatus_reg_var;

        wait();
    }
}

void m_dm_registers::proc_clk_abstractcs()
{
    while(true)
    {
        sc_uint<32> abstractcs_reg_var = abstractcs_reg.read();

        if(reset.read() == 1)
        {
            abstractcs_reg_var = 0;
            abstractcs_reg_var.range(ABSTRACTCS_DATACOUNT) = NUM_DATA_REGS;
            abstractcs_reg_var.range(ABSTRACTCS_PROGBUFSIZE) = 0; // NUM_PROGBUF_REGS;
        }
        else
        {
            if(dmi_we_i.read() == 1 &&
                dmi_adr_i.read() == e_dmi_reg_adrs::DMI_ABSTRACTCS)
            {
                abstractcs_reg_var[ABSTRACTCS_RELAXEDPRIV] = dmi_dat_i.read()[ABSTRACTCS_RELAXEDPRIV];

                if(dmi_dat_i.read().range(ABSTRACTCS_CMDERR) == 0b111)
                {
                    abstractcs_reg_var.range(ABSTRACTCS_CMDERR) = ABSTRACTCS_CMDERR_NONE;
                }
            }

            // cmderr
            // if(s_command_new.read() == 1 &&
            //     s_hartstatus_commands_running.read() == 1 &&
            //     abstractcs_reg_var.range(ABSTRACTCS_CMDERR) == ABSTRACTCS_CMDERR_NONE)
            // {
            //     abstractcs_reg_var.range(ABSTRACTCS_CMDERR) = ABSTRACTCS_CMDERR_BUSY;
            // }
            if(c_acmds_error_not_supported_i.read() == 1)
            {
                abstractcs_reg_var.range(ABSTRACTCS_CMDERR) = ABSTRACTCS_CMDERR_NOTSUPPORT;
            }
            // else if(s_command_new.read() == 1 &&
            //         s_hartstatus_running.read() == 1)
            // {
            //     abstractcs_reg_var.range(ABSTRACTCS_CMDERR) = ABSTRACTCS_CMDERR_HALT_RESUME;
            // }

            // busy
            if(c_acmds_busy_i.read() == 1)
            {
                abstractcs_reg_var[ABSTRACTCS_BUSY] = 1;
            }
            else
            {
                abstractcs_reg_var[ABSTRACTCS_BUSY] = 0;
            }
        }

        abstractcs_reg = abstractcs_reg_var;

        wait();
    }
}

void m_dm_registers::proc_cmb_abstractcs_out()
{
    s_abstractcs_cmderr = abstractcs_reg.read().range(ABSTRACTCS_CMDERR);
    s_abstractcs_busy = abstractcs_reg.read()[ABSTRACTCS_BUSY];
}

void m_dm_registers::proc_clk_command()
{
    while(true)
    {
        if(reset.read() == 1)
        {
            command_reg = 0;
        }
        else if(dmi_we_i.read() == 1 &&
                dmi_adr_i.read() == e_dmi_reg_adrs::DMI_COMMAND &&
                s_abstractcs_cmderr.read() == ABSTRACTCS_CMDERR_NONE)
        {
            command_reg = dmi_dat_i.read();

            command_reg_written = 1;
        }
        else
        {
            command_reg_written = 0;
        }

        wait();
    }
}

void m_dm_registers::proc_cmb_command_out()
{
    command_reg_o = command_reg.read();
}

void m_dm_registers::proc_clk_abstractcauto()
{
    while(true)
    {
        if(reset.read() == 1)
        {
            abstractauto_reg = 0;
        }
        else if(dmi_we_i.read() == 1 &&
                dmi_adr_i.read() == e_dmi_reg_adrs::DMI_ABSTRACTAUTO)
        {
            abstractauto_reg = dmi_dat_i.read();
        }

        wait();
    }
}

void m_dm_registers::proc_clk_progbuf()
{
    while(true)
    {
        if(reset.read() == 1)
        {
            for(size_t i = 0; i < NUM_PROGBUF_REGS; ++i)
            {
                progbuf_regs[i] = 0;
            }
        }
        else if(dmi_we_i.read() == 1)
        {
            uint32_t progbuf_offset_var =
                dmi_adr_i.read() - e_dmi_reg_adrs::DMI_PROGBUF0;
            if(progbuf_offset_var < NUM_PROGBUF_REGS)
            {
                progbuf_regs[progbuf_offset_var] = dmi_dat_i.read();
            }
        }

        wait();
    }
}

void m_dm_registers::proc_clk_hartcontrol()
{
    while(true)
    {
        sc_uint<32> hartcontrol_reg_var = hartcontrol_reg.read();
        if(reset.read() == 1)
        {
            hartcontrol_reg_var = 0;
        }
        else
        {
            hartcontrol_reg_var[HARTCONTROL_RESUMEREQ] = c_hart_resumereq_i.read();
            hartcontrol_reg_var[HARTCONTROL_ACMDS_RUNREQ] = c_acmds_runreq_i.read();
        }

        hartcontrol_reg = hartcontrol_reg_var;

        wait();
    }
}

void m_dm_registers::proc_clk_hartstatus()
{
    while(true)
    {
        sc_uint<32> hartstatus_reg_var = hartstatus_reg.read();
        if(reset.read() == 1)
        {
            hartstatus_reg_var = 0;
            hartstatus_reg_var |= 1U << 1; // Running
        }
        else if(c_wb_write_en.read() == 1 &&
                (wb_adr_i.read() - wb_base_address) == e_wb_reg_adrs::WB_HARTSTATUS)
        {
            hartstatus_reg_var = write_with_byte_select(wb_dat_i.read());
        }

        hartstatus_reg = hartstatus_reg_var;

        wait();
    }
}

void m_dm_registers::proc_cmb_hartstatus_out()
{
    s_hartstatus_halted_o = hartstatus_reg.read()[HARTSTATUS_HALTED];
    s_hartstatus_running_o = hartstatus_reg.read()[HARTSTATUS_RUNNING];
    // s_hartstatus_havereset_o = hartstatus_reg.read()[HARTSTATUS_HAVERESSET];
    s_hartstatus_acmds_running_o = hartstatus_reg.read()[HARTSTATUS_COMMANDS_RUNNING];
}

void m_dm_registers::proc_cmb_new_acmds_out()
{
    s_new_acmds_o = command_reg_written || (data_regs_written.read() & abstractauto_reg.read().range(NUM_DATA_REGS - 1, 0));
}

//////////////////////// Helper functions ////////////////////////
sc_uint<WB_DAT_WIDTH> m_dm_registers::write_with_byte_select(
    sc_uint<WB_DAT_WIDTH> input_word)
{
    sc_uint<WB_DAT_WIDTH> mask = 0;
    sc_uint<WB_DAT_WIDTH / 8> wb_sel_i_var = wb_sel_i.read();

    // Selective byte masking
    if(wb_sel_i_var & 0b0001)
    {
        mask |= 0x000000FF; // Byte 0
    }
    if(wb_sel_i_var & 0b0010)
    {
        mask |= 0x0000FF00; // Byte 1
    }
    if(wb_sel_i_var & 0b0100)
    {
        mask |= 0x00FF0000; // Byte 2
    }
    if(wb_sel_i_var & 0b1000)
    {
        mask |= 0xFF000000; // Byte 3
    }

    return (input_word & ~mask) | (wb_dat_i.read() & mask);
}

sc_uint<WB_DAT_WIDTH> m_dm_registers::read_with_byte_select(
    sc_uint<WB_DAT_WIDTH> input_word)
{
    sc_uint<WB_DAT_WIDTH> mask = 0;
    sc_uint<WB_DAT_WIDTH / 8> wb_sel_i_var = wb_sel_i.read();

    // Selective byte masking
    if(wb_sel_i_var & 0b0001)
    {
        mask |= 0x000000FF; // Byte 0
    }
    if(wb_sel_i_var & 0b0010)
    {
        mask |= 0x0000FF00; // Byte 1
    }
    if(wb_sel_i_var & 0b0100)
    {
        mask |= 0x00FF0000; // Byte 2
    }
    if(wb_sel_i_var & 0b1000)
    {
        mask |= 0xFF000000; // Byte 3
    }

    return (input_word & mask);
}
