/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg


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

#include "nucleus_ref.h"

#include "alu.h"
#include "pc.h"
#include "ir.h"
#include "regfile.h"
#include "byteselector.h"
#include "extender.h"
#include "immgen.h"
#include "controller.h"
#include "datahandler.h"
#include "csr_master.h"
#include "csr.h"


void m_nucleus_ref::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, iport_ack_in);
    PN_TRACE(tf, dport_ack_in);

    PN_TRACE(tf, iport_stb_out);
    PN_TRACE(tf, dport_stb_out);

    PN_TRACE(tf, iport_rdata_in);
    PN_TRACE(tf, dport_rdata_in);

    PN_TRACE(tf, iport_adr_out);
    PN_TRACE(tf, dport_adr_out);

    PN_TRACE(tf, iport_bsel_out);
    PN_TRACE(tf, dport_bsel_out);

    PN_TRACE(tf, dport_wdata_out);
    PN_TRACE(tf, dport_we_out);
    PN_TRACE(tf, dport_lrsc_out);
    PN_TRACE(tf, dport_amo_out);

    PN_TRACE(tf, reg_dport_rdata)
    PN_TRACE(tf, reg_dport_wdata);
    PN_TRACE(tf, reg_dport_adr);
    PN_TRACE(tf, reg_dport_bsel);

    PN_TRACE(tf, signal_bsel_input);
    PN_TRACE(tf, signal_adr_reg_input);
    PN_TRACE(tf, signal_reg_in);
    PN_TRACE(tf, signal_reg_in);
    PN_TRACE(tf, signal_alu_in_a);
    PN_TRACE(tf, signal_alu_in_b);
    PN_TRACE(tf, signal_dport_wdata);

    PN_TRACE(tf, debug_haltrequest_in);
    PN_TRACE(tf, debug_haltrequest_ack_out);

    if(level >= 2)
    {
        alu->pn_trace(tf, level);
        pc->pn_trace(tf, level);
        ir->pn_trace(tf, level);
        regfile->pn_trace(tf, level);
        immgen->pn_trace(tf, level);
        byteselector->pn_trace(tf, level);
        extender->pn_trace(tf, level);
        controller->pn_trace(tf, level);
        datahandler->pn_trace(tf, level);
        csr_master->pn_trace(tf, level);
        csr->pn_trace(tf, level);
    }
}

void m_nucleus_ref::init_submodules()
{

    /* ALU */
    alu = sc_new<m_alu>("alu");
    pc = sc_new<m_pc>("pc");
    ir = sc_new<m_ir>("ir");
    immgen = sc_new<m_immgen>("immgen");
    regfile = sc_new<m_regfile>("regfile");
    byteselector = sc_new<m_byteselector>("byteselector");
    extender = sc_new<m_extender>("extender");
    controller = sc_new<m_controller>("controller");
    datahandler = sc_new<m_datahandler>("datahandler");
    csr_master = sc_new<m_csr_master>("csr_master");
    csr = sc_new<m_csr>("csr");

    alu->a_in(signal_alu_in_a);
    alu->b_in(signal_alu_in_b);
    alu->funct3_in(signal_funct3);
    alu->force_add_in(signal_c_force_add);
    alu->force_amo_in(signal_c_force_amo);
    alu->funct7_in(signal_funct7);

    alu->y_out(signal_alu_out);
    alu->less_out(signal_s_alu_less);
    alu->lessu_out(signal_s_alu_lessu);
    alu->equal_out(signal_s_alu_equal);
    alu->alu_mode_in(signal_alu_mode);

    /* Program counter */
    pc->clk(clk);
    pc->reset(reset);
    pc->inc_in(signal_c_pc_inc4);
    pc->en_load_in(signal_c_pc_ld_en);
    pc->pc_in(signal_pc_in);
    pc->pc_out(signal_pc_out);
    pc->debug_level_enter_in(signal_s_debug_level_enter);
    pc->debug_level_leave_in(signal_c_debug_level_leave);
    pc->csr_dpc_in(signal_dpc);

    /* Instruction register */
    ir->clk(clk);
    ir->reset(reset);
    ir->ir_in(iport_rdata_in);
    ir->ir_out(signal_ir_out);
    ir->en_load_in(signal_c_ir_ld_en);

    /* Register file */
    regfile->clk(clk);
    regfile->reset(reset);
    regfile->data_in(signal_reg_in);
    regfile->select_in(signal_rd);
    regfile->rs1_select_in(signal_rs1);
    regfile->rs2_select_in(signal_rs2);
    regfile->rs1_out(signal_reg_out_rs1);
    regfile->rs2_out(signal_reg_out_rs2);
    regfile->en_load_in(signal_c_reg_ld_en);

    /* Immediate generator */
    immgen->data_in(signal_ir_out);
    immgen->imm_out(signal_immgen_out);

    /* Byteselector */
    byteselector->adr_in(signal_bsel_input);
    byteselector->funct3_in(signal_funct3);
    byteselector->bsel_out(signal_bsel_out);
    byteselector->invalid_out(signal_bsel_invalid);

    /* Extender */
    extender->data_in(reg_dport_rdata);
    extender->bsel_in(reg_dport_bsel); // byteselector value from load during read
    extender->funct3_in(signal_funct3);
    extender->extend_out(signal_extend_out);

    /* Datahandler */
    datahandler->bsel_in(signal_bsel_out); // bsel from immediate address calculation. (Datahandler does not get the bsel from register but immediate to ensure wdata is valid on strobe.)
    datahandler->data_in(signal_reg_out_rs2);
    datahandler->data_out(signal_datahandler_out);

    /* Controller ------------------------------------------------------------*/
    controller->clk(clk);
    controller->reset(reset);
    /* --------------- Status signals --------------- */
    controller->s_instruction_in(signal_ir_out);
    controller->s_alu_less_in(signal_s_alu_less);
    controller->s_alu_lessu_in(signal_s_alu_lessu);
    controller->s_alu_equal_in(signal_s_alu_equal);
    controller->s_dport_ack_in(dport_ack_in);
    controller->s_iport_ack_in(iport_ack_in);

    /* Debug */
    controller->s_debug_haltrequest_in(debug_haltrequest_in);
    controller->s_debug_step_in(signal_s_debug_step);

    /* --------------- Control signals --------------- */
    /* IPort/DPort*/
    controller->c_iport_stb_out(signal_c_iport_stb);
    controller->c_dport_stb_out(signal_c_dport_stb);
    controller->c_dport_we_out(signal_c_dport_we);
    controller->c_dport_lrsc_out(signal_c_dport_lrsc);
    controller->c_dport_amo_out(signal_c_dport_amo);

    controller->c_ld_en_rdata_out(signal_c_ld_en_rdata);
    controller->c_ld_en_wdata_out(signal_c_ld_en_wdata);
    controller->c_ld_en_adr_bsel_out(signal_c_ld_en_adr_bsel);

    /* Control flow signals */
    controller->c_ir_ld_en_out(signal_c_ir_ld_en);
    controller->c_pc_ld_en_out(signal_c_pc_ld_en);
    controller->c_pc_inc4_out(signal_c_pc_inc4);
    controller->c_force_add_out(signal_c_force_add);
    controller->c_force_amo_out(signal_c_force_amo);
    controller->c_alu_imm_out(signal_c_alu_imm);
    controller->c_alu_pc_out(signal_c_alu_pc);
    controller->c_alu_rdata_reg_out(signal_c_alu_rdata_reg);
    controller->c_rs1_adr_bsel_out(signal_c_rs1_adr_bsel);
    controller->c_alu_out_to_wdata_out(signal_c_alu_out_to_wdata);

    controller->c_alu_mode_out(signal_alu_mode);

    controller->c_reg_ld_en_out(signal_c_reg_ld_en);
    controller->c_reg_ldpc_out(signal_c_reg_ldpc);
    controller->c_reg_ldmem_out(signal_c_reg_ldmem);
    controller->c_reg_ldalu_out(signal_c_reg_ldalu);
    controller->c_reg_ldimm_out(signal_c_reg_ldimm);
    controller->c_reg_ldcsr_out(signal_c_reg_ldcsr);

    /* Debug */
    controller->c_debug_haltrequest_ack_out(debug_haltrequest_ack_out);
    controller->c_debug_level_enter_ebreak_out(signal_c_debug_level_enter_ebreak);
    controller->c_debug_level_enter_haltrequest_out(signal_c_debug_level_enter_haltrequest);
    controller->c_debug_level_enter_step_out(signal_c_debug_level_enter_step);
    controller->c_debug_level_leave_out(signal_c_debug_level_leave);

    /* Csr */
    controller->c_csr_bus_adr_out(signal_csr_bus_adr);
    controller->c_csr_bus_read_en_out(signal_csr_bus_read_en);
    controller->c_csr_bus_write_en_out(signal_csr_bus_write_en);
    controller->c_csr_imm_en_out(signal_c_csr_imm_en);
    controller->c_csr_imm_out(signal_c_csr_imm);
    controller->c_csr_write_mode_out(signal_c_csr_write_mode);
    controller->c_csr_bus_wdata_out(signal_controller_csr_bus_wdata);

    /*------------------------------------------------------------------------*/

    /* Csr master */
    csr_master->csr_bus_rdata_in(signal_csr_bus_rdata);
    csr_master->source_reg_in(signal_reg_out_rs1);
    csr_master->imm_en_in(signal_c_csr_imm_en);
    csr_master->imm_in(signal_c_csr_imm);
    csr_master->write_mode_in(signal_c_csr_write_mode);
    csr_master->csr_bus_wdata_out(signal_csr_master_wdata);

    /* Csr */
    csr->clk(clk);
    csr->reset(reset);

    csr->csr_bus_wdata_in(signal_csr_bus_wdata);
    csr->csr_bus_rdata_out(signal_csr_bus_rdata);

    csr->pc_in(signal_pc_out);
    csr->debug_level_enter_ebreak_in(signal_c_debug_level_enter_ebreak);
    csr->debug_level_enter_haltrequest_in(signal_c_debug_level_enter_haltrequest);
    csr->debug_level_enter_step_in(signal_c_debug_level_enter_step);
    csr->debug_level_leave_in(signal_c_debug_level_leave);

    csr->dpc_out(signal_dpc);
    csr->debug_step_out(signal_s_debug_step);
    csr->debug_level_enter_out(signal_s_debug_level_enter);

    /*Interrupt */
    controller->c_csr_mret_out(signal_controller_mret);
    csr->mret_in(signal_controller_mret);
    pc->mret_in(signal_controller_mret);
    controller->c_csr_interrupt_out(signal_controller_interrupt);
    csr->interrupt_in(signal_controller_interrupt);
    controller->s_interrupt_pending_in(signal_interrupt_pending);
    csr->interrupt_pending_out(signal_interrupt_pending);

    csr->mepc_out(signal_mepc);
    pc->csr_mepc_in(signal_mepc);

    csr->mie_msie_out(signal_mie_msie);
    csr->mstatus_mie_out(signal_mstatus_mie);
    csr->mip_msip_out(signal_mip_msip);
    csr->mip_mtip_out(signal_mip_mtip);
    csr->mip_meip_out(signal_mip_meip);
    csr->mie_mtie_out(signal_mie_mtie);
    csr->mie_meie_out(signal_mie_meie);

    csr->mtvec_trap_address_out(signal_mtvec_trap_address);

    controller->c_trap_handler_enter_out(signal_trap_handler);
    pc->trap_handler_enter_in(signal_trap_handler);

    pc->csr_mtvec_in(signal_mtvec_trap_address); // Connect MTVEC register to PC via public signal

    controller->trap_handler_enter_in(signal_interrupt_pending);

    // Connect controller to CSR interrupt status signals
    controller->s_mip_msip_in(signal_mip_msip);
    controller->s_mie_msie_in(signal_mie_msie);
    controller->s_mip_mtip_in(signal_mip_mtip);
    controller->s_mie_mtie_in(signal_mie_mtie);
    controller->s_mip_meip_in(signal_mip_meip);
    controller->s_mie_meie_in(signal_mie_meie);
    controller->s_mstatus_mie_in(signal_mstatus_mie); // Connect the missing MIE status signal

    // Connect interrupt signals
    csr->msip_in(msip_in);
    csr->mtip_in(mtip_in);
    csr->meip_in(meip_in);

    /* Connect controller to CSR */
    csr->csr_bus_adr_in(signal_csr_bus_adr);
    csr->csr_bus_read_en_in(signal_csr_bus_read_en);
    csr->csr_bus_write_en_in(signal_csr_bus_write_en);

    csr->write_mode_in(signal_c_csr_write_mode);
}

void m_nucleus_ref::proc_cmb_nucleus()
{
    /* Nucleus outputs */
    iport_adr_out = reg_iport_adr.read();
    iport_bsel_out = reg_iport_bsel.read();
    dport_bsel_out = reg_dport_bsel.read();
    dport_adr_out = reg_dport_adr.read();
    dport_wdata_out = reg_dport_wdata.read();
    dport_stb_out = reg_dport_stb.read();
    dport_we_out = reg_dport_we.read();
    dport_lrsc_out = reg_dport_lrsc.read();
    dport_amo_out = reg_dport_amo.read();
    iport_stb_out = reg_iport_stb.read();

    /* Program counter signal assignments */
    signal_pc_in = signal_alu_out.read();

    /* IR signal assignments */
    signal_opcode = signal_ir_out.read().range(6, 2);   // truncated opcode block
    signal_rd = signal_ir_out.read().range(11, 7);      // destination register
    signal_funct3 = signal_ir_out.read().range(14, 12); // funct3 block
    signal_rs1 = signal_ir_out.read().range(19, 15);    // source register 1
    signal_rs2 = signal_ir_out.read().range(24, 20);    // source register 2
    signal_funct7 = signal_ir_out.read().range(31, 25); // funct7 block

    /* Byteselector and adr signals */
    if(signal_c_rs1_adr_bsel.read() == 0x1)
    {
        signal_bsel_input = signal_reg_out_rs1.read().range(1, 0);
        signal_adr_reg_input = (signal_reg_out_rs1.read().range(31, 2), sc_uint<2>(0x0));
    }
    else
    {
        signal_bsel_input = signal_alu_out.read().range(1, 0);
        signal_adr_reg_input = (signal_alu_out.read().range(31, 2), sc_uint<2>(0x0));
    }

    /* Regfile input selection */
    signal_reg_in = 0x0;

    if(signal_c_reg_ldpc == 0x1)
    {
        signal_reg_in = signal_pc_out.read() + 0x4;
    }
    else if(signal_c_reg_ldmem == 0x1)
    {
        signal_reg_in = signal_extend_out.read();
    }
    else if(signal_c_reg_ldimm == 0x1)
    {
        signal_reg_in = signal_immgen_out.read();
    }
    else if(signal_c_reg_ldalu == 0x1)
    {
        signal_reg_in = signal_alu_out.read();
    }
    else if(signal_c_reg_ldcsr == 0x1)
    {
        signal_reg_in = signal_csr_bus_rdata.read();
    }
    else
    {
        // PN_WARNING("Invalid regfile input selection");
    }

    /* ALU control signals */
    if(signal_c_alu_pc == 0x1) // select PC as operand A
    {
        signal_alu_in_a = signal_pc_out;
    }
    else if(signal_c_alu_rdata_reg == 0x1)
    {
        signal_alu_in_a = reg_dport_rdata;
    }
    else
    {
        signal_alu_in_a = signal_reg_out_rs1; // select regfile output 0 as operand A
    }

    if(signal_c_alu_imm == 0x1)
    {
        signal_alu_in_b = signal_immgen_out; // select immgen output as operand B
    }
    else
    {
        signal_alu_in_b = signal_reg_out_rs2; // select regfile output 1 as operand B
    }

    if(signal_c_alu_out_to_wdata == 0x1)
    {
        signal_dport_wdata = signal_alu_out;
    }
    else
    {
        signal_dport_wdata = signal_datahandler_out;
    }
}

void m_nucleus_ref::proc_clk_nucleus()
{
    // Reset section
    // Reset output registers when reset signal is high
    reg_iport_stb = 0x0;
    reg_iport_adr = 0x0;
    reg_iport_bsel = 0x0;
    reg_dport_stb = 0x0;
    reg_dport_we = 0x0;
    reg_dport_lrsc = 0x0;
    reg_dport_amo = 0x0;
    reg_dport_adr = 0x0;
    reg_dport_wdata = 0x0;
    reg_dport_bsel = 0x0;
    reg_dport_rdata = 0x0;
    while(true)
    {
        wait();

        // Update output registers
        reg_iport_stb = signal_c_iport_stb.read();
        reg_iport_adr = signal_pc_out.read();
        reg_iport_bsel = 0xF;
        reg_dport_stb = signal_c_dport_stb.read();
        reg_dport_we = signal_c_dport_we.read();
        reg_dport_lrsc = signal_c_dport_lrsc.read();
        reg_dport_amo = signal_c_dport_amo.read();

        // only holding state when needed for multi-op instructions
        if(signal_c_ld_en_rdata.read() == 0x1)
        {
            reg_dport_rdata = dport_rdata_in.read();
        }
        /* conditionally setting adr&wdata to allow prefilling. */
        if(signal_c_ld_en_adr_bsel.read() == 0x1)
        {
            reg_dport_adr = signal_adr_reg_input.read();
            reg_dport_bsel = signal_bsel_out.read();
        }
        if(signal_c_ld_en_wdata.read() == 0x1)
        {
            reg_dport_wdata = signal_dport_wdata.read();
        }
    }
}

sc_uint<6> m_nucleus_ref::nucleus_get_controller_state()
{
    return controller->get_state_signal();
}

void m_nucleus_ref::proc_cmb_csr_bus_wdata_mux()
{
    if(signal_csr_bus_adr.read() == 0x342 && signal_csr_bus_write_en.read())
    {
        // Use controller's value for MCAUSE (0x342)
        signal_csr_bus_wdata.write(signal_controller_csr_bus_wdata.read());
    }
    else
    {
        // Default to CSR master's value
        signal_csr_bus_wdata.write(signal_csr_master_wdata.read());
    }
}


bool m_nucleus_ref::state_is_not_halt () {
    return nucleus_get_controller_state() != STATE_HALT;
}
