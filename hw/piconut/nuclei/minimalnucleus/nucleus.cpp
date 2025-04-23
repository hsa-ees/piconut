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

#include "nucleus.h"



void m_nucleus::Trace(sc_trace_file* tf, int level)
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

    PN_TRACE(tf, debug_haltrequest_in);
    PN_TRACE(tf, debug_haltrequest_ack_out);

    if(level >= 2)
    {
        alu->Trace(tf, level);
        pc->Trace(tf, level);
        ir->Trace(tf, level);
        regfile->Trace(tf, level);
        immgen->Trace(tf, level);
        byteselector->Trace(tf, level);
        extender->Trace(tf, level);
        controller->Trace(tf, level);
        datahandler->Trace(tf, level);
        csr_master->Trace(tf, level);
        csr->Trace(tf, level);
    }
}

void m_nucleus::init_submodules()
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
    byteselector->adr_in(signal_alu_out_bsel);
    byteselector->funct3_in(signal_funct3);
    byteselector->bsel_out(signal_bsel_out);
    byteselector->invalid_out(signal_bsel_invalid);

    /* Extender */
    extender->data_in(dport_rdata_in);
    extender->bsel_in(signal_bsel_out);
    extender->funct3_in(signal_funct3);
    extender->extend_out(signal_extend_out);
    extender->clk(clk);
    extender->reset(reset);

    /* Datahandler */
    datahandler->bsel_in(signal_bsel_out);
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

    /* Control flow signals */
    controller->c_ir_ld_en_out(signal_c_ir_ld_en);
    controller->c_pc_ld_en_out(signal_c_pc_ld_en);
    controller->c_pc_inc4_out(signal_c_pc_inc4);
    controller->c_force_add_out(signal_c_force_add);
    controller->c_alu_imm_out(signal_c_alu_imm);
    controller->c_alu_pc_out(signal_c_alu_pc);

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

    /*------------------------------------------------------------------------*/

    /* Csr master */
    csr_master->csr_bus_rdata_in(signal_csr_bus_rdata);
    csr_master->source_reg_in(signal_reg_out_rs1);
    csr_master->imm_en_in(signal_c_csr_imm_en);
    csr_master->imm_in(signal_c_csr_imm);
    csr_master->write_mode_in(signal_c_csr_write_mode);
    csr_master->csr_bus_wdata_out(signal_csr_bus_wdata);

    /* Csr */
    csr->clk(clk);
    csr->reset(reset);

    csr->csr_bus_read_en_in(signal_csr_bus_read_en);
    csr->csr_bus_write_en_in(signal_csr_bus_write_en);
    csr->csr_bus_adr_in(signal_csr_bus_adr);
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
}

void m_nucleus::proc_cmb_nucleus()
{
    /* Nucleus outputs */
    iport_adr_out = reg_iport_adr.read();
    iport_bsel_out = reg_iport_bsel.read();
    dport_bsel_out = reg_dport_bsel.read();
    dport_adr_out = reg_dport_adr.read();
    dport_wdata_out = reg_dport_wdata.read();
    dport_stb_out = reg_dport_stb.read();
    dport_we_out = reg_dport_we.read();
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

    /* Byteselector signals */
    signal_alu_out_bsel = signal_alu_out.read().range(1, 0);

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
}

void m_nucleus::proc_clk_nucleus()
{
    // Reset section
    // Reset output registers when reset signal is high
    reg_iport_stb = 0x0;
    reg_iport_adr = 0x0;
    reg_iport_bsel = 0x0;
    reg_dport_stb = 0x0;
    reg_dport_we = 0x0;
    reg_dport_adr = 0x0;
    reg_dport_wdata = 0x0;
    reg_dport_bsel = 0x0;
    while(true)
    {
        wait();

        // Update output registers
        reg_iport_stb = signal_c_iport_stb.read();
        reg_iport_adr = signal_pc_out.read();
        reg_iport_bsel = 0xF;
        reg_dport_stb = signal_c_dport_stb.read();
        reg_dport_we = signal_c_dport_we.read();
        reg_dport_adr = (signal_alu_out.read().range(31, 2), sc_uint<2>(0x0));
        reg_dport_wdata = signal_datahandler_out.read();
        reg_dport_bsel = signal_bsel_out.read();
    }
}

sc_uint<6> m_nucleus::nucleus_get_controller_state()
{
    return controller->get_state_signal();
}
