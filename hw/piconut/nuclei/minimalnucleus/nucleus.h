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

 /** Nucleus
 *
 * The Nucleus module combines the submodules (ALU, regfile, byteselector, controller, datahandler, extender,
 * immgen, ir, pc) to form a complete RISC-V core.
 *
 * Input ports:
 *  - dport_rdata_in<32>:  Data read from the DPort interface.
 *  - dport_ack_in<1>:     Acknowledge signal from the DPort interface.
 *
 *  - iport_rdata_in<32>:  Data read from the IPort interface.
 *  - iport_ack_in<1>:     Acknowledge signal from the IPort interface.
 *
 *  - debug_haltrequest_in<1>: Halt request for debugging.
 *
 * Output ports:
 *  - iport_stb_out<1>:    Strobe signal to the IPort interface.
 *  - iport_bsel_out<4>:   Byte select signal to the IPort interface. Permanently set to '1111' .
 *  - iport_adr_out<32>:   Address signal to the IPort interface. Assigned to the output of the program counter.
 *
 *  - dport_stb_out<1>:    Strobe signal to the DPort interface.
 *  - dport_wdata_out<32>: Data to be written to memory via the DPort interface. Assigned to the output of the datahandler.
 *  - dport_we_out<1>:     Write enable signal to the DPort interface.
 *  - dport_adr_out<32>:   Address signal to the DPort interface. Assigned to alu_out[31:2].
 *  - dport_bsel_out<4>:   Byte select signal to the DPort interface. Assigned to the output of the byteselector.
 *
 *  - debug_haltrequest_ack_out<1>: Acknowledge halt request for debugging.
 *
 * Beyond simply connecting submodule ports, this top level module also contains some RTL logic.
 *  - The regfile has multiple possible input sources. The source is selected by the controller through status signals.
 *    These signals are checked in this module and the regfile input is set accordingly.
 *  - Should the program counter be selected as the regfile input, '0x4' is added to it. This is done to ensure that
 *    the JALR instruction can be executed within one clock cycle.
 *  - The ALU operands A and B also have multiple possible sources. The control signals c_alu_pc_out and c_alu_imm_out are used
 *    to distinguish. The default sources are the regfile outputs rs1_out and rs2_out for operands A and B respectively.
 *  - The current instruction (IR) is deconstructed into its components opcode, rd, rs1, rs2 and funct3 for use by the
 *    individual submodules.
 *  - The byteselector is assigned the lower two bits of the ALU output as its input.
 *
 * Note: The outputs of this module are behind registers and therefore delayed by one clock cycle.
 */

#ifndef __NUCLEUS_H__
#define __NUCLEUS_H__

#include <systemc.h>
#include <stdint.h>
#include <base.h>
#include <piconut-config.h>
#include "elab_alloc.h"

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

SC_MODULE(m_nucleus)
{
public:
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    /* ----------------- Input signals -----------------  */
    /* IPort */
    sc_in<sc_uint<32>> PN_NAME(iport_rdata_in);
    sc_in<bool> PN_NAME(iport_ack_in);

    /* DPort */
    sc_in<sc_uint<32>> PN_NAME(dport_rdata_in);
    sc_in<bool> PN_NAME(dport_ack_in);

    /* Debug */
    sc_in<bool> PN_NAME(debug_haltrequest_in);

    /* ----------------- Output signals -----------------  */
    /* IPort */
    sc_out<sc_uint<32>> PN_NAME(iport_adr_out);
    sc_out<sc_uint<4>> PN_NAME(iport_bsel_out);
    sc_out<bool> PN_NAME(iport_stb_out);

    /* DPort */
    sc_out<sc_uint<32>> PN_NAME(dport_adr_out);
    sc_out<sc_uint<32>> PN_NAME(dport_wdata_out);
    sc_out<sc_uint<4>> PN_NAME(dport_bsel_out);
    sc_out<bool> PN_NAME(dport_we_out);
    sc_out<bool> PN_NAME(dport_stb_out);

    /* Debug */
    sc_out<bool> PN_NAME(debug_haltrequest_ack_out);

    /* Constructor... */
    SC_CTOR(m_nucleus)
    {
        init_submodules();
        SC_CTHREAD(proc_clk_nucleus, clk.pos());
        reset_signal_is(reset, true);

        SC_METHOD(proc_cmb_nucleus);
        sensitive << dport_ack_in << iport_ack_in << dport_rdata_in << iport_rdata_in
                  << signal_alu_out << signal_ir_out << signal_pc_out << signal_bsel_out
                  << signal_extend_out << signal_immgen_out << signal_reg_out_rs1 << signal_reg_out_rs2
                  << signal_c_reg_ldpc << signal_c_reg_ldmem << signal_c_reg_ldimm << signal_c_reg_ldalu
                  << signal_c_reg_ldcsr << signal_c_alu_imm << signal_c_alu_pc << reg_dport_adr
                  << reg_dport_bsel << reg_dport_stb << reg_dport_we << reg_dport_wdata << reg_iport_adr
                  << reg_iport_bsel << reg_iport_stb << dport_wdata_out << signal_datahandler_out << signal_csr_bus_rdata;
    }

    void Trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_nucleus();
    void proc_clk_nucleus();
    sc_uint<6> nucleus_get_controller_state();

    // Modules
    m_alu* alu;
    m_pc* pc;
    m_ir* ir;
    m_regfile* regfile;
    m_byteselector* byteselector;
    m_extender* extender;
    m_immgen* immgen;
    m_controller* controller;
    m_datahandler* datahandler;
    m_csr_master* csr_master;
    m_csr* csr;

protected:
    void init_submodules();

    /* Signals */

    /* Ports */
    sc_signal<sc_uint<32>> PN_NAME(signal_dport_rdata);

    /* ALU */
    sc_signal<sc_uint<32>> PN_NAME(signal_alu_out);
    sc_signal<sc_uint<32>> PN_NAME(signal_alu_in_a);
    sc_signal<sc_uint<32>> PN_NAME(signal_alu_in_b);
    sc_signal<sc_uint<3>> PN_NAME(signal_alu_mode);
    sc_signal<bool> PN_NAME(signal_c_force_add);
    sc_signal<bool> PN_NAME(signal_s_alu_equal);
    sc_signal<bool> PN_NAME(signal_s_alu_less);
    sc_signal<bool> PN_NAME(signal_s_alu_lessu);
    sc_signal<bool> PN_NAME(signal_c_alu_imm);
    sc_signal<bool> PN_NAME(signal_c_alu_pc);

    /* Program counter signals */
    sc_signal<sc_uint<32>> PN_NAME(signal_pc_in);
    sc_signal<sc_uint<32>> PN_NAME(signal_pc_out);
    sc_signal<bool> PN_NAME(signal_pc_load_last);

    sc_signal<bool> PN_NAME(signal_c_pc_inc4);
    sc_signal<bool> PN_NAME(signal_c_pc_ld_en);

    /* Regfile output signals */
    sc_signal<sc_uint<32>> PN_NAME(signal_reg_out_rs1);
    sc_signal<sc_uint<32>> PN_NAME(signal_reg_out_rs2);

    /* IR signals */
    sc_signal<sc_uint<32>> PN_NAME(signal_ir_out);
    sc_signal<sc_uint<5>> PN_NAME(signal_opcode);
    sc_signal<sc_uint<3>> PN_NAME(signal_funct3);
    sc_signal<sc_uint<7>> PN_NAME(signal_funct7);
    sc_signal<sc_uint<5>> PN_NAME(signal_rd);
    sc_signal<sc_uint<5>> PN_NAME(signal_rs1);
    sc_signal<sc_uint<5>> PN_NAME(signal_rs2);

    sc_signal<bool> PN_NAME(signal_c_ir_ld_en);

    /* Regfile */
    sc_signal<sc_uint<32>> PN_NAME(signal_reg_in);
    sc_signal<bool> PN_NAME(signal_c_reg_ld_en);

    // sc_signal<sc_uint<4>> PN_NAME(signal_c_sel_reg_in);

    sc_signal<bool> PN_NAME(signal_c_reg_ldpc);
    sc_signal<bool> PN_NAME(signal_c_reg_ldmem);
    sc_signal<bool> PN_NAME(signal_c_reg_ldalu);
    sc_signal<bool> PN_NAME(signal_c_reg_ldimm);
    sc_signal<bool> PN_NAME(signal_c_reg_ldcsr);

    /* Immediate generator */
    sc_signal<sc_uint<32>> PN_NAME(signal_immgen_out);

    /* Byteselector */
    sc_signal<sc_uint<2>> PN_NAME(signal_alu_out_bsel);
    sc_signal<sc_uint<4>> PN_NAME(signal_bsel_out);
    sc_signal<bool> PN_NAME(signal_bsel_invalid);

    /* Extender */
    sc_signal<sc_uint<32>> PN_NAME(signal_extend_out);

    /* Controller */
    sc_signal<bool> PN_NAME(signal_c_iport_stb);
    sc_signal<bool> PN_NAME(signal_c_dport_stb);
    sc_signal<bool> PN_NAME(signal_c_dport_we);

    /* Data Handler */
    sc_signal<sc_uint<32>> PN_NAME(signal_dport_wdata);
    sc_signal<sc_uint<32>> PN_NAME(signal_datahandler_out);

    /* Csr master */
    sc_signal<bool> PN_NAME(signal_c_csr_imm_en);
    sc_signal<sc_uint<5>> PN_NAME(signal_c_csr_imm);
    sc_signal<sc_uint<2>> PN_NAME(signal_c_csr_write_mode);

    /* Csr-bus */
    sc_signal<bool> PN_NAME(signal_csr_bus_read_en);
    sc_signal<bool> PN_NAME(signal_csr_bus_write_en);
    sc_signal<sc_uint<CFG_CSR_BUS_ADR_WIDTH>> PN_NAME(signal_csr_bus_adr);
    sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(signal_csr_bus_wdata);
    sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(signal_csr_bus_rdata);

    /* Csr */
    sc_signal<bool> PN_NAME(signal_c_debug_level_enter_ebreak);
    sc_signal<bool> PN_NAME(signal_c_debug_level_enter_haltrequest);
    sc_signal<bool> PN_NAME(signal_c_debug_level_enter_step);
    sc_signal<bool> PN_NAME(signal_c_debug_level_leave);
    sc_signal<bool> PN_NAME(signal_s_debug_level_enter);
    sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(signal_dpc);
    sc_signal<bool> PN_NAME(signal_s_debug_step);

    /* Output registers */
    sc_signal<bool> PN_NAME(reg_iport_stb);
    sc_signal<bool> PN_NAME(reg_dport_stb);
    sc_signal<bool> PN_NAME(reg_dport_we);
    sc_signal<sc_uint<32>> PN_NAME(reg_dport_wdata);
    sc_signal<sc_uint<32>> PN_NAME(reg_dport_adr);
    sc_signal<sc_uint<32>> PN_NAME(reg_iport_adr);
    sc_signal<sc_uint<4>> PN_NAME(reg_iport_bsel);
    sc_signal<sc_uint<4>> PN_NAME(reg_dport_bsel);
};

#endif //__NUCLEUS_H__