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

/** ALU (Arithmetic Logic Unit)
 * The ALU (Arithmetic Logic Unit) is a_in circuit that performs arithmetic and logical operations.
 * It is implemented implemented using pure combinatorics.
 *
 *
 * Input ports:
 *  - a_in<32>:             32-bit input a_in.
 *  - b_in<32>:             32-bit input b_in.
 *  - funct3_in<3>:         3-bit control signal that selects the operation to be performed. Derived from IR[14:12].
 *  - funct7_in<7>:         func7 block of the current instruction word
 *  - force_add_in<1>:      If set, forces the ALU to perform an addition operation.
 *
 * Output ports:
 *  - y_out<32>:            32-bit output y_out.
 *  - equal_out<1>:         1 if a_in == b_in, else 0.
 *  - less_out<1>:          1 if a_in < b_in, else 0.
 *  - lessu_out<1>:         1 if a_in < b_in, else 0 (unsigned).
 *
 * Note: funct7 is a_in term defined in the RISC-V specification and is originally a_in 7-bit signal derived from IR[31:25].
 *       Because the only relevant values of this signal are 0x00 and 0x20,
 *       it is sufficient to reduce this signal to its 6th bit.
 *       This assertion holds true for I-Type instructions. Register-immediate type instructions
 *       make the same SRA/SRL distinction. This information is formally derived from
 *       the bits imm[11:5] in the I-Type instruction format. This value is then used to generate the concrete
 *       immediate value associated with the instruction. Despite this, the funct7_flag_in bit is still
 *       present in the same location IR[30] and no extra logic is needed to differentiate between
 *       R-Type or I-Type instruction within the ALU.
 *
 * Given the control signals, the ALU can perform the following operations:
 *  - ADD:   y_out = a_in + b_in         - funct3 = 0x0   - funct7_flag = 0    - Regular addition
 *  - SUB:   y_out = a_in - b_in         - funct3 = 0x0   - funct7_flag = 1    - Two's complement of b_in is added to a_in
 *  - AND:   y_out = a_in & b_in         - funct3 = 0x7   - funct7_flag = 0    - Bitwise AND
 *  - OR:    y_out = a_in | b_in         - funct3 = 0x6   - funct7_flag = 0    - Bitwise OR
 *  - XOR:   y_out = a_in ^ b_in         - funct3 = 0x4   - funct7_flag = 0    - Bitwise XOR
 *  - SLL:   y_out = a_in << b_in        - funct3 = 0x1   - funct7_flag = 0    - Shift left logical
 *  - SRL:   y_out = a_in >> b_in        - funct3 = 0x5   - funct7_flag = 0    - Shift right logical
 *  - SRA:   y_out = a_in >> b_in        - funct3 = 0x5   - funct7_flag = 1    - Shift right arithmetic (sign extends)
 *  - SLT:   y_out = a_in < b_in ? 1,0   - funct3 = 0x2   - funct7_flag = 0    - Set less_out than
 *  - SLTU:  y_out = a_in < b_in ? 1,0   - funct3 = 0x3   - funct7_flag = 0    - Set less_out than unsigned (zero extends)
 *
 * Note: The funct3_in signal is not mapped uniquely.
 *       For example, the ADD and SUB operation share the same funct3 value and are distinguished by the funct7_flag signal.
 *       The same goes for the SRL and SRA operations.
 *
 * Note: The ALU is indirectly used to perform branch and jump instructions as well as the LUI and AUIPC instructions.
 *
 */


#ifndef __ALU_H__
#define __ALU_H__

#include <systemc.h>
#include <stdint.h>
#include <base.h>
#include <typedef.h>

SC_MODULE(m_alu)
{
public:
    sc_in<sc_uint<32>> PN_NAME(a_in);
    sc_in<sc_uint<32>> PN_NAME(b_in);
    sc_in<sc_uint<3>> PN_NAME(funct3_in);
    sc_in<sc_uint<7>> PN_NAME(funct7_in);
    sc_in<sc_uint<3>> PN_NAME(alu_mode_in);
    sc_in<bool> PN_NAME(force_add_in);

    sc_out<sc_uint<32>> PN_NAME(y_out);
    sc_out<bool> PN_NAME(equal_out);
    sc_out<bool> PN_NAME(less_out);
    sc_out<bool> PN_NAME(lessu_out);

    SC_CTOR(m_alu)
    {
        SC_METHOD(proc_cmb_alu_rv32i);
        sensitive << a_in << b_in << funct3_in << funct7_in << alu_mode_in;

        SC_METHOD(proc_cmb_alu_output);
        sensitive << force_add_in << funct3_in << funct7_in << alu_mode_in << add_sub_result << and_result << or_result
                  << xor_result << slt_result << sltu_result << sll_result << srl_sra_result << a_in << b_in;
    }

    void Trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_alu_rv32i();
    void proc_cmb_alu_output();

protected:
    sc_signal<sc_uint<32>> shamt;
    sc_signal<sc_uint<32>> add_sub_result;
    sc_signal<sc_uint<32>> and_result;
    sc_signal<sc_uint<32>> or_result;
    sc_signal<sc_uint<32>> xor_result;
    sc_signal<sc_uint<32>> slt_result;
    sc_signal<sc_uint<32>> sltu_result;
    sc_signal<sc_uint<32>> sll_result;
    sc_signal<sc_uint<32>> srl_sra_result;
};

#endif //__ALU_H__