/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
                     2025 Niklas Sirch  <niklas.sirch1@tha.de>
                          Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
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
 * @par Input ports:
 * | Port name   | Width in bits | Description                                                                                   |
 * |-------------|--------------|-----------------------------------------------------------------------------------------------|
 * | a           | 32           | Operand A.                                                                                   |
 * | b           | 32           | Operand B.                                                                                   |
 * | funct3      | 3            | 3-bit control signal that selects the operation to be performed.<br>Derived from IR[14:12].     |
 * | funct7      | 7            | Decides between ADD/SUB, SRA/SRL operations.<br>Derived from IR[31:25].                         |
 * | force_add   | 1            | If set, forces the ALU to add the operands.                                                  |
 * | force_amo   | 1            | If set, forces the ALU to select the operation from funct5.<br>funct5(IR[31:27]) is in funct7.  |
 *
 * @par Output ports:
 * | Port name | Width in bits | Description |
 * |-----------|---------------|-------------|
 * | y         | 32            | Result of the logical and arithmetic operation performed on<br>operands A and B. |
 * | equal     | 1             | Status signal, 1 if a == b, else 0. |
 * | less      | 1             | Status signal, 1 if a < b, else 0. |
 * | lessu     | 1             | Status signal, 1 if a < b, else 0 (unsigned). |
 *
 * @par Operation table:
 * Given the input signals funct3 and funct7, the ALU performs the following operations on the operands A and B.
 * | Operation | Shorthand         | funct3 | funct7 | Description                                 |
 * |-----------|-------------------|--------|--------|---------------------------------------------|
 * | ADD       | `y = a + b`       | 0x0    | 0x0    | Regular addition                            |
 * | SUB       | `y = a - b`       | 0x0    | 0x20   | Two's complement of B is added to A.<br>R-Type only. |
 * | AND       | `y = a & b`       | 0x7    | 0x0    | Bitwise AND                                 |
 * | OR        | `y = a OR b`       | 0x6    | 0x0    | Bitwise OR                                 |
 * | XOR       | `y = a ^ b`       | 0x4    | 0x0    | Bitwise XOR                                 |
 * | SLL       | `y = a << b`      | 0x1    | 0x0    | Shift left logical                          |
 * | SRL       | `y = a >> b`      | 0x5    | 0x0    | Shift right logical                         |
 * | SRA       | `y = a >> b`      | 0x5    | 0x20   | Shift right arithmetic (sign extends)       |
 * | SLT       | `y = a < b ? 1,0` | 0x2    | 0x0    | Set less than                               |
 * | SLTU      | `y = a < b ? 1,0` | 0x3    | 0x0    | Set less than unsigned (zero extends)       |
 *
 * @par Notes:
 * - The funct3_in signal is not mapped uniquely.
 *   For example, the ADD and SUB operation share the same funct3 value and are distinguished by the funct7 signal.
 *   The same goes for the SRL and SRA operations.
 * - The ALU is indirectly used to perform branch and jump instructions as well as the LUI and AUIPC instructions.
 * - The ALU is also used to perform AMO operations, that are determined by the funct5 (inside funct5) field of the instruction.
 * - For AMOs the ALU also performs the unique swap, min(u) and max(u) operations.
 */

#ifndef __ALU_H__
#define __ALU_H__

#include <systemc.h>
#include <piconut.h>

#include <stdint.h>

#include "typedef.h"


SC_MODULE(m_alu)
{
public:
    sc_in<sc_uint<32>> PN_NAME(a_in);
    sc_in<sc_uint<32>> PN_NAME(b_in);
    sc_in<sc_uint<3>> PN_NAME(funct3_in);
    sc_in<sc_uint<7>> PN_NAME(funct7_in);
    sc_in<sc_uint<3>> PN_NAME(alu_mode_in);
    sc_in<bool> PN_NAME(force_add_in);
    sc_in<bool> PN_NAME(force_amo_in);

    sc_out<sc_uint<32>> PN_NAME(y_out);
    // Branching signals
    sc_out<bool> PN_NAME(equal_out);
    sc_out<bool> PN_NAME(less_out);
    sc_out<bool> PN_NAME(lessu_out);

    SC_CTOR(m_alu)
    {
        SC_METHOD(proc_cmb_alu_rv32i);
        sensitive << a_in << b_in << funct3_in << funct7_in << alu_mode_in;

        SC_METHOD(proc_cmb_alu_output);
        sensitive << force_add_in << force_amo_in << funct3_in << funct7_in << alu_mode_in << add_sub_result << and_result << or_result
                  << xor_result << slt_result << sltu_result << sll_result << srl_sra_result
                  << max_result << min_result << maxu_result << minu_result
                  << a_in << b_in;
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_alu_rv32i();
    void proc_cmb_alu_output();

protected:
    sc_signal<sc_uint<32>> shamt;
    sc_signal<sc_uint<32>> add_sub_result; // add or sub depending on funct7
    sc_signal<sc_uint<32>> and_result;
    sc_signal<sc_uint<32>> or_result;
    sc_signal<sc_uint<32>> xor_result;
    sc_signal<sc_uint<32>> slt_result;
    sc_signal<sc_uint<32>> sltu_result;
    sc_signal<sc_uint<32>> sll_result;
    sc_signal<sc_uint<32>> srl_sra_result;
    /* Results for min/max operators for AMOs */
    sc_signal<sc_uint<32>> max_result;
    sc_signal<sc_uint<32>> min_result;
    sc_signal<sc_uint<32>> maxu_result;
    sc_signal<sc_uint<32>> minu_result;
};

#endif //__ALU_H__
