/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lorenz Sommer <lorenz.sommer@tha.de>
                     2025 Niklas Sirch  <niklas.sirch1@tha.de>
                          Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
                          Tristan Kundrat <tristan.kundrat@tha.de>
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

/**
 * @fn SC_MODULE(m_alu)
 * @authors Lorenz Sommer, Niklas Sirch, Daniel Sommerfeldt, Tristan Kundrat
 *
 * @brief The ALU (Arithmetic Logic Unit) is a circuit that performs arithmetic and logical operations.
 *
 *
 * @par Input ports:
 * | Port name   | Width in bits | Description                                                                                     |
 * |-------------|---------------|-------------------------------------------------------------------------------------------------|
 * | a           | 32            | Operand A.                                                                                      |
 * | b           | 32            | Operand B.                                                                                      |
 * | funct3      | 3             | 3-bit control signal that selects the operation to be performed.<br>Derived from IR[14:12].     |
 * | funct7      | 7             | Decides between ADD/SUB, SRA/SRL operations.<br>Derived from IR[31:25].                         |
 * | force_add   | 1             | If set, forces the ALU to add the operands.                                                     |
 * | force_amo   | 1             | If set, forces the ALU to select the operation from funct5.<br>funct5(IR[31:27]) is in funct7.  |
 * | alu_mode    | 3             | Selects mode of operation for ALU. Either Idle, Reg-Reg operations or Reg-Imm operations.       |
 * | clk         | 1             | Clock signal for multiplication and division                                                    |
 * | reset       | 1             | Reset signal for multiplication and division                                                    |
 *
 * @par Output ports:
 * | Port name | Width in bits | Description                                                                                                                                                                                                                                                                                                                                      |
 * |-----------|---------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * | y         | 32            | Result of the logical and arithmetic operation performed on<br>operands A and B.                                                                                                                                                                                                                                                                 |
 * | equal     | 1             | Status signal, 1 if a == b, else 0.                                                                                                                                                                                                                                                                                                              |
 * | less      | 1             | Status signal, 1 if a < b, else 0.                                                                                                                                                                                                                                                                                                               |
 * | lessu     | 1             | Status signal, 1 if a < b, else 0 (unsigned).                                                                                                                                                                                                                                                                                                    |
 * | valid     | 1             | Status signal, 1 when calculation finished, else 0.<br>When calculating a number using the multiplication or division module,<br>the valid signal of the ALU corresponds to the valid signal of the calculating module.<br>When calculating any RV32I instruction, the logic will be purely combinatorial and the valid signal will always be 1. |
 *
 * @par Operation table:
 * Given the input signals funct3 and funct7, the ALU performs the following operations on the operands A and B.
 * | Operation | Shorthand                                                                | funct3 | funct7 | Description                                                                               |
 * |-----------|--------------------------------------------------------------------------|--------|--------|-------------------------------------------------------------------------------------------|
 * | ADD       | `y = a + b`                                                              | 0x0    | 0x0    | Regular addition                                                                          |
 * | SUB       | `y = a - b`                                                              | 0x0    | 0x20   | Two's complement of B is added to A.<br>R-Type only.                                      |
 * | AND       | `y = a & b`                                                              | 0x7    | 0x0    | Bitwise AND                                                                               |
 * | OR        | `y = a OR b`                                                             | 0x6    | 0x0    | Bitwise OR                                                                                |
 * | XOR       | `y = a ^ b`                                                              | 0x4    | 0x0    | Bitwise XOR                                                                               |
 * | SLL       | `y = a << b`                                                             | 0x1    | 0x0    | Shift left logical                                                                        |
 * | SRL       | `y = a >> b`                                                             | 0x5    | 0x0    | Shift right logical                                                                       |
 * | SRA       | `y = a >> b`                                                             | 0x5    | 0x20   | Shift right arithmetic (sign extends)                                                     |
 * | SLT       | `y = a < b ? 1,0`                                                        | 0x2    | 0x0    | Set less than                                                                             |
 * | SLTU      | `y = a < b ? 1,0`                                                        | 0x3    | 0x0    | Set less than unsigned (zero extends)                                                     |
 * | MUL       | `y = (int)(a * b)`                                                       | 0x0    | 0x01   | multiplication (lower half)                                                               |
 * | MULH      | `y = ((a * b) >> sizeof(int))`                                           | 0x1    | 0x01   | signed*signed multiplication (upper half)                                                 |
 * | MULHU     | `y = (unsigned int)((a * b) >> sizeof(int))`                             | 0x3    | 0x01   | unsigned*unsigned multiplication (upper half)                                             |
 * | MULHSU    | `y = (int)((a * b) >> sizeof(int))`                                      | 0x2    | 0x01   | signed*unsigned multiplication (upper half)                                               |
 * | DIV       | `y = b == 0 ? -1 : rtz(a / b)`                                           | 0x4    | 0x01   | signed/signed, rounding towards zero (rtz), dividing with 0 gives -1.                     |
 * | DIVU      | `y = b == 0 ? 2^sizeof(int)-1 : rtz(a / b)`                              | 0x5    | 0x01   | unsigned/unsigned, rounding towards zero (rtz), dividing with 0 gives 2^sizeof(int)-1.    |
 * | REM       | `y = b == 0 ? a : a mod b`                                               | 0x6    | 0x01   | signed mod signed, dividing with 0 gives a. a == -2^(sizeof(int) - 1) && b == -1 gives 0. |
 * | REMU      | `y = b == 0 ? a : a mod b`                                               | 0x7    | 0x01   | unsigned mod unsigned, dividing with 0 gives a.                                           |
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

#include "../nucleus_ref_defs.h"

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

    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    sc_out<sc_uint<32>> PN_NAME(y_out);
    // Branching signals
    sc_out<bool> PN_NAME(equal_out);
    sc_out<bool> PN_NAME(less_out);
    sc_out<bool> PN_NAME(lessu_out);
    // finished/valid signal (useful, when calculation takes more than one cycle)
    sc_out<bool> PN_NAME(valid_out);

    SC_CTOR(m_alu)
    {
        SC_METHOD(proc_cmb_alu_rv32im);
        sensitive << a_in << b_in << funct3_in << funct7_in << alu_mode_in
#if PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1
                  << mul_result << mul_valid
#endif
#if PN_CFG_ALU_ENABLE_M_EXTENSION == 1
                  << div_rem_result << div_valid
#endif
#if PN_CFG_ENABLE_ZKNE_ZKND == 1
                  << crypto_valid
#endif
            ;

        SC_METHOD(proc_cmb_alu_output);
        sensitive << force_add_in << force_amo_in << funct3_in << funct7_in
                  << alu_mode_in << add_sub_mul_result << and_remu_result
                  << or_rem_result << xor_div_result << slt_mulhsu_result
                  << sltu_mulhu_result << sll_mulh_result << srl_sra_divu_result
                  << max_result << min_result << maxu_result << minu_result
#if PN_CFG_ENABLE_ZKNE_ZKND == 1
                  << crypto_result
#endif
                  << a_in << b_in;

        init_submodules();
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_alu_rv32im();
    void proc_cmb_alu_output();

    /* Submodules */
#if PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1
    // Change class here, if you want to use a different multiplication module.
    // Also change imports accordingly.
    class m_shift_and_add* multiplication_module;
#endif
#if PN_CFG_ALU_ENABLE_M_EXTENSION == 1
    // Change class here, if you want to use a different division module.
    // Also change imports accordingly.
    class m_shift_and_subtract* division_module;
#endif
#if PN_CFG_ENABLE_ZKNE_ZKND == 1
    class m_scalar_crypto* scalar_crypto;
#endif

protected:
    sc_signal<sc_uint<32>> shamt;
    sc_signal<sc_uint<32>> add_sub_mul_result;
    sc_signal<sc_uint<32>> and_remu_result;
    sc_signal<sc_uint<32>> or_rem_result;
    sc_signal<sc_uint<32>> xor_div_result;
    sc_signal<sc_uint<32>> slt_mulhsu_result;
    sc_signal<sc_uint<32>> sltu_mulhu_result;
    sc_signal<sc_uint<32>> sll_mulh_result;
    sc_signal<sc_uint<32>> srl_sra_divu_result;
    /* Results for min/max operators for AMOs */
    sc_signal<sc_uint<32>> max_result;
    sc_signal<sc_uint<32>> min_result;
    sc_signal<sc_uint<32>> maxu_result;
    sc_signal<sc_uint<32>> minu_result;
    /* Results/signals for M-Extension */
#if PN_CFG_ALU_ENABLE_ZMMUL_EXTENSION == 1
    sc_signal<sc_uint<32>> mul_result; // mul* depending on funct3
    sc_signal<bool> mul_valid;
    sc_signal<bool> mul_start;
#endif
#if PN_CFG_ALU_ENABLE_M_EXTENSION == 1
    sc_signal<sc_uint<32>> div_rem_result; // div*/rem* depending on funct3
    sc_signal<bool> div_valid;
    sc_signal<bool> div_start;
#endif
#if PN_CFG_ENABLE_ZKNE_ZKND == 1
    sc_signal<sc_uint<32>> crypto_result;
    sc_signal<bool> crypto_valid;
    sc_signal<bool> crypto_start;
#endif
    /* Methods */
    void init_submodules();
};

#endif //__ALU_H__
