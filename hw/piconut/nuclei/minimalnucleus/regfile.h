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

 /** regfile (Register File)
 *
 * The regfile module represents the CPUs internal registers.
 *
 * Input ports:
 *  - data_in<32>:        The value to be loaded into the register selected by select_in.
 *  - en_load_in<1>:      If en_load_in = 1, the value on data_in is loaded into the register selected by select_in,
 *                        else the register is not changed.
 *  - select_in<5>:       The register to be loaded with the value on data_in.
 *  - rs1_select_in<5>:   Selects which registers value is to be output at the rs1_out port.
 *  - rs2_select_in<5>:   Selects which registers value is to be output at the rs2_out port.
 *
 *
 * Output ports:
 *  - rs1_out<32>: The value of the register selected by rs1_select_in.
 *  - rs2_out<32>: The value of the register selected by rs2_select_in.
 *
 * Internal signals:
 *  - regfile<32>:  An array of 32 32-bit registers.
 *
 * Note: The register x0 at index 0 always holds the value 0.
 *
 * The outputs of this register file are used for arithmetic operations in the ALU or
 * to store the result of an operation in memory.
 */



#ifndef __REGFILE_H__
#define __REGFILE_H__

#include <systemc.h>
#include <stdint.h>
#include <base.h>
#include <piconut-config.h>

SC_MODULE(m_regfile)
{
public:
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    sc_in<sc_uint<32>> PN_NAME(data_in);
    sc_in<sc_uint<5>> PN_NAME(select_in);
    sc_in<sc_uint<5>> PN_NAME(rs1_select_in);
    sc_in<sc_uint<5>> PN_NAME(rs2_select_in);
    sc_in<bool> PN_NAME(en_load_in);

    sc_out<sc_uint<32>> PN_NAME(rs1_out);
    sc_out<sc_uint<32>> PN_NAME(rs2_out);

    /* Constructor... */
    SC_CTOR(m_regfile)
    {
        SC_CTHREAD(proc_clk_regfile, clk.pos());
        reset_signal_is(reset, true);
        SC_METHOD(proc_cmb_regfile);
        sensitive << rs1_select_in << rs2_select_in;
        for(size_t i = 0; i < 32; i++)
        {
            sensitive << regfile[i];
        }
    }

    void Trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_regfile();
    void proc_clk_regfile();

protected:
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(regfile, CFG_REGFILE_SIZE);
};

#endif //__REGFILE_H__