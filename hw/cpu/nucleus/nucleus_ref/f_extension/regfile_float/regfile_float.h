/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2026 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
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

#ifndef __REGFILE_FLOAT_H__
#define __REGFILE_FLOAT_H__

#include <systemc.h>
#include <piconut.h>

#include <stdint.h>


#ifndef PN_CFG_CPU_REGFILE_FLOAT_SIZE
#define PN_CFG_CPU_REGFILE_FLOAT_SIZE 32U
#endif

/**
 * @fn SC_MODULE(m_regfile_float)
 * @brief Register file for floating-point numbers.
 * 
 * These floating-point registers differ slightly from the integer registers.
 * Floating-Point register f0 can be any value, as opposed to integer register a0 being forced to 0.
 * 
 * @par Ports:
 * @param[in] clk Clock of the module.
 * @param[in] reset Reset of the module.
 * @param[in] data_in 32-bit floating-point data to be written into the destination register.
 * @param[in] select_in 5-bit address specifying the destination register (rd) to write to.
 * @param[in] rs1_select_in 5-bit address specifying source register 1 (rs1) to read from.
 * @param[in] rs2_select_in 5-bit address specifying source register 2 (rs2) to read from.
 * @param[in] rs3_select_in 5-bit address specifying source register 3 (rs3) to read from (used for fused multiply-add operations).
 * @param[in] en_load_in Write enable signal; when true, data_in is written to the register specified by select_in on the active clock edge.
 * @param[out] rs1_out 32-bit floating-point data read from source register 1.
 * @param[out] rs2_out 32-bit floating-point data read from source register 2.
 * @param[out] rs3_out 32-bit floating-point data read from source register 3.
 */
SC_MODULE(m_regfile_float)
{
public:
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    sc_in<sc_uint<32>> PN_NAME(data_in);
    sc_in<sc_uint<5>> PN_NAME(select_in);
    sc_in<sc_uint<5>> PN_NAME(rs1_select_in);
    sc_in<sc_uint<5>> PN_NAME(rs2_select_in);
    sc_in<sc_uint<5>> PN_NAME(rs3_select_in);
    sc_in<bool> PN_NAME(en_load_in);

    sc_out<sc_uint<32>> PN_NAME(rs1_out);
    sc_out<sc_uint<32>> PN_NAME(rs2_out);
    sc_out<sc_uint<32>> PN_NAME(rs3_out);

    /* Constructor... */
    SC_CTOR(m_regfile_float)
    {
        SC_CTHREAD(proc_clk_regfile, clk.pos());
        reset_signal_is(reset, true);
        SC_METHOD(proc_cmb_regfile);
        sensitive << rs1_select_in << rs2_select_in << rs3_select_in;
        for(size_t i = 0; i < 32; i++)
        {
            sensitive << regfile[i];
        }
    }

    void pn_trace(sc_trace_file * tf, int level = 1);

    /* Process functions */
    void proc_cmb_regfile();
    void proc_clk_regfile();

protected:
    sc_vector<sc_signal<sc_uint<32>>> PN_NAME_VEC(regfile, PN_CFG_CPU_REGFILE_FLOAT_SIZE);
};

#endif //__REGFILE_FLOAT_H__