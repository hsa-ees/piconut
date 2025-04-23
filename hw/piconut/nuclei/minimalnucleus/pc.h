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

/** PC (Program Counter)
 *
 * The PC module is a simple register that holds the program counter value.
 * The program counter value represents the current position of the program in memory.
 *
 * Input ports:
 *  - pc_in<30>:        The value to be loaded into the register.
 *  - en_load_in<1>:    If en_load_in = 1, the value on ir_in is loaded into the register with the next rising edge,
 *                      else the current value is held.
 *  - inc_in<1>:        If inc_in = 1, the value of the register is incremented by 4 with the next rising edge,
 *                      else the current value is held.
 *
 * Output ports:
 *  - pc_out<32>:       Permanent output of the current register value.
 *
 * Note: The input and the internal register are 30 bits wide. This is because instruction addresses are
 *       inherently 32-bit word aligned. Thus the lowest two bits are omissible.
 */

#ifndef __PC_H__
#define __PC_H__

#include <systemc.h>
#include <stdint.h>
#include <base.h>
#include <piconut-config.h>

SC_MODULE(m_pc)
{
public:
    sc_in_clk PN_NAME(clk);
    sc_in<bool> PN_NAME(reset);

    sc_in<sc_uint<32>> PN_NAME(pc_in);

    sc_in<bool> PN_NAME(inc_in);
    sc_in<bool> PN_NAME(en_load_in);

    sc_out<sc_uint<32>> PN_NAME(pc_out);

    // Debug level
    sc_in<bool> PN_NAME(debug_level_enter_in);
    sc_in<bool> PN_NAME(debug_level_leave_in);
    sc_in<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_dpc_in);

    /* Constructor... */
    SC_CTOR(m_pc)
    {
        SC_CTHREAD(proc_clk_pc, clk.pos());
        reset_signal_is(reset, true);
        SC_METHOD(proc_cmb_pc);
        sensitive << pc_reg;
    }

    void Trace(sc_trace_file * tf, int level = 1);

    void proc_cmb_pc();
    void proc_clk_pc();

protected:
    sc_signal<sc_uint<30>> PN_NAME(pc_reg);
};

#endif //__PC_H__