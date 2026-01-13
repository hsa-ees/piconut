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
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR a_in PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *************************************************************************/


#include "immgen.h"
#include <systemc.h>
#include <stdint.h>
#include <piconut.h>

#define PERIOD_NS 10.0

sc_signal<sc_uint<32>> PN_NAME(data_in);
sc_signal<sc_uint<32>> PN_NAME(imm_out);

void run_cycle(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {

        sc_start(PERIOD_NS, SC_NS);
    }
}

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("immgen_tb");

    m_immgen dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.data_in(data_in);
    dut_inst.imm_out(imm_out);

    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;

    run_cycle();
    data_in = 0x00428293; // I-type
    run_cycle();
    PN_ASSERTM(imm_out.read() == 0x4, "I-type immgen 1 failed");

    data_in = 0xed028293; // I-Type
    run_cycle();
    PN_ASSERTM((uint32_t)imm_out.read() == ((~304) + 0x1), "I-type immgen 2 failed");

    data_in = 0x0062ac23; // S-type
    run_cycle();
    PN_ASSERTM(imm_out.read() == 24, "S-type immgen 1 failed");

    data_in = 0x0062a023; // S-type
    run_cycle();
    PN_ASSERTM(imm_out.read() == 0x0, "S-type immgen 2 failed");

    data_in = 0x00028a63; // B-type
    run_cycle();
    PN_ASSERTM(imm_out.read() == 20, "B-type immgen 1 failed");

    data_in = 0x00050463; // B-type
    run_cycle();
    PN_ASSERTM(imm_out.read() == 8, "B-type immgen 2 failed");

    data_in = 0x00730863; // B-type
    run_cycle();
    PN_ASSERTM(imm_out.read() == 16, "B-type immgen 3 failed");

    data_in = 0x458070ef; // J-type
    run_cycle();
    PN_ASSERTM(imm_out.read() == 29784, "J-type immgen 1 failed");

    data_in = 0x155140ef; // J-type
    run_cycle();
    PN_ASSERTM(imm_out.read() == 84308, "J-type immgen 2 failed");

    data_in = 0x8e10f06f; // J-type
    run_cycle();
    PN_ASSERTM((uint32_t)imm_out.read() == ((~984864) + 0x1), "J-type immgen 3 failed");

    data_in = 0x00000517; // U-type
    run_cycle();
    PN_ASSERTM(imm_out.read() == 0x0, "U-type immgen 1 failed");

    data_in = 0x00015697; // U-type
    run_cycle();
    PN_ASSERTM(imm_out.read() == 0x15 << 12, "U-type immgen 2 failed");

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}