/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
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

#include "../dtm.h"

#include <piconut.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);
sc_signal<bool> PN_NAME(tck);
sc_signal<bool> PN_NAME(tms);
sc_signal<bool> PN_NAME(tdi);
sc_signal<bool> PN_NAME(trst_n);
sc_signal<bool> PN_NAME(tdo);
sc_signal<sc_uint<PN_CFG_DEBUG_DMI_BUS_ADR_WIDTH>> PN_NAME(dmi_adr);
sc_signal<sc_uint<32>> PN_NAME(dmi_dat_o);
sc_signal<sc_uint<32>> PN_NAME(dmi_dat_i);
sc_signal<bool> PN_NAME(dmi_re);
sc_signal<bool> PN_NAME(dmi_we);

/////////////// Helpers ///////////////
void run_cycle(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("dtm_tb");

    PN_INFO("Start unit test ...");

    m_dtm PN_NAME(i_dut);
    i_dut.clk(clk);
    i_dut.reset(reset);
    i_dut.tck_i(tck);
    i_dut.tms_i(tms);
    i_dut.tdi_i(tdi);
    i_dut.trst_n_i(trst_n);
    i_dut.tdo_o(tdo);
    i_dut.dmi_adr_o(dmi_adr);
    i_dut.dmi_dat_o(dmi_dat_o);
    i_dut.dmi_dat_i(dmi_dat_i);
    i_dut.dmi_re_o(dmi_re);
    i_dut.dmi_we_o(dmi_we);

    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME);
    cout << "\n\t\t*****Simulation started*****" << endl;

    trst_n = 1;

    // Reset-Test
    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle();

    // TBD(jh): Implement unit test.

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
