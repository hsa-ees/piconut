/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
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

#include <systemc.h>

#include "top.h"

#include <remote_bitbang.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);
sc_signal<bool> PN_NAME(tx);
sc_signal<bool> PN_NAME(rx);
sc_signal<bool> PN_NAME(tck);
sc_signal<bool> PN_NAME(tms);
sc_signal<bool> PN_NAME(tdi);
sc_signal<bool> PN_NAME(tdo);
sc_signal<bool> PN_NAME(trst_n);

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

    pn_parse_enable_trace_core = 1;     // enable core trace dump
    pn_cfg_enable_application_path = 1; // enable application path in program args
    PN_PARSE_CMD_ARGS(argc, argv);      // parse command line arguments

    sc_trace_file* tf = PN_BEGIN_TRACE("piconut_tb"); // create trace file

    auto callback_jtag_set_inputs = [](bool _tck, bool _tms, bool _tdi) {
        tck = _tck;
        tms = _tms;
        tdi = _tdi;
        run_cycle(2);
    };

    auto callback_jtag_get_outputs = []() -> bool {
        bool _tdo = tdo.read();
        run_cycle();

        return _tdo;
    };

    auto callback_jtag_trigger_reset = []() {
        trst_n = 0;
        run_cycle();
        trst_n = 1;
        run_cycle();
    };

    c_remote_bitbang remote_bitbang{
        callback_jtag_set_inputs,
        callback_jtag_get_outputs,
        callback_jtag_trigger_reset};

    m_demo_debugger i_dut{"i_dut"};

    i_dut.clk(clk);
    i_dut.reset(reset);
    i_dut.tx_o(tx);
    i_dut.rx_i(rx);
    i_dut.tck_i(tck);
    i_dut.tms_i(tms);
    i_dut.tdi_i(tdi);
    i_dut.tdo_o(tdo);

    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME);
    cout << "\n\t\t*****Simulation started*****" << endl;

    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle(2);

    while(!remote_bitbang.connected())
    {
        remote_bitbang.process();
    }

    while(remote_bitbang.connected() ||
          i_dut.cpu->state_is_not_halt())
    {
        remote_bitbang.process();

        run_cycle();
    }

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
