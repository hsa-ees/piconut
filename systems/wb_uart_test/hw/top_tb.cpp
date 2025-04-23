/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Lukas Bauer <lukas.bauer1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the c_soft_uart  simulation ONLY

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

#include <stdint.h>
#include <systemc.h>
#include "top.h"

#define PERIOD_NS 40
#define SYS_FREQ 1 / 40E-9
#define BAUDRATE 115200
#define BAUDDIV (int)((SYS_FREQ / BAUDRATE) - 1)
#define BAUDTICKS (int)(((float)1 / BAUDRATE) / 40E-9)


// initialize TB signals
sc_signal<bool> PN_NAME(clk_25);
sc_signal<bool> PN_NAME(reset);
sc_signal<bool> PN_NAME(rx);
sc_signal<bool> PN_NAME(tx);

void run_cycle(int cycles = 1)
{
    for (int i = 0; i < cycles; i++)
    {
        clk_25 = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk_25 = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int sc_main(int argc, char **argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);                    // parse command line arguments
    sc_trace_file *tf = PN_BEGIN_TRACE("piconut_tb"); // create trace file

    // Initialliaze the Design under Testing (DUT)
    m_top dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk_25(clk_25);
    dut_inst.reset(reset);
    dut_inst.rx(rx);
    dut_inst.tx(tx);

    dut_inst.Trace(tf, pn_cfg_vcd_level); // trace signals of DUT

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here

    // seting the rx signal to high (default in UART)
    rx = 1;

    run_cycle(20);

    PN_INFOF(("Sending: 0x55"));
    // Sending start bit
    rx = 0;
    PN_INFOF(("Sending %d", rx.read()));
    run_cycle(BAUDTICKS);

    for (int i = 0; i < 8; i++)
    {

        rx = (0x55 >> i) & 0x1;
        PN_INFOF(("Sending %d", rx.read()));
        run_cycle(BAUDTICKS);
    }

    // Sending stop bit
    rx = 1;
    PN_INFOF(("Sending %d", rx.read()));
    run_cycle(BAUDTICKS * 1);


    // Receive the data that was sent before as an echo

    // Receiving the data
    PN_INFO("Receiving 0x55");
    PN_INFOF(("Receiving %d", tx.read()));
    PN_ASSERTM(tx == 0, "Starbit should be set");

    for (int i = 0; i < 8; i++)
    {

        run_cycle(BAUDTICKS);
        PN_INFOF(("Receiving %d", tx.read()));
        PN_ASSERTM(tx == ((0x55 >> i) & 0x1), "Data bit should be set");
    }

    // receiving stop bit
    run_cycle(BAUDTICKS);
    PN_INFOF(("Receiving %d", tx.read()));
    PN_ASSERTM(tx == 1, "Stop bit should be set");

    run_cycle(); // end with a wait

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}
