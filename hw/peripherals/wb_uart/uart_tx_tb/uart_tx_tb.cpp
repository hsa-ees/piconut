/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
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


// Design template
#include "uart_tx.h"
#include <stdint.h>
#include <systemc.h>

#define PERIOD_NS 10.0
#define BAUDTICK_PAUSE 100

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);
sc_signal<bool> PN_NAME(baudtick_i);
sc_signal<bool> PN_NAME(tx_start_i);
sc_signal<bool> PN_NAME(stopbit_cnt_i);
sc_signal<sc_uint<8>> PN_NAME(tx_data_i);
sc_signal<bool> PN_NAME(tx_finish_o);
sc_signal<bool> PN_NAME(tx_o);




void run_cycle(int cycles = 1){
    for (int i = 0; i < cycles; i++){
        clk = 0;
        sc_start(PERIOD_NS/2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS/2, SC_NS);
    }

}

void run_baudtick(int cycles = 1){
    // Send a baudtick in the 16th clk cycle
    for (int i = 0; i < cycles; i++){
        baudtick_i = 1;
        run_cycle(1);
        baudtick_i = 0;
        run_cycle(BAUDTICK_PAUSE - 1);
    }
}

int sc_main(int argc, char **argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file *tf = PN_BEGIN_TRACE("uart_tx");


    // Initialliaze the Design under Testing (DUT)
    m_uart_tx dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.baudtick_i(baudtick_i);
    dut_inst.tx_start_i(tx_start_i);
    dut_inst.stopbit_cnt_i(stopbit_cnt_i);
    dut_inst.tx_data_i(tx_data_i);
    dut_inst.tx_finish_o(tx_finish_o);
    dut_inst.tx_o(tx_o);


    // Traces of signals here

    dut_inst.Trace(tf, pn_cfg_vcd_level); // trace signals of

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here

    // Reset
    reset = 1;
    run_cycle(1);
    reset = 0;
    run_cycle(1);

    // Test Sending the Word "10101010"
    // This should be read at the tx_o signal
    PN_INFO("Test Sending the Word 10101010");

    tx_data_i = 0xAA; // 10101010
    tx_start_i = 1;

    run_cycle(1);

    // before transmitting the uart is idle -> tx_o should be 1
    PN_INFOF(("Tx Value is: %d", tx_o.read()));
    PN_ASSERTM(tx_o == 1, "tx_o should be 1, because no baudtick_i was send");

    run_baudtick();

    // start bit is sent -> tx_o should be 0
    PN_INFOF(("Tx Value is: %d", tx_o.read()));
    PN_ASSERTM(tx_o == 0, "tx_o should be 0, because the start bit is sent");
    tx_start_i = 0;

    // send the 8 data bits should be the value of the tx_data_i signals bit
    for (int i = 0; i < 8; i++){
        run_baudtick();
        PN_INFOF(("Sending bit data[%d]=%d", i, (uint8_t)tx_data_i.read()[i]));
        PN_ASSERTF(tx_o == (tx_data_i.read()[i]), ("tx_o should be equal to %d", tx_data_i.read()[i]));
    }

    // send the stop bit -> tx_o should be 1
    run_baudtick();
    PN_INFOF(("Tx Value is: %d", tx_o.read()));
    PN_ASSERTM(tx_o == 1, "tx_o should be 1, because the stop bit is sent");
    PN_ASSERTM(tx_finish_o == 1, "tx_finish_o should be 1, because the transmission is complete");

    run_baudtick();

    // after the stopbit the tx_o should be 1
    PN_INFOF(("Tx Value is: %d", tx_o.read()));
    PN_ASSERTM(tx_o == 1, "tx_o should be 1, because the tx is in idle state");

    // Testing with 2 stop bits

    PN_INFO("Test Sending the Word 10101010 with 2 stop bits");

    tx_data_i = 0xAA; // 10101010
    tx_start_i = 1;
    stopbit_cnt_i = 1;

    run_cycle(1);

    // before transmitting the uart is idle -> tx_o should be 1
    PN_INFOF(("Tx Value is: %d", tx_o.read()));
    PN_ASSERTM(tx_o == 1, "tx_o should be 1, because no baudtick_i was send");

    run_baudtick();

    // start bit is sent -> tx_o should be 0
    PN_INFOF(("Tx Value is: %d", tx_o.read()));
    PN_ASSERTM(tx_o == 0, "tx_o should be 0, because the start bit is sent");
    tx_start_i = 0;

    // send the 8 data bits should be the value of the tx_data_i signals bit
    for (int i = 0; i < 8; i++){
        run_baudtick();
        PN_INFOF(("Sending bit data[%d]=%d", i, (uint8_t)tx_data_i.read()[i]));
        PN_ASSERTF(tx_o == (tx_data_i.read()[i]), ("tx_o should be equal to %d", tx_data_i.read()[i]));
    }

    run_baudtick();

    // send the stop bit -> tx_o should be 1
    PN_INFOF(("Tx Value is: %d", tx_o.read()));
    PN_ASSERTM(tx_o == 1, "tx_o should be 1, because the stop bit is sent");

    run_baudtick();

    // send the second stop bit -> tx_o should be 1
    PN_INFOF(("Tx Value is: %d", tx_o.read()));
    PN_ASSERTM(tx_o == 1, "tx_o should be 1, because the second stop bit is sent");
    PN_ASSERTM(tx_finish_o == 1, "tx_finish_o should be 1, because the transmission is complete");

    run_baudtick();

    // after the stopbit the tx_o should be 1
    PN_INFOF(("Tx Value is: %d", tx_o.read()));
    PN_ASSERTM(tx_o == 1, "tx_o should be 1, because the tx is in idle state");




    run_cycle(); // end with a wait

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}