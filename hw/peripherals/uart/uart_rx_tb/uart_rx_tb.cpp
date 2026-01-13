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
#include "uart_rx.h"
#include <stdint.h>
#include <systemc.h>

#define PERIOD_NS 10.0
#define BAUD_RATE 9600

uint8_t baud_cnt_ack = 0;

// initialize TB signals
sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<bool> PN_NAME(baudtick_i);
sc_signal<bool> PN_NAME(baudtick_x16_i);
sc_signal<bool> PN_NAME(stopbit_8_9_i);
sc_signal<bool> PN_NAME(rx_i);

sc_signal<sc_uint<8>> PN_NAME(data_o);
sc_signal<bool> PN_NAME(rx_finished_o);
sc_signal<bool> PN_NAME(rx_baudgen_disable_o);




void run_cycle(int cycles = 1){
    for (int i = 0; i < cycles; i++){
        clk = 0;
        sc_start(PERIOD_NS/2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS/2, SC_NS);
    }

}

void run_baudtick(int cycles = 1){

    uint8_t baud_cnt = 0;

    // creates a baud tick by counting 16 clk cycles
    // and in the 16th cycle the baudtick is set
    for (int i = 0; i < cycles; i++){
        for (int j = 0; j < 16; j++){

            run_cycle(15);
            if (baud_cnt >= 15){
                baudtick_i = 1;
                baud_cnt = 0;
            } else {
                baudtick_i = 0;
                baud_cnt++;
            }
            baudtick_x16_i = 1;
            run_cycle(1);
            baudtick_x16_i = 0;
            baudtick_i = 0;
        }
    }

}


int sc_main(int argc, char **argv)
{

    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file *tf = PN_BEGIN_TRACE("uart_rx");


    // Initialliaze the Design under Testing (DUT)
    m_uart_rx dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);
    dut_inst.baudtick_i(baudtick_i);
    dut_inst.baudtick_x16_i(baudtick_x16_i);
    dut_inst.stopbit_8_9_i(stopbit_8_9_i);
    dut_inst.rx_i(rx_i);
    dut_inst.data_o(data_o);
    dut_inst.rx_finished_o(rx_finished_o);
    dut_inst.baudtick_disable_o(rx_baudgen_disable_o);

    // Traces of signals here

    dut_inst.pn_trace(tf, pn_cfg_vcd_level); // trace signals of

    sc_start(SC_ZERO_TIME); // start simulation
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Testbench code here
    rx_i = 1;

    // Reset
    PN_INFO("Resetting ...");
    reset = 1;
    run_cycle(1);
    reset = 0;
    run_cycle(1);
    PN_INFO("Reset done ...");

    // sending startbit
    run_baudtick();
    rx_i = 0;
    run_baudtick();

    // send 0x55 (01010101)
    PN_INFO("Sending the byte 0x55 ...");

    for (size_t i = 0; i < 8; i++){

        uint8_t bit = (0x55 >> i) & 0x01;
        rx_i = bit;
        run_baudtick();

    }

    // sending the stopbit
    rx_i = 1;



    PN_INFOF(("Data received: 0x%02X RX Finished: %d", (uint8_t)data_o.read(), (uint8_t)rx_finished_o.read()));
    PN_ASSERTM((uint8_t)data_o.read() == 0x55, "Data received is not correct. Expected 0x55");
    PN_ASSERTM((uint8_t)rx_finished_o.read() == 1, "RX Finished is not correct. Finish not set");


    run_baudtick();

    PN_INFOF(("Data received: 0x%02X RX Finished: %d", (uint8_t)data_o.read(), (uint8_t)rx_finished_o.read()));
    PN_ASSERTM((uint8_t)data_o.read() == 0x55, "Data received is not correct. Expected 0x55");
    PN_ASSERTM((uint8_t)rx_finished_o.read() == 0, "RX Finished is not correct. Finish is set");

    // Test sending 0xAA with 2 stopbits

    stopbit_8_9_i =  1; // 2 stopbits

    // sending startbit
    rx_i = 0;
    run_baudtick();

    // send 0xAA (10101010)
    PN_INFO("Sending the byte 0xAA ...");

    for (size_t i = 0; i < 8; i++){

        uint8_t bit = (0xAA >> i) & 0x01;
        rx_i = bit;
        run_baudtick();

    }

    // sending the stopbit
    rx_i = 1;


    PN_INFOF(("Data received: 0x%02X RX Finished: %d", (uint8_t)data_o.read(), (uint8_t)rx_finished_o.read()));
    PN_ASSERTM((uint8_t)data_o.read() == 0xAA, "Data received is not correct. Expected 0x55");
    PN_ASSERTM((uint8_t)rx_finished_o.read() == 0, "RX Finished is not correct. Finish is set");

    run_baudtick();


    PN_INFOF(("Data received: 0x%02X RX Finished: %d", (uint8_t)data_o.read() , (uint8_t)rx_finished_o.read()));
    PN_ASSERTM((uint8_t)data_o.read() == 0xAA, "Data received is not correct. Expected 0xAA");
    PN_ASSERTM((uint8_t)rx_finished_o.read() == 1, "RX Finished is not correct. Finish not set");


    run_baudtick();


    PN_INFOF(("Data received: 0x%02X RX Finished: %d", (uint8_t)data_o.read(), (uint8_t)rx_finished_o.read()));
    PN_ASSERTM((uint8_t)data_o.read() == 0xAA, "Data received is not correct. Expected 0xAA");
    PN_ASSERTM((uint8_t)rx_finished_o.read() == 0, "RX Finished is not correct. Finish is set");




    run_cycle(); // end with a wait

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}