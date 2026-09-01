#include "../compare_float.h"

#include <systemc.h>
#include <stdint.h>
#include <piconut.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<sc_uint<32>> PN_NAME(a_in);
sc_signal<sc_uint<32>> PN_NAME(b_in);
sc_signal<sc_uint<2>> PN_NAME(mode_in);

sc_signal<sc_uint<1>> PN_NAME(data_out);

void run_cycle(int cycles = 1) {
    for(int i = 0; i < cycles; i++) {
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

int sc_main(int argc, char** argv) {
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("compare_float_tb");

    m_compare_float i_dut{"i_dut"};

    i_dut.clk(clk);
    i_dut.reset(reset);

    i_dut.a_in(a_in);
    i_dut.b_in(b_in);
    i_dut.mode_in(mode_in);
    i_dut.data_out(data_out);

    i_dut.pn_trace(tf, pn_cfg_vcd_level);

    sc_start(SC_ZERO_TIME);

    cout << "\n\t\t*****Simulation started*****" << endl;

    run_cycle();
    reset = 1;
    run_cycle();
    reset = 0;
    run_cycle();

    PN_INFO("TEST 1: 2 is less than 3");
    a_in = 2.0f;
    b_in = 3.0f;
    mode_in = LESS_THAN;
    run_cycle();
    PN_ASSERTM(data_out.read() == 1, "TEST 1 failed");
    run_cycle();

    PN_INFO("TEST 2: 3 is NOT less than 2");
    a_in = 3.0f;
    b_in = 2.0f;
    mode_in = LESS_THAN;
    run_cycle();
    PN_ASSERTM(data_out.read() == 0, "TEST 2 failed");
    run_cycle();

    PN_INFO("TEST 3: 3 is NOT less than 3");
    a_in = 3.0f;
    b_in = 3.0f;
    mode_in = LESS_THAN;
    run_cycle();
    PN_ASSERTM(data_out.read() == 0, "TEST 3 failed");
    run_cycle();

    PN_INFO("TEST 4: 3 is equal to 3");
    a_in = 3.0f;
    b_in = 3.0f;
    mode_in = EQUAL;
    run_cycle();
    PN_ASSERTM(data_out.read() == 1, "TEST 4 failed");
    run_cycle();

    PN_INFO("TEST 5: 2 is NOT equal to 3");
    a_in = 3.0f;
    b_in = 2.0f;
    mode_in = EQUAL;
    run_cycle();
    PN_ASSERTM(data_out.read() == 0, "TEST 5 failed");
    run_cycle();

    PN_INFO("TEST 6: 0 is equal to -0");
    a_in = 0.0f;
    b_in = -0.0f;
    mode_in = EQUAL;
    run_cycle();
    PN_ASSERTM(data_out.read() == 1, "TEST 6 failed");
    run_cycle();

    PN_INFO("TEST 7: 2 is less or equal to 3");
    a_in = 2.0f;
    b_in = 3.0f;
    mode_in = LESS_EQUAL;
    run_cycle();
    PN_ASSERTM(data_out.read() == 1, "TEST 7 failed");
    run_cycle();

    PN_INFO("TEST 8: 3 is less or equal to 3");
    a_in = 3.0f;
    b_in = 3.0f;
    mode_in = LESS_EQUAL;
    run_cycle();
    PN_ASSERTM(data_out.read() == 1, "TEST 8 failed");
    run_cycle();

    PN_INFO("TEST 9: 3 is NOT less or equal to 2");
    a_in = 3.0f;
    b_in = 2.0f;
    mode_in = LESS_EQUAL;
    run_cycle();
    PN_ASSERTM(data_out.read() == 0, "TEST 9 failed");
    run_cycle();

    PN_INFO("TEST 10: NAN is always false");
    a_in = 0b01111111110000000000000000000000;
    b_in = 2.0f;
    mode_in = LESS_EQUAL;
    run_cycle();
    PN_ASSERTM(data_out.read() == 0, "TEST 10 failed");
    run_cycle();

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}