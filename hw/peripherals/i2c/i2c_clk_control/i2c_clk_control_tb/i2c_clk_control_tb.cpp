/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Eikya Lagisetti <eikya.lagisetti1@tha.de>
                     2025 Johannes Fleiner <johannes.fleiner1@tha.de>
                     2025 Sebastian Ebenhöh <sebastian.moritz.ebenhöh@tha.de>
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

#include "../i2c_clk_control.h"
#include <systemc.h>
#include <piconut.h>
#include <string>

#define PERIOD_NS 40

enum test_phase
{
    TP_RESET,
    TP_CONFIG_STD,
    TP_START_STD,
    TP_DATA_STD,
    TP_STOP_STD,
    TP_STD_SLOW_50KHZ,
    TP_STD_MIN_TIMING,
    TP_STD_EMPTY_FRAME,
    TP_STD_RESET_RECOVERY,
    TP_STD_STRESS_STRETCH,
    TP_CONFIG_FAST,
    TP_START_FAST,
    TP_SLAVE_STRETCH,
    TP_MASTER_STRETCH,
    TP_RESTART,
    TP_FINISHED
};

// ============================= Testbench-Signals =============================

sc_signal<int> PN_NAME(test_phase_sig);

sc_signal<bool> PN_NAME(clk_sig);
sc_signal<bool> PN_NAME(reset_sig);

sc_signal<sc_uint<16>> PN_NAME(ccr_sig);
sc_signal<sc_uint<6>> PN_NAME(trise_sig);
sc_signal<bool> PN_NAME(scl_on_sig);
sc_signal<bool> PN_NAME(master_stretch_sig);
sc_signal<bool> PN_NAME(init_master_sig);

sc_signal<bool> PN_NAME(scl_in_sig);
sc_signal<bool> PN_NAME(scl_out_sig);

sc_signal<bool> PN_NAME(data_clk_pulse_write_sig);
sc_signal<bool> PN_NAME(data_clk_pulse_read_sig);
sc_signal<bool> PN_NAME(scl_ready_sig);
sc_signal<bool> PN_NAME(start_finished_sig);
sc_signal<bool> PN_NAME(start_ready_sig);
sc_signal<bool> PN_NAME(generate_stop_sig);
sc_signal<bool> PN_NAME(stop_ready_sig);

sc_signal<bool> PN_NAME(slave_stretch_sig);
sc_signal<bool> PN_NAME(stop_finished_sig);

// ========================== Helper module and functions =============================

SC_MODULE(bus_model)
{
    sc_in<bool> scl_out;
    sc_in<bool> slave_stretch;
    sc_out<bool> scl_in;

    void bus_logic()
    {
        if(scl_out.read() == false || slave_stretch.read() == true)
        {
            scl_in.write(false);
        }
        else
        {
            scl_in.write(true);
        }
    }

    SC_CTOR(bus_model)
    {
        SC_METHOD(bus_logic);
        sensitive << scl_out << slave_stretch;
    }
};

void run_cycle(int cycles = 1)
{
    for(int i = 0; i < cycles; i++)
    {
        clk_sig = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk_sig = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

// Returns bool for use in PN_ASSERT, prints name on timeout
bool wait_for_signal_high(sc_signal<bool>& sig, int timeout_cycles, const char* name)
{
    int count = 0;
    while(!sig.read())
    {
        run_cycle(1);
        count++;
        if(count >= timeout_cycles)
        {
            std::string msg = "TIMEOUT waiting for signal: ";
            msg += name;
            PN_INFO(msg.c_str());
            return false;
        }
    }
    return true;
}

void setup_transaction()
{
    scl_on_sig = 0;
    start_ready_sig = 0;
    generate_stop_sig = 0;
    run_cycle(5);

    // Init Pulse
    init_master_sig = 1;
    run_cycle(2);
    init_master_sig = 0;
    run_cycle(10);

    // Start Trigger
    scl_on_sig = 1;
    start_ready_sig = 1;
}

void reset_dut()
{
    PN_INFO("Resetting DUT...");
    test_phase_sig = TP_RESET;
    reset_sig = 1;
    scl_on_sig = 0;
    init_master_sig = 0;
    ccr_sig = 0;
    trise_sig = 0;
    slave_stretch_sig = 0;
    master_stretch_sig = 0;
    start_ready_sig = 0;
    generate_stop_sig = 0;

    run_cycle(5);
    reset_sig = 0;
    run_cycle(5);
}

// ============================= Test Functions =============================

void test_std_mode_basic()
{
    PN_INFO("Test: Standard Mode (100 kHz) - Basic");
    test_phase_sig = TP_CONFIG_STD;
    ccr_sig = 0x007D;
    trise_sig = 0x1A;
    run_cycle(2);

    setup_transaction();

    test_phase_sig = TP_START_STD;
    PN_ASSERT(wait_for_signal_high(start_finished_sig, 1000, "start_finished_sig"));

    test_phase_sig = TP_DATA_STD;
    run_cycle(200);

    test_phase_sig = TP_STOP_STD;
    generate_stop_sig = 1;
    PN_ASSERT(wait_for_signal_high(stop_ready_sig, 1000, "stop_ready_sig"));

    generate_stop_sig = 0;
    PN_ASSERT(wait_for_signal_high(stop_finished_sig, 2000, "stop_finished_sig"));

    scl_on_sig = 0;
    run_cycle(10);
}

void test_slow_mode_50khz()
{
    PN_INFO("Test: Slow Standard Mode (50 kHz)");
    test_phase_sig = TP_STD_SLOW_50KHZ;
    ccr_sig = 0x00FA;
    trise_sig = 0x1A;
    run_cycle(2);

    setup_transaction();

    PN_ASSERT(wait_for_signal_high(start_finished_sig, 2000, "start_finished_sig"));
    run_cycle(500);

    generate_stop_sig = 1;
    PN_ASSERT(wait_for_signal_high(stop_ready_sig, 2000, "stop_ready_sig"));

    generate_stop_sig = 0;
    PN_ASSERT(wait_for_signal_high(stop_finished_sig, 2000, "stop_finished_sig"));
    scl_on_sig = 0;
    run_cycle(10);
}

void test_min_timing()
{
    PN_INFO("Test: Minimum Timing (Fastest Std Mode)");
    test_phase_sig = TP_STD_MIN_TIMING;
    ccr_sig = 0x0020;
    run_cycle(2);

    setup_transaction();

    PN_ASSERT(wait_for_signal_high(start_finished_sig, 500, "start_finished_sig"));
    run_cycle(100);

    generate_stop_sig = 1;
    PN_ASSERT(wait_for_signal_high(stop_ready_sig, 500, "stop_ready_sig"));

    generate_stop_sig = 0;
    PN_ASSERT(wait_for_signal_high(stop_finished_sig, 1000, "stop_finished_sig"));
    scl_on_sig = 0;
    run_cycle(10);
}

void test_empty_frame()
{
    PN_INFO("Test: Empty Frame (Start -> immediate Stop)");
    test_phase_sig = TP_STD_EMPTY_FRAME;
    ccr_sig = 0x007D; // Back to normal 100kHz
    run_cycle(2);

    setup_transaction();

    PN_ASSERT(wait_for_signal_high(start_finished_sig, 1000, "start_finished_sig"));

    // Request immediate stop without data phase
    generate_stop_sig = 1;
    PN_ASSERT(wait_for_signal_high(stop_ready_sig, 1000, "stop_ready_sig"));

    generate_stop_sig = 0;
    PN_ASSERT(wait_for_signal_high(stop_finished_sig, 1000, "stop_finished_sig"));
    scl_on_sig = 0;
    run_cycle(10);
}

void test_reset_recovery()
{
    PN_INFO("Test: Reset Recovery during Operation");
    test_phase_sig = TP_STD_RESET_RECOVERY;

    setup_transaction();
    PN_ASSERT(wait_for_signal_high(start_finished_sig, 1000, "start_finished_sig"));

    run_cycle(100);

    PN_INFO("   -> Asserting RESET...");
    reset_sig = 1;
    run_cycle(10);
    reset_sig = 0;
    run_cycle(10);

    PN_INFO("   -> Restarting after Reset...");
    setup_transaction();

    // Check if module recovers
    bool recovered = wait_for_signal_high(start_finished_sig, 1000, "start_finished_sig");
    if(!recovered)
    {
        PN_INFO("   ERROR: Module did not restart after reset!");
    }
    PN_ASSERT(recovered);

    generate_stop_sig = 1;
    PN_ASSERT(wait_for_signal_high(stop_ready_sig, 1000, "stop_ready_sig"));

    generate_stop_sig = 0;
    PN_ASSERT(wait_for_signal_high(stop_finished_sig, 1000, "stop_finished_sig"));
    scl_on_sig = 0;
    run_cycle(10);
}

void test_master_stress()
{
    PN_INFO("Test: Master Stress (Repeated Stretching)");
    test_phase_sig = TP_STD_STRESS_STRETCH;

    setup_transaction();
    PN_ASSERT(wait_for_signal_high(start_finished_sig, 1000, "start_finished_sig"));

    for(int k = 0; k < 5; k++)
    {
        run_cycle(50);
        master_stretch_sig = 1;
        run_cycle(40);
        master_stretch_sig = 0;
    }
    run_cycle(50);

    generate_stop_sig = 1;
    PN_ASSERT(wait_for_signal_high(stop_ready_sig, 2000, "stop_ready_sig"));

    generate_stop_sig = 0;
    PN_ASSERT(wait_for_signal_high(stop_finished_sig, 2000, "stop_finished_sig"));
    scl_on_sig = 0;
    run_cycle(10);
}

void test_fast_mode()
{
    PN_INFO("Test: Configuration Fast Mode (400 kHz)");
    test_phase_sig = TP_CONFIG_FAST;
    ccr_sig = 0x8015;
    trise_sig = 0x0A;
    run_cycle(2);

    setup_transaction();

    test_phase_sig = TP_START_FAST;
    PN_ASSERT(wait_for_signal_high(start_finished_sig, 500, "start_finished_sig"));

    // Cleanup for next test
    generate_stop_sig = 1;
    PN_ASSERT(wait_for_signal_high(stop_ready_sig, 500, "stop_ready_sig"));
    generate_stop_sig = 0;
    PN_ASSERT(wait_for_signal_high(stop_finished_sig, 1000, "stop_finished_sig"));
    scl_on_sig = 0;
    run_cycle(10);
}

void test_slave_clock_stretching()
{
    PN_INFO("Test: Slave Clock Stretching");
    test_phase_sig = TP_SLAVE_STRETCH;

    // Ensure we are in a transaction or start one
    setup_transaction();
    PN_ASSERT(wait_for_signal_high(start_finished_sig, 500, "start_finished_sig"));

    run_cycle(50);

    slave_stretch_sig = 1;
    run_cycle(200);
    slave_stretch_sig = 0;
    run_cycle(100);

    generate_stop_sig = 1;
    PN_ASSERT(wait_for_signal_high(stop_ready_sig, 1000, "stop_ready_sig"));
    generate_stop_sig = 0;
    PN_ASSERT(wait_for_signal_high(stop_finished_sig, 2000, "stop_finished_sig"));
    scl_on_sig = 0;
    run_cycle(10);
}

void test_master_clock_stretching()
{
    PN_INFO("Test: Master Clock Stretching");
    test_phase_sig = TP_MASTER_STRETCH;

    setup_transaction();
    PN_ASSERT(wait_for_signal_high(start_finished_sig, 500, "start_finished_sig"));

    master_stretch_sig = 1;
    run_cycle(250);
    master_stretch_sig = 0;
    run_cycle(100);

    generate_stop_sig = 1;
    PN_ASSERT(wait_for_signal_high(stop_ready_sig, 1000, "stop_ready_sig"));
    generate_stop_sig = 0;
    PN_ASSERT(wait_for_signal_high(stop_finished_sig, 2000, "stop_finished_sig"));
    scl_on_sig = 0;
    run_cycle(10);
}

void test_restart_sequence()
{
    PN_INFO("Test: Restart Sequence (Stop -> Start)");
    test_phase_sig = TP_RESTART;

    // First transaction
    setup_transaction();
    PN_ASSERT(wait_for_signal_high(start_finished_sig, 500, "start_finished_sig"));

    generate_stop_sig = 1;
    PN_ASSERT(wait_for_signal_high(stop_ready_sig, 2000, "stop_ready_sig"));
    generate_stop_sig = 0;

    PN_ASSERT(wait_for_signal_high(stop_finished_sig, 2000, "stop_finished_sig"));

    start_ready_sig = 0;
    run_cycle(20);

    // Restart logic
    init_master_sig = 1;
    run_cycle(2);
    init_master_sig = 0;
    run_cycle(10);

    scl_on_sig = 1;
    start_ready_sig = 1;

    PN_ASSERT(wait_for_signal_high(start_finished_sig, 1000, "start_finished_sig"));

    run_cycle(100);

    PN_INFO("Final Stop");
    generate_stop_sig = 1;

    bool stop_ok = wait_for_signal_high(stop_ready_sig, 2000, "stop_ready_sig");
    if(!stop_ok)
    {
        PN_INFO("Warning: Last Stop not reached.");
    }
    PN_ASSERT(stop_ok);

    generate_stop_sig = 0;
    PN_ASSERT(wait_for_signal_high(stop_finished_sig, 2000, "stop_finished_sig"));

    scl_on_sig = 0;
    run_cycle(20);
}

// ============================= Main Function =============================

int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("m_i2c_clk_control");

    m_i2c_clk_control i_dut("i_dut");

    i_dut.clk(clk_sig);
    i_dut.reset(reset_sig);
    i_dut.ccr_in(ccr_sig);
    i_dut.trise_in(trise_sig);
    i_dut.scl_on_in(scl_on_sig);
    i_dut.master_stretch_in(master_stretch_sig);
    i_dut.init_master_in(init_master_sig);
    i_dut.scl_in(scl_in_sig);
    i_dut.scl_out(scl_out_sig);
    i_dut.data_clk_pulse_write_out(data_clk_pulse_write_sig);
    i_dut.data_clk_pulse_read_out(data_clk_pulse_read_sig);
    i_dut.scl_ready_out(scl_ready_sig);
    i_dut.start_finished_out(start_finished_sig);
    i_dut.start_ready_in(start_ready_sig);
    i_dut.generate_stop_in(generate_stop_sig);
    i_dut.stop_ready_out(stop_ready_sig);
    i_dut.stop_finished_out(stop_finished_sig);

    bus_model i_bus("i_bus");
    i_bus.scl_out(scl_out_sig);
    i_bus.slave_stretch(slave_stretch_sig);
    i_bus.scl_in(scl_in_sig);

    i_dut.pn_trace(tf, 1);
    PN_TRACE(tf, slave_stretch_sig);
    PN_TRACE(tf, test_phase_sig);
    PN_TRACE(tf, init_master_sig);
    PN_TRACE(tf, start_ready_sig);

    PN_INFO("***** Simulation Started *****");

    reset_dut();

    // Execute Test Cases
    test_std_mode_basic();
    test_slow_mode_50khz();
    test_min_timing();
    test_empty_frame();
    test_reset_recovery();
    test_master_stress();
    test_fast_mode();
    test_slave_clock_stretching();
    test_master_clock_stretching();
    test_restart_sequence();

    test_phase_sig = TP_FINISHED;

    PN_INFO("***** Simulation Finished *****");
    PN_END_TRACE();
    return 0;
}