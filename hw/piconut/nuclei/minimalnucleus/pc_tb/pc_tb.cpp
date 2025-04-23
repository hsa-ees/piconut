// Design template
#include "pc.h"
#include <systemc.h>
#include <stdint.h>
#include <base.h>

#define PERIOD_NS 10.0

sc_signal<bool> PN_NAME(clk);
sc_signal<bool> PN_NAME(reset);

sc_signal<sc_uint<32>> PN_NAME(pc_in);
sc_signal<sc_uint<32>> PN_NAME(pc_out);
sc_signal<bool> PN_NAME(inc_in);
sc_signal<bool> PN_NAME(en_load_in);

sc_signal<bool> PN_NAME(debug_level_enter_in);
sc_signal<bool> PN_NAME(debug_level_leave_in);
sc_signal<sc_uint<CFG_CSR_BUS_DATA_WIDTH>> PN_NAME(csr_dpc_in);

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
    sc_trace_file* tf = PN_BEGIN_TRACE("pc_tb");

    m_pc dut_inst{"dut_inst"}; // this is the Design name needed by the svc_tool

    // connects signals from TOP to TB
    dut_inst.clk(clk);
    dut_inst.reset(reset);

    dut_inst.pc_in(pc_in);
    dut_inst.pc_out(pc_out);
    dut_inst.inc_in(inc_in);
    dut_inst.en_load_in(en_load_in);
    dut_inst.debug_level_enter_in(debug_level_enter_in);
    dut_inst.debug_level_leave_in(debug_level_leave_in);
    dut_inst.csr_dpc_in(csr_dpc_in);

    dut_inst.Trace(tf, pn_cfg_vcd_level); // Trace signals of the DUT

    sc_start(SC_ZERO_TIME); // start simulation

    cout << "\n\t\t*****Simulation started*****" << endl;

    run_cycle();
    PN_INFOF(("Set reset high"));
    PN_INFOF(("Set reset high"));
    reset = 1;
    run_cycle();
    PN_INFOF(("Set reset low"));
    PN_INFOF(("Set reset low"));
    reset = 0;
    run_cycle();

    /* ------------------- Test 1 ------------------- */
    PN_INFOF(("Test 1: Check if start adress is correct"));
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == 0x10000000, "Test 1 failed");
    PN_INFOF(("Test 1: Passed"));
    PN_INFOF(("\n"));

    /* ------------------- Test 2 ------------------- */
    PN_INFOF(("Test 2: Attempt loading of new value, check if value is loaded"));
    pc_in = 0x0000ffe0;
    PN_INFOF(("pc_in: 0x%x", (uint32_t)pc_in.read()));
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));

    PN_INFOF(("Test 1: Check if start adress is correct"));
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == 0x10000000, "Test 1 failed");
    PN_INFOF(("Test 1: Passed"));
    PN_INFOF(("\n"));

    PN_INFOF(("Test 2: Attempt loading of new value, check if value is loaded"));
    pc_in = 0x0000ffe0;
    PN_INFOF(("pc_in: 0x%x", (uint32_t)pc_in.read()));
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    en_load_in = 1;
    PN_INFOF(("Clock cycle"));
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM((uint32_t)pc_out.read() == (0x0000ffe0), "Test 2 failed");
    PN_INFOF(("Test 2: Passed"));
    PN_INFOF(("\n"));

    /* ------------------- Test 3 ------------------- */
    PN_INFOF(("Test 3: Disable load, enable increment, check if PC is incremented by 4"));
    en_load_in = 0;
    pc_in = 0x0;
    inc_in = 1;
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_INFOF(("Clock cycle"));
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == ((0x0000ffe0) + 0x4), "Test 3 failed");
    PN_INFOF(("Test 3: Passed"));
    PN_INFOF(("\n"));

    /* ------------------- Test 4 ------------------- */
    PN_INFOF(("Test 4: Disable load, disable increment, check if PC is held"));
    inc_in = 0;
    pc_in = 0x0;
    en_load_in = 0;
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_INFOF(("Clock cycle"));
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == ((0x0000ffe0) + 0x4), "Test 4 failed");
    PN_INFOF(("Test 4: Passed"));
    PN_INFOF(("\n"));

    /* ------------------- Test 5 ------------------- */
    PN_INFOF(("Test 5: Set pc_in to some value, don't set load enable, check if value remains"));
    pc_in = 0x11111111;
    run_cycle();
    PN_INFOF(("pc_in: 0x%x", (uint32_t)pc_in.read()));
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_INFOF(("Clock cycle"));
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == ((0x0000ffe0) + 0x4), "Test 5 failed");
    PN_INFOF(("Test 5: Passed"));
    PN_INFOF(("\n"));

    /* ------------------- Test 6 ------------------- */
    PN_INFOF(("Test 6: Perform two more increments. Check if PC is incremented correctly"));
    inc_in = 1;
    run_cycle(2);
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == ((0x0000ffe0) + 0xC), "Test 6 failed");
    PN_INFOF(("Test 6: Passed"));
    inc_in = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 7 ------------------- */
    PN_INFOF(("Test 7: Trigger resest, check if PC is reset correctly"));
    reset = 0;
    PN_INFOF(("Set reset high and advance one clock cycle"));
    run_cycle();
    PN_INFOF(("Set reset low and advance one clock cycle"));
    reset = 1;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == 0x10000000, "Test 7 failed");
    PN_INFOF(("Test 7: Passed"));
    reset = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 8 ------------------- */
    PN_INFOF(("Test 8: Perform debug level enter. Check if PC is set correctly"));
    debug_level_enter_in = 1;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS, "Test 8 failed");
    PN_INFOF(("Test 8: Passed"));
    debug_level_enter_in = 0;
    PN_INFOF(("\n"));

    /* ------------------- Test 9 ------------------- */
    PN_INFOF(("Test 9: Perform debug level leave. Check if PC is set to csr_dpc_in"));
    debug_level_leave_in = 1;
    csr_dpc_in = 0xaffeaff0;
    run_cycle();
    PN_INFOF(("pc_out: 0x%x", (uint32_t)pc_out.read()));
    PN_ASSERTM(pc_out.read() == 0xaffeaff0, "Test 9 failed");
    PN_INFOF(("Test 9: Passed"));
    debug_level_leave_in = 0;
    csr_dpc_in = 0x0;
    PN_INFOF(("\n"));

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}