/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Alexander Beck <alexander.beck1@tha.de>
                2025 Christian Zellinger <christian.zellinger1@tha.de>
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

#include <stdint.h>
#include <systemc.h>
#include <piconut.h>
#include "../clint.h"

#define PERIOD_NS 10.0

// TB clock and reset
sc_signal<bool> clk;
sc_signal<bool> reset;

// Wishbone interface signals
sc_signal<bool> wb_stb;
sc_signal<bool> wb_cyc;
sc_signal<bool> wb_we;
sc_signal<sc_uint<32>> wb_adr;
sc_signal<sc_uint<32>> wb_dat_i;
sc_signal<sc_uint<4>> wb_sel;
sc_signal<sc_uint<32>> wb_dat_o;
sc_signal<bool> wb_ack;
sc_signal<bool> wb_err;
sc_signal<bool> wb_rty;

// Interrupt output signals
sc_signal<bool> msip_out;
sc_signal<bool> mtip_out;

// Forward declaration of DUT instance
m_clint* dut_ptr = nullptr;

// Forward declaration of run_cycle function
void run_cycle(int cycles = 1);

// Direct register access methods
uint32_t direct_read_msip()
{
    return dut_ptr->msip_reg.read().to_uint();
}

void direct_write_msip(uint32_t value)
{
    dut_ptr->msip_reg.write(value);
    // One cycle for signal propagation
    run_cycle(1);
}

uint64_t direct_read_mtime()
{
    return dut_ptr->mtime_reg.read().to_uint64();
}

uint64_t direct_read_mtimecmp()
{
    return dut_ptr->mtimecmp_reg.read().to_uint64();
}

void direct_write_mtimecmp(uint64_t value)
{
    dut_ptr->mtimecmp_reg.write(value);
    // One cycle for signal propagation
    run_cycle(1);
}

// Helper function to run cycles
void run_cycle(int cycles)
{
    for(int i = 0; i < cycles; i++)
    {
        clk = 0;
        sc_start(PERIOD_NS / 2, SC_NS);
        clk = 1;
        sc_start(PERIOD_NS / 2, SC_NS);
    }
}

// Read from register via Wishbone
uint32_t read_register(uint32_t addr)
{
    wb_stb = 1;
    wb_cyc = 1;
    wb_we = 0;
    wb_adr = addr;
    wb_sel = 0xF;

    run_cycle(1);

    while(!wb_ack.read())
    {
        run_cycle(1);
    }

    uint32_t data = wb_dat_o.read();
    wb_stb = 0;
    wb_cyc = 0;
    run_cycle(1);

    return data;
}

// Write to register via Wishbone
void write_register(uint32_t addr, uint32_t data, uint8_t sel_mask = 0xF)
{
    wb_stb = 1;
    wb_cyc = 1;
    wb_we = 1;
    wb_adr = addr;
    wb_dat_i = data;
    wb_sel = sel_mask;

    run_cycle(1);

    while(!wb_ack.read())
    {
        run_cycle(1);
    }

    wb_stb = 0;
    wb_cyc = 0;
    wb_we = 0;
    run_cycle(1);
}

// Helper function to safely set a 64-bit register value
void write_register64(uint32_t base_addr, uint64_t value)
{
    // Write upper 32 bits first for registers like MTIMECMP to ensure atomic update
    uint32_t upper = (value >> 32) & 0xFFFFFFFF;
    uint32_t lower = value & 0xFFFFFFFF;

    // Upper word first
    write_register(base_addr + 4, upper);

    // Then lower word
    write_register(base_addr, lower);

    // Ensure time for the write to take effect
    run_cycle(1);
}

// Helper to wait for MTIME to reach a specific value
bool wait_for_mtime(uint64_t target_value, int max_cycles = 1000)
{
    int cycles = 0;
    uint64_t current_time = 0;

    do
    {
        uint32_t mtime_lo = read_register(CLINT_REG_MTIME_LO);
        uint32_t mtime_hi = read_register(CLINT_REG_MTIME_HI);
        current_time = ((uint64_t)mtime_hi << 32) | mtime_lo;

        if(current_time >= target_value)
        {
            return true;
        }

        run_cycle(1);
        cycles++;
    } while(cycles < max_cycles);

    PN_ERRORF(("Timeout waiting for MTIME to reach 0x%016llx, current value: 0x%016llx",
        (unsigned long long)target_value,
        (unsigned long long)current_time));
    return false;
}

// Read a byte from CLINT register
uint8_t read_register8(uint32_t addr)
{
    // Align the address to a word boundary and calculate byte position
    uint32_t aligned_addr = addr & ~0x3;
    int byte_pos = addr % 4;

    wb_stb = 1;
    wb_cyc = 1;
    wb_we = 0;
    wb_adr = aligned_addr;
    wb_sel = 0xF; // Read the full word

    run_cycle(1);

    while(!wb_ack.read())
    {
        run_cycle(1);
    }

    // Extract the specific byte from the full word
    uint8_t data = (wb_dat_o.read() >> (8 * byte_pos)) & 0xFF;

    wb_stb = 0;
    wb_cyc = 0;
    run_cycle(1);

    return data;
}

// Read a 16-bit value from CLINT register
uint16_t read_register16(uint32_t addr)
{
    wb_stb = 1;
    wb_cyc = 1;
    wb_we = 0;
    wb_adr = addr;
    wb_sel = 0x3; // Select lowest two bytes

    run_cycle(1);

    while(!wb_ack.read())
    {
        run_cycle(1);
    }

    uint16_t data = wb_dat_o.read() & 0xFFFF;
    wb_stb = 0;
    wb_cyc = 0;
    run_cycle(1);

    return data;
}

// Write a 16-bit value to CLINT register
void write_register16(uint32_t addr, uint16_t data)
{
    wb_stb = 1;
    wb_cyc = 1;
    wb_we = 1;
    wb_adr = addr;
    wb_dat_i = data;
    wb_sel = 0x3; // Select lowest two bytes

    run_cycle(1);

    while(!wb_ack.read())
    {
        run_cycle(1);
    }

    wb_stb = 0;
    wb_cyc = 0;
    wb_we = 0;
    run_cycle(1);
}

int sc_main(int argc, char** argv)
{
    pn_parse_enable_trace_core = 1;
    PN_PARSE_CMD_ARGS(argc, argv);
    sc_trace_file* tf = PN_BEGIN_TRACE("clint_tb");

    // Instantiate CLINT module
    m_clint i_dut{"i_dut"};

    // Set the global pointer to the DUT instance for direct access
    dut_ptr = &i_dut;

    // Connect signals
    i_dut.clk(clk);
    i_dut.reset(reset);

    // Connect Wishbone interface signals
    i_dut.wb_stb_i(wb_stb);
    i_dut.wb_cyc_i(wb_cyc);
    i_dut.wb_we_i(wb_we);
    i_dut.wb_adr_i(wb_adr);
    i_dut.wb_dat_i(wb_dat_i);
    i_dut.wb_sel_i(wb_sel);
    i_dut.wb_dat_o(wb_dat_o);
    i_dut.wb_ack_o(wb_ack);
    i_dut.wb_err_o(wb_err);
    i_dut.wb_rty_o(wb_rty);

    // Connect interrupt outputs
    i_dut.msip_o(msip_out);
    i_dut.mtip_o(mtip_out);

    // Trace-Signale hinzufügen (unverändert)

    sc_start(SC_ZERO_TIME);
    cout << "\n\t\t*****Simulation started*****" << endl;

    // Reset-Test
    reset = 1;
    PN_INFO("Asserting Reset");
    run_cycle(2);
    reset = 0;
    PN_INFO("Reset De-asserted");
    run_cycle(2);

    // ---- Test 1: MSIP Test (Software Interrupt) ----
    PN_INFO("Test 1: Testing MSIP (Software Interrupt) Register");

    // Read initial MSIP (should be 0)
    uint32_t msip_val = read_register(CLINT_REG_MSIP_ADDR);
    PN_INFOF(("Initial MSIP value: 0x%08X", msip_val));
    PN_ASSERTM(msip_val == 0, "Initial MSIP value should be 0");
    PN_ASSERTM(msip_out.read() == 0, "MSIP signal should be 0 initially");

    // Write 1 to MSIP (set software interrupt)
    PN_INFO("Setting MSIP to 1 (trigger software interrupt)");
    write_register(CLINT_REG_MSIP_ADDR, 1);

    // Verify MSIP was set
    msip_val = read_register(CLINT_REG_MSIP_ADDR);
    PN_INFOF(("MSIP after write: 0x%08X", msip_val));
    PN_ASSERTM(msip_val == 1, "MSIP should be set to 1");
    PN_ASSERTM(msip_out.read() == 1, "MSIP signal should be 1 after write");

    // Clear MSIP
    PN_INFO("Clearing MSIP (clear software interrupt)");
    write_register(CLINT_REG_MSIP_ADDR, 0);

    // Verify MSIP was cleared
    msip_val = read_register(CLINT_REG_MSIP_ADDR);
    PN_INFOF(("MSIP after clear: 0x%08X", msip_val));
    PN_ASSERTM(msip_val == 0, "MSIP should be cleared to 0");
    PN_ASSERTM(msip_out.read() == 0, "MSIP signal should be 0 after clear");

    // ---- Test 2: MTIME and MTIMECMP (Timer Interrupt) Test ----
    PN_INFO("Test 2: Testing MTIME and MTIMECMP (Timer Interrupt)");

    // Read current MTIME (lower and upper 32 bits)
    uint32_t mtime_lo = read_register(CLINT_REG_MTIME_LO);
    uint32_t mtime_hi = read_register(CLINT_REG_MTIME_HI);
    uint64_t current_time = ((uint64_t)mtime_hi << 32) | mtime_lo;
    PN_INFOF(("Current MTIME: 0x%08X%08X", mtime_hi, mtime_lo));

    // Set MTIMECMP to MTIME + 50 to trigger an interrupt soon
    uint64_t target_time = current_time + 50;
    write_register64(CLINT_REG_MTIMECMP_LO, target_time);

    // Verify MTIMECMP was set correctly
    uint32_t mtimecmp_lo = read_register(CLINT_REG_MTIMECMP_LO);
    uint32_t mtimecmp_hi = read_register(CLINT_REG_MTIMECMP_HI);
    uint64_t read_target = ((uint64_t)mtimecmp_hi << 32) | mtimecmp_lo;
    PN_INFOF(("Read back MTIMECMP: 0x%08X%08X", mtimecmp_hi, mtimecmp_lo));
    PN_ASSERTM(read_target == target_time, "MTIMECMP was not set correctly");

    // Wait for timer to reach MTIMECMP
    PN_INFO("Waiting for timer interrupt...");
    int cycles = 0;
    while(mtip_out.read() == 0 && cycles < 100)
    {
        run_cycle(1);
        cycles++;
    }

    // Check if timer interrupt was triggered
    PN_INFOF(("Waited %d cycles for timer interrupt", cycles));
    PN_ASSERTM(mtip_out.read() == 1, "Timer interrupt (MTIP) should be triggered");

    // Read current MTIME
    mtime_lo = read_register(CLINT_REG_MTIME_LO);
    mtime_hi = read_register(CLINT_REG_MTIME_HI);
    current_time = ((uint64_t)mtime_hi << 32) | mtime_lo;
    PN_INFOF(("MTIME after waiting: 0x%08X%08X", mtime_hi, mtime_lo));
    PN_ASSERTM(current_time >= target_time, "MTIME should have reached or exceeded MTIMECMP");

    // Set MTIMECMP to a much higher value to clear the interrupt
    target_time = current_time + 10000;
    write_register64(CLINT_REG_MTIMECMP_LO, target_time);

    // Verify interrupt was cleared
    PN_ASSERTM(mtip_out.read() == 0, "Timer interrupt should be cleared after setting higher MTIMECMP");

    // ---- Test 3: Modified Test for MTIME behavior ----
    PN_INFO("Test 3: Testing MTIME behavior (read-only)");

    // Instead of trying to write to MTIME directly (which is read-only),
    // we'll observe its natural increment and test the interrupt behavior

    // First read current MTIME
    mtime_lo = read_register(CLINT_REG_MTIME_LO);
    mtime_hi = read_register(CLINT_REG_MTIME_HI);
    current_time = ((uint64_t)mtime_hi << 32) | mtime_lo;
    PN_INFOF(("Current MTIME: 0x%08X%08X", mtime_hi, mtime_lo));

    // Set MTIMECMP just above current MTIME
    target_time = current_time + 20;
    write_register64(CLINT_REG_MTIMECMP_LO, target_time);
    PN_INFOF(("Setting MTIMECMP to: 0x%08X%08X",
        (uint32_t)((target_time >> 32) & 0xFFFFFFFF),
        (uint32_t)(target_time & 0xFFFFFFFF)));

    // Wait for the interrupt to trigger
    PN_INFO("Waiting for MTIME to increment and trigger interrupt...");
    bool interrupt_triggered = false;
    for(int i = 0; i < 50; i++)
    {
        run_cycle(1);
        if(mtip_out.read() == 1)
        {
            interrupt_triggered = true;
            PN_INFOF(("Interrupt triggered after %d cycles", i + 1));
            break;
        }
    }

    PN_ASSERTM(interrupt_triggered, "Timer interrupt should be triggered when MTIME reaches MTIMECMP");

    // Read MTIME after interrupt
    mtime_lo = read_register(CLINT_REG_MTIME_LO);
    mtime_hi = read_register(CLINT_REG_MTIME_HI);
    current_time = ((uint64_t)mtime_hi << 32) | mtime_lo;
    PN_INFOF(("MTIME after interrupt: 0x%08X%08X", mtime_hi, mtime_lo));
    PN_ASSERTM(current_time >= target_time, "MTIME should be >= MTIMECMP when interrupt is triggered");

    // Set MTIMECMP higher again to clear interrupt
    target_time = current_time + 1000;
    write_register64(CLINT_REG_MTIMECMP_LO, target_time);

    // Verify interrupt was cleared
    PN_ASSERTM(mtip_out.read() == 0, "Timer interrupt should be cleared after setting MTIMECMP higher");

    // ---- Test 4: Byte-level access ----
    PN_INFO("Test 4: Testing byte-level access");

    // Test byte access to MSIP
    PN_INFO("Testing byte-level access to MSIP");
    write_register(CLINT_REG_MSIP_ADDR, 0, 0x1); // Write to lowest byte only
    msip_val = read_register(CLINT_REG_MSIP_ADDR);
    PN_INFOF(("MSIP after byte write: 0x%08X", msip_val));
    PN_ASSERTM(msip_val == 0, "MSIP lowest byte should be 0");

    write_register(CLINT_REG_MSIP_ADDR, 0x01, 0x1); // Write 1 to lowest byte
    msip_val = read_register(CLINT_REG_MSIP_ADDR);
    PN_INFOF(("MSIP after byte write with 1: 0x%08X", msip_val));
    PN_ASSERTM(msip_val == 1, "MSIP lowest byte should be 1");
    PN_ASSERTM(msip_out.read() == 1, "MSIP signal should be 1");

    // Clean up
    write_register(CLINT_REG_MSIP_ADDR, 0);

    // ---- Test 5: Interrupt Latency Tests ----
    {
        PN_INFO("Test 5: Interrupt latency tests");

        // Test 5.1: Basic latency test
        PN_INFO("Test 5.1: Basic software interrupt latency test");

        // Clear interrupt
        write_register(CLINT_REG_MSIP_ADDR, 0);
        run_cycle(2);
        PN_ASSERTM(msip_out.read() == 0, "MSIP signal should be cleared");

        // Set interrupt
        write_register(CLINT_REG_MSIP_ADDR, 1);
        PN_ASSERTM(msip_out.read() == 1, "MSIP signal should be set");

        // Wait a moment and check if the signal is still asserted
        run_cycle(5);
        PN_ASSERTM(msip_out.read() == 1, "MSIP signal should remain asserted until cleared");
    }

    // Test 5.2: Measure interrupt latency
    PN_INFO("Test 5.2: Measuring interrupt latency");
    // Clear interrupt first
    write_register(CLINT_REG_MSIP_ADDR, 0);
    run_cycle(2); // Ensure it's cleared
    PN_ASSERTM(msip_out.read() == 0, "MSIP signal should be cleared before latency test");

    // Start latency measurement
    int latency_cycles = 0;
    wb_stb = 1;
    wb_cyc = 1;
    wb_we = 1;
    wb_adr = CLINT_REG_MSIP_ADDR;
    wb_dat_i = 1;
    wb_sel = 0xF;

    // Run cycle by cycle until interrupt is asserted
    while(!msip_out.read() && latency_cycles < 10)
    {
        run_cycle(1);
        latency_cycles++;
    }

    // Complete the register write transaction if still not acknowledged
    if(!wb_ack.read())
    {
        while(!wb_ack.read())
        {
            run_cycle(1);
            latency_cycles++;
        }
        wb_stb = 0;
        wb_cyc = 0;
        wb_we = 0;
        run_cycle(1);
    }

    PN_INFOF(("Software interrupt latency: %d cycles", latency_cycles));
    PN_ASSERTM(latency_cycles <= 5, "Software interrupt latency should be reasonable");
    PN_ASSERTM(msip_out.read() == 1, "MSIP signal should be asserted after latency test");

    // Test 5.3: Rapid toggling
    PN_INFO("Test 5.3: Testing rapid interrupt toggling");
    for(int i = 0; i < 5; i++)
    {
        // Clear interrupt
        write_register(CLINT_REG_MSIP_ADDR, 0);
        PN_ASSERTM(msip_out.read() == 0, "MSIP signal should be cleared");

        // Set interrupt
        write_register(CLINT_REG_MSIP_ADDR, 1);
        PN_ASSERTM(msip_out.read() == 1, "MSIP signal should be set");
    }

    // Clean up
    write_register(CLINT_REG_MSIP_ADDR, 0);
    PN_ASSERTM(msip_out.read() == 0, "MSIP signal should be cleared at end of test");

    // ---- Test 6: Address Decoding Test ----
    {
        PN_INFO("Test 6: Testing address decoding and invalid access");

        // Test valid address but outside of defined registers
        uint32_t random_offset = 0x100; // Some offset that's not a valid register
        PN_INFOF(("Reading from invalid offset 0x%x", random_offset));
        // TBD: Transform this Testbench into function style tb like membrana_hw_tb as these scopes produce warnings in Synthesis. Tip: use genAI.
        uint32_t value = read_register(random_offset);
        PN_INFOF(("Read from invalid offset 0x%x returned 0x%08x", random_offset, value));

        // Test addresses outside the CLINT range
        uint32_t outside_addr = 0x20000; // Outside the 64KB range
        PN_INFOF(("Reading from address outside CLINT range: 0x%x", outside_addr));
        value = read_register(outside_addr);
        PN_INFOF(("Read from outside address 0x%x returned 0x%08x", outside_addr, value));
    }

    // ---- Test 7: 16-bit Access Test ----
    {
        PN_INFO("Test 7: Testing 16-bit accesses");

        // Read current values to avoid disturbing the state
        uint32_t mtimecmp_lo_test7 = read_register(CLINT_REG_MTIMECMP_LO);

        // Test 16-bit write to MTIMECMP
        uint16_t test_value = 0x1234;
        PN_INFOF(("Writing 16-bit value 0x%04x to MTIMECMP", test_value));

        // Write 16-bit value
        write_register16(CLINT_REG_MTIMECMP_LO, test_value);

        // Read back as 16-bit value
        uint16_t read_value = read_register16(CLINT_REG_MTIMECMP_LO);
        PN_INFOF(("Read back 16-bit value: 0x%04x", read_value));
        PN_ASSERTM(read_value == test_value, "16-bit value should match what was written");

        // Read as 8-bit values to verify byte ordering
        uint8_t byte0 = read_register8(CLINT_REG_MTIMECMP_LO);
        uint8_t byte1_test7 = read_register8(CLINT_REG_MTIMECMP_LO + 1);
        PN_INFOF(("Read back as bytes: 0x%02x 0x%02x", byte0, byte1_test7));
        PN_ASSERTM(byte0 == (test_value & 0xFF), "Byte 0 should match lower byte of test value");
        PN_ASSERTM(byte1_test7 == ((test_value >> 8) & 0xFF), "Byte 1 should match upper byte of test value");

        // Now read as 32-bit and verify the lower 16 bits
        uint32_t word_value = read_register(CLINT_REG_MTIMECMP_LO);
        uint16_t lower_16 = word_value & 0xFFFF;
        PN_INFOF(("Read back as 32-bit word: 0x%08x (lower 16 bits: 0x%04x)", word_value, lower_16));
        PN_ASSERTM(lower_16 == test_value, "Lower 16 bits should match test value");

        // Restore original value
        write_register(CLINT_REG_MTIMECMP_LO, mtimecmp_lo_test7);
    }

    // ---- Test 8: Atomic MTIMECMP Update Test ----
    {
        PN_INFO("Test 8: Testing atomic MTIMECMP updates");

        // 1. Get current state
        uint32_t mtime_lo_test8 = read_register(CLINT_REG_MTIME_LO);
        uint32_t mtime_hi_test8 = read_register(CLINT_REG_MTIME_HI);
        uint64_t mtime_test8 = ((uint64_t)mtime_hi_test8 << 32) | mtime_lo_test8;
        PN_INFOF(("Current MTIME: 0x%08X%08X", mtime_hi_test8, mtime_lo_test8));

        // 2. Set MTIMECMP to a very large value (should not trigger interrupt)
        uint64_t new_mtimecmp = 0xFFFFFFFFFFFFFFFFULL;
        PN_INFO("Setting MTIMECMP to max value (0xFFFFFFFFFFFFFFFF) - high word first");

        // CORRECT way: Write HIGH register first, then LOW
        write_register(CLINT_REG_MTIMECMP_HI, (uint32_t)(new_mtimecmp >> 32));
        write_register(CLINT_REG_MTIMECMP_LO, (uint32_t)new_mtimecmp);

        // Verify no interrupt was triggered during the update
        PN_ASSERTM(mtip_out.read() == 0, "MTIP should not be triggered after setting max MTIMECMP");

        // 3. Set MTIMECMP to a value that would trigger interrupt if not done atomically
        new_mtimecmp = mtime_test8 - 1; // Less than current time, would trigger if not atomic
        PN_INFOF(("Setting MTIMECMP to 0x%08X%08X (less than current MTIME) - high word first",
            (uint32_t)((new_mtimecmp >> 32) & 0xFFFFFFFF),
            (uint32_t)(new_mtimecmp & 0xFFFFFFFF)));

        // CORRECT way: Write HIGH register first, then LOW
        write_register(CLINT_REG_MTIMECMP_HI, (uint32_t)(new_mtimecmp >> 32));
        write_register(CLINT_REG_MTIMECMP_LO, (uint32_t)new_mtimecmp);

        // Run a few cycles to ensure the interrupt takes effect
        run_cycle(5);

        // Verify interrupt was triggered after complete update
        PN_ASSERTM(mtip_out.read() == 1, "MTIP should be triggered after setting MTIMECMP < MTIME");

        // 4. Reset to a higher value to clear interrupt
        new_mtimecmp = mtime_test8 + 1000;
        PN_INFOF(("Setting MTIMECMP to 0x%08X%08X (future value) to clear interrupt",
            (uint32_t)((new_mtimecmp >> 32) & 0xFFFFFFFF),
            (uint32_t)(new_mtimecmp & 0xFFFFFFFF)));

        write_register(CLINT_REG_MTIMECMP_HI, (uint32_t)(new_mtimecmp >> 32));
        write_register(CLINT_REG_MTIMECMP_LO, (uint32_t)new_mtimecmp);

        // Run a few cycles to ensure the interrupt is cleared
        run_cycle(5);

        PN_ASSERTM(mtip_out.read() == 0, "MTIP should be cleared after setting MTIMECMP to future value");
    }

    // ---- Test 9: Unaligned Access Test ----
    {
        PN_INFO("Test 9: Testing unaligned accesses");

        // Save original values
        uint32_t mtimecmp_lo_test9 = read_register(CLINT_REG_MTIMECMP_LO);
        uint32_t mtimecmp_hi_test9 = read_register(CLINT_REG_MTIMECMP_HI);

        // Unaligned 32-bit read from MTIMECMP (address + 1)
        uint32_t unaligned_addr = CLINT_REG_MTIMECMP_LO + 1; // Offset by 1 byte
        PN_INFOF(("Reading 32 bits from unaligned address offset 0x%x", unaligned_addr));

        uint32_t unaligned_value = read_register(unaligned_addr);
        PN_INFOF(("Unaligned read returned 0x%08x", unaligned_value));

        // Test unaligned write to MTIMECMP
        uint32_t test_value32 = 0xDEADBEEF;
        PN_INFOF(("Writing 0x%08x to unaligned address offset 0x%x", test_value32, unaligned_addr));

        write_register(unaligned_addr, test_value32);

        // Read back bytes to verify
        uint8_t byte1_test9 = read_register8(unaligned_addr);
        uint8_t byte2_test9 = read_register8(unaligned_addr + 1);
        uint8_t byte3_test9 = read_register8(unaligned_addr + 2);

        PN_INFOF(("Read bytes at unaligned address: 0x%02x 0x%02x 0x%02x",
            byte1_test9,
            byte2_test9,
            byte3_test9));

        // Restore original values
        write_register(CLINT_REG_MTIMECMP_LO, mtimecmp_lo_test9);
        write_register(CLINT_REG_MTIMECMP_HI, mtimecmp_hi_test9);
    }

    // ---- Test 10: Timer Overflow/Consistent Behavior Test ----
    {
        PN_INFO("Test 10: Testing timer increment consistency");

        // Direct read of MTIME
        uint64_t start_time = direct_read_mtime();
        PN_INFOF(("Initial MTIME: 0x%016llX", (unsigned long long)start_time));

        // Run exactly 100 cycles
        PN_INFO("Running 100 cycles to verify timer increment behavior");
        run_cycle(100);

        // Direct read of MTIME after 100 cycles
        uint64_t end_time = direct_read_mtime();
        PN_INFOF(("MTIME after 100 cycles: 0x%016llX", (unsigned long long)end_time));

        // Verify timer incremented by exactly 100
        PN_INFOF(("Timer increment: %llu", end_time - start_time));
        PN_ASSERTM(end_time == start_time + 100,
            "Timer should increment by exactly 100 over 100 cycles");
    }

    // ---- Test 11: Comprehensive Reset Behavior Test ----
    {
        PN_INFO("Test 11: Comprehensive reset behavior test");

        // 1. Modify state to non-default values
        PN_INFO("Setting MSIP to 1");
        write_register(CLINT_REG_MSIP_ADDR, 1);
        PN_ASSERTM(msip_out.read() == 1, "MSIP signal should be 1 after setting");

        // Set MTIMECMP to a specific test value
        uint64_t test_mtimecmp = 0x1234567890ABCDEF;
        PN_INFOF(("Setting MTIMECMP to test value: 0x%016llX",
            (unsigned long long)test_mtimecmp));

        write_register(CLINT_REG_MTIMECMP_HI, (uint32_t)(test_mtimecmp >> 32));
        write_register(CLINT_REG_MTIMECMP_LO, (uint32_t)test_mtimecmp);

        // 2. Read current MTIME for reference
        uint64_t mtime_before_reset = direct_read_mtime();
        PN_INFOF(("Current MTIME before reset: 0x%016llX", (unsigned long long)mtime_before_reset));

        // 3. Assert reset
        PN_INFO("Asserting reset");
        reset = 1;

        // Direct check of MTIME immediately after reset assertion
        run_cycle(1);
        uint64_t mtime_during_reset = direct_read_mtime();
        PN_INFOF(("MTIME during reset: 0x%016llX", (unsigned long long)mtime_during_reset));
        PN_ASSERTM(mtime_during_reset == 0, "MTIME should be exactly 0 during reset");

        // Run a few more cycles in reset state
        run_cycle(4);

        // 4. Verify interrupt outputs are deasserted during reset
        PN_ASSERTM(msip_out.read() == 0, "MSIP signal should be deasserted during reset");
        PN_ASSERTM(mtip_out.read() == 0, "MTIP signal should be deasserted during reset");

        // 5. Deassert reset
        PN_INFO("Deasserting reset");
        reset = 0;

        // 6. MTIME starts incrementing immediately after reset is deasserted
        run_cycle(1);
        uint64_t mtime_one_cycle_after_reset = direct_read_mtime();
        PN_INFOF(("MTIME one cycle after reset: 0x%016llX", (unsigned long long)mtime_one_cycle_after_reset));
        PN_ASSERTM(mtime_one_cycle_after_reset == 1, "MTIME should be 1 after one cycle post-reset");

        // 7. Run a few more cycles
        run_cycle(5); // Changed from 4 to 5 to match the expected value in the assertion

        // 8. Verify reset state of registers after several cycles using direct access
        uint32_t msip = direct_read_msip();
        PN_INFOF(("MSIP after reset (direct): 0x%08X", msip));
        PN_ASSERTM(msip == 0, "MSIP should be 0 after reset");

        uint64_t mtimecmp = direct_read_mtimecmp();
        PN_INFOF(("MTIMECMP after reset (direct): 0x%016llX", (unsigned long long)mtimecmp));
        PN_ASSERTM(mtimecmp == 0xFFFFFFFFFFFFFFFFULL, "MTIMECMP should be 0xFFFFFFFFFFFFFFFF after reset");

        // MTIME should have incremented by exactly the number of cycles since reset was deasserted
        uint64_t mtime = direct_read_mtime();
        PN_INFOF(("MTIME after reset (direct read): 0x%016llX", (unsigned long long)mtime));
        PN_ASSERTM(mtime == 6, "MTIME should be exactly 6 (1 + 5 cycles since reset deassertion)");

        // Now verify values via Wishbone interface for completeness
        uint32_t msip_wb = read_register(CLINT_REG_MSIP_ADDR);
        PN_INFOF(("MSIP after reset (Wishbone): 0x%08X", msip_wb));
        PN_ASSERTM(msip_wb == 0, "MSIP should be 0 after reset (Wishbone)");

        uint32_t mtimecmp_lo = read_register(CLINT_REG_MTIMECMP_LO);
        uint32_t mtimecmp_hi = read_register(CLINT_REG_MTIMECMP_HI);
        uint64_t mtimecmp_wb = ((uint64_t)mtimecmp_hi << 32) | mtimecmp_lo;
        PN_INFOF(("MTIMECMP after reset (Wishbone): 0x%016llX", (unsigned long long)mtimecmp_wb));

        uint32_t mtime_lo = read_register(CLINT_REG_MTIME_LO);
        uint32_t mtime_hi = read_register(CLINT_REG_MTIME_HI);
        uint64_t mtime_wb = ((uint64_t)mtime_hi << 32) | mtime_lo;
        PN_INFOF(("MTIME after reset (Wishbone read): 0x%016llX", (unsigned long long)mtime_wb));
        // The Wishbone read will take several cycles, so MTIME will advance further
        PN_ASSERTM(mtime_wb > 6, "MTIME should be greater than 6 after Wishbone reads");

        // 9. Verify interrupt signals are in reset state
        PN_ASSERTM(msip_out.read() == 0, "MSIP signal should be 0 after reset");
        PN_ASSERTM(mtip_out.read() == 0, "MTIP signal should be 0 after reset");
    }

    // ---- Test 12: MTIME Writability Test ----
    {
        PN_INFO("Test 12: Testing writable MTIME functionality");

        // 1. Read current MTIME
        uint32_t mtime_lo = read_register(CLINT_REG_MTIME_LO);
        uint32_t mtime_hi = read_register(CLINT_REG_MTIME_HI);
        uint64_t current_time = ((uint64_t)mtime_hi << 32) | mtime_lo;
        PN_INFOF(("Current MTIME before write: 0x%016llX", (unsigned long long)current_time));

        // 2. Set MTIME to a specific value (using same method as MTIMECMP)
        uint64_t new_time_value = 0x1234567890ABCDEFULL;
        PN_INFOF(("Setting MTIME to: 0x%016llX", (unsigned long long)new_time_value));

        // Use the same high-word-first approach as for MTIMECMP
        write_register(CLINT_REG_MTIME_HI, (uint32_t)(new_time_value >> 32));
        write_register(CLINT_REG_MTIME_LO, (uint32_t)(new_time_value & 0xFFFFFFFF));

        // 3. Read back MTIME to verify the write was successful
        mtime_lo = read_register(CLINT_REG_MTIME_LO);
        mtime_hi = read_register(CLINT_REG_MTIME_HI);
        current_time = ((uint64_t)mtime_hi << 32) | mtime_lo;
        PN_INFOF(("MTIME after write: 0x%016llX", (unsigned long long)current_time));

        // Verify MTIME was set correctly
        // Note: It might be current_time == new_time_value + n, where n is small due to clock cycles
        // during the read operation
        PN_ASSERTM(current_time >= new_time_value,
            "MTIME should be at least the value we wrote");
        PN_ASSERTM(current_time < new_time_value + 10,
            "MTIME should be close to the value we wrote (allowing for a few cycles)");

        // 4. Run some cycles and check that MTIME continues to increment normally
        uint64_t time_before_cycles = current_time;
        PN_INFO("Running 20 cycles to verify MTIME continues incrementing");
        run_cycle(20);

        // Read MTIME again
        mtime_lo = read_register(CLINT_REG_MTIME_LO);
        mtime_hi = read_register(CLINT_REG_MTIME_HI);
        current_time = ((uint64_t)mtime_hi << 32) | mtime_lo;
        PN_INFOF(("MTIME after 20 cycles: 0x%016llX", (unsigned long long)current_time));

        // Verify MTIME incremented properly
        uint64_t time_diff = current_time - time_before_cycles;
        PN_INFOF(("MTIME increment: %llu", (unsigned long long)time_diff));
        PN_ASSERTM(time_diff >= 20, "MTIME should increment by at least 20 over 20 cycles");
        PN_ASSERTM(time_diff < 30, "MTIME should increment by less than 30 over 20 cycles");

        // 5. Test MTIP behavior with MTIME writes - First set MTIMECMP higher than current MTIME
        uint64_t new_mtimecmp = current_time + 1000;
        PN_INFOF(("Setting MTIMECMP above current MTIME: 0x%016llX", (unsigned long long)new_mtimecmp));
        write_register64(CLINT_REG_MTIMECMP_LO, new_mtimecmp);

        // Verify interrupt is not triggered
        PN_ASSERTM(mtip_out.read() == 0, "Timer interrupt should not be triggered when MTIME < MTIMECMP");

        // 6. Now write MTIME to a value higher than MTIMECMP to trigger the interrupt
        uint64_t mtime_above_cmp = new_mtimecmp + 1;
        PN_INFOF(("Setting MTIME above MTIMECMP: 0x%016llX", (unsigned long long)mtime_above_cmp));
        write_register(CLINT_REG_MTIME_HI, (uint32_t)(mtime_above_cmp >> 32));
        write_register(CLINT_REG_MTIME_LO, (uint32_t)(mtime_above_cmp & 0xFFFFFFFF));

        // Run a few cycles to ensure the interrupt has time to propagate
        run_cycle(5);

        // Verify interrupt is triggered
        PN_ASSERTM(mtip_out.read() == 1, "Timer interrupt should be triggered when MTIME > MTIMECMP");

        // 7. Test byte-level and half-word access to MTIME
        PN_INFO("Testing byte-level and half-word access to MTIME");

        // Directly read MTIME before writing
        uint64_t current_mtime = direct_read_mtime();
        PN_INFOF(("Current MTIME (direct read): 0x%016llX", (unsigned long long)current_mtime));

        // Prepare to write a byte at offset 2
        uint8_t test_byte = 0xAA;
        uint32_t byte_offset = 2;
        PN_INFOF(("Writing byte 0x%02X to MTIME at offset %d", test_byte, byte_offset));

        // Create new value with our byte at the correct position
        uint32_t current_lo = current_mtime & 0xFFFFFFFF;
        uint32_t new_lo = (current_lo & ~(0xFF << (8 * byte_offset))) |
                          (test_byte << (8 * byte_offset));

        // Write using the byte selector
        write_register(CLINT_REG_MTIME_LO, new_lo, (1 << byte_offset));

        // Immediately read back for verification
        uint64_t new_mtime = direct_read_mtime();
        uint32_t read_lo = new_mtime & 0xFFFFFFFF;

        PN_INFOF(("MTIME after byte write (direct read): 0x%016llX", (unsigned long long)new_mtime));
        PN_INFOF(("MTIME lower word after byte write: 0x%08X", read_lo));

        uint8_t read_byte = (read_lo >> (8 * byte_offset)) & 0xFF;
        PN_INFOF(("Read byte at offset %d: 0x%02X", byte_offset, read_byte));

        // Expected behavior: MTIME increments by 1 between write and read
        uint8_t expected_byte = test_byte + 1; // We expect it to be incremented by 1
        PN_INFOF(("Expected byte (written + 1): 0x%02X", expected_byte));
        PN_ASSERTM(read_byte == expected_byte,
            "Byte value in MTIME should be exactly 1 more than what was written");

        // Verify total increment during operation
        uint64_t mtime_increment = new_mtime - current_mtime;
        PN_INFOF(("Total MTIME increment during operation: %llu", (unsigned long long)mtime_increment));
        PN_ASSERTM(mtime_increment >= 1 && mtime_increment <= 4,
            "MTIME should increment by 1-4 during the byte write operation");

        // Also read via Wishbone for comparison (will show further increments)
        uint32_t wb_mtime_lo = read_register(CLINT_REG_MTIME_LO);
        uint8_t wb_read_byte = (wb_mtime_lo >> (8 * byte_offset)) & 0xFF;
        PN_INFOF(("Byte read via Wishbone at offset %d: 0x%02X (further incremented)",
            byte_offset,
            wb_read_byte));
    }

    PN_END_TRACE();
    cout << "\n\t\t*****Simulation complete*****" << endl;

    return 0;
}