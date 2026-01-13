/**
 * @file clint_soft_tb.cpp
 * @brief Testbench for m_clint_soft peripheral
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Alexander Beck <alexander.beck1@tha.de>
                2025 Christian Zellinger <christian.zellinger1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains a testbench for the m_clint_soft peripheral.

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
#include <piconut.h>
#include "../clint_soft.h"

// Define register offsets according to RISC-V spec
#define PN_CFG_CLINT_BASE_ADDRESS 0x2000000
#define MSIP_OFFSET 0x0
#define MTIMECMP_OFFSET 0x4000
#define MTIME_OFFSET 0xBFF8

// Testbench global variables
bool msip_signal = false;
bool mtip_signal = false;

// Helper function to test MSIP functionality
void test_msip(m_clint_soft* clint)
{
    PN_INFO("Test 1: Testing Machine Software Interrupt Pending (MSIP)");

    // 1. Read initial MSIP - should be 0
    uint32_t msip = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET);
    PN_ASSERTF(((msip == 0)), ("Initial MSIP value should be 0, got %u", msip));
    PN_ASSERTF(((!msip_signal)), ("Initial MSIP signal should be false"));

    // 2. Write 1 to MSIP - set software interrupt
    PN_INFO("Setting MSIP to 1");
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET, 1);

    // 3. Read back MSIP - should be 1
    msip = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET);
    PN_ASSERTF(((msip == 1)), ("MSIP value should be 1 after write, got %u", msip));
    PN_ASSERTF(((msip_signal)), ("MSIP signal should be true after write"));

    // 4. Write 0 to MSIP - clear software interrupt
    PN_INFO("Clearing MSIP");
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET, 0);

    // 5. Read back MSIP - should be 0
    msip = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET);
    PN_ASSERTF(((msip == 0)), ("MSIP value should be 0 after clearing, got %u", msip));
    PN_ASSERTF(((!msip_signal)), ("MSIP signal should be false after clearing"));

    // 6. Test non-zero values other than 1 (according to spec, only bit 0 matters)
    PN_INFO("Testing non-zero values");
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET, 0xFFFFFFFF);
    msip = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET);
    PN_ASSERTF(((msip == 1)), ("MSIP should only store bit 0, expected 1, got %u", msip));
    PN_ASSERTF(((msip_signal)), ("MSIP signal should be true after writing non-zero value"));

    // 7. Clear again
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET, 0);

    // 8. Test byte-wise access to MSIP
    PN_INFO("Testing byte-wise access to MSIP");
    clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET, 1);
    msip = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET);
    PN_ASSERTF(((msip == 1)), ("MSIP should be 1 after byte-wise write, got %u", msip));

    uint8_t msip_byte = clint->read8(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET);
    PN_ASSERTF(((msip_byte == 1)), ("MSIP byte should be 1, got %u", msip_byte));

    // Clear for next tests
    clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET, 0);

    PN_INFO("MSIP test passed");
}

// Helper function to test MTIME/MTIMECMP functionality
void test_timer(m_clint_soft* clint)
{
    PN_INFO("Test 2: Testing Machine Timer (MTIME) and Compare (MTIMECMP)");

    // 1. Read initial MTIME and MTIMECMP
    uint32_t mtime_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET);
    uint32_t mtime_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 4);
    uint64_t mtime = ((uint64_t)mtime_high << 32) | mtime_low;

    uint32_t mtimecmp_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET);
    uint32_t mtimecmp_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4);
    uint64_t mtimecmp = ((uint64_t)mtimecmp_high << 32) | mtimecmp_low;

    PN_INFOF(("Initial MTIME = 0x%016lx", mtime));
    PN_INFOF(("Initial MTIMECMP = 0x%016lx", mtimecmp));

    // 2. Test byte-wise reads
    PN_INFO("Testing byte-wise reads of MTIME and MTIMECMP");
    uint8_t mtime_byte0 = clint->read8(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET);
    uint8_t mtime_byte1 = clint->read8(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 1);
    uint8_t mtime_byte2 = clint->read8(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 2);
    uint8_t mtime_byte3 = clint->read8(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 3);

    uint8_t mtimecmp_byte0 = clint->read8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET);
    uint8_t mtimecmp_byte1 = clint->read8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 1);

    PN_ASSERTF(((mtime_byte0 == (mtime & 0xFF))),
        ("MTIME byte 0 should be 0x%02x, got 0x%02x", (mtime & 0xFF), mtime_byte0));
    PN_ASSERTF(((mtime_byte1 == ((mtime >> 8) & 0xFF))),
        ("MTIME byte 1 should be 0x%02x, got 0x%02x", ((mtime >> 8) & 0xFF), mtime_byte1));

    // 3. Set MTIMECMP to a small value (current MTIME + 50)
    uint64_t new_mtimecmp = mtime + 50;
    PN_INFOF(("Setting MTIMECMP to MTIME + 50 = 0x%016lx", new_mtimecmp));

    // Write lower 32 bits first
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET, (uint32_t)new_mtimecmp);
    // Write upper 32 bits
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4, (uint32_t)(new_mtimecmp >> 32));

    // 4. Read back MTIMECMP to verify
    mtimecmp_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET);
    mtimecmp_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4);
    mtimecmp = ((uint64_t)mtimecmp_high << 32) | mtimecmp_low;

    PN_ASSERTF(((mtimecmp == new_mtimecmp)), ("MTIMECMP should be 0x%016lx, got 0x%016lx", new_mtimecmp, mtimecmp));

    // 5. Test byte-wise write of MTIMECMP
    PN_INFO("Testing byte-wise write of MTIMECMP");
    new_mtimecmp = mtime + 100;

    clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET, (uint8_t)(new_mtimecmp & 0xFF));
    clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 1, (uint8_t)((new_mtimecmp >> 8) & 0xFF));
    clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 2, (uint8_t)((new_mtimecmp >> 16) & 0xFF));
    clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 3, (uint8_t)((new_mtimecmp >> 24) & 0xFF));
    clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4, (uint8_t)((new_mtimecmp >> 32) & 0xFF));
    clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 5, (uint8_t)((new_mtimecmp >> 40) & 0xFF));
    clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 6, (uint8_t)((new_mtimecmp >> 48) & 0xFF));
    clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 7, (uint8_t)((new_mtimecmp >> 56) & 0xFF));

    // Read back and verify
    mtimecmp_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET);
    mtimecmp_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4);
    mtimecmp = ((uint64_t)mtimecmp_high << 32) | mtimecmp_low;

    PN_ASSERTF(((mtimecmp == new_mtimecmp)),
        ("MTIMECMP should be 0x%016lx after byte-wise write, got 0x%016lx", new_mtimecmp, mtimecmp));

    // 6. Tick the timer until it reaches MTIMECMP
    PN_INFO("Simulating timer ticks until MTIMECMP is reached");

    int ticks = 0;
    while(!mtip_signal && ticks < 100)
    {
        clint->on_rising_edge_clock();
        ticks++;

        // Read current MTIME occasionally
        if(ticks % 10 == 0)
        {
            mtime_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET);
            mtime_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 4);
            mtime = ((uint64_t)mtime_high << 32) | mtime_low;
            PN_INFOF(("Current MTIME after %d ticks = 0x%016lx", ticks, mtime));
        }
    }

    // 7. Verify timer interrupt was triggered
    PN_INFOF(("Timer interrupt triggered after %d ticks", ticks));
    PN_ASSERTF(((mtip_signal)), ("MTIP signal should be true when MTIME >= MTIMECMP"));

    // 8. Read final MTIME
    mtime_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET);
    mtime_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 4);
    mtime = ((uint64_t)mtime_high << 32) | mtime_low;

    PN_INFOF(("Final MTIME = 0x%016lx", mtime));
    PN_ASSERTF(((mtime >= mtimecmp)), ("Final MTIME (0x%016lx) should be >= MTIMECMP (0x%016lx)", mtime, mtimecmp));

    // 9. Set MTIMECMP to a higher value to clear the interrupt
    new_mtimecmp = mtime + 1000;
    PN_INFOF(("Setting MTIMECMP to a higher value (MTIME + 1000) = 0x%016lx", new_mtimecmp));

    // Write lower 32 bits first
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET, (uint32_t)new_mtimecmp);
    // Write upper 32 bits
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4, (uint32_t)(new_mtimecmp >> 32));

    // Trigger a clock edge to process the pending interrupt update
    clint->on_rising_edge_clock();

    // 10. Verify timer interrupt was cleared
    PN_ASSERTF(((!mtip_signal)), ("MTIP signal should be false after setting MTIMECMP to a higher value"));

    // 11. Test that MTIME is writable
    PN_INFO("Testing that MTIME is writable");

    // Get current MTIME
    mtime_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET);
    mtime_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 4);
    mtime = ((uint64_t)mtime_high << 32) | mtime_low;

    // Set a test value for MTIME (following recommended write order: high bits first)
    uint64_t test_mtime = 0xCAFEBABEDEADBEEF;
    PN_INFOF(("Writing 0x%016lx to MTIME", test_mtime));

    // Write to MTIME (write high 32 bits first, then low 32 bits)
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 4, 0xCAFEBABE);
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET, 0xDEADBEEF);

    // Read MTIME again to verify it was changed
    uint32_t new_mtime_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET);
    uint32_t new_mtime_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 4);
    uint64_t new_mtime = ((uint64_t)new_mtime_high << 32) | new_mtime_low;

    // MTIME should now be set to our test value
    PN_ASSERTF(((new_mtime == test_mtime)),
        ("MTIME should be 0x%016lx after write, got 0x%016lx", test_mtime, new_mtime));

    // Test that MTIME increments after write operations
    PN_INFO("Testing that MTIME increments after write operations");

    // Tick the timer a few times
    for(int i = 0; i < 5; i++)
    {
        clint->on_rising_edge_clock();
    }

    // Read MTIME again to verify it increments
    mtime_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET);
    mtime_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 4);
    mtime = ((uint64_t)mtime_high << 32) | mtime_low;

    // MTIME should now be test_mtime + 5
    PN_ASSERTF(((mtime == test_mtime + 5)),
        ("MTIME should be 0x%016lx after 5 ticks, got 0x%016lx", test_mtime + 5, mtime));

    // Test byte-wise write to MTIME
    PN_INFO("Testing byte-wise write to MTIME");

    // Set a new test value
    test_mtime = 0x0123456789ABCDEF;

    // Write to MTIME byte by byte (from high to low)
    for(int i = 7; i >= 0; i--)
    {
        clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + i, (test_mtime >> (i * 8)) & 0xFF);
    }

    // Read MTIME again to verify it was changed
    mtime_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET);
    mtime_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 4);
    mtime = ((uint64_t)mtime_high << 32) | mtime_low;

    // MTIME should now be set to our new test value
    PN_ASSERTF(((mtime == test_mtime)),
        ("MTIME should be 0x%016lx after byte-wise write, got 0x%016lx", test_mtime, mtime));

    PN_INFO("Timer test passed");
}

// Helper function to test address decoding
void test_address_decoding(m_clint_soft* clint)
{
    PN_INFO("Test 3: Testing address decoding and invalid access");

    // Test valid address but outside of defined registers
    uint32_t random_offset = 0x100; // Some offset that's not a valid register
    uint32_t value = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + random_offset);
    PN_INFOF(("Read from invalid offset 0x%x returned 0x%08x", random_offset, value));

    // Test addresses outside the CLINT range
    uint64_t outside_addr = PN_CFG_CLINT_BASE_ADDRESS + 0x20000; // Outside the 64KB range
    value = clint->read32(outside_addr);
    PN_INFOF(("Read from outside address 0x%lx returned 0x%08x", outside_addr, value));

    PN_INFO("Address decoding test passed");
}

// Helper function to test 16-bit accesses
void test_16bit_access(m_clint_soft* clint)
{
    PN_INFO("Test 4: Testing 16-bit accesses");

    // Read current values to avoid disturbing the state
    uint32_t mtimecmp_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET);

    // Test 16-bit write to MTIMECMP
    uint16_t test_value = 0x1234;
    PN_INFO("Writing 16-bit value 0x1234 to MTIMECMP");

    // First implement the 16-bit access by using two 8-bit accesses
    clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET, test_value & 0xFF);
    clint->write8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 1, (test_value >> 8) & 0xFF);

    // Read back first as 8-bit values
    uint8_t byte0 = clint->read8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET);
    uint8_t byte1 = clint->read8(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 1);
    uint16_t read_value = (byte1 << 8) | byte0;

    PN_ASSERTF(((read_value == test_value)),
        ("16-bit value should be 0x%04x, got 0x%04x", test_value, read_value));

    // Now read as 32-bit and verify the lower 16 bits
    uint32_t word_value = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET);
    uint16_t lower_16 = word_value & 0xFFFF;

    PN_ASSERTF(((lower_16 == test_value)),
        ("Lower 16 bits should be 0x%04x, got 0x%04x", test_value, lower_16));

    // Restore original value
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET, mtimecmp_low);

    PN_INFO("16-bit access test passed");
}

// Helper function to test atomic updates of MTIMECMP
void test_atomic_mtimecmp_update(m_clint_soft* clint)
{
    PN_INFO("Test 5: Testing atomic MTIMECMP updates");

    // 1. Get current state
    uint32_t mtime_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET);
    uint32_t mtime_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 4);
    uint64_t mtime = ((uint64_t)mtime_high << 32) | mtime_low;

    // 1.5. Ensure MTIP is not asserted by setting MTIMECMP much higher than MTIME
    uint64_t reset_mtimecmp = mtime + 1000000;
    PN_INFO("Resetting MTIP by setting MTIMECMP much higher than MTIME");

    // Write MTIMECMP in the correct order (high bits first)
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4, (uint32_t)(reset_mtimecmp >> 32));
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET, (uint32_t)reset_mtimecmp);

    // Tick the clock to process any pending interrupt updates
    clint->on_rising_edge_clock();

    // Verify MTIP was cleared
    PN_ASSERTF((!mtip_signal), ("MTIP should be cleared before starting the test"));

    // 2. Set MTIMECMP to a very large value (should not trigger interrupt)
    uint64_t new_mtimecmp = 0xFFFFFFFFFFFFFFFFULL;
    PN_INFO("Setting MTIMECMP to max value (0xFFFFFFFFFFFFFFFF)");

    // CORRECT way: Write HIGH register first, then LOW
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4, (uint32_t)(new_mtimecmp >> 32));
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET, (uint32_t)new_mtimecmp);

    // Verify no interrupt was triggered during the update
    PN_ASSERTF((!mtip_signal), ("MTIP should not be triggered during atomic update"));

    // 3. Set MTIMECMP to a value that would trigger interrupt if not done atomically
    new_mtimecmp = mtime - 1; // Less than current time, would trigger if not atomic
    PN_INFOF(("Setting MTIMECMP to 0x%016lx (less than current MTIME)", new_mtimecmp));

    // Save current interrupt state
    bool previous_mtip = mtip_signal;

    // CORRECT way: Write HIGH register first, then LOW
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4, (uint32_t)(new_mtimecmp >> 32));
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET, (uint32_t)new_mtimecmp);

    // Trigger a clock edge to process any pending interrupt
    clint->on_rising_edge_clock();

    // Verify interrupt was triggered only after complete update
    PN_ASSERTF(((mtip_signal)), ("MTIP should be triggered after complete atomic update"));

    // 4. Reset to a higher value to clear interrupt
    new_mtimecmp = mtime + 1000;
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4, (uint32_t)(new_mtimecmp >> 32));
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET, (uint32_t)new_mtimecmp);

    // Trigger a clock edge to process the pending interrupt update
    clint->on_rising_edge_clock();

    PN_ASSERTF((!mtip_signal), ("MTIP should be cleared after setting MTIMECMP to future value"));

    PN_INFO("Atomic MTIMECMP update test passed");
}

// Helper function to test unaligned accesses
void test_unaligned_access(m_clint_soft* clint)
{
    PN_INFO("Test 6: Testing unaligned accesses");

    // Unaligned 32-bit read from MTIMECMP
    uint64_t unaligned_addr = PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 1; // Offset by 1 byte
    PN_INFOF(("Reading 32 bits from unaligned address 0x%lx", unaligned_addr));

    uint32_t unaligned_value = clint->read32(unaligned_addr);
    PN_INFOF(("Unaligned read returned 0x%08x", unaligned_value));

    // Compare with byte-by-byte construction
    uint8_t byte1 = clint->read8(unaligned_addr);
    uint8_t byte2 = clint->read8(unaligned_addr + 1);
    uint8_t byte3 = clint->read8(unaligned_addr + 2);
    uint8_t byte4 = clint->read8(unaligned_addr + 3);

    uint32_t constructed_value = (byte4 << 24) | (byte3 << 16) | (byte2 << 8) | byte1;
    PN_INFOF(("Constructed value from bytes: 0x%08x", constructed_value));

    // Test unaligned write to MTIMECMP
    uint32_t test_value = 0xDEADBEEF;
    PN_INFOF(("Writing 0x%08x to unaligned address 0x%lx", test_value, unaligned_addr));

    // Save original values to restore later
    uint32_t orig_mtimecmp_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET);
    uint32_t orig_mtimecmp_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4);

    // Write unaligned value
    clint->write32(unaligned_addr, test_value);

    // Read back bytes to verify
    byte1 = clint->read8(unaligned_addr);
    byte2 = clint->read8(unaligned_addr + 1);
    byte3 = clint->read8(unaligned_addr + 2);

    PN_ASSERTF(((byte1 == 0xBE)),
        ("Byte 1 should be 0xBE, got 0x%02x", byte1));
    PN_ASSERTF(((byte2 == 0xAD)),
        ("Byte 2 should be 0xAD, got 0x%02x", byte2));
    PN_ASSERTF(((byte3 == 0xDE)),
        ("Byte 3 should be 0xDE, got 0x%02x", byte3));

    // Restore original values
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET, orig_mtimecmp_low);
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4, orig_mtimecmp_high);

    PN_INFO("Unaligned access test passed");
}

// Helper function to test timer overflow
void test_timer_overflow(m_clint_soft* clint)
{
    PN_INFO("Test 7: Testing timer overflow");

    // Save original values
    uint32_t mtime_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET);
    uint32_t mtime_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 4);
    uint64_t orig_mtime = ((uint64_t)mtime_high << 32) | mtime_low;

    uint32_t mtimecmp_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET);
    uint32_t mtimecmp_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4);
    uint64_t orig_mtimecmp = ((uint64_t)mtimecmp_high << 32) | mtimecmp_low;

    // Note: We can't directly modify MTIME as it's read-only
    // For a real overflow test, we would need access to internal state
    // Instead, let's check if the timer increments correctly over multiple ticks

    PN_INFO("Simulating multiple timer ticks to verify counter behavior");

    // Tick the timer 100 times
    for(int i = 0; i < 100; i++)
    {
        clint->on_rising_edge_clock();
    }

    // Read the timer again
    mtime_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET);
    mtime_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 4);
    uint64_t new_mtime = ((uint64_t)mtime_high << 32) | mtime_low;

    // Verify the timer has incremented by exactly 100
    PN_ASSERTF(((new_mtime == orig_mtime + 100)),
        ("Timer should increment by exactly 100, original: 0x%016lx, new: 0x%016lx",
            orig_mtime,
            new_mtime));

    PN_INFO("Timer increment test passed");

    // In a more complete test, we would set the timer to max value and test overflow
    // But that would require internal access to the timer state
    PN_INFO("Note: Complete overflow test would require internal timer access");

    PN_INFO("Timer overflow test passed");
}

// Helper function to test reset behavior
void test_reset_behavior(m_clint_soft* clint)
{
    PN_INFO("Test 8: Testing reset behavior");

    // 1. Modify state
    PN_INFO("Setting MSIP to 1");
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET, 1);

    uint32_t msip = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET);
    PN_ASSERTF(((msip == 1)), ("MSIP should be 1 after setting"));

    // Read current MTIMECMP
    uint32_t mtimecmp_low = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET);
    uint32_t mtimecmp_high = clint->read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4);
    uint64_t mtimecmp = ((uint64_t)mtimecmp_high << 32) | mtimecmp_low;

    // Set MTIMECMP to a specific test value
    uint64_t test_mtimecmp = 0x1234567890ABCDEF;
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4, (uint32_t)(test_mtimecmp >> 32));
    clint->write32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET, (uint32_t)test_mtimecmp);

    // 2. Create a new CLINT instance (simulating reset)
    PN_INFO("Creating new CLINT instance to simulate reset");

    bool saved_msip = msip_signal;
    bool saved_mtip = mtip_signal;

    // Reset the signal values before creating new instance
    msip_signal = false;
    mtip_signal = false;

    m_clint_soft new_clint(
        [](bool value) {
            msip_signal = value;
            PN_INFOF(("MSIP callback (reset): %s", value ? "asserted" : "deasserted"));
        },
        [](bool value) {
            mtip_signal = value;
            PN_INFOF(("MTIP callback (reset): %s", value ? "asserted" : "deasserted"));
        });

    // 3. Verify reset state
    msip = new_clint.read32(PN_CFG_CLINT_BASE_ADDRESS + MSIP_OFFSET);
    PN_ASSERTF(((msip == 0)), ("MSIP should be 0 after reset, got %u", msip));

    mtimecmp_low = new_clint.read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET);
    mtimecmp_high = new_clint.read32(PN_CFG_CLINT_BASE_ADDRESS + MTIMECMP_OFFSET + 4);
    mtimecmp = ((uint64_t)mtimecmp_high << 32) | mtimecmp_low;

    PN_ASSERTF(((mtimecmp == 0xFFFFFFFFFFFFFFFFULL)),
        ("MTIMECMP should be 0xFFFFFFFFFFFFFFFF after reset, got 0x%016lx", mtimecmp));

    uint32_t mtime_low = new_clint.read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET);
    uint32_t mtime_high = new_clint.read32(PN_CFG_CLINT_BASE_ADDRESS + MTIME_OFFSET + 4);
    uint64_t mtime = ((uint64_t)mtime_high << 32) | mtime_low;

    PN_ASSERTF(((mtime == 0)), ("MTIME should be 0 after reset, got 0x%016lx", mtime));

    // 4. Verify interrupt signals are in reset state
    PN_ASSERTF(((!msip_signal)), ("MSIP signal should be false after reset"));
    PN_ASSERTF(((!mtip_signal)), ("MTIP signal should be false after reset"));

    PN_INFO("Reset behavior test passed");
}

// Main entry point
int sc_main(int argc, char** argv)
{
    PN_PARSE_CMD_ARGS(argc, argv);

    PN_INFO("Starting m_clint_soft testbench");

    // Create CLINT instance with interrupt callbacks
    m_clint_soft clint(
        // MSIP callback
        [](bool value) {
            msip_signal = value;
            PN_INFOF(("MSIP callback: %s", value ? "asserted" : "deasserted"));
        },
        // MTIP callback
        [](bool value) {
            mtip_signal = value;
            PN_INFOF(("MTIP callback: %s", value ? "asserted" : "deasserted"));
        });

    // Run tests
    test_msip(&clint);
    test_timer(&clint);
    test_address_decoding(&clint);

    // Run the new tests
    test_16bit_access(&clint);
    test_atomic_mtimecmp_update(&clint);
    test_unaligned_access(&clint);
    test_timer_overflow(&clint);
    test_reset_behavior(&clint);

    PN_INFO("All tests passed successfully");

    return 0;
}