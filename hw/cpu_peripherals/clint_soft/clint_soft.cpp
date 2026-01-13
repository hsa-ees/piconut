/**
 * @file clint_soft.cpp
 * @brief Implementation of the m_clint_soft class.
 *        For simulation ONLY.
 */

/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Alexander Beck
                2025 Christian Zellinger
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
      This file contains the implementation of the m_clint_soft class.
      For simulation ONLY.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
     list of conditions and the following disclaimer in the documentation and/or
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

#include "clint_soft.h"
#include <piconut.h>
#include <cstring>
#include <cstdio>

// Static member initialization
const char* m_clint_soft::MODULE_NAME = "CLINT";

m_clint_soft::m_clint_soft(
    std::function<void(bool)> callback_signal_sw_interrupt,
    std::function<void(bool)> callback_signal_timer_interrupt)
    : msip(0)
    , mtimecmp(0xFFFFFFFFFFFFFFFFULL)
    , // Set to max value initially
    mtime(0)
    , mtip_pending(false)
    , timer_interrupt_update_pending(false)
    , mtimecmp_high_written(false)
    , // Initialize the new field
    mtime_high_written(false)
    , // Initialize MTIME HIGH written flag
    mtime_update_suspended(false)
    , // Initialize MTIME update suspended flag
    callback_signal_sw_interrupt(callback_signal_sw_interrupt)
    , callback_signal_timer_interrupt(callback_signal_timer_interrupt)
{
    base_address = 0x2000000; // Default CLINT base address as per RISC-V spec
    size = 0x10000;           // Default CLINT size as per RISC-V spec

    // Initialize peripheral name
    strncpy(peripheral_name, "CLINT", sizeof(peripheral_name) - 1);
    peripheral_name[sizeof(peripheral_name) - 1] = '\0';

    // Initial state: no interrupts pending
    if(callback_signal_sw_interrupt)
    {
        callback_signal_sw_interrupt(false);
    }
    if(callback_signal_timer_interrupt)
    {
        callback_signal_timer_interrupt(false);
    }

    PN_INFOF(("clint_soft: Initialized at base address 0x%08lx", (unsigned long)base_address));
}

const char* m_clint_soft::get_info()
{
    static char info_str[256];
    snprintf(info_str, sizeof(info_str), "CLINT (Core Local Interrupter)"
                                         " Size: 0x%lx"
                                         " MSIP: 0x%08x"
                                         " MTIMECMP: 0x%016llx"
                                         " MTIME: 0x%016llx\n",
        (unsigned long)size,
        msip,
        (unsigned long long)mtimecmp,
        (unsigned long long)mtime);
    return info_str;
}

bool m_clint_soft::is_addressed(uint64_t adr)
{
    return (adr >= base_address && adr < base_address + size);
}

uint32_t m_clint_soft::read32(uint64_t adr)
{
    if(!is_addressed(adr))
    {
        PN_WARNINGF(("clint_soft: Read from invalid address 0x%08lx", (unsigned long)adr));
        return 0;
    }

    uint64_t offset = adr - base_address;

    // Handle register accesses based on the base register offset
    uint64_t aligned_offset = offset & ~0x3ULL; // Align to 32-bit boundary

    // MSIP register region (0x0000-0x0003)
    if(aligned_offset == 0x0000)
    {
        return msip;
    }

    // MTIMECMP register (low 32 bits) region (0x4000-0x4003)
    else if(aligned_offset == 0x4000)
    {
        uint32_t value = static_cast<uint32_t>(mtimecmp & 0xFFFFFFFF);
        return value;
    }

    // MTIMECMP register (high 32 bits) region (0x4004-0x4007)
    else if(aligned_offset == 0x4004)
    {
        uint32_t value = static_cast<uint32_t>((mtimecmp >> 32) & 0xFFFFFFFF);
        return value;
    }

    // MTIME register (low 32 bits) region (0xBFF8-0xBFFB)
    else if(aligned_offset == 0xBFF8)
    {
        uint32_t value = static_cast<uint32_t>(mtime & 0xFFFFFFFF);
        return value;
    }

    // MTIME register (high 32 bits) region (0xBFFC-0xBFFF)
    else if(aligned_offset == 0xBFFC)
    {
        uint32_t value = static_cast<uint32_t>((mtime >> 32) & 0xFFFFFFFF);
        return value;
    }

    // Invalid register
    else
    {
        PN_WARNINGF(("clint_soft: Read from unsupported register at offset 0x%08lx", (unsigned long)offset));
        return 0;
    }
}

uint8_t m_clint_soft::read8(uint64_t adr)
{
    if(!is_addressed(adr))
    {
        PN_ERRORF(("clint_soft: Read8 from invalid address 0x%08lx", (unsigned long)adr));
        return 0;
    }

    uint64_t offset = adr - base_address;
    uint64_t aligned_offset = offset & ~0x3ULL; // Align to 32-bit boundary
    uint8_t byte_offset = offset & 0x3;         // Which byte within the 32-bit word

    // Read the complete 32-bit word
    uint32_t word_value = read32(base_address + aligned_offset);

    // Extract the desired byte (Little Endian)
    uint8_t byte_value = (word_value >> (byte_offset * 8)) & 0xFF;

    return byte_value;
}

void m_clint_soft::write32(uint64_t adr, uint32_t data)
{
    if(!is_addressed(adr))
    {
        PN_ERRORF(("clint_soft: Write to invalid address 0x%08lx", (unsigned long)adr));
        return;
    }

    uint64_t offset = adr - base_address;
    uint64_t aligned_offset = offset & ~0x3ULL; // Align to 32-bit boundary

    // MSIP register region (0x0000-0x0003)
    if(aligned_offset == 0x0000)
    {
        // Only bit 0 is used in MSIP register
        msip = data & 0x1;

        // MSIP can be signaled immediately as it doesn't cause the conflict
        if(callback_signal_sw_interrupt)
        {
            callback_signal_sw_interrupt(msip != 0);
        }
    }

    // MTIMECMP register (low 32 bits) region (0x4000-0x4003)
    else if(aligned_offset == 0x4000)
    {
        // Store the value
        mtimecmp = (mtimecmp & 0xFFFFFFFF00000000ULL) | data;

        // If HIGH was written first (as recommended), now update the interrupt state
        if(mtimecmp_high_written)
        {
            mtimecmp_high_written = false;
            update_timer_interrupt();
        }
    }

    // MTIMECMP register (high 32 bits) region (0x4004-0x4007)
    else if(aligned_offset == 0x4004)
    {

        // Update upper 32 bits of MTIMECMP
        mtimecmp = (mtimecmp & 0x00000000FFFFFFFFULL) | (static_cast<uint64_t>(data) << 32);

        // Set flag indicating HIGH was written but don't update interrupt yet
        mtimecmp_high_written = true;
    }

    // MTIME register (low 32 bits) region (0xBFF8-0xBFFB)
    else if(aligned_offset == 0xBFF8)
    {
        // Update lower 32 bits of MTIME
        mtime = (mtime & 0xFFFFFFFF00000000ULL) | data;

        // Always reset the suspended flag after writing to MTIME_LOW
        mtime_update_suspended = false;

        // If HIGH was written first, clear that flag too
        if(mtime_high_written)
        {
            mtime_high_written = false;
        }

        // Update timer interrupt state since MTIME has changed
        update_timer_interrupt();
    }

    // MTIME register (high 32 bits) region (0xBFFC-0xBFFF)
    else if(aligned_offset == 0xBFFC)
    {
        // Suspend MTIME updates during write
        mtime_update_suspended = true;

        // Update upper 32 bits of MTIME
        mtime = (mtime & 0x00000000FFFFFFFFULL) | (static_cast<uint64_t>(data) << 32);

        // Set flag indicating HIGH was written
        mtime_high_written = true;
    }

    // Invalid register
    else
    {
        PN_ERRORF(("clint_soft: Write to unsupported register at offset 0x%08lx", (unsigned long)offset));
    }
}

void m_clint_soft::write8(uint64_t adr, uint8_t data)
{
    if(!is_addressed(adr))
    {
        PN_ERRORF(("clint_soft: Write8 to invalid address 0x%08lx", (unsigned long)adr));
        return;
    }

    uint64_t offset = adr - base_address;
    uint64_t aligned_offset = offset & ~0x3ULL; // Align to 32-bit boundary
    uint8_t byte_offset = offset & 0x3;         // Which byte within the 32-bit word

    // For MTIME register writes, suspend updates
    if(aligned_offset == 0xBFF8 || aligned_offset == 0xBFFC)
    {
        mtime_update_suspended = true;
    }

    // Read the complete 32-bit word
    uint32_t word_value = read32(base_address + aligned_offset);

    // Update the desired byte (Little Endian)
    uint32_t byte_mask = 0xFF << (byte_offset * 8);
    uint32_t new_value = (word_value & ~byte_mask) | (static_cast<uint32_t>(data) << (byte_offset * 8));

    // Write back the updated word
    write32(base_address + aligned_offset, new_value);

    // If this is the last byte of MTIME_LOW (byte 0 at offset 0xBFF8), resume timer updates
    // This handles the case in the test where bytes are written from high to low
    if(aligned_offset == 0xBFF8 && byte_offset == 0)
    {
        mtime_update_suspended = false;
    }
}

void m_clint_soft::update_timer_interrupt()
{
    // Timer interrupt triggers when mtime >= mtimecmp (full 64-bit comparison)
    bool new_mtip_state = (mtime >= mtimecmp);

    // Only update the pending state, don't signal immediately
    if(new_mtip_state != mtip_pending)
    {
        mtip_pending = new_mtip_state;
        timer_interrupt_update_pending = true;
    }
}

void m_clint_soft::on_rising_edge_clock()
{
    // Only increment the timer if updates aren't suspended during a write
    if(!mtime_update_suspended)
    {
        // Increment the timer by 1 on each clock edge
        mtime++;
    }

    // Check for pending timer interrupt updates
    if(timer_interrupt_update_pending)
    {
        timer_interrupt_update_pending = false;

        // Signal the timer interrupt state change
        if(callback_signal_timer_interrupt)
        {
            callback_signal_timer_interrupt(mtip_pending);
        }
    }

    // Check if timer condition has changed on this clock edge
    bool current_mtip_state = (mtime >= mtimecmp);
    if(current_mtip_state != mtip_pending)
    {
        mtip_pending = current_mtip_state;

        // Signal the timer interrupt state change
        if(callback_signal_timer_interrupt)
        {
            callback_signal_timer_interrupt(mtip_pending);
        }
    }
}

void m_clint_soft::register_msip_callback(std::function<void(bool)> callback)
{
    callback_signal_sw_interrupt = callback;
    PN_INFOF(("clint_soft: Registered MSIP callback"));
}

void m_clint_soft::register_mtip_callback(std::function<void(bool)> callback)
{
    callback_signal_timer_interrupt = callback;
    PN_INFOF(("clint_soft: Registered MTIP callback"));
}

void m_clint_soft::write64(uint64_t adr, uint64_t data)
{
    if(!is_addressed(adr))
    {
        PN_ERRORF(("clint_soft: Write64 to invalid address 0x%08lx", (unsigned long)adr));
        return;
    }

    uint64_t offset = adr - base_address;

    // Check if this is a 64-bit CLINT register (MTIME or MTIMECMP)
    if((offset == 0x4000) || (offset == 0xBFF8))
    {
        // For CLINT registers, use HIGH->LOW write order to avoid race conditions
        // Write HIGH 32-bit first (recommended sequence)
        write32(adr + 4, (uint32_t)((data >> 32) & 0xFFFFFFFF));
        // Then write LOW 32-bit
        write32(adr, (uint32_t)(data & 0xFFFFFFFF));

        PN_INFOF(("clint_soft: 64-bit write to offset 0x%08lx, value=0x%016llx (HIGH->LOW order)",
            (unsigned long)offset,
            (unsigned long long)data));
    }
    else
    {
        // For other addresses, fall back to base class implementation (LOW->HIGH order)
        c_soft_peripheral::write64(adr, data);
    }
}