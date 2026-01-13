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

// SystemC CLINT implementation
#include "clint.h"

void m_clint::proc_comb_wb_slave()
{
    // Proper Wishbone slave state machine implementation following template
    // Set idle outputs to default
    wb_ack_o.write(false);
    wb_dat_o.write(0);
    wb_err_o.write(false);
    wb_rty_o.write(false);

    c_wb_write_en.write(false);

    wb_next_state.write(wb_current_state.read()); // default next state is current state

    switch(wb_current_state.read())
    {
        case WB_IDLE: // Wishbone Idle State to check if we have a valid transaction
            if(wb_stb_i.read() == 1 && wb_cyc_i.read() == 1)
            { // WB Strobe and cycle valid
                sc_uint<32> addr = wb_adr_i.read();

                // Check if address is within CLINT range
                if(addr >= PN_CFG_CLINT_BASE_ADDRESS && addr < PN_CFG_CLINT_BASE_ADDRESS + CLINT_SIZE)
                {
                    if(wb_we_i.read() == 1)
                    {
                        wb_next_state.write(WB_WRITE1); // Master writes to us (Slave)
                    }
                    else
                    {
                        wb_next_state.write(WB_READ); // Master reads from us (Slave)
                    }
                }
                else
                {
                    // Address is out of range - generate an error response
                    wb_err_o.write(true);
                    wb_ack_o.write(true); // Still acknowledge to avoid hanging the bus

                    if(wb_stb_i.read() == 0 && wb_cyc_i.read() == 0)
                    {
                        wb_next_state.write(WB_IDLE);
                    }
                }
            }
            break;

        case WB_WRITE1: // Master writes to us -> read from input, write to our register
            c_wb_write_en.write(true);
            wb_next_state.write(WB_WRITE2);
            break;

        case WB_WRITE2: // Write Acknowledge
            c_wb_write_en.write(true);
            wb_ack_o.write(true);

            if(wb_stb_i.read() == 0 && wb_cyc_i.read() == 0)
            {
                wb_next_state.write(WB_IDLE);
            }
            break;

        case WB_READ: // Master reads from us -> Write data to the bus output
            sc_uint<32> addr = wb_adr_i.read();
            wb_dat_o.write(read_register(addr));
            wb_ack_o.write(true);

            if(wb_stb_i.read() == 0 && wb_cyc_i.read() == 0)
            {
                wb_next_state.write(WB_IDLE);
            }
            break;
    }
}

void m_clint::proc_clk_state()
{
    // State machine clock process - initializes and transitions states
    wb_current_state.write(WB_IDLE);

    while(true)
    {
        wait();

        // State transition on clock edge
        wb_current_state.write(wb_next_state.read());
    }
}

void m_clint::proc_clk_wb_slave()
{
    // Wishbone register write process - handles actual register updates
    mtimecmp_reg.write(0xFFFFFFFFFFFFFFFFULL); // Initialize to max value
    msip_reg.write(0);                         // Initialize to 0
    mtimecmp_high_written = false;             // Initialize the flag
    mtime_high_written = false;                // Initialize the MTIME flag
    mtime_write_active = false;                // Initialize the new flag
    mtip_pending = false;                      // Initialize timer interrupt pending to false
    msip_pending = false;                      // Initialize software interrupt pending to false
    mtime_reg.write(0);                        // Initialize MTIME to 0

    while(true)
    {
        wait();

        // Handle reset state first
        if(reset.read())
        {
            // Reset all internal state
            mtip_pending.write(false);
            msip_pending.write(false);
            in_reset.write(true);
            mtime_reg.write(0);              // Reset MTIME to 0 when reset is asserted
            mtime_write_active.write(false); // Clear write flag on reset
        }
        else
        {
            in_reset.write(false);

            // Only process register writes when write enable is active
            if(c_wb_write_en.read() == 1)
            {
                sc_uint<32> addr = wb_adr_i.read();
                sc_uint<32> data = wb_dat_i.read();
                sc_uint<4> sel = wb_sel_i.read();

                // Write to register using helper function
                write_register(addr, data, sel);
            }

            // Only increment MTIME if we're not in the middle of any MTIME write operation
            if(!mtime_write_active.read())
            {
                // Increment MTIME on each clock cycle
                mtime_reg.write(mtime_reg.read() + 1);
            }

            // Check for timer interrupt condition on every clock cycle
            bool new_mtip_state = (mtime_reg.read() >= mtimecmp_reg.read());
            if(new_mtip_state != mtip_pending.read())
            {
                mtip_pending.write(new_mtip_state);
            }

            // Update software interrupt pending state based on MSIP register
            bool new_msip_state = (msip_reg.read() & 0x1) != 0;
            if(new_msip_state != msip_pending.read())
            {
                msip_pending.write(new_msip_state);
            }
        }
    }
}

void m_clint::proc_comb_process_interrupts()
{
    if(reset.read())
    {
        // During reset, just set outputs to false but don't modify pending flags
        // as those are managed by proc_clk_wb_slave
        msip_o.write(false);
        mtip_o.write(false);
    }
    else
    {
        // Simply forward the pending flags to output signals
        // This ensures we don't have multiple drivers for the same signals
        msip_o.write(msip_pending.read());
        mtip_o.write(mtip_pending.read());
    }
}

void m_clint::write_register(sc_uint<32> addr, sc_uint<32> data, sc_uint<8> sel)
{
    sc_uint<32> write_mask = 0;
    for(unsigned int i = 0; i < 4; i++)
    {
        if((sel >> i) & 0x1U)
        {
            write_mask |= (0xFFU << (i * 8U));
        }
    }

    switch(addr)
    {
        case PN_CFG_CLINT_BASE_ADDRESS: {
            // MSIP Register (32-bit, only bit 0 is relevant)
            sc_uint<32> old_msip = msip_reg.read();
            sc_uint<32> new_msip = (old_msip & sc_uint<32>(~write_mask)) | (data & write_mask & 0x1);
            msip_reg.write(new_msip);

            // If MSIP value changed, update the pending flag
            if((old_msip & 0x1) != (new_msip & 0x1))
            {
                bool new_msip_state = (new_msip & 0x1) != 0;
                msip_pending.write(new_msip_state);
            }
            break;
        }

        case CLINT_REG_MTIMECMP_LO: {
            sc_uint<64> old_mtimecmp = mtimecmp_reg.read();
            // Lower 32-bits - Apply selective byte write using mask
            sc_uint<32> old_low = old_mtimecmp & 0xFFFFFFFFULL;
            sc_uint<32> new_low = (old_low & sc_uint<32>(~write_mask)) | (data & write_mask);
            sc_uint<64> new_mtimecmp = (old_mtimecmp & 0xFFFFFFFF00000000ULL) | new_low;
            mtimecmp_reg.write(new_mtimecmp);

            // If HIGH was written first (as recommended), now check the interrupt condition
            if(mtimecmp_high_written.read())
            {
                mtimecmp_high_written.write(false);

                // Check if timer interrupt state needs to change
                bool new_mtip_state = (mtime_reg.read() >= mtimecmp_reg.read());
                if(new_mtip_state != mtip_pending.read())
                {
                    mtip_pending.write(new_mtip_state);
                }
            }
            break;
        }

        case CLINT_REG_MTIMECMP_HI: {
            sc_uint<64> old_mtimecmp = mtimecmp_reg.read();
            // Upper 32-bits - Apply selective byte write using mask
            sc_uint<32> old_high = (old_mtimecmp >> 32) & 0xFFFFFFFFULL;
            sc_uint<32> new_high = (old_high & sc_uint<32>(~write_mask)) | (data & write_mask);
            sc_uint<64> new_mtimecmp = (old_mtimecmp & 0x00000000FFFFFFFFULL) | (((sc_uint<64>)new_high) << 32);
            sc_uint<64> mask64 = (sc_uint<64>)write_mask;

            mtimecmp_reg.write(new_mtimecmp);

            // Set flag indicating HIGH was written
            mtimecmp_high_written.write(true);
            break;
        }

        case CLINT_REG_MTIME_LO: {
            // Set flag indicating an MTIME write is active
            mtime_write_active.write(true);

            sc_uint<64> old_mtime = mtime_reg.read();
            // Lower 32-bits - Apply selective byte write using mask
            sc_uint<32> old_low = old_mtime & 0xFFFFFFFFULL;
            sc_uint<32> new_low = (old_low & sc_uint<32>(~write_mask)) | (data & write_mask);
            sc_uint<64> new_mtime = (old_mtime & 0xFFFFFFFF00000000ULL) | new_low;
            mtime_reg.write(new_mtime);

            if(mtime_high_written.read())
            {
                mtime_high_written.write(false);
                mtime_write_active.write(false); // Clear write flag now that write is complete

                bool new_mtip_state = (mtime_reg.read() >= mtimecmp_reg.read());
                if(new_mtip_state != mtip_pending.read())
                {
                    mtip_pending.write(new_mtip_state);
                }
            }
            else
            {
                // If this was a standalone low-word write, clear the flag immediately
                mtime_write_active.write(false);

                bool new_mtip_state = (mtime_reg.read() >= mtimecmp_reg.read());
                if(new_mtip_state != mtip_pending.read())
                {
                    mtip_pending.write(new_mtip_state);
                }
            }
            break;
        }

        case CLINT_REG_MTIME_HI: {
            // Set flag indicating an MTIME write is active
            mtime_write_active.write(true);

            sc_uint<64> old_mtime = mtime_reg.read();
            // Upper 32-bits - Apply selective byte write using mask
            sc_uint<32> old_high = (old_mtime >> 32) & 0xFFFFFFFFULL;
            sc_uint<32> new_high = (old_high & sc_uint<32>(~write_mask)) | (data & write_mask);
            sc_uint<64> new_mtime = (old_mtime & 0x00000000FFFFFFFFULL) | (((sc_uint<64>)new_high) << 32);
            mtime_reg.write(new_mtime);

            // Set flag indicating HIGH was written
            mtime_high_written.write(true);
            // Keep mtime_write_active true until low word is written or timeout
            break;
        }

        default:
            // No matching address — do nothing or handle error
            break;
    }
}

sc_uint<32> m_clint::read_register(sc_uint<32> address)
{
    sc_uint<32> result = 0;

    switch(address)
    {
        // MSIP Register (PN_CFG_CLINT_BASE_ADDRESS)
        case PN_CFG_CLINT_BASE_ADDRESS: {
            // Read the full 32-bit MSIP register
            result = msip_reg.read();
            break;
        }

        // MTIMECMP Lower 32-bits
        case CLINT_REG_MTIMECMP_LO: {
            // Lower 32-bits of MTIMECMP
            result = (mtimecmp_reg.read() & 0xFFFFFFFFU);
            break;
        }

        // MTIMECMP Upper 32-bits
        case CLINT_REG_MTIMECMP_HI: {
            // Upper 32-bits of MTIMECMP
            result = ((mtimecmp_reg.read() >> 32) & 0xFFFFFFFFU);
            break;
        }

        // MTIME Lower 32-bits
        case CLINT_REG_MTIME_LO: {
            // Lower 32-bits of MTIME
            // Note: In real hardware, the two 32-bit reads of MTIME should be atomic,
            // but reading the full 64-bit signal here is SystemC best practice.
            result = (mtime_reg.read() & 0xFFFFFFFFU);
            break;
        }

        // MTIME Upper 32-bits
        case CLINT_REG_MTIME_HI: {
            // Upper 32-bits of MTIME
            result = ((mtime_reg.read() >> 32) & 0xFFFFFFFFU);
            break;
        }

        default:
            result = 0;
            break;
    }

    return result;
}