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

#include "timer.h"

void m_timer::pn_trace(sc_trace_file* tf, int level)
{
    if(level >= 1)
    {
        PN_TRACE(tf, clk);
        PN_TRACE(tf, reset);
        PN_TRACE(tf, wb_stb_i);
        PN_TRACE(tf, wb_cyc_i);
        PN_TRACE(tf, wb_we_i);
        PN_TRACE(tf, wb_sel_i);
        PN_TRACE(tf, wb_ack_o);
        PN_TRACE(tf, wb_err_o);
        PN_TRACE(tf, wb_rty_o);
        PN_TRACE(tf, wb_adr_i);
        PN_TRACE(tf, wb_dat_i);
        PN_TRACE(tf, wb_dat_o);
        PN_TRACE(tf, state);
        PN_TRACE(tf, timer_irq);
        PN_TRACE(tf, autoreload_pulse);

        PN_TRACE_BUS(tf, ioregs_regs, 10);
    }
}

void m_timer::proc_comb_wb_slave()
{
    // Set idle outputs to default
    wb_ack_o = 0;
    wb_dat_o = 0;
    wb_err_o = 0;
    wb_rty_o = 0;

    // Default values for write control signals
    wb_write_control = false;
    wb_write_reload = false;
    wb_write_cc1 = false;
    wb_write_cc2 = false;
    wb_write_cc3 = false;
    wb_write_cc4 = false;
    wb_write_ctrl_reg = false;
    wb_write_status_reg = false;
    wb_write_int_enable = false;
    wb_write_data = 0;

    wb_next_state = wb_current_state.read(); // Default next state is current state

    switch(wb_current_state.read())
    {
        case WB_IDLE:                                        // Wishbone Idle State to check if we have a valid transaction
            if(wb_stb_i.read() == 1 && wb_cyc_i.read() == 1) // WB Strobe and cycle valid
            {
                if((PN_CFG_TIMER_BASE_ADDRESS <= wb_adr_i.read()) &&
                    (wb_adr_i.read() < (PN_CFG_TIMER_BASE_ADDRESS + 0x50)))
                {
                    if(wb_we_i.read() == 1)
                    {
                        wb_next_state = WB_WRITE1; // Master writes to us (Slave)
                    }
                    else
                    {
                        wb_next_state = WB_READ; // Master reads from us (Slave)
                    }
                }
            }
            break;

        case WB_WRITE1: // Master writes to us -> read from input, write to our register
        {
            wb_write_data = wb_dat_i.read();
            sc_uint<WB_ADR_WIDTH> reg_addr = wb_adr_i.read() - PN_CFG_TIMER_BASE_ADDRESS;

            switch(reg_addr)
            {
                // STM32 TIM2 compatible register mapping for writes
                case M_TIMER_REG_CR1: // 0x00 - Control Register 1
                    wb_write_ctrl_reg = true;
                    break;
                case M_TIMER_REG_CR2: // 0x04 - Control Register 2 (dummy)
                    // Write to dummy register - store but don't use
                    break;
                case M_TIMER_REG_SMCR: // 0x08 - Slave Mode Control Register (dummy)
                    // Write to dummy register - store but don't use
                    break;
                case M_TIMER_REG_DIER: // 0x0C - Interrupt Enable Register
                    wb_write_int_enable = true;
                    break;
                case M_TIMER_REG_SR: // 0x10 - Status Register
                    wb_write_status_reg = true;
                    break;
                case M_TIMER_REG_EGR: // 0x14 - Event Generation Register (write-only)
                    // Ignore writes to EGR for now (could trigger events in future)
                    break;
                case M_TIMER_REG_CCMR1: // 0x18 - Capture/Compare Mode Register 1 (dummy)
                    // Write to dummy register - store but don't use
                    break;
                case M_TIMER_REG_CCMR2: // 0x1C - Capture/Compare Mode Register 2 (dummy)
                    // Write to dummy register - store but don't use
                    break;
                case M_TIMER_REG_CCER: // 0x20 - Capture/Compare Enable Register (dummy)
                    // Write to dummy register - store but don't use
                    break;
                case M_TIMER_REG_CNT: // 0x24 - Counter Register
                    // COUNTER register is read-only, ignore write
                    break;
                case M_TIMER_REG_PSC: // 0x28 - Prescaler Register
                    wb_write_control = true;
                    break;
                case M_TIMER_REG_ARR: // 0x2C - Auto-Reload Register
                    wb_write_reload = true;
                    break;
                case M_TIMER_REG_CCR1: // 0x34 - Capture/Compare Register 1
                    wb_write_cc1 = true;
                    break;
                case M_TIMER_REG_CCR2: // 0x38 - Capture/Compare Register 2
                    wb_write_cc2 = true;
                    break;
                case M_TIMER_REG_CCR3: // 0x3C - Capture/Compare Register 3
                    wb_write_cc3 = true;
                    break;
                case M_TIMER_REG_CCR4: // 0x40 - Capture/Compare Register 4
                    wb_write_cc4 = true;
                    break;
                case M_TIMER_REG_DCR: // 0x48 - DMA Control Register (dummy)
                    // Write to dummy register - store but don't use
                    break;
                case M_TIMER_REG_DMAR: // 0x4C - DMA Address Register (dummy)
                    // Write to dummy register - store but don't use
                    break;
                default:
                    // Unknown register, ignore write
                    break;
            }

            // Set the next wishbone state because writing is not finished yet
            wb_next_state = WB_WRITE2;
        }
        break;

        case WB_WRITE2: // Write Acknowledge
            wb_ack_o = 1;

            // After setting the wishbone ack the writing is finished and we can go back to idle state
            if(wb_stb_i.read() == 0 && wb_cyc_i.read() == 0)
            {
                wb_next_state = WB_IDLE;
            }
            break;

        case WB_READ:                                           // Master Reads from us -> Write data to the bus output
            switch(wb_adr_i.read() - PN_CFG_TIMER_BASE_ADDRESS) // Use Address offset to determine target register
            {
                // STM32 TIM2 compatible register mapping
                case M_TIMER_REG_CR1:                                        // 0x00 - Control Register 1
                    wb_dat_o = read_with_byte_select(ioregs_regs[7].read()); // Maps to CONTROL
                    break;
                case M_TIMER_REG_CR2: // 0x04 - Control Register 2 (dummy)
                    wb_dat_o = read_with_byte_select(dummy_cr2.read());
                    break;
                case M_TIMER_REG_SMCR: // 0x08 - Slave Mode Control Register (dummy)
                    wb_dat_o = read_with_byte_select(dummy_smcr.read());
                    break;
                case M_TIMER_REG_DIER:                                       // 0x0C - Interrupt Enable Register
                    wb_dat_o = read_with_byte_select(ioregs_regs[9].read()); // Maps to INTERRUPT_ENABLE
                    break;
                case M_TIMER_REG_SR:                                         // 0x10 - Status Register
                    wb_dat_o = read_with_byte_select(ioregs_regs[8].read()); // Maps to STATUS
                    break;
                case M_TIMER_REG_EGR: // 0x14 - Event Generation Register (write-only, return 0)
                    wb_dat_o = 0x00000000;
                    break;
                case M_TIMER_REG_CCMR1: // 0x18 - Capture/Compare Mode Register 1 (dummy)
                    wb_dat_o = read_with_byte_select(dummy_ccmr1.read());
                    break;
                case M_TIMER_REG_CCMR2: // 0x1C - Capture/Compare Mode Register 2 (dummy)
                    wb_dat_o = read_with_byte_select(dummy_ccmr2.read());
                    break;
                case M_TIMER_REG_CCER: // 0x20 - Capture/Compare Enable Register (dummy)
                    wb_dat_o = read_with_byte_select(dummy_ccer.read());
                    break;
                case M_TIMER_REG_CNT:                                        // 0x24 - Counter Register
                    wb_dat_o = read_with_byte_select(ioregs_regs[0].read()); // Maps to COUNTER
                    break;
                case M_TIMER_REG_PSC:                                        // 0x28 - Prescaler Register
                    wb_dat_o = read_with_byte_select(ioregs_regs[2].read()); // Maps to PRESCALER
                    break;
                case M_TIMER_REG_ARR:                                        // 0x2C - Auto-Reload Register
                    wb_dat_o = read_with_byte_select(ioregs_regs[1].read()); // Maps to AUTO_RELOAD
                    break;
                case M_TIMER_REG_CCR1:                                       // 0x34 - Capture/Compare Register 1
                    wb_dat_o = read_with_byte_select(ioregs_regs[3].read()); // Maps to CAPTURE_COMPARE_1
                    break;
                case M_TIMER_REG_CCR2:                                       // 0x38 - Capture/Compare Register 2
                    wb_dat_o = read_with_byte_select(ioregs_regs[4].read()); // Maps to CAPTURE_COMPARE_2
                    break;
                case M_TIMER_REG_CCR3:                                       // 0x3C - Capture/Compare Register 3
                    wb_dat_o = read_with_byte_select(ioregs_regs[5].read()); // Maps to CAPTURE_COMPARE_3
                    break;
                case M_TIMER_REG_CCR4:                                       // 0x40 - Capture/Compare Register 4
                    wb_dat_o = read_with_byte_select(ioregs_regs[6].read()); // Maps to CAPTURE_COMPARE_4
                    break;
                case M_TIMER_REG_DCR: // 0x48 - DMA Control Register (dummy)
                    wb_dat_o = read_with_byte_select(dummy_dcr.read());
                    break;
                case M_TIMER_REG_DMAR: // 0x4C - DMA Address Register (dummy)
                    wb_dat_o = read_with_byte_select(dummy_dmar.read());
                    break;
                default:
                    wb_dat_o = 0x00000000; // Return 0 for unknown registers
                    break;
            }

            wb_ack_o = 1;

            if(wb_stb_i.read() == 0 && wb_cyc_i.read() == 0)
            {
                wb_next_state = WB_IDLE;
            }
            break;
    }
}

void m_timer::proc_clk_state()
{
    wb_current_state = WB_IDLE;

    while(true)
    {
        wait();
        wb_current_state = wb_next_state;
    }
}

void m_timer::proc_clk_transition()
{
    // Initialize registers for new layout
    ioregs_regs[0].write(0x00000000); // COUNTER = 0
    ioregs_regs[1].write(0x00000001); // AUTO_RELOAD = 1 (default)
    ioregs_regs[2].write(0x00000001); // PRESCALER = 1 (default)
    ioregs_regs[3].write(0x00000000); // CAPTURE_COMPARE_1 = 0
    ioregs_regs[4].write(0x00000000); // CAPTURE_COMPARE_2 = 0
    ioregs_regs[5].write(0x00000000); // CAPTURE_COMPARE_3 = 0
    ioregs_regs[6].write(0x00000000); // CAPTURE_COMPARE_4 = 0
    ioregs_regs[7].write(0x00000000); // CONTROL = 0 (timer disabled)
    ioregs_regs[8].write(0x00000000); // STATUS = 0 (no flags)
    ioregs_regs[9].write(0x00000000); // INTERRUPT_ENABLE = 0 (all disabled)
    state.write(0);

    // Initialize dummy registers for STM32 compatibility
    dummy_cr2.write(0x00000000);
    dummy_smcr.write(0x00000000);
    dummy_ccmr1.write(0x00000000);
    dummy_ccmr2.write(0x00000000);
    dummy_ccer.write(0x00000000);
    dummy_dcr.write(0x00000000);
    dummy_dmar.write(0x00000000);

    prescaler_counter.write(0);

    while(true)
    {
        wait(); // Wait for clock edge

        // Handle wishbone write operations first
        if(wb_write_control.read())
        {
            // Writing to PRESCALER register (reg 2) with byte select
            ioregs_regs[2].write(write_with_byte_select(ioregs_regs[2].read()));

            // Auto-enable timer for backward compatibility when prescaler is set
            sc_uint<32> control_reg = ioregs_regs[7].read();
            control_reg |= 0x01; // Set CEN bit
            ioregs_regs[7].write(control_reg);
        }

        if(wb_write_reload.read())
        {
            // Writing to AUTO_RELOAD register (reg 1) with byte select
            ioregs_regs[1].write(write_with_byte_select(ioregs_regs[1].read()));

            // Auto-enable timer for backward compatibility when auto-reload is set
            sc_uint<32> control_reg = ioregs_regs[7].read();
            control_reg |= 0x01; // Set CEN bit
            ioregs_regs[7].write(control_reg);
        }

        if(wb_write_cc1.read())
        {
            // Writing to CAPTURE_COMPARE_1 register (reg 3) with byte select
            ioregs_regs[3].write(write_with_byte_select(ioregs_regs[3].read()));
        }

        if(wb_write_cc2.read())
        {
            // Writing to CAPTURE_COMPARE_2 register (reg 4) with byte select
            ioregs_regs[4].write(write_with_byte_select(ioregs_regs[4].read()));
        }

        if(wb_write_cc3.read())
        {
            // Writing to CAPTURE_COMPARE_3 register (reg 5) with byte select
            ioregs_regs[5].write(write_with_byte_select(ioregs_regs[5].read()));
        }

        if(wb_write_cc4.read())
        {
            // Writing to CAPTURE_COMPARE_4 register (reg 6) with byte select
            ioregs_regs[6].write(write_with_byte_select(ioregs_regs[6].read()));
        }

        if(wb_write_ctrl_reg.read())
        {
            // Writing to CONTROL register (reg 7) with byte select
            ioregs_regs[7].write(write_with_byte_select(ioregs_regs[7].read()));
        }

        if(wb_write_status_reg.read())
        {
            // Writing to STATUS register (reg 8) with write-1-to-clear semantics
            // Writing 1 to a bit clears that bit, writing 0 leaves it unchanged
            sc_uint<32> current_status = ioregs_regs[8].read();

            // Get the data being written (without merging with current value)
            sc_uint<WB_DAT_WIDTH> mask = 0;
            sc_uint<WB_DAT_WIDTH / 8> wb_sel_i_var = wb_sel_i.read();

            // Build byte select mask
            if(wb_sel_i_var & 0b0001)
                mask |= 0x000000FF; // Byte 0
            if(wb_sel_i_var & 0b0010)
                mask |= 0x0000FF00; // Byte 1
            if(wb_sel_i_var & 0b0100)
                mask |= 0x00FF0000; // Byte 2
            if(wb_sel_i_var & 0b1000)
                mask |= 0xFF000000; // Byte 3

            // Get only the written data (mask it to only selected bytes)
            sc_uint<32> write_data = wb_dat_i.read() & mask;

            // Clear bits where write_data has 1s (write-1-to-clear)
            sc_uint<32> new_status = current_status & ~write_data;
            ioregs_regs[8].write(new_status);
        }

        if(wb_write_int_enable.read())
        {
            // Writing to INTERRUPT_ENABLE register (reg 9) with byte select
            ioregs_regs[9].write(write_with_byte_select(ioregs_regs[9].read()));
        }

        // Handle dummy register writes (STM32 compatibility)
        sc_uint<WB_ADR_WIDTH> reg_addr = wb_adr_i.read() - PN_CFG_TIMER_BASE_ADDRESS;
        switch(reg_addr)
        {
            case M_TIMER_REG_CR2: // 0x04
                dummy_cr2.write(write_with_byte_select(dummy_cr2.read()));
                break;
            case M_TIMER_REG_SMCR: // 0x08
                dummy_smcr.write(write_with_byte_select(dummy_smcr.read()));
                break;
            case M_TIMER_REG_CCMR1: // 0x18
                dummy_ccmr1.write(write_with_byte_select(dummy_ccmr1.read()));
                break;
            case M_TIMER_REG_CCMR2: // 0x1C
                dummy_ccmr2.write(write_with_byte_select(dummy_ccmr2.read()));
                break;
            case M_TIMER_REG_CCER: // 0x20
                dummy_ccer.write(write_with_byte_select(dummy_ccer.read()));
                break;
            case M_TIMER_REG_DCR: // 0x48
                dummy_dcr.write(write_with_byte_select(dummy_dcr.read()));
                break;
            case M_TIMER_REG_DMAR: // 0x4C
                dummy_dmar.write(write_with_byte_select(dummy_dmar.read()));
                break;
        }

        // Extract values from new register layout
        sc_uint<32> prescaler = ioregs_regs[2].read();   // PRESCALER register
        sc_uint<32> autoreload = ioregs_regs[1].read();  // AUTO_RELOAD register
        sc_uint<32> control_reg = ioregs_regs[7].read(); // CONTROL register
        sc_uint<32> status_reg = ioregs_regs[8].read();  // STATUS register (after any software writes)
        sc_uint<32> newstate = state.read();

        // Extract control bits
        bool counter_enable = (control_reg & 0x01); // CEN bit
        bool direction = (control_reg & 0x10);      // DIR bit (0=up, 1=down)
        bool one_pulse_mode = (control_reg & 0x08); // OPM bit

        // Only proceed if counter is enabled
        if(counter_enable)
        {
            // STM32 TIM2 compatible prescaler logic
            // PSC = 0 means divide by 1, PSC = N means divide by (N+1)
            sc_uint<32> prescaler_divisor = prescaler + 1;

            // Update prescaler counter
            sc_uint<16> prescaler_count = prescaler_counter.read();
            prescaler_count++;

            // Check if we should count (prescaler reached)
            bool should_count = (prescaler_count >= prescaler_divisor);
            if(should_count)
            {
                prescaler_count = 0; // Reset prescaler counter

                // Count up or down based on direction bit
                if(direction == 0)
                {
                    // Up counting
                    newstate++;

                    // STM32-compatible: Check Capture/Compare matches FIRST (before overflow reset)
                    if(newstate == ioregs_regs[3].read())
                    {                       // CAPTURE_COMPARE_1
                        status_reg |= 0x02; // Set CC1IF
                    }
                    if(newstate == ioregs_regs[4].read())
                    {                       // CAPTURE_COMPARE_2
                        status_reg |= 0x04; // Set CC2IF
                    }
                    if(newstate == ioregs_regs[5].read())
                    {                       // CAPTURE_COMPARE_3
                        status_reg |= 0x08; // Set CC3IF
                    }
                    if(newstate == ioregs_regs[6].read())
                    {                       // CAPTURE_COMPARE_4
                        status_reg |= 0x10; // Set CC4IF
                    }

                    // THEN check for auto-reload (overflow) - this ensures compare at reload value works
                    if(autoreload > 0 && newstate >= autoreload)
                    {
                        newstate = 0;
                        autoreload_pulse.write(true); // Signal autoreload occurred

                        // Set Update Interrupt Flag (UIF)
                        status_reg |= 0x01;

                        // In one-pulse mode, disable counter after overflow
                        if(one_pulse_mode)
                        {
                            control_reg &= ~0x01U; // Clear CEN bit
                            ioregs_regs[7].write(control_reg);
                        }
                    }
                    else
                    {
                        autoreload_pulse.write(false);
                    }
                }
                else
                {
                    // Down counting
                    if(newstate > 0)
                    {
                        newstate--;

                        // Check Capture/Compare matches for down counting
                        if(newstate == ioregs_regs[3].read())
                        {                       // CAPTURE_COMPARE_1
                            status_reg |= 0x02; // Set CC1IF
                        }
                        if(newstate == ioregs_regs[4].read())
                        {                       // CAPTURE_COMPARE_2
                            status_reg |= 0x04; // Set CC2IF
                        }
                        if(newstate == ioregs_regs[5].read())
                        {                       // CAPTURE_COMPARE_3
                            status_reg |= 0x08; // Set CC3IF
                        }
                        if(newstate == ioregs_regs[6].read())
                        {                       // CAPTURE_COMPARE_4
                            status_reg |= 0x10; // Set CC4IF
                        }
                    }
                    else
                    {
                        // Underflow - reload with autoreload value
                        newstate = autoreload;
                        autoreload_pulse.write(true);

                        // Set Update Interrupt Flag (UIF)
                        status_reg |= 0x01;

                        // In one-pulse mode, disable counter after underflow
                        if(one_pulse_mode)
                        {
                            control_reg &= ~0x01U; // Clear CEN bit
                            ioregs_regs[7].write(control_reg);
                        }
                    }
                }

                // Update STATUS register with new flags
                ioregs_regs[8].write(status_reg);
            }
            else
            {
                autoreload_pulse.write(false);
            }
            prescaler_counter.write(prescaler_count);
        }
        else
        {
            autoreload_pulse.write(false);
        }

        // Update registers
        state.write(newstate);
        ioregs_regs[0].write(newstate); // Update COUNTER register
    }
}

/**
 * Interrupt handler method - STM32-style interrupt logic
 */
void m_timer::proc_comb_handle_interrupts()
{
    // Extract status and interrupt enable register values
    sc_uint<32> status_reg = ioregs_regs[8].read();     // STATUS register
    sc_uint<32> int_enable_reg = ioregs_regs[9].read(); // INTERRUPT_ENABLE register

    // Check each interrupt source and generate interrupt if enabled
    bool interrupt_pending = false;

    // Update Interrupt (UIF & UIE)
    if((status_reg & 0x01) && (int_enable_reg & 0x01))
    {
        interrupt_pending = true;
    }

    // Capture/Compare 1 Interrupt (CC1IF & CC1IE)
    if((status_reg & 0x02) && (int_enable_reg & 0x02))
    {
        interrupt_pending = true;
    }

    // Capture/Compare 2 Interrupt (CC2IF & CC2IE)
    if((status_reg & 0x04) && (int_enable_reg & 0x04))
    {
        interrupt_pending = true;
    }

    // Capture/Compare 3 Interrupt (CC3IF & CC3IE)
    if((status_reg & 0x08) && (int_enable_reg & 0x08))
    {
        interrupt_pending = true;
    }

    // Capture/Compare 4 Interrupt (CC4IF & CC4IE)
    if((status_reg & 0x10) && (int_enable_reg & 0x10))
    {
        interrupt_pending = true;
    }

    // Trigger Interrupt (TIF & TIE)
    if((status_reg & 0x20) && (int_enable_reg & 0x20))
    {
        interrupt_pending = true;
    }

    // Break Interrupt (BIF & BIE)
    if((status_reg & 0x40) && (int_enable_reg & 0x40))
    {
        interrupt_pending = true;
    }

    // Set timer_irq output based on interrupt pending state
    timer_irq.write(interrupt_pending);
}

sc_uint<WB_DAT_WIDTH> m_timer::write_with_byte_select(
    sc_uint<WB_DAT_WIDTH> input_word)
{
    sc_uint<WB_DAT_WIDTH> mask = 0;
    sc_uint<WB_DAT_WIDTH / 8> wb_sel_i_var = wb_sel_i.read();

    // Selective byte masking
    if(wb_sel_i_var & 0b0001)
    {
        mask |= 0x000000FF; // Byte 0
    }
    if(wb_sel_i_var & 0b0010)
    {
        mask |= 0x0000FF00; // Byte 1
    }
    if(wb_sel_i_var & 0b0100)
    {
        mask |= 0x00FF0000; // Byte 2
    }
    if(wb_sel_i_var & 0b1000)
    {
        mask |= 0xFF000000; // Byte 3
    }

    return (input_word & ~mask) | (wb_dat_i.read() & mask);
}

sc_uint<WB_DAT_WIDTH> m_timer::read_with_byte_select(
    sc_uint<WB_DAT_WIDTH> input_word)
{
    sc_uint<WB_DAT_WIDTH> mask = 0;
    sc_uint<WB_DAT_WIDTH / 8> wb_sel_i_var = wb_sel_i.read();

    // Selective byte masking
    if(wb_sel_i_var & 0b0001)
    {
        mask |= 0x000000FF; // Byte 0
    }
    if(wb_sel_i_var & 0b0010)
    {
        mask |= 0x0000FF00; // Byte 1
    }
    if(wb_sel_i_var & 0b0100)
    {
        mask |= 0x00FF0000; // Byte 2
    }
    if(wb_sel_i_var & 0b1000)
    {
        mask |= 0xFF000000; // Byte 3
    }

    return (input_word & mask);
}
