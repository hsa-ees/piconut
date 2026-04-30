/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)     2025 Eikya Lagisetti <eikya.lagisetti1@tha.de>
                    2025 Sebastian Ebenhöh <sebastian.moritz.ebenhoeh@tha.de>
                    2025 Johannes Fleiner <Johannes.Fleiner1@tha.de>

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

#include "i2c_wishbone.h"


void m_i2c_wishbone::pn_trace(sc_trace_file* tf, int level)
{

    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE(tf, wb_slave.ack_o);
    PN_TRACE(tf, wb_slave.adr_i);
    PN_TRACE(tf, wb_current_state);
    PN_TRACE(tf, wb_slave.cyc_i);
    PN_TRACE(tf, wb_slave.stb_i);
    PN_TRACE(tf, wb_slave.dat_i);
    PN_TRACE(tf, wb_slave.dat_o);
    PN_TRACE(tf, wb_slave.sel_i);
    PN_TRACE(tf, wb_slave.we_i);
    PN_TRACE(tf, wb_slave.err_o);

    PN_TRACE(tf, CR1);
    PN_TRACE(tf, CR2);
    PN_TRACE(tf, OAR1);
    PN_TRACE(tf, OAR2);
    PN_TRACE(tf, DR);
    PN_TRACE(tf, SR1);
    PN_TRACE(tf, SR2);
    PN_TRACE(tf, CCR);
    PN_TRACE(tf, TRISE);
    PN_TRACE(tf, FLTR);

    PN_TRACE(tf, c_wb_write_en);

    PN_TRACE(tf, clear_cr1_start_in);
    PN_TRACE(tf, clear_cr1_stop_in);
    PN_TRACE(tf, CPU_read_SR2_out);
    PN_TRACE(tf, CPU_read_SR1_out);
    PN_TRACE(tf, CPU_write_DR_out);
    PN_TRACE(tf, clear_regs_in);
    PN_TRACE(tf, shift_data_in);
    PN_TRACE(tf, enable_dr_write_in);
    PN_TRACE(tf, dr_out);
    PN_TRACE(tf, tx_data_out);
    PN_TRACE(tf, cr1_enable_out);
    PN_TRACE(tf, cr1_start_out);
    PN_TRACE(tf, cr1_stop_out);
    PN_TRACE(tf, sr1_start_bit_in);
    PN_TRACE(tf, sr1_addr_bit_in);
    PN_TRACE(tf, sr1_txe_in);
    PN_TRACE(tf, sr1_btf_in);
    PN_TRACE(tf, sr1_af_in);
    PN_TRACE(tf, sr2_msl_in);
    PN_TRACE(tf, sr2_busy_in);
    PN_TRACE(tf, sr2_tra_in);
    PN_TRACE(tf, sr1_rxne_in);
}

void m_i2c_wishbone::proc_comb_wb_slave() // Wishbone Statetransition logic (combinatory)
{
    // set idle Outputs to default
    wb_slave.ack_o = 0;
    wb_slave.dat_o = 0;
    wb_slave.err_o = 0;
    wb_slave.rty_o = 0;

    c_wb_write_en = 0;

    CPU_read_SR1_out.write(false);
    CPU_read_SR2_out.write(false);
    CPU_read_DR_out.write(false);

    wb_next_state = wb_current_state.read(); // default next state is current state

    switch(wb_current_state.read())
    {
        case WB_IDLE: // Wishbone Idle State to check if we have a valid transaction

            if(wb_slave.stb_i.read() == 1 && wb_slave.cyc_i.read() == 1) // WB Strobe and cycle valid,
            {
                if((PN_CFG_I2C_BASE_ADDRESS <= wb_slave.adr_i.read()) &&
                    (wb_slave.adr_i.read() < (PN_CFG_I2C_BASE_ADDRESS + PN_I2C_REG_SIZE_BYTES)))
                {
                    if(wb_slave.we_i.read() == 1)
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

            c_wb_write_en = 1;

            // set the next wishbone state because writing is not finished yet
            wb_next_state = WB_WRITE2;
            break;

        case WB_WRITE2: // Write Acknowledge

            c_wb_write_en = 1;
            wb_slave.ack_o = 1;

            // after setting the wishbone ack the writing is finished and we can go back to the idle state
            if(wb_slave.stb_i.read() == 0 && wb_slave.cyc_i.read() == 0)
            {
                wb_next_state = WB_IDLE;
            }
            break;

        case WB_READ: // Master Reads from us -> Write data to the bus output

            // Address offset within the block
            const sc_uint<PN_CFG_WB_ADDR_WIDTH> off = wb_slave.adr_i.read() - PN_CFG_I2C_BASE_ADDRESS;
            sc_uint<PN_CFG_WB_DATA_WIDTH> dout = 0;

            switch(off.to_uint())
            {
                case I2C_REG_CR1:
                    dout = CR1.read();
                    break;
                case I2C_REG_CR2:
                    dout = CR2.read();
                    break;
                case I2C_REG_OAR1:
                    dout = OAR1.read();
                    break;
                case I2C_REG_OAR2:
                    dout = OAR2.read();
                    break;
                case I2C_REG_DR:
                    dout = (DR.read() & 0xFFu);
                    CPU_read_DR_out.write(true);
                    break; // only LSB valid
                case I2C_REG_SR1:
                    dout = SR1.read();
                    CPU_read_SR1_out.write(true);
                    break;
                case I2C_REG_SR2:
                    dout = SR2.read();
                    CPU_read_SR2_out.write(true);
                    break;
                case I2C_REG_CCR:
                    dout = CCR.read();
                    break;
                case I2C_REG_TRISE:
                    dout = TRISE.read();
                    break;
                default:
                    dout = 0;
                    break;
            }

            // Optional byte-lane mask on reads
            wb_slave.dat_o = (sc_uint<32>(0), read_with_byte_select(dout));
            wb_slave.ack_o = 1;

            if(wb_slave.stb_i.read() == 0 && wb_slave.cyc_i.read() == 0)
            {
                wb_next_state = WB_IDLE;
            }
            break;
    }
}

void m_i2c_wishbone::proc_clk_wb_slave()
{
    CR1 = 0;
    CR2 = 0;
    OAR1 = 0;
    OAR2 = 0;
    DR = 0;
    SR1 = 0;
    SR2 = 0;
    CCR = 0;
    TRISE = 0x02;
    FLTR = 0;

    while(true)
    {
        wait();
        if(clear_regs_in.read() == false)
        {

            CPU_write_DR_out.write(false);

            if(c_wb_write_en.read() == 1)
            {
                switch(wb_slave.adr_i.read() - PN_CFG_I2C_BASE_ADDRESS) // use Address offset to determine target register
                {
                    case I2C_REG_CR1:
                        // write seomthing in the ADR_REG_0 addressrange

                        CR1 = write_with_byte_select(CR1.read());

                        break;

                    case I2C_REG_CR2:
                        // write something in the ADR_REG_1 addressrange
                        CR2 = write_with_byte_select(CR2.read());
                        break;

                    case I2C_REG_OAR1:

                        OAR1 = write_with_byte_select(OAR1.read());
                        break;

                    case I2C_REG_OAR2:

                        OAR2 = write_with_byte_select(OAR2.read());
                        break;

                    case I2C_REG_DR:

                        DR = write_with_byte_select(DR.read());
                        CPU_write_DR_out.write(true);

                        break;

                    case I2C_REG_SR1:
                        // Read-only - ignore writes
                        break;

                    case I2C_REG_SR2:
                        // Read-only - ignore writes
                        break;

                    case I2C_REG_CCR:
                        CCR = write_with_byte_select(CCR.read());
                        break;

                    case I2C_REG_TRISE:
                        TRISE = write_with_byte_select(TRISE.read());
                        break;

                    case I2C_REG_FLTR:
                        FLTR = write_with_byte_select(FLTR.read());
                        break;
                    default:
                        break;
                }
            }
            // Update signals from/to submodules
            // CR1
            cr1_enable_out.write((CR1.read() & I2C_CR1_PE) != 0);
            cr1_start_out.write((CR1.read() & I2C_CR1_START) != 0);
            cr1_stop_out.write((CR1.read() & I2C_CR1_STOP) != 0);
            cr1_ack_out.write((CR1.read() & I2C_CR1_ACK) != 0);

            if(clear_cr1_start_in.read())
            {
                CR1.write(CR1.read() & ~I2C_CR1_START);
            }

            if(clear_cr1_stop_in.read())
            {
                CR1.write(CR1.read() & ~I2C_CR1_STOP);
            }

            // SR1 Register
            sc_uint<PN_CFG_WB_DATA_WIDTH> sr1_val = SR1.read();
            sr1_val = (sr1_val & ~(I2C_SR1_SB | I2C_SR1_ADDR | I2C_SR1_BTF | I2C_SR1_TXE | I2C_SR1_AF | I2C_SR1_RXNE)) | (sr1_start_bit_in.read() ? I2C_SR1_SB : 0) | (sr1_addr_bit_in.read() ? I2C_SR1_ADDR : 0) | (sr1_btf_in.read() ? I2C_SR1_BTF : 0) | (sr1_txe_in.read() ? I2C_SR1_TXE : 0) | (sr1_af_in.read() ? I2C_SR1_AF : 0) | (sr1_rxne_in.read() ? I2C_SR1_RXNE : 0);
            SR1.write(sr1_val);

            // SR2
            sc_uint<PN_CFG_WB_DATA_WIDTH> sr2_val = SR2.read();
            sr2_val = (sr2_val & ~(I2C_SR2_MSL | I2C_SR2_BUSY | I2C_SR2_TRA)) | (sr2_msl_in.read() ? I2C_SR2_MSL : 0) | (sr2_busy_in.read() ? I2C_SR2_BUSY : 0) | (sr2_tra_in.read() ? I2C_SR2_TRA : 0);
            SR2.write(sr2_val);

            // DR
            if(enable_dr_write_in.read())
            {
                DR.write(shift_data_in.read() & 0xFF);
            }

            dr_out.write(DR.read() & 0xFF);
            tx_data_out.write(DR.read() & 0xFF);

            // CCR and TRISE
            ccr_out.write(CCR.read() & 0xFFFF);
            trise_out.write(TRISE.read() & 0x3F);
        }
        else
        {
            reset_all_registers();
        }
    }
}

// state transition process
void m_i2c_wishbone::proc_clk_state()
{
    wb_current_state = WB_IDLE;

    while(true)
    {
        wait();

        wb_current_state = wb_next_state;
    }
}

void m_i2c_wishbone::reset_all_registers()
{
    CR1.write(CR1.read() & ~I2C_CR1_START);
    CR1.write(CR1.read() & ~I2C_CR1_STOP);
    DR.write(0);
    SR1.write(0);
    SR2.write(0);
    FLTR.write(0);
}

sc_uint<PN_CFG_WB_DATA_WIDTH> m_i2c_wishbone::write_with_byte_select(
    sc_uint<PN_CFG_WB_DATA_WIDTH> input_word)
{
    sc_uint<PN_CFG_WB_DATA_WIDTH> mask = 0;
    sc_uint<PN_CFG_WB_DATA_WIDTH / 8> wb_sel_i_var = wb_slave.sel_i.read().range(32 / 8 - 1, 0);

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

    return (input_word & ~mask) | (wb_slave.dat_i.read().range(31, 0) & mask);
}

sc_uint<PN_CFG_WB_DATA_WIDTH> m_i2c_wishbone::read_with_byte_select(
    sc_uint<PN_CFG_WB_DATA_WIDTH> input_word)
{
    sc_uint<PN_CFG_WB_DATA_WIDTH> mask = 0;
    sc_uint<PN_CFG_WB_DATA_WIDTH / 8> wb_sel_i_var = wb_slave.sel_i.read().range(32 / 8 - 1, 0);

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
