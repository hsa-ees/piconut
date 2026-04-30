/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Daniel Sommerfeldt <daniel.sommerfeldt1@tha.de>
                2026 Johannes Hofmann <johannes.hofmann1@tha.de>
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

#include "gpio.h"
#include "gpio_defs.h"

namespace {

pn_wb_dat_t write_with_byte_select(
    pn_wb_dat_t word,
    pn_wb_dat_t new_word,
    pn_wb_sel_t sel)
{
    for(int i = 0; i < PN_WISHBONE_DLEN_MAX / 8; ++i)
        if(sel[i])
            word.range(i * 8 + 7, i * 8) = new_word.range(i * 8 + 7, i * 8);

    return word;
}

pn_wb_dat_t read_with_byte_select(
    pn_wb_dat_t word,
    pn_wb_sel_t sel)
{
    for(int i = 0; i < PN_WISHBONE_DLEN_MAX / 8; ++i)
        if(!sel[i])
            word.range(i * 8 + 7, i * 8) = 0; // Clear unselected bytes

    return word;
}

} // namespace

void m_gpio::pn_trace(sc_trace_file* tf, int level)
{
    // Ports ...
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);
    PN_TRACE_INTERFACE(tf, wb_slave);
    PN_TRACE(tf, input);
    PN_TRACE(tf, output);

    // Internal ...
    PN_TRACE(tf, r_input_val);
    PN_TRACE(tf, r_input_en);
    PN_TRACE(tf, r_output_en);
    PN_TRACE(tf, r_output_val);
    PN_TRACE(tf, c_wb_write_en);
    PN_TRACE(tf, r_wb_current_state);
}

void m_gpio::proc_cmb_wb_slave()
{
    wb_slave.ack_o = 0;
    wb_slave.dat_o = 0;
    wb_slave.err_o = 0;
    wb_slave.rty_o = 0;

    c_wb_write_en = 0;
    wb_next_state = r_wb_current_state.read();

    switch(r_wb_current_state.read())
    {
        case WB_IDLE:
            if(wb_slave.stb_i.read() == 1 && wb_slave.cyc_i.read() == 1)
            {
                if(wb_slave.adr_i.read() >= wb_slave.base_address &&
                    wb_slave.adr_i.read() < wb_slave.base_address + wb_slave.size)
                {
                    if(wb_slave.we_i.read() == 1)
                    {
                        c_wb_write_en = 1;
                        wb_next_state = WB_WRITE;
                    }
                    else
                    {
                        wb_next_state = WB_READ;
                    }
                }
            }
            break;

        case WB_WRITE:
            wb_slave.ack_o = 1;

            if(wb_slave.stb_i.read() == 0 && wb_slave.cyc_i.read() == 0)
            {
                wb_next_state = WB_IDLE;
            }
            break;

        case WB_READ:
            switch(wb_slave.adr_i.read() - wb_slave.base_address)
            {
                case e_gpio_regs::GPIO_REG_INPUT_VAL:
                    wb_slave.dat_o = read_with_byte_select(r_input_val.read(), wb_slave.sel_i.read());
                    break;
                case e_gpio_regs::GPIO_REG_INPUT_EN:
                    wb_slave.dat_o = read_with_byte_select(r_input_en.read(), wb_slave.sel_i.read());
                    break;
                case e_gpio_regs::GPIO_REG_OUTPUT_EN:
                    wb_slave.dat_o = read_with_byte_select(r_output_en.read(), wb_slave.sel_i.read());
                    break;
                case e_gpio_regs::GPIO_REG_OUTPUT_VAL:
                    wb_slave.dat_o = read_with_byte_select(r_output_val.read(), wb_slave.sel_i.read());
                    break;
            }

            wb_slave.ack_o = 1;

            if(wb_slave.stb_i.read() == 0 && wb_slave.cyc_i.read() == 0)
            {
                wb_next_state = WB_IDLE;
            }
            break;
    }
}

void m_gpio::proc_clk_state()
{
    while(true)
    {
        if(reset.read() == 1)
        {
            r_wb_current_state = WB_IDLE;
        }
        else
        {
            r_wb_current_state = wb_next_state.read();
        }

        wait();
    }
}

void m_gpio::proc_clk_wb_slave()
{
    while(true)
    {
        if(reset.read() == 1)
        {
            r_input_en = 0;
            r_output_en = 0;
            r_output_val = 0;
        }
        else
        {
            if(c_wb_write_en.read() == 1)
            {
                switch(wb_slave.adr_i.read() - wb_slave.base_address)
                {
                    case e_gpio_regs::GPIO_REG_INPUT_VAL:
                        // Read only
                        break;
                    case e_gpio_regs::GPIO_REG_INPUT_EN:
                        r_input_en = (num_inputs <= 0) ? 0 : write_with_byte_select(r_input_en.read().range(num_inputs - 1, 0), wb_slave.dat_i.read(), wb_slave.sel_i.read()).range(num_inputs - 1, 0);
                        break;
                    case e_gpio_regs::GPIO_REG_OUTPUT_EN:
                        r_output_en = (num_outputs <= 0) ? 0 : write_with_byte_select(r_output_en.read().range(num_outputs - 1, 0), wb_slave.dat_i.read(), wb_slave.sel_i.read()).range(num_outputs - 1, 0);
                        break;
                    case e_gpio_regs::GPIO_REG_OUTPUT_VAL:
                        r_output_val = (num_outputs <= 0) ? 0 : write_with_byte_select(r_output_val.read().range(num_outputs - 1, 0), wb_slave.dat_i.read(), wb_slave.sel_i.read()).range(num_outputs - 1, 0);
                        break;
                }
            }
        }

        wait();
    }
}

void m_gpio::proc_cmb_gpio_output()
{
    // clang-format off
    output = ((num_outputs <= 0) ? 0 : r_output_val.read().range(num_outputs - 1, 0)) & 
             ((num_outputs <= 0) ? 0 : r_output_en.read().range(num_outputs - 1, 0));
    // clang-format on
}

void m_gpio::proc_clk_gpio_input()
{
    while(true)
    {
        if(reset.read() == 1)
        {
            r_input_val = 0;
        }
        else
        {
            // clang-format off
            r_input_val = ((num_inputs <= 0) ? 0 : input.read().range(num_inputs - 1, 0)) &
                          ((num_inputs <= 0) ? 0 : r_input_en.read().range(num_inputs - 1, 0));
            // clang-format on
        }

        wait();
    }
}
