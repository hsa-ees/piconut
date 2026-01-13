/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2025 Claus Janicher <claus.janicher@tha.de>
                     2025 Johannes Hofmann <johannes.hofmann1@tha.de>
                     2025 Lukas Bauer <lukas.bauer1@tha.de>
                     2025 Tristan Kundrat <tristan.kundrat@tha.de>
                     2025 Beaurel Ngaleu <beaurel.ingride.ngaleu@tha.de>
        
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


#include "wb_slave.h"

void m_wb_slave::pn_trace(sc_trace_file *gf, int level)
{
    PN_TRACE(gf, clk);
    PN_TRACE(gf, reset);
    PN_TRACE(gf, wb_ack_o);
    PN_TRACE(gf, wb_adr_i);
    PN_TRACE(gf, wb_current_state);
    PN_TRACE(gf, wb_cyc_i);
    PN_TRACE(gf, wb_dat_i);
    PN_TRACE(gf, wb_we_i);
    PN_TRACE(gf, wb_dat_o);
    PN_TRACE(gf, wb_sel_i);
    PN_TRACE(gf, wb_err_o);

    PN_TRACE(gf, reg_control);
    PN_TRACE(gf, reg_status);
    PN_TRACE(gf, reg_resolution_mode);
    PN_TRACE(gf, reg_resolution_mode_support);
    PN_TRACE(gf, reg_color_mode);
    PN_TRACE(gf, reg_color_mode_support);

    for (unsigned int i = 0; i < 256; i++) {
        PN_TRACE(gf, reg_color_map[i]);
    }
}

void m_wb_slave::proc_comb_wb_slave()
{
    // Alle Outputs zurücksetzen
    wb_ack_o = 0;
    wb_dat_o = 0;
    wb_err_o = 0;
    wb_rty_o = 0;

    wb_next_state = wb_current_state.read();

    bool wb_error_occured = false;
    uint32_t addr_offset = wb_adr_i.read() - CFG_WB_SLAVE_ADDRESS;

    switch (wb_current_state.read())
    {
        case WB_IDLE:
            if (wb_stb_i.read() && wb_cyc_i.read())
            {
                if ((CFG_WB_SLAVE_ADDRESS <= wb_adr_i.read()) &&
                    (wb_adr_i.read() < CFG_WB_SLAVE_ADDRESS + 0x500))
                {
                    if (wb_we_i.read())  // Write
                    {
                        wb_next_state = WB_WRITE1;
                    }
                    else  // Read
                    {
                        wb_next_state = WB_READ;
                    }
                }
                else
                {
                    // Adresse außerhalb Bereich -> Fehler setzen
                    wb_err_o = 1;
                    wb_next_state = WB_IDLE;
                }
            }
            break;

        case WB_WRITE1:
            if (!wb_we_i.read())
            {
                // Kein Write Enable - Fehler
                wb_err_o = 1;
                wb_next_state = WB_IDLE;
            }
            else
            {
                switch (addr_offset)
                {
                    case ADR_CONTROL:
                        reg_control.write(
                            apply_byte_select(reg_control.read(), wb_dat_i.read(), wb_sel_i.read())
                        );
                        break;

                    // TODO: Weitere Register hier behandeln
                    default:
                        wb_err_o = 1;
                        wb_next_state = WB_IDLE;
                        break;
                }

                if (!wb_err_o)
                {
                    wb_next_state = WB_WRITE2;  // ACK-Zustand
                }
            }
            break;

        case WB_WRITE2:
            wb_ack_o = 1;
            wb_next_state = WB_IDLE;
            break;

        case WB_READ:
            switch (addr_offset)
            {
                case ADR_CONTROL:
                    wb_dat_o.write(read_with_byte_select(reg_control.read(), wb_sel_i.read()));
                    break;

                case ADR_STATUS:
                    wb_dat_o.write(read_with_byte_select(reg_status.read(), wb_sel_i.read()));
                    break;

                case ADR_RESOLUTION_MODE:
                    wb_dat_o.write(read_with_byte_select(reg_resolution_mode.read(), wb_sel_i.read()));
                    break;

                case ADR_RESOLUTION_MODE_SUPPORT:
                    wb_dat_o.write(read_with_byte_select(reg_resolution_mode_support.read(), wb_sel_i.read()));
                    break;

                case ADR_COLOR_MODE:
                    wb_dat_o.write(read_with_byte_select(reg_color_mode.read(), wb_sel_i.read()));
                    break;

                case ADR_COLOR_MODE_SUPPORT:
                    wb_dat_o.write(read_with_byte_select(reg_color_mode_support.read(), wb_sel_i.read()));
                    break;

                default:
                    wb_error_occured = true;
                    break;
            }

            wb_ack_o = wb_error_occured ? 0 : 1;
            wb_err_o = wb_error_occured ? 1 : 0;
            wb_next_state = WB_IDLE;
            break;

        default:
            wb_next_state = WB_IDLE;
            break;
    }
}


void m_wb_slave::proc_clk_state()
{
    wb_current_state = WB_IDLE;

    while (true)
    {
        wait();
        wb_current_state = wb_next_state;
    }
}

uint32_t m_wb_slave::apply_byte_select(uint32_t current_data, uint32_t new_data, uint8_t byte_select) {
    uint32_t mask = 0;
    uint32_t ret_buffer, full_buffer;
    bool full_selection = false;

    if (byte_select == 0xF) {
        full_buffer = new_data;
        full_selection = true;
    } 
    else if (byte_select == 0xC) {
        mask = 0xFFFF0000;
    } 
    else if (byte_select == 0x3) {
        mask = 0x0000FFFF;
    } 
    else {
        if (byte_select & 0x1) mask |= 0x000000FF;
        if (byte_select & 0x2) mask |= 0x0000FF00;
        if (byte_select & 0x4) mask |= 0x00FF0000;
        if (byte_select & 0x8) mask |= 0xFF000000;
    }

    if (full_selection) {
        ret_buffer = full_buffer;
    } else {
        ret_buffer = (current_data & ~mask) | (new_data & mask);
    }

    return ret_buffer;
}

uint32_t m_wb_slave::read_with_byte_select(uint32_t data, uint8_t byte_select) {
    uint32_t extracted_data = 0;
    uint32_t ret_buffer;
    bool simple_selection = false;

    if (byte_select == 0xF) {
        ret_buffer = data;
    } 
    else if (byte_select == 0xC) {
        ret_buffer = (data & 0xFFFF0000);
    } 
    else if (byte_select == 0x3) {
        ret_buffer = (data & 0x0000FFFF);
    } 
    else {
        if (byte_select & 0x1) extracted_data |= (data & 0x000000FF);
        if (byte_select & 0x2) extracted_data |= (data & 0x0000FF00);
        if (byte_select & 0x4) extracted_data |= (data & 0x00FF0000);
        if (byte_select & 0x8) extracted_data |= (data & 0xFF000000);
        simple_selection = true;
    }

    if (simple_selection) {
        ret_buffer = extracted_data;
    }

    return ret_buffer;
}
