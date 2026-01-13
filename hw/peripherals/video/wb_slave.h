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

/**
 * @fn SC_MODULE(m_wb_slave)
 * @author Beaurel I. Ngaleu
 * @brief
 * The "wb_slave" serves as the central interface for control and communication
 * via the Wishbone bus. It enables reading and writing of register values or
 * memory data within the wb_graphics_module. It responds to bus accesses to
 * execute internal control functions (e.g., framebuffer accesses).
 *
 * @par Ports:
 * @param[in] clk clock of the module
 * @param[in] reset reset of the module
 * @param[in] wb_adr_i Wishbone Adress input
 * @param[out] wb_dat_o Wishbone Data out
 * @param[in] wb_dat_i Wishbone Data in
 * @param[in] wb_we_i Wishbone Write-Enable in
 * @param[in] wb_stb_i Wishbone Strobe in
 * @param[in] wb_cyc_i Wishbone Cycle in
 * @param[in] wb_we_i
 * @param[out] wb_ack_o Wishbone ACK input
 * @param[in] wb_cti_i // Wishbone cycle type identifier (optional, for registered feedback)
 * @param[in] wb_bte_i // Wishbone burst type extension  (optional, for registered feedback)
 * @param[out] wb_error_o // Wishbone termination w/ error (optional)
 * @param[out] wb_rty_o // Wishbone termination w/ retry (optional)
 * @param[out] reg_control Register zur Steuerung
 * @param[in]  reg_status Statusregister
 * @param[in]  reg_line Aktuelle Zeile aus Statusmodul
 * @param[out] reg_resolution_mode Ausgewählter Auflösungsmodus
 * @param[in]  reg_resolution_mode_support Unterstützte Auflösungen
 * @param[out] reg_color_mode Ausgewählter Farbmodus
 * @param[in]  reg_color_mode_support Unterstützte Farbmodi
 *
 *
 */

#ifndef _WB_GRAPHICS_H_
#define _WB_GRAPHICS_H_

#include <systemc.h>
#include "wb_slave_config.h"
#include <piconut.h>

#ifndef WB_DAT_WIDTH
#define WB_DAT_WIDTH 32
#endif

#ifndef WB_ADR_WIDTH
#define WB_ADR_WIDTH 32
#endif

#define CFG_WB_SLAVE_ADDRESS 0x40000000 // Basisadresse

SC_MODULE(m_wb_slave)
{

public:
    enum e_wb_state // Beschreibung der verschiedenen Zustände eines Wishbone-Slaves im State-Machine-Modell
    {
        WB_IDLE = 0, // Startzustand
        WB_READ,
        WB_WRITE1,
        WB_WRITE2
    };

    enum e_wb_addressrange // verschiedene Register und Speicherbereiche aufgelistet
    {
        ADR_CONTROL = 0x000,
        ADR_STATUS = 0x004,
        ADR_RESOLUTION_MODE = 0x008,
        ADR_RESOLUTION_MODE_SUPPORT = 0x00C,
        ADR_COLOR_MODE = 0x010,
        ADR_COLOR_MODE_SUPPORT = 0x014,
        ADR_LINE = 0x018,
        INTERN_FB_ADDRESS = 0x01C,
        INTERN_FB_DATA = 0x100,
    };

    // Wishbone ports & Signale
    sc_in_clk PN_NAME(clk);     // Der Takt für das Modul
    sc_in<bool> PN_NAME(reset); // Reset Siganl

    sc_in<bool> PN_NAME(wb_stb_i);                      // Siganl für Wishbone-Status (Strobe)
    sc_in<bool> PN_NAME(wb_cyc_i);                      // cycle
    sc_in<sc_uint<WB_DAT_WIDTH / 8>> PN_NAME(wb_sel_i); // Byte-Selektoren
    sc_out<bool> PN_NAME(wb_ack_o);                     // Output für Acknowledge
    sc_out<bool> PN_NAME(wb_err_o);                     // Output für Error
    sc_out<bool> PN_NAME(wb_rty_o);                     // Output für Retry

    sc_in<sc_uint<WB_ADR_WIDTH>> PN_NAME(wb_adr_i);  // Signal für Adressen
    sc_in<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_i);  // Signal für Eingabedaten
    sc_out<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_o); // Output für die zurückgegebenen Daten
    sc_in<bool> PN_NAME(wb_we_i);

    sc_out<sc_uint<FB_RAM_ADDR_WIDTH>> fb_addr;
    sc_out<bool> fb_write_en;
    sc_in<sc_uint<FB_RAM_DATA_WIDTH>> fb_data_read;
    sc_out<sc_uint<FB_RAM_DATA_WIDTH>> fb_data_write;

    /** Register */
    sc_signal<sc_uint<32>> state;
    sc_signal<sc_uint<3>> PN_NAME(wb_current_state);
    sc_signal<sc_uint<3>> PN_NAME(wb_next_state);

    /** Graphics registers */
    sc_out<sc_uint<32>> PN_NAME(reg_control);
    sc_in<sc_uint<32>> PN_NAME(reg_status);
    sc_in<sc_uint<32>> PN_NAME(reg_line);
    sc_out<sc_uint<5>> PN_NAME(reg_resolution_mode);
    sc_in<sc_uint<32>> PN_NAME(reg_resolution_mode_support);
    sc_out<sc_uint<5>> PN_NAME(reg_color_mode);
    sc_in<sc_uint<32>> PN_NAME(reg_color_mode_support);
    sc_signal<sc_uint<32>> reg_color_map[256];

    SC_CTOR(m_wb_slave)
    {
        SC_METHOD(proc_comb_wb_slave);
        sensitive << wb_stb_i << wb_cyc_i << wb_we_i << wb_adr_i << wb_dat_i
                  << wb_current_state << wb_sel_i << fb_addr
                  << fb_data_write << fb_data_read << fb_data_write
                  << reg_resolution_mode_support << reg_color_mode
                  << reg_color_mode_support << reg_line;

        SC_CTHREAD(proc_clk_state, clk.pos()); // erstellt clock sensitive Process
        reset_signal_is(reset, true);
    }

    void pn_trace(sc_trace_file * gf, int level = 1);
    void proc_clk_wb_slave();
    void proc_clk_state();
    void proc_comb_wb_slave();

    uint32_t apply_byte_select(uint32_t input_word, uint32_t resulting_word, uint8_t byte_select);
    uint32_t read_with_byte_select(uint32_t input_word, uint8_t byte_select);
};

#endif //_WB_GRAPHICS_H_