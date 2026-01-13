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

#ifndef __HW_TIMER_H__
#define __HW_TIMER_H__

#include <systemc.h>
#include <piconut.h>
#include <piconut-config.h>
#include "timer_defs.h"

#ifndef WB_DAT_WIDTH
#define WB_DAT_WIDTH 32
#endif

#ifndef WB_ADR_WIDTH
#define WB_ADR_WIDTH 32
#endif

SC_MODULE(m_timer)
{

public:
    // STM32 TIM2 compatible register offsets
    typedef enum
    {
        WB_READ = 0,
        WB_WRITE1,
        WB_WRITE2,
        WB_IDLE
    } e_wb_state;

    // Wisbone slave ports ...
    sc_in_clk PN_NAME(clk);     // clock input
    sc_in<bool> PN_NAME(reset); // reset

    sc_in<bool> PN_NAME(wb_stb_i);                      // strobe input
    sc_in<bool> PN_NAME(wb_cyc_i);                      // cycle valid input
    sc_in<bool> PN_NAME(wb_we_i);                       // indicates write transfer
    sc_in<sc_uint<WB_DAT_WIDTH / 8>> PN_NAME(wb_sel_i); // byte select inputs
    sc_out<bool> PN_NAME(wb_ack_o);                     // normal termination
    sc_out<bool> PN_NAME(wb_err_o);                     // termination w/ error (optional)
    sc_out<bool> PN_NAME(wb_rty_o);                     // termination w/ retry (optional)

    sc_in<sc_uint<WB_ADR_WIDTH>> PN_NAME(wb_adr_i);  // address bus inputs
    sc_in<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_i);  // input data bus
    sc_out<sc_uint<WB_DAT_WIDTH>> PN_NAME(wb_dat_o); // output data bus

    sc_out<bool> PN_NAME(timer_irq); // Timer interrupt request output

    // Autoreload pulse output - high for one cycle when autoreload occurs
    sc_out<bool> PN_NAME(autoreload_pulse); // Autoreload pulse output

    SC_CTOR(m_timer)
    { // constructor
        // Initialize outputs
        timer_irq.initialize(false);
        autoreload_pulse.initialize(false);

        SC_METHOD(proc_comb_wb_slave);
        sensitive << wb_stb_i << wb_cyc_i << wb_we_i << wb_adr_i << wb_dat_i << wb_sel_i << wb_current_state << reset;
        for(int i = 0; i < ioregs_num; i++)
        {
            sensitive << ioregs_regs[i];
        }
        // Add dummy registers to sensitivity list
        sensitive << dummy_cr2 << dummy_smcr << dummy_ccmr1 << dummy_ccmr2
                  << dummy_ccer << dummy_dcr << dummy_dmar;

        SC_CTHREAD(proc_clk_state, clk.pos());
        reset_signal_is(reset, true);

        SC_CTHREAD(proc_clk_transition, clk.pos()); // state transition process
        reset_signal_is(reset, true);

        SC_METHOD(proc_comb_handle_interrupts);
        sensitive << ioregs_regs[8] << ioregs_regs[9]; // STATUS and INTERRUPT_ENABLE registers
    }

    void pn_trace(
        sc_trace_file * tf,
        int level = 1);

    /**
     * @brief Handels the wishbone slave functions */
    void proc_clk_wb_slave();
    void proc_clk_state();

    /**
     * @brief Handels the wishbone slave functions */
    void proc_comb_wb_slave();

    void proc_clk_transition();

    void proc_comb_handle_interrupts();

    /**
     * @brief This function is for the write byte select functionality of the wishbone bus.
     * @param input_word Hand over the Dataword you would like to get changed.
     */
    sc_uint<32> write_with_byte_select(sc_uint<32> input_word);

    /**
     * @brief This function is for the read byte select functionality of the wishbone bus.
     * @param input_word Hand over the Dataword you would like to get changed.
     */
    sc_uint<32> read_with_byte_select(sc_uint<32> input_word);

    // Control register bit structure
    union control_reg_t
    {
        struct
        {
            sc_uint<1> counter_enable;                // Bit 0: CEN - Counter Enable
            sc_uint<1> update_disable;                // Bit 1: UDIS - Update Disable
            sc_uint<1> update_request_source;         // Bit 2: URS - Update Request Source
            sc_uint<1> one_pulse_mode;                // Bit 3: OPM - One Pulse Mode
            sc_uint<1> direction;                     // Bit 4: DIR - Direction (0=up, 1=down)
            sc_uint<2> center_aligned_mode_selection; // Bit 5-6: CMS - Center-aligned Mode
            sc_uint<1> auto_reload_preload_enable;    // Bit 7: ARPE - Auto-reload Preload Enable
            sc_uint<24> reserved;                     // Bit 8-31: Reserved
        } bits;
        sc_uint<32> value;
    };

    // Status register bit structure
    union status_reg_t
    {
        struct
        {
            sc_uint<1> update_interrupt_flag;  // Bit 0: UIF - Update Interrupt Flag
            sc_uint<1> capture_compare_1_flag; // Bit 1: CC1IF - Capture/Compare 1 Flag
            sc_uint<1> capture_compare_2_flag; // Bit 2: CC2IF - Capture/Compare 2 Flag
            sc_uint<1> capture_compare_3_flag; // Bit 3: CC3IF - Capture/Compare 3 Flag
            sc_uint<1> capture_compare_4_flag; // Bit 4: CC4IF - Capture/Compare 4 Flag
            sc_uint<1> trigger_interrupt_flag; // Bit 5: TIF - Trigger Interrupt Flag
            sc_uint<1> break_interrupt_flag;   // Bit 6: BIF - Break Interrupt Flag
            sc_uint<25> reserved;              // Bit 7-31: Reserved
        } bits;
        sc_uint<32> value;
    };

    // Interrupt enable register bit structure
    union interrupt_enable_reg_t
    {
        struct
        {
            sc_uint<1> update_interrupt_enable;  // Bit 0: UIE - Update Interrupt Enable
            sc_uint<1> capture_compare_1_enable; // Bit 1: CC1IE - Capture/Compare 1 Interrupt Enable
            sc_uint<1> capture_compare_2_enable; // Bit 2: CC2IE - Capture/Compare 2 Interrupt Enable
            sc_uint<1> capture_compare_3_enable; // Bit 3: CC3IE - Capture/Compare 3 Interrupt Enable
            sc_uint<1> capture_compare_4_enable; // Bit 4: CC4IE - Capture/Compare 4 Interrupt Enable
            sc_uint<1> trigger_interrupt_enable; // Bit 5: TIE - Trigger Interrupt Enable
            sc_uint<1> break_interrupt_enable;   // Bit 6: BIE - Break Interrupt Enable
            sc_uint<25> reserved;                // Bit 7-31: Reserved
        } bits;
        sc_uint<32> value;
    };

protected:
    /** Registers... */

    // Total number of registers: 10 => log2(16) = 4 (next power of 2)
    static const int ioregs_num_ld = 4; // Log2 of the number of WB slave registers
    static const int ioregs_num = 10;   // Number of WB slave registers (exactly 10)
    sc_vector<sc_signal<sc_uint<WB_DAT_WIDTH>>> PN_NAME_VEC(ioregs_regs, ioregs_num);

    sc_signal<sc_uint<32>> state;

    sc_signal<sc_uint<3>> PN_NAME(wb_current_state);
    sc_signal<sc_uint<3>> PN_NAME(wb_next_state);

    // Internal signals for wishbone write operations
    sc_signal<bool> PN_NAME(wb_write_control);    // Used for PRESCALER register
    sc_signal<bool> PN_NAME(wb_write_reload);     // Used for AUTO_RELOAD register
    sc_signal<bool> PN_NAME(wb_write_cc1);        // Used for CAPTURE_COMPARE_1 register
    sc_signal<bool> PN_NAME(wb_write_cc2);        // Used for CAPTURE_COMPARE_2 register
    sc_signal<bool> PN_NAME(wb_write_cc3);        // Used for CAPTURE_COMPARE_3 register
    sc_signal<bool> PN_NAME(wb_write_cc4);        // Used for CAPTURE_COMPARE_4 register
    sc_signal<bool> PN_NAME(wb_write_ctrl_reg);   // Used for CONTROL register
    sc_signal<bool> PN_NAME(wb_write_status_reg); // Used for STATUS register
    sc_signal<bool> PN_NAME(wb_write_int_enable); // Used for INTERRUPT_ENABLE register
    sc_signal<sc_uint<32>> PN_NAME(wb_write_data);

    // Dummy registers for STM32 compatibility (store values but don't use functionally)
    sc_signal<sc_uint<32>> PN_NAME(dummy_cr2);   // STM32_CR2 dummy storage
    sc_signal<sc_uint<32>> PN_NAME(dummy_smcr);  // STM32_SMCR dummy storage
    sc_signal<sc_uint<32>> PN_NAME(dummy_ccmr1); // STM32_CCMR1 dummy storage
    sc_signal<sc_uint<32>> PN_NAME(dummy_ccmr2); // STM32_CCMR2 dummy storage
    sc_signal<sc_uint<32>> PN_NAME(dummy_ccer);  // STM32_CCER dummy storage
    sc_signal<sc_uint<32>> PN_NAME(dummy_dcr);   // STM32_DCR dummy storage
    sc_signal<sc_uint<32>> PN_NAME(dummy_dmar);  // STM32_DMAR dummy storage

    sc_signal<sc_uint<16>> PN_NAME(prescaler_counter);
};

#endif // __HW_TIMER_H__