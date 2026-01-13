/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Johannes Hofmann <johannes.hofmann1@tha.de>
                     2025 Alexander Beck <alexander.beck1@tha.de>
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

#include "csr.h"
#include "typedef.h"

// Write masks
static const sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> dcsr_write_mask = 0b00000000000000001011111000010111;

void m_csr::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, clk);
    PN_TRACE(tf, reset);

    // Csr bus
    PN_TRACE(tf, csr_bus_read_en_in);
    PN_TRACE(tf, csr_bus_write_en_in);
    PN_TRACE(tf, csr_bus_adr_in);
    PN_TRACE(tf, csr_bus_wdata_in);
    PN_TRACE(tf, csr_bus_rdata_out);

    // Status signals
    // Program counter
    PN_TRACE(tf, pc_in);

    // Debug
    PN_TRACE(tf, debug_level_enter_ebreak_in);
    PN_TRACE(tf, debug_level_enter_haltrequest_in);
    PN_TRACE(tf, debug_level_enter_step_in);
    PN_TRACE(tf, debug_level_leave_in);

    // Control signals
    // Program counter
    PN_TRACE(tf, dpc_out);

    // Debug
    PN_TRACE(tf, debug_level_enter_out);
    PN_TRACE(tf, debug_step_out);

    // Registers
    PN_TRACE(tf, mstatus_reg);
    PN_TRACE(tf, misa_reg);
    PN_TRACE(tf, dcsr_reg);
    PN_TRACE(tf, dpc_reg);
    PN_TRACE(tf, dscratch0_reg);
    PN_TRACE(tf, dscratch1_reg);

    // Internal registers
    PN_TRACE(tf, debug_level_reg);

    // Internal status signals
    PN_TRACE(tf, s_debug_level_enter);
}

void m_csr::proc_clk_bus_read()
{
    bool read_en_in_var;
    sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH> adr_in_var;
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> data_out_var;

    while(true)
    {
        wait();

        read_en_in_var = csr_bus_read_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_out_var = 0;

        if(read_en_in_var == 1)
        {
            switch(adr_in_var)
            {
                case e_csr_address::csr_adr_mstatus:
                    data_out_var = mstatus_reg.read();
                    break;
                case e_csr_address::csr_adr_misa:
                    data_out_var = misa_reg.read();
                    break;
                case e_csr_address::csr_adr_dcsr:
                    data_out_var = dcsr_reg.read();
                    break;
                case e_csr_address::csr_adr_dpc:
                    data_out_var = dpc_reg.read();
                    break;
                case e_csr_address::csr_adr_dscratch0:
                    data_out_var = dscratch0_reg.read();
                    break;
                case e_csr_address::csr_adr_dscratch1:
                    data_out_var = dscratch1_reg.read();
                    break;
                case e_csr_address::csr_adr_mie:
                    data_out_var = mie_reg.read();
                    break;
                case e_csr_address::csr_adr_mtvec:
                    data_out_var = mtvec_reg.read();
                    break;
                case e_csr_address::csr_adr_mepc:
                    data_out_var = mepc_reg.read();
                    break;
                case e_csr_address::csr_adr_mcause:
                    data_out_var = mcause_reg.read();
                    break;
                case e_csr_address::csr_adr_mtval:
                    data_out_var = mtval_reg.read();
                    break;
                case e_csr_address::csr_adr_mip:
                    data_out_var = mip_reg.read();
                    break;
                default:
                    data_out_var = 0x0;
                    // PN_ERROR("CSR: Invalid csr read address!");
                    break;
            }
        }

        csr_bus_rdata_out = data_out_var;
    }
}

void m_csr::proc_clk_mstatus()
{
    bool write_en_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;
    bool mret_in_var = 0;

    // Reset
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> mstatus_reg_var = 0;
    mstatus_reg_var |= ((sc_uint<32>)(0U) << (uint)e_mstatus_fieldpos::SIE);  // Disable interrupt enable s-mode
    mstatus_reg_var |= ((sc_uint<32>)(0U) << (uint)e_mstatus_fieldpos::MIE);  // Disable interrupt enable m-mode
    mstatus_reg_var |= ((sc_uint<32>)(1U) << (uint)e_mstatus_fieldpos::MPRV); // Machine mode active
    mstatus_reg = mstatus_reg_var;

    // Trap handling state management
    enum TrapState
    {
        TRAP_IDLE,        // Not in trap handler
        TRAP_ENTRY,       // Just entered trap handler
        TRAP_HANDLING,    // In trap handler
        TRAP_EXIT_PENDING // MRET executed, waiting to exit
    };

    TrapState trap_state = TRAP_IDLE;
    bool last_interrupt_signal = false;

    while(true)
    {
        wait();

        write_en_in_var = csr_bus_write_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_in_var = csr_bus_wdata_in.read();
        mret_in_var = mret_in.read();
        mstatus_reg_var = mstatus_reg.read();
        bool current_interrupt = interrupt_in.read();

        // Direct CSR write operations
        if(write_en_in_var == 1 && adr_in_var == e_csr_address::csr_adr_mstatus)
        {
            sc_uint<32> writable_mask = 0x0000088E;
            sc_uint<32> new_value;

            switch(write_mode_in.read())
            {
                case e_csr_write_mode::CSR_WRITE_ALL:
                    new_value = (mstatus_reg_var & static_cast<sc_uint<32>>(~writable_mask)) | (data_in_var & writable_mask);
                    break;
                case e_csr_write_mode::CSR_WRITE_SET:
                    new_value = mstatus_reg_var | (data_in_var & writable_mask);
                    break;
                case e_csr_write_mode::CSR_WRITE_CLEAR:
                    new_value = mstatus_reg_var & static_cast<sc_uint<32>>(~(data_in_var & writable_mask));
                    break;
                default:
                    new_value = mstatus_reg_var;
            }

            mstatus_reg_var = new_value;
        }
        // Trap entry handling: Save MIE to MPIE and clear MIE
        else if(current_interrupt && !last_interrupt_signal && trap_state == TRAP_IDLE)
        {
            // Save current MIE value to MPIE
            bool current_mie = (mstatus_reg_var >> (uint)e_mstatus_fieldpos::MIE) & 0x1;

            // Clear MIE bit
            mstatus_reg_var &= ~((sc_uint<32>)(1U) << (uint)e_mstatus_fieldpos::MIE);

            // Set MPIE to current MIE value
            if(current_mie)
            {
                // TBD(ns): Workaround for ICSC Issue https://github.com/intel/systemc-compiler/issues/85
                // maybe this gets fixed soon.
                mstatus_reg_var |= ((sc_uint<32>)(1U) << (uint)e_mstatus_fieldpos::MPIE);
            }
            else
            {
                mstatus_reg_var &= ~((sc_uint<32>)(1U) << (uint)e_mstatus_fieldpos::MPIE);
            }

            trap_state = TRAP_ENTRY;
        }
        // MRET handling: Restore MIE from MPIE, set MPIE to 1
        else if(mret_in_var && trap_state != TRAP_EXIT_PENDING)
        {
            // Get current MPIE value
            bool current_mpie = (mstatus_reg_var >> (uint)e_mstatus_fieldpos::MPIE) & 0x1;

            // Set MIE to MPIE value
            if(current_mpie)
            {
                mstatus_reg_var |= ((sc_uint<32>)(1U) << (uint)e_mstatus_fieldpos::MIE);
            }
            else
            {
                mstatus_reg_var &= ~((sc_uint<32>)(1U) << (uint)e_mstatus_fieldpos::MIE);
            }

            // Set MPIE to 1 (per RISC-V spec)
            mstatus_reg_var |= ((sc_uint<32>)(1U) << (uint)e_mstatus_fieldpos::MPIE);

            trap_state = TRAP_EXIT_PENDING;
        }

        // State transition logic
        if(trap_state == TRAP_ENTRY)
        {
            // After one cycle in ENTRY state, move to HANDLING
            trap_state = TRAP_HANDLING;
        }
        else if(trap_state == TRAP_EXIT_PENDING && !mret_in_var)
        {
            // After MRET completes, return to IDLE state
            trap_state = TRAP_IDLE;
        }

        // Update outputs every cycle based on the latest register value
        bool current_mie = (mstatus_reg_var >> (uint)e_mstatus_fieldpos::MIE) & 0x1;
        mstatus_mie_out.write(current_mie);

        // Remember interrupt state for next cycle to detect edges
        last_interrupt_signal = current_interrupt;

        mstatus_reg = mstatus_reg_var;
    }
}

void m_csr::proc_clk_misa()
{
    // Reset
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> misa_reg_var = 0;
    misa_reg_var |= ((uint)e_misa_mxl::BITS_32 << (PN_CFG_CSR_BUS_DATA_WIDTH - 2));

    misa_reg_var |= ((sc_uint<32>)(1U) << (uint)e_misa_extensions::A);
    misa_reg_var |= ((sc_uint<32>)(1U) << (uint)e_misa_extensions::I);
    // misa_reg_var |= ((sc_uint<32>)(1U) << (int)e_misa_extensions::M); maybe soon?

    // Add newly implemented extensions here to the misa csr

    misa_reg = misa_reg_var;

    while(true)
    {
        wait();

        // No write logic needed because this register is read only.
    }
}

void m_csr::proc_clk_dcsr()
{
    bool write_en_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> dcsr_reg_var = 0;
    dcsr_reg_var |= ((uint)e_dcsr_prv::MACHINE << (uint)e_dcsr_fieldpos::PRV);
    dcsr_reg_var |= ((sc_uint<32>)(0U) << (uint)e_dcsr_fieldpos::MPRVEN);
    dcsr_reg_var |= ((sc_uint<32>)(0U) << (uint)e_dcsr_fieldpos::STOPTIME);
    dcsr_reg_var |= ((sc_uint<32>)(0U) << (uint)e_dcsr_fieldpos::STOPCOUNT);
    dcsr_reg_var |= ((sc_uint<32>)(1U) << (uint)e_dcsr_fieldpos::EBREAKM);
    dcsr_reg_var |= ((sc_uint<32>)(1U) << (uint)e_dcsr_fieldpos::EBREAKS);
    dcsr_reg_var |= ((sc_uint<32>)(1U) << (uint)e_dcsr_fieldpos::EBREAKU);
    dcsr_reg_var |= ((uint)e_dcsr_xdebugver::AS_DESCRIBED << (uint)e_dcsr_fieldpos::XDEBUGVER);
    dcsr_reg = dcsr_reg_var;

    while(true)
    {
        wait();

        write_en_in_var = csr_bus_write_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_in_var = csr_bus_wdata_in.read();
        dcsr_reg_var = dcsr_reg.read();

        if(write_en_in_var == 1 &&
            adr_in_var == e_csr_address::csr_adr_dcsr)
        {
            dcsr_reg_var &= static_cast<sc_uint<32>>(~dcsr_write_mask);
            dcsr_reg_var |= data_in_var & dcsr_write_mask;
        }

        // Cause
        if(debug_level_enter_ebreak_in.read() == 1)
        {
            dcsr_reg_var.range(8, 6) = (uint)e_dcsr_cause::EBREAK;
        }
        else if(debug_level_enter_haltrequest_in.read() == 1)
        {
            dcsr_reg_var.range(8, 6) = (uint)e_dcsr_cause::HALTREQUEST;
        }
        else if(debug_level_enter_step_in.read() == 1)
        {
            dcsr_reg_var.range(8, 6) = (uint)e_dcsr_cause::SINGLESTEP;
        }

        dcsr_reg = dcsr_reg_var;
    }
}

void m_csr::proc_clk_dpc()
{
    bool write_en_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> dpc_reg_var = 0;
    dpc_reg = dpc_reg_var;

    while(true)
    {
        wait();

        write_en_in_var = csr_bus_write_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_in_var = csr_bus_wdata_in.read();
        dpc_reg_var = dpc_reg.read();

        if(write_en_in_var == 1 &&
            adr_in_var == e_csr_address::csr_adr_dpc)
        {
            dpc_reg_var = data_in_var;
        }

        if(debug_level_reg.read() == 0)
        {
            // Save pc in dpc when debug mode is entered.

            if(debug_level_enter_ebreak_in.read() == 1)
            {
                dpc_reg_var = pc_in.read();
            }
            // Note: Add trigger cause here when trigger is implemented.
            else if(debug_level_enter_haltrequest_in.read() == 1)
            {
                dpc_reg_var = pc_in.read();
            }
            else if(debug_level_enter_step_in.read() == 1)
            {
                dpc_reg_var = pc_in.read();
            }
            else
            {
                // PN_ASSERT(false);
            }
        }

        dpc_reg = dpc_reg_var;
    }
}

void m_csr::proc_clk_dscratch0()
{
    bool write_en_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> dscratch0_reg_var = 0;
    dscratch0_reg = dscratch0_reg_var;

    while(true)
    {
        wait();

        write_en_in_var = csr_bus_write_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_in_var = csr_bus_wdata_in.read();
        dscratch0_reg_var = dscratch0_reg.read();

        if(write_en_in_var == 1 &&
            adr_in_var == e_csr_address::csr_adr_dscratch0)
        {
            dscratch0_reg_var = data_in_var;
        }

        dscratch0_reg = dscratch0_reg_var;
    }
}

void m_csr::proc_clk_dscratch1()
{
    bool write_en_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> dscratch1_reg_var = 0;
    dscratch1_reg = dscratch1_reg_var;

    while(true)
    {
        wait();

        write_en_in_var = csr_bus_write_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_in_var = csr_bus_wdata_in.read();
        dscratch1_reg_var = dscratch1_reg.read();

        if(write_en_in_var == 1 &&
            adr_in_var == e_csr_address::csr_adr_dscratch1)
        {
            dscratch1_reg_var = data_in_var;
        }

        dscratch1_reg = dscratch1_reg_var;
    }
}

void m_csr::proc_cmb_dpc()
{
    dpc_out = dpc_reg.read();
}

void m_csr::proc_cmb_dcsr()
{
    debug_step_out =
        dcsr_reg.read()[(int)e_dcsr_fieldpos::STEP] &
        !debug_level_reg.read();
}

void m_csr::proc_clk_debug_level()
{
    bool debug_level_reg_var = 0;

    // Reset
    debug_level_reg = 0;

    while(true)
    {
        wait();

        debug_level_reg_var = debug_level_reg.read();

        if(s_debug_level_enter.read() == 1)
        {
            debug_level_reg_var = 1;
        }

        if(debug_level_leave_in.read() == 1)
        {
            debug_level_reg_var = 0;
        }

        debug_level_reg = debug_level_reg_var;
    }
}

void m_csr::proc_cmb_debug_level_enter()
{
    debug_level_enter_out = s_debug_level_enter.read();
}

void m_csr::proc_cmb_debug_mode_enter()
{
    s_debug_level_enter = debug_level_enter_ebreak_in.read() ||
                          debug_level_enter_haltrequest_in.read() ||
                          debug_level_enter_step_in.read();
}

// Interrupt
void m_csr::proc_clk_mie()
{
    bool write_en_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset - all interrupts disabled initially
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> mie_reg_var = 0;
    mie_reg = mie_reg_var;

    while(true)
    {
        wait();

        write_en_in_var = csr_bus_write_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_in_var = csr_bus_wdata_in.read();
        mie_reg_var = mie_reg.read();

        if(write_en_in_var == 1 &&
            adr_in_var == e_csr_address::csr_adr_mie)
        {
            // Remove debug tracking code that uses old_mie, old_mtie, etc.
            mie_reg_var = data_in_var;
        }

        // CRITICAL FIX: Make sure interrupts are preserved during and after MRET
        // The MIE register should not be affected by trap entry/exit directly
        // It is the MSTATUS.MIE bit that gets manipulated during traps

        mie_reg = mie_reg_var;
        // Update all three output signals
        mie_msie_out.write((mie_reg_var >> 3) & 0x1);  // MSIE - bit 3
        mie_mtie_out.write((mie_reg_var >> 7) & 0x1);  // MTIE - bit 7
        mie_meie_out.write((mie_reg_var >> 11) & 0x1); // MEIE - bit 11
    }
}

void m_csr::proc_clk_mtvec()
{
    bool write_en_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset - default trap handler address mit Direct Mode (0)
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> mtvec_reg_var = 0;
    mtvec_reg = mtvec_reg_var;

    while(true)
    {
        wait();

        write_en_in_var = csr_bus_write_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_in_var = csr_bus_wdata_in.read();
        mtvec_reg_var = mtvec_reg.read();

        if(write_en_in_var == 1 &&
            adr_in_var == e_csr_address::csr_adr_mtvec)
        {
            // Only bits 31:2 for base address and bits 1:0 for mode
            // Other values are ignored
            mtvec_reg_var = data_in_var & 0xFFFFFFFF;
        }

        mtvec_reg = mtvec_reg_var;
        // printf("MTVEC Register: 0x%08x\n", (unsigned)mtvec_reg.read());
    }
}

void m_csr::proc_cmb_mtvec_address()
{
    sc_uint<32> mtvec_reg_var = mtvec_reg.read();
    sc_uint<2> mode = mtvec_reg_var & 0x3;            // Mode bits [1:0]
    sc_uint<32> base_address = mtvec_reg_var & ~0x3U; // Base address [31:2]

    if(mode == 1 && interrupt_in.read())
    {
        // Vectored mode: base + 4*cause
        sc_uint<32> cause = mcause_reg.read() & 0x1F; // Cause bits [4:0]
        sc_uint<32> trap_address = base_address + (cause << 2);

        mtvec_trap_address_out.write(trap_address);
    }
    else
    {
        // Direct mode or exception: just use the base address

        mtvec_trap_address_out.write(base_address);
    }
}

void m_csr::proc_clk_mepc()
{
    bool write_en_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> mepc_reg_var = 0;
    mepc_reg = mepc_reg_var;

    while(true)
    {
        wait();

        write_en_in_var = csr_bus_write_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_in_var = csr_bus_wdata_in.read();
        mepc_reg_var = mepc_reg.read();

        if(write_en_in_var == 1 &&
            adr_in_var == e_csr_address::csr_adr_mepc)
        {
            mepc_reg_var = data_in_var;
        }
        else if(interrupt_in.read() == 1)
        {

            mepc_reg_var = pc_in.read();
        }

        mepc_reg = mepc_reg_var;
        mepc_out.write(mepc_reg_var); // Update the output port
    }
}

void m_csr::proc_clk_mcause()
{
    bool write_en_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> mcause_reg_var = 0;
    mcause_reg = mcause_reg_var;

    while(true)
    {
        wait();

        write_en_in_var = csr_bus_write_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_in_var = csr_bus_wdata_in.read();
        mcause_reg_var = mcause_reg.read();

        if(write_en_in_var == 1 && adr_in_var == e_csr_address::csr_adr_mcause)
        {
            mcause_reg_var = data_in_var;
        }

        mcause_reg = mcause_reg_var;
    }
}

void m_csr::proc_clk_mtval()
{
    bool write_en_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> mtval_reg_var = 0;
    mtval_reg = mtval_reg_var;

    while(true)
    {
        wait();

        write_en_in_var = csr_bus_write_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_in_var = csr_bus_wdata_in.read();
        mtval_reg_var = mtval_reg.read();

        if(write_en_in_var == 1 &&
            adr_in_var == e_csr_address::csr_adr_mtval)
        {
            mtval_reg_var = data_in_var;
        }

        mtval_reg = mtval_reg_var;
    }
}

void m_csr::proc_clk_mip()
{
    bool write_en_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;
    sc_uint<2> write_mode_in_var = 0;

    // Reset - Anfänglich keine ausstehenden Interrupts
    sc_uint<PN_CFG_CSR_BUS_DATA_WIDTH> mip_reg_var = 0;
    mip_reg = mip_reg_var;

    while(true)
    {
        wait();

        write_en_in_var = csr_bus_write_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_in_var = csr_bus_wdata_in.read();
        write_mode_in_var = write_mode_in.read();
        mip_reg_var = mip_reg.read();

        // Update MIP register based on external interrupt sources
        if(msip_in.read())
        {
            mip_reg_var |= ((sc_uint<32>)(1U) << 3); // MSIP (Machine Software Interrupt Pending)
        }
        else
        {
            mip_reg_var &= ~((sc_uint<32>)(1U) << 3); // Clear MSIP
        }

        if(mtip_in.read())
        {
            mip_reg_var |= ((sc_uint<32>)(1U) << 7); // MTIP (Machine Timer Interrupt Pending)
        }
        else
        {
            mip_reg_var &= ~((sc_uint<32>)(1U) << 7); // Clear MTIP
        }

        if(meip_in.read())
        {
            mip_reg_var |= ((sc_uint<32>)(1U) << 11); // MEIP (Machine External Interrupt Pending)
        }
        else
        {
            mip_reg_var &= ~((sc_uint<32>)(1U) << 11); // Clear MEIP
        }

        if(write_en_in_var == 1 && adr_in_var == e_csr_address::csr_adr_mip)
        {
            // MIP write mask: Only MSIP (bit 3) is software writable per RISC-V spec
            sc_uint<32> msip_mask = 0x00000008; // Bit 3 (MSIP)

            switch(write_mode_in_var)
            {
                case e_csr_write_mode::CSR_WRITE_ALL:
                    mip_reg_var = (mip_reg_var & static_cast<sc_uint<32>>(~msip_mask)) | (data_in_var & msip_mask);
                    break;
                case e_csr_write_mode::CSR_WRITE_SET:
                    mip_reg_var |= (data_in_var & msip_mask);
                    break;
                case e_csr_write_mode::CSR_WRITE_CLEAR:
                    mip_reg_var &= static_cast<sc_uint<32>>(~(data_in_var & msip_mask));
                    break;
                default:
                    break;
            }
        }

        mip_msip_out.write((mip_reg_var >> 3) & 0x1);  // MSIP (Software interrupt)
        mip_mtip_out.write((mip_reg_var >> 7) & 0x1);  // MTIP (Timer interrupt)
        mip_meip_out.write((mip_reg_var >> 11) & 0x1); // MEIP (External interrupt)

        mip_reg = mip_reg_var;
    }
}

void m_csr::proc_cmb_update_interrupt_pending()
{
    // Check if each type of interrupt is pending and enabled
    bool msi_pending = (mip_msip_out.read() == 1) && (mie_msie_out.read() == 1);
    bool mti_pending = (mip_mtip_out.read() == 1) && (mie_mtie_out.read() == 1);
    bool mei_pending = (mip_meip_out.read() == 1) && (mie_meie_out.read() == 1);

    // Any interrupt is pending, and global interrupts are enabled in MSTATUS
    bool result = (msi_pending || mti_pending || mei_pending) && (mstatus_mie_out.read() == 1);

    // Set the final interrupt pending signal
    interrupt_pending_out.write(result);
}