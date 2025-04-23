/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)      2024 Johannes Hofmann <johannes.hofmann1@tha.de>
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

// Write masks
static const sc_uint<CFG_CSR_BUS_DATA_WIDTH> dcsr_write_mask = 0b00000000000000001011111000010111;

void m_csr::Trace(sc_trace_file* tf, int level)
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
    sc_uint<CFG_CSR_BUS_ADR_WIDTH> adr_in_var;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> data_out_var;

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
    sc_uint<CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> mstatus_reg_var = 0;
    mstatus_reg_var |= (0U << (uint)e_mstatus_fieldpos::SIE);  // Disable interrupt enable s-mode
    mstatus_reg_var |= (0U << (uint)e_mstatus_fieldpos::MIE);  // Disable interrupt enable m-mode
    mstatus_reg_var |= (1U << (uint)e_mstatus_fieldpos::MPRV); // Machine mode active
    mstatus_reg = mstatus_reg_var;

    while(true)
    {
        wait();

        write_en_in_var = csr_bus_write_en_in.read();
        adr_in_var = csr_bus_adr_in.read();
        data_in_var = csr_bus_wdata_in.read();
        mstatus_reg_var = mstatus_reg.read();

        if(write_en_in_var == 1 &&
            adr_in_var == e_csr_address::csr_adr_mstatus)
        {
            mstatus_reg_var = data_in_var;
        }

        mstatus_reg = mstatus_reg_var;
    }
}

void m_csr::proc_clk_misa()
{
    // Reset
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> misa_reg_var = 0;
    misa_reg_var |= ((uint)e_misa_mxl::BITS_32 << (CFG_CSR_BUS_DATA_WIDTH - 2));

    // misa_reg_var |= (1U << (int)e_misa_extensions::A); maybe soon?
    misa_reg_var |= (1U << (uint)e_misa_extensions::I);
    // misa_reg_var |= (1U << (int)e_misa_extensions::M); maybe soon?

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
    sc_uint<CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> dcsr_reg_var = 0;
    dcsr_reg_var |= ((uint)e_dcsr_prv::MACHINE << (uint)e_dcsr_fieldpos::PRV);
    dcsr_reg_var |= (0U << (uint)e_dcsr_fieldpos::MPRVEN);
    dcsr_reg_var |= (0U << (uint)e_dcsr_fieldpos::STOPTIME);
    dcsr_reg_var |= (0U << (uint)e_dcsr_fieldpos::STOPCOUNT);
    dcsr_reg_var |= (1U << (uint)e_dcsr_fieldpos::EBREAKM);
    dcsr_reg_var |= (1U << (uint)e_dcsr_fieldpos::EBREAKS);
    dcsr_reg_var |= (1U << (uint)e_dcsr_fieldpos::EBREAKU);
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
            dcsr_reg_var &= ~dcsr_write_mask;
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
    sc_uint<CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> dpc_reg_var = 0;
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
    sc_uint<CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> dscratch0_reg_var = 0;
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
    sc_uint<CFG_CSR_BUS_ADR_WIDTH> adr_in_var = 0;
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> data_in_var = 0;

    // Reset
    sc_uint<CFG_CSR_BUS_DATA_WIDTH> dscratch1_reg_var = 0;
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
