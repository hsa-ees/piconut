
/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This file contains the system bus module.

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

#include "pn_interconnect_backend.h"

namespace {

std::vector<pn_wb_adr_t> extract_base_addresses(
    const std::vector<pn_module_if*>& modules)
{
    std::vector<pn_wb_adr_t> _base_addresses;
    for(const auto& module : modules)
    {
        for(const auto& wb_slave : module->wb_slaves)
        {
            _base_addresses.push_back(wb_slave->base_address);
        }
    }
    return _base_addresses;
}

std::vector<pn_wb_adr_t> extract_sizes(
    const std::vector<pn_module_if*>& modules)
{
    std::vector<pn_wb_adr_t> _sizes;
    for(const auto& module : modules)
    {
        for(const auto& wb_slave : module->wb_slaves)
        {
            _sizes.push_back(wb_slave->size);
        }
    }
    return _sizes;
}

size_t get_num_wb_slaves(
    std::vector<pn_module_if*>& modules)
{
    size_t num = 0;
    for(size_t i = 0; i < modules.size(); i++)
    {
        num += modules[i]->wb_slaves.size();
    }
    return num;
}

} // namespace

void m_pn_interconnect_backend::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, wb_stb);
    PN_TRACE(tf, wb_cyc);
    PN_TRACE(tf, wb_we);
    PN_TRACE(tf, wb_sel);
    PN_TRACE_BUS(tf, wb_ack, num_slaves_wishbone);
    PN_TRACE(tf, wb_adr);
    PN_TRACE(tf, wb_dat_o);
    PN_TRACE_BUS(tf, wb_dat_i, num_slaves_wishbone);
    PN_TRACE_BUS(tf, wb_rty, num_slaves_wishbone);
    PN_TRACE_BUS(tf, wb_err, num_slaves_wishbone);
    PN_TRACE(tf, wb_ack_master);
    PN_TRACE(tf, wb_dat_i_master);
    PN_TRACE(tf, wb_rty_master);
    PN_TRACE(tf, wb_err_master);
}

m_pn_interconnect_backend::m_pn_interconnect_backend(
    sc_module_name name,
    std::vector<pn_module_if*>& modules)
    : sc_module{name}
    , num_slaves_wishbone{get_num_wb_slaves(modules)}
    , base_addresses_wishbone{extract_base_addresses(modules)}
    , sizes_wishbone{extract_sizes(modules)}
    //
    , wb_ack("wb_ack", num_slaves_wishbone)
    , wb_dat_i("wb_dat_i", num_slaves_wishbone)
    , wb_rty("wb_rty", num_slaves_wishbone)
    , wb_err("wb_err", num_slaves_wishbone)
{
    PN_ASSERT(num_slaves_wishbone == base_addresses_wishbone.size());
    PN_ASSERT(num_slaves_wishbone == sizes_wishbone.size());

    size_t signal_index = 0;
    for(auto& module : modules)
    {
        for(auto& wb_master : module->wb_masters)
        {
            wb_master->adr_o(wb_adr);
            wb_master->dat_i(wb_dat_i_master);
            wb_master->dat_o(wb_dat_o);
            wb_master->sel_o(wb_sel);
            wb_master->stb_o(wb_stb);
            wb_master->cyc_o(wb_cyc);
            wb_master->we_o(wb_we);
            wb_master->ack_i(wb_ack_master);
            wb_master->rty_i(wb_rty_master);
            wb_master->err_i(wb_err_master);
        }

        for(auto& wb_slave : module->wb_slaves)
        {
            wb_slave->adr_i(wb_adr);
            wb_slave->dat_i(wb_dat_o);
            wb_slave->dat_o(wb_dat_i[signal_index]);
            wb_slave->sel_i(wb_sel);
            wb_slave->stb_i(wb_stb);
            wb_slave->cyc_i(wb_cyc);
            wb_slave->we_i(wb_we);
            wb_slave->ack_o(wb_ack[signal_index]);
            wb_slave->rty_o(wb_rty[signal_index]);
            wb_slave->err_o(wb_err[signal_index]);

            signal_index++;
        }
    }

    SC_METHOD(proc_cmb);
    sensitive << wb_adr;
    for(size_t i = 0; i < num_slaves_wishbone; i++)
    {
        sensitive << wb_ack[i]
                  << wb_dat_i[i]
                  << wb_rty[i]
                  << wb_err[i];
    }
}

void m_pn_interconnect_backend::proc_cmb()
{
    wb_ack_master = 0;
    wb_dat_i_master = 0;
    wb_rty_master = 0;
    wb_err_master = 0;

    //
    // wishbone master <-> wishbone slave
    //
    for(size_t i = 0; i < num_slaves_wishbone; i++)
    {
        if(wb_adr.read() >= base_addresses_wishbone[i] &&
            wb_adr.read() < base_addresses_wishbone[i] + sizes_wishbone[i])
        {
            wb_ack_master = wb_ack[i].read();
            wb_dat_i_master = wb_dat_i[i].read();
            wb_rty_master = wb_rty[i].read();
            wb_err_master = wb_err[i].read();
        }
    }

    //
    // wishbone master <-> soft slave
    //
    // TBD(jh): Implement wishbone master <-> soft slave bus translation.

    // #ifndef __SYNTHESIS__

    //     // TBD: Remove this rising edge detection stuff.
    //     // Currently wishbone write/read is called multiple times uintendedly.
    //     static bool rising_edge = false;

    //     if(wb_stb.read() && wb_cyc.read())
    //     {
    //         if(rising_edge)
    //         {
    //             return;
    //         }
    //         rising_edge = true;

    //         for(auto slave_soft : slaves_soft)
    //         {
    //             uint64_t adr = wb_adr.read().to_uint64();
    //             if(slave_soft->is_addressed(adr))
    //             {
    //                 wb_ack_master = 1;
    //                 wb_err_master = 0;
    //                 wb_rty_master = 0;

    //                 // PN_INFO("timestamp");

    //                 if(wb_we.read() == 1)
    //                 {
    //                     slave_soft->write32(adr, wb_dat_i.read());
    //                 }
    //                 else
    //                 {
    //                     wb_dat_o_master = slave_soft->read32(adr);
    //                 }
    //             }
    //         }
    //     }
    //     else
    //     {
    //         rising_edge = false;
    //     }

    // #endif
}
