
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

#include "pn_interconnect_backend_debug.h"

#include <numeric>

namespace {

size_t get_num_debug_masters(
    const std::vector<pn_module_if*>& modules)
{
    return std::accumulate(
        modules.begin(), modules.end(),
        size_t(0), //
        [](size_t sum, const pn_module_if* module) {
            return sum + module->debug_masters.size();
        });
}

size_t get_num_debug_slaves(
    const std::vector<pn_module_if*>& modules)
{
    return std::accumulate(
        modules.begin(), modules.end(),
        size_t(0), //
        [](size_t sum, const pn_module_if* module) {
            return sum + module->debug_slaves.size();
        });
}

} // namespace

void m_pn_interconnect_backend_debug::pn_trace(sc_trace_file* tf, int level)
{
    PN_TRACE(tf, master_haltrequest);
    PN_TRACE(tf, master_haltrequest_ack);
}

m_pn_interconnect_backend_debug::m_pn_interconnect_backend_debug(
    sc_module_name name,
    std::vector<pn_module_if*>& modules)
    : sc_module{name}
    , num_masters_debug{get_num_debug_masters(modules)}
    , num_slaves_debug{get_num_debug_slaves(modules)}
{
    // Currently max 1 slave and master are allowed.
    PN_ASSERT(num_masters_debug <= 1);
    PN_ASSERT(num_slaves_debug <= 1);

    for(auto& module : modules)
    {
        for(auto& debug_master : module->debug_masters)
        {
            debug_master->haltrequest_o(master_haltrequest);
            debug_master->haltrequest_ack_i(master_haltrequest_ack);
        }

        for(auto& debug_slave : module->debug_slaves)
        {
            debug_slave->haltrequest_i(slave_haltrequest);
            debug_slave->haltrequest_ack_o(slave_haltrequest_ack);
        }
    }

    SC_METHOD(proc_cmb);
    sensitive << master_haltrequest << slave_haltrequest_ack;
}

void m_pn_interconnect_backend_debug::proc_cmb()
{
    if(num_masters_debug == 0)
    {
        slave_haltrequest = 0;
    }
    else if(num_slaves_debug == 0)
    {
        master_haltrequest_ack = 0;
    }
    else
    {
        master_haltrequest_ack = slave_haltrequest_ack.read();
        slave_haltrequest = master_haltrequest.read();
    }
}
