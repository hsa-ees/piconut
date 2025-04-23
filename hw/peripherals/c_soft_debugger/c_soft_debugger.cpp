/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <Johannes.Hofmann1@tha.de>
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

#include "c_soft_debugger.h"

#include <base.h>

c_soft_debugger::c_soft_debugger(
    std::function<void(bool)> callback_signal_debug_haltrequest,
    uint16_t openocd_port)
{
    dm = std::make_unique<c_soft_dm>(
        callback_signal_debug_haltrequest);

    auto callback_dmi_write = [this](uint8_t address, uint32_t data) {
        dm->dmi_write(address, data);
    };

    auto callback_dmi_read = [this](uint8_t address) -> uint32_t {
        return dm->dmi_read(address);
    };

    auto callback_dmi_reset = [this]() {
        dm->dmi_reset();
    };

    dtm = std::make_unique<c_dtm>(
        callback_dmi_write,
        callback_dmi_read,
        callback_dmi_reset);

    auto jtag_set_input_pins = [this](bool tck, bool tms, bool tdi) {
        dtm->jtag_set_input_pins({tck, tms, tdi});
    };

    auto jtag_get_output_pin = [this]() -> bool {
        return dtm->jtag_get_output_pin();
    };

    auto jtag_trigger_reset = [this]() {
        dtm->jtag_reset();
    };

    remote_bitbang = std::make_unique<c_remote_bitbang>(
        jtag_set_input_pins,
        jtag_get_output_pin,
        jtag_trigger_reset,
        openocd_port);

    PN_ASSERT(dm != nullptr);
    PN_ASSERT(dtm != nullptr);
    PN_ASSERT(remote_bitbang != nullptr);
}

void c_soft_debugger::set_signal_debug_haltrequest_ack(bool value)
{
    dm->set_signal_debug_haltrequest_ack(value);
}

const char* c_soft_debugger::get_info()
{
    return dm->get_info();
}

bool c_soft_debugger::is_addressed(uint64_t adr)
{
    return dm->is_addressed(adr);
}

uint32_t c_soft_debugger::read32(uint64_t adr)
{
    return dm->read32(adr);
}
void c_soft_debugger::write32(uint64_t adr, uint32_t data)
{
    dm->write32(adr, data);
}

void c_soft_debugger::tick()
{
    remote_bitbang->process();
}