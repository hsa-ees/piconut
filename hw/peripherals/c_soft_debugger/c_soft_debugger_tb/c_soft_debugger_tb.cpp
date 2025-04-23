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

#include "c_soft_debugger.h"

#include "reg_cast.h"

#include <base.h>
#include <piconut-config.h>

#include <cstdint>
#include <functional>
#include <memory>

/////////////// Helpers ///////////////
std::unique_ptr<c_soft_debugger> make_default_soft_debugger()
{
    auto callback_signal_debug_haltrequest = [](bool value) {};

    return std::make_unique<c_soft_debugger>(
        callback_signal_debug_haltrequest,
        CFG_DEBUG_OPENOCD_PORT);
}

/////////////// Tests ///////////////
void test_init_successfull();
void test_get_info_is_dm_get_info();

int sc_main(int argc, char** argv)
{

    PN_INFO("c_soft_debugger_tb: Start unit test ...");

    test_init_successfull();
    test_get_info_is_dm_get_info();

    PN_INFO("c_soft_debugger_tb: All tests passed");

    return 0;
}

void test_init_successfull()
{
    PN_INFO("c_soft_debugger_tb: run test test_init_successfull() ...");

    auto soft_debugger = make_default_soft_debugger();
    PN_ASSERT(soft_debugger.get() != nullptr);

    PN_INFO("c_soft_debugger_tb: test passed");
}

void test_get_info_is_dm_get_info()
{
    PN_INFO("c_soft_debugger_tb: run test test_get_info_is_dm_get_info() ...");

    auto soft_debugger = make_default_soft_debugger();
    c_soft_dm dummy_dm{
        [](bool) {}};

    PN_ASSERT(soft_debugger->get_info() == dummy_dm.get_info());

    PN_INFO("c_soft_debugger_tb: test passed");
}
