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

#include "c_remote_bitbang.h"

#include "c_fake_client.h"

#include <base.h>

#define PORT 9824

void test_connection()
{
    PN_INFO("c_remote_bitbang_tb: run test test_connection() ...");

    c_remote_bitbang remote_bitbang{
        nullptr,
        nullptr,
        nullptr,
        PORT};

    c_fake_client fake_client{};

    fake_client.connect(PORT);
    remote_bitbang.process();
    PN_ASSERT(remote_bitbang.connected());

    fake_client.close();
    remote_bitbang.process();
    PN_ASSERT(!remote_bitbang.connected());

    PN_INFO("c_remote_bitbang_tb: test passed");
}

void test_jtag_input()
{
    PN_INFO("c_remote_bitbang_tb: run test test_jtag_input() ...");

    static const std::string message = "01234567";

    auto callback_jtag_set_inputs = [](bool tck, bool tms, bool tdi) {
        static constexpr bool expected[8][3] = {
            {false, false, false},
            {false, false, true},
            {false, true, false},
            {false, true, true},
            {true, false, false},
            {true, false, true},
            {true, true, false},
            {true, true, true},
        };
        static int index = 0;

        PN_ASSERT(index < 8);
        PN_ASSERT(tck == expected[index][0]);
        PN_ASSERT(tms == expected[index][1]);
        PN_ASSERT(tdi == expected[index][2]);

        ++index;
    };

    auto callback_jtag_get_outputs = []() -> bool {
        PN_ASSERT(false);
        return false;
    };

    auto callback_jtag_trigger_reset = []() {
        PN_ASSERT(false);
    };

    c_remote_bitbang remote_bitbang{
        callback_jtag_set_inputs,
        callback_jtag_get_outputs,
        callback_jtag_trigger_reset,
        PORT};

    c_fake_client fake_client{};

    fake_client.connect(PORT);
    remote_bitbang.process();
    PN_ASSERT(remote_bitbang.connected());

    fake_client.send(message);
    remote_bitbang.process();
    // Asserts happen in callback function

    fake_client.close();
    remote_bitbang.process();
    PN_ASSERT(!remote_bitbang.connected());

    PN_INFO("c_remote_bitbang_tb: test passed");
}

void test_jtag_output()
{
    PN_INFO("c_remote_bitbang_tb: run test test_jtag_output() ...");

    static const std::string message = "RR";

    auto callback_jtag_set_inputs = [](bool /*tck*/, bool /*tms*/, bool /*tdi*/) {
        PN_ASSERT(false);
    };

    auto callback_jtag_get_output = []() -> bool {
        static constexpr bool output[2] = {true, false};
        static int index = 0;

        PN_ASSERT(index < 2);
        return output[index++];
    };

    auto callback_jtag_trigger_reset = []() {
        PN_ASSERT(false);
    };

    c_remote_bitbang remote_bitbang{
        callback_jtag_set_inputs,
        callback_jtag_get_output,
        callback_jtag_trigger_reset,
        PORT};

    c_fake_client fake_client{};

    fake_client.connect(PORT);
    remote_bitbang.process();
    PN_ASSERT(remote_bitbang.connected());

    fake_client.send(message);
    remote_bitbang.process();

    const auto maybe_recieved = fake_client.recieve();
    PN_ASSERT(maybe_recieved.has_value());
    PN_ASSERT(maybe_recieved.value().size() == 2);
    PN_ASSERT(maybe_recieved.value()[0] == '1');
    PN_ASSERT(maybe_recieved.value()[1] == '0');

    fake_client.close();
    remote_bitbang.process();
    PN_ASSERT(!remote_bitbang.connected());

    PN_INFO("c_remote_bitbang_tb: test passed");
}

void test_jtag_reset()
{
    PN_INFO("c_remote_bitbang_tb: run test test_jtag_reset() ...");

    static const std::string message = "r";

    bool reset_triggered = false;

    auto callback_jtag_set_inputs = [](bool /*tck*/, bool /*tms*/, bool /*tdi*/) {
        PN_ASSERT(false);
    };

    auto callback_jtag_get_outputs = []() -> bool {
        PN_ASSERT(false);
        return false;
    };

    auto callback_jtag_trigger_reset = [&reset_triggered]() {
        reset_triggered = true;
    };

    c_remote_bitbang remote_bitbang{
        callback_jtag_set_inputs,
        callback_jtag_get_outputs,
        callback_jtag_trigger_reset,
        PORT};

    c_fake_client fake_client{};

    fake_client.connect(PORT);
    remote_bitbang.process();
    PN_ASSERT(remote_bitbang.connected());

    fake_client.send(message);
    remote_bitbang.process();

    PN_ASSERT(reset_triggered);

    fake_client.close();
    remote_bitbang.process();
    PN_ASSERT(!remote_bitbang.connected());

    PN_INFO("c_remote_bitbang_tb: test passed");
}

int sc_main(int argc, char** argv)
{
    PN_INFO("c_remote_bitbang_tb: Start unit test ...");

    test_connection();
    test_jtag_input();
    test_jtag_output();
    test_jtag_reset();

    PN_INFO("c_remote_bitbang_tb: All tests passed");

    return 0;
}
