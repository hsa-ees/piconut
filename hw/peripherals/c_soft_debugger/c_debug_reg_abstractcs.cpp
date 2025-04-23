/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This file contains the definition of the c_debug_reg_abstractcs class.

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

#include "c_debug_reg_abstractcs.h"

#include "reg_cast.h"

namespace {

c_debug_reg_abstractcs::reg_t make_default_reg_abstractcs(
    const size_t data_size,
    const size_t progbuf_size)
{
    c_debug_reg_abstractcs::reg_t reg = {0};
    reg.datacount = data_size;
    reg.progbufsize = progbuf_size;

    return reg;
}

} // namespace

c_debug_reg_abstractcs::c_debug_reg_abstractcs(
    const size_t data_size,
    const size_t progbuf_size)
    : reg{make_default_reg_abstractcs(data_size, progbuf_size)}
{
}

c_debug_reg_abstractcs::~c_debug_reg_abstractcs()
{
}

void c_debug_reg_abstractcs::write(uint32_t /*data*/)
{
    // W1C access -> Write 1 to clear (Write try by openocd)
    reg.cmderr = e_cmderr::NONE;

    // Rest of this register is ready-only
}

uint32_t c_debug_reg_abstractcs::read() const
{
    return reg_to_int(reg);
}

void c_debug_reg_abstractcs::set_datacount(uint32_t datacount)
{
    reg.datacount = datacount;
}

void c_debug_reg_abstractcs::set_cmderr(e_cmderr cmderr)
{
    reg.cmderr = cmderr;
}

void c_debug_reg_abstractcs::set_busy(e_busy busy)
{
    reg.busy = busy;
}

void c_debug_reg_abstractcs::set_progbufsize(uint32_t progbufsize)
{
    reg.progbufsize = progbufsize;
}

uint32_t c_debug_reg_abstractcs::datacount()
{
    return reg.datacount;
}

c_debug_reg_abstractcs::e_cmderr c_debug_reg_abstractcs::cmderr()
{
    return reg.cmderr;
}

c_debug_reg_abstractcs::e_busy c_debug_reg_abstractcs::busy()
{
    return reg.busy;
}

uint32_t c_debug_reg_abstractcs::progbufsize()
{
    return reg.progbufsize;
}