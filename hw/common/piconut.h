/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
                     Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    Master include file for PicoNut hardware modules in SystemC.

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

#pragma once

// Include all system-specific and/or installation-specific configurations ...
#include <piconut-config.h>

// Include all basic definitions
#include "pn_base.h"

#include "pn_module.h"

/** @brief Declare an SC_MODULE pre-synthesised
 *
 * If this macro is added to the SC_MODULE declaration (typically in the beginning
 * of the 'protected:' section), ICSC will not synthesize the module. Instead,
 * The user must provide a Verilog (.v) or VHDL (.vhdl) file for synthesis.
 */
#define PN_PRESYNTHESIZED std::string __SC_TOOL_VERILOG_MOD__ = ""

/** @brief Preprocessor condition for reducing an SC_MODULE declarations to its interface only.
 *
 * If a module is pre-synthesized, the build system sometimes only includes the main header
 * file of the module, but does not compile any C/C++ file of the module. In this case, the
 * module declaration must contain all ports, but does not need to provide any functionality.
 *
 * Methods, variables and internal signals can then be hidden by a `#if !PN_PRESYNTHESIZED_H_ONLY(mod)`
 * preprocessor condition, where `mod` is the name of the current module.
 */
#define PN_PRESYNTHESIZED_H_ONLY(mod) (defined(__SYNTHESIS__) && !defined(PN_TOPLEVEL_IS_##mod))
