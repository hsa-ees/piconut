/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2026 Hermann Zoha <hermann.zoha@tha.de>
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
#pragma once


#ifndef __SDRAM_CONTROLLER_DEFS_H__
#define __SDRAM_CONTROLLER_DEFS_H__

/**
* @file sdram_controller_defs.h
* @brief SDRAM-Controller module definitions.
*
* @defgroup sdram_controller_defs SDRAM-Controller Definitions
* @{
*/

/** 
 * @brief SDRAM Mode Register Op-Code configuration.
 *
 * Binary Value: 0000000110001 (13-bit / 0x0031)
 * * Bit Breakdown (Standard SDRAM Mode Register Mapping):
 * - A12 - A10 : 000 - Reserved
 * - A9        : 0   - Burst Read & Burst Write (0 = Programmed Burst, 1 = Single Write)
 * - A8 - A7   : 00  - Operating Mode (00 = Standard Operation)
 * - A6 - A4   : 011 - CAS Latency (011 = CL3)
 * - A3        : 0   - Burst Type (0 = Sequential, 1 = Interleave)
 * - A2 - A0   : 001 - Burst Length (001 = 2 Words)
 *
 * @note This configuration sets the SDRAM to Standard Operation, 
 * CAS Latency 3, Sequential Burst Type, and a Burst Length of 2.
 */
#define SDRAM_CONTROLLER_OP_CODE_INIT 0x0031  // Binary: 0000000110001

/**
 * @brief SDRAM Precharge Command Configuration.
 *
 * 
 * Setting A10 to '1' ensures that all internal memory banks are 
 * deactivated and precharged simultaneously, preparing them for the next Active command.
 */
#define SDRAM_CONTROLLER_PRECHARGE 0x0400  // Binary: 0010000000000

/**
 * @brief Set the Base address of the SDRAM_CONTROLLER module in address space.
 * This is the base adress where the module is located and con be accessed.
 */
#ifndef PN_CFG_SDRAM_CONTROLLER_BASE_ADDRESS
#define PN_CFG_SDRAM_CONTROLLER_BASE_ADDRESS 0x40000000U

/**
 * @brief Widht of the SDRAM_CONTROLLER modul 32M (33554432 address)
 */
#define PN_CFG_SDRAM_CONTROLLER_SIZE 0x2000000U
#endif
/** @} */ 
#endif
