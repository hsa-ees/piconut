/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C)  2023-2024 Lukas Bauer <lukas.bauer@hs-ausgsburg.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    This file contains the configuration options for the PicoNut.

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

#ifndef _CONFIG_
#define _CONFIG_

////////////////// NUCLEUS /////////////////////////////////////////////////////
/**
 * @brief Number of registers in the regfile.
 * Defines the number of registers in the regfile.
 * This is relevant for the RV32E-Set.
 */
#define CFG_REGFILE_SIZE {CFG_REGFILE_SIZE}U
/**
 *
 * @brief Config parameter for the blockram of the hw_memu
 *
 */
#define CFG_MEMU_BRAM_SIZE {CFG_MEMU_BRAM_SIZE}U

/**
 * @brief Sets the start adress of the processor
 */
#define CFG_START_ADDRESS {CFG_START_ADDRESS}U

////////////////// CSR /////////////////////////////////////////////////////////
/**
 * @brief Width of the address in the csr bus.
 */
#define CFG_CSR_BUS_ADR_WIDTH {CFG_CSR_BUS_ADR_WIDTH}U

/**
 * @brief Width of the data in the csr bus.
 * Note: Also the width of the csr registers.the processor
 */
#define CFG_CSR_BUS_DATA_WIDTH {CFG_CSR_BUS_DATA_WIDTH}U

////////////////// DEBUG ///////////////////////////////////////////////////////
/**
 * @brief Sets the base address of the DEBUGGER module in address space
 * This is the base address where the module is located and can be accessed.
 * Note: This valu is fixed according to the `External Debug Support` standard
 * and should not be changed.
 */
#define CFG_DEBUG_DEBUGGER_BASE_ADDRESS {CFG_DEBUG_DEBUGGER_BASE_ADDRESS}U

/**
 * @brief Start address of the debug handler.
 */
#define CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS {CFG_DEBUG_DEBUG_HANDLER_START_ADDRESS}U

/**
 * @brief Port to which openocd has to connect.
 * Note: The config for openocd is found at tools/etc/openocd-*.cfg
 */
#define CFG_DEBUG_OPENOCD_PORT {CFG_DEBUG_OPENOCD_PORT}U

/**
 * @brief Width of the address of the DMI-bus.
 * Note: The DMI-bus is betweem the Debug Transport Module (DTM) and
 * one or multiple Debug Modules (DM).
 */
#define CFG_DEBUG_DMI_BUS_ADR_WIDTH {CFG_DEBUG_DMI_BUS_ADR_WIDTH}U

////////////////// PERIPHERALS /////////////////////////////////////////////////
/**
 * @brief Sets the base address of the WB_UART module in address space
 * This is the base address where the module is located and can be accessed.
 */
#define CFG_WB_UART_BASE_ADDRESS {CFG_WB_UART_BASE_ADDRESS}U

/**
 * @brief Disables the UARTs FIFO modules
 * This option disables the rx and tx FIFOs of the UART module. Then there is only
 * space for one byte in the rx and tx data registers
 */
#define CFG_WB_UART_DISABLE_FIFO {CFG_WB_UART_DISABLE_FIFO}U

#endif
