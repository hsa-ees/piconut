/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2025 Gundolf Kiefer <gundolf.kiefer@tha.de>
                2025 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description:
    Classes for managing the PicoNut (soft) hardware modules.

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

#ifndef __PN_MODULE_H__
#define __PN_MODULE_H__

#include <systemc.h>
#include <vector>

// ***** Doxygen Header *****

/// @file

/// @defgroup pn_module *PicoNut* Module
/// @brief Basic classes and declaration for *PicoNut* modules.
///

// *****************************************************************************
// *                                                                           *
// *         IPort, DPort                                                      *
// *                                                                           *
// *****************************************************************************

/// @name *IPort* and *DPort* Interface Declarations ...
///
/// @{

#define PN_IPORT_ALEN_MAX 64 ///< maximum number of address bits for an instruction port (*IPort*).
#define PN_IPORT_DLEN_MAX 64 ///< maximum number of data bits for an instruction port (*IPort*).

#define PN_DPORT_ALEN_MAX 64  ///< maximum number of address bits for a data port (*DPort*).
#define PN_DPORT_DLEN_MAX 512 ///< maximum number of data bits for a data port (*DPort*).

typedef sc_uint<PN_IPORT_ALEN_MAX> pn_iport_adr_t; ///< *IPort* address
typedef sc_uint<PN_IPORT_DLEN_MAX> pn_iport_dat_t; ///< *IPort* data
typedef sc_uint<PN_DPORT_ALEN_MAX> pn_dport_adr_t; ///< *DPort* address
typedef sc_uint<PN_DPORT_DLEN_MAX> pn_dport_dat_t; ///< *DPort* data

///@}

// *****************************************************************************
// *                                                                           *
// *         Wishbone                                                          *
// *                                                                           *
// *****************************************************************************

/// @name *Wishbone* Bus Interface Declarations ...
///
/// @{

// TBD: Discuss & validate the best way to represent wishbone interfaces.
//
//      * Variant A is based on individual port signals and a structure with references
//        to them. Macros can help to maintain short/readable code.
//
//      * Variant B is to define structures for all input and output signals, respectively.
//        ICSC supports signals of structures (to be tested).
//
//      * Variant C would be to define structures of signals. However, this is presently
//        unsupported by ICSC. -> JH 2025-10-24: seems to work
#define PN_WISHBONE_ALEN_MAX 64 ///< maximum number of address bits for a *Wishbone* bus.
#define PN_WISHBONE_DLEN_MAX 64 ///< maximum number of data bits for a *Wishbone* bus.

typedef sc_uint<PN_WISHBONE_ALEN_MAX> pn_wb_adr_t;     ///< *Wishbone* address
typedef sc_uint<PN_WISHBONE_DLEN_MAX> pn_wb_dat_t;     ///< *Wishbone* data
typedef sc_uint<PN_WISHBONE_DLEN_MAX / 8> pn_wb_sel_t; ///< *Wishbone* byte select

/// @brief *Wishbone* slave interface
///
struct pn_wishbone_slave_t
{
    int alen, dlen;
    sc_uint<PN_WISHBONE_ALEN_MAX> base_address; ///< base address of the interface
    sc_uint<PN_WISHBONE_ALEN_MAX> size;         ///< size of the interface

    sc_in<pn_wb_adr_t> PN_NAME(adr_i);
    sc_in<pn_wb_dat_t> PN_NAME(dat_i);
    sc_out<pn_wb_dat_t> PN_NAME(dat_o);

    sc_in<pn_wb_sel_t> PN_NAME(sel_i);
    sc_in<bool> PN_NAME(stb_i);
    sc_in<bool> PN_NAME(cyc_i);
    sc_in<bool> PN_NAME(we_i);
    sc_out<bool> PN_NAME(ack_o);
};

/// @brief *Wishbone* master interface
///
struct pn_wishbone_master_t
{
    int alen, dlen;

    sc_out<pn_wb_adr_t> PN_NAME(adr_o);
    sc_out<pn_wb_dat_t> PN_NAME(dat_o);
    sc_in<pn_wb_dat_t> PN_NAME(dat_i);

    sc_out<pn_wb_sel_t> PN_NAME(sel_o);
    sc_out<bool> PN_NAME(stb_o);
    sc_out<bool> PN_NAME(cyc_o);
    sc_out<bool> PN_NAME(we_o);
    sc_in<bool> PN_NAME(ack_i);
};

///@}

// *****************************************************************************
// *                                                                           *
// *         Soft                                                              *
// *                                                                           *
// *****************************************************************************

/// @name *Soft* Interface Declarations ...
///
/// @{

typedef uint64_t pn_soft_adr_t;
typedef uint64_t pn_soft_dat_t;

/// @brief *Soft* slave interface
///
struct pn_soft_slave_t
{
    pn_soft_adr_t base_address;
    pn_soft_adr_t size;

    std::function<pn_soft_dat_t(pn_soft_adr_t)> read;
    std::function<void(pn_soft_adr_t, pn_soft_dat_t)> write;
};

/// @brief *Soft* master interface
///
struct pn_soft_master_t
{
    std::function<pn_soft_dat_t(pn_soft_adr_t)> read;
    std::function<void(pn_soft_adr_t, pn_soft_dat_t)> write;
};

/// @}

// *****************************************************************************
// *                                                                           *
// *         PicoNut Module                                                    *
// *                                                                           *
// *****************************************************************************

/// @brief *PicoNut* Module Interface
///
/// The class covers all meta-information for managing hardware modules to be
/// integrated into a *PicoNut* system. It allows to specify various bus interfaces
/// (*Wishbone*, *Soft*, in the future *AXI*) and *PicoNut* core interfaces
/// (*IPort*, *DPort*).
///
/// Modules are expected to inherit from this class secondarily or to instantiate
/// an interface object. Typically, modules are primarily derived from *sc_module*,
/// but are not required to be a *SystemC* module.
///
class pn_module_if
{
public:
    /// @name Container holding interfaces definitions ...
    ///
    /// @{

    std::vector<pn_wishbone_slave_t*> wb_slaves{};
    std::vector<pn_wishbone_master_t*> wb_masters{};
    //~ std::vector<pn_soft_slave_t*> soft_slaves;
    //~ std::vector<pn_soft_master_t*> soft_masters;

    /// @}

    /// @name Constructor, Destructor ...
    /// @{

    //~ pn_module_if() {}

    //~ ~pn_module_if() {}

    /// @}

    /// @name Adding Interfaces ...
    ///
    /// **Note:** Interfaces can only be added during the elaboration phase.
    ///
    /// @{

    /// @brief Add wishbone slave interface.
    ///
    void pn_add_wishbone_slave(
        pn_wishbone_slave_t* wb_slave)
    {
        PN_ASSERT(!elaborated);
        wb_slaves.push_back(wb_slave);
    }

    /// @brief Add wishbone master interface.
    ///
    void pn_add_wishbone_master(
        pn_wishbone_master_t* wb_master)
    {
        PN_ASSERT(!elaborated);
        wb_masters.push_back(wb_master);
    }

    /// @brief Add soft slave interface.
    ///
    //~ void pn_add_soft_slave(
    //~     pn_soft_slave_t* soft_slave)
    //~ {
    //~     PN_ASSERT(!elaborated);
    //~     soft_slaves.push_back(soft_slave);
    //~ }

    /// @brief Add soft master interface.
    ///
    //~ void pn_add_soft_master(
    //~     pn_soft_master* soft_master)
    //~ {
    //~     PN_ASSERT(!elaborated);
    //~     soft_masters.push_back(soft_master);
    //~ }

    /// @}

    /// @name Helpers ...
    ///
    /// @{

    /// @brief Set elabted flag to true.
    ///
    void set_elaborated()
    {
        elaborated = true;
    }

    /// @brief Returns if module is already elaborated.
    ///
    /// @return True if module is elaborated else false.
    ///
    bool is_elaborated()
    {
        return elaborated;
    }

    /// @brief Returns if module is synthesizeable.
    ///
    /// @return True if module is synthesizeable else false.
    ///
    bool is_synthesizeable()
    {
        return true;
        //~ return soft_slaves.empty() &&
        //~        soft_masters.empty();
    }

    /// @}

private:
    bool elaborated = false;
};

#endif
