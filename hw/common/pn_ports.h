/* OBSOLETE: [2025-08-12] This file appears to be unused and will be removed soon. */


/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
                2024 Lukas Bauer <lukas.bauer1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains some interface structs for the PicoNut

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
#ifndef PN_PORTS_H
#define PN_PORTS_H
#include "systemc.h"
#include <piconut.h>

// IPort Struct for PicoNut
struct IPort_Core {
    sc_uint<1> PN_NAME(stb);
    sc_uint<4> PN_NAME(bsel);
    sc_uint<30> PN_NAME(adr);

    // overloaded compare operator
    bool operator==(const IPort_Core &rhs) const
    {
        return stb == rhs.stb && bsel == rhs.bsel && adr == rhs.adr;
    }

    IPort_Core& operator = (const IPort_Core& rhs) {
        stb = rhs.stb;
        return *this;
    }

    // overloaded output operator
    friend ostream &operator<<(ostream &o, const IPort_Core &t)
    {
        o << "{ stb: " << t.stb
          << ", bsel: " << t.bsel
          << ", adr: " << t.adr << " }";
        return o;
    }

    // modified tracing function
    friend void sc_trace(sc_trace_file *tf, const IPort_Core &t, const std::string &name)
    {
        sc_trace(tf, t.stb, name + ".stb");
        sc_trace(tf, t.bsel, name + ".bsel");
        sc_trace(tf, t.adr, name + ".adr");
    }
};

struct DPort_Core {
    sc_uint<1> PN_NAME(stb);
    sc_uint<4> PN_NAME(bsel);
    sc_uint<30> PN_NAME(adr);
    sc_uint<1> PN_NAME(we);
    sc_uint<32> PN_NAME(wdata);

    // overloaded compare operator
    bool operator==(const DPort_Core &rhs) const
    {
        return DPort_Core::operator==(rhs) && we == rhs.we && wdata == rhs.wdata && stb == rhs.stb && bsel == rhs.bsel && adr == rhs.adr;
    }

    // overloaded output operator
    friend ostream &operator<<(ostream &o, const DPort_Core &t)
    {
        o << "{ stb: " << t.stb
          << ", bsel: " << t.bsel
          << ", adr: " << t.adr
          << ", we: " << t.we
          << ", wdata: " << t.wdata << " }";
        return o;
    }

    // modified tracing function
    friend void sc_trace(sc_trace_file *tf, const DPort_Core &t, const std::string &name)
    {
        sc_trace(tf, t.stb, name + ".stb");
        sc_trace(tf, t.bsel, name + ".bsel");
        sc_trace(tf, t.adr, name + ".adr");
        sc_trace(tf, t.we, name + ".we");
        sc_trace(tf, t.wdata, name + ".wdata");
    }
};

struct IPort_MemU {
    sc_uint<1> PN_NAME(ack);

    // overloaded compare operator
    bool operator==(const IPort_MemU &rhs) const
    {
        return ack == rhs.ack;
    }

    // overloaded output operator
    friend ostream &operator<<(ostream &o, const IPort_MemU &t)
    {
        o << "{ ack: " << t.ack << " }";
        return o;
    }

    // modified tracing function
    friend void sc_trace(sc_trace_file *tf, const IPort_MemU &t, const std::string &name)
    {
        sc_trace(tf, t.ack, name + ".ack");
    }
};

    struct DPort_MemU {
    sc_uint<1> PN_NAME(ack);
    sc_uint<32> PN_NAME(rdata);

    // overloaded compare operator
    bool operator==(const DPort_MemU &rhs) const
    {
        return DPort_MemU::operator==(rhs) && ack == rhs.ack && rdata == rhs.rdata;
    }

    // overloaded output operator
    friend ostream &operator<<(ostream &o, const DPort_MemU &t)
    {
        o << "{ ack: " << t.ack
          << ", rdata: " << t.rdata << " }";
        return o;
    }

    // modified tracing function
    friend void sc_trace(sc_trace_file *tf, const DPort_MemU &t, const std::string &name)
    {
        sc_trace(tf, t.ack, name + ".ack");
        sc_trace(tf, t.rdata, name + ".rdata");
    }
};

struct WishboneMasterInterface {
    sc_uint<32> PN_NAME(adr);    // Address signal
    sc_uint<32> PN_NAME(dat_o);  // Output data signal
    sc_uint<4> PN_NAME(sel);     // Byte select signals
    bool we;            // Write enable signal
    bool stb;           // Strobe signal, indicates valid bus cycle
    bool cyc;           // Cycle valid signal

    // Operator for comparisons, necessary for signal updates
    bool operator==(const WishboneMasterInterface &rhs) const
    {
        return adr == rhs.adr && dat_o == rhs.dat_o && sel == rhs.sel &&
               we == rhs.we && stb == rhs.stb && cyc == rhs.cyc;
    }

    // Operator for output, simplifies debugging
    friend ostream &operator<<(ostream &o, const WishboneMasterInterface &t)
    {
        o << "{ adr: " << t.adr
          << ", dat_o: " << t.dat_o
          << ", sel: " << t.sel
          << ", we: " << t.we
          << ", stb: " << t.stb
          << ", cyc: " << t.cyc
          << " }";
        return o;
    }

    // Function for tracing, allows recording of the struct in a SystemC trace file
    friend void sc_trace(sc_trace_file *tf, const WishboneMasterInterface &t, const std::string &name)
    {
        sc_trace(tf, t.adr, name + ".adr");
        sc_trace(tf, t.dat_o, name + ".dat_o");
        sc_trace(tf, t.sel, name + ".sel");
        sc_trace(tf, t.we, name + ".we");
        sc_trace(tf, t.stb, name + ".stb");
        sc_trace(tf, t.cyc, name + ".cyc");
    }
};

struct WishboneSlaveInterface {
    sc_uint<32> PN_NAME(dat_i);  // Input data signal
    sc_uint<4> PN_NAME(sel);     // Byte select signals
    bool ack;           // Acknowledge signal
    bool err;           // Error signal
    bool rty;           // Retry signal

    // Operator for comparisons, necessary for signal updates
    bool operator==(const WishboneSlaveInterface &rhs) const
    {
        return dat_i == rhs.dat_i && sel == rhs.sel && ack == rhs.ack &&
               err == rhs.err && rty == rhs.rty;
    }

    // Operator for output, simplifies debugging
    friend ostream &operator<<(ostream &o, const WishboneSlaveInterface &t)
    {
        o << "{ dat_i: " << t.dat_i
          << ", sel: " << t.sel
          << ", ack: " << t.ack
          << ", err: " << t.err
          << ", rty: " << t.rty
          << " }";
        return o;
    }

    // Function for tracing, allows recording of the struct in a SystemC trace file
    friend void sc_trace(sc_trace_file *tf, const WishboneSlaveInterface &t, const std::string &name)
    {
        sc_trace(tf, t.dat_i, name + ".dat_i");
        sc_trace(tf, t.sel, name + ".sel");
        sc_trace(tf, t.ack, name + ".ack");
        sc_trace(tf, t.err, name + ".err");
        sc_trace(tf, t.rty, name + ".rty");
    }
};

#endif // PN_PORTS_H
