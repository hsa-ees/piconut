/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2024 Marco Milenkovic <Marco.Milenkovic@hs-augsburg.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg

  Description: This file contains the implementation of the Software Interface Class for custom peripherals

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

#ifndef CSoftPeripheral_h
#define CSoftPeripheral_h
#include <stdint.h>

class c_soft_peripheral
{
public:
    uint64_t size;
    uint64_t base_address;

    virtual ~c_soft_peripheral() {}
    virtual const char *get_info();

    /*** Read access:
     *      Adress has to be aligned according the datawidth.
     *      derived class !!MUST!! implement at least one of these methods.
     *      the others can be implemented using one of the "read*_using_read*" methods.
     ***/

    virtual uint8_t read8(uint64_t adr) { return read8_using_read32(adr); }     // read from peripheral
    virtual uint16_t read16(uint64_t adr) { return read16_using_read8(adr); };  // read8 2 times
    virtual uint32_t read32(uint64_t adr) { return read32_using_read8(adr); };  // read8 4 times
    virtual uint64_t read64(uint64_t adr) { return read64_using_read32(adr); }; // read from peripheral

    /*** Write access:
     *     Adress has to be aligned according the datawidth.
     *     derived class !!MUST!! implement at least one of these methods.
     *     the others can be implemented using one of the "write*_using_write*" methods.
     ***/

    virtual void write8(uint64_t adr, uint8_t data) { write8_using_write32(adr, data); };    // write to peripheral
    virtual void write16(uint64_t adr, uint16_t data) { write16_using_write8(adr, data); };  // write8 2 times
    virtual void write32(uint64_t adr, uint32_t data) { write32_using_write8(adr, data); };  // write8 4 times
    virtual void write64(uint64_t adr, uint64_t data) { write64_using_write32(adr, data); }; // write32 2 times


    // Other functions

    /**
     * @brief checks if a peripheral is addressed
     *
     * @param adr address to search for
     * @return true if a peripheral is found
     * @return false if no peripheral is found
     */
    virtual bool is_addressed(uint64_t adr) = 0; // check if the adress is valid

    /**
     * @brief defines reset behavior for peripherals
     */
    virtual void on_reset() {}; // reset the peripheral (if possible)

    /**
     * @brief Performs all internal functions depending on the system clock
     *
     * This function is automatically called at the rising edge of every simulated clock cycle
     */
    virtual void on_rising_edge_clock() {}; // update the peripheral // -> on rising edge clock

protected:
    /**  Helper methods
     * These methods can be used to implement the read/write methods in the derived class
     * If there is need for a more efficient way to read/write data from/to the peripheral, the derived class can implement it directly
     * */
    uint8_t read8_using_read32(uint64_t adr);                // read from peripheral
    uint16_t read16_using_read8(uint64_t adr);               // read8 2 times
    uint32_t read32_using_read8(uint64_t adr);               // read8 4 times
    uint64_t read64_using_read32(uint64_t adr);              // read from peripheral
    void write8_using_write32(uint64_t adr, uint8_t data);   // write to peripheral
    void write16_using_write8(uint64_t adr, uint16_t data);  // write8 2 times
    void write32_using_write8(uint64_t adr, uint32_t data);  // write8 4 times
    void write64_using_write32(uint64_t adr, uint64_t data); // write to peripheral
};

#endif
