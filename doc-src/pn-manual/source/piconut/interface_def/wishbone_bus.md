(sec:idef:wishbone)=

# Wishbone Systembus

**Author: Claus Janicher 2025**

This is a wishbone template module, which allows a simple copy paste or module extension for your slave systembus participant. The template has three dummy registers and you can use an address offset to access them. The byte select functionality is also implemented.

## Overview

The general wishbone bus signal overview is displayed in the image below.

```{figure} ./figures/wishbone/Wishbone_Interface.png
:name: wishbone_interface
:width: 75%
:align: center
Diagramm of the Wishbone bus interface.
```

```{doxygenfunction} SC_MODULE(m_wishbone_t)
:project: wishbone_template
```

This define is for the testbench and as a template, this MUST be changed when used for a custom design. The define should then be at /hw/piconut/config.mk .

```c++
#define CFG_WB_SLAVE_ADDRESS 0xF0000000
```

## Wishbone Read

A simple wishbone read cycle is displayed in the image below.

```{figure} ./figures/wishbone/waveform_wishbone_simple_read.svg
:name: wishbone_read
:width: 75%
:align: center
Diagramm of a simple read access via the Wishbone bus protocol.
```

## Wishbone Write

A simple wishbone write cycle is displayed in the image below.

```{figure} ./figures/wishbone/waveform_wishbone_simple_write.svg
:name: wishbone_write
:width: 75%
:align: center
Diagramm of a simple write access via the Wishbone bus protocol.
```

## Wishbone Byte Select

The bus supports the byte select signal which is four bits wide. You can precisely select every eight bit word in the register for writing and reading.

Examples in a binary and hexadecimal representation:

- 0b1111 = 0xF The complete four words will be used for your transaction.
- 0x1100 = 0xC The two upper words will be used.
- 0x0010 = 0x2 The second lowest word will be used.
- 0x0001 = 0x1 The lowest word will be used.

[Wishbone system overview](https://en.wikipedia.org/wiki/Wishbone_%28computer_bus%29)

[Wishbone interface documentation](https://wishbone-interconnect.readthedocs.io/en/latest/03_classic.html)

<!-- Image reference wishbone systembus overview: https://en.wikipedia.org/wiki/Wishbone_%28computer_bus%29 (last accessed 29.01.2025) -->

<!-- Single read and write reference + how it works: https://wishbone-interconnect.readthedocs.io/en/latest/03_classic.html -->
