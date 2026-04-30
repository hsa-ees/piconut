# Membrana Reference Version

**Author: Claus Janicher 2025, Daniel Sommerfeldt 2026**

The membrana_ref module is the main interface between the Nucleus and the Systembus. As Systembus this project uses Wishbone which is a well documented bus.

This module has a state-machine which performs the necessary tasks the processor is asking for. The program is in the current state stored in the built in Blockram module. This module is being created with the membrana_hw and only accessible via the IPort and DPort interface from the processor.

Functionally it works like the Membrana Hardware Variant, with the added possiblity of using Soft-Peripherals like the Membrana Software Variant. For this it is only necessary to set the correct flag (TECHS=sim).

```{figure} ./figures/membrana_hw/memu_core_perfect.png
:name: membrana_ref_currently
:width: 75%
:align: center
Diagram of the current implemented Hardware Membrana.
```

```{doxygenfunction} SC_MODULE(m_membrana_ref)
:project: membrana_ref
```

## Integrated Blockram

```{doxygenfunction} SC_MODULE(m_membrana_ref_emem)
:project: membrana_ref
```

## Interfaces

The main interface connections from the Membrana to the Nucleus processor are the {ref}`sec:idef:ipdp` bus systems.
The other side of the Membrana is connected to the {ref}`sec:idef:wishbone`, on which the membrana communicates as a Master.

### Soft Peripheral Interface

```{doxygenfunction} init_memory
:project: membrana_ref
```

```{doxygenfunction} load_elf
:project: membrana_ref
```

```{doxygenfunction} add_peripheral
:project: membrana_ref
```

```{doxygenfunction} find_peripheral
:project: membrana_ref
```

```{doxygenfunction} write_peripheral
:project: membrana_ref
```

```{doxygenfunction} read_peripheral
:project: membrana_ref
```

```{doxygenfunction} list_all_peripherals
:project: membrana_ref
```

```{doxygenfunction} get_num_peripherals
:project: membrana_ref
```

```{doxygenfunction} get_peripheral
:project: membrana_ref
```

```{doxygenfunction} on_rising_edge_clock_all_peripherals 
:project: membrana_ref
```

## config-parameters in hw/piconut/config.mk

The BRAM size defines the size the internal blockram module has, which is being used for simulation
and hardware synthesis. The START_ADDRESS is being used as a address subtraction method for the
blockram, which doesn't use the offset because its addresses start at 0x00000000.

- CFG_START_ADDRESS = 0x10000000
- CFG_MEMBRANA_BRAM_SIZE = 102399
