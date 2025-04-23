# Hardware-MemU

**Author: Claus Janicher 2025**

The hw_memu module is the main interface between the Nucleus and the Systembus. As Systembus this project uses Wishbone which is a well documented bus.

This module has a state-machine which performs the necessary tasks the processor is asking for. The program is in the current state stored in the built in Blockram module. This module is being created with the hw_memu and only accessible via the IPort and DPort interface from the processor.

```{figure} ./figures/hw_memu/memu_core_perfect.png
:name: hw_memu_currently
:width: 75%
:align: center
Diagramm of the current implemented Hardware Memory Unit.
```

```{doxygenfunction} SC_MODULE(m_hw_memu)
:project: hw_memu
```

## Integrated Blockram

```{doxygenfunction} SC_MODULE(m_blockram_memu)
:project: hw_memu
```

## Interfaces

The main interface connections from the Memory Unit to the Nucleus processor are the {ref}`sec:idef:ipdp` bus systems.
The other side of the memory unit is connected to the {ref}`sec:idef:wishbone`, on which the MemU communicates as a Master.

## config-parameters in hw/piconut/config.mk

The BRAM size defines the size the internal blockram module has, which is being used for simulation and hardware synthesis. The START_ADDRESS is being used as a address substraction method for the blockram, which doesn't use the offset because its addresses start at 0x00000000.

- CFG_START_ADDRESS = 0x10000000
- CFG_MEMU_BRAM_SIZE = 102399
