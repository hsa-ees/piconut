(sec:idef:ipdp)=

# IPort/DPort Interface

**Author: Lorenz Sommer 2024**

## Data Port (DPort)

The data port is an interface between the nucleus and the memory unit. It utilizes a simple bus protocol to transfer data between them.

## Instruction Port

The instruction port, similarly to the data port, is an interface between the nucleus and the memory unit. It is used to transfer instructions read from memory to the CPU. In contrast to the data port, the instruction port is a one-way interface. Instructions only need to be read by the CPU.

## Data port and instruction port signals

The data port/instruction port interface has the following input/output signals.

| Name     | Driven by | Optional   | Width                              | Description  |
| -------- | --------- | ---------- | ---------------------------------- | ------------ |
| stb      | Nucleus   | -          | 1                                  | Strobe       |
| we       | Nucleus   | DPort only | 1                                  | Write enable |
| bsel[n]  | Nucleus   | -          | 1 bit per byte of the data signals | Byte Select  |
| adr[n]   | Nucleus   | -          | 32/64/128                          | Address      |
| wdata[n] | Nucleus   | DPort only | Any multiple of 32                 | Write data   |
| ack      | MemU      | -          | 1                                  | Acknowledge  |
| rdata[n] | MemU      | -          | Any multiple of 32                 | Read data    |

### Strobe (stb)

The strobe signal begins a bus transaction. In single mode it is to be held high for no longer than a single clock cycle.

### Write enable (we)

The write enable signal is exclusive to the data port interface. It signals that the nucleus is writing data to
the memory unit. Similarly to the strobe signal, it is to be held high for no longer than a single clock cycle
synchronous to the strobe signal at the beginning of a transaction cycle. It must be set by the nucleus to
either 0 or 1. A signal vlaue of `we=1` and begins a write transaction, `we=0` begins a read transaction.

### Byte select (bsel)

The byte select signal is used to select the range of valid bytes that is to be read from or written to the target address.
It can have a maximum width of 16.

| bsel Width | wdata/rdata width |
| ---------- | ----------------- |
| 4          | 32                |
| 8          | 64                |
| 16         | 128               |

For example, with a width of 4, a bsel value of `0001` would select the 8 lowest bits (one byte) of the target address data content.

For the PicoNut processor in its first iteration, a width of 4 is sufficient.

### Address (adr)

The address signal holds the value of the address which is to be accessed. The signal must be stable
and contain a valid value at the time of the corresponding `ack` pulse.

### Write data (wdata)

The write data signal contains the actual data to be written by the nucleus to an address managed by the
memory unit. This signal, similarly to the write enable signal, is exclusive to the DPort interface.
The signal must be stable and contain a valid value at the time of the corresponding `ack` pulse.

### Acknowledge (ack)

The acknowledge signal, set by the memory unit, signals to the nucleus that the previous bus
transaction was successful. The acknowledge signal stalls the bus until it is set
high (a response occurs).

### Read data (rdata)

The read data signal is driven by the memory unit and holds the data is has read from the nucleus during a bus
transaction. The data is valid when ack=1 and thus valid for one clock cycle. The width of this signal
(rdata) and the write data signal (wdata) must be identical. The signal must be stable and hold a valid value
at the time of the corresponding `ack` pulse.

## Modes of operation

### Single mode

In default operation mode, the strobe (stb) signal may only stay high for a single clock cycle. The
corresponding ack-pulse response sent by the memory unit stalls the bus until it is set. There is no
constraint on how many clock cycles may pass before the memory unit sets the acknowledge signal high.

### Overlap mode

The protocol also supports overlap mode. In overlap mode, one additional transaction may be initiated, before
the previous one is complete (terminated by receiving a pulse of the `ack` signal). This means that at most
two transactions may occupy the bus at any given time and their response windows may overlap. Following that,
if two transactions have been initiated and not completed, at least one pulse of the `ack` signal has
to be awaited, before a new transaction may be initiated.

This modification of the protocol allows for a doubling of bus-throughput, given the memory unit responds
within one clock cycle. In single mode, even when the memory unit responds quickly, a new transaction may
only be initiated once the previous one is complete, leaving a gap of one clock cycle where no data
is transmitted. In overlap mode two transactions may overlap and thus the aforementioned gap can be
used to transmit valid data.

## Timing diagrams

### Single mode

```{figure} figures/iportdport/dport_iport_read.svg
:scale: 120
:alt: Alt text
:align: center

DPort/IPort read cycle
```

```{figure} figures/iportdport/dport_write.svg
:scale: 120
:alt: Alt text
:align: center

DPort write cycle
```

### Overlap mode

```{figure} figures/iportdport/dport_iport_read_overlap.svg
:scale: 120
:alt: Alt text
:align: center

DPort/Iport write cycle with overlap enabled
```

```{figure} figures/iportdport/dport_write_overlap.svg
:scale: 120
:alt: Alt text
:align: center

DPort read cycle with overlap enabled
```

```{figure} figures/iportdport/ipdp_overlap_optimal.svg
:scale: 120
:alt: Alt text
:align: center

Optimal bus usage in overlap mode
```
