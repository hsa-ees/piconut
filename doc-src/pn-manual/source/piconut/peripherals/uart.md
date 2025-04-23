
# UART
**Author: Lukas Bauer 2024**

The UART module allows the PicoNut processor to communicate with the outside world. It is a simple module that can be used to send and receive data between the PicoNut and a user's computer. The module is based on the [SiFive UART (SiFive FE310-G000 Manual)](https://sifive.cdn.prismic.io/sifive/4faf3e34-4a42-4c2f-be9e-c77baa4928c7_fe310-g000-manual-v3p2.pdf), which has been implemented in the PicoNut project.

There are two implementations of the UART module in the PicoNut project. The first is the Wishbone UART, a UART module connected to the Wishbone bus, primarily intended for use on an FPGA. The second implementation is the SimUART, a UART module that can be connected to the `CSoftPeripheral` interface of the PicoNut processor, designed for use in the simulator.

## UART Registers

A list of the registers for the UART module can be found in the [documentation (SiFive FE310-G000 Manual)](https://sifive.cdn.prismic.io/sifive/4faf3e34-4a42-4c2f-be9e-c77baa4928c7_fe310-g000-manual-v3p2.pdf) of the SiFive UART module.

## Wishbone UART

The following figure provides an overview of the components of the Wishbone UART module.
All subsequent diagrams omit the `clk` and `rst` signals for clarity.

```{figure} ./figures/uart/wb_uart_overview.drawio.svg
:name: wb_uart_overview
:width: 100%
:align: center
Overview of the Wishbone UART module.
```

```{doxygenfunction} SC_MODULE(m_wb_uart)
:project: wb_uart
```

The Wishbone UART module consists of the following components:

- **[Baudgen](sec:uart:wb_uart:baudgen)**: Multiple Baudgenerators that divide the system clock to generate the baudrate for the UART module.
- **[Majority Filter](sec:uart:wb_uart:majr_filter)**: The majority filter is used to filter the received data. To ensure that the bit is received correctly.
- **[FIFOs](sec:uart:wb_uart:fifo)**: The FIFOs are used to buffer the data that is sent and received by the UART module.
- **[uart_rx](sec:uart:wb_uart:rx)**: The receiver receives data from the outside world.
- **[uart_tx](sec:uart:wb_uart:tx)**: The transmitter sends data to the outside world.

(sec:uart:wb_uart:baudgen)=

### Baudgen

```{figure} ./figures/uart/wb_uart_baudgen.drawio.svg
:name: wb_uart_baudgen
:width: 75%
:align: center
Diagramm of the Baudrate Generator.
```

```{doxygenfunction} SC_MODULE(m_baudgen)
:project: wb_uart
```

(sec:uart:wb_uart:majr_filter)=

### Majority Filter

```{figure} ./figures/uart/wb_uart_majr_filter.drawio.svg
:name: wb_uart_majority_filter
:width: 75%
:align: center
Diagramm of the Majority Filter.
```

```{doxygenfunction} SC_MODULE(m_majority_filter)
:project: wb_uart
```

(sec:uart:wb_uart:fifo)=

### FIFOs

```{figure} ./figures/uart/wb_uart_fifo.drawio.svg
:name: wb_uart_fifo
:width: 75%
:align: center
Diagramm of the FIFOs.
```

```{doxygenfunction} SC_MODULE(m_uart_fifo)
:project: wb_uart
```

(sec:uart:wb_uart:rx)=

### uart_rx

```{figure} ./figures/uart/wb_uart_rx.drawio.svg
:name: wb_uart_rx
:width: 75%
:align: center
Diagramm of the Receiver.
```

```{doxygenfunction} SC_MODULE(m_uart_rx)
:project: wb_uart
```

(sec:uart:wb_uart:tx)=

### uart_tx

```{figure} ./figures/uart/wb_uart_tx.drawio.svg
:name: wb_uart_tx
:width: 75%
:align: center
Diagramm of the Transmitter.
```

```{doxygenfunction} SC_MODULE(m_uart_tx)
:project: wb_uart
```

### System for hardware test

To test the Wishbone UART module on hardware, a system has been created.
This system contains a test module that can configure the UART module and echo
the received data back. The system is synthesizable for the ULX3S board.

It contains two SystemC modules:

- **[uart_wb_uart](sec:uart_wb_uart:system:uart_test)**: Contains the test hardware for the UART module.
- **[top](sec:uart_wb_uart:system:top)**: Contains a wrapper for synthesis

(sec:uart_wb_uart:system:uart_test)=

#### uart_test

```{doxygenfunction} SC_MODULE(m_uart_test)
:project: wb_uart_system
```

(sec:uart_wb_uart:system:top)=

#### top

```{doxygenfunction} SC_MODULE(m_top)
:project: wb_uart_system
```

## SimUART

The SimUART module implements a UART module that can be connected to the `CSoftPeripheral` interface of the PicoNut processor, which is described in Chapter {ref}`sec:idef:csoft`.

The following figure provides an overview of the components of the SimUART module.

```{figure} ./figures/uart/CSoftUart.drawio.svg
:name: sim_uart_overview
:width: 100%
:align: center
Overview of the SimUART module.
```

```{doxygengroup} c_soft_uart
:project: softuart
```

### CSoftPeripheral Interface

The following methods are those implemented from the `CSoftPeripheral` interface.
To enable the UART module to be connected to the `CSoftPeripheral` interface, the following methods must be implemented:

```{doxygenfunction} c_soft_uart
:project: softuart
```

```{doxygenfunction} get_info
:project: softuart
```

```{doxygenfunction} read8
:project: softuart
```

```{doxygenfunction} write8
:project: softuart
```

```{doxygenfunction} write32
:project: softuart
```

```{doxygenfunction} read32
:project: softuart
```

```{doxygenfunction} is_addressed
:project: softuart
```

### UART Emulation

The following methods are implemented to emulate the functionality of the SiFive UART module.
They handle data transmission, the FIFO, and interrupt generation.

```{doxygenfunction} handel_transmit
:project: softuart
```

```{doxygenfunction} update_txfull
:project: softuart
```

```{doxygenfunction} handel_interrupt
:project: softuart
```

```{doxygenfunction} update_uart
:project: softuart
```
