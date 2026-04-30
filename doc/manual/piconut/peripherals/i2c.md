# I²C 
**Authors: Johannes Fleiner, Sebastion Ebenhöh, Eikya Lagisetti 2025 - 2026**
## Introduction
The Inter-Integrated Circuit (I²C) bus is a serial communication bus used to transfer data between multiple connected bus participants. Communication is performed via two serial lines, referred to as the Serial Data Line (SDA) and the Serial Clock Line (SCL). Each bus participant is assigned a unique address and operates either as a receiver or a transmitter.

Within the PicoNut project, the I²C module is implemented as an independent peripheral module and is connected to the PicoNut processor via the Wishbone bus. Software access to the I²C module is exclusively performed through the provided address-based register interface.

## Specifications
### Concept & Architecture
The module features a strict encapsulation of functionalities, divided into specialized subcomponents: a Wishbone Slave Interface, a central Finite State Machine (Controller) for protocol handling, and separate units for Data (SDA) and Clock (SCL) control. 
The system routes the CPU instructions via the Wishbone bus to the I2C modul with a unidirectional signal interface. This modul is enclosed by a VHDL wrapper that converts the internal split signals into physical bidirectional Tri-State levels (SDA, SCL) to ensure correct synthesis and electrical behavior.

```{figure} ./figures/i2c/modul_global.svg
:name: modul global
:width: 100%
:align: center
Overview of the I2C Module in PicoNut Project.
```
### Technical Specifications
Operating Mode:
- Operates exclusively in Single-Master mode (Multi-Master is not supported).

- Supports both Master-Transmitter and Master-Receiver functionalities.

Bus Timing:
- Designed for Standard-Mode (SM) operation.

- Supports data rates of up to 100 kHz.

Addressing: Supports the 7-Bit addressing standard.

### Protocol Features & Error Handling
Signaling: Capable of generating Start and Stop conditions.

ACK/NACK: Performs automatic generation and verification of ACK/NACK bits after every byte transfer.

Flow Control: Supports Clock Stretching, allowing the Master to pause transmission if a Slave holds the SCL line LOW.

Error Detection: Includes detection of Acknowledge Failures (AF) when an addressed slave fails to respond.

## I²C-Registers
The register layout and functionality are strongly oriented towards the STM32 I²C peripheral design.
[STM32F4 datasheet](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf).

```{doxygengroup} i2c_defs
:project: m_i2c
```
## Submodules
The I2C module consists of the following components:

```{figure} ./figures/i2c/i2c_hardware.svg
:name: i2c_hardware
:width: 100%
:align: center
Overview of the I2C Hardware modules.
```

```{doxygenfunction} SC_MODULE(m_i2c)
:project: m_i2c
```

(sec:i2c:i2c:wishbone)=
### Wishbone
```{doxygenfunction} SC_MODULE(m_i2c_wishbone)
:project: m_i2c
```

(sec:i2c:i2c:controller)=
### Controller
```{doxygenfunction} SC_MODULE(m_i2c_controller)
:project: m_i2c
```

(sec:i2c:i2c:data_control)=
### Data Control
```{doxygenfunction} SC_MODULE(m_i2c_data_control)
:project: m_i2c
```

(sec:i2c:i2c:clk_control)=
### Clock Control
```{doxygenfunction} SC_MODULE(m_i2c_clk_control)
:project: m_i2c
```
## Driver
```{doxygengroup} i2c_driver
:project: m_i2c
```
## Hardware setup of the temperature demonstrator (WS2025/26)
The hardware can be wired manually using a breadboard:
- Slave_1: SSD1306 (OLED)  
- Slave_2: BME280 or BMP280

```{figure} ./figures/i2c/i2c_schematic.svg
:name: i2c_schematic
:width: 100%
:align: center
Overview of the I2C Hardware setup.
```
