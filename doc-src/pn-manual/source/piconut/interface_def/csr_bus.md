# CSR Bus

**Author: Johannes Hofmann 2025**

Each module that uses CSRs (Timer, Interrupt Handler, etc) implements the used CSRs inside the module.
The implemented CSRs are connected to the PicoNut via the CSR bus. It is used to allow read/write
access to the registers. The bus has one master and multiple slaves. The master is typical the nucleus.

Currently there is no acknowledge signal for read access. At read access the CSR slave has to
set the register value at the next positive clock edge. An overview about the read and write access is shown in the figures below.

The bus signals are:
| Signal | Description |
|------------|-----------------------------------------------------|
| read_en | Read-enable: High when read access is going on. |
| write_en | Write-enable: High when write access is going on. |
| adr | Address: Address of the target register. |
| data_write | Data write: Data to write. |
| data_read | Data read: Read data. |

The address signal width is set in the config with `CFG_CSR_BUS_ADR_WIDTH`. The default value is 12.
The data read/write signal width is set in the config with `CFG_CSR_BUS_DATA_WIDTH`. The default value is 32.

```{figure} figures/csr-bus/CSR-bus.drawio.svg
:align: center
Overview CSR-bus
```

(fig:csr-bus:csr-bus_read)=

```{figure} figures/csr-bus/csr-bus_read.svg
:align: center
Overview CSR-bus read access
```

(fig:csr-bus:csr-bus_write)=

```{figure} figures/csr-bus/csr-bus_write.svg
:align: center
Overview CSR-bus write access
```
