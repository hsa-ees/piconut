# PicoNut Manual

```{figure} figures/logo-piconut.png
:name: Piconut_Logo
:align: center
:width: 30%
```

## Licence

This manual is licensed under the [Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/).

```{figure} figures/cc-by.png
:name: licence_cc-by
:align: center
```
<br>

Please cite as: <br>
    &nbsp;&nbsp;&nbsp;&nbsp;PicoNut Documentation, <Author(s)>, https://ees.tha.de/, <br>
    &nbsp;&nbsp;&nbsp;&nbsp;CC-BY 4.0, https://creativecommons.org/licenses/by/4.0/ <br>

Replace <Author(s)> with the author(s) of the respective artwork or
chapter, or with "Editors Gundolf Kiefer, Michael Schäferling, Johannes Hofmann", if no author is given.

## Document History

| Version | Date       | Description                                                                                                                  |
| ------- | ---------- | ---------------------------------------------------------------------------------------------------------------------------- |
| 0.0.1   | 2024-04-03 | Initial version (how to document, welcome page)                                                                              |
| 1.0.0   | 2025-04-23 | Initial GitHub Release (Minimal Nucleus, Soft-MemU, Wishbone-MemU, Soft-Debugger, Soft-Uart, Wishbone-Uart)                  |
| 1.1.0   | 2026-01-13 | GitHub Release v1.1.0 (New build system, Debugger, Clint, Timer, Soft-Video (Qt-Gui), Soft-Audio, RISCOF-Tests, A-Extension) |
| 1.1.1   | 2026-04-30 | GitHub Release v1.1.1 (I2C, Video and GPIO peripheral; PlayNut; OrangeCrab board support)                                    |
| 1.1.2   | 2026-09-01 | Initial PicoNut AI release, SDRAM Controller, add FPU (F-Extension) to reference nucleus                                     |

% Add files that should be added ot the document here

```{toctree}
:maxdepth: 2
:caption: "PicoNut Manual"

introduction.md
getting-started.md
piconut/nuclei.md
piconut/membrana.md
piconut/peripherals.md
piconut/interface_def.md
piconut/software.md
piconut/debugging.md
piconut/interrupt.md
piconut/simulator.md
piconut/supported_boards.md
```

```{toctree}
:maxdepth: 1
:caption: "PicoNut AI"
piconut_ai/overview.md
piconut_ai/applications.md
```

```{toctree}
:maxdepth: 2
:caption: "For Contributors"


contributing/build_system.md
contributing/code_style.md
contributing/how_to_document.md
contributing/docker.md
contributing/riscof.md
contributing/embedded_ram.md
```

```{toctree}
:maxdepth: 2
:caption: "Appendix"


appendix/toolchain.md
appendix/udev.md



```
