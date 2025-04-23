# PicoNut Manual

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

| Version | Date       | Description                                                                                                 |
| ------- | ---------- | ----------------------------------------------------------------------------------------------------------- |
| 0.0.1   | 2024-04-03 | Initial version (how to document, welcome page)                                                             |
| 1.0.0   | 2025-04-23 | Initial GitHub Release (Minimal Nucleus, Soft-MemU, Wishbone-MemU, Soft-Debugger, Soft-Uart, Wishbone-Uart) |

% Add files that should be added ot the document here

```{toctree}
:maxdepth: 2
:caption: "PicoNut Manual"

introduction.md
getting-started.md
piconut/interface_def.md
piconut/nuclei.md
piconut/memus.md
piconut/debugging.md
piconut/software_library.md
piconut/peripherals.md

```

```{toctree}
:maxdepth: 2
:caption: "For Contributors"


contributor_resources/formatting.md
contributor_resources/how_to_document.md
contributor_resources/build_system.md
contributor_resources/embedded_ram.md



```

```{toctree}
:maxdepth: 2
:caption: "Appendix"


appendix/appendix.md



```
