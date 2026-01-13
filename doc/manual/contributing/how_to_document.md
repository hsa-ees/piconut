# How to Document
**Authors: Lukas Bauer 2024, Lorenz Sommer 2024**

## Introduction

This page will detail how to document development on the PicoNut project using the setup provided.

### Software dependencies

In order to build the documentation from the source directory, a few software packages have to be installed.

- python3
- python3-sphinx
- python3-myst-parser
- python3-sphinx-rtd-theme
- python3-breathe
- python3-sphinxcontrib.svg2pdfconverter
- inkscape
- doxygen

Please install these packages with your package manager of choice.

e.g. for Debian based systems:

```console
$ sudo apt install python3 python3-sphinx python3-myst-parser python3-sphinx-rtd-theme python3-breathe python3-sphinxcontrib.svg2pdfconverter inkscape doxygen
```

## Syntax

This documentation uses the markdown language embedded into Sphinx through the extension myst-parser.
For a full documentation of the features that myst provides, navigate to the official [myst-parser documentation page](https://myst-parser.readthedocs.io/en/v0.18.1/index.html).

A brief overview of standard markdown can be found [markdownguide.org](https://www.markdownguide.org/basic-syntax/).

```{note}
Ensure that the selected version of the documentation on the previously linked page is set to 0.18.1.
```

## How to build

To build the documentation, navigate to `piconut/doc/manuals` and invoke `make` and the doc will be installed in your $PN_BUILD_DIR.
Open the file `$PN_BUILD_DIR/html/index.html` in a web browser to view the documentation.
Install the documentation with `PREFIX=your-prefix make install`.

To see all available options use `make help`.

## How to include new files

In order to add Markdown files to the documentation overall, it has to be included in `toctree` structure the file `index.md`.
Simply listing the file by it's relative path is sufficient.


## Draw.io diagrams

When creating diagrams for the documentation with [draw.io](https://app.diagrams.net/)
the file should be exported into either of the following formats:
* `.drawio.svg`
* `.drawio.png`

These files can still be edited with draw.io but can also be used in the documentation.

For VS Code users, the [Draw.io Integration](https://marketplace.visualstudio.com/items?itemName=hediet.vscode-drawio)
extension can be used to directly create and edit draw.io diagrams within VS Code.

## Create wave diagrams

When creating wave diagrams for documentation it is suggested to use
[wavedrom](https://wavedrom.com/). An example can be found [here](fig:csr-bus:csr-bus_read).

## Doxygen

### Usage

To include Doxygen code documentation into the Sphinx documentation, the `breathe` extension is used.

When adding a new file to the documentation, the following steps have to be taken:

1. Add the file to the `breathe_projects_source` in the `conf.py` file.

    Either add it to an existing project entry by adding the wanted file to the list of files or 
    creating a new entry.

    ```python
    breathe_projects_source = {
        "how_to_document": (
            os.path.join(src_base_dir, "hw/peripherals/my_module"),
            ["my_module.h", "module.hpp"]
        )
    }
    ```

2. Use the commands of the `breathe` extension to add the documentation to the document.
A list of available commands can be found in the
[breathe documentation](https://breathe.readthedocs.io/en/latest/directives.html).


### Example

#### Documenting a function

To document a function, the following syntax is used:

````markdown

```{doxygenfunction} SC_MODULE(m_module)
:project: how_to_document
```
````

This will render as:

```{doxygenfunction} SC_MODULE(m_module)
:project: how_to_document
```
The name of the function is given as an option to the directive and the project specifies the project the function is part of.

#### Documenting a whole file

To document a whole file, the following syntax is used:

````markdown
```{autodoxygenfile} docs-example.hpp
:project: how_to_document
:allow-dot-graphs:
```
````

```{autodoxygenfile} docs-example.hpp
:project: how_to_document
:allow-dot-graphs:
```

The name of the file is given as an option to the directive and the project specifes the project the file is part of.
The `allow-dot-graphs` generates dot graphs to visualize the dependencies of the file.

## Tips and Workarounds

### Images

To include an image in a MySt file, use the following syntax:

````markdown
```{figure} figures/how_to_doc/pn_logo_long.svg
:scale: 20
:alt: Alt text
:align: center

Caption text
```
````
this will render as:

```{figure} figures/how_to_doc/pn_logo_long.svg
:scale: 20
:alt: Alt text
:align: center

Caption text
```

```{note}
Instead of the `figure` directive, the `image` directive can be used. The `figure` directive is used to include images with captions.\
Additional options can be found in the [MySt documentation](https://myst-parser.readthedocs.io/en/v0.18.1/syntax/roles-and-directives.html#images-and-figures)
```

### Table Formatting

When creating tables, it is recommended to format them as standard Markdown tables not using Myst
`list-ables` or `csv-tables`.  This method allows the author to use line breaks within the cells of the
table. Using other MyST table directives can lead to missing table lines and misplaced content when
generating a PDF.  Therefore, it is recommended to use standard Markdown table formatting, from this
documentation.

### Console commands

When including console commands in the documentation, it is recommended to use code blocks
with the `console` language option. This allows for proper formatting of the commands
and their output.

````markdown
```console
$ gcc --version
gcc (Ubuntu 9.3.0-17ubuntu1~20.04) 9.3.0
```
````