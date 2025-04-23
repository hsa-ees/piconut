# How to Document
**Authors: Lukas Bauer 2024, Lorenz Sommer 2024**

## Introduction

This page will detail how to document development on the PicoNut project using the setup provided.

### Software dependencies

In order to build the documentation from the source directory, a few software packages have to be installed.

- pyhton3
- python3-sphinx
- python3-myst-parser
- python3-sphinx-rtd-theme
- python3-breathe
- python3-sphinxcontrib.svg2pdfconverter
- inkscape

Please install these packages with your package manager of choice.

### MyST parser

This documentation uses the markdown language embedded into Sphinx through the extension myst-parser. For a full documentation of the features that myst provides, navigate to the official [myst-parser documentation page](https://myst-parser.readthedocs.io/en/v0.18.1/index.html).

```{note}
Ensure that the selected version of the documentation on the previously linked page is set to 0.18.1.
```

### How to build

To build the documentation, navigate to `piconut/doc-src/pn-manuals` and invoke `make html`.
This will create a file `index.html` in `piconut/doc-src/build/html` which can be opened with any web browser.

Additional targets for the `make` command include:

- `make clean` to clear the build files.
- `make latexpdf` to generate a PDF file.

To see all available options use `make help`.

### Standard markdown syntax

This documentation is created using the markdown language.

A brief overview of standard markdown can be found [markdownguide.org](https://www.markdownguide.org/basic-syntax/).


### How to include markdown files

In order to add Markdown files to the documentation overall, it has to be included in `toctree` structure the file `index.md`.
Simply listing the file by it's relative path is sufficient.



## Additional Syntax Provided by MyST

The MyST-parser provides a range of extra features beyond standard markdown.

### Comments

Comments can be made within source `.md` files. For this the `%` symbol is used.

```markdown
% comment
```


### Directives and Roles

MyST acts as a wrapper for RestructuredText roles and directives, simplifying the syntax.

Directives and roles allow the user to format text in a predefined manner. Directives are structured as a block surrounded by backticks, roles are constructed in-line.

According to the MyST manual, all [docutils roles](https://docutils.sourceforge.io/docs/ref/rst/roles.html), [docutils directives](https://docutils.sourceforge.io/docs/ref/rst/directives.html), [sphinx roles](https://www.sphinx-doc.org/en/master/usage/restructuredtext/roles.html) and [sphinx directives](https://www.sphinx-doc.org/en/master/usage/restructuredtext/directives.html) should be functional within MyST.

A short overview over the most commonly used directives will be provided here.

#### Admonitions

Admonitions such as warnings and notes are created as follows:

````markdown
```{note}
    This is a note.
```
````
will render as:

```{note}
    This is a note.
```

Options include: `note`, `warning`, `attention`, `error`, `hint`, `important` and `tip`

#### Code

Code blocks can be constructed with the standard markdown syntax or the MyST provided syntax.

##### Standard Markdown code blocks
````
```c
int main()
    {
        printf("Hello World!\r");
    }
```
````
renders as
```c
int main()
    {
        printf("Hello World!\r");
    }
```


##### MyST directive

````
```{code-block} $language
    code
```
````

Is the default syntax.

````
```{code-block} c
    int main()
    {
        printf("Hello World!\r");
    }
```
````
renders as
```{code-block} c
    int main()
    {
        printf("Hello World!\r");
    }
```

#### Tables and figures

Documentation on how to creates tables and how to include images and figures into the document can be found under [tables](fig:how_to:tables) and [figures and images](fig:how_to:imagesfigures).


#### Roles

Roles in essence achieve the same things that directives do, but they are used in-line instead of requiring their own block.

The default syntax is:
```markdown
{role-name}`role content`
```

For example:
```
    {math}`1+2 = 3`
```
renders as

{math}`1+2 = 3`


##### References

References are useful when the user wants to create a quick and clickable reference to another section of the documentation.
References can be created for headlines, images, tables and code blocks.

To begin with, a reference has to be declared at the target of the reference. The syntax for this is

```
(referencename)=
```

To create a clickable object to lead to a reference, the following snytax options can be used:


```
[text](referencename)
```

Example:

```markdown
(testreference)=
# Some headline

This is an example.

[This reference](testreference) will jump to the headline.
```

Alternative syntax for the clickable link is
```markdown
{ref}`testreference`
```


### Nesting of Backticks

When a block encased in backticks needs to be nested in another block encased in backticks,
the outer block needs to have an additional backtick at the beginning and end of the block.

````` markdown
````
``` markdown
# hello
```
````
`````

% ### Bibliographie ???


### Footnotes

To add a footnote, use the following syntax:

```markdown
Here is a footnote reference,[^1] and another.[^longnote]

[^1]: Here is the footnote.
[^longnote]: Here's one with multiple blocks.
```

This will render as:

Here is a footnote reference,[^1] and another.[^longnote]

[^1]: Here is the footnote.
[^longnote]: Here's one with multiple blocks.


### Embed other files

To include links to other files e.g. documents in the pdf format, use the following syntax:

```markdown
Here is a link to the technical report [zybo-z7_rm.pdf](doc/zybo-z7_rm.pdf).
```

This will render as:

Here is a link to the technical report [zybo-z7_rm.pdf](doc/zybo-z7_rm.pdf).

```{note}
The link will not be usable in the pdf version of the document.
```
(fig:how_to:tables)=
## Tables

To create tables the [Github Markdown Syntax](https://github.github.com/gfm/#tables-extension-) can be used.

Use pipes `|` to separate columns and hyphens `-` to create a header row.

```markdown
| Header 1 | Header 2 | Header 3 |
| -------- | -------- | -------- |
| Cell 1   | Cell 2   | Cell 3   |
| Cell 4   | Cell 5   | Cell 6   |
```

This will render as:

| Header 1 | Header 2 | Header 3 |
| -------- | -------- | -------- |
| Cell 1   | Cell 2   | Cell 3   |
| Cell 4   | Cell 5   | Cell 6   |

The table can be aligned to the left, right, or center by adding colons `:` to the header row.

```markdown

| Left-aligned | Center-aligned | Right-aligned |
| :---         | :---:          | ---:          |
| Cell 1       | Cell 2         | Cell 3        |
| Cell 4       | Cell 5         | Cell 6        |
```

This will render as:

| Left-aligned | Center-aligned | Right-aligned |
| :---         | :---:          | ---:          |
| Cell 1       | Cell 2         | Cell 3        |
| Cell 4       | Cell 5         | Cell 6        |

It is also possible to add a caption and other options to the table.

````markdown
```{table} Table Title
:align: center

| Header 1 | Header 2 | Header 3 |
| :------- | :------: | -------: |
| Cell 1   | Cell 2   | Cell 3   |
| Cell 4   | Cell 5   | Cell 6   |
```
````

This will render as:

```{table} Table Title
:align: center

| Header 1 | Header 2 | Header 3 |
| :------- | :------: | -------: |
| Cell 1   | Cell 2   | Cell 3   |
| Cell 4   | Cell 5   | Cell 6   |
```

```{note}
The `table` directive is used to include tables with captions and additional options.\
These can be found in the [MySt Parser Table Documentation](https://myst-parser.readthedocs.io/en/v0.18.1/syntax/roles-and-directives.html#tables)
```

(fig:how_to:imagesfigures)=
## Images and Figures

Images and figures can be includes in multiple ways.

### Supported Formats

The following image formats are supported:
- png
- jpg
- jpeg
- pdf
- svg

### Markdown Syntax

To include an image in a markdown file, use the following syntax:

```markdown
![Alt text](figures/how_to_doc/Pictail_BM70.jpeg)
```

this will render as:

![Alt text](figures/how_to_doc/Pictail_BM70.jpeg)

### MySt Syntax

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
insead of the `figure` directive, the `image` directive can be used. The `figure` directive is used to include images with captions.\
Additional options can be found in the [MySt documentation](https://myst-parser.readthedocs.io/en/v0.18.1/syntax/roles-and-directives.html#images-and-figures)
```

## Using draw.io diagrams

When creating diagrams for the documentation with [draw.io](https://app.diagrams.net/)
the file should be exported into either of the following formats:
* `.drawio.svg`
* `.drawio.png`

These files can still be edited with draw.io but can also be used in the documentation.

## Create wave diagrams

When creating wave diagrams for documentation it is suggested to use [wavedrom](https://wavedrom.com/). An example can be found [here](fig:csr-bus:csr-bus_read).

## Doxygen

### Usage

To include Doxygen code documentation into the Sphinx documentation, the `breathe` extension is used.

When adding a new file to the documentation, the following steps have to be taken:

1. Add the file to the `breathe_projects` and `breathe_projects_source` in the `conf.py` file.

Either add it to an existing project entry by adding the wanted file to the list of files, like so:

```python
breathe_projects = {
    "how_to_document": ("../build/breathe/doxygen/how_to_document/xml")
}
breathe_projects_source = {
    "how_to_document": ("how_to_document/", ["paranut.h", "piconut.hpp"])
}
```

Or create a new project entry by adding a new key-value pair to the dictionary, like so:

```python
breathe_projects = {
    "how_to_document": ("../build/breathe/doxygen/how_to_document/xml"),
    "<new_project>": ("../build/breathe/doxygen/<new_project>/xml")
}
breathe_projects_source = {
    "how_to_document": ("how_to_document/", ["paranut.h", "piconut.hpp"]),
    ">new_project>": ("<path to directory>", ["<list of filenames>"])
}

```

2. Use the commands of the `breathe` extension to add the documentation to the document.
A list of available commands can be found in the [breathe documentation](https://breathe.readthedocs.io/en/latest/directives.html).


### Example

#### Documenting a function

To document a function, the following syntax is used:

````markdown

```{doxygenfunction} start_sim
:project: how_to_document
```
````

This will render as:

```{doxygenfunction} pn_halt
:project: how_to_document
```
The name of the function is given as an option to the directive and the project specifes the project the function is part of.

#### Documenting a whole file

To document a whole file, the following syntax is used:

````markdown
```{autodoxygenfile} piconut.hpp
:project: how_to_document
:allow-dot-graphs:
```
````

```{autodoxygenfile} piconut.hpp
:project: how_to_document
:allow-dot-graphs:
```

The name of the file is given as an option to the directive and the project specifes the project the file is part of.
The `allow-dot-graphs` generates dot graphs to visualize the dependencies of the file.

## Tips and Workarounds

### Table Formatting

When creating tables, it is recommended to format them as shown in the [tables](fig:how_to:tables) section. This method allows the author to use line breaks within the cells of the table. Using other MyST table directives, such as `csv-table` or `list-table`, can lead to missing table lines and misplaced content when generating a PDF. Therefore, it is recommended to use standard Markdown table formatting, from this documentation.