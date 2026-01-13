############################################################
#
#  This file is part of the PicoNut project.
#
#  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
#                          Lorenz Sommer <lorenz.sommer@tha.de>
#                     2025 Tristan Kundrat <tristan.kundrat@tha.de>
#                     2025 Martin Erichsen <martin.erichsen@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, this
#     list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation and/or
#     other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
#  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
#  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#############################################################

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import re
import datetime

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information


def parse_contributors():
    with open(
        os.path.join(os.path.dirname(__file__), "../../", "CONTRIBUTORS.md"), "r"
    ) as f:
        lines = f.readlines()
        contrib_list = [e for e in lines if e.startswith("-")]
        contrib_list = [
            re.sub(r"<.*?>", "", e.replace("- ", "")).strip() for e in contrib_list
        ]
        return ", ".join(contrib_list)


# generating version from git tag
# The full version, including alpha/beta/rc tags.
release = (
    os.popen(
        "git describe --tags --match 'v[0-9]*\\.[0-9]*' --long --dirty='*' --abbrev=4 --always 2>/dev/null || echo 'v0.0-0'"
    )
    .read()
    .strip()
)

# if no tag is available, use 0.0.0
if release == "":
    release = "0.0.0"


# The short X.Y version.
version = release

master_doc = "index"
project_base = "PicoNut Manual"
# rtd theme does not show version since 3.0.0 anymore if just one
project = f"{project_base}<br>{version}"

copyright = parse_contributors()
# add the current date
copyright += " " + datetime.datetime.now().strftime("%Y-%m-%d")
author = "Efficient Embedded Systems Group (EES)"


# allowing linebreakes in the latex \author field
latex_documents = [
    (
        master_doc,
        "pn-manual.tex",
        project_base,
        author.replace(", ", "\\and ").replace(" and ", "\\and and "),
        "manual",
    ),
]

# force figures to be displayed at exact positions instead of space optimized placement.
latex_elements = {
    "preamble": r"""
\usepackage{float}""",
    "figure_align": r"""H""",
}


# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# extensions for the sphinx project
extensions = [
    "myst_parser",  # markdown parser
    "breathe",  # doxygen parser
    "sphinx.ext.graphviz",  # graphviz support
    "sphinxcontrib.inkscapeconverter",  # convert svg to pdf
]

templates_path = ["_templates"]
exclude_patterns = []

# enable figures and tables to be numbered
numfig = True

# set supported file types
source_suffix = {".rst": "restructuredtext", ".md": "markdown"}

# options for the markdown parser

# enable markdown extensions
myst_enable_extensions = [
    "amsmath",
    "colon_fence",
    "deflist",
    "dollarmath",
    "fieldlist",
    "html_admonition",
    "html_image",
    "replacements",
    "smartquotes",
    "strikethrough",
    "substitution",
    "tasklist",
]

# options for the doxygen parser
breathe_show_enumvalue_initializer = True

src_base_dir = os.path.abspath("../../")

# path to the doxygen source files
raw_breathe_projects_source = {
    "how_to_document": (
        "doc/manual/contributing/code",
        ["docs-example.h", "docs-example.hpp"],
    ),
    "m_uart": (
        "hw/peripherals/uart",
        [
            "baudgen.h",
            "uart_tx.h",
            "majority_filter.h",
            "uart_rx.h",
            "uart_fifo.h",
            "uart.h",
        ],
    ),
    "m_uart_system": (
        "systems/uart_test/hw",
        ["uart_echo.h", "uart_test_top.h"],
    ),
    "uart_soft": (
        "hw/peripherals/uart_soft",
        ["uart_soft.h"],
    ),
    "graphics_soft": (
        "hw/peripherals/video_soft",
        ["c_soft_graphics.h"],
    ),
    "gui": (
        "hw/gui",
        ["gui.h", "framebuffer_to_image.h", "mainwindow.h"],
    ),
    "membrana_hw": (
        "hw/cpu/membrana/membrana_hw",
        ["membrana_hw.h", "membrana_hw_emem.h"],
    ),
    "wishbone_template": (
        "hw/peripherals/template",
        ["wishbone_template.h"],
    ),
    "nucleus_ref": (
        "hw/cpu/nucleus/nucleus_ref",
        ["csr_master.h", "csr.h", "alu.h", "controller.h"],
    ),
    "remote_bitbang": (
        "hw/cpu_peripherals/debugger_soft/remote_bitbang",
        ["remote_bitbang.h"],
    ),
    "debugger_soft": (
        "hw/cpu_peripherals/debugger_soft",
        [
            "dtm_soft/dtm_soft.h",
            "dm_soft/dm_soft.h",
            "dm_soft/c_debug_regs.h",
            "dm_soft/c_debug_reg_command.h",
            "dm_soft/c_debug_reg_abstractcs.h",
        ],
    ),
    "wb_audio": (
        "hw/peripherals/audio",
        [
            "audio.h",
            "audio_single.h",
            "squarewave.h",
            "trianglewave.h",
            "sawtoothwave.h",
            "sinewave.h",
            "counter.h",
        ],
    ),
    "clint_soft": (
        "hw/cpu_peripherals/clint_soft",
        ["clint_soft.h"],
    ),
    "m_clint": ("hw/cpu_peripherals/clint", ["clint.h"]),
    "m_graphics": (
        "hw/peripherals/video",
        [
            "color_translator.h",
            "framebuffer_ram.h",
            "framebuffer_source.h",
            "vga_color_generator.h",
            "vga_sync_generator.h",
            "vga_timings.h",
            "wb_graphics_config.h",
            "demo_image_source.h",
            "wb_graphics.h",
            "wb_slave.h",
        ],
    ),
    "audio_soft": (
        "hw/peripherals/audio_soft",
        ["c_soft_audio.h", "audiooutput.h"],
    ),
}

breathe_projects_source = {
    breathe_key: (
        os.path.join(src_base_dir, module_path),
        files,
    )
    for breathe_key, (module_path, files) in raw_breathe_projects_source.items()
}

BUILDDIR = os.environ["BUILD_DIR"]
# path to the doxygen xml files
project_base = os.path.abspath(os.path.join(BUILDDIR, "breathe", "doxygen"))
breathe_projects = {
    project: os.path.join(project_base, project, "xml")
    for project in breathe_projects_source.keys()
}

breathe_doxygen_config_options = {
    "WARN_IF_UNDOCUMENTED": "NO",  # exhaustive documentation is too strict so we rely on code reviews
}


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]
html_favicon = "_static/favicon.png"

