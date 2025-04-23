############################################################
#
#  This file is part of the PicoNut project.
#
#  Copyright (C)      2024 Lukas Bauer <lukas.bauer1@tha.de>
#                          Lorenz Sommer <lorenz.sommer@tha.de>
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

master_doc = "index"
project = 'PicoNut Manual'
copyright = "Marco Milenkovic, Lorenz Sommer, Claus Janicher, Johannes Hofmann, Lukas Bauer, Gundolf Kiefer, Michael Schäferling" 

# add the current date
copyright +=  " " + datetime.datetime.now().strftime("%Y-%m-%d")
author = "Research Group Efficient Embedded Systems (EES)"


# allowing linebreakes in the latex \author field
latex_documents = [
    (master_doc, 'pn-manual.tex', project,
     author.replace(', ', '\\and ').replace(' and ', '\\and and '),
     'manual'),
]

# force figures to be displayed at exact positions instead of space optimized placement.
latex_elements = {
    'preamble': r'''
\usepackage{float}''',
'figure_align': r'''H'''
}


# generating version from git tag
# The full version, including alpha/beta/rc tags.
release = re.sub('^v', '', os.popen('git describe').read().strip())

# if no tag is available, use 0.0.0
if release == "":
    release = "0.0.0"


# The short X.Y version.
version = release


# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# extensions for the sphinx project
extensions = ['myst_parser',                         #markdown parser
            'breathe',                               #doxygen parser
            "sphinx.ext.graphviz",                   #graphviz support
            "sphinxcontrib.inkscapeconverter",       #convert svg to pdf
                         ]

templates_path = ['_templates']
exclude_patterns = []

# enable figures and tables to be numbered
numfig = True

# set supported file types
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown"
}

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

# path to the doxygen xml files
breathe_projects = {
    "how_to_document": ("../build/breathe/doxygen/how_to_document/xml"),
    "wb_uart": ("../build/breathe/doxygen/wb_uart/xml"),
    "wb_uart_system": ("../build/breathe/doxygen/wb_uart_system/xml"),
    "softuart": ("../build/breathe/doxygen/softuart/xml"),
    "hw_memu": ("../build/breathe/doxygen/hw_memu/xml"),
    "wishbone_template": ("../build/breathe/doxygen/wishbone_template/xml"),
    "minimalnucleus": ("../build/breathe/doxygen/minimalnucleus/xml"),
    "c_remote_bitbang": ("../build/breathe/doxygen/c_remote_bitbang/xml"),
    "c_soft_debugger": ("../build/breathe/doxygen/c_soft_debugger/xml"),
}

# path to the doxygen source files
breathe_projects_source = {
    "how_to_document": ("contributor_resources/code_example/", ["paranut.h", "piconut.hpp"]),
    "wb_uart": ("../../../hw/peripherals/wb_uart", ["baudgen.h", "uart_tx.h", "majority_filter.h", "uart_rx.h", "uart_fifo.h", "wb_uart.h"]),
    "wb_uart_system": ("../../../systems/wb_uart_test/hw", ["uart_test.h", "top.h"]),
    "softuart": ("../../../hw/peripherals/c_soft_uart", ["c_soft_uart.h"]),
    "hw_memu": ("../../../hw/piconut/memus/hw_memu", ["hw_memu.h", "blockram_memu.h"]),
    "wishbone_template": ("../../../hw/peripherals/wb_template", ["wishbone_template.h"]),
    "minimalnucleus": ("../../../hw/piconut/nuclei/minimalnucleus", ["csr_master.h", "csr.h"]),
    "c_remote_bitbang": ("../../../hw/peripherals/c_remote_bitbang", ["c_remote_bitbang.h"]),
    "c_soft_debugger": ("../../../hw/peripherals/c_soft_debugger", ["c_dtm.h", "c_soft_dm.h", "c_debug_regs.h", "c_debug_reg_command.h", "c_debug_reg_abstractcs.h", "debug_handler.h"]),
}

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
