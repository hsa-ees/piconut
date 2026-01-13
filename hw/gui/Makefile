#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2023 Felix Wagner <felix.wagner1@hs-augsburg.de
#                2024 Lukas Bauer <lukas.bauer1.tha.de>
#      Hochschule Augsburg, University of Applied Sciences
#
#  Description:
#    Makefile of the module CPU which provides the PicoNut cores of
#    all currently available Capability levels
#
#  --------------------- LICENSE -----------------------------------------------
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
#  -----------------------------------------------------------------------------

##### EDIT HERE
################# Module Configuration #########################################

################ MAKEFILE FOR: peripheral modules ##############################

################################## QT ONLY !!!!#################################

##### To make Qt work, copy the following line in the .bashrc file

# MODULE_SRC:               Sources that are used for Simulation as well as
#                           synthesis. (including the testbench/sc_main file)
MODULE_SRC =

# TESTBENCH_SRC:            Source that contains tb (and thus sc_main)
TESTBENCH_SRC = gui_tb/gui_tb.cpp

# SIM_SRC:                  Sources that are only used in simulationon
SIM_SRC = \
	framebuffer_to_image.cpp \
	mainwindow.cpp \
    gui.cpp

# MEMBRANA_VERSION:             Version of the membrana that is used in this module
MEMBRANA_VERSION = membrana_soft

# MODULE_INCLUDES:          Include directives for the compiler (e.g. -I..)
MODULE_INCLUDES = -I.. \
                  -I$(QT_HOME)/include/QtCore \
                  -I$(QT_HOME)/include/QtGui \
                  -I$(QT_HOME)/include/QtWidgets \
                  -I$(QT_HOME)/include

# MODULE_CFLAGS:            Additional compiler flags this module needs
MODULE_CFLAGS =

# MODULE_LDFLAGS:           Additional linker flags this module needs
MODULE_LDFLAGS = -fuse-ld=mold -L$(QT_HOME)/lib \
                 -lQt6Core -lQt6Gui -lQt6Widgets -lsystemc -DQT_CORE_LIB -DQT_GUI_LIB -DQT_WIDGETS_LIB

# SYNTHESIS_TOPMODULE:      Defines the Top Module for synthesis as defined inside
#							the test bench cpp
SYNTHESIS_TOP_MODULE =

# MOC_SRC						Files that need to be moced with qt
MOC_SRC ?= mainwindow.h gui.h

# UI_SRC						UI files to convert to .h files
UI_SRC ?= mainwindow.ui

################################################################################
##### DO NOT EDIT FROM HERE ON #################################################
################################################################################

################# Module Makefile ##############################################
SYSC_MODULE_DIR:= $(abspath $(CURDIR))
PICONUT_HOME ?= $(CURDIR)/../../..

################# Automatic Name Generation ####################################
# Get Module name from folder name
MODULE_NAME = $(subst module_,,$(lastword $(subst /, ,$(SYSC_MODULE_DIR))))


################# Directory Makefile Include #####################################
include $(PICONUT_HOME)/directory-base.mk

################# Master Makefile Include #####################################
# include make targets for the qt application
# needs to be included before the systemc base mk in order
# to have moc files built automatically with the other files
# otherwise they need to be added to the SIM_SRC manually
include $(QT_BASE_MK)

include $(SYSC_BASE_MK)