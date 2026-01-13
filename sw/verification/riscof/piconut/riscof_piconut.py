#  -----------------------------------------------------------------------------
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2025 Niklas Sirch <niklas.sirch1@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
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

import os
import re
import shutil
import subprocess
import shlex
import logging
import random
import string
from string import Template
import sys

import riscof.utils as utils
import riscof.constants as constants
from riscof.pluginTemplate import pluginTemplate

logger = logging.getLogger()

CROSS_COMPILE = "riscv64-unknown-elf-"


class piconut(pluginTemplate):
    __model__ = "piconut"

    __version__ = (
        os.popen(
            "git describe --tags --match 'v[0-9]*\\.[0-9]*' --long --dirty='*' --abbrev=4 --always 2>/dev/null || echo 'v0.0-0'"
        )
        .read()
        .strip()
    )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        config = kwargs.get("config")

        # If the config node for this DUT is missing or empty. Raise an error. At minimum we need
        # the paths to the ispec and pspec files
        if config is None:
            print("Please enter input file paths in configuration.")
            raise SystemExit(1)

        # In case of an RTL based DUT, this would be point to the final binary executable of your
        # test-bench produced by a simulator (like verilator, vcs, incisive, etc). In case of an iss or
        # emulator, this variable could point to where the iss binary is located. If 'PATH variable
        # is missing in the config.ini we can hardcode the alternate here.
        self.dut_exe = os.path.abspath(
            os.path.join(
                config["PATH"] if "PATH" in config else "",
                "systems/refdesign/hw/pn-sim",
            )
        )

        # Number of parallel jobs that can be spawned off by RISCOF
        # for various actions performed in later functions, specifically to run the tests in
        # parallel on the DUT executable. Can also be used in the build function if required.
        self.num_jobs = str(config["jobs"] if "jobs" in config else 1)

        # Path to the directory where this python file is located. Collect it from the config.ini
        self.pluginpath = os.path.abspath(config["pluginpath"])

        # Collect the paths to the  riscv-config absed ISA and platform yaml files. One can choose
        # to hardcode these here itself instead of picking it from the config.ini file.
        self.isa_spec = os.path.abspath(config["ispec"])
        self.platform_spec = os.path.abspath(config["pspec"])

        # We capture if the user would like the run the tests on the target or
        # not. If you are interested in just compiling the tests and not running
        # them on the target, then following variable should be set to False
        if "target_run" in config and config["target_run"] == "0":
            self.target_run = False
        else:
            self.target_run = True

    def initialise(self, suite, work_dir, archtest_env):
        # capture the working directory. Any artifacts that the DUT creates should be placed in this
        # directory. Other artifacts from the framework and the Reference plugin will also be placed
        # here itself.
        self.work_dir = work_dir

        # capture the architectural test-suite directory.
        self.suite_dir = suite

        # Note the march is not hardwired here, because it will change for each
        # test. Similarly the output elf name and compile macros will be assigned later in the
        # runTests function
        self.compile_cmd = (
            CROSS_COMPILE
            + "gcc -march={0} -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles -g -T "
            + self.pluginpath
            + "/env/link.ld\
         -I "
            + self.pluginpath
            + "/env/\
         -I "
            + archtest_env
            + " {1} -o {2} {3}"
        )

        # add more utility snippets here
        self.objdump_cmd = CROSS_COMPILE + "objdump -D {0} > {1}"
        self.get_sig_addrs_cmd = (
            CROSS_COMPILE + "nm {0} | grep ' rvtest_sig_{1}' | awk '{{print $$1}}'"
        )
        self.dump_sig_addrs_cmd = (
            CROSS_COMPILE + "nm {0} | grep ' rvtest_sig_' | awk '{{print $$1}}' > {1}"
        )

    def build(self, isa_yaml, platform_yaml):
        # load the isa yaml as a dictionary in python.
        ispec = utils.load_yaml(isa_yaml)["hart0"]

        # capture the XLEN value by picking the max value in 'supported_xlen' field of isa yaml. This
        # will be useful in setting integer value in the compiler string (if not already hardcoded);
        self.xlen = "64" if 64 in ispec["supported_xlen"] else "32"

        # for piconut start building the '--isa' argument. the self.isa is dutnmae specific and may not be
        # useful for all DUTs
        self.isa = "rv" + self.xlen
        if "I" in ispec["ISA"]:
            self.isa += "i"
        if "M" in ispec["ISA"]:
            self.isa += "m"
        if "F" in ispec["ISA"]:
            self.isa += "f"
        if "D" in ispec["ISA"]:
            self.isa += "d"
        if "C" in ispec["ISA"]:
            self.isa += "c"

        self.compile_cmd = (
            self.compile_cmd
            + " -mabi="
            + ("lp64 " if 64 in ispec["supported_xlen"] else "ilp32 ")
        )

    def runTests(self, testList):
        # Delete Makefile if it already exists.
        if os.path.exists(self.work_dir + "/Makefile." + self.name[:-1]):
            os.remove(self.work_dir + "/Makefile." + self.name[:-1])
        # create an instance the makeUtil class that we will use to create targets.
        make = utils.makeUtil(
            makefilePath=os.path.join(self.work_dir, "Makefile." + self.name[:-1])
        )

        # set the make command that will be used. The num_jobs parameter was set in the __init__
        # function earlier
        make.makeCommand = "make -k -j" + self.num_jobs

        for testname in testList:
            testentry = testList[testname]
            test = testentry["test_path"]
            test_dir = testentry["work_dir"]

            execute = []

            execute.append("@cd " + testentry["work_dir"])

            elf = "my.rv32"

            # for each test there are specific compile macros that need to be enabled. The macros in
            # the testList node only contain the macros/values. For the gcc toolchain we need to
            # prefix with "-D". The following does precisely that.
            compile_macros = " -D" + " -D".join(testentry["macros"])

            cc_cmd = self.compile_cmd.format(
                testentry["isa"].lower(), test, elf, compile_macros
            )

            execute.append(cc_cmd)

            execute.append(self.objdump_cmd.format(elf, elf + ".dump.S"))
            signature_start_cmd = self.get_sig_addrs_cmd.format(elf, "begin")
            signature_end_cmd = self.get_sig_addrs_cmd.format(elf, "end")
            execute.append(self.dump_sig_addrs_cmd.format(elf, elf + ".sigaddrs"))

            # name of the signature file as per requirement of RISCOF. RISCOF expects the signature to
            # be named as DUT-<dut-name>.signature. The below variable creates an absolute path of
            # signature file.
            sig_file = os.path.join(test_dir, self.name[:-1] + ".signature")

            # if the user wants to disable running the tests and only compile the tests
            if self.target_run:
                trace_level = os.getenv("TRACE_LVL", "0")
                simcmd = (
                    f"{self.dut_exe} \\\n"
                    f"\t--signature-path={sig_file} "
                    f"--signature-begin=`{signature_start_cmd}` "
                    f"--signature-end=`{signature_end_cmd}` \\\n"
                    f"\t-t{trace_level} {elf}"
                )
            else:
                simcmd = 'echo "NO RUN"'

            execute.append(simcmd)

            # create a target. The makeutil will create a target with the name "TARGET<num>" where num
            # starts from 0 and increments automatically for each new target that is added
            make.add_target(" && \\\n".join(execute))

        make.execute_all(self.work_dir)

        # if target runs are not required then we simply exit as this point after running all
        # the makefile targets.
        if not self.target_run:
            raise SystemExit(0)
