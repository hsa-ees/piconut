#!/usr/bin/env python3

# This file is part of the PicoNut project.
#
# Copyright (C)      2025 Tristan Kundrat <tristan.kundrat@tha.de>
#     Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation and/or
#    other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
This file checks, if the sha256 checksum of audio_tb.vcd matches
audio_tb.vcd.sha256, ignoring the timestamp metadata at the
beginning of the vcd file.

It also generates the checksum file when called with the "update" argument.
"""

import subprocess
import sys


def mismatch_error(sum_actual: str, sum_expected: str):
    print("Error: Checksum mismatch!")
    print("Actual checksum: " + sum_actual)
    print("Expected checksum: " + sum_expected)
    print(
        "If this is expected and there is a valid change in the vcd file, update it using:"
    )
    print("$ make verify-sim-update")
    exit(1)


def proc_error(returncode: int, stderr: str):
    print("Error: Subprocess exited with " + str(returncode))
    print("STDERR: " + stderr)


def skip_metadata(lines: list[str]) -> list[str]:
    # skips date and version blocks
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        if line == "$date" or line == "$version":
            # go to next $end
            while i < len(lines) and lines[i].strip() != "$end":
                i += 1
            i += 1  # skip $end
        else:
            break
    return lines[i:]


def compare_checksum(file: str, sumfile: str):
    with (
        open(file, "r", encoding="utf-8") as f,
        open(sumfile, "r", encoding="utf-8") as sf,
    ):
        lines = skip_metadata(f.readlines())
        checksum = "".join(skip_metadata(sf.readlines())).strip()

    proc = subprocess.Popen(
        ["sha256sum"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    stdout, stderr = proc.communicate("".join(lines))

    if proc.returncode != 0:
        proc_error(proc.returncode, stderr)

    if stdout.strip() != checksum:
        mismatch_error(stdout.strip(), checksum)


def update_checksum(file: str, sumfile: str):
    sf = open(sumfile, "w", encoding="utf-8")

    with open(file, "r", encoding="utf-8") as f:
        lines = skip_metadata(f.readlines())

    proc = subprocess.Popen(
        ["sha256sum"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    stdout, stderr = proc.communicate("".join(lines))

    if proc.returncode != 0:
        proc_error(proc.returncode, stderr)

    sf.write(stdout)
    print("Checksum file updated.")
    exit(0)


if __name__ == "__main__":
    if len(sys.argv) == 2:
        if sys.argv[1] == "update":
            # exits if done/error
            update_checksum("audio_tb.vcd", "audio_tb.vcd.sha256")
        if sys.argv[1] == "--help":
            print(__doc__)

    # exists if error
    compare_checksum("audio_tb.vcd", "audio_tb.vcd.sha256")

    print("Success: Checksums match.")
    exit(0)
