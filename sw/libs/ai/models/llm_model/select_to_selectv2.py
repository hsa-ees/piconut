#!/usr/bin/env python3
"""
Replace the SELECT builtin operator with SELECT_V2 inside a .tflite
FlatBuffer file, in place, without needing TensorFlow installed.

Why this works without rebuilding the whole FlatBuffer:
FlatBuffers store fixed-size scalar table fields (like an int8 or int32
enum value) inline in the buffer, at an offset given by the object's
vtable. Overwriting those bytes in place -- without touching the vtable,
other fields, or any offsets -- produces a byte-identical-structure
FlatBuffer that just reports a different scalar value. This is exactly
what the official FlatBuffers "mutate" accessors do under the hood.

SELECT and SELECT_V2 both take the same three inputs (condition, x, y)
and one output, so swapping the op code is safe as long as your model
does not rely on SELECT's numpy-broadcasting restrictions (SELECT_V2 is
a superset: it additionally allows broadcasting between condition/x/y,
so it will behave correctly for tensors that were valid for SELECT too).

Usage:
    pip install flatbuffers tflite
    python replace_select.py input.tflite output.tflite
"""

import argparse
import struct
import sys

from tflite.Model import Model
from tflite.BuiltinOperator import BuiltinOperator

SELECT = BuiltinOperator.SELECT
SELECT_V2 = BuiltinOperator.SELECT_V2
PLACEHOLDER_FOR_GREATER_OP_CODES = BuiltinOperator.PLACEHOLDER_FOR_GREATER_OP_CODES

# Field IDs come from tflite/schema.fbs -> table OperatorCode:
#   0: deprecated_builtin_code (int8)   -> vtable slot offset 4
#   3: builtin_code            (int32)  -> vtable slot offset 10
DEPRECATED_BUILTIN_CODE_VOFFSET = 4
BUILTIN_CODE_VOFFSET = 10


def _mutate_scalar(tab, voffset, fmt, new_value):
    """Overwrite an existing inline scalar field of a FlatBuffer table.

    Returns True if the field was present (and was patched), False if
    the field is absent from this table's vtable (nothing to patch).
    """
    field_offset = tab.Offset(voffset)
    if field_offset == 0:
        return False
    pos = tab.Pos + field_offset
    struct.pack_into(fmt, tab.Bytes, pos, new_value)
    return True


def replace_select_with_selectv2(input_path, output_path):
    with open(input_path, "rb") as f:
        buf = bytearray(f.read())

    model = Model.GetRootAs(buf, 0)

    num_codes = model.OperatorCodesLength()
    replaced = 0

    for i in range(num_codes):
        opcode = model.OperatorCodes(i)
        current = opcode.BuiltinCode()  # resolves deprecated/new field correctly

        if current != SELECT:
            continue

        tab = opcode._tab

        # SELECT_V2 (123) fits in both the int8 deprecated field and the
        # int32 field, so just write it into whichever field(s) exist.
        patched_new = _mutate_scalar(tab, BUILTIN_CODE_VOFFSET, "<i", SELECT_V2)
        patched_old = _mutate_scalar(tab, DEPRECATED_BUILTIN_CODE_VOFFSET, "<b", SELECT_V2)

        if not patched_new and not patched_old:
            print(
                f"WARNING: operator_codes[{i}] reads as SELECT but neither "
                "builtin_code nor deprecated_builtin_code field is present "
                "in the buffer -- skipped.",
                file=sys.stderr,
            )
            continue

        replaced += 1

    if replaced == 0:
        print("No SELECT operator codes found; nothing to do.")
    else:
        print(f"Replaced {replaced} SELECT operator_code entr"
              f"{'y' if replaced == 1 else 'ies'} with SELECT_V2.")

    with open(output_path, "wb") as f:
        f.write(buf)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="path to input .tflite file")
    parser.add_argument("output", help="path to write patched .tflite file")
    args = parser.parse_args()

    replace_select_with_selectv2(args.input, args.output)


if __name__ == "__main__":
    main()
