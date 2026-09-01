#!/usr/bin/env python3

################################################################################
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2026 Johannes Hofmann <johannes.hofmann1@tha.de>
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
################################################################################


import argparse
import os
import numpy as np
from PIL import Image

from ai_edge_litert.interpreter import Interpreter


W, H, C = 640, 640, 3
FLAT_SIZE = W * H * C


def load_tflite_input_info(model_path: str):
    itp = Interpreter(model_path=model_path)
    itp.allocate_tensors()
    inp = itp.get_input_details()[0]

    dtype = inp["dtype"]
    shape = inp["shape"]
    scale, zero_point = inp["quantization"]  # (float, int)

    return dtype, shape, float(scale), int(zero_point), inp.get("name", "")


def image_to_int8_tensor(image_path: str, scale: float, zero_point: int, nearest: bool) -> np.ndarray:
    img = Image.open(image_path).convert("RGB")
    resample = Image.Resampling.NEAREST if nearest else Image.Resampling.BILINEAR
    img = img.resize((W, H), resample=resample)
    
    img.save("output_image.jpg", "JPEG", quality=90, optimize=True)

    img_u8 = np.asarray(img, dtype=np.uint8)  # (640,640,3), RGB

    # Common YOLO float preprocessing used by exporters: [0..255] -> [0..1]
    x = img_u8.astype(np.float32) / 255.0

    # Quantize to int8 using TFLite affine quantization:
    # q = round(x/scale) + zero_point
    q = np.rint(x / scale + zero_point)
    q = np.clip(q, -128, 127).astype(np.int8)  # int8 input tensor

    return q  # shape (640,640,3)


def write_c_int8(arr_hwc: np.ndarray, out_path: str, header_path: str, var_name: str, scale: float, zero_point: int):
    flat = arr_hwc.reshape(-1)
    if flat.size != FLAT_SIZE:
        raise ValueError(f"Expected {FLAT_SIZE} elements, got {flat.size}")

    vals = flat.astype(np.int16).tolist()  # easier to format negatives
    per_line = 16

    lines = []
    for i in range(0, len(vals), per_line):
        chunk = vals[i:i + per_line]
        lines.append("  " + ", ".join(str(v) for v in chunk) + ",")

    init = "\n".join(lines)

    c_source = f"""\
#include "{header_path}"

#define YOLO_IN_W {W}
#define YOLO_IN_H {H}
#define YOLO_IN_C {C}
#define YOLO_IN_SIZE (YOLO_IN_W*YOLO_IN_H*YOLO_IN_C)

// Layout: HWC interleaved RGB
// Quantization: q = round(x/scale) + zero_point, with x = pixel/255.0
// Model input scale: {scale}
// Model input zero_point: {zero_point}
const int8_t {var_name}[YOLO_IN_SIZE] = {{
    {init}}};
const size_t {var_name}_len =  YOLO_IN_SIZE;

const int8_t* cone_detection_input_data[] = {{{var_name}}};

const size_t cone_detection_input_data_len[] = {{{var_name}_len}};
"""
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(c_source)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="Path to int8 .tflite model")
    ap.add_argument("--image", required=True, help="Path to input image (png/jpg/...)")
    ap.add_argument("-o", "--output", default=None, help="Output .h path (default: <image_basename>_yolo_int8.h)")
    ap.add_argument("--name", default="cone_detection_input_data_00000", help="C array variable name")
    ap.add_argument("--header_path", default="cone_detection_input_data.h", help="Absolute path of the dedicated header file")
    ap.add_argument("--nearest", action="store_true", help="Use nearest-neighbor resize (default: bilinear)")
    args = ap.parse_args()

    dtype, shape, scale, zero_point, tname = load_tflite_input_info(args.model)

    if dtype != np.int8:
        raise SystemExit(f"Expected int8 input, got {dtype}")
    if list(shape) != [1, H, W, C]:
        raise SystemExit(f"Expected shape [1,{H},{W},{C}], got {shape}")

    if scale == 0.0:
        raise SystemExit("Model reports scale=0.0; input may not be quantized.")

    print(f"Input tensor name: {tname}")
    print(f"Input tensor dtype: {dtype}")
    print(f"Input tensor shape: {shape}")
    print(f"Quant params: scale={scale}, zero_point={zero_point}")

    arr = image_to_int8_tensor(args.image, scale=scale, zero_point=zero_point, nearest=args.nearest)

    out_path = args.output
    if out_path is None:
        base = os.path.splitext(os.path.basename(args.image))[0]
        out_path = f"{base}_yolo_int8.c"

    write_c_int8(arr, out_path=out_path, header_path=args.header_path, var_name=args.name, scale=scale, zero_point=zero_point)

    print(f"Wrote: {out_path}")
    print(f"Elements: {FLAT_SIZE}")


if __name__ == "__main__":
    main()
