#!/usr/bin/python3

###########################################################################
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2026 Johannes Hofmann <johannes.hofmann1@tha.de>
#      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg
#
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
###########################################################################

import argparse
import keras_hub
import numpy as np
import tensorflow as tf

parser = argparse.ArgumentParser(description="Export OPT-125M model as .tflite")
parser.add_argument("output", help="Output")

args = parser.parse_args()

# ============================================================
# Load OPT-125M model and tokenizer
# ============================================================

print("Loading OPT-125M model and tokenizer...")

opt_lm = keras_hub.models.OPTCausalLM.from_preset("opt_125m_en")
tokenizer = keras_hub.models.OPTTokenizer.from_preset("opt_125m_en")

# ============================================================
# Configuration
# ============================================================

SEQ_LEN = 64
BATCH_SIZE = 1

# ============================================================
# Define serving function with static input shapes
# ============================================================

@tf.function(
    input_signature=[
        tf.TensorSpec(
            shape=(BATCH_SIZE, SEQ_LEN),
            dtype=tf.int32,
            name="token_ids",
        ),
        tf.TensorSpec(
            shape=(BATCH_SIZE, SEQ_LEN),
            dtype=tf.bool,
            name="padding_mask",
        ),
    ]
)
def serving_fn(token_ids, padding_mask):
    return opt_lm(
        {
            "token_ids": token_ids,
            "padding_mask": padding_mask,
        }
    )

print("Tracing concrete function...")
concrete_func = serving_fn.get_concrete_function()

# ============================================================
# Configure TFLite converter
# ============================================================

print("Configuring TFLite converter for INT8 quantization...")

converter = tf.lite.TFLiteConverter.from_concrete_functions(
    [concrete_func]
)

converter.optimizations = [tf.lite.Optimize.DEFAULT]

# ============================================================
# Representative dataset
# ============================================================

from datasets import load_dataset

print("Loading calibration dataset...")

dataset = load_dataset(
    "Salesforce/wikitext",
    "wikitext-103-raw-v1",
    split="train"
)

calibration_texts = []
short_texts = []   # will tokenize to roughly < 20 tokens
medium_texts = []  # roughly 20-50 tokens
long_texts = []    # roughly 50+ tokens (fills or exceeds SEQ_LEN)

for item in dataset:
    text = item["text"].strip()

    if not text:
        continue

    length = len(text)

    if 10 <= length < 80 and len(short_texts) < 300:
        short_texts.append(text)
    elif 80 <= length < 250 and len(medium_texts) < 400:
        medium_texts.append(text)
    elif length >= 250 and len(long_texts) < 300:
        long_texts.append(text)

    if len(short_texts) >= 300 and len(medium_texts) >= 400 and len(long_texts) >= 300:
        break

print(f"Short: {len(short_texts)}, Medium: {len(medium_texts)}, Long: {len(long_texts)}")

calibration_texts = short_texts + medium_texts + long_texts

import random
random.shuffle(calibration_texts) 

print(
    "Calibration samples:",
    len(calibration_texts)
)

def representative_dataset():

    for text in calibration_texts:

        tokenized = tokenizer([text])

        if hasattr(tokenized, "numpy"):
            token_ids = tokenized.numpy().astype(np.int32)
        else:
            token_ids = np.array(tokenized, dtype=np.int32)

        token_ids = token_ids[:, :SEQ_LEN]

        if token_ids.shape[1] < SEQ_LEN:
            pad = np.zeros(
                (1, SEQ_LEN-token_ids.shape[1]),
                dtype=np.int32
            )

            token_ids = np.concatenate(
                [token_ids, pad],
                axis=1
            )

        padding_mask = np.zeros(
            (1, SEQ_LEN),
            dtype=np.bool_
        )

        real_tokens = min(
            len(tokenized[0]),
            SEQ_LEN
        )

        padding_mask[:, :real_tokens] = True

        yield [token_ids, padding_mask]

converter.representative_dataset = representative_dataset

# ============================================================
# INT8 conversion settings
# ============================================================

converter.target_spec.supported_ops = [
    tf.lite.OpsSet.EXPERIMENTAL_TFLITE_BUILTINS_ACTIVATIONS_INT16_WEIGHTS_INT8,
    tf.lite.OpsSet.TFLITE_BUILTINS,
]


# converter.target_spec.supported_ops = [
#     tf.lite.OpsSet.TFLITE_BUILTINS_INT8,
#     tf.lite.OpsSet.TFLITE_BUILTINS,
# ]

converter.experimental_new_quantizer = True

# ============================================================
# Step 1: find every node with empty calibration stats
# ============================================================

print("Running quantization debugger to find unsupported nodes...")

debugger = tf.lite.experimental.QuantizationDebugger(
    converter=converter,
    debug_dataset=representative_dataset,
)
debugger.run()

layer_stats = debugger.layer_statistics
missing_stats_nodes = [
    name for name, s in layer_stats.items()
    if s.get("min") is None or s.get("max") is None
]

print(f"Found {len(missing_stats_nodes)} node(s) with no calibration stats:")
for n in missing_stats_nodes:
    print("  ", n)

# ============================================================
# Step 2: denylist exactly those nodes and re-run
# ============================================================

debug_options = tf.lite.experimental.QuantizationDebugOptions(
    denylisted_nodes=missing_stats_nodes
)

debugger = tf.lite.experimental.QuantizationDebugger(
    converter=converter,
    debug_dataset=representative_dataset,
    debug_options=debug_options,
)
debugger.run()

# ============================================================
# Step 3: export the final quantized model
# ============================================================

tflite_model = debugger.get_nondebug_quantized_model()

# converter.target_spec.supported_ops = [
#     tf.lite.OpsSet.TFLITE_BUILTINS,
#]

# Uncomment if you want strict INT8 I/O
# converter.inference_input_type = tf.int8
# converter.inference_output_type = tf.int8

# ============================================================
# Convert
# ============================================================

#print("Converting model (this may take several minutes)...")

# tflite_model = converter.convert()

# ============================================================
# Save model
# ============================================================

output_filename = args.output

with open(output_filename, "wb") as f:
    f.write(tflite_model)

print(
    f"Successfully exported INT8 quantized model to "
    f"{output_filename}"
)
