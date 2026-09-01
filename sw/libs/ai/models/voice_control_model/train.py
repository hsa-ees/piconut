#!/usr/bin/python3

###########################################################################
#
#  This file is part of the PicoNut project.
#
#  Copyright (C) 2026 Albert Müller <albert.mueller1@tha.de>
#                     Johannes Hofmann <johannes.hofmann1@tha.de>
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
import os
import pathlib
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models
import traceback

# =============================================================================
# MCU-OPTIMIZED VOICE COMMAND MODEL TRAINING SCRIPT
#
# Target hardware: RISC-V MCU running at 25 MHz (PicoNut), no hardware FPU.
# This script trains a small CNN to classify 8 spoken commands and exports
# a fully INT8-quantized TFLite model that can run on the microcontroller.
#
# Key design decisions for this constrained target:
#
#   1. Shorter audio window (see AUDIO_SAMPLES below) reduces both the
#      amount of data processed per inference and the size of the
#      resulting spectrogram, which directly shrinks the model's input
#      layer and the number of MACs (multiply-accumulate operations).
#
#   2. Fewer Mel bins (32 instead of a more typical 64) halve the
#      frequency resolution of the spectrogram, trading some accuracy
#      for a smaller input tensor and fewer weights in the first layer.
#
#   3. A smaller network (16->32 filters, Dense(64) instead of Dense(256))
#      keeps the parameter count and RAM footprint low enough for an MCU
#      with very limited SRAM/Flash.
#
#   4. Spectrogram computation is done OUTSIDE the TFLite model (i.e. not
#      as a graph op). The exported model only ever sees a precomputed
#      spectrogram as input. This matters because TFLite's STFT/FFT
#      support on embedded targets is poor or nonexistent, so the
#      equivalent computation must be reimplemented in C on the MCU
#      (see audio_input.h) and MUST match this Python implementation
#      bit-for-bit in terms of parameters (window size, hop size,
#      number of mel bins, etc.), or the model will see out-of-distribution
#      inputs at inference time and accuracy will collapse.
#
#   5. Full INT8 quantization (input, weights, AND output) avoids any
#      float32 fallback. This is critical: without a hardware FPU,
#      float32 arithmetic on a RISC-V core is roughly 10-50x slower than
#      INT8 fixed-point arithmetic, which would make real-time inference
#      impossible on a 25 MHz core.
#
#   6. DepthwiseConv2D is used instead of regular Conv2D wherever
#      possible. A depthwise convolution applies one filter per input
#      channel instead of combining all channels in every filter, which
#      cuts the number of multiplications by roughly 8-10x compared to a
#      standard Conv2D of similar shape, at only a small accuracy cost.
# =============================================================================


parser = argparse.ArgumentParser(description="Train a small voice control net")
parser.add_argument("--data_dir", type=str, default="dataset/", help="Path to dataset directory")
parser.add_argument("output", help="Output .tflite")

args = parser.parse_args()

try:
    print("--- TRAINING START (MCU-optimized) ---")

    # =========================================================================
    # CONFIGURATION - adjust these values here
    #
    # IMPORTANT: Every one of these parameters is baked into the exported
    # model's expected input shape AND into audio_input_template.h. If you
    # change a value here, you must regenerate voice_model.h AND update the
    # matching C constants in audio_input.h, or the MCU's spectrogram will
    # no longer match what the model was trained on.
    # =========================================================================
    SAMPLE_RATE        = 16000   # Microphone input sample rate in Hz.
                                  # Must match the ADC/mic driver's actual
                                  # sampling rate on the MCU.
    AUDIO_LENGTH_MS    = 16000   # Nominal length label (ms) for documentation
                                  # purposes only; not used in computation.
    AUDIO_SAMPLES       = 16000  # Number of raw audio samples per inference
                                  # window. At 16 kHz this is a 1-second
                                  # window. This directly sets how much
                                  # audio must be buffered on the MCU before
                                  # a spectrogram can be computed.
    NUM_MEL_BINS       = 32      # Number of Mel filterbank output bins
                                  # (reduced from a more common 64 to shrink
                                  # the spectrogram and thus the model).
    FRAME_LENGTH       = 255     # STFT analysis window length in samples.
    FRAME_STEP         = 128     # STFT hop size in samples (i.e. how far
                                  # the analysis window advances between
                                  # frames). Together with FRAME_LENGTH this
                                  # determines NUM_FRAMES (time resolution
                                  # of the spectrogram).
    WANTED_COMMANDS    = ['down', 'go', 'left', 'no', 'right', 'stop', 'up', 'yes']
                                  # The 8 target classes, in a fixed order.
                                  # This order becomes the output index
                                  # order of the model (index 0 = 'down',
                                  # etc.) - the firmware must map output
                                  # indices back to commands using this
                                  # exact list.
    BATCH_SIZE         = 64      # Training/validation batch size. Only
                                  # affects training speed/memory on the PC,
                                  # not the exported model.

    # =========================================================================
    # LOAD DATASET
    #
    # Expects a directory structure of data/full_speech_commands/<label>/*.wav
    # where each subfolder name matches an entry in WANTED_COMMANDS.
    # =========================================================================
    data_dir = pathlib.Path(args.data_dir)
    
    def download_dataset(target_dir):
        target_dir = pathlib.Path(target_dir)
        if not target_dir.exists():
            print(f"Downloading dataset to {target_dir}...")
            tf.keras.utils.get_file(
                'mini_speech_commands.zip',
                origin="http://storage.googleapis.com/download.tensorflow.org/data/mini_speech_commands.zip",
                extract=True,
                cache_dir='.', cache_subdir=str(target_dir)
            )
            print("Download and extraction complete.")
        else:
            print(f"Dataset already exists at {target_dir}. Skipping download.")
    download_dataset(data_dir)
    data_dir = data_dir / "mini_speech_commands_extracted" / "mini_speech_commands"


    train_ds, val_ds = tf.keras.utils.audio_dataset_from_directory(
        directory=data_dir,
        batch_size=BATCH_SIZE,
        validation_split=0.2,        # 80/20 train/validation split
        seed=0,                      # Fixed seed for reproducible split
        output_sequence_length=AUDIO_SAMPLES,   # Pads/truncates every clip
                                                 # to exactly AUDIO_SAMPLES
                                                 # samples so all inputs have
                                                 # a uniform shape.
        subset="both",                # Returns (train_ds, val_ds) in one call
        class_names=WANTED_COMMANDS   # Restricts loading to only the 8
                                       # wanted classes and fixes their
                                       # label order.
    )

    label_names = np.array(train_ds.class_names)
    print("Labels:", label_names)

    # =========================================================================
    # AUDIO -> SPECTROGRAM
    #
    # IMPORTANT: This function only ever runs on the PC during training/
    # conversion. On the MCU, the *same* mathematical steps must be
    # reimplemented in C (see audio_input.h) because the TFLite model
    # itself contains no STFT/FFT op - it only accepts a ready-made
    # spectrogram tensor as input. Any mismatch between this function and
    # the C implementation (window size, hop, mel matrix, normalization,
    # etc.) will silently degrade accuracy at inference time since there
    # is no error, just wrong predictions.
    # =========================================================================
    def get_spectrogram(waveform, label):
        # Remove the trailing channel dimension added by the audio loader
        # (mono audio comes in as shape (samples, 1)).
        waveform = tf.squeeze(waveform, axis=-1)
        waveform = tf.cast(waveform, tf.float32)

        # Short-Time Fourier Transform: slides a window of FRAME_LENGTH
        # samples across the waveform in steps of FRAME_STEP samples,
        # producing one complex spectrum per frame.
        stft = tf.signal.stft(
            waveform,
            frame_length=FRAME_LENGTH,
            frame_step=FRAME_STEP
        )
        # Magnitude spectrogram: discard phase information, keep only
        # the magnitude of each frequency bin per frame -> shape
        # (time_frames, freq_bins).
        spectrogram = tf.abs(stft)

        # Mel filterbank: projects the linear-frequency spectrogram onto
        # NUM_MEL_BINS perceptually-spaced (mel-scale) bands. This both
        # reduces dimensionality and better matches how speech
        # discriminative information is distributed across frequency.
        num_spectrogram_bins = spectrogram.shape[-1]
        mel_matrix = tf.signal.linear_to_mel_weight_matrix(
            num_mel_bins=NUM_MEL_BINS,
            num_spectrogram_bins=num_spectrogram_bins,
            sample_rate=SAMPLE_RATE,
            lower_edge_hertz=20.0,     # Lowest frequency considered relevant
            upper_edge_hertz=8000.0    # Highest frequency considered relevant
                                        # (Nyquist limit for 16 kHz audio)
        )
        mel = tf.tensordot(spectrogram, mel_matrix, 1)  # -> (time, NUM_MEL_BINS)

        # Log compression: converts the wide dynamic range of raw
        # magnitudes into a more compact, roughly logarithmic scale that
        # matches human loudness perception and is more stable to train
        # on than raw linear magnitudes. A small epsilon (1e-6) avoids
        # log(0).
        mel = tf.math.log(mel + 1e-6)

        # Per-sample min-max normalization to [-1, 1]. This is done
        # BEFORE quantization so that the float range the model is
        # trained on matches the float range TFLite's INT8 quantizer will
        # calibrate against. The C implementation on the MCU must apply
        # the same normalization strategy before feeding data to the
        # model, otherwise the INT8 input values will be scaled
        # incorrectly.
        mel_min = tf.reduce_min(mel)
        mel_max = tf.reduce_max(mel)
        mel = (mel - mel_min) / (mel_max - mel_min + 1e-6)
        mel = mel * 2.0 - 1.0

        # Add a channel dimension so the tensor has the (height, width,
        # channels) shape a Conv2D/DepthwiseConv2D layer expects.
        mel = mel[..., tf.newaxis]  # -> (time, NUM_MEL_BINS, 1)

        return mel, label

    train_ds = train_ds.map(get_spectrogram, num_parallel_calls=tf.data.AUTOTUNE)
    val_ds   = val_ds.map(get_spectrogram,   num_parallel_calls=tf.data.AUTOTUNE)

    # Cache the (expensive) spectrogram computation in memory after the
    # first epoch, shuffle training data each epoch, and prefetch batches
    # so the GPU/CPU is never waiting on I/O.
    train_ds = train_ds.cache().shuffle(10000).prefetch(tf.data.AUTOTUNE)
    val_ds   = val_ds.cache().prefetch(tf.data.AUTOTUNE)

    # Determine the model's input shape directly from one real batch,
    # rather than computing it by hand, to avoid off-by-one errors from
    # the STFT framing math.
    for spec, _ in train_ds.take(1):
        input_shape = spec.shape[1:]
    print("Input shape:", input_shape)
    # Expected: (time_frames, NUM_MEL_BINS, 1), e.g. roughly (122, 32, 1)
    # depending on AUDIO_SAMPLES/FRAME_LENGTH/FRAME_STEP.

    # =========================================================================
    # MODEL ARCHITECTURE
    #
    # Comparison to a non-MCU-optimized baseline:
    #   Baseline:  Conv(32) -> Conv(64) -> Conv(128) -> Dense(256) -> Dense(8)
    #   This model: DWConv(16) -> DWConv(32) -> Dense(64) -> Dense(8)
    #
    # Each DepthwiseConv2D + pointwise Conv2D(1x1) pair approximates a
    # regular Conv2D but factorizes it into a per-channel spatial filter
    # (depthwise) followed by a 1x1 filter that mixes channels
    # (pointwise). This factorization is the same trick used by
    # MobileNet and typically needs ~8-10x fewer multiply-accumulate
    # operations than an equivalent standard Conv2D, at a small cost in
    # representational power - an acceptable trade-off for a keyword
    # spotting task with only 8 classes.
    # =========================================================================
    model = models.Sequential([
        layers.Input(shape=input_shape),

        # Block 1: low-level spectral features (e.g. edges/transitions
        # in time and frequency).
        layers.DepthwiseConv2D(3, activation='relu', padding='same'),
        layers.Conv2D(16, 1, activation='relu'),    # Pointwise: mixes channels
        layers.MaxPooling2D(2),                     # Downsamples time/freq by 2x
        layers.BatchNormalization(),                # Stabilizes/speeds up training

        # Block 2: higher-level, more abstract features built from
        # Block 1's outputs.
        layers.DepthwiseConv2D(3, activation='relu', padding='same'),
        layers.Conv2D(32, 1, activation='relu'),    # Pointwise
        layers.MaxPooling2D(2),
        layers.BatchNormalization(),
        layers.Dropout(0.25),                       # Regularization to reduce overfitting

        # Classification head.
        layers.GlobalAveragePooling2D(),            # Collapses spatial dims to a
                                                      # single vector per channel;
                                                      # more robust and far cheaper
                                                      # than Flatten() + a large
                                                      # Dense layer, since it has
                                                      # no learnable parameters.
        layers.Dense(64, activation='relu'),
        layers.Dropout(0.3),
        layers.Dense(len(label_names))               # One logit per command class
                                                       # (no softmax here - logits
                                                       # are used directly with
                                                       # from_logits=True below).
    ])

    model.summary()

    model.compile(
        optimizer=tf.keras.optimizers.Adam(0.001),
        # from_logits=True because the final Dense layer has no
        # activation function - the loss applies softmax internally,
        # which is numerically more stable than doing it manually.
        loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
        metrics=['accuracy']
    )

    # =========================================================================
    # TRAINING
    # =========================================================================
    print("Training starting...")

    history = model.fit(
        train_ds,
        validation_data=val_ds,
        epochs=40,
        callbacks=[
            # Stops training once validation performance stops improving
            # for 6 consecutive epochs, and restores the weights from the
            # best epoch rather than the last one - guards against
            # overfitting and wasted compute.
            tf.keras.callbacks.EarlyStopping(
                patience=6,
                restore_best_weights=True,
                verbose=1
            ),
            # Halves the learning rate if validation performance plateaus
            # for 3 epochs, allowing finer convergence later in training.
            tf.keras.callbacks.ReduceLROnPlateau(
                factor=0.5,
                patience=3,
                verbose=1
            )
        ]
    )

    # Report the best validation accuracy seen across all epochs (not
    # necessarily the last one, since EarlyStopping may have restored an
    # earlier checkpoint).
    val_acc = max(history.history['val_accuracy'])
    print(f"\nBest validation accuracy: {val_acc:.1%}")

    # =========================================================================
    # SAVE KERAS MODEL
    #
    # Kept as a normal float32 Keras model for reference/debugging and as
    # a starting point if retraining or fine-tuning is needed later. This
    # is NOT the file that runs on the MCU.
    # =========================================================================
    model.save("speech_model_mcu.h5")
    print("Keras model saved: speech_model_mcu.h5")

    # =========================================================================
    # TFLITE CONVERSION - fully INT8
    #
    # Goal: every operation in the exported graph runs in INT8, with no
    # float32 fallback anywhere. On a RISC-V core without a hardware FPU,
    # float32 operations are roughly 10-50x slower than INT8 fixed-point
    # operations, so even a single float32 op left in the graph could
    # make real-time inference infeasible at 25 MHz.
    # =========================================================================
    print("\nConverting to TFLite INT8...")

    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]

    # Representative dataset: TFLite needs real example inputs to
    # calibrate the INT8 quantization ranges (min/max) for every
    # activation in the network. Using too few or unrepresentative
    # samples here leads to poor quantization and a noticeable accuracy
    # drop compared to the float32 model. Using more batches generally
    # improves quantization quality at the cost of longer conversion
    # time.
    def representative_dataset():
        count = 0
        for audio_batch, _ in train_ds.take(100):
            for i in range(audio_batch.shape[0]):
                sample = tf.expand_dims(audio_batch[i], axis=0)
                yield [sample]
                count += 1
                if count >= 200:
                    return

    converter.representative_dataset = representative_dataset

    # Restrict the converter to INT8-only builtin ops. If any op in the
    # graph cannot be represented in INT8, conversion will fail loudly
    # here instead of silently producing a model with a hidden float32
    # fallback op.
    converter.target_spec.supported_ops = [
        tf.lite.OpsSet.TFLITE_BUILTINS_INT8
    ]
    converter.inference_input_type  = tf.int8   # Model expects INT8 input directly
                                                  # (no float->int8 conversion op
                                                  # needed at the model boundary,
                                                  # since the MCU produces INT8
                                                  # spectrogram data itself).
    converter.inference_output_type = tf.int8   # Model produces INT8 logits;
                                                  # firmware must dequantize or
                                                  # simply take the argmax, since
                                                  # argmax is scale-invariant.

    tflite_model = converter.convert()

    # Write the raw .tflite file (useful for testing with the TFLite
    # Python interpreter or tools like Netron before deploying).
    tflite_path = args.output
    with open(tflite_path, "wb") as f:
        f.write(tflite_model)

    size_kb = len(tflite_model) / 1024
    print(f"TFLite model saved: {tflite_path}")
    print(f"Model size: {size_kb:.1f} KB")

    # =========================================================================
    # GENERATE C ARRAY (voice_model.h)
    #
    # Embeds the TFLite flatbuffer directly as a C byte array so it can be
    # linked into the firmware image and read straight from Flash - no
    # filesystem needed on the MCU, and no external xxd/bin2c step
    # required since this is done directly in Python.
    # =========================================================================
    print("\nGenerating voice_model.h...")

    array_str = ", ".join(f"0x{b:02x}" for b in tflite_model)
    header = f"""/*
 * voice_model.h
 * Auto-generated - do not edit by hand.
 * Model size: {size_kb:.1f} KB
 * Labels: {list(label_names)}
 *
 * Label order (output index -> command):
{chr(10).join(f" *   {i}  {name}" for i, name in enumerate(label_names))}
 */

#pragma once
#include <stdint.h>

const unsigned char g_voice_model[] = {{
  {array_str}
}};
const unsigned int g_voice_model_len = {len(tflite_model)};
"""

    with open("voice_model.h", "w") as f:
        f.write(header)

    print("voice_model.h generated - copy directly into voice/")

    # =========================================================================
    # GENERATE audio_input.h TEMPLATE
    #
    # Documents exactly how the spectrogram must be computed on the MCU
    # so its output matches the format the model was trained on. This is
    # a TEMPLATE only - audio_get_frame() itself still needs a real
    # fixed-point implementation of STFT + Mel filterbank + log + INT8
    # scaling in C; only the constants and function signatures are
    # generated automatically here.
    # =========================================================================
    audio_input_template = f"""/*
 * voice/audio_input.h
 *
 * Microphone driver for piconut.
 * Computes the same spectrogram as the Python training script.
 *
 * Parameters (MUST match training exactly):
 *   Sample rate   : {SAMPLE_RATE} Hz
 *   Window length : {AUDIO_SAMPLES} samples ({AUDIO_LENGTH_MS} ms)
 *   STFT window   : {FRAME_LENGTH} samples
 *   STFT step     : {FRAME_STEP} samples
 *   Mel bins      : {NUM_MEL_BINS}
 *   Input shape   : {input_shape}
 */

#pragma once
#include <stdint.h>

/* Spectrogram dimensions (from training) */
#define AUDIO_SAMPLE_RATE    {SAMPLE_RATE}
#define AUDIO_WINDOW_SAMPLES {AUDIO_SAMPLES}
#define MEL_BINS             {NUM_MEL_BINS}
#define STFT_FRAME_LEN       {FRAME_LENGTH}
#define STFT_FRAME_STEP      {FRAME_STEP}

/* Number of time frames: (AUDIO_WINDOW_SAMPLES - STFT_FRAME_LEN) / STFT_FRAME_STEP + 1 */
#define NUM_FRAMES           ((AUDIO_WINDOW_SAMPLES - STFT_FRAME_LEN) / STFT_FRAME_STEP + 1)

/* Starts ADC sampling and background buffering (interrupt/DMA driven) */
void audio_init(void);

/* Returns 1 once a new window has been fully captured */
int audio_frame_ready(void);

/*
 * Fills buf with the precomputed Mel spectrogram as INT8.
 * n_samples = NUM_FRAMES * MEL_BINS
 *
 * Steps that must be implemented here, in this exact order, to match
 * the training pipeline:
 *   1. Compute the STFT over AUDIO_WINDOW_SAMPLES samples
 *      (frame length STFT_FRAME_LEN, hop STFT_FRAME_STEP).
 *   2. Take the magnitude of each complex STFT bin.
 *   3. Apply the Mel filterbank to reduce to MEL_BINS outputs per frame.
 *   4. Apply log(x + 1e-6) compression.
 *   5. Normalize the full frame to the range [-1.0, 1.0] using
 *      min-max normalization (must match the Python training script's
 *      normalization, not a fixed/global scale).
 *   6. Quantize to INT8: int8 = round(float_value * 127).
 *
 * Any deviation from these steps (different window function, different
 * mel matrix, skipped normalization, etc.) will produce inputs the
 * model was never trained on and silently degrade or break recognition
 * accuracy - there will be no crash, only wrong predictions.
 */
void audio_get_frame(int8_t *buf, int n_samples);
"""

    with open("audio_input_template.h", "w") as f:
        f.write(audio_input_template)

    print("audio_input_template.h generated - use as a template for audio_input.h")

    print("\n=== DONE ===")
    print(f"Model size:        {size_kb:.1f} KB")
    print(f"Val accuracy:      {val_acc:.1%}")
    print("\nProject files:")
    print("  speech_model_mcu.tflite  -> for testing")
    print("  voice_model.h            -> copy into voice/")
    print("  audio_input_template.h   -> template for audio_input.h")

except Exception as e:
    print("\nERROR:")
    print(e)
    traceback.print_exc()

