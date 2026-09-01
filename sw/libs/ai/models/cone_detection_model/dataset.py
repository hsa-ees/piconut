#!/usr/bin/env python3

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
import random
import shutil
import tarfile
import tempfile
from pathlib import Path


def copy_pairs(pairs, images_out, labels_out):
    images_out.mkdir(parents=True, exist_ok=True)
    labels_out.mkdir(parents=True, exist_ok=True)

    for image, label in pairs:
        shutil.copy2(image, images_out / image.name)
        new_label_name = image.stem + ".txt"
        shutil.copy2(label, labels_out / new_label_name)


def main():
    parser = argparse.ArgumentParser(
        description="Extract, shuffle, split and prepare an image/label dataset."
    )

    parser.add_argument(
        "archive",
        type=Path,
        help="Path to the input .tar.gz archive",
    )

    parser.add_argument(
        "output",
        type=Path,
        help="Output directory",
    )

    parser.add_argument(
        "--train-ratio",
        type=float,
        default=0.8,
        help="Fraction of samples used for training (default: 0.8)",
    )

    parser.add_argument(
        "--calibration-ratio",
        type=float,
        default=0.1,
        help="Fraction of each split copied into the calibration dataset (default: 0.1)",
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Random seed (default: 42)",
    )

    args = parser.parse_args()

    if not args.archive.exists():
        parser.error(f"Archive does not exist: {args.archive}")

    if not (0 < args.train_ratio < 1):
        parser.error("--train-ratio must be between 0 and 1")

    if not (0 < args.calibration_ratio <= 1):
        parser.error("--calibration-ratio must be between 0 and 1")

    random.seed(args.seed)

    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)

        print(f"Extracting {args.archive}...")
        with tarfile.open(args.archive, "r:gz") as tar:
            tar.extractall(tmpdir)

        images_dir = tmpdir / "yolo_dataset" / "images"
        labels_dir = tmpdir / "yolo_dataset" / "labels"

        if not images_dir.exists():
            raise RuntimeError("Archive does not contain an 'images/' directory.")

        if not labels_dir.exists():
            raise RuntimeError("Archive does not contain a 'labels/' directory.")

        pairs = []

        for image in sorted(images_dir.iterdir()):
            if not image.is_file():
                continue

            label = labels_dir / f"{image.name}.txt"

            if label.exists():
                pairs.append((image, label))
            else:
                print(f"Warning: Missing label for {image.name}")

        if not pairs:
            raise RuntimeError("No image/label pairs found.")

        print(f"Found {len(pairs)} image/label pairs.")

        random.shuffle(pairs)

        split_idx = int(len(pairs) * args.train_ratio)

        train_pairs = pairs[:split_idx]
        val_pairs = pairs[split_idx:]

        splits = {
            "train": train_pairs,
            "val": val_pairs,
        }

        for split_name, split_pairs in splits.items():
            # Training dataset
            copy_pairs(
                split_pairs,
                args.output / "training" / split_name / "images",
                args.output / "training" / split_name / "labels",
            )

            # Calibration dataset (subset)
            calib_count = max(1, int(len(split_pairs) * args.calibration_ratio))
            calib_count = min(calib_count, len(split_pairs))

            calib_pairs = random.sample(split_pairs, calib_count)

            copy_pairs(
                calib_pairs,
                args.output / "calibration" / split_name / "images",
                args.output / "calibration" / split_name / "labels",
            )

        print()
        print("Done.")
        print(f"Training samples   : {len(train_pairs)}")
        print(f"Validation samples : {len(val_pairs)}")
        print(
            f"Calibration train  : {max(1, int(len(train_pairs) * args.calibration_ratio))}"
        )
        print(
            f"Calibration val    : {max(1, int(len(val_pairs) * args.calibration_ratio))}"
        )


if __name__ == "__main__":
    main()
