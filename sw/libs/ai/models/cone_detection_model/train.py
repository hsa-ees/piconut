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
from pathlib import Path
from ultralytics import YOLO

def main():
    parser = argparse.ArgumentParser(description="Train and/or Export a YOLO model to INT8 TFLite.")

    # Phase toggles
    parser.add_argument("--train", action="store_true", help="Enable the training phase")
    parser.add_argument("--export", action="store_true", help="Enable the export & INT8 calibration phase")

    # Working directory
    parser.add_argument("--project", type=str, help="Project directory")

    # Model and Data parameters
    parser.add_argument("--weights", type=str, default="yolo11n.pt", help="Path to initial weights (e.g., yolov8n.pt for training, or best.pt for export)")
    parser.add_argument("--data", type=str, required=True, help="Path to dataset.yaml (Required for both training and INT8 calibration, if --data-calib is unspecified)")
    parser.add_argument("--data_calib", type=str, required=False, help="Path to dataset.yaml (Required INT8 calibration)")
    parser.add_argument("--epochs", type=int, default=400, help="Number of training epochs (default:250)")
    parser.add_argument("--patience", type=int, default=100, help="Number of epochs with no improvement before stopping (default:50)")
    parser.add_argument("--imgsz", type=int, default=640, help="Image size for training and export (default: 640)")
    parser.add_argument("--output", type=str, default="cone_detection.tflite", help="OUtput .tflite file")

    args = parser.parse_args()

    # Ensure the user selected at least one phase
    if not args.train and not args.export:
        print("Please specify at least one action: --train and/or --export")
        parser.print_help()
        return

    model = None

    # --- TRAINING PHASE ---
    if args.train:
        print(f"\n{'='*50}\nStarting Training Phase\n{'='*50}")
        # Load the base model (e.g., yolov8n.pt)
        model = YOLO(args.weights)
        
        # Train the model
        model.train(
            data=args.data,
            epochs=args.epochs,
            patience=args.patience,
            imgsz=args.imgsz,
            project=args.project
        )
        print("Training completed. The model object now contains the best weights.")

    # --- EXPORT & QUANTIZATION PHASE ---
    if args.export:
        print(f"\n{'='*50}\nStarting Export & INT8 Quantization Phase\n{'='*50}")
        
        if model is None:
            if not os.path.exists(args.weights):
                print(f"Error: Model weights not found at '{args.weights}'. Please provide a valid path using --weights.")
                return
            model = YOLO(args.weights)
            print(f"Loaded weights from {args.weights}")

        print("Exporting to TFLite INT8 format. This may take a few minutes for calibration...")
        
        
        if args.data_calib is None:
            data = args.data
        else:
            data = args.data_calib

        model.model.eval() 
        # TBD: Checkout end2end option for YOLO26n model.
        export_path = model.export(
            format="tflite",
            quantize=8,
            data=data,
            imgsz=args.imgsz,
            embed=True, # needed by YOLO11n,
        )
        
        # Move quantized model to specified output path
 #       export_dir = Path(export_path) if os.path.isdir(export_path) else Path(export_path).parent
 #       full_quant_files = list(export_dir.glob("*_full_integer_quant.tflite"))
 #       
 #       if full_quant_files:
 #           source_file = full_quant_files[0]
 #       else:
 #           print("Warning: Could not find *_full_integer_quant.tflite. Falling back to export_path.")
 #           source_file = Path(export_path)
 #       
 #       # 2. Move the full INT8 model to specified output path
 #       if args.output and source_file.resolve() != Path(args.output).resolve():
 #           os.makedirs(os.path.dirname(os.path.abspath(args.output)) or ".", exist_ok=True)
 #           os.replace(source_file, args.output)
 #           export_path = args.output
 #       else:
 #           export_path = str(source_file)

        print(f"\nExport successful! TFLite model saved at: {export_path}")

if __name__ == "__main__":
    main()
