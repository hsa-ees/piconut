/*************************************************************************

  This file is part of the PicoNut project.

  Copyright (C) 2026 Johannes Hofmann <johannes.hofmann1@tha.de>
      Technische Hochschule Augsburg, Technical University of Applied Sciences Augsburg


  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// LiteRT Micro
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

// Model
#include <cone_detection_model_data.h>
#include <cone_detection_model_settings.h>
#include "cone_detection_data/cone_detection_input_data.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

constexpr size_t tensor_arena_size = 8 * 1024 * 1024; // 16 MiB
alignas(16) uint8_t tensor_arena[tensor_arena_size];

int run_test()
{
    tflite::MicroErrorReporter micro_error_reporter;
    tflite::ErrorReporter* error_reporter = &micro_error_reporter;

    const tflite::Model* model = tflite::GetModel(cone_detection_model_data);

    static tflite::MicroMutableOpResolver<20> resolver;
    resolver.AddFullyConnected();
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddAveragePool2D();
    resolver.AddReshape();
    resolver.AddSoftmax();
    resolver.AddPad();
    resolver.AddLogistic();
    resolver.AddMul();
    resolver.AddStridedSlice();
    resolver.AddAdd();
    resolver.AddConcatenation();
    resolver.AddMaxPool2D();
    resolver.AddTranspose();
    resolver.AddBatchMatMul();
    resolver.AddResizeNearestNeighbor();
    resolver.AddQuantize();
    resolver.AddSub();
    resolver.AddReduceMax();
    resolver.AddDequantize();

    tflite::MicroInterpreter interpreter(model, resolver, tensor_arena, tensor_arena_size);

    if(interpreter.AllocateTensors() != kTfLiteOk)
    {
        TF_LITE_REPORT_ERROR(error_reporter, "ERROR: In AllocateTensors().");
        return -1;
    }

    memcpy(interpreter.input(0)->data.int8, (int8_t*)cone_detection_input_data[0], cone_detection_input_data_len[0]);

    if(interpreter.Invoke() != kTfLiteOk)
    {
        TF_LITE_REPORT_ERROR(error_reporter, "ERROR: In Invoke().");
        return -1;
    }

    TfLiteTensor* output_tensor = interpreter.output(0);

    int8_t* output_data = (int8_t*)output_tensor->data.int8;

    if(output_data == nullptr)
    {
        return -1;
    }

    // Output
    const int num_anchors = 8400;
    const int num_classes = 5;
    const int8_t threshold = 51;

    // Output mapping
    const int zero_point = -119;
    const int32_t box_multiplier = 170130;

    // Loop through all 8400 geographical anchor locations
    for(int i = 0; i < num_anchors; ++i)
    {
        int8_t max_score = -128;
        int max_class_id = -1;

        // Check channels 4 to 8 to find the highest class confidence
        for(int c = 0; c < num_classes; ++c)
        {
            int memory_index = (c + 4) * num_anchors + i;
            int8_t score = output_data[memory_index];

            if(score > max_score)
            {
                max_score = score;
                max_class_id = c;
            }
        }

        // Print immediately if it clears your int8 threshold
        if(max_score > threshold)
        {
            int32_t raw_cx = output_data[0 * num_anchors + i];
            int32_t raw_cy = output_data[1 * num_anchors + i];
            int32_t raw_w = output_data[2 * num_anchors + i];
            int32_t raw_h = output_data[3 * num_anchors + i];

            // 1. Convert raw int8 straight to 640-space pixels using fixed-point math (>> 16)
            int32_t cx = ((raw_cx - zero_point) * box_multiplier) >> 16;
            int32_t cy = ((raw_cy - zero_point) * box_multiplier) >> 16;
            int32_t w = ((raw_w - zero_point) * box_multiplier) >> 16;
            int32_t h = ((raw_h - zero_point) * box_multiplier) >> 16;

            // 2. Convert Center [cx, cy, w, h] to Corners [x1, y1, x2, y2]
            int32_t x1 = cx - (w / 2);
            int32_t y1 = cy - (h / 2);
            int32_t x2 = cx + (w / 2);
            int32_t y2 = cy + (h / 2);

            // 3. Clamp coordinates so they stay inside the 640x640 boundaries
            x1 = MAX(0, MIN(x1, 640));
            y1 = MAX(0, MIN(y1, 640));
            x2 = MAX(0, MIN(x2, 640));
            y2 = MAX(0, MIN(y2, 640));

            printf("Class: %d | Conf: %d | Pixel Box: [%d, %d, %d, %d]\n",
                max_class_id,
                max_score,
                x1,
                y1,
                x2,
                y2);
        }
    }
    return 0;
}

int main(int argc, char* argv[])
{
    int ret = run_test();
    if(ret != 0)
    {
        printf("Test Failed!\n");
    }
    else
    {
        printf("Test Success!\n");
    }

    return ret;
}
