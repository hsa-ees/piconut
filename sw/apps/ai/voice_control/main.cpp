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
#include <ctype.h>

// LiteRT Micro
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

// Model
#include <voice_control_model_data.h>
#include <voice_control_model_settings.h>
#include "voice_control_data/voice_control_input_data.h"

// Ideal peak activation memory needed: 83328 Byte
constexpr size_t tensor_arena_size = 85 * 1024;
alignas(16) uint8_t tensor_arena[tensor_arena_size];

int run_test()
{
    tflite::MicroErrorReporter micro_error_reporter;
    tflite::ErrorReporter* error_reporter = &micro_error_reporter;

    const tflite::Model* model = tflite::GetModel(voice_control_model_data);

    static tflite::MicroMutableOpResolver<8> resolver;
    resolver.AddFullyConnected();
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddReshape();
    resolver.AddMaxPool2D();
    resolver.AddMul();
    resolver.AddAdd();
    resolver.AddMean();

    tflite::MicroInterpreter interpreter(model, resolver, tensor_arena, tensor_arena_size);

    if(interpreter.AllocateTensors() != kTfLiteOk)
    {
        TF_LITE_REPORT_ERROR(error_reporter, "ERROR: In AllocateTensors().");
        return -1;
    }

    memcpy(interpreter.input(0)->data.int8, voice_control_input_data[0], voice_control_input_data_len[0]);

    if(interpreter.Invoke() != kTfLiteOk)
    {
        TF_LITE_REPORT_ERROR(error_reporter, "ERROR: In Invoke().");
        return -1;
    }

    TfLiteTensor* output_tensor = interpreter.output(0);

    int8_t* output_data = (int8_t*)output_tensor->data.int8;

    if(output_data == nullptr)
    {
        TF_LITE_REPORT_ERROR(error_reporter, "ERROR: Output data is nullptr.");
        return -1;
    }

    int8_t top_index = 0;
    for(size_t j = 0; j < voice_control_model_label_cnt; j++)
    {
        if(output_tensor->data.int8[j] > output_tensor->data.int8[top_index])
        {
            top_index = j;
        }
    }

    printf("Top: %s\n\n\n", voice_control_model_labels[top_index]);
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
