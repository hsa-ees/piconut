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

#include <cstdint>
#include <cstdio>
#include <cstring>

// LiteRT Micro
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

// Model
#include <llm_model_data.h>
#include "llm_data/llm_input_data.h"

constexpr size_t tensor_arena_size = 32 * 1024 * 1024;
alignas(16) uint8_t tensor_arena[tensor_arena_size];

constexpr int SEQ_LEN = 64;
constexpr int MAX_NEW_TOKENS = 32;

// OPT-125M end-of-sequence token id (</s>).
constexpr int32_t EOS_TOKEN_ID = 2;

//~ constexpr int MAX_TOTAL_TOKENS = llm_input_data_len[0] + MAX_NEW_TOKENS;
constexpr int MAX_TOTAL_TOKENS = 6 + MAX_NEW_TOKENS;
int32_t generated[MAX_TOTAL_TOKENS];
int generated_len = 0;

void set_input_tensors(TfLiteTensor* token_input, TfLiteTensor* mask_input, int* out_valid_length)
{
    int start = generated_len > SEQ_LEN ? generated_len - SEQ_LEN : 0;
    int valid_length = generated_len - start;

    int32_t* token_data = token_input->data.i32;
    bool* mask_data = mask_input->data.b;

    memset(token_data, 0, SEQ_LEN * sizeof(int32_t));
    memset(mask_data, 0, SEQ_LEN * sizeof(bool));

    for(int i = 0; i < valid_length; i++)
    {
        token_data[i] = generated[start + i];
        mask_data[i] = true;
    }

    *out_valid_length = valid_length;
}

int32_t argmax_next_token(const TfLiteTensor* output, int valid_length)
{
    // output shape is expected to be [1, SEQ_LEN, VOCAB_SIZE]
    const int vocab_size = output->dims->data[2];
    const float* logits = output->data.f + (valid_length - 1) * vocab_size;

    int32_t best_index = 0;
    float best_value = logits[0];

    for(int v = 1; v < vocab_size; v++)
    {
        if(logits[v] > best_value)
        {
            best_value = logits[v];
            best_index = v;
        }
    }

    return best_index;
}

int run_test()
{
    tflite::MicroErrorReporter micro_error_reporter;
    tflite::ErrorReporter* error_reporter = &micro_error_reporter;

    const tflite::Model* model = tflite::GetModel(llm_model_data);

    static tflite::MicroMutableOpResolver<19> resolver;
    resolver.AddCast();
    resolver.AddReshape();
    resolver.AddMinimum();
    resolver.AddGreater();
    resolver.AddLess();
    resolver.AddAdd();
    resolver.AddSelectV2();
    resolver.AddGather();
    resolver.AddMean();
    resolver.AddDequantize();
    resolver.AddNeg();
    resolver.AddQuantize();
    resolver.AddSquaredDifference();
    resolver.AddRsqrt();
    resolver.AddMul();
    resolver.AddFullyConnected();
    resolver.AddTranspose();
    resolver.AddBatchMatMul();
    resolver.AddSoftmax();

    tflite::MicroInterpreter interpreter(model, resolver, tensor_arena, tensor_arena_size);

    if(interpreter.AllocateTensors() != kTfLiteOk)
    {
        TF_LITE_REPORT_ERROR(error_reporter, "ERROR: In AllocateTensors().");
        return -1;
    }

    TfLiteTensor* token_input = interpreter.input(0);
    TfLiteTensor* mask_input = interpreter.input(1);
    TfLiteTensor* output = interpreter.output(0);

    generated_len = llm_input_data_len[0];
    memcpy(generated, llm_input_data[0], llm_input_data_len[0] * sizeof(int32_t));

    printf("Initial token count: %d\n", generated_len);

    for(int step = 0; step < MAX_NEW_TOKENS; step++)
    {
        int valid_length = 0;
        set_input_tensors(token_input, mask_input, &valid_length);

        if(interpreter.Invoke() != kTfLiteOk)
        {
            TF_LITE_REPORT_ERROR(error_reporter, "ERROR: In Invoke().");
            return -1;
        }

        int32_t next_token = argmax_next_token(output, valid_length);

        if(generated_len >= MAX_TOTAL_TOKENS)
        {
            printf("\nToken buffer full, stopping.\n");
            break;
        }
        generated[generated_len++] = next_token;

        printf("\nStep %d\n", step + 1);
        printf("Next token: %ld\n", (long)next_token);
        printf("Tokens so far: ");
        for(int i = 0; i < generated_len; i++)
        {
            printf("%ld ", (long)generated[i]);
        }
        printf("\n");

        if(next_token == EOS_TOKEN_ID)
        {
            printf("\nEOS reached.\n");
            break;
        }
    }

    printf("\n==============================\n");
    printf("Final output token ids:\n");
    for(int i = 0; i < generated_len; i++)
    {
        printf("%ld ", (long)generated[i]);
    }
    printf("\n");

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
