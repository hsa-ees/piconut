import numpy as np
import ai_edge_litert.interpreter as tflite
import keras_hub

# ============================================================
# Configuration
# ============================================================

MODEL_PATH = "opt_125m_en.tflite"
PRESET = "opt_125m_en"

SEQ_LEN = 64
MAX_NEW_TOKENS = 32

# ============================================================
# Load tokenizer
# ============================================================

tokenizer = keras_hub.models.OPTTokenizer.from_preset(PRESET)

# ============================================================
# Load TFLite model
# ============================================================

print("Loading model...")

interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

TOKEN_INPUT = input_details[0]["index"]
MASK_INPUT = input_details[1]["index"]
OUTPUT = output_details[0]["index"]

# ============================================================
# Tokenize prompt
# ============================================================

prompt = "The university of applied sciences is"

tokenized = tokenizer([prompt])

if hasattr(tokenized, "numpy"):
    token_ids = tokenized.numpy().astype(np.int32)
else:
    token_ids = np.asarray(tokenized, dtype=np.int32)

generated = token_ids[0].tolist()

print("Initial token count:", len(generated))

c_array_elements = ", ".join(str(token_id) for token_id in generated)
print("\n/* Copy-pasteable C array of input tokens */")
print(f"int input_tokens[] = {{{c_array_elements}}};")
print("---------------------------------------------\n")

# ============================================================
# Generation loop
# ============================================================

for step in range(MAX_NEW_TOKENS):

    # Keep only last SEQ_LEN tokens
    context = generated[-SEQ_LEN:]

    valid_length = len(context)

    # Pad
    padded = np.zeros((1, SEQ_LEN), dtype=np.int32)
    padded[0, :valid_length] = context

    # Padding mask
    padding_mask = np.zeros((1, SEQ_LEN), dtype=np.bool_)
    padding_mask[0, :valid_length] = True

    # Run inference
    interpreter.set_tensor(TOKEN_INPUT, padded)
    interpreter.set_tensor(MASK_INPUT, padding_mask)

    interpreter.invoke()

    logits = interpreter.get_tensor(OUTPUT)

    # Next-token logits
    next_logits = logits[0, valid_length - 1]

    # Greedy decoding
    next_token = int(np.argmax(next_logits))

    generated.append(next_token)

    # Decode current text
    current = np.array([generated], dtype=np.int32)

    decoded = tokenizer.detokenize(current)

    if hasattr(decoded, "numpy"):
        text = decoded.numpy()[0].decode("utf-8")
    else:
        text = str(decoded)

    print(f"\nStep {step+1}")
    print("Next token:", next_token)
    print(text)

    # Stop at EOS if tokenizer exposes one
    eos = getattr(tokenizer, "end_token_id", None)

    if eos is not None and next_token == eos:
        print("\nEOS reached.")
        break

print("\n==============================")
print("Final output:")
print(text)
