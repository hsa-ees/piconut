
# AI applications for embedded systems

## Introduction

### Tensorflow Lite Micro

TensorFlow Lite for Microcontrollers [1] is a port of TensorFlow Lite designed to
run machine learning models on DSPs, microcontrollers and other devices with limited memory.




### muRISCV-NN

muRISCV-NN [2, 3] is a collection of efficient deep learning kernels for embedded
platforms and microcontrollers. It is based on ARM's CMSIS-NN library but targets
the RISC-V ISA instead.

It offers accelerated kernels using the RISC-V "V" vector extension v1.0, and the
RISC-V packed "P" extension v0.9.6.



## Applications

### MLPerf® tiny: AWW

AWW is a model from MLPerf® Tiny Benchmark [4].



### Voice Control

Keyword spotting for basic control commands.

The model is based on the TensorFlow Simple Audio Recognition tutorial [5] with several modifications.

The input consists of audio features extracted from the input audio stream.

The output is one of the following voice command classes: "down", "go", "left", "no", "right", "stop", "up", and "yes".



### Cone Detection

Object detection and localization of colored traffic cones.

The model is YOLO11n [6]. It was trained on the FSOCO dataset [7] and quantized for integer inference.

The input is a 640 × 640 RGB image.

The output consists of bounding boxes and one of the following classes: "blue", "yellow", "orange", "large_orange", or "unknown".


The model was trained using the fsoco dataset. To simplify the integration a ready to use version for YOLO is provided.
To get the YOLO compatible version of the dataset these steps where done:

1. Download the FSOCO dataset (~24GB): 
```
http://fsoco.cs.uni-freiburg.de/src/download_boxes_train.php
```

2. Clone the fsoco-devkit repository: 
```
$ git clone https://github.com/fsoco/fsoco-devkit.git
```

3. Install the tool according to `fsoco-devkit/tools/README.md`

4. Run the converter:
```
$ fsoco label-converter sly2yolo -p project_name -d dataset_name img/ yolo_labels/
```


### ECG Classification

Classification of 12-lead ECG recordings into nine diagnostic classes.

The model is available at https://github.com/JoHofmann/piconut_ecg-diagnosis. It was developed and trained using PyTorch and converted to TensorFlow Lite using the LiteRT-Torch project.

The input consists of ECG recordings from all 12 leads.

The output consists of prediction scores for the following classes: "SNR", "AF", "IAVB", "LBBB", "RBBB", "PAC", "PVC", "STD", and "STE".




### LLM

Text generation using a decoder-only large language model.

The model is OPT-125M [8].


The input consists of token IDs and an attention mask.

The output consists of the predicted logits for the next token.

To run this model, the `SELECT_V2` operator of tensorflow lite micro must be modified to support `int32` inputs. This is required by the select layer that filters the input IDs using the attention mask.

The following modification was made to `tensorflow/lite/micro/kernels/select.cc`:

```cpp
case kTfLiteInt32:
  CallSelect<int32_t>(input_condition, input_x, input_y, output,
                      data->requires_broadcast);
  break;
```




## Results

TBD




## Useful Tools

#### TFLite Model Explorer

https://github.com/google-ai-edge/model-explorer

A graphical tool for inspecting TensorFlow Lite models. It provides an overview of the model graph, operators, and tensor connections, and allows you to inspect model inputs and outputs.

#### TFLite Model Visualizer

https://github.com/tensorflow/tflite-micro/blob/main/tensorflow/lite/tools/visualize.py

Generate an HTML visualization of a TensorFlow Lite model:

```bash
python -m tensorflow.lite.tools.visualize model.tflite index.html
```

The generated HTML file provides a layer-by-layer overview of the model, making it useful for identifying the operators required for TensorFlow Lite Micro.




## References

[1] TensorFlow Lite for Microcontrollers, [https://github.com/tensorflow/tflite-micro](https://github.com/tensorflow/tflite-micro)

[2] Philipp van Kempen, Jefferson Parker Jones, Daniel Mueller-Gritschneder, Ulf Schlichtmann: ["muRISCV-NN: Challenging Zve32x Autovectorization with TinyML Inference Library for RISC-V Vector Extension"](https://dl.acm.org/doi/10.1145/3637543.3652878), CF '24 Companion: Proceedings of the 21st ACM International Conference on Computing Frontiers: Workshops and Special Sessions, 2024

[3] muRISCV-NN, [https://github.com/tum-ei-eda/muriscv-nn](https://github.com/tum-ei-eda/muriscv-nn)

[4] Banbury, Colby and Reddi, Vijay Janapa and Torelli, Peter and Holleman, Jeremy and Jeffries, Nat and Kiraly, Csaba and Montino, Pietro and Kanter, David and Ahmed, Sebastian and Pau, Danilo and others, ["MLPerf Tiny Benchmark"](https://github.com/mlcommons/tiny)

[5] Tensorflow Tutorials, "Simple audio recognition: Recognizing keywords", [https://www.tensorflow.org/tutorials/audio/simple_audio](https://www.tensorflow.org/tutorials/audio/simple_audio)

[6] Ultralytics, YOLO11, [https://docs.ultralytics.com/models/yolo11](https://docs.ultralytics.com/models/yolo11)

[7] Vödisch, Niclas and Dodel, David and Schötz, Michael, FSOCO: The Formula Student Objects in Context Dataset [https://fsoco.github.io/fsoco-dataset/](https://fsoco.github.io/fsoco-dataset/)

[8] OPT-125M, [https://www.kaggle.com/models/keras/opt/keras/opt_125m_en](https://www.kaggle.com/models/keras/opt/keras/opt_125m_en)
