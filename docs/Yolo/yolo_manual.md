# YOLO Manual

## 1. Overview

This project uses [YOLO (You Only Look Once)](https://en.wikipedia.org/wiki/You_Only_Look_Once), maintained by Ultralytics.

This manual focuses on **practical usage and deployment**, not internal implementation.  
For full details, refer to the official documentation: https://docs.ultralytics.com/

---

## 2. Hardware

- A **dedicated GPU is strongly recommended**,  CPU is typically ~10× slower，for example RTX 3050 (used for development in Spring 2026)  
- Ensure sufficient disk space. During development, generated 4k video data can quickly accumulate and may reach up to ~100 GB.

If no sufficiently powerful GPU is available, use the Aalto HPC cluster (Paniikki) with RTX 3080: https://scicomp.aalto.fi/aalto/paniikki/

## 3. Development environment Setup

### Python Environment (Optional)

Use a virtual environment:

- venv (recommended)
- Conda (optional), especially on Paniikki. 

Notes:
- Virtual environments do NOT include CUDA / cuDNN automatically
- These must be installed at the system level
- Use pip, since most NVIDIA-related instructions are written via pip 

---

## 4. Installation

### Jetson (aarch64) Setup

For Jetson devices, follow the official guide for Jetson: https://docs.ultralytics.com/guides/nvidia-jetson/
(especially `Run on JetPack 6.1` section)

Newest PyTorch & Torchvision for Jetson:  https://pypi.jetson-ai-lab.io/jp6/cu126

Related pages:
- https://forums.developer.nvidia.com/t/pytorch-for-jetson/340102
- https://developer.nvidia.com/cuda/gpus

### Other Setup: 
For other devices (e.g. x86 computers), follow the official guide: https://docs.ultralytics.com/#how-can-i-get-started-with-yolo-installation-and-setup

---

## 5. Quickstart

Follow the official quickstart guide: https://docs.ultralytics.com/quickstart/

---

## 6. Usage

### 6.1 Model Types

- .pt (PyTorch)  
  Default format for training and inference  

- .onnx  
  Cross-platform and flexible  

- .engine (TensorRT)  
  Fastest performance  
  Must be generated on the target device  

Important:
TensorRT models are NOT portable, copying .engine files between devices will NOT work  

---

### 6.2 Predict

Find objects using images or video.

The official guide for predict: https://docs.ultralytics.com/modes/predict/

    yolo detect predict model=your_model.pt source=path/to/source

Example:

    yolo detect predict model=yolo26n.pt source=video.mp4

---

### 6.3 Train

Train the model by using customize dataset.

The official guide for train: https://docs.ultralytics.com/modes/train/

    yolo detect train data=your_data.yaml model=your_model epochs=100 imgsz=640

---

### 6.4 Export (TensorRT)

Change the format of the model.

The official guide for export: https://docs.ultralytics.com/modes/export/

    yolo export model=yolo26n.pt format=engine

---

### 6.5 Other Features

- Track: https://docs.ultralytics.com/modes/track/
- Segment: https://docs.ultralytics.com/tasks/segment/
- Solutions: https://docs.ultralytics.com/solutions/

Note:
Some solutions are experimental and may require further development

---

## 7. Common Issues

### 7.1 TensorRT Not Found in virtual environment

Error:

    ModuleNotFoundError: No module named 'tensorrt'

Cause:
TensorRT is installed system-wide but not visible in virtual environment

Temporary solution:

    ln -s /usr/lib/python3.10/dist-packages/tensorrt \
          ~/your_env/lib/python3.10/site-packages/tensorrt

---

### 7.2 Other Common Problems

- Version mismatch (PyTorch / CUDA / TensorRT/ Python)
- Architecture mismatch: Jetson uses aarch64 (ARM).
- Typos in commands  
- Out of memory (OOM)  

---

## 8. Dataset & Training Pipeline

### 8.1 COCO128 Dataset

Default dataset for yolo:  
https://docs.ultralytics.com/datasets/detect/coco128/

- Contains 128 common object categories  
- Used for testing and validation  

---

### 8.2 Project Dataset

#### 8.2.1 ball_tree
A customize dataset contains ball and trees, mainly designed for nordic environment.
- [ball_tree on Roboflow](https://app.roboflow.com/wonderland-csu0a/ball_tree-ymspq/1)  
- [ball_tree on HuggingFace](https://huggingface.co/datasets/niy1/ball_tree)

pretrained Yolo26 models with result (nano, small, medium, large, extra large): 
https://huggingface.co/niy1/

---

### 8.3 Data Sources

Google Open Images:  
https://docs.ultralytics.com/datasets/detect/open-images-v7/

Download tool:
https://github.com/DmitryRyumin/OIDv6

---

### 8.4 Annotation

We use Roboflow for dataset preparation.

Notes:
- Online training in Roboflow may require payment, train models locally instead  

Guide:  
https://blog.roboflow.com/getting-started-with-roboflow

---

## 9. Best Practices

### Performance

- Use FP16 when exporting TensorRT models, faster with minimal accuracy loss  

---

### Training

- Official model training tips: https://docs.ultralytics.com/guides/model-training-tips

- Control batch size  
  Large batch → high memory usage → swapping → slowdown  

- Do NOT blindly increase imgsz during train, may not improve accuracy  

- Example performance  
  ~14 hours to train 4000 images (YOLO26x) on RTX 3050

---

### Development

- Always name output models clearly  (yolo XXX name="your_name"), avoid relying on default filenames.
- Prefer Ultralytics built-in tools for better performance. For example use "yolo export format=engine" instead of "trtexec", 


### Workflow Tips

- Read official documentation first  
- Search forums when encountering issues  
- Allocate sufficient time for annotation (depend on image, time can vary from few seconds to few minutes per image)

---

### Practical Advice

- Training on local machines can be noisy (GPU load)  
- Start from small models, the drone cannot run heavy models.

---

## 10. Additional Resources

- YOLO paper:  
  https://doi.org/10.48550/arXiv.1506.02640  

- Video tutorial:  
  https://www.youtube.com/watch?v=r0RspiLG260  

- Extract frames from video:  
  https://www.geeksforgeeks.org/python/python-program-extract-frames-using-opencv/  

---

## 11. Future Work

- Improve labeling quality  
- Current method: semi-automatic labeling using SAM3  

Known issue:
- SAM3 may fail on certain objects (e.g., spruce)

---