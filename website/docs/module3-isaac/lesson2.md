---
sidebar_position: 2
---

# Module 3: AI-Robot Brain - Lesson 2: Training YOLOv8 with Synthetic Data

This lesson will guide you through the process of training a YOLOv8 object detection model using the synthetic datasets generated from NVIDIA Isaac Sim. Training with synthetic data is a powerful technique to overcome the challenges of acquiring and annotating large real-world datasets, especially for robotic applications where specific objects or environments might be rare or difficult to access.

## Introduction to YOLOv8

YOLO (You Only Look Once) is a popular family of object detection models known for its speed and accuracy. YOLOv8 is the latest iteration, offering improved performance and ease of use. It can be used for various tasks, including object detection, instance segmentation, and image classification. For our purposes, we will focus on object detection.

## Why Synthetic Data for Training?

*   **Cost-Effective:** Generating synthetic data in simulation can be significantly cheaper and faster than collecting and annotating real-world data.
*   **Scale:** Easily generate thousands or millions of diverse images with varying lighting, textures, backgrounds, and object poses using domain randomization.
*   **Perfect Annotations:** Synthetic data comes with pixel-perfect ground truth annotations (bounding boxes, masks, etc.), eliminating human annotation errors.
*   **Edge Cases:** Easily simulate rare or dangerous scenarios that are difficult or unsafe to capture in the real world.
*   **Privacy:** Avoids privacy concerns associated with real-world image collection.

## Prerequisites

*   A generated synthetic dataset (from Task T039) with annotated images (e.g., in COCO or YOLO format).
*   Your Dockerized ROS 2 environment (from Module 1), which includes Python and pip.
*   Familiarity with Python and deep learning frameworks (e.g., PyTorch).
*   Access to a GPU for efficient training.

## Step 1: Prepare Your Synthetic Dataset

Ensure your synthetic dataset is organized in a format compatible with YOLOv8. Ultralytics YOLOv8 typically expects data in a specific structure, often with images and labels (annotations) stored in separate directories. Common formats include COCO JSON or YOLO format (text files with bounding box coordinates).

If your Isaac Sim output is in a different format (e.g., Omniverse annotations), you will need to write a conversion script to transform it into the expected YOLOv8 format.

Example directory structure:

```
/path/to/your/dataset
├── images
│   ├── train
│   │   ├── image1.jpg
│   │   └── image2.jpg
│   └── val
│       ├── image3.jpg
│       └── image4.jpg
└── labels
    ├── train
    │   ├── image1.txt
    │   └── image2.txt
    └── val
        ├── image3.txt
        └── image4.txt
```

And a `data.yaml` file:

```yaml
train: /path/to/your/dataset/images/train
val: /path/to/your/dataset/images/val
test: /path/to/your/dataset/images/test # Optional

# Classes
nc: 2  # number of classes
names: ['robot_arm', 'cube'] # class names
```

## Step 2: Install YOLOv8

Install the Ultralytics YOLOv8 library using pip within your Python environment (ideally within your Docker container or a virtual environment):

```bash
pip install ultralytics
```

## Step 3: Train the YOLOv8 Model

You can train a YOLOv8 model using a simple Python script or directly from the command line.

**Using Command Line:**

```bash
yolo task=detect mode=train model=yolov8n.pt data=/path/to/your/data.yaml epochs=100 imgsz=640 name=yolov8_custom_model
```

*   `task=detect`: Specifies an object detection task.
*   `mode=train`: Indicates training mode.
*   `model=yolov8n.pt`: Starts training from a pre-trained `yolov8n` (nano) model, which is a good baseline. You can choose other sizes like `yolov8s.pt` (small), `yolov8m.pt` (medium), etc.
*   `data=/path/to/your/data.yaml`: Path to your dataset configuration file.
*   `epochs=100`: Number of training epochs. Adjust based on your dataset size and desired accuracy.
*   `imgsz=640`: Image size for training.
*   `name=yolov8_custom_model`: Name for your training run.

**Using Python Script:**

```python
from ultralytics import YOLO

# Load a model
model = YOLO('yolov8n.pt')  # load a pretrained model

# Train the model
results = model.train(data='/path/to/your/data.yaml', epochs=100, imgsz=640)
```

## Step 4: Evaluate and Export the Model

After training, you can evaluate your model and export it for deployment.

**Evaluate:**

```bash
yolo task=detect mode=val model=runs/detect/yolov8_custom_model/weights/best.pt data=/path/to/your/data.yaml
```

**Export (e.g., to ONNX):**

```bash
yolo task=detect mode=export model=runs/detect/yolov8_custom_model/weights/best.pt format=onnx
```

This will produce an ONNX file (`best.onnx`) that can be used for inference in various environments, including integration with Isaac ROS.

## Conclusion

By following these steps, you can effectively leverage synthetic data from Isaac Sim to train robust YOLOv8 object detection models. This forms a critical part of developing the perception capabilities for your humanoid robot, preparing it to identify objects in simulated and eventually real-world environments.