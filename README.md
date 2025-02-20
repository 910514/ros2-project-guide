# ROS2 Project Guide

![TurtleBot3 Burger](https://github.com/910514/ros2-project-guide/blob/main/images/turtlebot3.jpg)

## Project Introduction

This project documents the methods we have researched so far and also uploads some of the code we have written.

---

## Folder Structure

Below is an overview of the project's folder structure:

```plaintext
ros2-project-guide
├── Documents
│   ├── ROS2 Humble Installation Guide.pdf
│   └── TurtleBot3 Burger Quick Start Guide.pdf
├── Example Code
│   ├── camera
│   │   └── capture.ipynb
│   ├── lidar
│   │   ├── Cartesian_gui_dbscan.py
│   │   └── True_north.py
│   └── yolo
│       ├── best_ncnn_model
│       │   ├── metadata.yaml
│       │   ├── model.ncnn.bin
│       │   ├── model.ncnn.param
│       │   └── model_ncnn.py
│       ├── bus.jpg
│       ├── object_detect.ipynb
│       ├── rotate_go.py
│       ├── rotate_with_yolo.py
│       ├── torch2nccc.ipynb
│       └── yolo11n.pt
├── README.md
└── images
    ├── Cartesian_gui_dbscan.png
    ├── distance.png
    ├── rotate_go.gif
    └── turtlebot3.jpg
```

---

## Documentation

### Documents

- **ROS2 Humble Installation Guide.pdf**  
  This document provides detailed instructions on how to install ROS2 Humble on your PC, including complete steps and common Linux commands. It also includes useful tutorial links to help you quickly get familiar with the ROS2 environment.

- **TurtleBot3 Burger Quick Start Guide.pdf**  
  This is an introductory guide that includes basic operation commands for the TurtleBot3 Burger robot, such as `bring up` (starting the robot) and launching the camera node, allowing you to easily grasp the fundamentals of robot control.

---

## Example Code

The example code is divided into three core modules: **Camera**, **LiDAR**, and **YOLO Object Detection**. Below is a detailed description:

#### Camera

- **capture.ipynb**  
  This is a Jupyter Notebook file that uses ROS2's `v4l2_camera_node` to capture photos, demonstrating how to operate the camera in a ROS2 environment.

#### LiDAR

- **Cartesian_gui_dbscan.py**  
  This Python script uses the DBSCAN algorithm to classify and group LiDAR data points for further analysis and visualization.  
  ![LiDAR Data Classification](https://github.com/910514/ros2-project-guide/blob/main/images/Cartesian_gui_dbscan.png)

- **True_north.py**  
  This script measures the distance directly in front of the camera (i.e., the direction the camera is facing).  
  ![Distance Measurement](https://github.com/910514/ros2-project-guide/blob/main/images/distance.png)

#### YOLO (Object Detection)

- **rotate_go.py**  
  This script integrates YOLO object detection to make the robot approach a specified object once it is detected.  
  ![Approach Object](https://github.com/910514/ros2-project-guide/blob/main/images/rotate_go.gif)

- **object_detect.ipynb**  
  This Jupyter Notebook demonstrates how to use YOLO for object detection, ideal for learning the basics of image recognition.

- **rotate_with_yolo.py**  
  This script shows how to combine YOLO with robot rotation and movement control.

- **torch2nccc.ipynb**  
  This Notebook explains how to convert a PyTorch model to NCNN format for efficient deployment on embedded devices.

---

## Getting Started

1. **Install ROS2 Humble**  
   Refer to `Documents/ROS2 Humble Installation Guide.pdf` and follow the steps to complete the installation on your PC.

2. **Learn TurtleBot3 Operations**  
   Read `Documents/TurtleBot3 Burger Quick Start Guide.pdf` to master the basic control methods of the robot.

3. **Run Example Code**  
   - Use `capture.ipynb` to practice camera capture.  
   - Explore LiDAR data processing with `Cartesian_gui_dbscan.py` and `True_north.py`.  
   - Experience YOLO object detection and robot control with `rotate_go.py` and `object_detect.ipynb`.
