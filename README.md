# Lidar Object Detection

<div align="center">
	[📘Documentation]() | [💻Install]() | [🐼Model Zoo]() |
</div>

# Introduction
**Lidar Object Detection** - algorithm for detection object on lidar point cloud. This algorithm based on CenterPoint algorithm and connected with [ROS 1 Noetic]()/[ROS 2 Humble]().

# Requirements
- Ubuntu 20.04
- [Python 3.8]()
- [ROS 1 Noetic]()/[ROS 2 Humble]()
- [MMDetection3D 1.0.0rc4]()

# Install
## Local host
```bash

```

## Docker
```bash
git clone 
docker build ./Dockerfile
```

# For Developers
## Dev container
This repository contains the necessary files to build a Docker container for Lidar Object Detection development using Visual Studio Code.

### Prerequisites
Before you begin, ensure you have the following installed:

- Docker: [Install Docker]()
- Visual Studio Code: [Download VS Code]()
- Visual Studio Code Remote - Containers extension: [Install the extension]()

### Getting Started
To build and run this project in the development container, follow these steps:

1. Clone the repository:
```bash
git clone https://github.com/BleynChannel/3d_detection.git
```
2. Open the repository in Visual Studio Code:
```bash
code 3d_detection
```
3. Press F1 to open the command palette and select `Remote-Containers: Open Folder in Container...`.
4. Select the root folder of the this repository.
5. To run the project, open the terminal in Visual Studio Code and run the following command:
```bash
cd ./app
python main.py detection configs/centerpoint_v2.py checkpoints/centerpoint_v2.pth -s
```

# Model Zoo and Benchmark

|Model        |mAP    |Mem (GB)|Inf time (fps)|Download             |
|-------------|-------|--------|--------------|---------------------|
|CenterPoint  |56.11  |5.2     |              |[model](), [config]()|

------------------------

# Credit
This project based on [MMDetection3D API]() and [Supervisely]().

# Licence
MIT