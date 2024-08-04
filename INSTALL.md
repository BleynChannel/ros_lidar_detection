# Install & Setup

## Local host

### Requirements
- [Ubuntu 20.04](https://ubuntu.com/)
- Nvidia graphics with CUDA 11.3 (Check your GPU driver and CUDA version with `nvidia-smi`)
- [Python 3.8](https://www.python.org/downloads/)
- [ROS 1 Noetic](https://www.ros.org/)
- [Pytorch 1.11.0 with CUDA 11.3](https://pytorch.org/)
- [MMDetection3D 1.0.0rc4](https://github.com/open-mmlab/mmdetection3d/tree/v1.0.0rc4)
- [Open3D 0.18.0](https://github.com/intel-isl/Open3D)
- [ros_numpy](https://github.com/eric-wieser/ros_numpy)

> Note: For a better understanding of the necessary installation requirements for Python, you can follow the [bleyn/mmdet3d-ros:1.0.0rc4-noetic](https://hub.docker.com/r/bleyn/mmdet3d-ros/) image on Docker Hub.

```bash
git clone https://github.com/BleynChannel/3d_detection.git && cd ./3d_detection
python main.py detection configs/centerpoint_v2.py checkpoints/centerpoint_v2.pth -s
```

## Docker
```bash
git clone https://github.com/BleynChannel/3d_detection.git && cd ./3d_detection
docker-compose up --build
```

# For Developers
## Dev container
This repository contains the necessary files to build a Docker container for Lidar Object Detection in development using Visual Studio Code.

### Prerequisites
Before you begin, ensure you have the following installed:

- Docker: [Install Docker](https://docs.docker.com/get-docker/)
- Visual Studio Code: [Download VS Code](https://code.visualstudio.com/)
- Visual Studio Code Remote - Containers extension: [Install the extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

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