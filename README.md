# ProvHook

This repository contains test cases for AutoTrace and instrumentation tools for runtime log collection. The test case is based on Nova Carter, please see [link](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_navigation.html) for more information.

## System Requirements

| Component  | Minimum Spec              |
|------------|---------------------------|
| OS         | Ubuntu 22.04              |
| GPU        | NVIDIA GeForce RTX 4080   |
| GPU Driver | 580.65.06                 |
| RAM        | 32GB                      |
| Storage    | 100GB                     |

## Isaac Sim Setup

### Install `CUDA` and `cuDNN`

Follow [link](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/) to install `CUDA`.

Follow [link](https://docs.nvidia.com/deeplearning/cudnn/installation/latest/linux.html) to install `cuDNN`.

### Install Isaac Sim 5.1.0

Download Isaac Sim 5.1.0:
```sh
mkdir -p downloads
wget -P downloads https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.1.0-linux-x86_64.zip
```

Unzip:
```sh
mkdir -p isaacsim510
unzip -n downloads/isaac-sim-standalone-5.1.0-linux-x86_64.zip -d isaacsim510
```

Verify Compatibility:
```sh
cd isaacsim510
./isaac-sim.compatibility_check.sh
```
The application checks the compatibility of the system and shows the results in color.

## PyTorch Setup

Install latest PyTorch:
```sh
python3 -m pip install torch
```

## ROS2 Setup

Prepare:
```sh
cd ros2_ws/scripts
```

### ROS2 Humble Setup

Setup ros2 dev tools and ros2 repo:
```sh
./setup-ros2-repo.sh
```

Get latest ros2 source code:
```sh
./ros2_humble/1-update-src.sh
```

Install ros2 dependencies:
```sh
./ros2_humble/2-install-deps.sh
```

### AutoTrace Setup

Get latest AutoTrace ros2 dependency source code:
```sh
./rt/1-update-src.sh
```

Install AutoTrace dependencies:
```sh
./rt/2-install-deps.sh
```

### Build

Build ros2 packages:
```sh
./build.sh
```
This step may take more than one hour.
