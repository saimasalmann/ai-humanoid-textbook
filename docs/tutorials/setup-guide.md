---
title: Setup Guide
description: Comprehensive setup instructions for the Physical AI & Humanoid Robotics development environment
sidebar_position: 2
learning_objectives:
  - Install required software and dependencies
  - Configure the development environment
  - Verify the installation and setup
  - Troubleshoot common setup issues
---

# Setup Guide

## Learning Objectives

After completing this tutorial, you will be able to:

1. Install required software and dependencies
2. Configure the development environment
3. Verify the installation and setup
4. Troubleshoot common setup issues

## Introduction

This guide provides comprehensive instructions for setting up your development environment for Physical AI & Humanoid Robotics development. The setup process involves installing ROS 2, simulation environments (Gazebo and Isaac Sim), and other required tools. This environment will support all the modules covered in this textbook.

## System Requirements

Before beginning the setup, ensure your system meets the following requirements:

### Minimum Requirements
- **Operating System**: Ubuntu 22.04 LTS or Windows 10/11 (with WSL2)
- **CPU**: 4+ cores (8+ recommended)
- **RAM**: 16GB+ (32GB recommended)
- **GPU**: NVIDIA GPU with CUDA support (RTX series recommended for Isaac)
- **Storage**: 50GB+ free space
- **Network**: Stable internet connection for package downloads

### Recommended Requirements
- **CPU**: 8+ cores (Intel i7 or AMD Ryzen 7+)
- **RAM**: 32GB+
- **GPU**: NVIDIA RTX 3070 or higher
- **Storage**: 100GB+ SSD

## Installing ROS 2 Humble Hawksbill

ROS 2 Humble Hawksbill is the recommended distribution for this textbook.

### On Ubuntu 22.04

1. **Set locale**:
   ```bash
   locale  # check for UTF-8
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```

2. **Add ROS 2 apt repository**:
   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

3. **Install ROS 2 packages**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   sudo apt install ros-humble-cv-bridge ros-humble-tf2-tools ros-humble-nav2-bringup
   ```

4. **Install colcon and rosdep**:
   ```bash
   sudo apt install python3-colcon-common-extensions python3-rosdep
   sudo rosdep init
   rosdep update
   ```

5. **Setup environment**:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

### On Windows with WSL2

1. Install WSL2 with Ubuntu 22.04
2. Follow the Ubuntu installation instructions above within WSL2
3. Configure X11 forwarding for GUI applications

## Installing Gazebo

### Gazebo Garden

1. **Add Gazebo repository**:
   ```bash
   sudo apt update && sudo apt install wget lsb-release gnupg
   sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
   ```

2. **Install Gazebo Garden**:
   ```bash
   sudo apt update
   sudo apt install gazebo
   ```

## Installing NVIDIA Isaac Sim (Optional)

For Isaac Sim, you'll need:

1. **NVIDIA GPU with CUDA support**
2. **Install NVIDIA drivers** (latest recommended)
3. **Install Isaac Sim** from NVIDIA Developer website:
   - Create an NVIDIA Developer account
   - Download Isaac Sim from developer.nvidia.com
   - Follow installation instructions for your platform

### Docker-based Installation (Recommended)

```bash
# Pull Isaac Sim Docker image
docker pull nvcr.io/nvidia/isaac-sim:latest

# Run Isaac Sim
docker run --gupts all -it --rm --network=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/home/$USER:/home/$USER:rw" --volume="/etc/group:/etc/group:ro" --volume="/etc/passwd:/etc/passwd:ro" --volume="/etc/shadow:/etc/shadow:ro" --volume="/etc/sudoers.d:/etc/sudoers.d:ro" --privileged --pid=host nvcr.io/nvidia/isaac-sim:latest
```

## Python Environment Setup

1. **Install Python 3.8+** (usually pre-installed on Ubuntu 22.04):
   ```bash
   python3 --version
   ```

2. **Install pip and virtual environment tools**:
   ```bash
   sudo apt install python3-pip python3-venv
   ```

3. **Create a virtual environment for robotics projects**:
   ```bash
   python3 -m venv ~/robotics_env
   source ~/robotics_env/bin/activate
   pip install --upgrade pip
   ```

4. **Install common Python packages**:
   ```bash
   pip install numpy scipy matplotlib opencv-python openai gymnasium
   ```

## Development Tools

### Git Setup
```bash
sudo apt install git git-lfs
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### VS Code Setup
```bash
# Install VS Code
sudo snap install code --classic

# Install recommended extensions
code --install-extension ms-python.python
code --install-extension ms-iot.vscode-ros
code --install-extension ms-vscode.cpptools
```

## Verification Steps

1. **Test ROS 2 installation**:
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic list
   ```

2. **Test Gazebo installation**:
   ```bash
   gazebo --version
   gazebo
   ```

3. **Test Python packages**:
   ```bash
   python3 -c "import rclpy; print('ROS 2 Python client available')"
   python3 -c "import cv2; print('OpenCV available')"
   ```

4. **Create a simple test workspace**:
   ```bash
   mkdir -p ~/robotics_ws/src
   cd ~/robotics_ws
   colcon build
   source install/setup.bash
   ```

## Common Issues and Troubleshooting

### ROS 2 Issues
- **"command not found"**: Ensure ROS 2 environment is sourced
- **Permission errors**: Check user permissions and ROS 2 installation

### Gazebo Issues
- **Rendering problems**: Update graphics drivers
- **Performance issues**: Check system requirements

### Isaac Sim Issues
- **GPU not detected**: Verify CUDA installation and NVIDIA drivers
- **Docker permission errors**: Add user to docker group

## Summary

This setup guide provides the foundation for all robotics development covered in this textbook. A properly configured environment is essential for success in the subsequent modules. Take time to ensure all components are working correctly before proceeding to the first module.

## Exercises

1. Verify that ROS 2, Gazebo, and Python packages are correctly installed
2. Create a simple ROS 2 workspace and build it successfully

## References

1. ROS 2 Documentation. (2023). "Installation Guide". Retrieved from https://docs.ros.org/en/humble/Installation.html
2. Open Robotics. (2023). "Gazebo Installation". Retrieved from http://gazebosim.org/docs
3. NVIDIA. (2023). "Isaac Sim Installation Guide". NVIDIA Corporation.
4. ROS 2 Working Group. (2023). "ROS 2 with Python". Open Robotics.
5. OSRF. (2023). "Gazebo Simulation Environment". Open Source Robotics Foundation.
6. NVIDIA. (2023). "CUDA Installation Guide". NVIDIA Corporation.
7. MXRC Team. (2023). "ROS 2 Setup Best Practices". Open Robotics.
8. ROS Industrial Consortium. (2023). "Industrial ROS Setup Guide".

## Safety Disclaimer

When setting up your development environment, ensure that your system meets the hardware requirements to prevent overheating or system instability. Monitor system temperatures during intensive operations, especially when using GPU-accelerated simulation. Always follow the manufacturer's guidelines for hardware installation and configuration.