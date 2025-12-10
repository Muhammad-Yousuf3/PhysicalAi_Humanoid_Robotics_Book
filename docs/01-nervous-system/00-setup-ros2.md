# ROS 2 Humble Installation and Setup Guide

This section will provide a detailed, step-by-step guide for setting up your ROS 2 development environment.

## 1. Prerequisites

-   **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish)
-   **Network**: Stable internet connection
-   **User**: A user account with `sudo` privileges

## 2. Configure Locale

Ensure your locale is set correctly to avoid issues with ROS 2.

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
```

## 3. Setup Sources

Add the ROS 2 apt repository to your system.

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 4. Install ROS 2 Packages

Install the `ros-humble-desktop` package, which includes ROS, RViz, and demos.

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-humble-desktop
```

## 5. Environment Setup

Source the ROS 2 setup script to make ROS 2 commands available in your terminal. For persistent setup, add it to your `~/.bashrc`.

```bash
# Temporarily source
source /opt/ros/humble/setup.bash

# For persistent setup, add to .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 6. Install Colcon

`colcon` is the recommended build tool for ROS 2 packages.

```bash
sudo apt install -y python3-colcon-common-extensions
```

## 7. Verify Installation

Check if ROS 2 is installed correctly by running a demo.

```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

You should see the talker publishing messages and the listener receiving them.
