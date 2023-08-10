[![ROS1 VERSION](https://img.shields.io/badge/ROS1-Noetic-green)](http://wiki.ros.org/noetic) &nbsp;
[![Moveit](https://img.shields.io/badge/Moveit-noetic-green)](https://ros-planning.github.io/moveit_tutorials/) &nbsp;
[![Ubuntu for ROS1](https://img.shields.io/badge/Ubuntu-20.04-green)](https://ubuntu.com/) &nbsp;
[![ROS2 VERSION](https://img.shields.io/badge/ROS2-Humble-brightgreen)](http://docs.ros.org/en/humble/index.html) &nbsp;
[![Ubuntu_for_ROS2](https://img.shields.io/badge/Ubuntu-22.04-brightgreen)](https://ubuntu.com/) &nbsp;
[![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://github.com/mangdangroboticsclub/gpt4_ros2/blob/main/LICENSE) &nbsp;


# ros-install-one-click

This is a one-click shell script that enables the installation of either ROS1 Noetic or ROS2 Humble on Ubuntu, specifically optimized for users in China. It now also includes a one-click installation script for MoveIt under ROS1 Noetic, realsense camera install with Nvidia Orin nano.

## ROS 1 Noetic Installation

### One-click Installation

To install ROS1 Noetic with a single command, copy and execute the following command in the terminal:

```bash
wget -O $HOME/ros1_noetic_install.sh https://raw.githubusercontent.com/auromix/ros-install-one-click/main/ros1_noetic_install.sh && sudo chmod +x $HOME/ros1_noetic_install.sh && sudo bash $HOME/ros1_noetic_install.sh && rm $HOME/ros1_noetic_install.sh
```

### Manual Installation

If it is not possible to connect to `raw.githubusercontent.com`, download the `ros1_noetic_install.sh` file manually in the root directory`/home/<your_username>`, and execute the following command:

```bash
sudo bash ros1_noetic_install.sh
```

## MoveIt 1 Installation

### One-click MoveIt Installation for ROS1 Noetic

To install MoveIt for ROS1 Noetic with a single command, copy and execute the following command in the terminal:

```bash
wget -O $HOME/moveit1_install.sh https://raw.githubusercontent.com/auromix/ros-install-one-click/main/moveit1_install.sh && sudo chmod +x $HOME/moveit1_install.sh && sudo bash $HOME/moveit1_install.sh && rm $HOME/moveit1_install.sh
```

### Manual MoveIt Installation for ROS1 Noetic

If it is not possible to connect to `raw.githubusercontent.com`, download the `moveit1_install.sh` file manually in the root directory`/home/<your_username>`, and execute the following command:

```bash
sudo bash moveit1_install.sh
```
## Realsense Installation

### One-click Realsense Installation for ROS1 Noetic

To install Realsense for Nvidia Jetson Orin nano with a single command, copy and execute the following command in the terminal:

```bash
wget -O $HOME/realsense_install_nvidia.sh https://raw.githubusercontent.com/auromix/ros-install-one-click/main/realsense_install_nvidia.sh && sudo chmod +x $HOME/realsense_install_nvidia.sh && ./$HOME/realsense_install_nvidia.sh && rm $HOME/realsense_install_nvidia.sh
```

### Manual Realsense Installation

If it is not possible to connect to `raw.githubusercontent.com`, download the `realsense_install_nvidia.sh` file manually in the root directory`/home/<your_username>`, and execute the following command:

```bash
./realsense_install_nvidia.sh
```
 
