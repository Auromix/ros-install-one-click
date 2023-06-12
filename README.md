[![ROS1 VERSION](https://img.shields.io/badge/ROS1-Noetic-green)](http://wiki.ros.org/noetic) &nbsp;
[![Ubuntu for ROS1](https://img.shields.io/badge/Ubuntu-20.04-green)](https://ubuntu.com/) &nbsp;
[![ROS2 VERSION](https://img.shields.io/badge/ROS2-Humble-brightgreen)](http://docs.ros.org/en/humble/index.html) &nbsp;
[![Ubuntu_for_ROS2](https://img.shields.io/badge/Ubuntu-22.04-brightgreen)](https://ubuntu.com/) &nbsp;
[![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://github.com/mangdangroboticsclub/gpt4_ros2/blob/main/LICENSE) &nbsp;
# ros-install-one-click

This is a one-click shell script that enables the installation of either ROS1 Noetic or ROS2 Humble on Ubuntu, specifically optimized for users in China.

## Installation

### One-click Installation

To install ROS with a single command, copy and execute the following command in the terminal:

```bash
wget -O $HOME/ros1_noetic_install.sh https://raw.githubusercontent.com/hermanye996/ros-install-one-click/main/ros1_noetic_install.sh && sudo chmod +x $HOME/ros1_noetic_install.sh && sudo bash $HOME/ros1_noetic_install.sh && rm $HOME/ros1_noetic_install.sh
```

### Manual Installation

If it is not possible to connect to `raw.githubusercontent.com`, download the `ros1_noetic_install.sh` file manually in the root directory, and execute the following command:

```bash
sudo bash ros1_noetic_install.sh
```
