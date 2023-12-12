[![ROS1 VERSION](https://img.shields.io/badge/ROS1-Noetic-green)](http://wiki.ros.org/noetic) &nbsp;
[![ROS2 VERSION](https://img.shields.io/badge/ROS2-Humble-brightgreen)](http://docs.ros.org/en/humble/index.html) &nbsp;
[![Moveit](https://img.shields.io/badge/Moveit-noetic-green)](https://ros-planning.github.io/moveit_tutorials/) &nbsp;
[![Ubuntu for ROS1](https://img.shields.io/badge/Ubuntu-20.04-green)](https://ubuntu.com/) &nbsp;
[![Ubuntu_for_ROS2](https://img.shields.io/badge/Ubuntu-22.04-brightgreen)](https://ubuntu.com/) &nbsp;
[![LICENSE](https://img.shields.io/badge/license-Apache--2.0-informational)](https://github.com/mangdangroboticsclub/gpt4_ros2/blob/main/LICENSE) &nbsp;


# ros-install-one-click

This is a one-click shell script that enables the installation of either ROS1 Noetic or ROS2 Humble on Ubuntu, specifically optimized for users in China. It now also includes a one-click installation script for MoveIt under ROS1 Noetic, realsense camera install with Nvidia Orin nano.

## ROS 1 Noetic Installation
To install ROS1 Noetic with a single command, copy and execute the following command in the terminal:

```bash
wget -O $HOME/ros1_noetic_install.sh https://raw.githubusercontent.com/auromix/ros-install-one-click/main/ros1_noetic_install.sh && sudo chmod +x $HOME/ros1_noetic_install.sh && sudo bash $HOME/ros1_noetic_install.sh && rm $HOME/ros1_noetic_install.sh
```
## ROS 2 Humble Installation
To install ROS2 Humble with a single command, copy and execute the following command in the terminal:

```bash
wget -O $HOME/ros2_humble_install.sh https://raw.githubusercontent.com/auromix/ros-install-one-click/main/ros2_humble_install.sh && sudo chmod +x $HOME/ros2_humble_install.sh && bash $HOME/ros2_humble_install.sh && rm $HOME/ros2_humble_install.sh
```

## MoveIt 1 Installation
To install MoveIt for ROS1 Noetic with a single command, copy and execute the following command in the terminal:

```bash
wget -O $HOME/moveit1_install.sh https://raw.githubusercontent.com/auromix/ros-install-one-click/main/moveit1_install.sh && sudo chmod +x $HOME/moveit1_install.sh && sudo bash $HOME/moveit1_install.sh && rm $HOME/moveit1_install.sh
```

## Intel realsense D400 series camera Installation
To install realsense D400 series camera with a single command, copy and execute the following command in the terminal:

```bash
wget -O $HOME/realsense_d400_series_install.sh https://raw.githubusercontent.com/auromix/ros-install-one-click/main/realsense_d400_series_install.sh && sudo chmod +x $HOME/realsense_d400_series_install.sh && bash $HOME/realsense_d400_series_install.sh
```

To install Realsense for Nvidia Jetson Orin nano with a single command, copy and execute the following command in the terminal:

```bash
wget -O $HOME/realsense_install_nvidia.sh https://raw.githubusercontent.com/auromix/ros-install-one-click/main/realsense_install_nvidia.sh && sudo chmod +x $HOME/realsense_install_nvidia.sh && bash $HOME/realsense_install_nvidia.sh && rm $HOME/realsense_install_nvidia.sh
```
## Orbbec Femto Bolt Installation
To install Orbbec Femto Bolt Camera ROS1 Noetic version with a single command, copy and execute the following command in the terminal:

```bash
wget -O $HOME/orbbec_femto_bolt_ros1_install.sh https://raw.githubusercontent.com/auromix/ros-install-one-click/main/orbbec_femto_bolt_ros1_install.sh && sudo chmod +x $HOME/orbbec_femto_bolt_ros1_install.sh && bash $HOME/orbbec_femto_bolt_ros1_install.sh && rm $HOME/orbbec_femto_bolt_ros1_install.sh
```

To install Orbbec Femto Bolt Camera ROS2 Humble version with a single command, copy and execute the following command in the terminal:


```bash
wget -O $HOME/orbbec_femto_bolt_ros2_install.sh https://raw.githubusercontent.com/auromix/ros-install-one-click/main/orbbec_femto_bolt_ros2_install.sh && sudo chmod +x $HOME/orbbec_femto_bolt_ros2_install.sh && bash $HOME/orbbec_femto_bolt_ros2_install.sh && rm $HOME/orbbec_femto_bolt_ros2_install.sh
```

### Microsoft Azure Kinect DK Installation

To install Azure Kinect DK with a single command, copy and execute the following command in the terminal:

```bash
wget -O $HOME/azure_kinect_dk_install.sh https://raw.githubusercontent.com/auromix/ros-install-one-click/main/azure_kinect_dk_install.sh && sudo chmod +x $HOME/azure_kinect_dk_install.sh && bash $HOME/azure_kinect_dk_install.sh
```
### Isaac Sim python assistant
The Isaac Sim python assistant script facilitates the execution of an Isaac Sim standalone Python script provided by the user.
```bash
# Go to your isaac sim standalone python directory
cd <your_python_directory>
# Download the script
wget https://raw.githubusercontent.com/auromix/ros-install-one-click/main/isaacsim_python_assistant.sh
# Grant execute permission
chmod +x isaacsim_python_assistant.sh
```
```bash
# Go to your isaac sim standalone python directory
cd <your_python_directory>
# Run the script to launch your isaac sim standalone python file
./isaacsim_python_assistant.sh
```
### Config Static IP
This script provides a command-line method to configure a static IP address for an Ethernet interface on a robot. It is particularly useful when a graphical user interface (GUI) is not easily accessible or feasible.
```bash
./config_static_ip.sh
```
### Test CUDA
This script test your CUDA and Pytorch environment.
```bash
wget https://raw.githubusercontent.com/auromix/ros-install-one-click/main/test_cuda.py && python3 test_cuda.py
```
