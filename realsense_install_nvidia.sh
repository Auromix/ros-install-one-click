#!/bin/bash
#
# Copyright 2023 Herman Ye @Auromix
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
# This script is used to install realsense2_camera and realsense2_description
#
# Version: 1.0
# Date: 2023-07-10
# Author: Herman Ye @Auromix
#
# Warning: This script assumes that the ubuntu20.04 system and ROS1 Noetic have been installed correctly
# If not, please execute ros1_noetic_install.sh first.
# The script is tested with Nvidia Jetson Orin nano
#
# set -x
set -e
# Get script directory
SCRIPT_DIR=$(dirname "$0")
# Check if script is run in Ubuntu 20.04
if [ "$(lsb_release -sc)" != "focal" ]; then
    echo "This script must be run in Ubuntu 20.04"
    read -p "Press any key to exit..."
    exit 1
fi
echo "Ubuntu 20.04 check passed"
# Check if script is run with ROS1 Noetic
if dpkg -s ros-noetic-desktop-full >/dev/null 2>&1; then
    echo "ROS1 Noetic check passed"
else
    echo "This script must be run with ROS1 Noetic-desktop-full"
    read -p "Press any key to exit..."
    exit 1
fi
# Save logs to files
LOG_FILE="${SCRIPT_DIR}/moveit1_install.log"
ERR_FILE="${SCRIPT_DIR}/moveit1_install.err"
rm -f "${LOG_FILE}"
rm -f "${ERR_FILE}"
# Redirect output to console and log files
exec 1> >(tee -a "${LOG_FILE}")
exec 2> >(tee -a "${ERR_FILE}" >&2)
# Install realsense from ROS distribution
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera -y
# Install realsense2 description
sudo apt-get install ros-$ROS_DISTRO-realsense2-description -y
# Register the server's public key
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
# Add the server to the list of repositories
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
# Install the SDK
sudo apt-get install librealsense2-utils -y
sudo apt-get install librealsense2-dev -y
# Reconnect the RealSense device
clear
echo "Installation complete!"
echo "Now, please reconnect the RealSense device"
echo "Open a new terminal and execute the following command to test:"
echo "realsense-viewer"
