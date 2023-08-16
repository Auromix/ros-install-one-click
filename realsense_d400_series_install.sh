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
# Description: This script automates the installation of Intel Realsense D400 series on x86 PC Ubuntu 20.04.
# Version: 1.0
# Date: 2023-08-16
# Author: Herman Ye @Auromix
# Reference: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
# Warning: UEFI/BIOS Security boot must be disabled before running this script.
#set -x
set -e

# Preconditions
echo "Warning: UEFI/BIOS Security boot must be disabled before running this script."
echo "Please check first if the conditions are met."
sleep 3

# Download transport-https
sudo apt install apt-transport-https -y

# Register the server's public key
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# Add the server to the list of repositories
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update

# Install the librealsense2
sudo apt install librealsense2-dkms -y
sudo apt install librealsense2-utils -y

# Install librealsense2 ros
sudo apt install ros-noetic-realsense2-camera -y
sudo apt-get install ros-noetic-realsense2-description -y

# Reset udev
echo "Resetting udev..."
sudo service udev reload
sleep 1
sudo service udev restart
sleep 1

# Reset USB
echo "Resetting USB..."

for port in $(lspci | grep USB | cut -d' ' -f1); do
    echo -n "0000:${port}" | sudo tee /sys/bus/pci/drivers/xhci_hcd/unbind > /dev/null
done

sleep 3

for port in $(lspci | grep USB | cut -d' ' -f1); do
    echo -n "0000:${port}" | sudo tee /sys/bus/pci/drivers/xhci_hcd/bind > /dev/null
done

# Test
sleep 2
clear
# Set color
GREEN='\033[1;32m'
NC='\033[0m'
COMMAND_ROS="roslaunch realsense2_camera rs_camera.launch"
# Print
echo "Intel Realsense Camera installed successfully!"
echo "If you want to test realsense ros:"
echo "Open a new terminal, then run"
echo -e "${GREEN}$(printf '%s' "${COMMAND_ROS}")${NC}"
echo "If you want to test realsense camera only:"
read -n 1 -s -r -p "Press any key to run realsense-viewer for testing..."
realsense-viewer



