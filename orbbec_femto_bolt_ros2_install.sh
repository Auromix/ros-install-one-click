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
# This is a shell script for installing Orbbec Femto Bolt ROS 2 version.
#
# Version: 1.0
# Date: 2023-12-12
# Author: Herman Ye @Auromix
#
# Warning: This script assumes ROS2 Humble is already installed on Ubuntu22.04.
# set -x
set -e


# Get script directory
SCRIPT_DIR=$(dirname "$0")

# Get the username of the non-root user
USERNAME=$USERNAME
echo "Current user is: $USERNAME"
echo "Script directory is: $SCRIPT_DIR"

# Save logs to files
LOG_FILE="${SCRIPT_DIR}/orbbec_femto_bolt_ros2_install.log"
ERR_FILE="${SCRIPT_DIR}/orbbec_femto_bolt_ros2_install.err"
echo "Cleaning up traces of last installation..."
rm -f ${LOG_FILE}
rm -f ${ERR_FILE}

# Redirect output to console and log files
exec 1> >(tee -a ${LOG_FILE} )
exec 2> >(tee -a ${ERR_FILE} >&2)

# Output log info to console
echo "Installation logs will be saved to ${LOG_FILE}"
echo "Installation errors will be saved to ${ERR_FILE}"

# Waiting to start
echo "Start to install Orbbec Femto Bolt ROS2 Humble..."
echo "Warning: This script assumes ROS2 Humble is already installed on Ubuntu22.04."
sleep 3

# Create ROS2 workspace
echo "Removing old orbbec test workspace..."
rm -rf /home/$USERNAME/orbbec_test_ws
echo "Creating new orbbec test workspace..."
mkdir -p /home/$USERNAME/orbbec_test_ws/src && cd /home/$USERNAME/orbbec_test_ws/src

# Download Camera ROS2 SDK
echo "Downloading Orbbec ROS2 SDK..."
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

# Install dependencies
echo "Installing dependencies..."
cd /home/$USERNAME/orbbec_test_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev -y
sudo apt install ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher ros-$ROS_DISTRO-camera-info-manager -y

# Build
echo "Building..."
cd /home/$USERNAME/orbbec_test_ws
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release

# Set workspace environment
echo "Setting workspace environment..."
echo "source /home/$USERNAME/orbbec_test_ws/install/setup.bash" >> /home/$USERNAME/.bashrc
source /home/$USERNAME/orbbec_test_ws/install/setup.bash

# Set udev rules for camera
echo "Setting udev rules for camera..."
cd ~/orbbec_test_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
# Reset udev to activate rules for camera
echo "Resetting udev to activate rules for camera..."
sudo udevadm control --reload-rules && sudo udevadm trigger

# Replug the camera
echo ""
echo "Please replug the camera now and press any key to continue!"
read -n 1

# Verify installation
clear

# Define the colors
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[1;32m'
NC='\033[0m'

# Define the variables to be printed
TEXT1="Orbbec Femto Bolt ROS2 installation completed!"
TEXT2="Please open new terminals and run commands to verify the installation:"
TEXT3="ros2 launch femto_bolt.launch.py"
TEXT4="ros2 run rqt_image_view rqt_image_view /camera/color/image_raw"

# Calculate the center of the terminal window
TERMINAL_WIDTH=$(tput cols)
TEXT1_PADDING=$((($TERMINAL_WIDTH-${#TEXT1})/2))
TEXT2_PADDING=$((($TERMINAL_WIDTH-${#TEXT2})/2))
TEXT3_PADDING=$((($TERMINAL_WIDTH-${#TEXT3})/2))
TEXT4_PADDING=$((($TERMINAL_WIDTH-${#TEXT4})/2))

# Print the text in the center of the screen in the desired colors
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo -e "${GREEN}$(printf '%*s' $TEXT1_PADDING)${TEXT1} ${NC}"
echo -e "${NC}$(printf '%*s' $TEXT2_PADDING)${TEXT2} ${NC}"
echo -e "${RED}$(printf '%*s' $TEXT3_PADDING)${TEXT3} ${NC}"
echo -e "${RED}$(printf '%*s' $TEXT4_PADDING)${TEXT4} ${NC}"
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
echo ""
