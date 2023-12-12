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
# This is a shell script for installing Orbbec Femto Bolt ROS 1 version.
#
# Version: 1.0
# Date: 2023-12-12
# Author: Herman Ye @Auromix
#
# Warning: This script assumes ROS1 Noetic is already installed on Ubuntu20.04.
# set -x
set -e


# Get script directory
SCRIPT_DIR=$(dirname "$0")

# Get the username of the non-root user
USERNAME=$USERNAME
echo "Current user is: $USERNAME"
echo "Script directory is: $SCRIPT_DIR"

# Save logs to files
LOG_FILE="${SCRIPT_DIR}/orbbec_femto_bolt_ros1_install.log"
ERR_FILE="${SCRIPT_DIR}/orbbec_femto_bolt_ros1_install.err"
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
echo "Start to install Orbbec Femto Bolt ROS1 Noetic..."
echo "Warning: This script assumes ROS1 Noetic is already installed on Ubuntu20.04."
sleep 3

# Create ROS workspace
echo "Removing old orbbec test workspace..."
rm -rf /home/$USERNAME/orbbec_test_ws
echo "Creating new orbbec test workspace..."
mkdir -p /home/$USERNAME/orbbec_test_ws/src && cd /home/$USERNAME/orbbec_test_ws/src

# Download Camera ROS SDK
echo "Downloading Orbbec ROS1 SDK..."
git clone  https://github.com/orbbec/OrbbecSDK_ROS1.git

# Install dependencies
echo "Installing dependencies..."
cd /home/$USERNAME/orbbec_test_ws
rosdep install --from-paths src --ignore-src -r -y
sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager -y
sudo apt install ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher -y
sudo apt install libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev -y

# Build
echo "Building..."
cd /home/$USERNAME/orbbec_test_ws
catkin_make

# Set workspace environment
echo "Setting workspace environment..."
echo "source /home/$USERNAME/orbbec_test_ws/devel/setup.bash" >> /home/$USERNAME/.bashrc
source /home/$USERNAME/orbbec_test_ws/devel/setup.bash

# Set udev rules for camera
echo "Setting udev rules for camera..."
roscd orbbec_camera
cd script
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules

# Reset udev to activate rules for camera
echo "Resetting udev to activate rules for camera..."
sudo udevadm control --reload && sudo udevadm trigger

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
TEXT1="Orbbec Femto Bolt ROS1 installation completed!"
TEXT2="Please open new terminals and run commands to verify the installation:"
TEXT3="roslaunch orbbec_camera femto_bolt.launch"
TEXT4="rosrun rqt_image_view rqt_image_view /camera/color/image_raw"

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
