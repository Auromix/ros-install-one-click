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
# This is a shell script for installing ROS2 Humble and its dependencies in Ubuntu 22.04.
# It uses mirrors from Tsinghua University to speed up the download.
# The script also sets up the sudo privileges and modifies the sources.list file to update the repositories accordingly.
# It installs ROS2 Humble and its dependencies, initializes rosdep.
# It logs the installation progress and redirects the output to the console and logs files.
#
# Version: 1.0
# Date: 2023-10-31
# Author: Herman Ye @Auromix
#
# Warning: This script is ONLY for ROS2 Humble in ubuntu 22.04
# set -x
set -e

# Get script directory
SCRIPT_DIR=$(dirname "$0")

# Get the username of the non-root user
USERNAME=$USERNAME
echo "Current user is: $USERNAME"
echo "Script directory is: $SCRIPT_DIR"

# Save logs to files
LOG_FILE="${SCRIPT_DIR}/ros2_humble_install.log"
ERR_FILE="${SCRIPT_DIR}/ros2_humble_install.err"
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
echo "Start to install ROS2 Humble..."
sleep 3


# No Password sudo config
echo "Setting no-passwd sudo"
sudo sed -i 's/^%sudo.*/%sudo ALL=(ALL) NOPASSWD:ALL/g' /etc/sudoers

# Get architecture of the system
if [ $(uname -m) = "x86_64" ]; then
    MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ubuntu/"
else
    MIRROR="https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/"
fi
echo "Current system architecture is: $(uname -m)"
echo "Current mirror is: $MIRROR"

# Backup original software sources
echo "Backing up sources.list to /etc/apt/sources.list.backup ..."
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup

# Clear original software sources
echo "# Ubuntu Mirror Settings" | sudo tee /etc/apt/sources.list

# Replace software sources using tee
echo "deb $MIRROR jammy main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list
echo "deb $MIRROR jammy-updates main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list
echo "deb $MIRROR jammy-backports main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list

if [ $(uname -m) = "x86_64" ]; then
    echo "deb http://security.ubuntu.com/ubuntu/ jammy-security main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list
else
    echo "deb http://ports.ubuntu.com/ubuntu-ports/ jammy-security main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list
fi

# System update
echo "Start to update software..."
sudo apt update
echo "Start to upgrade software..."
sudo apt upgrade -y

# Install pip
echo "Installing pip..."
sudo apt install python3-dev -y
sudo apt install pip -y # If you haven't already installed pip

# Set default pip source
echo "configuring pip source..."
pip config set global.index-url http://pypi.tuna.tsinghua.edu.cn/simple
pip config set global.trusted-host pypi.tuna.tsinghua.edu.cn

# check for UTF-8
echo "Checking locale..."
echo "Current locale:"
locale

# Set language
echo "Setting language..."
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# verify settings
echo "Checking locale..."
locale

# Enable Ubuntu Universe repository
echo "Enabling Ubuntu Universe repository..."
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

# Add the ROS 2 GPG key with apt
echo "Adding the ROS 2 GPG key with apt..."
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

#  Add the repository to your sources list
echo "Adding the ROS 2 repository to your sources list..."
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update apt repository caches
echo "Updating software caches..."
sudo apt update
echo "Upgrading software..."
sudo apt upgrade -y

# Install ROS
echo "Installing ROS2 Humble..."
# ROS Desktop Install(ROS, RViz, demos, tutorials)
sudo apt install ros-humble-desktop -y
# ROS Base Install(Communication libraries, message packages, command line tools but no GUI tools)
sudo apt install ros-humble-ros-base -y
# Development tools(Compilers and other tools to build ROS packages)
sudo apt install ros-dev-tools -y
# Install build tool
sudo apt install python3-colcon-common-extensions -y

# Environment setup
if ! grep -q "source /opt/ros/humble/setup.bash" /home/$USERNAME/.bashrc; then
    echo "# ROS2 HUMBLE ENVIRONMENT SETTINGS" | sudo tee -a /home/$USERNAME/.bashrc
    echo "source /opt/ros/humble/setup.bash" | sudo tee -a /home/$USERNAME/.bashrc
    echo "ROS2 Humble environment setup added to /home/$USERNAME/.bashrc"
else
    echo "ROS2 Humble environment is already set in /home/$USERNAME/.bashrc"
fi
source /home/$USERNAME/.bashrc

# Create your ROS2 workspace
if [ -d "$workspace_dir" ]; then
    echo " ROS2 workspace already exists, skip creating."
else
    echo "Creating ROS2 workspace"
    cd /home/$USERNAME
    mkdir -p ros2_workspace/src
    cd /home/$USERNAME/ros2_workspace
    # Install package dependencies
    echo "Installing package dependencies..."
    sudo pip install rosdep
    sudo pip install rosdepc
    sudo rosdepc init > /dev/null
    rosdepc update > /dev/null
    rosdep install --from-paths src --ignore-src --rosdistro humble -y
    echo "Building workspace..."
    colcon build
fi

# System update again
sudo apt update
sudo apt dist-upgrade -y

# Verifying ROS2 installation
clear

# Define the variables to be printed
TEXT1="ROS2 Humble installation completed!"
TEXT2="Please open new terminals and run commands to verify the installation:"
TEXT3="ros2 run demo_nodes_cpp talker"
TEXT4="ros2 run demo_nodes_py listener"

# Define the colors
RED='\033[0;31m'
BLUE='\033[0;34m'
GREEN='\033[1;32m'
NC='\033[0m'

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

# # Remove ROS2
# sudo apt remove ~nros-humble-* && sudo apt autoremove
# sudo rm /etc/apt/sources.list.d/ros2.list
# sudo apt update
# sudo apt autoremove
# # Consider upgrading for packages previously shadowed.
# sudo apt upgrade
