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
# This is a shell script for installing basic development environment and setting best performance for NVIDIA Jetson AGX Orin.
# Version: 1.1
# Date: 2023-02-20
# Author: Herman Ye @Auromix
#
# Warning: Only for NVIDIA Jetson AGX Orin with JetPack 5.1.2, ubuntu 20.04
# set -x
set -e




# Get script directory
SCRIPT_DIR=$(dirname "$0")

# Get the username of the non-root user
USERNAME=$USER
echo "Current user is: $USERNAME"
echo "Script directory is: $SCRIPT_DIR"


# No Password sudo config
echo "Setting no-passwd sudo"
sudo sed -i 's/^%sudo.*/%sudo ALL=(ALL) NOPASSWD:ALL/g' /etc/sudoers

# No password auto login
echo "Setting no-passwd auto login"
echo "[Seat:*]" | sudo tee /etc/lightdm/lightdm.conf.d/50-nvidia.conf
echo "autologin-user=$USERNAME" | sudo tee -a /etc/lightdm/lightdm.conf.d/50-nvidia.conf
echo "autologin-user-timeout=0" | sudo tee -a /etc/lightdm/lightdm.conf.d/50-nvidia.conf

# Set auto login for gdm3
echo "Setting auto login for gdm3"
echo "Backing up /etc/gdm3/custom.conf to /etc/gdm3/custom.conf.backup ..."
sudo cp /etc/gdm3/custom.conf /etc/gdm3/custom.conf.backup
echo -e "[daemon]\nWaylandEnable=false\nAutomaticLoginEnable=true\nAutomaticLogin=$USERNAME\n\n[security]\n\n[xdmcp]" | sudo tee /etc/gdm3/custom.conf

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
echo "deb $MIRROR focal main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list
echo "deb $MIRROR focal-updates main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list
echo "deb $MIRROR focal-backports main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list

if [ $(uname -m) = "x86_64" ]; then
    echo "deb http://security.ubuntu.com/ubuntu/ focal-security main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list
else
    echo "deb http://ports.ubuntu.com/ubuntu-ports/ focal-security main restricted universe multiverse" | sudo tee -a /etc/apt/sources.list
fi

# System update
echo "Start to update software..."
sudo apt update
echo "Start to upgrade software..."
sudo apt upgrade -y

# Install pip
echo "Installing pip..."
sudo apt install python3-dev -y
sudo apt install pip -y

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

# Install curl
sudo apt install curl -y

# Backup sleep config
echo "Backing up /etc/systemd/sleep.conf to /etc/systemd/sleep.conf.backup ..."
sudo cp /etc/systemd/sleep.conf /etc/systemd/sleep.conf.backup
# Forbid sleep
sudo sed -i 's/#AllowSuspend=yes/AllowSuspend=no/g' /etc/systemd/sleep.conf
sudo sed -i 's/#AllowHibernation=yes/AllowHibernation=no/g' /etc/systemd/sleep.conf
sudo systemctl daemon-reload

# Update apt repository caches
echo "Updating software caches..."
sudo apt update
echo "Upgrading software..."
sudo apt upgrade -y

# Set fan and clock to max
sudo /usr/bin/jetson_clocks --show
sudo /usr/bin/jetson_clocks --fan
# sudo /usr/bin/jetson_clocks --store

# Set power perfomance
echo "Rebooting to reset the power mode..."
sleep 3
sudo nvpmodel -m 0 --force

echo "Basic env install done."
