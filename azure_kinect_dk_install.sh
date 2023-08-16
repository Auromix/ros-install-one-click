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
# Description: This script automates the installation of Azure Kinect DK on Ubuntu 20.04.
# Version: 1.1
# Date: 2023-08-14
# Author: Herman Ye @Auromix
# set -x
set -e

# Download key and add to system key list
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -

# Add Microsoft Ubuntu18.04 packages list to source
curl -sSL https://packages.microsoft.com/config/ubuntu/18.04/prod.list | sudo tee /etc/apt/sources.list.d/microsoft-prod.list

# Update software source
sudo apt update -y

# Upgrade software
sudo apt upgrade -y

# Install package of Microsoft
# sudo apt install libk4a1.3-dev -y
# sudo apt install libk4abt1.0-dev -y
# sudo apt install k4a-tools=1.3.0 -y
sudo apt install wget -y
cd /tmp
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3/libk4a1.3_1.3.0_amd64.deb 
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0/libk4abt1.0_1.0.0_amd64.deb 
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3-dev/libk4a1.3-dev_1.3.0_amd64.deb 
wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0-dev/libk4abt1.0-dev_1.0.0_amd64.deb 
wget https://packages.microsoft.com/repos/microsoft-ubuntu-bionic-prod/pool/main/k/k4a-tools/k4a-tools_1.3.0_amd64.deb 
wget http://archive.ubuntu.com/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.0.2-1_amd64.deb 
# [libk4a1.3-dev](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.3-dev)
# [libk4abt1.0-dev](https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.0-dev/)
# [k4a-tools](https://packages.microsoft.com/repos/microsoft-ubuntu-bionic-prod/pool/main/k/k4a-tools/)
sudo dpkg -i libk4a1.3_1.3.0_amd64.deb
sudo dpkg -i libk4abt1.0_1.0.0_amd64.deb
sudo dpkg -i libk4a1.3-dev_1.3.0_amd64.deb
sudo dpkg -i libk4abt1.0-dev_1.0.0_amd64.deb
sudo dpkg -i libsoundio1_1.0.2-1_amd64.deb
sudo dpkg -i k4a-tools_1.3.0_amd64.deb

# Forbidden the DK related package to update to keep specific version
sudo apt-mark hold libk4a1.3
sudo apt-mark hold libk4a1.3-dev
sudo apt-mark hold libk4abt1.0
sudo apt-mark hold libk4abt1.0-dev
sudo apt-mark hold k4a-tools

# Add current user to plugdev user group
sudo usermod -aG plugdev $USER

# Write DK rules to /etc/udev/rules.d
UDEV_RULES_FILE="/etc/udev/rules.d/99-k4a.rules"
sudo tee $UDEV_RULES_FILE > /dev/null <<EOF
# Bus 002 Device 116: ID 045e:097a Microsoft Corp.  - Generic Superspeed USB Hub
# Bus 001 Device 015: ID 045e:097b Microsoft Corp.  - Generic USB Hub
# Bus 002 Device 118: ID 045e:097c Microsoft Corp.  - Azure Kinect Depth Camera
# Bus 002 Device 117: ID 045e:097d Microsoft Corp.  - Azure Kinect 4K Camera
# Bus 001 Device 016: ID 045e:097e Microsoft Corp.  - Azure Kinect Microphone Array

BUS!="usb", ACTION!="add", SUBSYSTEM!=="usb_device", GOTO="k4a_logic_rules_end"

ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097a", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097b", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097c", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097d", MODE="0666", GROUP="plugdev"
ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097e", MODE="0666", GROUP="plugdev"

LABEL="k4a_logic_rules_end"
EOF

echo "Azure Kinect DK udev Rules have been added to $UDEV_RULES_FILE"

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

# K4a Test
echo ""
echo "Azure Kinect DK installed successfully!"
read -n 1 -s -r -p "Press any key to open K4A Viewer for testing..."
k4aviewer


