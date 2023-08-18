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
# Description: This script is for Ethernet IPv4 Setting without GUI
#
# Version: 1.0
# Date: 2023-08-17
# Author: Herman Ye @Auromix
#
# set -x
set -e

# Get a list of all Ethernet interface names
INTERFACE_LIST=($(ip link | awk -F: '$0 !~ "lo|vir|wl|^[^0-9]"{print $2}'))

# Print the available Ethernet interface list for user selection
echo "Here is the list of available Ethernet interfaces:"
for idx in ${!INTERFACE_LIST[@]}; do
    echo "$((idx+1)). ${INTERFACE_LIST[idx]}"
done

echo ""

# Prompt user to select an Ethernet interface
read -p "Please select the number of the Ethernet interface to configure (press Enter for default interface): " choice

# Use user's selection or default interface
if [[ -z "$choice" ]]; then
    INTERFACE=${INTERFACE_LIST[0]}  # Default to the first interface
    echo ""
    echo "Default Ethernet interface ${INTERFACE_LIST[0]} has been selected."
else
    idx=$((choice-1))
    if ((idx >= 0 && idx < ${#INTERFACE_LIST[@]})); then
        INTERFACE=${INTERFACE_LIST[idx]}
        echo ""
        echo "Ethernet interface ${INTERFACE_LIST[($choice-1)]} has been selected."
    else
        echo ""
        echo "Invalid choice. Exiting the script."
        exit 1
    fi
fi

# Prompt user to enter a new IP address, with a default value of "192.168.1.50"
DEFAULT_IP="192.168.1.50"
DEFAULT_NETMASK="255.255.255.0"
DEFAULT_GATEWAY="192.168.1.1"

read -p "Please enter the new IP address (press Enter for default IP): " NEW_IP && NEW_IP=${NEW_IP:-$DEFAULT_IP} && echo "New IP address: $NEW_IP"
read -p "Please enter the new subnet mask (press Enter for default subnet mask): " NEW_NETMASK && NEW_NETMASK=${NEW_NETMASK:-$DEFAULT_NETMASK} && echo "New subnet mask: $NEW_NETMASK"
read -p "Please enter the new gateway (press Enter for default gateway): " NEW_GATEWAY && NEW_GATEWAY=${NEW_GATEWAY:-$DEFAULT_GATEWAY} && echo "New gateway: $NEW_GATEWAY"

# Prompt user for confirmation
echo ""
read -p $'Changes to be made:\nEthernet interface: '"$INTERFACE"$'\nIP address: '"$NEW_IP"$'\nSubnet mask: '"$NEW_NETMASK"$'\nGateway: '"$NEW_GATEWAY"$'\n\nDo you confirm the changes? (y/n) ' confirm && [[ $confirm == [yY] || $confirm == [yY][eE][sS] ]] || exit 1

# Configure static IP address
sudo ip addr add $NEW_IP/$NEW_NETMASK dev $INTERFACE

# Configure gateway
sudo ip route add default via $NEW_GATEWAY dev $INTERFACE

# Done
clear
echo "Static IP configuration complete!"
read -p "Press ENTER to display the new network configuration: " dummy
clear
ip addr show $INTERFACE
echo ""
echo "Route:"
ip route show
