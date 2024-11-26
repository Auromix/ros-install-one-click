#!/bin/bash
#
# Copyright 2023-2024 Herman Ye @Auromix
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
# This is a shell script for Configuring udev rules.
# Version: 1.0
# Date: 2024-11-26
# Author: Herman Ye @Auromix
#
# set -x
# set -e

# Check if the script is being run with root privileges
if [ "$(id -u)" -ne 0 ]; then
    echo "Please run this script with root privileges."
    exit 1
fi

# Save the list of devices before the user plugs in any devices
echo "Please unplug your device and press Enter when done."
read -r

# Get the list of all devices before inserting the new one
before_devices=$(ls /dev/)

# Ask the user to plug in the device
echo "Now, please plug in the device and press Enter when done."
read -r

# Get the list of all devices after inserting the new one
after_devices=$(ls /dev/)

# Find the difference between the two device lists (new devices)
new_devices=$(comm -13 <(echo "$before_devices" | sort) <(echo "$after_devices" | sort))

if [ -z "$new_devices" ]; then
    echo "No new devices detected."
    exit 1
fi

# Print the newly detected devices and ask the user to select one
echo "The following new devices were detected. Please choose the device you need:"
select device in $new_devices; do
    if [ -n "$device" ]; then
        # Set the full path of the selected device
        device="/dev/$device"
        echo "You selected the device: $device"
        break
    else
        echo "Invalid selection, please choose a valid device number."
    fi
done

# Get the udev information of the selected device
udev_info=$(udevadm info --query=all --name="$device")
# Print the udev information for the selected device
echo "Udev information for $device:"
echo "$udev_info"

# Ask user if they want to create a udev rule for the device
echo "Do you want to create a udev rule for this device? (y/n)"
read -r create_rule

if [ "$create_rule" == "y" ]; then
    # Ask the user to provide a name for the rule file
    default_rule_name="99-my-auromix-device.rules"
    echo "Please specify a name for the udev rule file (default: $default_rule_name):"
    read -r udev_rule_name

    # Use the default name if the user does not specify one
    if [ -z "$udev_rule_name" ]; then
        udev_rule_name="$default_rule_name"
    fi

    # Ensure the rule name ends with ".rules"
    if [[ "$udev_rule_name" != *.rules ]]; then
        udev_rule_name="$udev_rule_name.rules"
    fi

    # Full path to the udev rule file
    udev_rule_file="/etc/udev/rules.d/$udev_rule_name"
    echo "Creating udev rule file: $udev_rule_file"

    # Extract key information from udevadm info
    id_vendor_id=$(echo "$udev_info" | grep -oP '^E: ID_VENDOR_ID=\K.*')
    id_model_id=$(echo "$udev_info" | grep -oP '^E: ID_MODEL_ID=\K.*')
    id_serial=$(echo "$udev_info" | grep -oP '^E: ID_SERIAL=\K.*')
    dev_name=$(echo "$udev_info" | grep -oP '^E: DEVNAME=\K.*')

    # Validate extracted data
    if [ -z "$id_vendor_id" ] || [ -z "$id_model_id" ]; then
        echo "Error: Could not extract sufficient information from udevadm."
        exit 1
    fi

    # Prompt the user for a custom SYMLINK name, with a default value
    default_symlink="my_auromix_device"
    echo "Please specify a custom SYMLINK name (default: $default_symlink):"
    read -r user_symlink

    # Use the default SYMLINK name if the user does not specify one
    if [ -z "$user_symlink" ]; then
        user_symlink="$default_symlink"
    fi

    # Generate the udev rule
    udev_rule="SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$id_vendor_id\", ATTRS{idProduct}==\"$id_model_id\", SYMLINK+=\"$user_symlink\""

    # Save the rule to the file
    echo "$udev_rule" >"$udev_rule_file"
    echo "Udev rule created at $udev_rule_file with the following content:"
    echo "$udev_rule"

    # Reload the udev rules to apply the new rule
    udevadm control --reload-rules
    udevadm trigger
    echo "Udev rules reloaded. Device $dev_name should now have a persistent symlink as /dev/$user_symlink."
else
    echo "No udev rule created."

fi
