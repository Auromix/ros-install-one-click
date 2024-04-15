#!/bin/bash
#
# Copyright 2024 Herman Ye @Auromix
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
# This is a shell script for configuring the SSH port for GitHub.
# Version: 1.0
# Date: 2024-04-15
# Author: Herman Ye @Auromix
#
# VPNs typically route traffic through different ports than the default SSH port (port 22).
# By default, SSH connections to GitHub are made over port 22. 
# However, if you are using a VPN that operates on a different port (e.g., https port 443),
# GitHub's SSH traffic may need to be redirected accordingly.
#
# set -x
set -e

# Set configuration parameters
github_host="github.com"
github_hostname="ssh.github.com"
github_port="443"

# Ensure the ~/.ssh directory exists
mkdir -p ~/.ssh

# Check if the configuration already exists in the ~/.ssh/config file
if ! grep -q "Host $github_host" ~/.ssh/config; then
    # Create or append the configuration to the ~/.ssh/config file
    echo "Host $github_host" >> ~/.ssh/config
    echo "Hostname $github_hostname" >> ~/.ssh/config
    echo "Port $github_port" >> ~/.ssh/config
    echo "" >> ~/.ssh/config
    echo "Configuration added to the ~/.ssh/config file."
else
    echo "Configuration already exists in the ~/.ssh/config file. Skipping."
fi

