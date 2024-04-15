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
# This is a shell script for Setting up SSH Keys for github.
# Version: 1.0
# Date: 2024-04-15
# Author: Herman Ye @Auromix
#
# set -x
set -e

# Install xclip
sudo apt install xclip -y

# Set variables
key_file="ssh_for_auromix_ed25519"
key_type="ed25519"
key_email="auromixsshautoconfigemail@icloud.com"
github_ssh_site="https://github.com/settings/ssh/new"

# Remove existing SSH key
if [ -f "$HOME/.ssh/$key_file" ]; then
    rm "$HOME/.ssh/$key_file"
    echo "Existing SSH key has been removed."
fi
# Execute ssh-keygen command and simulate user input
echo -e "\n\n\n" | ssh-keygen -t $key_type -C $key_email -f "$HOME/.ssh/$key_file" -N ""

# Output prompt
echo "SSH key has been generated and saved in $HOME/.ssh/$key_file."

# Enable ssh agent
eval "$(ssh-agent -s)"

# Add private key to ssh-agent
ssh-add "$HOME/.ssh/$key_file"

# Output prompt
echo "SSH private key has been added to ssh-agent."

# Copy public key to clipboard
echo " "
cat "$HOME/.ssh/$key_file.pub" | xclip -selection clipboard
echo "Public key has been copied to clipboard."
echo "Please paste the public key in your GitHub account SSH key page."

# Open GitHub SSH key page in browser
echo "Press any key to open GitHub SSH key page $github_ssh_site in your browser..."
read -n 1 -s
if which xdg-open > /dev/null; then
    xdg-open $github_ssh_site
    elif which open > /dev/null; then
    open $github_ssh_site
else
    echo "Could not open browser. Please visit $github_ssh_site in your browser."
fi

