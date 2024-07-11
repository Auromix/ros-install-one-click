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
# This is a shell script for Configuring CUDA env.
# Version: 1.0
# Date: 2024-07-11
# Author: Herman Ye @Auromix
#
# set -x
# set -e

# Prompt the user to enter the CUDA version
read -p "Please enter the CUDA version (e.g., 11.8): " CUDA_TO_SET

# Remove existing CUDA settings
sed -i '/# CUDA SETTINGS/d' ~/.bashrc
sed -i '/export CUDA_HOME/d' ~/.bashrc
sed -i '/export PATH=\/usr\/local\/cuda/d' ~/.bashrc
sed -i '/export LD_LIBRARY_PATH=\/usr\/local\/cuda/d' ~/.bashrc

# Add new CUDA settings
echo "# CUDA SETTINGS" >> ~/.bashrc
echo "export CUDA_HOME=/usr/local/cuda-$CUDA_TO_SET" >> ~/.bashrc
echo "export PATH=/usr/local/cuda-$CUDA_TO_SET/bin:\$PATH" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/usr/local/cuda-$CUDA_TO_SET/lib64:\$LD_LIBRARY_PATH" >> ~/.bashrc

# Apply the new settings immediately
source ~/.bashrc

echo "CUDA version $CUDA_TO_SET has been configured"
