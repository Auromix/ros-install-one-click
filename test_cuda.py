#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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
# Author: Herman Ye @Auromix
# Description: Test CUDA and Pytorch environment

import torch
import torchvision


def main():
    # Check PyTorch version
    torch_version = torch.__version__
    print(f"Torch Version: {torch_version}")

    # Check torchvision version
    torchvision_version = torchvision.__version__
    print(f"Torchvision Version: {torchvision_version}")

    # Check if CUDA is available
    cuda_available = torch.cuda.is_available()
    print(f"CUDA with torch available: {cuda_available}")

    # Check CUDA version
    cuda_version = torch.version.cuda if cuda_available else "N/A (CUDA not available)"
    print(f"CUDA version: {cuda_version}")

    # Check cuDNN version
    print(f"cuDNN Version: {str(torch.backends.cudnn.version())}")

    if cuda_available:
        # Get the current CUDA device
        current_device = torch.cuda.current_device()
        print(f"Current CUDA device: {current_device}")

        # Get the number of CUDA devices
        num_devices = torch.cuda.device_count()
        print(f"Number of CUDA devices: {num_devices}")

        # Get the name of the current CUDA device
        device_name = torch.cuda.get_device_name(current_device)
        print(f"Name of current CUDA device: {device_name}")

    # TEST
    print("")
    print("TESTING CUDA WITH PYTORCH")
    print("========================")
    # Generate a random matrix on CPU
    x = torch.rand(5, 3)
    print("Random matrix on CPU:")
    print(x)

    if cuda_available:
        # Move the matrix to CUDA (GPU)
        x = x.cuda()
        print("\nRandom matrix on CUDA (GPU):")
        print(x)

    if cuda_available:
        print("\nTensor operations:")
        a = torch.cuda.FloatTensor(2).zero_()
        b = torch.randn(2).cuda()
        c = a + b
        print('Tensor a = ' + str(a))
        print('Tensor b = ' + str(b))
        print('Tensor c = ' + str(c))


if __name__ == "__main__":
    main()
