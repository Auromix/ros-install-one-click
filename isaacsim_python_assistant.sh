#!/bin/bash
#
# Description: This script facilitates the execution of an Isaac Sim standalone Python script provided by the user.
# Version: 1.0.0
# Author: Herman Ye @Auromix
# Date: 2023-11-14

# Exit the script immediately if a command exits with a non-zero status
# set -x
set -e

# Clear the terminal screen
clear

# Get the directory of this script
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Go to the directory of this script
cd "$script_dir"

# ANSI color codes
red='\033[0;31m'
green='\033[0;32m'
yellow='\033[0;33m'
reset='\033[0m'

# List all Python files in the current directory
echo "####################################################"
echo "#                                                  #"
echo -e "# ${red}Isaac Sim Standalone Python Execution Assistant${reset}  #"
echo "#                                                  #"
echo "####################################################"
echo "#                                                  #"
echo "# YOU CAN EXECUTE THE FOLLOWING PYTHON FILES:      #"

# Loop through all Python files in the current directory
for file in *.py; do
    file_length=${#file}
    space_length=$((49 - file_length))
    printf "# "
    printf "${green}$file${reset}"
    printf "%0.s " $(seq 1 $space_length)
    printf "#\n"
done

echo "#                                                  #"
echo "####################################################"

# Prompt the user to enter the name of the Python file to execute
echo ""
echo "Enter the standalone Isaac sim Python filename"
read -e -p "to execute (use Tab for completion): " filename

# Check if the user entered a valid Python file name
if [ -z "$filename" ]; then
    echo -e "${red}Error: Please enter a valid Python file name.${reset}"
    exit 1
fi

# Print the script directory
echo ""
echo "Script directory:"
echo "$script_dir"

# Print the Python file path
python_path="$script_dir/$filename"
echo "Python file:"
echo "$python_path"

# Check if the Python file exists
if [ ! -f "$python_path" ]; then
    echo -e "${red}Error: Python file '$filename' not found.${reset}"
    exit 1
fi
echo -e "${green}Python file found.${reset}"

# Search for a directory with 'isaac_sim' in the name under ~/.local/share/ov/pkg/
isaac_sim_dir=$(find ~/.local/share/ov/pkg/ -type d -name "*isaac_sim*" -print -quit)

# Check if the directory is found
if [ -z "$isaac_sim_dir" ]; then
    echo -e "${red}Error: Unable to find a directory with 'isaac_sim' in the name under ~/.local/share/ov/pkg/.${reset}"
    exit 1
else
    echo "Isaac Sim directory:"
    echo "$isaac_sim_dir"
    echo -e "${green}Isaac Sim found.${reset}"
fi

# Check if the $isaacrepo environment variable is set
if [ -z "$isaacrepo" ]; then
    echo -e "${yellow}Warning: \$isaacrepo environment variable is not set.${reset}"
    
    # Ask the user if they want to add it to the environment variable
    read -p "Add to the environment variable \$isaacrepo? [y/n] " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Adding to the environment variable \$isaacrepo..."
        echo "# ISAAC SIM ENVIRONMENT VARIABLE" >> ~/.bashrc
        echo "export isaacrepo=$isaac_sim_dir" >> ~/.bashrc
        source ~/.bashrc
    else
        # Skip adding
        echo "Not adding to the environment variable \$isaacrepo."
    fi
else
    echo "\$isaacrepo environment variable:"
    echo "$isaacrepo"
    echo -e "${green}Isaac Sim environment variable found.${reset}"
fi

# Change directory to Isaac Sim directory
echo "Going to $isaac_sim_dir"
cd "$isaac_sim_dir"

# Start the Python script
echo -e "${green}Starting $filename...${reset}"
sleep 1
bash python.sh "$python_path"
exit 0

