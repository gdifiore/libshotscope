#!/bin/bash

# Check if Clang is installed
#if ! command -v clang &> /dev/null; then
#    echo "Clang is not installed. Installing Clang..."
#    sudo apt-get update
#    sudo apt-get install clang
#fi
# TODO: investigate gtest compiler errors
#export CC=/usr/bin/clang
#export CXX=/usr/bin/clang++

# Create build directory
mkdir -p build
cd build

# Generate build files using CMake
cmake ..

# Build the project
make