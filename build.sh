#!/bin/bash

# Use Clang if available and not overridden, otherwise use system default
if [ -z "$CC" ] && [ -z "$CXX" ]; then
    if command -v clang &> /dev/null; then
        echo "Using Clang compiler"
        export CC=clang
        export CXX=clang++
    else
        echo "Using system default compiler"
    fi
else
    echo "Using compiler from environment: CC=$CC, CXX=$CXX"
fi

# Create build directory
mkdir -p build
cd build

# Generate build files using CMake
cmake ..

# Build the project
cmake --build .

# Optionally install (uncomment if you want to install system-wide)
# echo "Installing to system directories (password may be required)"
# sudo cmake --install .