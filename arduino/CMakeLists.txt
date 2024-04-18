cmake_minimum_required(VERSION 3.10)
project(ShotScope)

# Set the Arduino board and other options
set(ARDUINO_BOARD nano)
set(ARDUINO_PORTS /dev/ttyUSB0)
set(ARDUINO_PROGRAMMER usbtiny)

# Add the Arduino library
add_arduino_library(golf-ball-tracker-radar-arduino
    ${CMAKE_CURRENT_SOURCE_DIR}/golf-ball-tracker-radar.ino)

# Link the necessary libraries
target_link_libraries(golf-ball-tracker-radar-arduino
    fftw
    eigen
    )