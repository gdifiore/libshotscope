# Check if the Arduino SDK is installed
find_package(Arduino REQUIRED)

# Add the Arduino target
add_arduino_library(ShotScope
    ${CMAKE_CURRENT_SOURCE_DIR}/arduino/ShotScope.ino)

# Link the necessary libraries
target_link_libraries(ShotScope
    fftw
    eigen
    )