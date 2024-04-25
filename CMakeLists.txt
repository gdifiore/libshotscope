cmake_minimum_required(VERSION 3.14)
project(libshotscope)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configure the include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Specify the source files
set(SOURCES
   ${PROJECT_SOURCE_DIR}/src/math_utils.cpp
   ${PROJECT_SOURCE_DIR}/src/GolfBallPhysicsVariables.cpp
)

# Specify the header files
set(HEADERS
   ${PROJECT_SOURCE_DIR}/include/math_constants.hpp
   ${PROJECT_SOURCE_DIR}/include/golf_ball.hpp
   ${PROJECT_SOURCE_DIR}/include/math_utils.hpp
   ${PROJECT_SOURCE_DIR}/include/GolfBallPhysicsVariables.hpp
   ${PROJECT_SOURCE_DIR}/include/atmosphere.hpp
)

# Create the static library
add_library(${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})

# Additional configuration (if needed)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

# Include Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This adds the following targets: gtest, gtest_main, gmock
FetchContent_MakeAvailable(googletest)

# Enable testing
enable_testing()

set(TEST_SOURCES
    ${PROJECT_SOURCE_DIR}/test/test_math_helpers.cpp
    ${PROJECT_SOURCE_DIR}/test/test_math_utils.cpp
    ${PROJECT_SOURCE_DIR}/src/GolfBallPhysicsVariables.cpp
    ${PROJECT_SOURCE_DIR}/src/math_utils.cpp
    # Add other test source files here
)

# Add test executable and link with gtest_main and the static library
add_executable(libshotscope_tests ${TEST_SOURCES})
target_link_libraries(libshotscope_tests ${PROJECT_NAME} GTest::gtest_main)

# Discover tests
include(GoogleTest)
gtest_discover_tests(libshotscope_tests)