cmake_minimum_required(VERSION 3.14)
project(shotscope)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(PROJECT_SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(PROJECT_TEST_DIR ${PROJECT_SOURCE_DIR}/test)

include_directories(${PROJECT_INCLUDE_DIR})

set(SOURCES
   ${PROJECT_SRC_DIR}/math_utils.cpp
   ${PROJECT_SRC_DIR}/GolfBallPhysicsVariables.cpp
   ${PROJECT_SRC_DIR}/Simulator.cpp
   ${PROJECT_SRC_DIR}/GolfBallFlight.cpp
)

set(HEADERS
   ${PROJECT_INCLUDE_DIR}/libshotscope.hpp
   ${PROJECT_INCLUDE_DIR}/math_constants.hpp
   ${PROJECT_INCLUDE_DIR}/golf_ball.hpp
   ${PROJECT_INCLUDE_DIR}/math_utils.hpp
   ${PROJECT_INCLUDE_DIR}/GolfBallPhysicsVariables.hpp
   ${PROJECT_INCLUDE_DIR}/atmosphere.hpp
   ${PROJECT_INCLUDE_DIR}/GolfBallFlight.hpp
   ${PROJECT_INCLUDE_DIR}/Simulator.hpp
)

# Find OpenGL
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS})

# Create the static library
add_library(${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${OPENGL_LIBRARIES})

# Set compilation flags
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

# Include Google Test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Enable testing
enable_testing()

# Specify the test source files
set(TEST_SOURCES
    ${PROJECT_TEST_DIR}/test_ball_physics_vars.cpp
    ${PROJECT_TEST_DIR}/test_simulation.cpp
    ${PROJECT_TEST_DIR}/test_math_utils.cpp
    ${PROJECT_TEST_DIR}/test_ball_flight.cpp
    ${PROJECT_SRC_DIR}/GolfBallPhysicsVariables.cpp
    ${PROJECT_SRC_DIR}/math_utils.cpp
)

# Add test executable and link with gtest_main and the static library
add_executable(libshotscope_tests ${TEST_SOURCES})
target_link_libraries(libshotscope_tests ${PROJECT_NAME} GTest::gtest_main ${OPENGL_LIBRARIES})

# Eamples
add_executable(calculate_ball_landing examples/calculate_ball_landing.cpp)
target_link_libraries(calculate_ball_landing ${PROJECT_NAME})

add_executable(calculate_ball_trajectory examples/calculate_ball_trajectory.cpp)
target_link_libraries(calculate_ball_trajectory ${PROJECT_NAME})

# Discover tests
include(GoogleTest)
gtest_discover_tests(libshotscope_tests)

install(TARGETS shotscope DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)