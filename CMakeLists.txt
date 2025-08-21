cmake_minimum_required(VERSION 3.10)
project(mechatronics_platform_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/controller.cpp
    src/ackerman.cpp
    src/quadcopter.cpp
    src/mission.cpp
    src/main.cpp
)

# Create executable
add_executable(platform_controller ${SOURCES})

# Link pthread for multithreading
target_link_libraries(platform_controller pthread)