cmake_minimum_required(VERSION 3.0)
project(gtsam_playground)

add_subdirectory(gtsam)
# Find GTSAM components
# find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

add_subdirectory(src/robot_motion)