cmake_minimum_required(VERSION 3.0.2)
project(ilo)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    tf2_geometry_msgs
    nav_msgs
    tf
)

# yaml-cpp
find_package(yaml-cpp REQUIRED)
# Eigen3
find_package(Eigen3 REQUIRED)
# gram_savitzky_golay
find_package(gram_savitzky_golay REQUIRED)


catkin_package()

include_directories(    
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(ilo_lib
    src/kalmanFilter/KF.hpp
    src/kalmanFilter/basicEKF.cpp

    src/legKinematics/kinematics.cpp

    src/utils/parameters.cpp
    src/utils/utility.cpp
    src/utils/visualization.cpp
)

target_link_libraries(ilo_lib
    ${YAML_CPP_LIBRARIES}
    ${catkin_LIBRARIES}
)

add_executable(ilo
    src/main.cpp
)

target_link_libraries(ilo
    ilo_lib
    gram_savitzky_golay::gram_savitzky_golay
)