cmake_minimum_required(VERSION 2.8.3)
project(hop_tf)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

catkin_simple()

cs_add_executable(hopai_tf_broadcaster src/tf_broadcaster.cpp)

cs_install()

cs_export()
