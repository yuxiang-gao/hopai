cmake_minimum_required(VERSION 2.8.3)
project(hop_detection)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_subdirectory(/home/nvidia/dlib dlib_build)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(libenemy_detection src/enemy_detection_nodelet.cpp src/enemy_detection_class.cpp)
target_link_libraries(libenemy_detection dlib::dlib)
cs_add_executable(enemy_detection_node src/enemy_detection.cpp)
target_link_libraries(enemy_detection_node libenemy_detection dlib::dlib)

cs_install()

cs_install_scripts(scripts/my_script.py)

cs_export()
