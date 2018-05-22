cmake_minimum_required(VERSION 2.8.3)
project(hop_detection)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_subdirectory(/home/nvidia/dlib dlib_build)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS system)
#find_package(Eigen 3.3 REQUIRED)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(libobject_detector  include/object_detection/object_detector_class.cpp include/object_detection/fhog_object_detector/fhog_object_detector.cpp include/object_detection/tracking/multi_track.cpp)
target_link_libraries(libobject_detector dlib::dlib  ${OpenCV_LIBRARIES})

cs_add_library(libarmor_detector include/color_detection/color_detection.cpp)
target_link_libraries(libarmor_detector ${OpenCV_LIBRARIES})

cs_add_library(enemy_detection_nodelet src/enemy_detection_nodelet.cpp src/enemy_detection_class.cpp)
target_link_libraries(enemy_detection_nodelet libobject_detector libarmor_detector dlib::dlib ${OpenCV_LIBRARIES})

cs_add_executable(enemy_detection_node src/enemy_detection.cpp)
target_link_libraries(enemy_detection_node libobject_detector libarmor_detector enemy_detection_nodelet dlib::dlib ${OpenCV_LIBRARIES})

cs_install()

#cs_install_scripts(scripts/my_script.py)

cs_export()
