cmake_minimum_required(VERSION 2.8.3)
project(hop_firing)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_install()

cs_install_scripts(scripts/auto_firing_0.2.py)

cs_export()
