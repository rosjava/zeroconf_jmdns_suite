##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

##############################################################################
# Ros
##############################################################################

rosbuild_init()
rosbuild_genmsg()
rosbuild_gensrv()
