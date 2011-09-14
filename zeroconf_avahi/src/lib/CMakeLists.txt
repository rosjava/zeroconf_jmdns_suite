###############################################################################
# Sources
###############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

###############################################################################
# Library
###############################################################################

rosbuild_add_library(zeroconf_avahi ${SOURCES})
target_link_libraries(zeroconf_avahi avahi-client)
rosbuild_link_boost(zeroconf_avahi thread)