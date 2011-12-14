###############################################################################
# Sources
###############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

###############################################################################
# Library
###############################################################################

rosbuild_add_library(zeroconf_avahi ${SOURCES})
target_link_libraries(zeroconf_avahi avahi-client)
#set_target_properties(zeroconf_avahi PROPERTIES LINK_FLAGS "-Wl,--as-needed")
rosbuild_link_boost(zeroconf_avahi thread)