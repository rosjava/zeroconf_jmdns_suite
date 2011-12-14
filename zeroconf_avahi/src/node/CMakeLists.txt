###############################################################################
# Executables
###############################################################################

rosbuild_add_executable(zeroconf zeroconf.cpp)
target_link_libraries(zeroconf zeroconf_avahi)

# if we want to make specialised services
# rosbuild_add_executable(ros_master ros_master.cpp)
# target_link_libraries(zeroconf zeroconf_avahi)
