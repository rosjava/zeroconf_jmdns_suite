###############################################################################
# Executables
###############################################################################

rosbuild_add_executable(zeroconf zeroconf.cpp)
target_link_libraries(zeroconf zeroconf_avahi)
