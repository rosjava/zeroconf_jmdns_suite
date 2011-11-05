###############################################################################
# Executables
###############################################################################

rosbuild_add_executable(official-publisher-example official_publish_example.cpp)
target_link_libraries(official-publisher-example avahi-client)
    
rosbuild_add_executable(official-browse-example official_browse_example.cpp)
target_link_libraries(official-browse-example avahi-client)

rosbuild_find_ros_package(zeroconf_avahi)

rosbuild_add_executable(demo_zeroconf_api demo_zeroconf_api.cpp)

# Need to rewrite the library without ros msg/srv's to do this
#add_definitions(-DZEROCONF_DEBUG)
#add_executable(demo_zeroconf_api demo_zeroconf_api.cpp)
#target_link_libraries(demo_zeroconf_api zeroconf_avahi) 