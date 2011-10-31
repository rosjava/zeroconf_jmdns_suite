###############################################################################
# Executables
###############################################################################

rosbuild_add_executable(official-publisher-example official_publish_example.cpp)
target_link_libraries(official-publisher-example avahi-client)
    
rosbuild_add_executable(official-browse-example official_browse_example.cpp)
target_link_libraries(official-browse-example avahi-client)

rosbuild_add_executable(dude dude.cpp)
target_link_libraries(dude avahi-client)

rosbuild_add_executable(demo_zeroconf_api demo_zeroconf_api.cpp)
