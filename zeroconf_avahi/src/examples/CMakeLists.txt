###############################################################################
# Executables
###############################################################################

set(ZEROCONF_AVAHI_COMPILE_EXAMPLES OFF CACHE BOOL "Flag that enables/disables compilation of the official avahi examples.")

if(ZEROCONF_AVAHI_COMPILE_EXAMPLES)
    rosbuild_add_executable(official-publisher-example official_publish_example.cpp)
    target_link_libraries(official-publisher-example avahi-client)
    
    rosbuild_add_executable(official-browse-example official_browse_example.cpp)
    target_link_libraries(official-browse-example avahi-client)

    rosbuild_add_executable(demo_zeroconf_api demo_zeroconf_api.cpp)
    target_link_libraries(demo_zeroconf_api zeroconf_avahi)
endif()
