/**
 * @file /zeroconf_avahi/src/bin/publisher.cpp
 *
 * @brief This is the example program from avahi.org.
 *
 * It advertises two printer services, then after 10 seconds, modifies
 * their names.
 *
 *   http://avahi.org/download/doxygen/client-publish-service_8c-example.html
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../../include/zeroconf_avahi/zeroconf.hpp"
#include <zeroconf_comms/PublishedService.h>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	zeroconf_avahi::Zeroconf zeroconf;
	zeroconf_comms::PublishedService service;
	service.name = "Dudette";
	service.type = "_ros-master._tcp";
	service.port = 8888;
	service.domain = "local";
	zeroconf.add_service(service);
	sleep(3);
	zeroconf.add_listener(service.type);
	sleep(3);
	service.name = "Dude";
	service.port = 8889;
	zeroconf.add_service(service);
	sleep(3);
	// collision
	service.port = 8890;
	zeroconf.add_service(service);
	sleep(3);
	zeroconf.remove_service(service);
	sleep(3);
	return 0;
}
