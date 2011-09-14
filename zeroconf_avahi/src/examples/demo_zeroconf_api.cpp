/**
 * @file /zeroconf_avahi/src/bin/publisher.cpp
 *
 * @brief This demos the zeroconf_avahi c++ api.
 *
 * It advertises two services, then advertises a collision which gets
 * automatically renamed.
 *
 * To compile this make sure the cmake cache variable
 * ZEROCONF_AVAHI_COMPILE_EXAMPLES is turned on (by default is off).
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
