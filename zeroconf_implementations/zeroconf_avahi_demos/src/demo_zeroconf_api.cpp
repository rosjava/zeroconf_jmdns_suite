/**
 * @file /zeroconf_avahi/src/bin/publisher.cpp
 *
 * @brief This demos the zeroconf_avahi c++ api.
 *
 * It advertises two unique services, then tries to confuse it by
 * re-advertising an already advertised service, then re-advertising a
 * service on a different port which creates a collision event. In the former
 * case, it will simply abort, in the latter it will rename it after the
 * collision.
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

//#include <gtest/gtest.h>
#include <ros/ros.h>
#include <zeroconf_avahi/zeroconf.hpp>
#include <zeroconf_comms/PublishedService.h>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	zeroconf_avahi::Zeroconf zeroconf;
	zeroconf_comms::PublishedService service_dudette, service_dude, service_colliding_dude;
	service_dudette.name = "Dudette";
	service_dudette.type = "_ros-master._tcp";
	service_dudette.port = 8888;
	service_dudette.domain = "local";
	zeroconf.add_service(service_dudette);
	sleep(3);
	zeroconf.add_listener(service_dudette.type);
	sleep(3);
	service_dude = service_dudette;
	service_dude.name = "Dude";
	service_dude.port = 8889;
	zeroconf.add_service(service_dude);
	sleep(3);
	zeroconf.add_service(service_dude);
	sleep(3);
	service_colliding_dude = service_dude;
	// collision
	service_colliding_dude.port = 8890;
	zeroconf.add_service(service_colliding_dude);
	sleep(3);
	zeroconf.remove_service(service_dudette);
	zeroconf.remove_service(service_dude);
	zeroconf.remove_service(service_colliding_dude);
	zeroconf.remove_listener(service_dudette.type);
	sleep(3);
	return 0;
}
