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

#include <zeroconf_avahi/zeroconf.hpp>
#include <zeroconf_comms/PublishedService.h>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	zeroconf_avahi::Zeroconf zeroconf;
	zeroconf_comms::PublishedService service_dudette, service_dude, service_colliding_dude, service_not_advertised;
	service_dudette.name = "Dudette";
	service_dudette.type = "_ros-master._tcp";
	service_dudette.port = 8888;
	service_dudette.domain = "local";
	ROS_INFO("===== Adding a service =====");
	zeroconf.add_service(service_dudette);
	sleep(2);
	ROS_INFO("===== Adding a listener =====");
	zeroconf.add_listener(service_dudette.type);
	sleep(2);
	service_dude = service_dudette;
	service_dude.name = "Dude";
	service_dude.port = 8889;
	ROS_INFO("===== Adding another service =====");
	zeroconf.add_service(service_dude);
	sleep(2);
	ROS_INFO("===== Accidentally adding the service again =====");
	zeroconf.add_service(service_dude);
	sleep(2);
	service_colliding_dude = service_dude;
	// collision
	ROS_INFO("===== Local Collision =====");
	service_colliding_dude.port = 8890;
	zeroconf.add_service(service_colliding_dude);
	sleep(3);
	ROS_INFO("===== Removing some services =====");
	zeroconf.remove_service(service_dudette);
	zeroconf.remove_service(service_colliding_dude);
	service_not_advertised = service_dude;
	service_not_advertised.name = "OtherDude";
	ROS_INFO("===== Remove a non-advertised service =====");
	zeroconf.remove_service(service_not_advertised);
	ROS_INFO("===== Waiting for listener to detect removals =====");
	sleep(2); // chance for the removals to show up
	ROS_INFO("===== Remove a listener =====");
	zeroconf.remove_listener(service_dudette.type);
	return 0;
}
