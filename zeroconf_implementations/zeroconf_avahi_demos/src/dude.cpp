/**
 * @file /zeroconf_avahi_demos/src/dude.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 31/10/2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <avahi-client/publish.h>
#include <avahi-client/client.h>
#include <avahi-client/lookup.h>
#include <avahi-common/malloc.h>
#include <avahi-common/error.h>
#include <avahi-common/thread-watch.h>
#include <cstdio>
#include <cstdlib>
#include <vector>

struct Zeroconf {
	AvahiThreadedPoll *threaded_poll;
    AvahiClient *client;
	AvahiServiceBrowser *service_browser;
	std::vector<AvahiServiceResolver*> resolvers;
	Zeroconf() : threaded_poll(NULL), client(NULL), service_browser(NULL)
	{}
};

void client_callback(AvahiClient *c, AvahiClientState state,void * userdata) {

}
void resolve_callback(
							AvahiServiceResolver *r,
							AvahiIfIndex interface,
							AvahiProtocol protocol,
							AvahiResolverEvent event,
							const char *name,
							const char *type,
							const char *domain,
							const char *host_name,
							const AvahiAddress *address,
							uint16_t port,
							AvahiStringList *txt,
							AvahiLookupResultFlags flags,
							void* userdata) {

	Zeroconf *zeroconf = reinterpret_cast<Zeroconf*>(userdata);
    switch (event) {
        case AVAHI_RESOLVER_FAILURE: {
        	ROS_ERROR_STREAM("Zeroconf: failed to resolve service [" << name << "][" <<  type << "][" << domain << "][" << avahi_strerror(avahi_client_errno(avahi_service_resolver_get_client(r))) << "]");
            break;
        }
        case AVAHI_RESOLVER_FOUND: {
            char a[AVAHI_ADDRESS_STR_MAX], *t;
            t = avahi_string_list_to_string(txt);
            avahi_address_snprint(a, sizeof(a), address);

            /*********************
			** Saving Details
			**********************/
			/*********************
			** Logging
			**********************/
        	ROS_INFO_STREAM("Zeroconf: resolved new service [" << name << "][" << type << "][" << domain << "][" << a << "]");
        	ROS_DEBUG_STREAM("Zeroconf: \tname: " << name );
        	ROS_DEBUG_STREAM("Zeroconf: \ttype: " << type );
        	ROS_DEBUG_STREAM("Zeroconf: \tdomain: " << domain );
        	ROS_DEBUG_STREAM("Zeroconf: \taddress: " << a );
        	ROS_DEBUG_STREAM("Zeroconf: \tport: " << port );
        	ROS_DEBUG_STREAM("Zeroconf: \tinterface: " << interface );

            avahi_free(t);
            break;
        }
    }
}

void discovery_callback(
							AvahiServiceBrowser *b, AvahiIfIndex interface,
							AvahiProtocol protocol, AvahiBrowserEvent event,
							const char *name, const char *type,
							const char *domain, AvahiLookupResultFlags flags,
							void* userdata) {
	Zeroconf *zeroconf = reinterpret_cast<Zeroconf*>(userdata);
    switch (event) {
		case AVAHI_BROWSER_NEW: {
			ROS_INFO_STREAM("Zeroconf: discovered new service [" << name << "][" << type << "][" << domain << "]");
			// Kind of bad - it makes a new resolver but only keeps a pointer to the last

			AvahiServiceResolver* resolver = avahi_service_resolver_new(zeroconf->client, interface, protocol, name, type, domain, AVAHI_PROTO_UNSPEC, static_cast<AvahiLookupFlags>(0), resolve_callback, zeroconf);
			if ( !resolver ) {
				ROS_ERROR_STREAM("Zeroconf: failed to resolve service [" << name << "][" <<  avahi_strerror(avahi_client_errno(zeroconf->client)) << "]");
			} else {
				zeroconf->resolvers.push_back(resolver);
			}
			break;
		}
		case AVAHI_BROWSER_REMOVE: {
			std::string proto_txt;
			if ( protocol == AVAHI_PROTO_INET ) {
				proto_txt = "ipv4";
			} else {
				proto_txt = "ipv6";
			}

			/*********************
			** Logging
			**********************/
			ROS_INFO_STREAM("Zeroconf: service was removed [" << name << "][" << type << "][" << domain << "][" << interface << "][" << proto_txt << "]");
			ROS_DEBUG_STREAM("Zeroconf: \tname: " << name );
			ROS_DEBUG_STREAM("Zeroconf: \ttype: " << type );
			ROS_DEBUG_STREAM("Zeroconf: \tdomain: " << domain );
			ROS_DEBUG_STREAM("Zeroconf: \tinterface: " << interface );
			ROS_DEBUG_STREAM("Zeroconf: \tprotocol: " << proto_txt );
			for ( unsigned int i = 0; i < zeroconf->resolvers.size(); ++i ) {
				avahi_service_resolver_free(zeroconf->resolvers[i]);
			}
			zeroconf->resolvers.clear();
			break;
		}
    }
}

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    const int interface = AVAHI_IF_UNSPEC;
    const int protocol = AVAHI_PROTO_UNSPEC;
    Zeroconf zeroconf;
    /* Allocate main loop object */
    if (!(zeroconf.threaded_poll = avahi_threaded_poll_new())) {
        fprintf(stderr, "Failed to create simple poll object.\n");
        exit(EXIT_FAILURE);
    }
    /* Allocate a new client */
    int error;
    zeroconf.client = avahi_client_new(avahi_threaded_poll_get(zeroconf.threaded_poll), static_cast<AvahiClientFlags>(0), client_callback, &zeroconf, &error);

    /* Check whether creating the client object succeeded */
    if (!zeroconf.client) {
        fprintf(stderr, "Failed to create client: %s\n", avahi_strerror(error));
        exit(EXIT_FAILURE);
    }
    if (!(zeroconf.service_browser = avahi_service_browser_new(zeroconf.client, interface, protocol, "_ros-master._tcp", NULL, static_cast<AvahiLookupFlags>(0), discovery_callback, &zeroconf))) {
        fprintf(stderr, "Failed to create service browser: %s\n", avahi_strerror(avahi_client_errno(zeroconf.client)));
        exit(EXIT_FAILURE);
    }

	avahi_threaded_poll_start(zeroconf.threaded_poll);

	int i = 0;
    while(1) {
//    	if ( i == 5 ) {
//    		std::cout << "Freeing" << std::endl;
//			avahi_threaded_poll_lock(zeroconf.threaded_poll);
//			avahi_service_browser_free(zeroconf.service_browser);
//			avahi_threaded_poll_unlock(zeroconf.threaded_poll);
//    	}
    	++i;
    	sleep(1);
    }
	avahi_threaded_poll_stop(zeroconf.threaded_poll);
	avahi_client_free(zeroconf.client);
	avahi_threaded_poll_free(zeroconf.threaded_poll);


	return 0;
}
