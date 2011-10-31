/**
 * @file /zeroconf_avahi/src/lib/publisher.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 16/08/2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <cstdio>
#include <ctime>
#include <avahi-common/alternative.h> // avahi_alternative_service_name
#include <avahi-common/timeval.h>
#include <zeroconf_comms/Protocols.h>
#include <avahi-common/thread-watch.h>
#include "../../include/zeroconf_avahi/zeroconf.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace zeroconf_avahi {

/*****************************************************************************
** Implementation
*****************************************************************************/

Zeroconf::Zeroconf() :
	invalid_object(false),
	threaded_poll(NULL),
	client(NULL),
	interface(AVAHI_IF_UNSPEC),
	protocol(AVAHI_PROTO_UNSPEC) // AVAHI_PROTO_INET, AVAHI_PROTO_INET6
{
    int error;

    /* Allocate main loop object */
    if (!(threaded_poll = avahi_threaded_poll_new())) {
        fprintf(stderr, "Failed to create simple poll object.\n");
        invalid_object = true;
        return;
    }
    /* Allocate a new client */
    client = avahi_client_new(avahi_threaded_poll_get(threaded_poll), static_cast<AvahiClientFlags>(0), Zeroconf::client_callback, this, &error);

    /* Check whether creating the client object succeeded */
    if (!client) {
        fprintf(stderr, "Failed to create client: %s\n", avahi_strerror(error));
        invalid_object = true;
        return;
    }

    /* Don't start spinning until client_callback detects AVAHI_CLIENT_S_RUNNING. This may mean the user has to wait a
     * little bit before adding a service.
     */
}

Zeroconf::~Zeroconf() {
    if (threaded_poll) {
    	avahi_threaded_poll_stop(threaded_poll);
    }
	/*********************
	** Listeners
	**********************/
	{
		boost::mutex::scoped_lock lock(service_mutex);
		for ( discovery_bimap::left_const_iterator iter = discovery_service_types.left.begin(); iter != discovery_service_types.left.end(); ++iter ) {
			avahi_service_browser_free(iter->first);
		}
		discovery_service_types.clear();
	}
	/*********************
	** Publishers
	**********************/
	// free'ing entry groups shouldn't be necessary (don't know exactly, but haven't seen it in any of the official
	// code, so its probably done when calling avahi_client_free().
	/*********************
	** General
	**********************/
    if (client) {
        avahi_client_free(client);
    }
    if (threaded_poll) {
        avahi_threaded_poll_free(threaded_poll);
    }
}

void Zeroconf::spin() {
	if ( !invalid_object ) {
		ROS_DEBUG("Zeroconf: starting the threaded poll.");
		// does this have a return value I should check?
		avahi_threaded_poll_start(threaded_poll);
	}
}

bool Zeroconf::add_listener(std::string &service_type) {
	avahi_threaded_poll_lock(threaded_poll);
    /* Create the service browser */
	AvahiServiceBrowser *service_browser = NULL;
    if (!(service_browser = avahi_service_browser_new(client, interface, protocol, service_type.c_str(), NULL, static_cast<AvahiLookupFlags>(0), Zeroconf::discovery_callback, this))) {
        fprintf(stderr, "Failed to create service browser: %s\n", avahi_strerror(avahi_client_errno(client)));
        return false;
    }
    {
		boost::mutex::scoped_lock lock(service_mutex);
		discovery_service_types.insert(discovery_bimap::value_type(service_browser,service_type));
	}
	avahi_threaded_poll_unlock(threaded_poll);
	ROS_INFO_STREAM("Zeroconf: added a listener [" << service_type << "]");
	return true;
}
/**
 * Add a service.
 *
 * This goes through the threaded poll, so services can be added
 * in runtime (i.e. after initial configuration).
 *
 * We also keep a copy of the service to be added, so we can track
 * through to the entry_group_callback, exactly which service is
 * getting added.
 */
bool Zeroconf::add_service(const PublishedService &service) {

	avahi_threaded_poll_lock(threaded_poll);
	bool result = add_service_non_threaded(service);
	avahi_threaded_poll_unlock(threaded_poll);
	return result;
}

/**
 * Non threaded add service - only for internal use because we need to be
 * thread-safe.
 */
bool Zeroconf::add_service_non_threaded(const PublishedService &service) {
	/*
	 * We may still be initialising (from constructor)...check that we're up and running."
	 */
	if (avahi_client_get_state(client) != AVAHI_CLIENT_S_RUNNING) {
    	ROS_ERROR("Zeroconf: avahi_client_state is not running (probably still registering with avahi daemon)");
    	return false;
    }

	/*
	 * Check we're not already publishing this exact service
	 */
	{
		boost::mutex::scoped_lock lock(service_mutex);
		service_bimap::right_const_iterator iter;
		iter = committed_services.right.find(service);
		if ( iter != committed_services.right.end() ) {
			ROS_WARN_STREAM("Zeroconf: this node is currently already committing this service [" << service.name << "][" << service.type << "][" << service.port << "]");
			return true;
		}
		iter = established_services.right.find(service);
		if ( iter != established_services.right.end() ) {
			ROS_WARN_STREAM("Zeroconf: this node has already established this service [" << service.name << "][" << service.type << "][" << service.port << "]");
			return true;
		}
	}

	ROS_DEBUG_STREAM("Zeroconf: adding a new service [" << service.name << "][" << service.type << "]");
    /* If this is the first time we're called, let's create a new
     * entry group if necessary */
	AvahiEntryGroup* group = NULL;
	if (!(group = avahi_entry_group_new(client, entry_group_callback, this))) {
		ROS_ERROR_STREAM("Zeroconf: avahi_entry_group_new() failed: " << avahi_strerror(avahi_client_errno(client)));
		fail();
		return false;
	}

	int ret = avahi_entry_group_add_service(
        		group,
        		interface,
        		protocol,
        		static_cast<AvahiPublishFlags>(0), // AVAHI_PUBLISH_USE_MULTICAST - we don't seem to need this, but others said they have needed it.
        		service.name.c_str(),
        		service.type.c_str(),
        		service.domain.c_str(),
        		NULL, 				// automatically sets a hostname for us
        		static_cast<uint16_t>(service.port),
        		NULL);				// txt description
    if (ret < 0) {
        avahi_entry_group_free(group);
        if (ret == AVAHI_ERR_COLLISION) {
        	std::string old_name = service.name;
        	PublishedService new_service = service;
            new_service.name = avahi_alternative_service_name( service.name.c_str());
            ROS_WARN_STREAM("Zeroconf: local service name collision, renaming [" << service.name << "][" << new_service.name << "]");
            return add_service_non_threaded(new_service);
        } else {
			ROS_ERROR_STREAM("Zeroconf: failed to add service [" << service.type.c_str() << "][" << avahi_strerror(ret) << "]");
			fail();
			return false;
        }
    }
    /* Commit the entry group */
    // add to the map first, just so that the callback doesn't fire before we can find it in the map
    {
		boost::mutex::scoped_lock lock(service_mutex);
	    committed_services.insert( service_bimap::value_type(group,service) ); // should check the return value...
    }
    if ((ret = avahi_entry_group_commit(group)) < 0) {
        ROS_ERROR_STREAM("Zeroconf: failed to commit entry group [" <<  avahi_strerror(ret) << "]");
        avahi_entry_group_free(group);
        {
    		boost::mutex::scoped_lock lock(service_mutex);
    		committed_services.left.erase(group);
        }
        fail();
        return false;
    }
    ROS_DEBUG("Zeroconf: service committed, waiting for callback...");
    return true;
}
bool Zeroconf::remove_service(const PublishedService &service) {

	AvahiEntryGroup *group = NULL;
	bool erased = false;
	avahi_threaded_poll_lock(threaded_poll);
	{
		boost::mutex::scoped_lock lock(service_mutex);
		service_bimap::right_const_iterator iter = established_services.right.find(service);
		if ( iter != established_services.right.end() ) {
			group = iter->second;
			avahi_entry_group_reset(group);
			avahi_entry_group_free(group);
			established_services.right.erase( service );
			erased = true;
			ROS_INFO_STREAM("Zeroconf: removing service [" << service.name << "][" << service.type << "]");
		} else {
			ROS_WARN_STREAM("Zeroconf: couldn't remove this service as it is not advertised [" << service.name << "][" << service.type << "]");
		}
	}
	avahi_threaded_poll_unlock(threaded_poll);
	return erased;
}
/**
 * Retrieves all the currently discovered services, or just those of those of the specified type that have
 * been discovered. It goes the extra yard as well, just to make sure those services are resolvable
 * (i.e. haven't dropped out or gone out of wireless range) which is important for robotics.
 *
 * @param service_type : service type specification
 * @param list : list of services that have been discovered (return value)
 */
void Zeroconf::list_discovered_services(const std::string &service_type, std::vector<zeroconf_comms::DiscoveredService> &list) {
	list.clear();
	boost::mutex::scoped_lock lock(service_mutex);
	avahi_threaded_poll_lock(threaded_poll);
	if ( service_type == "" ) {
		for ( discovered_service_set::iterator iter = discovered_services.begin(); iter != discovered_services.end(); ++iter) {
            if (!(avahi_service_resolver_new(client, interface, protocol, iter->name.c_str(), iter->type.c_str(), iter->domain.c_str(), AVAHI_PROTO_UNSPEC, static_cast<AvahiLookupFlags>(0), Zeroconf::resolve_callback, this))) {
            	ROS_ERROR_STREAM("Zeroconf: failed to resolve service [" << iter->name << "][" <<  avahi_strerror(avahi_client_errno(client)) << "]");
            }
			list.push_back(*iter);
		}
	} else {
		for ( discovered_service_set::iterator iter = discovered_services.begin(); iter != discovered_services.end(); ++iter) {
			if ( iter->type == service_type ) {
	            if (!(avahi_service_resolver_new(client, interface, protocol, iter->name.c_str(), iter->type.c_str(), iter->domain.c_str(), AVAHI_PROTO_UNSPEC, static_cast<AvahiLookupFlags>(0), Zeroconf::resolve_callback, this))) {
	            	ROS_ERROR_STREAM("Zeroconf: failed to resolve service [" << iter->name << "][" <<  avahi_strerror(avahi_client_errno(client)) << "]");
	            }
				list.push_back(*iter);
			}
		}
	}
	avahi_threaded_poll_unlock(threaded_poll);
}
void Zeroconf::list_published_services(const std::string &service_type, std::vector<zeroconf_comms::PublishedService> &list) {
	list.clear();
	boost::mutex::scoped_lock lock(service_mutex);
	if ( service_type == "" ) {
		for ( service_bimap::left_const_iterator iter = established_services.left.begin(); iter != established_services.left.end(); ++iter ) {
			list.push_back(iter->second);
		}
	} else {
		for ( service_bimap::left_const_iterator iter = established_services.left.begin(); iter != established_services.left.end(); ++iter ) {
			if ( iter->second.type == service_type ) {
				list.push_back(iter->second);
			}
		}
	}
}

/*****************************************************************************
** Utilities
*****************************************************************************/

int Zeroconf::ros_to_avahi_protocol(const int &protocol) {
	switch (protocol) {
		case (zeroconf_comms::Protocols::UNSPECIFIED) : {
			return AVAHI_PROTO_UNSPEC;
		}
		case (zeroconf_comms::Protocols::IPV4) : {
			return AVAHI_PROTO_INET;
		}
		case (zeroconf_comms::Protocols::IPV6) : {
			return AVAHI_PROTO_INET6;
		}
		default :
			return AVAHI_PROTO_UNSPEC;
	}
}

int Zeroconf::avahi_to_ros_protocol(const int &protocol) {
	switch (protocol) {
		case (AVAHI_PROTO_UNSPEC) : {
			return zeroconf_comms::Protocols::UNSPECIFIED;
		}
		case (AVAHI_PROTO_INET) : {
			return zeroconf_comms::Protocols::IPV4;
		}
		case (AVAHI_PROTO_INET6) : {
			return zeroconf_comms::Protocols::IPV6;
		}
		default :
			return zeroconf_comms::Protocols::UNSPECIFIED;
	}
}

/*****************************************************************************
** Discovery Callbacks
*****************************************************************************/
/**
 * Called whenever a service that is being listened to is added or removed.
 * If you've got no listeners, this doesn't get called.
 *
 * @param browser
 * @param interface
 * @param protocol
 * @param event
 * @param name
 * @param type
 * @param domain
 * @param flags
 * @param userdata
 */
void Zeroconf::discovery_callback(
						AvahiServiceBrowser *browser,
						AvahiIfIndex interface,
						AvahiProtocol protocol,
						AvahiBrowserEvent event,
						const char *name,
						const char *type,
						const char *domain,
						AvahiLookupResultFlags flags,
						void* userdata) {

	Zeroconf *zeroconf = reinterpret_cast<Zeroconf*>(userdata);
    assert(browser);

    /* Called whenever a new services becomes available on the LAN or is removed from the LAN */

    switch (event) {
        case AVAHI_BROWSER_FAILURE:
        	ROS_ERROR_STREAM("Zeroconf: browser failure [" << avahi_strerror(avahi_client_errno(avahi_service_browser_get_client(browser))) );
            avahi_threaded_poll_quit(zeroconf->threaded_poll);
            return;

        case AVAHI_BROWSER_NEW: {
        	ROS_INFO_STREAM("Zeroconf: discovered new service [" << name << "][" << type << "][" << domain << "]");
            /* We ignore the returned resolver object. In the callback
               function we free it. If the server is terminated before
               the callback function is called the server will free
               the resolver for us. */
            if (!(avahi_service_resolver_new(zeroconf->client, interface, protocol, name, type, domain, AVAHI_PROTO_UNSPEC, static_cast<AvahiLookupFlags>(0), Zeroconf::resolve_callback, zeroconf))) {
            	ROS_ERROR_STREAM("Zeroconf: failed to resolve service [" << name << "][" <<  avahi_strerror(avahi_client_errno(zeroconf->client)) << "]");
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
			** Signals
			**********************/
    		zeroconf_comms::DiscoveredService service;
    		service.name = name;
    		service.type = type;
    		service.domain = domain;
    		service.hardware_interface = interface;
    		service.protocol = zeroconf->avahi_to_ros_protocol(protocol);
			if ( zeroconf->lost_connection_signal ) {
				zeroconf->lost_connection_signal(service);
			}
        	/*********************
			** Logging
			**********************/
        	ROS_INFO_STREAM("Zeroconf: service was removed [" << name << "][" << type << "][" << domain << "][" << interface << "][" << proto_txt << "]");
        	ROS_DEBUG_STREAM("Zeroconf: \tname: " << name );
        	ROS_DEBUG_STREAM("Zeroconf: \ttype: " << type );
        	ROS_DEBUG_STREAM("Zeroconf: \tdomain: " << domain );
        	ROS_DEBUG_STREAM("Zeroconf: \tinterface: " << interface );
        	if ( service.protocol == zeroconf_comms::Protocols::IPV4 ) {
            	ROS_DEBUG("Zeroconf: \tprotocol: ipv4");
        	} else {
            	ROS_DEBUG("Zeroconf: \tprotocol: ipv6");
        	}

        	/*********************
			** InHouse Removal
			**********************/
			{
				boost::mutex::scoped_lock lock(zeroconf->service_mutex);
				discovered_service_set::iterator iter = zeroconf->discovered_services.begin();
				while ( iter != zeroconf->discovered_services.end() ) {
					if ( ( iter->name == name ) && ( iter->type == type ) && ( iter->domain == domain ) && ( iter->hardware_interface == interface ) && ( iter->protocol == service.protocol )) {
						/*********************
						** Detailed Logging
						**********************/
			        	ROS_DEBUG_STREAM("Zeroconf: \thostname: " << iter->hostname );
			        	ROS_DEBUG_STREAM("Zeroconf: \taddress: " << iter->address );
			        	ROS_DEBUG_STREAM("Zeroconf: \tport: " << iter->port );
			        	ROS_DEBUG_STREAM("Zeroconf: \tdescription: " << iter->description );
			        	ROS_DEBUG_STREAM("Zeroconf: \tcookie: " << iter->cookie );
			        	ROS_DEBUG_STREAM("Zeroconf: \tis_local: " << (iter->is_local ? 1 : 0 ) );
			        	ROS_DEBUG_STREAM("Zeroconf: \tour_own: " << (iter->our_own ? 1 : 0 ) );
			        	ROS_DEBUG_STREAM("Zeroconf: \twide_area: " << (iter->wide_area ? 1 : 0 ) );
			        	ROS_DEBUG_STREAM("Zeroconf: \tmulticast: " << (iter->multicast ? 1 : 0 ) );
						zeroconf->discovered_services.erase(iter++);
					} else {
						++iter;
					}
				}
			}
            break;
        }
        case AVAHI_BROWSER_ALL_FOR_NOW:
        case AVAHI_BROWSER_CACHE_EXHAUSTED: {
        	if ( event == AVAHI_BROWSER_CACHE_EXHAUSTED ) {
        		ROS_DEBUG("Zeroconf: browser event occured [cache exhausted]" );
        	} else {
        		ROS_DEBUG("Zeroconf: browser event occured [all for now]" );
        	}
            break;
       	}
    }
}
/**
 * Just makes sure that you can actually resolve a discovered service whenever a
 * new service is discovered on the lan.
 *
 * @param resolver
 * @param interface
 * @param protocol
 * @param event
 * @param name
 * @param type
 * @param domain
 * @param host_name
 * @param address
 * @param port
 * @param txt
 * @param flags
 * @param userdata
 */
void Zeroconf::resolve_callback(
    AvahiServiceResolver *resolver,
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
    assert(resolver);

    /* Called whenever a service has been resolved successfully or timed out */

    switch (event) {
        case AVAHI_RESOLVER_FAILURE: {
        	ROS_ERROR_STREAM("Zeroconf: failed to resolve service [" << name << "][" <<  type << "][" << domain << "][" << avahi_strerror(avahi_client_errno(avahi_service_resolver_get_client(resolver))) << "]");
        	// If it has failed to resolve, the only meaningful information we have is name, type and domain.
        	// Hardware interface and protocol may also be valid - I'm not sure as with quick testing, it may have just been showing defaults of quick testing was just showing up defaults of 2 and 0.
            // std::cout << "Hardware Interface: " << interface << std::endl;
            // std::cout << "Protocol" << protocol << std::endl;
    		discovered_service_set::iterator iter = zeroconf->discovered_services.begin();
    		while ( iter != zeroconf->discovered_services.end() ) {
    			if ( ( iter->name == name ) &&
    				 ( iter->type == type ) &&
    				 ( iter->domain == domain ) ) { // might need to also add interface and protocol checks in here.
        			ROS_ERROR_STREAM("An existing service failed to resolve, removing it from the discovered services list.");
    				zeroconf->discovered_services.erase(iter++); // this is probably going to bomb as we're deleting from the avahi thread
    			} else { iter++; }
    		}
            break;
        }
        case AVAHI_RESOLVER_FOUND: {
            char a[AVAHI_ADDRESS_STR_MAX], *t;
            t = avahi_string_list_to_string(txt);
            avahi_address_snprint(a, sizeof(a), address);

            /*********************
			** Saving Details
			**********************/
            zeroconf_comms::DiscoveredService service;
            service.name = name;
            service.type = type;
            service.domain = domain;
            service.hostname = host_name;
            service.port = port;
            service.address = a;
    		service.hardware_interface = interface;
    		if ( protocol == AVAHI_PROTO_INET ) {
    			service.protocol = zeroconf_comms::Protocols::IPV4;
    		} else {
    			service.protocol = zeroconf_comms::Protocols::IPV6;
    		}
            service.description = t;
            service.cookie = avahi_string_list_get_service_cookie(txt);
            service.is_local = ((flags & AVAHI_LOOKUP_RESULT_LOCAL) == 0 ? false : true );
            service.our_own = ((flags & AVAHI_LOOKUP_RESULT_OUR_OWN) == 0 ? false : true );
            service.wide_area = ((flags & AVAHI_LOOKUP_RESULT_WIDE_AREA) == 0 ? false : true );
            service.multicast = ((flags & AVAHI_LOOKUP_RESULT_MULTICAST) == 0 ? false : true );
            service.cached = ((flags & AVAHI_LOOKUP_RESULT_CACHED) == 0 ? false : true );
			{
				boost::mutex::scoped_lock lock(zeroconf->service_mutex);
				if ( (zeroconf->discovered_services.insert(service)).second ) {
					// std::cout << "Inserted a new discovered sevice." << std::endl;
				} else {
					/* In this case, the callback is a result of the zeroconf just trying to resolve
					 * an already discovered service just to check if its still resolvable (i.e. not
					 * dropped out or gone out of wireless range). */
					// std::cout << "Tried inserting an existing discovered sevice." << std::endl;
				}
			}
			/*********************
			** Signals
			**********************/
			if ( zeroconf->new_connection_signal ) {
				zeroconf->new_connection_signal(service);
			}
			/*********************
			** Logging
			**********************/
        	ROS_INFO_STREAM("Zeroconf: discovered new service [" << name << "][" << type << "][" << domain << "][" << service.address << "]");
        	ROS_DEBUG_STREAM("Zeroconf: \tname: " << service.name );
        	ROS_DEBUG_STREAM("Zeroconf: \ttype: " << service.type );
        	ROS_DEBUG_STREAM("Zeroconf: \tdomain: " << service.domain );
        	ROS_DEBUG_STREAM("Zeroconf: \thostname: " << service.hostname );
        	ROS_DEBUG_STREAM("Zeroconf: \taddress: " << service.address );
        	ROS_DEBUG_STREAM("Zeroconf: \tport: " << service.port );
        	ROS_DEBUG_STREAM("Zeroconf: \tinterface: " << service.hardware_interface );
        	if ( service.protocol == zeroconf_comms::Protocols::IPV4 ) {
            	ROS_DEBUG("Zeroconf: \tprotocol: ipv4");
        	} else {
            	ROS_DEBUG("Zeroconf: \tprotocol: ipv6");
        	}
        	ROS_DEBUG_STREAM("Zeroconf: \tdescription: " << service.description );
        	ROS_DEBUG_STREAM("Zeroconf: \tcookie: " << service.cookie );
        	ROS_DEBUG_STREAM("Zeroconf: \tis_local: " << (service.is_local ? 1 : 0 ) );
        	ROS_DEBUG_STREAM("Zeroconf: \tour_own: " << (service.our_own ? 1 : 0 ) );
        	ROS_DEBUG_STREAM("Zeroconf: \twide_area: " << (service.wide_area ? 1 : 0 ) );
        	ROS_DEBUG_STREAM("Zeroconf: \tmulticast: " << (service.multicast ? 1 : 0 ) );

            avahi_free(t);
            break;
        }
    }
    avahi_service_resolver_free(resolver);
}


/*****************************************************************************
** Publisher Callback
*****************************************************************************/
/**
 * Called whenever the the state of the entry group changes (or in english,
 * whenever something noteworthy has happened to advertisements).
 *
 * Not sure exactly, but after some testing, it seems that this also locks the
 * threaded poll mutex for certain state changes, most notably, 'registering'
 * and 'established'.
 *
 * Making a big assumption here, but assuming that the callbacks will get
 * processed for registering services in the order that they were
 * committed. If this is true, we just look up the first service in the
 * committed_services variable to see which service we're handling.
 **/
void Zeroconf::entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state, void *userdata) {

	Zeroconf *zeroconf = static_cast<Zeroconf*>(userdata);
    switch (state) {
        case AVAHI_ENTRY_GROUP_ESTABLISHED : {
			PublishedService service;
			{
				boost::mutex::scoped_lock lock(zeroconf->service_mutex);
				service_bimap::left_const_iterator left = zeroconf->committed_services.left.find(g);
				if ( left != zeroconf->committed_services.left.end() ) {
					service = left->second;
				} else {
					ROS_ERROR("Zeroconf : should never reach here, please report a bug in zeroconf_avahi's entry_group_callback.");
					return;
				}
				zeroconf->established_services.insert( service_bimap::value_type(g,service));
				zeroconf->committed_services.left.erase(g);
			}
			ROS_INFO_STREAM("Zeroconf: service successfully established ["  << service.name << "][" << service.type << "][" << service.port << "]");
            break;
        }
        case AVAHI_ENTRY_GROUP_COLLISION : {
			/* A service name collision with a 'remote' service happened. Let's pick a new name */
			PublishedService service;
        	{
        		boost::mutex::scoped_lock lock(zeroconf->service_mutex);
				service_bimap::left_const_iterator left = zeroconf->committed_services.left.find(g);
				if ( left != zeroconf->committed_services.left.end() ) {
					service = left->second;
				} else {
					ROS_ERROR("Zeroconf : should never reach here, please report a bug in zeroconf_avahi's entry_group_callback.");
					return;
				}
				zeroconf->committed_services.left.erase(g);
        	}
            std::string alternative_name = avahi_alternative_service_name(service.name.c_str());
            ROS_INFO_STREAM("Zeroconf: service name collision, renaming service [" << service.name << "]" << "][" << alternative_name << "]");
            service.name = alternative_name;
            avahi_entry_group_free(g);
            /* And recreate the services - already in the poll thread, so don' tneed to lock. */
            zeroconf->add_service_non_threaded(service);
            break;
        }

        case AVAHI_ENTRY_GROUP_FAILURE :
            /* Some kind of failure happened while we were registering our services */
        	// drop our committed_service here.
        	ROS_DEBUG_STREAM("Zeroconf: group state changed, system failure when trying to register service [" << avahi_strerror(avahi_client_errno(avahi_entry_group_get_client(g))) << "]");
            avahi_entry_group_free(g);
            zeroconf->fail();
            break;

        case AVAHI_ENTRY_GROUP_UNCOMMITED: {
        	ROS_DEBUG_STREAM("Zeroconf: group state changed, service uncommitted");
        	// This is just mid-process, no need to handle committed_services
        	break;
        }
        case AVAHI_ENTRY_GROUP_REGISTERING: {
        	// This is just mid-process, no need to handle committed_services
			PublishedService service;
        	{
				boost::mutex::scoped_lock lock(zeroconf->service_mutex);
				service_bimap::left_const_iterator left = zeroconf->committed_services.left.find(g);
				if ( left != zeroconf->committed_services.left.end() ) {
					service = left->second;
				} else {
					ROS_ERROR("Zeroconf : should never reach here, please report a bug in zeroconf_avahi's entry_group_callback.");
					return;
				}
        	}
        	ROS_DEBUG_STREAM("Zeroconf: group state changed, service registering [" << service.name << "][" << service.type << "]");
        	break;
        }
        default: {
        	ROS_DEBUG_STREAM("Zeroconf: group state changed, ended in an unknown state [" << state << "]");
        	break;
        }
    }
}

/*****************************************************************************
** Daemon Callbacks
*****************************************************************************/
/**
 * Used to update client connection with the avahi daemon. This gets called in
 * Zeroconf's construction where it checks to see that the avahi-daemon is
 * actually up and running.
 *
 * @param c : client that is connecting.
 * @param state : updated state for the client-daemon connection.
 * @param userdata : this will always be the Zeroconf class.
 */
void Zeroconf::client_callback(AvahiClient *c, AvahiClientState state, void *userdata) {

	Zeroconf *zeroconf = static_cast<Zeroconf*>(userdata);
    assert(c);

    /* Called whenever the client or server state changes */

    switch (state) {
        case AVAHI_CLIENT_S_RUNNING: {
            /* The server has startup successfully and registered its host
             * name on the network, so it's time to fire up */
        	ROS_DEBUG("Zeroconf: avahi client up and running.");
        	zeroconf->spin();
            break;
        }
        case AVAHI_CLIENT_FAILURE: {
        	ROS_ERROR_STREAM("Zeroconf: avahi client failure [" << avahi_strerror(avahi_client_errno(c)) << "]");
            zeroconf->fail();

            break;
        }
        case AVAHI_CLIENT_S_COLLISION: {
        	ROS_DEBUG("Zeroconf: avahi client collision.");
            /* Let's drop our registered services. When the server is back
             * in AVAHI_SERVER_RUNNING state we will register them
             * again with the new host name. */
        	break;
        }
        case AVAHI_CLIENT_S_REGISTERING: {
        	ROS_DEBUG("Zeroconf: avahi client registering.");

            /* The server records are now being established. This
             * might be caused by a host name change. We need to wait
             * for our own records to register until the host name is
             * properly established. */

        	// official example resets the entry group here
        	// since we have multiple groups, handling should be a bit more
        	// complicated - ToDo
        	break;
        }
        case AVAHI_CLIENT_CONNECTING: {
        	ROS_DEBUG("Zeroconf: avahi client registering.");
        	break;
        }
    }
}


} // namespace zeroconf_avahi
