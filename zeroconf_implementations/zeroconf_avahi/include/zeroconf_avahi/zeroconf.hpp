/**
 * @file /zeroconf_avahi/include/zeroconf_avahi/zeroconf.hpp
 *
 * @brief Zeroconf client class for avahi.
 *
 * @date 16/08/2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>

#include <avahi-client/publish.h>
#include <avahi-client/client.h>
#include <avahi-client/lookup.h>
#include <avahi-common/malloc.h>
#include <avahi-common/error.h>
#include <avahi-common/thread-watch.h>

#include <boost/thread/mutex.hpp>
#include <boost/bimap/bimap.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>

#include <zeroconf_comms/PublishedService.h>
#include <zeroconf_comms/DiscoveredService.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace zeroconf_avahi {

/*****************************************************************************
** Utilities
*****************************************************************************/
/**
 * Required comparison functor for use of PublishedService types in an
 * ordered set.
 *
 * Merge this with the DiscoveredServiceCompare if we don't end up needing
 * a separate interface (i.e. use a template class).
 */
struct PublishedServiceCompare {
	bool operator() (const zeroconf_comms::PublishedService &a, const zeroconf_comms::PublishedService &b) const {
		if ( a.name != b.name ) {
			return a.name < b.name;
		} else if ( a.type != b.type ) {
			return a.type < b.type;
		} else {
			return a.port < b.port;
		}
	}
};

struct DiscoveredServiceCompare {
	bool operator() (const zeroconf_comms::DiscoveredService &a, const zeroconf_comms::DiscoveredService &b) const {
		if ( a.name != b.name ) {
			return a.name < b.name;
		} else if ( a.type != b.type ) {
			return a.type < b.type;
		} else if ( a.domain != b.domain ) {
			return a.domain < b.domain;
		} else if ( a.hostname != b.hostname ) {
			return a.hostname < b.hostname;
		} else if ( a.address != b.address ) {
			return a.address < b.address;
		} else if ( a.port != b.port ) {
			return a.port < b.port;
		} else if ( a.hardware_interface != b.hardware_interface ) {
			return a.hardware_interface < b.hardware_interface;
		} else {
			return a.protocol < b.protocol;
		}
	}
};
/**
 * It's important that a discovered service not only records all its characteristics,
 * but keeps it's resolver open so that it can update if the remote end disconnects,
 * reconnects or even disconnects and is then superceded by a new connection.
 */
class DiscoveredAvahiService {
public:
	DiscoveredAvahiService() : resolver(NULL) {}
	DiscoveredAvahiService(zeroconf_comms::DiscoveredService &discovered_service, AvahiServiceResolver *new_resolver ) :
		service(discovered_service),
		resolver(new_resolver) {}
	~DiscoveredAvahiService() {
		avahi_service_resolver_free(resolver);
	}
	zeroconf_comms::DiscoveredService service;
	AvahiServiceResolver *resolver;
};

struct DiscoveredAvahiServiceCompare {
	bool operator() (const boost::shared_ptr<DiscoveredAvahiService> avahi_service_a, const boost::shared_ptr<DiscoveredAvahiService> avahi_service_b) const {
		const zeroconf_comms::DiscoveredService &a = avahi_service_a->service;
		const zeroconf_comms::DiscoveredService &b = avahi_service_b->service;
		if ( a.name != b.name ) {
			return a.name < b.name;
		} else if ( a.type != b.type ) {
			return a.type < b.type;
		} else if ( a.domain != b.domain ) {
			return a.domain < b.domain;
		} else if ( a.hostname != b.hostname ) {
			return a.hostname < b.hostname;
		} else if ( a.address != b.address ) {
			return a.address < b.address;
		} else if ( a.port != b.port ) {
			return a.port < b.port;
		} else if ( a.hardware_interface != b.hardware_interface ) {
			return a.hardware_interface < b.hardware_interface;
		} else {
			return a.protocol < b.protocol;
		}
	}
};


/*****************************************************************************
** Interfaces
*****************************************************************************/

class Zeroconf {
private:
	typedef zeroconf_comms::PublishedService PublishedService;
	typedef boost::bimaps::bimap<AvahiEntryGroup*, boost::bimaps::set_of<PublishedService,PublishedServiceCompare> > service_bimap;
	typedef boost::bimaps::bimap<AvahiServiceBrowser*, boost::bimaps::set_of<std::string> > discovery_bimap;
	//typedef std::set<zeroconf_comms::DiscoveredService,DiscoveredServiceCompare> discovered_service_set;
	typedef std::set<boost::shared_ptr<DiscoveredAvahiService>,DiscoveredAvahiServiceCompare> discovered_service_set;
	typedef std::pair<AvahiEntryGroup*,PublishedService > service_map_pair;
	typedef boost::function<void (zeroconf_comms::DiscoveredService)> connection_signal_cb;

public:
	Zeroconf();
	~Zeroconf();
	bool add_service(const PublishedService &service);
	bool remove_service(const PublishedService &service);
	// Todo : would be useful to be able to remove services in various other ways
	// bool remove_services_by_name(const std::string& service_name);
	// bool remove_services_by_type(const std::string& service_type);
	// bool remove_services() <-- remove *all* services
	bool add_listener(std::string &service_type);
	void list_discovered_services(const std::string &service_type, std::vector<zeroconf_comms::DiscoveredService> &list);
	void list_published_services(const std::string &service_type, std::vector<zeroconf_comms::PublishedService> &list);

	void connect_signal_callbacks( connection_signal_cb new_connections, connection_signal_cb lost_connections ) {
		new_connection_signal = new_connections;
		lost_connection_signal = lost_connections;
	}

	void spin();

private:
	bool invalid_object;
	AvahiThreadedPoll *threaded_poll;
    AvahiClient *client;
    service_bimap committed_services;
    service_bimap established_services;
    discovery_bimap discovery_service_types;
    discovered_service_set discovered_services;
    boost::mutex service_mutex;
    const int interface;
    const int protocol;
    connection_signal_cb new_connection_signal, lost_connection_signal;

    int ros_to_avahi_protocol(const int &protocol);
    int avahi_to_ros_protocol(const int &protocol);
    std::string avahi_to_txt_protocol(const int &protocol);

	bool add_service_non_threaded(const PublishedService &service);
	void fail() { avahi_threaded_poll_quit(threaded_poll); invalid_object = true; }

	static void entry_group_callback(AvahiEntryGroup *g, AvahiEntryGroupState state, void *userdata);
	static void client_callback(AvahiClient *c, AvahiClientState state,void * userdata);
	static void discovery_callback(
							AvahiServiceBrowser *b, AvahiIfIndex interface,
							AvahiProtocol protocol, AvahiBrowserEvent event,
							const char *name, const char *type,
							const char *domain, AvahiLookupResultFlags flags,
							void* userdata);
	static void resolve_callback(
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
							void* userdata);

	static void modify_callback(AVAHI_GCC_UNUSED AvahiTimeout *e, void *userdata);
};


} // namespace zeroconf_avahi
