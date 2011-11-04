/**
 * @file /zeroconf_avahi/src/bin/zeroconf.cpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 05/09/2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <boost/function.hpp>

#include <ros/ros.h>
#include <zeroconf_comms/AddListener.h>
#include <zeroconf_comms/RemoveListener.h>
#include <zeroconf_comms/AddService.h>
#include <zeroconf_comms/ListDiscoveredServices.h>
#include <zeroconf_comms/ListPublishedServices.h>

#include "../../include/zeroconf_avahi/zeroconf.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

using namespace zeroconf_avahi;
using namespace zeroconf_comms;

/*****************************************************************************
** Interface
*****************************************************************************/

class ZeroconfNode {

private:
	typedef boost::function<void (zeroconf_comms::DiscoveredService)> connection_signal_cb;

public:
	void init( ros::NodeHandle &nh) {
		/*********************
		** Ros Comms
		**********************/
		server_add_listener = nh.advertiseService("add_listener", &ZeroconfNode::add_listener, this);
		server_remove_listener = nh.advertiseService("remove_listener", &ZeroconfNode::remove_listener, this);
		server_add_service = nh.advertiseService("add_service", &ZeroconfNode::add_service, this);
		server_remove_service = nh.advertiseService("remove_service", &ZeroconfNode::remove_service, this);
		server_list_discovered_services = nh.advertiseService("list_discovered_services", &ZeroconfNode::list_discovered_services, this);
		server_list_published_services = nh.advertiseService("list_published_services", &ZeroconfNode::list_published_services, this);
		pub_new_connections = nh.advertise<DiscoveredService>("new_connections",10);
		pub_lost_connections = nh.advertise<DiscoveredService>("lost_connections",10);

		/*********************
		** Signals
		**********************/
		// The callback functions use the publishers, so they must come after.
		connection_signal_cb new_connection_signal = std::bind1st(std::mem_fun(&ZeroconfNode::new_connections), this);
		connection_signal_cb old_connection_signal = std::bind1st(std::mem_fun(&ZeroconfNode::lost_connections), this);
		zeroconf.connect_signal_callbacks(new_connection_signal, old_connection_signal);

		/*********************
		** Parameters
		**********************/
		ros::NodeHandle local_nh("~");
		// listeners
	    XmlRpc::XmlRpcValue value;
	    XmlRpc::XmlRpcValue::iterator iter;
	    if ( local_nh.getParam("listeners", value) ) {
		    if ( value.getType() != XmlRpc::XmlRpcValue::TypeArray ) {
		        ROS_ERROR_STREAM("Zeroconf: param variable 'listeners' has malformed type, should be of type array");
		    } else {
			    for ( int i = 0; i < value.size(); ++i ) {
			    	zeroconf.add_listener(value[i]);
			    }
		    }
	    }
		// services
	    XmlRpc::XmlRpcValue value_services;
	    if ( local_nh.getParam("services", value_services) ) {
		    if ( value_services.getType() != XmlRpc::XmlRpcValue::TypeArray ) {
		        ROS_ERROR_STREAM("Zeroconf: param variable 'services' has malformed type, should be of type array");
		    } else {
			    for ( int i = 0; i < value_services.size(); ++i ) {
			    	XmlRpc::XmlRpcValue value_service = value_services[i];
			        if ( value_service.getType() != XmlRpc::XmlRpcValue::TypeStruct ) {
				        ROS_ERROR_STREAM("Zeroconf: " << i << "th element of param variable 'services' has malformed type, should be of type struct");
			            break;
			        }
					zeroconf_comms::PublishedService service;
			        XmlRpc::XmlRpcValue value_name = value_service["name"];
			        if ( value_name.getType() != XmlRpc::XmlRpcValue::TypeString ) {
				        ROS_ERROR_STREAM("Zeroconf: services[" << i << "]['name'] has malformed type, should be of type string");
			            break;
			        }
			        service.name = std::string(value_name);
			        XmlRpc::XmlRpcValue value_type = value_service["type"];
			        if ( value_type.getType() != XmlRpc::XmlRpcValue::TypeString ) {
				        ROS_ERROR_STREAM("Zeroconf: services[" << i << "]['type'] has malformed type, should be of type string");
			            break;
			        }
			        service.type = std::string(value_type);
			        XmlRpc::XmlRpcValue value_domain = value_service["domain"];
			        if ( value_domain.getType() != XmlRpc::XmlRpcValue::TypeString ) {
				        ROS_ERROR_STREAM("Zeroconf: services[" << i << "]['domain'] has malformed type, should be of type string");
			            break;
			        }
			        service.domain = std::string(value_domain);
			        XmlRpc::XmlRpcValue value_port = value_service["port"];
			        if ( value_port.getType() == XmlRpc::XmlRpcValue::TypeInvalid ) {
			        	service.port = ros::master::getPort();
			        	// wasn't set, so default to ros master's port
				        ROS_WARN_STREAM("Zeroconf: services[" << service.name << "]['port'] wasn't set, default to ros master's port [" << service.port << "]");
			        } else if ( value_port.getType() != XmlRpc::XmlRpcValue::TypeInt ) {
				        ROS_ERROR_STREAM("Zeroconf: services[" << service.name << "]['port'] has malformed type, should be of type int");
			            break;
			        } else {
			        	service.port = value_port;
			        }
			        XmlRpc::XmlRpcValue value_description = value_service["description"];
			        if ( value_description.getType() != XmlRpc::XmlRpcValue::TypeString ) {
				        ROS_ERROR_STREAM("Zeroconf: services[" << i << "]['description'] has malformed type, should be of type string");
			            break;
			        }
			        service.description = std::string(value_description);
			        zeroconf.add_service(service);
			    }
		    }
	    }
	}

private:
	bool add_listener(AddListener::Request &request, AddListener::Response &response) {
		ROS_DEBUG_STREAM("Zeroconf: serving an add_listener call... [" << request.service_type << "]");
		response.result = zeroconf.add_listener(request.service_type);
		return true;
	}
	bool remove_listener(RemoveListener::Request &request, RemoveListener::Response &response) {
		ROS_DEBUG_STREAM("Zeroconf: serving a remove_listener call... [" << request.service_type << "]");
		response.result = zeroconf.remove_listener(request.service_type);
		return true;
	}
	bool add_service(AddService::Request &request, AddService::Response &response) {
		response.result = zeroconf.add_service(request.service);
		return true;
	}
	bool remove_service(AddService::Request &request, AddService::Response &response) {
		response.result = zeroconf.remove_service(request.service);
		return true;
	}
	bool list_discovered_services(ListDiscoveredServices::Request &request, ListDiscoveredServices::Response &response) {
		ROS_DEBUG_STREAM("Zeroconf: serving a list_discovered_services call... [" << request.service_type << "]");
		zeroconf.list_discovered_services(request.service_type, response.services);
		return true;
	}
	bool list_published_services(ListPublishedServices::Request &request, ListPublishedServices::Response &response) {
		zeroconf.list_published_services(request.service_type, response.services);
		return true;
	}
	void new_connections(DiscoveredService service) {
		pub_new_connections.publish(service);
	}
	void lost_connections(DiscoveredService service) {
		pub_lost_connections.publish(service);
	}

	Zeroconf zeroconf;
	ros::ServiceServer server_add_listener, server_remove_listener, server_add_service, server_remove_service;
	ros::ServiceServer server_list_discovered_services, server_list_published_services;
	ros::Publisher pub_new_connections, pub_lost_connections;
};

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	ros::init(argc, argv, "zeroconf");
	ros::NodeHandle nh;
	ZeroconfNode zeroconf_node;
	zeroconf_node.init(nh);
	ros::spin();
	return 0;
}
