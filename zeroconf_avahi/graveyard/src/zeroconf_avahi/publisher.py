'''
Created on 01/08/2011

@author: Daniel Stonier
'''
##############################################################################
# Imports
##############################################################################

import avahi
import dbus
import gobject
import socket

import roslib; roslib.load_manifest('zeroconf_avahi')
import rospy

from zeroconf_comms.msg import *
from zeroconf_comms.srv import *

import utilities
from .dbus_engine import DBusEngine

##############################################################################
# Publisher
##############################################################################

'''
  Announces a ros master on the zeroconf network. 
  
  This is a wrapper around the connections to the avahi 
  server which will publish details about this ros master
  to the network.
  
  Configuration is via ros parameters:
  
  - master_name : name string given by the robot [Unknown]
  - service_type : avahi service type [_ros-master._tcp]
  - domain : the domain for this zeroconf network [local]
  - hostname : hostname [computer's hostname setting]
  - port : port number the ros master is running on [11311]
  
  Most of the time you'll only be interested in setting the 
  robot name. For a generic rosmaster the service type will
  always be _ros-master._tcp, the domain is usually just 'local' 
  and the hostname, port pair are automatically obtained from
  the currently executing ros master that this node is 
  attached to.
'''
class Publisher():
    '''
      Parameterise from the ros parameter server, 
      initialise the connection to the avahi server
      and finally publish (add the service) when 
      the avahi connection says its ready.
      
      Assumption: it has already initialised ros and has access to 
      ros logging, publishers and services.
      
      It also allows overrides of all parameters of the parameter lookup for the port
      number so this module can be used by someone can grab an unused
      port and then publish rather than assume a pre-fixed ros configuration.
    '''
    def __init__(self, dbus_engine):
        
        self._service = None
        self._dbus_engine = dbus_engine
        # The DBUS entry group
        self._group = None
        # Monitor server state changes
        self._dbus_engine._server.connect_to_signal( "StateChanged", self.state_changed_callback )
        # Build initial state information
        self.state_changed_callback( self._dbus_engine._server.GetState() )
        self._rename_count_limit = 12
        self._rename_count = self._rename_count_limit
        
        self._advertise_service_server = rospy.Service("~advertise_service",zeroconf_comms.srv.AdvertiseService, self.advertise_service)

    def __del__(self):
        if self._group is not None:
            self._group.Free()

    def provide_service_info(self, req):
        response = zeroconf_comms.srv.AdvertisedServiceResponse()
        if ( self._service is None ):
            rospy.logdebug("Zeroconf: request for service details [none][%s]"%rospy.get_name())
            response.service = zeroconf_comms.msg.Service()
        else:
            rospy.logdebug("Zeroconf: request for service details [%s]"%rospy.get_name())
            response = self._service
        return response
    
    def advertise_service(self,req):
        response = zeroconf_comms.srv.AdvertiseServiceResponse()
        req.service = self.check_service_details(req.service)
        # Currently can't add service when one already running (bug!)
        if self._service is None:
            rospy.logdebug("Zeroconf: subscriber received request to add service '%s'\n%s" % ( req.service.name, utilities.service_to_str(req.service)))
            self.add_service(req.service.name, req.service.service_type, req.service.domain, req.service.hostname, req.service.port)
            response.result = True
        else:
            response.result = False
            rospy.logwarn("Zeroconf: already advertising service '%s'\n%s" % ( req.service.name, utilities.service_to_str(req.service)))
        return response
    
    def state_changed_callback(self,state):
        '''
           Called when the state of the avahi server changes. Useful to us directly after
           connection when the initialised state changes to running. Subsequently this
           will usually run directly at the end of __init__.
        '''
        if state == avahi.SERVER_COLLISION:
            rospy.logwarn("Zeroconf: server name collision [%s]",rospy.get_name())
            self.remove_service()
        elif state == avahi.SERVER_RUNNING:
            rospy.logdebug("Zeroconf: server is up and running [%s]",rospy.get_name())
        elif state == avahi.SERVER_INVALID:
            rospy.logerr("Zeroconf: avahi server is invalid [%s]",rospy.get_name())
        elif state == avahi.SERVER_FAILURE:
            rospy.logerr("Zeroconf: avahi server failed [%s]",rospy.get_name())
        else:
            rospy.logerr("Zeroconf: unknown problem with the avahi server [%s]",rospy.get_name())
    
    def check_service_details(self, service):
        '''
          Check the service details and set good defaults if not set. 
        '''
        if service.service_type == "":
            service.service_type = "_ros-master._tcp"
        if service.domain == "":
            service.domain = "local"
        if service.hostname == "":
            service.hostname = roslib.network.get_host_name()
        if service.port == 0:
            service.port = roslib.network.parse_http_host_and_port(roslib.rosenv.get_master_uri())[1]
        return service
            
    def add_service(self, 
                    name = "Ros Master", 
                    service_type = "_ros-master._tcp", 
                    domain = "local", 
                    hostname = socket.gethostname(), # roslib.network.get_host_name(), 
                    port = roslib.network.parse_http_host_and_port(roslib.rosenv.get_master_uri())[1] 
                    ):
        '''
          Advertise the specified service on zeroconf.
        '''
        self._advertise_service_server.shutdown("spinning up the dbus main loop - currently no more changes can be made")
        
        if self._group is None:
            rospy.logdebug("Zeroconf: creating entry group[%s]" % ( rospy.get_name()))
            self._group = dbus.Interface(
                    self._dbus_engine._bus.get_object(avahi.DBUS_NAME, self._dbus_engine._server.EntryGroupNew()),
                    avahi.DBUS_INTERFACE_ENTRY_GROUP)
            self._group.connect_to_signal('StateChanged', self.group_state_changed_callback)
            # We can introspect the methods available to group here;
            # print self._dbus_engine._bus.get_object(avahi.DBUS_NAME,self._dbus_engine._server.EntryGroupNew()).Introspect()
        
        if self._service is not None:
            rospy.logwarn("Zeroconf: already advertising [%s][%s]" % ( self._service.name, rospy.get_name()))
            return

        self._service = zeroconf_comms.msg.Service()
        self._service.name = name
        self._service.service_type = service_type
        self._service.service_name = utilities.service_name(service_type)
        self._service.domain = domain
        self._service.hostname = socket.gethostname() # overriding this for now - getting hostname from ros often gets things muddled because of ROS_IP.
        self._service.port = port
        self._service.description = ""

        rospy.logdebug("Zeroconf: attempting to advertise service '%s'\n%s" % ( self._service.name, utilities.service_to_str(self._service)))
        
        # This is annoying, local collisions dont go through to the group_state_changed callback.
        # I'm handling them here in my ignorance for now.
        # Also, we're only publishing ipv4 services since ros can't really handle anything else yet..
        try:
            self._group.AddService(
                avahi.IF_UNSPEC,
                avahi.PROTO_INET,
                #avahi.PROTO_UNSPEC,
                dbus.UInt32(0),
                self._service.name, 
                self._service.service_type,
                self._service.domain,
                self._service.hostname + "." + self._service.domain,
                dbus.UInt16(str(self._service.port)),
                avahi.string_array_to_txt_array(self._service.description))
            self._group.Commit()
        except dbus.exceptions.DBusException as e:
            # is there  a better way to detect this?
            if e.get_dbus_name() == avahi.DBUS_NAME + ".CollisionError":
                rospy.logwarn("Zeroconf: local name collision error [%s][%s]"%(self._service.name,rospy.get_name()))
                self._rename_count -= 1
                if self._rename_count > 0:
                    service = self._service
                    self.remove_service() # resets self._service
                    service.name = self.get_alternative_name(service.name)
                    self.add_service(service.name, service. service_type, service.domain, service.hostname, service.port)
                    self._rename_count = self._rename_count_limit 
                    return
                else:
                    self.remove_service() 
                    rospy.logerr("Zeroconf: reached maximum number of name retries [%d][%s]"%(self._rename_count_limit,rospy.get_name()))
                    self._rename_count = self._rename_count_limit 
                    return
            else:
                rospy.logerr("Zeroconf: dbus exception caught when adding service [%s][%s]"%(self._service.name,rospy.get_name()))
                return
        
        # Hack for now since I can't add services after I start the dbug engine (avahi's threading issue?).
        self._dbus_engine.start()
        
    def remove_service(self):
        if not self._group is None:
            rospy.loginfo("Zeroconf: removing service[%s]"%(rospy.get_name()))
            self._group.Reset()
            self._service = None
        
    def get_alternative_name(self,name):
        return self._dbus_engine._server.GetAlternativeServiceName(name)

    def group_state_changed_callback(self, state, error):
        if state == avahi.ENTRY_GROUP_UNCOMMITED:
            #print "Zeroconf: entry group state change -> uncommited [%s]"%rospy.get_name()
            rospy.logdebug("Zeroconf: entry group state change -> uncommited [%s]"%rospy.get_name())
        elif state == avahi.ENTRY_GROUP_REGISTERING:
            #print "Zeroconf: entry group state change -> registering [%s]"%rospy.get_name()
            rospy.logdebug("Zeroconf: entry group state change -> registering [%s]"%rospy.get_name())
        elif state == avahi.ENTRY_GROUP_ESTABLISHED:
            #print "Zeroconf: entry group state change -> service established [%s]"%rospy.get_name()
            rospy.Service('~advertised_service', zeroconf_comms.srv.AdvertisedService, self.provide_service_info)
            rospy.logdebug("Zeroconf: advertising service [%s]\n%s" % ( self._service.name, utilities.service_to_str(self._service)))
            rospy.logdebug("Zeroconf: entry group state change -> service established [%s]"%rospy.get_name())
        elif state == avahi.ENTRY_GROUP_COLLISION:
            #print "Zeroconf: entry group state change -> error due to service name collision [%s]"%rospy.get_name()
            rospy.logwarn("Zeroconf: group name collision error [%s][%s]"%(self._service.name,rospy.get_name()))
            if self._rename_count > 0 :
                self._rename_count -= 1
                self._service.name = self.get_alternative_name(self._service.name)
                self.add_service(self._service.name, self._service.service_type, self._service.domain, self._service.hostname, self._service.port)
            else:
                self.remove_service()
                rospy.logerr("Zeroconf: reached maximum number of name retries [%d][%s]"%(self._rename_count_limit,rospy.get_name()))
                self._rename_count = self._rename_count_limit 
                self._dbus_engine.stop()
                return
        elif state == avahi.ENTRY_GROUP_FAILURE:
            #print "Zeroconf: entry group state change -> failure [%s]"%rospy.get_name()
            rospy.logerr("Zeroconf:  entry group state change -> failure [%s][%s]"% (error, rospy.get_name()))
            self._dbus_engine.stop()

def single_publisher_main():
    '''
      Entry point for a zeroconf publisher which will advertise a single service established from ros parameters.
    '''
    gobject.threads_init() # magic - don't start threads without it: http://dbus.freedesktop.org/doc/dbus-python/api/dbus.mainloop.glib-module.html
    rospy.init_node('zeroconf_single_publisher', log_level=rospy.DEBUG)
    name = rospy.get_param('~name','Unknown')
    service_type = rospy.get_param('~service_type', '_ros-master._tcp')
    domain = rospy.get_param('~domain', 'local')
    hostname = rospy.get_param('~hostname', roslib.network.get_host_name())
    port = rospy.get_param('~port', roslib.network.parse_http_host_and_port(roslib.rosenv.get_master_uri())[1])
    
    dbus_engine = DBusEngine()
    zconf_publisher = Publisher(dbus_engine)
    zconf_publisher.add_service(name, service_type, domain, hostname, port)
    #dbus_engine.start() # currently starting inside add_service
    rospy.spin()
    dbus_engine.stop()

def delayed_publisher_main():
    '''
      Entry point for a zeroconf publisher with delayed advertising services (via subscriber).
    '''
    gobject.threads_init() # magic - don't start threads without it: http://dbus.freedesktop.org/doc/dbus-python/api/dbus.mainloop.glib-module.html
    rospy.init_node('zeroconf_publisher', log_level=rospy.DEBUG)
    dbus_engine = DBusEngine()
    unused_zconf_publisher = Publisher(dbus_engine)
    #dbus_engine.start() # currently starting inside add_service
    rospy.spin()
    dbus_engine.stop()

if __name__ == '__main__':
    single_publisher_main()
