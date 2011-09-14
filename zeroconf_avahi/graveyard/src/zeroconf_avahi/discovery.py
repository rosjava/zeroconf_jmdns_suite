'''
Roughly based on the code here; http://avahi.org/wiki/PythonPublishExample

@author: Daniel Stonier
'''

import avahi
import dbus
import gobject
import threading

import roslib; roslib.load_manifest('zeroconf_avahi')
import rospy

import utilities
from .dbus_engine import DBusEngine

from zeroconf_comms.msg import *
from zeroconf_comms.srv import *

'''
   Scans the zeroconf network to discover services of a 
   particular type. Note that the network is constrained
   by the domain (I need to understand this better).
   
   Network configuration is via ros params:
   
  - service_type : avahi service type [_ros-master._tcp]
  - domain : the domain for this zeroconf network [local]

'''
class Discovery():
    '''
      This class does zeroconf discovery on the network and provides both
      a service for retrieving the discovered services and 
      publishers for additions/subtractions to the list.
     
      Assumption: it has already initialised ros and has access to 
      ros logging, publishers and services.
    '''
    def __init__(self,dbus_engine, service_type="_ros-master._tcp", domain="local"):
        self._services = []
        self._dbus_engine = dbus_engine
        self._service_type = service_type
        self._domain = domain
        self._browser = None

        if self._browser is None:
            self._browser = dbus.Interface(self._dbus_engine._bus.get_object(
                avahi.DBUS_NAME,
                self._dbus_engine._server.ServiceBrowserNew(
                    avahi.IF_UNSPEC,
                    avahi.PROTO_UNSPEC,
                    self._service_type,
                    self._domain,
                    dbus.UInt32(0))),
                avahi.DBUS_INTERFACE_SERVICE_BROWSER)
            self._browser.connect_to_signal("ItemNew", self.resolve_service_add_callback)
            self._browser.connect_to_signal("ItemRemove", self.resolve_service_del_callback)
            
        rospy.Service('~discover_services', DiscoverServices, self.discover_services)
        self.new_connection_publisher = rospy.Publisher('~new_connections', zeroconf_comms.msg.Service)
        self.lost_connection_publisher = rospy.Publisher('~lost_connections', zeroconf_comms.msg.Service)

    def service_add_resolved_callback(self,*args):
        ''' 
          Adds the resolved master details to the list.
          Args are an array of the following:
          
          interface, protocol, name, service_type, domain, url, ???, ip, port
          
          Note that locally you'll see a service multiply if you have ip4/6 and if you're local and
          have more than one network device set.
   
          Decoding the args:
            protocol:
              ipv4 = avahi.PROTO_INET = 0
              ipv6 = avahi.PROTO_INET6 = 1
        '''
        #print "Args: %s %s %s %s %s %s %s"%(args[0],args[1],args[2],args[3],args[4],args[5],args[6])
        if args[1] == avahi.PROTO_INET6:
            rospy.logdebug("Zeroconf: ignoring discovered service on ipv6  [ros only using ipv4 still]")
            return
        new_master = zeroconf_comms.msg.Service()
        new_master.name = str(args[2])
        new_master.service_type = str(args[3])
        new_master.service_name = utilities.service_name(new_master.service_type)
        new_master.domain = str(args[4])
        combo = str(args[5]) # this is usually hostname.domain - is this a valid assumption?
        new_master.hostname = combo.replace("."+ new_master.domain,"",1)   
        new_master.ip = str(args[7])
        new_master.port = int(args[8])
        #new_master.description = ""
        
        existing_master = next((master for master in self._services if master.name == new_master.name), None)
        if existing_master is None:
            rospy.logdebug("Zeroconf: auto-discovered a new %s [%s]" % (utilities.service_name(new_master.service_type),rospy.get_name()))
            rospy.logdebug("Zeroconf: \n%s" % (utilities.service_to_str(new_master)))
            self._services.append(new_master)
            self.new_connection_publisher.publish(new_master)
        elif utilities.same_service(new_master, existing_master):
            rospy.logdebug("Zeroconf: discovered same service on different devices (e.g. eth0 and wlan1), ignoring it [%s]\n%s"% (rospy.get_name(),utilities.service_to_str(new_master)))
        else:
            rospy.logwarn("Zeroconf: discovered duplicate service (same name) [%s]"%rospy.get_name())
            rospy.logdebug("Zeroconf: duplicate service: [%s]\n%s"% (rospy.get_name(),utilities.service_to_str(new_master)))
            rospy.logwarn("Zeroconf: bad situation - should be served unique names [%s]"%rospy.get_name())

    def print_error_callback(self,*args):
        rospy.logerr("Zeroconf: error handling callback -> %s"%args[0])
  
    def resolve_service_add_callback(self,interface, protocol, name, stype, domain, flags):
        '''
          New master was added, resolve the type details and add to the list.
          Todo: why does rospy.logxxx and print both kill this callback?
        '''
        self._dbus_engine._server.ResolveService(
            interface, protocol, name, stype,
            domain, avahi.PROTO_UNSPEC,
            dbus.UInt32(0) & avahi.LOOKUP_RESULT_CACHED,
            reply_handler=self.service_add_resolved_callback,
            error_handler=self.print_error_callback)

    def resolve_service_del_callback(self,interface, protocol, name, stype, domain, flags,*args):
        # Could usefully convert these to interface, protocol strings etc.
        # Note, protocol is ipv4|ipv6|any
        lost_master = next((master for master in self._services if master.name == name), None)
        if lost_master is not None:
            rospy.logdebug("Zeroconf: dropped a connection to service [%s]\n%s" % (rospy.get_name(),utilities.service_to_str(lost_master)))
            self.lost_connection_publisher.publish(lost_master)
            self._services.remove(lost_master)
            #del(self._services[name])
        else:
            rospy.logwarn("Zeroconf: dropped a connection to an undiscovered service (possibly due to multiple interfaces)[%s][%s]"%(name, rospy.get_name()))

    def discover_services(self, req):
        '''
          Ros callback for discovering services.
        '''
        rospy.logdebug("Zeroconf: servicing request to discover services [%s]"%rospy.get_name())
        response = []
        for service in self._services:
            response.append(service)
        return DiscoverServicesResponse(response)

def main():
    gobject.threads_init() # Magic? Yes, don't start threads without it.
    rospy.init_node('zeroconf_discovery', log_level=rospy.DEBUG)
    service_type = rospy.get_param('~service_type', '_ros-master._tcp')
    domain = rospy.get_param('~domain', 'local')
    dbus_engine = DBusEngine()
    unused_zconf_discovery = Discovery(dbus_engine, service_type, domain)
    dbus_engine.start()
    rospy.spin()
    dbus_engine.stop()
    
if __name__ == '__main__':
    main()
