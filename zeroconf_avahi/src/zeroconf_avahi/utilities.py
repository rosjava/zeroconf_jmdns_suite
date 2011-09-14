'''
Created on 01/08/2011

Utility functions

@author: Daniel Stonier
'''

def service_to_str(service):
    ''' 
      String representation of zeroconf announcement 
    '''
    print service
    return "\tname: %s\n\ttype: %s [%s]\n\tdomain: %s\n\thostname: %s\n\taddress: %s\n\tport: %d\n\t interface: %d\n\tprotocol: %d\n\tdescription: %s\n\tis_local:%d\n\tour_own: %d\n\twide_area: %d\n\tmulticast: %d" % (
        service.name, 
        service_name(service.type),
        service.type,
        service.domain,
        service.hostname, 
        service.address, 
        service.port,
        service.interface,
        service.protocol,
        service.description,
        service.is_local,
        service.our_own,
        service.wide_area,
        service.multicast
        )

def same_service(service_one, service_two):
    '''
      Sometimes you'll see the same service on multiple interfaces - this will often happen on localhost
      where the service will be getting advertised on whatever network devices you currently have
      running (e.g. eth0 and wlan1).
      
      In these cases, we assume they're equal. In fact, the important properties uniquely
      identifying the service for us (remembering that we're discovering on a particular domain and
      service_type) are:
      
        service_name, port
         
    '''
    if ( service_one.name == service_two.name ) and ( service_one.port == service_two.port ):
        return True
    else:
        return False
        
def service_name(service_type):
    '''
        This emulates what python-avahi's ServiceTypeDatabase class does in 
        conjuction with /usr/share/service-types and /usr/lib/avahi/service-types.db.
        
        We could just roll out further .db's which would then get 
        it to print nice englishified versions of the service types for any
        zeroconf browser, however we're really only interested in making it functional
        internally (ros-internally) for now. So just hacking it in code for now.
    '''
    if service_type == '_ros-master._tcp':
        return "Ros Master"
    if service_type == '_concert-master._tcp':
        return "Concert Master"
    if service_type == '_app-manager._tcp':
        return "App Manager"

    if service_type == '_ros-master._udp':
        return "Ros Master (UDP)"
    if service_type == '_concert-master._udp':
        return "Concert Master (UDP)"
    if service_type == '_app-manager._udp':
        return "App Manager (UDP)"
    
    # or else...
    return service_type
