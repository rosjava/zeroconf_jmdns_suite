#!/usr/bin/env python

# Executes the ros master publisher and discovery engine.
# Note that either can be turned on and off via the

import roslib; roslib.load_manifest('zeroconf_avahi')
import rospy

from zeroconf_comms.srv import *

if __name__ == '__main__':
    rospy.init_node('zeroconf_advertise_service', log_level=rospy.DEBUG)
    rospy.loginfo("AdvertiseService : waiting for service.")
    rospy.wait_for_service('~advertise_service')
    rospy.loginfo("AdvertiseService : requesting advertising service.")
    advertise_service = rospy.ServiceProxy('~advertise_service', zeroconf_comms.srv.AdvertiseService)  
    try:
        request = zeroconf_comms.srv.AdvertiseServiceRequest()
        request.service.name = "Dude"
        request.service.domain = "local"
        request.service.service_type = "_ros-master._tcp"
        # Defaults will get set for hostname and port
        response = advertise_service(request)
    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)
        
    