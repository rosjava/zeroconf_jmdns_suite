#!/usr/bin/env python

# Executes the ros master publisher and discovery engine.
# Note that either can be turned on and off via the

import roslib; roslib.load_manifest('zeroconf_avahi')
import rospy

from ros import zeroconf_avahi


if __name__ == '__main__':
    zeroconf_avahi.publisher.single_publisher_main()

