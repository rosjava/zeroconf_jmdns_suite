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
import threading

import roslib; roslib.load_manifest('zeroconf_avahi')
import rospy

from dbus.mainloop.glib import DBusGMainLoop

##############################################################################
# Classes
##############################################################################

class DBusEngine(threading.Thread):
    def __init__(self):
        # FIXME Review thread locking as needed.
        # init thread
        threading.Thread.__init__(self)
        # Gobjects are an event based model of evil, do not trust them,
        DBusGMainLoop( set_as_default=True )
        self._main_loop = gobject.MainLoop()
        self._bus = dbus.SystemBus()
        # Initialise interface to DBUS Server
        self._server = dbus.Interface(
            self._bus.get_object( avahi.DBUS_NAME, avahi.DBUS_PATH_SERVER ),
            avahi.DBUS_INTERFACE_SERVER )

    def run(self):
        # A better alternative (that might stop segfaults when adding services while running)
        # might be to create our own custom loop here and call the appropriate dbus poll methods
        # with locks appropriately scattered around.
        self._main_loop.run()

    def stop(self):
        self._main_loop.quit()
        if self.isAlive():
            self.join()
        del(self)
