#!/usr/bin/env python

# This is an example Avahi (zeroconf) extension for pyzmq.

# FIXME: This hasn't seen much love lately and really needs some
# move work. zmqconf could subclass pyzmq so that bind announces
# the service and connect discovers the service - using names
# instead of connection strings (eg. "tcp://192.168.0.1:5555").

# Copyright (C) 2010 David Robinson <zxvdr.au@gmail.com>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

import dbus
import gobject
import avahi    # ey, provided by dbus-ui-tools package on Fedora?!
from dbus import DBusException
from dbus.mainloop.glib import DBusGMainLoop
from threading import Thread

MAX_RENAMES = 12

class zmqconf:

    def __init__(self, domain='', host=''):
        self.domain = domain
        self.host = host
        self.rename_count = MAX_RENAMES
        self._services = []
        self.group = None

        DBusGMainLoop(set_as_default=True)
        gobject.threads_init()
        dbus.mainloop.glib.threads_init()
        self.main_loop = gobject.MainLoop()
        self.bus = dbus.SystemBus()

        self.server = dbus.Interface(self.bus.get_object(avahi.DBUS_NAME, avahi.DBUS_PATH_SERVER), avahi.DBUS_INTERFACE_SERVER)
        self.server.connect_to_signal("StateChanged", self._server_state_changed)
        self._server_state_changed(self.server.GetState())

        t = Thread(target=self.main_loop.run)
        t.setDaemon(True)
        t.start()

    def browse_services(self, type, domain=''):
        sbrowser = dbus.Interface(
                self.bus.get_object(
                    avahi.DBUS_NAME,
                    self.server.ServiceBrowserNew(
                        avahi.IF_UNSPEC,
                        avahi.PROTO_UNSPEC,
                        type, domain,
                        dbus.UInt32(0))
                    ),
                avahi.DBUS_INTERFACE_SERVICE_BROWSER)
        sbrowser.connect_to_signal("ItemNew", self._myhandler)

    def close(self):
        """Close the socket.

        This can be called to close the socket by hand. If this is not
        called, the socket will automatically be closed when it is
        garbage collected.
        """
        # We can overwrite closing here
        pass

    def bind(self, addr):
        """Bind the socket to an address.

        This causes the socket to listen on a network port. Sockets on the
        other side of this connection will use :meth:`Socket.connect` to
        connect to this socket.

        Parameters
        ----------
        addr : str
            The address string. This has the form 'protocol://interface:port',
            for example 'tcp://127.0.0.1:555'. Protocols supported are
            tcp, upd, pgm, iproc and ipc.
        """

        # Zeroconf doesn't makes sense for some transports
        if addr[:3].lower() == 'ipc':
            raise Exception("Cannot use Zeroconf with IPC")
        elif addr[:5].lower() == 'inproc':
            raise Exception("Cannot use Zeroconf with inproc")

        # bind to the socket
        #port = addr[addr.rindex(':') + 1:]
        #proto = addr[:addr.index(':')]
        # if the bind was successful (it'll raise an exception if not) publish details
        proto, host, port = addr.split(':')
        self.add_service("Example Service", "_example._tcp", 5555, "somethingcrazy")
        #self.add_service("Example", proto, int(port), addr)


    def _service_resolved(self, *args):
        print 'service resolved'
        print 'name:', args[2]
        print 'address:', args[7]
        print 'port:', args[8]

    def _print_error(self, *args):
        print args[0]

    def _myhandler(self, interface, protocol, name, stype, domain, flags):
        print "Found service '%s' type '%s' domain '%s' " % (name, stype, domain)
        if flags & avahi.LOOKUP_RESULT_LOCAL:
            # local service, skip
            pass

        self.server.ResolveService(interface, protocol, name, stype,
            domain, avahi.PROTO_UNSPEC, dbus.UInt32(0),
            reply_handler=self._service_resolved, error_handler=self._print_error)

    def _add_services(self):
        for service in self._services:
            name, type, port, txt = service
            self.add_service(name, type, port, txt)

    def add_service(self, name, proto, port, txt):
        print "Adding service '%s' of type '%s' ..." % (name, proto)
        if self.group is None:
            #self.bus = dbus.SystemBus()
            #ob = self.bus.get_object(avahi.DBUS_NAME, avahi.DBUS_PATH_SERVER)
            #self.server = dbus.Interface(ob, avahi.DBUS_INTERFACE_SERVER)
            #eg = self.server.EntryGroupNew()
            #ob = self.bus.get_object(avahi.DBUS_NAME, eg)
            #self.group = dbus.Interface(ob, avahi.DBUS_INTERFACE_ENTRY_GROUP)
            self.group = dbus.Interface(self.bus.get_object(avahi.DBUS_NAME, self.server.EntryGroupNew()), avahi.DBUS_INTERFACE_ENTRY_GROUP)
            self.group.connect_to_signal('StateChanged', self._entry_group_state_changed)
        self.group.AddService(
                avahi.IF_UNSPEC,                # interface
                avahi.PROTO_UNSPEC,             # protocol
                dbus.UInt32(0),                 # flags
                name, proto,
                self.domain, self.host,
                dbus.UInt16(port),
                avahi.string_array_to_txt_array(txt))
        self.group.Commit()
        self._services.append((name, proto, port, txt))

    def remove_services(self):
        #global group
        if not self.group is None:
            self.group.Reset()
        self._services = []

    def _server_state_changed(self, state):
        if state == avahi.SERVER_COLLISION:
            print "WARNING: Server name collision"
            self.remove_services()
        elif state == avahi.SERVER_RUNNING:
            self._add_services()

    def _entry_group_state_changed(self, state, error):
        if state == avahi.ENTRY_GROUP_ESTABLISHED:
            print "Service established."
        elif state == avahi.ENTRY_GROUP_COLLISION:

            self.rename_count -= 1
            if self.rename_count > 0:
                self.name = server.GetAlternativeServiceName(self.name)
                print "WARNING: Service name collision, changing name to '%s' ..." % self.name
                self.remove_services()
                self._add_services()
            else:
                print "ERROR: No suitable service name found after retrying, exiting."
                self.main_loop.quit()
        elif state == avahi.ENTRY_GROUP_FAILURE:
            print "Error in group state changed", error
            self.main_loop.quit()
            return

if __name__ == "__main__":
    # Publish a service
    #context = zmq.Context(1, 1)
    s = zmqconf()
    address = "tcp://lo:5555"
    #s.bind(address)
    s.add_service("Example Service", "_example._tcp", 5555, "somethingcrazy")
    s.add_service("Example Service 2", "_example._tcp", 5355, "somethingcrazy")
    raw_input("Press any key to exit")

    # Discover services
    #context = zmq.Context(1, 1)
    #s = zmqconf()
    #s.browse_services("_example._tcp")
    #raw_input("Press any key to exit")