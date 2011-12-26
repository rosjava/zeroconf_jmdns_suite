#
# Provides all the properties required for definition of a discovered 
# zeroconf service. Note that it contains rather alot more information
# than that which is used to publish a zeroconf service.

string name
string type
string domain
string hostname
string[] addresses
int32 port
int32 hardware_interface
int8 protocol
string description

# These are detailed variables that you should not normally need
# and can be safely ignored in most circumstances.

uint32 cookie
bool is_local
bool our_own
bool wide_area
bool multicast
bool cached
