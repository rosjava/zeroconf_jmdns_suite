#
# Provides all the properties required for definition of a zeroconf service.

string name
string type
string domain
string hostname
string address
int32 port
int32 hardware_interface
int8 protocol
string description

# These are detailed variables that you should not normally need

uint32 cookie
bool is_local
bool our_own
bool wide_area
bool multicast
bool cached
