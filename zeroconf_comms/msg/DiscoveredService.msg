#
# Provides all the properties required for definition of a discovered 
# zeroconf service. Note that it contains rather alot more information
# than that which is used to publish a zeroconf service.

# Service Part
string name
string type
string domain
string description

# Resolved part
string hostname
string[] ipv4_addresses
string[] ipv6_addresses
int32 port

# These are detailed variables that you should not normally need
# and can be safely ignored in most circumstances (currently used
# by the avahi implementation).

uint32 cookie
bool is_local
bool our_own
bool wide_area
bool multicast
bool cached

