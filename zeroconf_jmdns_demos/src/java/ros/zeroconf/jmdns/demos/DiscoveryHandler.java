package ros.zeroconf.jmdns.demos;

import ros.zeroconf.jmdns.ZeroconfDiscoveryHandler;
import org.ros.message.zeroconf_comms.DiscoveredService;

/**
 * This class is a handler that can be passed to
 * a ros.zeroconf.jmdns.Zeroconf instance to to allow zeroconf discovery callbacks 
 * (service added, resolved, removed) to be processed in a custom way for the user
 * of the class. 
 */
public class DiscoveryHandler implements ZeroconfDiscoveryHandler {
	
	public void serviceAdded(DiscoveredService discovered_service) {
		String result = "[+] " + discovered_service.name + "." + discovered_service.type + "." + discovered_service.domain + ".";
    	System.out.println(result);
	}
	public void serviceRemoved(DiscoveredService discovered_service) {
		String result = "[-] " + discovered_service.name + "." + discovered_service.type + "." + discovered_service.domain + ".";
    	System.out.println(result);
	}
	public void serviceResolved(DiscoveredService discovered_service) {
		String result = "[=] " + discovered_service.name + "." + discovered_service.type + "." + discovered_service.domain + ".\n";
    	result += "    Port     : " + discovered_service.port + "\n";
    	result += "    Hostname : " + discovered_service.hostname + "\n";
    	for ( String address : discovered_service.ipv4_addresses ) {
    		result += "    Address: " + address + "\n";
    	}
    	for ( String address : discovered_service.ipv6_addresses ) {
    		result += "    Address: " + address + "\n";
    	}
    	System.out.printf(result);
	}
}   
