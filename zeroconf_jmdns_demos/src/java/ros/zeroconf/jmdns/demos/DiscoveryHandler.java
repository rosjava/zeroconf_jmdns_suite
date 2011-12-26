package ros.zeroconf.jmdns.demos;

import ros.zeroconf.jmdns.ZeroconfListener;
import javax.jmdns.ServiceInfo;

/**
 * This class is a handler that can be passed to
 * a ros.zeroconf.jmdns.Zeroconf instance to to allow zeroconf discovery callbacks 
 * (service added, resolved, removed) to be processed in a custom way for the user
 * of the class. 
 */
public class DiscoveryHandler implements ZeroconfListener {
	
	public void serviceAdded(ServiceInfo service_info) {
    	String result = "[+] " + service_info.getQualifiedName();
    	System.out.println(result);
	}
	public void serviceRemoved(ServiceInfo service_info) {
		String result = "[-] " + service_info.getQualifiedName();
    	System.out.println(result);
	}
	public void serviceResolved(ServiceInfo service_info) {
    	String result = "[=] " + service_info.getQualifiedName() + "\n";
    	result += "    Port     : " + service_info.getPort() + "\n";
    	result += "    Hostname : " + service_info.getServer() + "\n";
    	for ( int i = 0; i < service_info.getInetAddresses().length; ++i ) {
    		result += "    Address: " + service_info.getInetAddresses()[i].getHostAddress() + "\n";
    	}
    	System.out.printf(result);
	}
}   
