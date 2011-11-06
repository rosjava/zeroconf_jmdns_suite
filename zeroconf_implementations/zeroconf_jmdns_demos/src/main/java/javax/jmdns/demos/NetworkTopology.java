package javax.jmdns.demos;

//import javax.jmdns.JmDNS;
import javax.jmdns.JmmDNS;
import javax.jmdns.ServiceInfo;
import javax.jmdns.NetworkTopologyListener;
import javax.jmdns.NetworkTopologyEvent;
import java.util.Map;
import java.util.HashMap;
import java.io.IOException;
import java.lang.Thread;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Enumeration;
import java.net.NetworkInterface;
import java.util.Collections;
import java.net.SocketException;

public class NetworkTopology {

	static class NetworkListener implements NetworkTopologyListener {
		public void inetAddressAdded(NetworkTopologyEvent event) {
			// Should be able to get the underlying jmdns of this event via event.getDNS();
			// I expect this is only possible if using JmmDNS()
			System.out.println("  NetworkListener: inetaddress added");
			System.out.printf("    Inet Address: %s\n", event.getInetAddress().getHostAddress());
		}
		public void inetAddressRemoved(NetworkTopologyEvent event) {
			System.out.println("  NetworkListener: inetaddress removed");
			System.out.printf("    Inet Address: %s\n", event.getInetAddress().getHostAddress());
		}
	}

	public NetworkTopology() {
        service_key = "description"; // Max 9 chars
        text = "Hypothetical ros master";
        properties = new HashMap<String, byte[]>();
        properties.put(service_key, text.getBytes());
        service = ServiceInfo.create(
        		"_ros-master._tcp.local.", 
        		"Ros Master", 
        		8000, 0, 0, true, properties);
	}
	
	public void init() {
	    System.out.println("Jmmdns Instance:");
    	jmmdns = JmmDNS.Factory.getInstance();
    	//network_listener = new NetworkListener();
    	//jmmdns.addNetworkTopologyListener(new NetworkListener());
    }
	
	public void discovery() {
		System.out.println("  Discovery:");
    	for ( int i = 0; i < jmmdns.getHostNames().length; i = i + 1 ) {
		    System.out.printf("    HostName: %s\n", jmmdns.getHostNames()[i]);
    	}
    	try {
	    	for ( int i = 0; i < jmmdns.getInetAddresses().length; i = i + 1 ) {
			    System.out.printf("    Address : %s\n", jmmdns.getInetAddresses()[i].getHostAddress());
	    	}
	    } catch (IOException e) {
	        e.printStackTrace();
	    }
	}
	
	public void shutdown() {
		try {
			jmmdns.close();
	    } catch (IOException e) {
	        e.printStackTrace();
	    }
	}

	public void inetaddress_search() {
		/**************************
		 * Localhost
		 **************************/
        try {
        	InetAddress[] addresses = InetAddress.getAllByName("localhost");
            System.out.println("");
        	System.out.println("Java Inet Addresses (localhost):");
        	for ( int i = 0; i < addresses.length; i = i + 1 ) {
        		System.out.printf("  Address: %s\n", addresses[i].toString() );
        	}
        } catch (UnknownHostException e) {
	        e.printStackTrace();
        } catch (SecurityException e) {
	        e.printStackTrace();
        }
		/**************************
		 * Network Interfaces
		 **************************/
        try {
	        Enumeration<NetworkInterface> nets = NetworkInterface.getNetworkInterfaces();
            System.out.println("");
            System.out.println("Java Interfaces");
	        for (NetworkInterface netint : Collections.list(nets)) {
//	        	 System.out.printf("  Display name: %s\n", netint.getDisplayName());
	        	 System.out.printf("  Name: %s\n", netint.getName());
	             Enumeration<InetAddress> inetAddresses = netint.getInetAddresses();
	             for (InetAddress inetAddress : Collections.list(inetAddresses)) {
	            	 System.out.printf("  InetAddress: %s\n", inetAddress);
	             }
	             System.out.println("");
	        }
        } catch (SocketException e) {
	        e.printStackTrace();
        }
	}
	
    public static void main(String[] args) {
    	System.out.println("========== network topology demo ==========");
    	System.out.println("");
    	System.out.println("Note that the interfaces is odd, it returns");
    	System.out.println("ipv6 addresses for these and the ipv4 gets");
    	System.out.println("gets lost in the wash.");
    	System.out.println("");
    	
    	NetworkTopology network_topology = new NetworkTopology();
    	network_topology.init();
		try {
	        Thread.sleep(2000L);
	    } catch (InterruptedException e) {
	        e.printStackTrace();
	    }
	    network_topology.discovery();
	    network_topology.shutdown();
        network_topology.inetaddress_search();
        
    }
    
	private String service_key;
	private String text;
	private Map<String, byte[]> properties;
	private ServiceInfo service;
	private JmmDNS jmmdns;
	private NetworkListener network_listener;

}
