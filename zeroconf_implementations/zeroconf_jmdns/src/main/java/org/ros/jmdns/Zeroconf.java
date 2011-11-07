package org.ros.jmdns;

import java.io.IOException;
import java.lang.Thread;

import javax.jmdns.JmmDNS;
import javax.jmdns.ServiceInfo;
import javax.jmdns.NetworkTopologyListener;
import javax.jmdns.NetworkTopologyEvent;

/**
 * @brief Zeroconf'ing with java.
 * 
 * This sets up the network topology interface discovery mechanisms newly
 * embraced by jmdns.
 */
public class Zeroconf {
	/**
	 * @brief Callbacks handling up-down of interfaces.
	 */
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

	/**********************************************************************
	 * Utility
	 *********************************************************************/
	public Zeroconf() {}
	
	/**
	 * @brief Initialise the multi-mdns instance.
	 */
	public void init() {
	    System.out.println("Jmmdns Instance:");
		jmmdns = JmmDNS.Factory.getInstance();
		network_listener = new NetworkListener();
		jmmdns.addNetworkTopologyListener(new NetworkListener());
	}
	
	public void shutdown() {
		try {
			jmmdns.close();
	    } catch (IOException e) {
	        e.printStackTrace();
	    }
	}

	private JmmDNS jmmdns;
	private NetworkListener network_listener;
	
	/**********************************************************************
	 * Debugging
	 *********************************************************************/
	/**
	 * @brief Print the list of currently associated addresses.
	 */
	public void print_addresses() {
		System.out.println("  Addresses:");
    	try {
	    	for ( int i = 0; i < jmmdns.getInetAddresses().length; i = i + 1 ) {
			    System.out.printf("    Address : %s\n", jmmdns.getInetAddresses()[i].getHostAddress());
	    	}
	    } catch (IOException e) {
	        e.printStackTrace();
	    }
	}

	public static void main(String[] args) {
        //System.out.println(System.getProperty("java.class.path"));
//		if ( args.length != 1 ) {
//			Zeroconf.usage();
//			return;
//		}
//		String demo_type = args[0];
//		if ( demo_type.equals("--network_topology") ) {
//			NetworkTopology.main(args);
//		} else if ( demo_type.equals("--register_services") ) {
//			RegisterServices.main(args);
//		} else if ( demo_type.equals("--discover_services") ) {
//			DiscoverServices.main(args);
//		} else {
//			Main.usage();
//			System.out.printf("Done *%s*\n",demo_type);
//		}
		Zeroconf zeroconf = new Zeroconf();
		zeroconf.init();
		try {
	        Thread.sleep(2000L);
	    } catch (InterruptedException e) {
	        e.printStackTrace();
	    }
		zeroconf.print_addresses();
		try {
	        Thread.sleep(1000L);
	    } catch (InterruptedException e) {
	        e.printStackTrace();
	    }
		int i = 12;
		while( i > 11 ) {
			try {
		        Thread.sleep(1000L);
		    } catch (InterruptedException e) {
		        e.printStackTrace();
		    }
		}
		zeroconf.shutdown();
	}

}
