package org.ros.jmdns;

import java.io.IOException;
import java.lang.Thread;

import javax.jmdns.JmmDNS;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;
import javax.jmdns.NetworkTopologyListener;
import javax.jmdns.NetworkTopologyEvent;
import javax.jmdns.ServiceListener;
import javax.jmdns.ServiceEvent;
import java.lang.Thread;

/**
 * @brief Zeroconf'ing with java.
 * 
 * This sets up the network topology interface discovery mechanisms newly
 * embraced by jmdns.
 */
public class Zeroconf {
	/**********************************************************************
	 * Callbacks
	 *********************************************************************/
	/**
	 * @brief Handles up-down events from hardware/virtual interfaces.
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
    static class SampleListener implements ServiceListener {
        @Override
        public void serviceAdded(ServiceEvent event) {
            System.out.println("Service added   : " + event.getName() + "." + event.getType());
        }

        @Override
        public void serviceRemoved(ServiceEvent event) {
            System.out.println("Service removed : " + event.getName() + "." + event.getType());
        }

        @Override
        public void serviceResolved(ServiceEvent event) {
            System.out.println("Service resolved: " + event.getInfo());
        }
    }

	static class Listener implements ServiceListener {
//		public Listener() {
//			System.out.println("  Listener: created");
//		}
		public void serviceAdded(ServiceEvent event) {
			System.out.println("  Listener: service added");
			System.out.printf("    Name: %s\n", event.getName());
			ServiceInfo info = event.getInfo();
			System.out.printf("      Type    : %s\n", info.getType());
			System.out.printf("      Name    : %s\n", info.getName());
//			System.out.printf("      Key     : %s\n", info.getKey());
//			System.out.printf("      QName   : %s\n", info.getQualifiedName());
//			System.out.printf("      Server  : %s\n", info.getServer());
//	    	for ( int i = 0; i < info.getHostAddresses().length; i = i + 1 ) {
//			    System.out.printf("    Address: %s\n", info.getHostAddresses()[i]);
//	    	}
//			System.out.printf("      Port    : %s\n", info.getPort());
//			System.out.printf("      Priority: %s\n", info.getPriority());
		}
		public void serviceRemoved(ServiceEvent event) {
			System.out.println("  Listener: service removed");
			System.out.printf("    Name: %s\n", event.getName());
			ServiceInfo info = event.getInfo();
			System.out.printf("      Type    : %s\n", info.getType());
			System.out.printf("      Name    : %s\n", info.getName());
	    }
		public void serviceResolved(ServiceEvent event) {
	    	
			System.out.println("  Listener: service resolved");
			System.out.printf("    Name: %s\n", event.getName());
			ServiceInfo info = event.getInfo();
			System.out.printf("      Type    : %s\n", info.getType());
			System.out.printf("      Name    : %s\n", info.getName());
			System.out.printf("      Key     : %s\n", info.getKey());
			System.out.printf("      QName   : %s\n", info.getQualifiedName());
			System.out.printf("      Server  : %s\n", info.getServer());
	    	for ( int i = 0; i < info.getHostAddresses().length; i = i + 1 ) {
			    System.out.printf("    Address: %s\n", info.getHostAddresses()[i]);
	    	}
			System.out.printf("      Port    : %s\n", info.getPort());
			System.out.printf("      Priority: %s\n", info.getPriority());
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
		jmmdns = JmmDNS.Factory.getInstance();
		jmmdns.registerServiceType("_ros-master._tcp.local");
		//network_listener = new NetworkListener();
		//jmmdns.addNetworkTopologyListener(new NetworkListener());
	}
	
	public void shutdown() {
		try {
			jmmdns.close();
	    } catch (IOException e) {
	        e.printStackTrace();
	    }
	}
	
	/**********************************************************************
	 * Services
	 *********************************************************************/
	public void add_listener() {
	    String service_type = "_ros-master._tcp.local";
	    System.out.printf("Listener for: %s\n", service_type);
		jmmdns.addServiceListener(service_type, new SampleListener());
	}

	private JmmDNS jmmdns;
	private NetworkListener network_listener;
	
	/**********************************************************************
	 * Debugging
	 *********************************************************************/
	/**
	 * @brief Print the list of currently associated addresses.
	 */
	public void print_debug_info() {
    	try {
    		System.out.println("Debugging:");
    		System.out.println("  Names:");
	    	for ( int i = 0; i < jmmdns.getNames().length; i = i + 1 ) {
			    System.out.printf("    Name   : %s\n", jmmdns.getNames()[i]);
	    	}
	    	// By default, names = hostnames it seems
    		// System.out.println("  Hosts:");
	    	// for ( int i = 0; i < jmmdns.getNames().length; i = i + 1 ) {
	    	//     System.out.printf("    Host   : %s\n", jmmdns.getHostNames()[i]);
	    	// }
			System.out.println("  Addresses:");
	    	for ( int i = 0; i < jmmdns.getInetAddresses().length; i = i + 1 ) {
			    System.out.printf("    Address: %s\n", jmmdns.getInetAddresses()[i].getHostAddress());
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
		zeroconf.print_debug_info();
		zeroconf.add_listener();
		try {
	        Thread.sleep(1000L);
	    } catch (InterruptedException e) {
	        e.printStackTrace();
	    }
        System.out.println("Press q to quit");
        try {
            int b;
	        while ((b = System.in.read()) != -1 && (char) b != 'q') {
	    		try {
	        		Thread.sleep(1000L);
			    } catch (InterruptedException e) {
			        e.printStackTrace();
			    }
	        }
	    } catch (IOException e) {
	        e.printStackTrace();
	    }
		zeroconf.shutdown();
	}

}
