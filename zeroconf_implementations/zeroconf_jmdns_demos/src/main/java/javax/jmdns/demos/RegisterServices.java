package javax.jmdns.demos;

import java.io.IOException;
import javax.jmdns.JmDNS;
import javax.jmdns.JmmDNS;
import javax.jmdns.ServiceInfo;

import java.util.Collections;
import java.util.Enumeration;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;
import java.util.HashSet;
import java.lang.Thread;
import java.net.InetAddress;
import java.net.Inet4Address;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;

public class RegisterServices {

	public RegisterServices() {
        service_key = "description"; // Max 9 chars
        text = "Hypothetical ros master";
        properties = new HashMap<String, byte[]>();
        properties.put(service_key, text.getBytes());
        service = ServiceInfo.create(
        		"_ros-master._tcp.local.", 
        		"Ros Master", 
        		8000, 0, 0, true, properties);
        jmdns = new HashSet<JmDNS>();
	}
	
	// Prints a pure java list of interfaces and addresses on those interfaces.
	// Then starts a jmdns instance on each one of those interface-address combinations. 
	public void init(InetAddress address) {
        try {
	        Enumeration<NetworkInterface> nets = NetworkInterface.getNetworkInterfaces();
            System.out.println("");
            System.out.println("Jmdns Instances");
            // if you don't hunt down interfaces and addresses, it will just grab a single
            // interface using some of its own internal logic.
	        for (NetworkInterface netint : Collections.list(nets)) {
	        	 System.out.printf("  Interface: %s\n", netint.getName());
	             Enumeration<InetAddress> inetAddresses = netint.getInetAddresses();
	             for (InetAddress inetAddress : Collections.list(inetAddresses)) {
	            	 System.out.printf("    InetAddress: %s\n", inetAddress.toString());
	            	 String name = netint.getName();
	            	 if ( inetAddress instanceof Inet4Address ) { 
	            		 name = netint.getName() + "-ipv4";
	            	 } else {
	            		 name = netint.getName() + "-ipv6";
	            	 }
	            	 if ( !netint.getName().equals("lo") ) { 
	            		 jmdns.add(JmDNS.create(inetAddress,name));
	            	 }
	             }
	             System.out.println("");
	        }
        } catch (SocketException e) {
	        e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        System.out.printf("Number of Jmdns Instances: %s\n", jmdns.size());
	}
	// for each instance, we register a service 
	// basically just broadcasting the service to everybody here.
	public void register() {
        System.out.printf("JmDNS Instances:\n");
        try {
        	for ( JmDNS mdns : jmdns ) {
        		System.out.printf("  jmdns: %s\n", mdns.getName());
        	}
            System.out.printf("Registering the service (this is blocking)(can only advertise on one jmdns)\n");
        	for ( JmDNS mdns : jmdns ) {
        		System.out.printf("  jmdns: %s\n", mdns.getName());
        		mdns.registerService(service);
        		// can only register the one 'service'..if you try and register it on multiple JmDNS
        		// instances, it will give you an exception.
        		break; 
        	}
	    } catch (IOException e) {
	        e.printStackTrace();
	    }
	}
	
	public void discovery() {
        System.out.printf("Listing services of type %s (also blocking)\n",service.getType());
    	for ( JmDNS mdns : jmdns ) {
            ServiceInfo[] services = mdns.list(service.getType());
            for ( int j = 0; j < services.length; j = j + 1) {
                System.out.printf("Service discovered: \n");
                System.out.printf("   Name   : %s\n", services[j].getName());
                try {
                	System.out.printf("   Address: %s\n", mdns.getInetAddress().getHostAddress());
	    	    } catch (IOException e) {
	    	        e.printStackTrace();
	    	    }
//                for ( int i = 0; i < services[j].getInet4Addresses().length; i = i + 1) {
//                	System.out.printf("   Address: %s\n", services[j].getInet4Addresses()[i].getHostAddress());
//                } 
//                for ( int i = 0; i < services[j].getInet6Addresses().length; i = i + 1) {
//                	System.out.printf("   Address: %s\n", services[j].getInet6Addresses()[i].getHostAddress());
//                }
                System.out.printf("   Port   : %d\n", services[j].getPort());
            }
    	}
	}
	
	public void unregister() {
        System.out.printf("Unregistering the service\n");
    	for ( JmDNS mdns : jmdns ) {
    		mdns.unregisterAllServices();
    	}
	}
	
	public void shutdown() {
		try {
	        Thread.sleep(1500L);
	        System.out.println("Closing jmdns instance");
	    	for ( JmDNS mdns : jmdns ) {
	    		mdns.close();
	    	}
	        System.out.println("Done");
	    } catch (IOException e) {
	        e.printStackTrace();
	    } catch (InterruptedException e) {
	        e.printStackTrace();
	    }
	}
	
	private String service_key;
	private String text;
	private Map<String, byte[]> properties;
	private ServiceInfo service;
	private Set<JmDNS> jmdns;
	
    /**
     * @param args
     *            the command line arguments
     */
    public static void main(String[] args) {
    	System.out.println("================== register service demo ==================");
    	System.out.println("");
    	InetAddress addr = null;
    	try {
    		addr = Inet4Address.getByName("192.168.10.66");
        	System.out.printf("Address: %s\n",addr.toString());
    	} catch (UnknownHostException e) {
	        e.printStackTrace();
	    }
    	RegisterServices reg = new RegisterServices();
    	reg.init(addr);
    	reg.register();
		try {
	        Thread.sleep(1000L);
	    } catch (InterruptedException e) {
	        e.printStackTrace();
	    }
    	reg.discovery();
    	reg.unregister();
		try {
	        Thread.sleep(1000L);
	    } catch (InterruptedException e) {
	        e.printStackTrace();
	    }
    	reg.discovery();
    	reg.shutdown();
    }
}
