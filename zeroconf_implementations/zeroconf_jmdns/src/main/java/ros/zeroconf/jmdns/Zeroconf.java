package ros.zeroconf.jmdns;

import java.io.IOException;
import java.lang.Thread;
import java.net.Inet4Address;
import java.util.Iterator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import javax.jmdns.JmmDNS;
import javax.jmdns.NetworkTopologyEvent;
import javax.jmdns.NetworkTopologyListener;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;
import javax.jmdns.ServiceListener;
import javax.jmdns.ServiceTypeListener;

public class Zeroconf implements ServiceListener, ServiceTypeListener, NetworkTopologyListener {

    JmmDNS jmmdns;
    Set<String> listeners;
    Set<ServiceInfo> services;

    Zeroconf() {
    	/********************
    	 * Variables
    	 *******************/
        this.jmmdns = JmmDNS.Factory.getInstance();
        this.listeners = new HashSet<String>();
        this.services = new HashSet<ServiceInfo>();

    	/********************
    	 * Methods
    	 *******************/
        this.jmmdns.addNetworkTopologyListener(this);
    }
    
	/*************************************************************************
	 * User Interface
	 ************************************************************************/
    public void addListener(String service_type, String domain) {
    	String service = service_type + "." + domain + ".";
    	System.out.printf("Adding listener to the set: %s\n",service);
    	listeners.add(service);
    	// add to currently established interfaces
    	System.out.printf("Adding listener to jmmdns: %s\n",service);
    	jmmdns.addServiceListener(service, this);
    }
    
    /**
     * Function stub (will implement later).
     */
    public void removeListener() {}
    /**
     * Publish a zeroconf service.
     *  
     * @param name : english readable name for the service
     * @param type : zeroconf service type, e.g. _ros-master._tcp
     * @param domain : domain to advertise on (usually 'local')
     * @param port : port number
     * @param description : 
     */
    public void addService(String name, String type, String domain, int port, String description) {
    	String full_service_type = type + "." + domain + ".";
        String service_key = "description"; // Max 9 chars
        String text = "Hypothetical ros master";
        HashMap<String, byte[]> properties = new HashMap<String, byte[]>();
        properties.put(service_key, text.getBytes());
        services.add(ServiceInfo.create(full_service_type, name, port, 0, 0, true, properties));
//        services.add(ServiceInfo.create(service_type, service_name, service_port, 0, 0, true, text));
    }
    /**
     * If you try calling this immediately after a service added callback
     * occurred, you probably wont see anything - it needs some time to resolve.
     */
    public void listDiscoveredServices() {
        ServiceInfo[] service_infos = this.jmmdns.list(service_type);
        for ( int i = 0; i < service_infos.length; i++ ) {
        	display(service_infos[i]);
        }
    }
    public void display(ServiceInfo service_info) {
    	System.out.println("Service Info:");
    	System.out.printf("  Name   : %s\n", service_info.getName() );
    	System.out.printf("  Type   : %s\n", service_info.getType() );
    	for ( int i = 0; i < service_info.getInetAddresses().length; ++i ) {
        	System.out.printf("  Address: %s\n", service_info.getInetAddresses()[i].getHostAddress() );
    	}
    }
    
	/*************************************************************************
	 * Network Topology Callbacks 
	 ************************************************************************/
	public void inetAddressAdded(NetworkTopologyEvent event) {
		System.out.printf("[+] NetworkInterface: %s\n", event.getInetAddress().getHostAddress());
        try {
        	event.getDNS().addServiceTypeListener(this);
        	for(String service : listeners ) {
        		System.out.printf("      Adding service listener '%s'\n",service);
            	event.getDNS().addServiceListener(service, this);
        	}
        	System.out.printf("Publishing Service on %s:\n",event.getInetAddress().getHostAddress());
        	System.out.printf("  Name   : %s\n", service_info.getName() );
        	System.out.printf("  Type   : %s\n", service_info.getType() );
        	System.out.printf("  Port   : %s\n", service_info.getPort() );
         	//event.getDNS().registerService(service_info.clone()); // if you don't clone it, it falls over badly!
        } catch (IOException e) {
	        e.printStackTrace();
        }
	}
	
	public void inetAddressRemoved(NetworkTopologyEvent event) {
		System.out.printf("[-] NetworkInterface: %s\n", event.getInetAddress().getHostAddress());
		event.getDNS().removeServiceTypeListener(this);
		event.getDNS().removeServiceListener(service_type, this);
    	System.out.println("Unpublishing Service:");
    	System.out.printf("  Name   : %s\n", service_info.getName() );
    	System.out.printf("  Type   : %s\n", service_info.getType() );
    	System.out.printf("  Port   : %s\n", service_info.getPort() );
    	//this.jmmdns.unregisterService(service_info);
    	//this.jmmdns.registerService(service_info);
    	event.getDNS().unregisterService(service_info); // this may not work because we're cloning it.
	}
    
	/*************************************************************************
	 * Listener Callbacks 
	 ************************************************************************/
    @Override
    public void serviceAdded(ServiceEvent event) {
        final String name = event.getName();
        System.out.println("[+] Service         : " + name);
    }

    @Override
    public void serviceRemoved(ServiceEvent event) {
        final String name = event.getName();
        System.out.println("[-] Service         : " + name);
    }

    @Override
    public void serviceResolved(ServiceEvent event) {
        final String name = event.getName();
        System.out.println("[+] Resolved Service: " + name);
    }

    @Override
    public void serviceTypeAdded(ServiceEvent event) {
//        final String aType = event.getType();
//        System.out.println("TYPE: " + aType);
    }

    @Override
    public void subTypeForServiceTypeAdded(ServiceEvent event) {
//        System.out.println("SUBTYPE: " + event.getType());
    }

	/*************************************************************************
	 * Main 
	 ************************************************************************/
    public static void main(String argv[]) throws IOException {
        Zeroconf browser = new Zeroconf();
        browser.addListener("_ros-master._tcp","local");
		try {
    		Thread.sleep(1000L);
	    } catch (InterruptedException e) { e.printStackTrace(); }
        while(true) {
    		try {
    			browser.listDiscoveredServices();
        		Thread.sleep(1000L);
		    } catch (InterruptedException e) {
		        e.printStackTrace();
		    }
        }
    }
}