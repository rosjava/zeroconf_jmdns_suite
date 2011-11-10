
package src.main.java.zeroconf.jmdns;

import java.io.IOException;
import java.lang.Thread;
import java.net.Inet4Address;
import java.util.HashMap;
import java.util.Set;

import javax.jmdns.JmmDNS;
import javax.jmdns.NetworkTopologyEvent;
import javax.jmdns.NetworkTopologyListener;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;
import javax.jmdns.ServiceListener;
import javax.jmdns.ServiceTypeListener;

public class Browser implements ServiceListener, ServiceTypeListener, NetworkTopologyListener {

    JmmDNS jmmdns;
    String service_type;
    String service_name;                
    int    service_port;
    ServiceInfo service_info; // service to be published

    Browser(JmmDNS mmDNS) {
    	/********************
    	 * Variables
    	 *******************/
        this.jmmdns = mmDNS;
        // publishing and listening details
        this.service_type = "_ros-master._tcp.local.";
        // publishing details
        this.service_name = "RosMaster";
        this.service_port = 8888;
        String service_key = "description"; // Max 9 chars
        String text = "Hypothetical ros master";
        HashMap<String, byte[]> properties = new HashMap<String, byte[]>();
        properties.put(service_key, text.getBytes());
        service_info = ServiceInfo.create(service_type, service_name, service_port, 0, 0, true, properties);
//        service_info = ServiceInfo.create(service_type, service_name, service_port, 0, 0, true, text);

    	/********************
    	 * Methods
    	 *******************/
        this.jmmdns.addNetworkTopologyListener(this);
    }
    
	/*************************************************************************
	 * User Interface
	 ************************************************************************/
    /**
     * If you try calling this immediately after a service added callback
     * occured, you probably wont see anything - it needs some time to resolve.
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
        	event.getDNS().addServiceListener(service_type, this);
        	System.out.printf("Publishing Service on %s:\n",event.getInetAddress().getHostAddress());
        	System.out.printf("  Name   : %s\n", service_info.getName() );
        	System.out.printf("  Type   : %s\n", service_info.getType() );
        	System.out.printf("  Port   : %s\n", service_info.getPort() );
//        	this.jmmdns.unregisterService(service_info);
//        	this.jmmdns.registerService(service_info);
         	event.getDNS().registerService(service_info.clone()); // if you don't clone it, it falls over badly!
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
        Browser browser = new Browser(JmmDNS.Factory.getInstance());
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