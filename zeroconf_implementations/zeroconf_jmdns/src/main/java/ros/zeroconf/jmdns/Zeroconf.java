package ros.zeroconf.jmdns;

import java.io.IOException;
import java.lang.Thread;
import java.net.Inet4Address;
import java.util.Arrays;
import java.util.Iterator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.ArrayList;
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

    public Zeroconf() {
    	/********************
    	 * Variables
    	 *******************/
        this.jmmdns = JmmDNS.Factory.getInstance();
        this.listeners = new HashSet<String>();
        this.services = new HashSet<ServiceInfo>();

    	/********************
    	 * Methods
    	 *******************/
        // be nice to get rid of this completely - and have it in the jmdns library itself.
        this.jmmdns.addNetworkTopologyListener(this);
    }
    
	/*************************************************************************
	 * User Interface
	 ************************************************************************/
    public void addListener(String service_type, String domain) {
    	String service = service_type + "." + domain + ".";
    	System.out.printf("Activating listener: %s\n",service);
    	listeners.add(service);
    	// add to currently established interfaces
    	jmmdns.addServiceListener(service, this);
    }
    
    /**
     * Removes a single listener - though we're not likely to use this much.
     */
    public void removeListener(String service_type, String domain) {
    	String listener_to_remove = service_type + "." + domain + ".";
    	for ( Iterator<String> listener = listeners.iterator(); listener.hasNext(); ) {
    		String this_listener = listener.next().toString();
    		if ( this_listener.equals(listener_to_remove) ) { 
    	    	System.out.printf("Deactivating listener: %s\n",this_listener);
    	    	listener.remove();
    	    	// remove from currently established interfaces
				jmmdns.removeServiceListener(listener_to_remove, this);
				break;
    		}
    	}
    }
    /**
     * Publish a zeroconf service.
     * 
     * Should actually provide a return value here, so the user can see the
     * actually published name.
     * 
     * @param name : english readable name for the service
     * @param type : zeroconf service type, e.g. _ros-master._tcp
     * @param domain : domain to advertise on (usually 'local')
     * @param port : port number
     * @param description : 
     */
    public void addService(String name, String type, String domain, int port, String description) {
    	String full_service_type = type + "." + domain + ".";
    	System.out.printf("Registering service: %s\n", full_service_type);
        String service_key = "description"; // Max 9 chars
        String text = "Hypothetical ros master";
        HashMap<String, byte[]> properties = new HashMap<String, byte[]>();
        properties.put(service_key, text.getBytes());
        ServiceInfo service_info = ServiceInfo.create(full_service_type, name, port, 0, 0, true, properties);
        // we need much better logic here to handle duplications.
        if ( services.add(service_info) ) {
        	try {
        		jmmdns.registerService(service_info);
            } catch (IOException e) {
    	        e.printStackTrace();
            }
        }
        
        // this is broken - it adds it, but fails to resolve it on other systems
        // https://sourceforge.net/tracker/?func=detail&aid=3435220&group_id=93852&atid=605791
        // services.add(ServiceInfo.create(service_type, service_name, service_port, 0, 0, true, text));
    }
    
    /**
     * If you try calling this immediately after a service added callback
     * occurred, you probably wont see anything - it needs some time to resolve.
     * 
     * It will block if it needs to resolve services (and aren't in its cache yet).
     * 
     * @return service_infos : an array of discovered ServiceInfo objects.
     */
    public List<ServiceInfo> listDiscoveredServices() {
    	List<ServiceInfo> service_infos = new ArrayList<ServiceInfo>();;
    	for(String service : listeners ) {
	        service_infos.addAll(Arrays.asList(this.jmmdns.list(service)));
    	}
        return service_infos;
    }
    
    /**
     * This should be called when your application shuts down to remove all services
     * so you don't pollute the zeroconf namespace with hanging, unresolvable services. 
     */
    public void removeAllServices() {
    	System.out.printf("Removing all services\n");
    	jmmdns.unregisterAllServices();
    	services.clear();
    }
    
    public void display(ServiceInfo service_info) {
    	System.out.println("Service Info:");
    	System.out.printf("  Name   : %s\n", service_info.getName() );
    	System.out.printf("  Type   : %s\n", service_info.getType() );
    	System.out.printf("  Port   : %s\n", service_info.getPort() );
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
        	for(String listener : listeners ) {
        		System.out.printf("      Adding service listener '%s'\n",listener);
            	event.getDNS().addServiceListener(listener, this);
        	}
        	for (ServiceInfo service : services ) {
            	System.out.printf("Publishing Service on %s:\n",event.getInetAddress().getHostAddress());
            	System.out.printf("  Name   : %s\n", service.getName() );
            	System.out.printf("  Type   : %s\n", service.getType() );
            	System.out.printf("  Port   : %s\n", service.getPort() );
             	event.getDNS().registerService(service.clone()); // if you don't clone it, it falls over badly!
        	}
        } catch (IOException e) {
	        e.printStackTrace();
        }
	}
	
	public void inetAddressRemoved(NetworkTopologyEvent event) {
		System.out.printf("[-] NetworkInterface: %s\n", event.getInetAddress().getHostAddress());
		event.getDNS().removeServiceTypeListener(this);
    	for(String listener : listeners ) {
    		System.out.printf("      Removing service listener '%s'\n",listener);
    		event.getDNS().removeServiceListener(listener, this);
    	}
    	for (ServiceInfo service : services ) {
	    	System.out.println("Unpublishing Service:");
	    	System.out.printf("  Name   : %s\n", service.getName() );
	    	System.out.printf("  Type   : %s\n", service.getType() );
	    	System.out.printf("  Port   : %s\n", service.getPort() );
	    	event.getDNS().unregisterService(service); // this may not work because we're cloning it.
    	}
	}
    
	/*************************************************************************
	 * Listener Callbacks 
	 ************************************************************************/
    @Override
    public void serviceAdded(ServiceEvent event) {
        final ServiceInfo service_info = event.getInfo();
        System.out.println("[+] Service         : " + service_info.getQualifiedName());
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
    public static void main_listener(String argv[]) throws IOException {
        Zeroconf browser = new Zeroconf();
        browser.addListener("_ros-master._tcp","local");
        int i = 0;
        while( i < 8 ) {
    		try {
    			System.out.println("************ Discovered Services ************");
    			List<ServiceInfo> service_infos = browser.listDiscoveredServices();
    			for ( ServiceInfo service_info : service_infos ) {
	        		browser.display(service_info);
	        }
        		Thread.sleep(1000L);
		    } catch (InterruptedException e) {
		        e.printStackTrace();
		    }
    		++i;
        }
        browser.removeListener("_ros-master._tcp","local");
        System.out.println("Done");
        browser.jmmdns.close();
    }
    
    public static void main_publisher(String argv[]) throws IOException {
        Zeroconf publisher = new Zeroconf();
        publisher.addService("DudeMaster", "_ros-master._tcp", "local", 8888, "Dude's test master");
        int i = 0;
        while(true) {
    		try {
        		Thread.sleep(1000L);
		    } catch (InterruptedException e) {
		        e.printStackTrace();
		    }
    		++i;
    		if ( i == 8 ) {
    			publisher.removeAllServices();
    		}
        }
    }
}