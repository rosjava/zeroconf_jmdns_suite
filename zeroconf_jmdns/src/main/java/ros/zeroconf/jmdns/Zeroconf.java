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
import java.util.Map;
import java.util.Set;

import javax.jmdns.JmmDNS;
import javax.jmdns.NetworkTopologyEvent;
import javax.jmdns.NetworkTopologyListener;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;
import javax.jmdns.ServiceListener;
import javax.jmdns.ServiceTypeListener;

public class Zeroconf implements ServiceListener, ServiceTypeListener, NetworkTopologyListener {

	private class DefaultListener implements ZeroconfListener {
		public void serviceAdded(ServiceInfo service) {}
		public void serviceRemoved(ServiceInfo service) {}
		public void serviceResolved(ServiceInfo service) {}
	}
    JmmDNS jmmdns;
    Set<String> listeners;
    Set<ServiceInfo> services;
    ZeroconfLogger logger;
    Map<String, ZeroconfListener> listener_callbacks;

    public Zeroconf() {
    	/********************
    	 * Variables
    	 *******************/
        this.jmmdns = JmmDNS.Factory.getInstance();
        this.listeners = new HashSet<String>();
        this.services = new HashSet<ServiceInfo>();
        this.logger = new Logger();
        this.listener_callbacks = new HashMap<String, ZeroconfListener>();

    	/********************
    	 * Methods
    	 *******************/
        // be nice to get rid of this completely - and have it in the jmdns library itself.
        this.jmmdns.addNetworkTopologyListener(this);
    }

    public Zeroconf(ZeroconfLogger logger) {
    	/********************
    	 * Variables
    	 *******************/
        this.jmmdns = JmmDNS.Factory.getInstance();
        this.listeners = new HashSet<String>();
        this.services = new HashSet<ServiceInfo>();
        this.logger = logger;
        this.listener_callbacks = new HashMap<String, ZeroconfListener>();

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
    	addListener(service_type, domain, new DefaultListener());
    }
    
    public void addListener(String service_type, String domain, ZeroconfListener listener_callback) {
    	String service = service_type + "." + domain + ".";
    	logger.println("Activating listener: " + service);
    	listeners.add(service);
    	// add to currently established interfaces
    	jmmdns.addServiceListener(service, this);
    	listener_callbacks.put(service, listener_callback);
    }
    
    /**
     * Removes a single listener - though we're not likely to use this much.
     */
    public void removeListener(String service_type, String domain) {
    	String listener_to_remove = service_type + "." + domain + ".";
    	for ( Iterator<String> listener = listeners.iterator(); listener.hasNext(); ) {
    		String this_listener = listener.next().toString();
    		if ( this_listener.equals(listener_to_remove) ) { 
    			logger.println("Deactivating listener: " + this_listener);
    	    	listener.remove();
    	    	// remove from currently established interfaces
				jmmdns.removeServiceListener(listener_to_remove, this);
				break;
    		}
    	}
    	listener_callbacks.remove(listener_to_remove);
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
    	logger.println("Registering service: " + full_service_type);
        String service_key = "description"; // Max 9 chars
        HashMap<String, byte[]> properties = new HashMap<String, byte[]>();
        properties.put(service_key,description.getBytes());
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
    	logger.println("Removing all services");
    	jmmdns.unregisterAllServices();
    	services.clear();
    }
    
    public void display(ServiceInfo service_info) {
    	logger.println("Service Info:");
    	logger.println("  Name   : " + service_info.getName() );
    	logger.println("  Type   : " + service_info.getType() );
    	logger.println("  Port   : " + service_info.getPort() );
    	for ( int i = 0; i < service_info.getInetAddresses().length; ++i ) {
        	logger.println("  Address: " + service_info.getInetAddresses()[i].getHostAddress() );
    	}
    }
    
    public String toString(ServiceInfo service_info) {
    	String result = "Service Info:\n";
    	result += "  Name   : " + service_info.getName() + "\n";
    	result += "  Type   : " + service_info.getType() + "\n";
    	result += "  Port   : " + service_info.getPort() + "\n";
    	for ( int i = 0; i < service_info.getInetAddresses().length; ++i ) {
    		result += "  Address: " + service_info.getInetAddresses()[i].getHostAddress() + "\n";
    	}
    	return result;
    }
    
    
    public void shutdown() throws IOException {
    	removeAllServices();
    	jmmdns.close();
    }
    
	/*************************************************************************
	 * Network Topology Callbacks 
	 ************************************************************************/
	public void inetAddressAdded(NetworkTopologyEvent event) {
		logger.println("[+] NetworkInterface: " + event.getInetAddress().getHostAddress());
        try {
        	event.getDNS().addServiceTypeListener(this);
        	for(String listener : listeners ) {
        		logger.println("      Adding service listener '" + listener + "'");
            	event.getDNS().addServiceListener(listener, this);
        	}
        	for (ServiceInfo service : services ) {
        		logger.println("Publishing Service on " + event.getInetAddress().getHostAddress());
        		logger.println("  Name   : " + service.getName() );
        		logger.println("  Type   : " + service.getType() );
        		logger.println("  Port   : " + service.getPort() );
             	event.getDNS().registerService(service.clone()); // if you don't clone it, it falls over badly!
        	}
        } catch (IOException e) {
	        e.printStackTrace();
        }
	}
	
	public void inetAddressRemoved(NetworkTopologyEvent event) {
		logger.println("[-] NetworkInterface: " + event.getInetAddress().getHostAddress());
		event.getDNS().removeServiceTypeListener(this);
    	for(String listener : listeners ) {
    		logger.println("      Removing service listener '" + listener + "'");
    		event.getDNS().removeServiceListener(listener, this);
    	}
    	for (ServiceInfo service : services ) {
    		logger.println("Unpublishing Service:");
    		logger.println("  Name   : " + service.getName() );
    		logger.println("  Type   : " + service.getType() );
    		logger.println("  Port   : " + service.getPort() );
	    	event.getDNS().unregisterService(service); // this may not work because we're cloning it.
    	}
	}
    
	/*************************************************************************
	 * Listener Callbacks 
	 ************************************************************************/
    @Override
    public void serviceAdded(ServiceEvent event) {
        final ServiceInfo service_info = event.getInfo();
        logger.println("[+] Service         : " + service_info.getQualifiedName());
        ZeroconfListener callback = listener_callbacks.get(service_info.getType());
        if ( callback != null ) {
        	callback.serviceAdded(service_info);
        }
    }

    @Override
    public void serviceRemoved(ServiceEvent event) {
        final String name = event.getName();
        final ServiceInfo service_info = event.getInfo();
        logger.println("[-] Service         : " + name);
        ZeroconfListener callback = listener_callbacks.get(service_info.getType());
        if ( callback != null ) {
        	callback.serviceRemoved(service_info);
        }
    }

    @Override
    public void serviceResolved(ServiceEvent event) {
        final String name = event.getName();
        final ServiceInfo service_info = event.getInfo();
        logger.println("[+] Resolved Service: " + name);
        ZeroconfListener callback = listener_callbacks.get(service_info.getType());
        if ( callback != null ) {
        	callback.serviceResolved(service_info);
        }
    }

    @Override
    public void serviceTypeAdded(ServiceEvent event) {
//        final String aType = event.getType();
//        logger.println("TYPE: " + aType);
    }

    @Override
    public void subTypeForServiceTypeAdded(ServiceEvent event) {
//        logger.println("SUBTYPE: " + event.getType());
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
        browser.shutdown();
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