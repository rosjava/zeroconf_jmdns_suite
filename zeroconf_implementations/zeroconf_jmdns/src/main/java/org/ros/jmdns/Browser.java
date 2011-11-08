
package org.ros.jmdns;

import java.io.IOException;
import java.lang.Thread;

import javax.jmdns.JmmDNS;
import javax.jmdns.NetworkTopologyEvent;
import javax.jmdns.NetworkTopologyListener;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;
import javax.jmdns.ServiceListener;
import javax.jmdns.ServiceTypeListener;

public class Browser implements ServiceListener, ServiceTypeListener, NetworkTopologyListener {

    JmmDNS                    jmmdns;
    String                    type;

    Browser(JmmDNS mmDNS) {
        this.jmmdns = mmDNS;
        this.type = "_ros-master._tcp.local.";
        this.jmmdns.addNetworkTopologyListener(this);
    }
    
	/*************************************************************************
	 * User Methods 
	 ************************************************************************/
    public void listDiscoveredServices() {
//        ServiceInfo[] serviceInfos = this.jmmdns.getServiceInfos(type, name);
    }
    
	/*************************************************************************
	 * Network Topology Callbacks 
	 ************************************************************************/
	public void inetAddressAdded(NetworkTopologyEvent event) {
		System.out.printf("[+] NetworkInterface: %s\n", event.getInetAddress().getHostAddress());
        try {
        	event.getDNS().addServiceTypeListener(this);
        	event.getDNS().addServiceListener(type, this);
        } catch (IOException e) {
	        e.printStackTrace();
        }
	}
	
	public void inetAddressRemoved(NetworkTopologyEvent event) {
		System.out.printf("[-] NetworkInterface: %s\n", event.getInetAddress().getHostAddress());
		event.getDNS().removeServiceTypeListener(this);
		event.getDNS().removeServiceListener(type, this);
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
        new Browser(JmmDNS.Factory.getInstance());
        while(true) {
    		try {
    			System.out.println("Sleeping");
        		Thread.sleep(1000L);
		    } catch (InterruptedException e) {
		        e.printStackTrace();
		    }
        }
    }
}