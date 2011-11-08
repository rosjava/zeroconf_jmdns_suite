
package org.ros.jmdns;

import java.io.IOException;
import java.lang.Thread;

import javax.jmdns.JmmDNS;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceListener;
import javax.jmdns.ServiceTypeListener;

public class Browser implements ServiceListener, ServiceTypeListener {

	private static final long serialVersionUID = 5750114542524415107L;
    JmmDNS                    jmmdns;
    String                    type;

    Browser(JmmDNS mmDNS) {
        try {
	        this.jmmdns = mmDNS;
	        this.type = "_ros-master._tcp.local.";

	        // we need some time for the system to find the interfaces. 
	        try {
        		Thread.sleep(1000L);
		    } catch (InterruptedException e) {
		        e.printStackTrace();
		    }


	        // 1) initiate a thread pool?
	        this.jmmdns.addServiceTypeListener(this);
	        this.jmmdns.addServiceListener(type, this);
        } catch (IOException e) {
	        e.printStackTrace();
        }
	    System.out.println("Browser init'd");
    }
    
	/*************************************************************************
	 * Listener Callbacks 
	 ************************************************************************/
    @Override
    public void serviceAdded(ServiceEvent event) {
        final String name = event.getName();

        System.out.println("ADD: " + name);
    }

    @Override
    public void serviceRemoved(ServiceEvent event) {
        final String name = event.getName();
        System.out.println("REMOVE: " + name);
    }

    @Override
    public void serviceResolved(ServiceEvent event) {
        final String name = event.getName();
        System.out.println("RESOLVED: " + name);
    }

    @Override
    public void serviceTypeAdded(ServiceEvent event) {
        final String aType = event.getType();
        System.out.println("TYPE: " + aType);
    }

    @Override
    public void subTypeForServiceTypeAdded(ServiceEvent event) {
        System.out.println("SUBTYPE: " + event.getType());
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