package org.ros.jmdns;

import java.io.IOException;
//import java.lang.Thread;
import java.net.InetAddress;

import javax.jmdns.JmDNS;
import javax.jmdns.JmmDNS;
import javax.jmdns.ServiceListener;
import javax.jmdns.ServiceTypeListener;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;


public class Browser2 implements ServiceListener, ServiceTypeListener {

	JmmDNS	jmmdns;
	
	Browser2(JmmDNS mmDNS) throws IOException {
		
		this.jmmdns = mmDNS;
		this.jmmdns.addServiceTypeListener(this);
		
	    // register some well known types
        // String list[] = new String[] { "_http._tcp.local.", "_ftp._tcp.local.", "_tftp._tcp.local.", "_ssh._tcp.local.", "_smb._tcp.local.", "_printer._tcp.local.", "_airport._tcp.local.", "_afpovertcp._tcp.local.", "_ichat._tcp.local.",
        // "_eppc._tcp.local.", "_presence._tcp.local.", "_rfb._tcp.local.", "_daap._tcp.local.", "_touchcs._tcp.local." };
        String[] list = new String[] {}; 
        // actually shouldn't need thsi anymore, it automatically registers new types as it finds them.
        for (int i = 0; i < list.length; i++) {
            this.jmmdns.registerServiceType(list[i]);
        }   
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
//        if (name.equals(serviceList.getSelectedValue())) {
//            ServiceInfo[] serviceInfos = this.jmmdns.getServiceInfos(type, name);
//            this.dislayInfo(serviceInfos);
//            // this.dislayInfo(new ServiceInfo[] { event.getInfo() });
//        }
    }
	
	/*************************************************************************
	 * Type Listener Callbacks 
	 ************************************************************************/
	@Override
    public void serviceTypeAdded(ServiceEvent event) {
        final String aType = event.getType();
        System.out.println("TYPE: " + aType);
    }
	@Override
    public void subTypeForServiceTypeAdded(ServiceEvent event) {
        System.out.println("SUBTYPE: " + event.getType());
    }


	public static void main(String[] args) throws IOException {
		System.out.println("Hey Dudes");
		InetAddress intf = InetAddress.getLocalHost();
		JmDNS jmdns = JmDNS.create(intf, "Browser");
		new Browser2(JmmDNS.Factory.getInstance());
//        try {
//            int b;
//	        while ((b = System.in.read()) != -1 && (char) b != 'q') {
//	    		try {
//	        		Thread.sleep(1000L);
//			    } catch (InterruptedException e) {
//			        e.printStackTrace();
//			    }
//	        }
//	    } catch (IOException e) {
//	        e.printStackTrace();
//	    }
	}
}
