package javax.jmdns.demos;

import java.io.IOException;
import javax.jmdns.JmDNS;
import javax.jmdns.JmmDNS;
import javax.jmdns.ServiceEvent;
import javax.jmdns.ServiceInfo;
import javax.jmdns.ServiceListener;
import java.util.Map;
import java.util.HashMap;
import java.lang.Thread;
import java.net.InetAddress;

/**
 * Sample Code for Service Discovery using JmDNS and a ServiceListener.
 * <p>
 * Run the main method of this class. It listens for HTTP services and lists all changes on System.out.
 *
 * @author Werner Randelshofer
 */
public class DiscoverServices {

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

    public static void listen_single_instance() {
        try {
            String serviceKey = "srvname"; // Max 9 chars
            String text = "Test hypothetical ros master";
            Map<String, byte[]> properties = new HashMap<String, byte[]>();
            properties.put(serviceKey, text.getBytes());
            ServiceInfo service = ServiceInfo.create(
            		"_ros-master._tcp.local.", 
            		"Ros Master", 
            		8000, 0, 0, true, properties);
            InetAddress address;
            address = InetAddress.getByName("192.168.10.66"); 
            JmDNS jmdns = JmDNS.create(address);
        	String ros_service_type = "_ros-master._tcp.local.";
            jmdns.addServiceListener(ros_service_type, new SampleListener());
            System.out.println("Press q to quit");
            int b;
            while ((b = System.in.read()) != -1 && (char) b != 'q') {
            	ServiceInfo[] ros_services = jmdns.list(ros_service_type);
            	System.out.printf("Discovered %1d ros services%n", ros_services.length);
            }
            System.out.println("Closing...");
            jmdns.close();
            System.out.println("Done");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    /**
     * This does not work - bloody annoying!
     */
    public static void listen_multiple_instances() {
        try {
//            InetAddress address;
//            address = InetAddress.getByName("192.168.10.66"); 
            JmmDNS jmmdns = JmmDNS.Factory.getInstance();
        	String ros_service_type = "_ros-master._tcp.local.";
            jmmdns.addServiceListener(ros_service_type, new SampleListener());
            System.out.println("Press q to quit");
            int b;
            while ((b = System.in.read()) != -1 && (char) b != 'q') {
            	ServiceInfo[] ros_services = jmmdns.list(ros_service_type);
            	System.out.printf("Discovered %1d ros services%n", ros_services.length);
            }
            System.out.println("Closing...");
            jmmdns.close();
            System.out.println("Done");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    /**
     * @param args
     *            the command line arguments
     */
    public static void main(String[] args) {
    	listen_single_instances();
    }
}
