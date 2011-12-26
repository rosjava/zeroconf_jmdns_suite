package ros.zeroconf.jmdns.demos;

import java.io.IOException;
import java.util.List;
import org.ros.message.zeroconf_comms.DiscoveredService;
import ros.zeroconf.jmdns.Zeroconf;
import ros.zeroconf.jmdns.StandardLogger;

public class Discovery {
	
	public static void main_handler(String argv[]) throws IOException {
        Zeroconf zeroconf = new Zeroconf(new StandardLogger());
        DiscoveryHandler listener = new DiscoveryHandler();
		zeroconf.setDefaultDiscoveryCallback(listener);
        zeroconf.addListener("_ros-master._tcp","local");
//        zeroconf.addListener("_ros-master._udp","local");
//        zeroconf.addListener("_concert-master._tcp","local");
//        zeroconf.addListener("_concert-master._udp","local");
//        zeroconf.addListener("_app-manager._tcp","local");
//        zeroconf.addListener("_app-manager._udp","local");
        while(true) {
    		try {
        		Thread.sleep(1000L);
		    } catch (InterruptedException e) {
		        e.printStackTrace();
		    }
        }
    }

    public static void main_polling(String argv[]) throws IOException {
        Zeroconf browser = new Zeroconf(new StandardLogger());
        browser.addListener("_ros-master._tcp","local");
        int i = 0;
        while( i < 8 ) {
    		try {
    			List<DiscoveredService> discovered_services = browser.listDiscoveredServices();
    			if ( discovered_services.size() > 0 ) {
        			System.out.println("************ Discovered Services ************");
	    			for ( DiscoveredService discovered_service : discovered_services ) {
		        		browser.display(discovered_service);
	    			}
    			} else {
    				System.out.println("Waiting for resolvers...");
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


}
