package ros.zeroconf.jmdns.demos;

import java.io.IOException;
import java.util.List;
import org.ros.message.zeroconf_comms.DiscoveredService;
import ros.zeroconf.jmdns.Zeroconf;
import ros.zeroconf.jmdns.StandardLogger;

public class Publisher {
	
	public static void main_publisher(String argv[]) throws IOException {
        Zeroconf publisher = new Zeroconf(new StandardLogger());
        System.out.println("Publishing DudeMaster for 8 seconds, then removing");
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
