package ros.zeroconf.jmdns.demos;

import java.io.IOException;
import ros.zeroconf.jmdns.Zeroconf;
import ros.zeroconf.jmdns.StandardLogger;

public class Discovery {
	
	public static void main(String argv[]) throws IOException {
        Zeroconf zeroconf = new Zeroconf(new StandardLogger());
        Listener listener = new Listener();
		zeroconf.setDefaultListenerCallback(listener);
        zeroconf.addListener("_ros-master._tcp","local");
        zeroconf.addListener("_ros-master._udp","local");
        zeroconf.addListener("_concert-master._tcp","local");
        zeroconf.addListener("_concert-master._udp","local");
        zeroconf.addListener("_app-manager._tcp","local");
        zeroconf.addListener("_app-manager._udp","local");
        while(true) {
    		try {
        		Thread.sleep(1000L);
		    } catch (InterruptedException e) {
		        e.printStackTrace();
		    }
        }
    }

}
