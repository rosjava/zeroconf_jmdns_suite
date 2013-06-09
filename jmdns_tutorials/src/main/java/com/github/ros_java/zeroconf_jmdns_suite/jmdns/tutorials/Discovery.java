/*
 * Copyright (C) 2013 Yujin Robot.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
package com.github.ros_java.zeroconf_jmdns_suite.jmdns.tutorials;

import java.io.IOException;
import java.util.List;

import com.github.ros_java.zeroconf_jmdns_suite.jmdns.DiscoveredService;
import com.github.ros_java.zeroconf_jmdns_suite.jmdns.StandardLogger;
import com.github.ros_java.zeroconf_jmdns_suite.jmdns.Zeroconf;

public class Discovery {
	
	public static void main_handler() throws IOException {
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

    public static void main_polling() throws IOException {
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
