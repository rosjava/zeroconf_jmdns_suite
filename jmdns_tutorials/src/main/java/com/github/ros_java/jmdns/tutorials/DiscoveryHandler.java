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
package com.github.ros_java.jmdns.tutorials;

import com.github.ros_java.jmdns.DiscoveredService;
import com.github.ros_java.jmdns.ZeroconfDiscoveryHandler;

/**
 * This class is a handler that can be passed to
 * a ros.zeroconf.jmdns.Zeroconf instance to to allow zeroconf discovery callbacks 
 * (service added, resolved, removed) to be processed in a custom way for the user
 * of the class. 
 */
public class DiscoveryHandler implements ZeroconfDiscoveryHandler {
	
	public void serviceAdded(DiscoveredService discovered_service) {
		String result = "[+] " + discovered_service.name + "." + discovered_service.type + "." + discovered_service.domain + ".";
    	System.out.println(result);
	}
	public void serviceRemoved(DiscoveredService discovered_service) {
		String result = "[-] " + discovered_service.name + "." + discovered_service.type + "." + discovered_service.domain + ".";
    	System.out.println(result);
	}
	public void serviceResolved(DiscoveredService discovered_service) {
		String result = "[=] " + discovered_service.name + "." + discovered_service.type + "." + discovered_service.domain + ".\n";
    	result += "    Port     : " + discovered_service.port + "\n";
    	result += "    Hostname : " + discovered_service.hostname + "\n";
    	for ( String address : discovered_service.ipv4_addresses ) {
    		result += "    Address: " + address + "\n";
    	}
    	for ( String address : discovered_service.ipv6_addresses ) {
    		result += "    Address: " + address + "\n";
    	}
    	System.out.printf(result);
	}
}   
