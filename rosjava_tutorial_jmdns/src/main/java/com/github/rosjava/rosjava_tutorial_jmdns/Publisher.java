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
package com.github.rosjava.rosjava_tutorial_jmdns;

import java.io.IOException;

import com.github.rosjava.rosjava_jmdns.StandardLogger;
import com.github.rosjava.rosjava_jmdns.Zeroconf;

public class Publisher {
	
	public static void main_publisher() throws IOException {
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
