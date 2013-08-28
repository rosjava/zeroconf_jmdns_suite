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

package com.github.rosjava.zeroconf_jmdns_suite.jmdns.tutorials;

import java.io.IOException;
//import java.util.Enumeration;
//import java.util.logging.ConsoleHandler;
//import java.util.logging.Level;
//import java.util.logging.LogManager;
//import java.util.logging.Logger;

/**
 * Modified main sample program for JmDNS.
 * 
 * If testing with an avahi publisher on linux, use:
 * 
 * @code
 * > avahi-publish -s DudeMaster _ros-master._tcp 8882
 * @endcode
 */
public class Main {
    public static void main(String argv[]) throws IOException {
        int argc = argv.length;
//        boolean debug = false;
//
//        if ((argc > 0) && "--debug".equals(argv[0])) {
//            System.arraycopy(argv, 1, argv, 0, --argc);
//
//            {
//                ConsoleHandler handler = new ConsoleHandler();
//                handler.setLevel(Level.FINEST);
//                for (Enumeration<String> enumerator = LogManager.getLogManager().getLoggerNames(); enumerator.hasMoreElements();) {
//                    String loggerName = enumerator.nextElement();
//                    Logger logger = Logger.getLogger(loggerName);
//                    logger.addHandler(handler);
//                    logger.setLevel(Level.FINEST);
//                }
//            }
//            debug = true;
//        }
//
		if (argc == 0) {
        	Main.usage();
            System.exit(1);
		} else if ( "--polling".equals(argv[0])) {
			Discovery.main_polling();
		} else if ( "--publisher".equals(argv[0]) ) {
			Publisher.main_publisher();
		} else if ( "--discovery".equals(argv[0]) ) {
			Discovery.main_handler();
        } else {
        	Main.usage();
            System.exit(1);
        }
    }
    public static void usage() {
        System.out.println();
        System.out.println("This bundle incorporates several jmdns tutorials, specify which one you wish to run:\n");
        System.out.println("     --publisher      - publishes a _ros-master._tcp for 8s");
        System.out.println("     --polling        - polling discovery for ros masters (_ros-master._tcp)");
        System.out.println("     --discovery      - callback discovery for ros services (ros, concert, app-manager)");
        System.out.println();
    }
}
