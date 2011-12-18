// Licensed under Apache License version 2.0
// Original license LGPL

//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

package ros.zeroconf.jmdns.demos;

import java.io.IOException;
import java.util.Enumeration;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.LogManager;
import java.util.logging.Logger;

/**
 * Main sample program for JmDNS.
 *
 * @author Arthur van Hoff, Werner Randelshofer
 */
public class Main {
    public static void main(String argv[]) throws IOException {
        int argc = argv.length;
        boolean debug = false;

        if ((argc > 0) && "-d".equals(argv[0])) {
            System.arraycopy(argv, 1, argv, 0, --argc);

            {
                ConsoleHandler handler = new ConsoleHandler();
                handler.setLevel(Level.FINEST);
                for (Enumeration<String> enumerator = LogManager.getLogManager().getLoggerNames(); enumerator.hasMoreElements();) {
                    String loggerName = enumerator.nextElement();
                    Logger logger = Logger.getLogger(loggerName);
                    logger.addHandler(handler);
                    logger.setLevel(Level.FINEST);
                }
            }
            debug = true;
        }

		if ((argc == 0) || ((argc >= 1) && "-listener".equals(argv[0]))) {
			Zeroconf.main_listener(argv);
		} else if ( "-publisher".equals(argv[0]) ) {
			Zeroconf.main_publisher(argv);
        } else if (!debug) {
            System.out.println();
            System.out.println("jmdns:");
            System.out.println("     -d                                       - output debugging info");
            System.out.println("     -listener                                  - listens for _ros-master._tcp");
            System.out.println("     -publisher                                 - publishes a _ros-master._tcp type");
            System.out.println();
            System.exit(1);
        }
    }
}
