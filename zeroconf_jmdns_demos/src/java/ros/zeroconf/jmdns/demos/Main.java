package ros.zeroconf.jmdns.demos;

import java.io.IOException;
import java.util.Enumeration;
import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.LogManager;
import java.util.logging.Logger;
import ros.zeroconf.jmdns.Zeroconf;

/**
 * Main sample program for JmDNS.
 *
 * @author Arthur van Hoff, Werner Randelshofer
 */
public class Main {
    public static void main(String argv[]) throws IOException {
        int argc = argv.length;
        boolean debug = false;

        if ((argc > 0) && "--debug".equals(argv[0])) {
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

		if (argc == 0) {
        	Main.usage();
            System.exit(1);
		} else if ( (argc >= 1) && "--jmdns-polling".equals(argv[0])) {
			Discovery.main_jmdns_polling(argv);
		} else if ( (argc >= 1) && "--polling".equals(argv[0])) {
			Discovery.main_polling(argv);
		} else if ( "--publisher".equals(argv[0]) ) {
			Zeroconf.main_publisher(argv);
		} else if ( "--discovery".equals(argv[0]) ) {
			Discovery.main_handler(argv);
        } else {
        	Main.usage();
            System.exit(1);
        }
    }
    public static void usage() {
        System.out.println();
        System.out.println("jmdns:");
        System.out.println("     --debug          - output debugging info");
        System.out.println("     --publisher      - publishes a _ros-master._tcp for 8s");
        System.out.println("     --polling        - polling discovery for ros masters (_ros-master._tcp)");
        System.out.println("     --discovery      - callback discovery for ros services (ros, concert, app-manager)");
        System.out.println();
    }
}
