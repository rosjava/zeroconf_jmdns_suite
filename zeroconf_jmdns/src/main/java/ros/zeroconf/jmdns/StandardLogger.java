package ros.zeroconf.jmdns;

import java.lang.String;

/**
 * A standard logger (to System.out.println) that can be
 * passed to the Zeroconf class.
 */
public class StandardLogger implements ZeroconfLogger {
	public void println(String msg) {
		System.out.println(msg);
	}
}
