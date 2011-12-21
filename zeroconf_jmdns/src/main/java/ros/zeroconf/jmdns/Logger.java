package ros.zeroconf.jmdns;

import java.lang.String;

public class Logger implements ZeroconfLogger {

	public void println(String msg) {
		System.out.println(msg);
	}
}
