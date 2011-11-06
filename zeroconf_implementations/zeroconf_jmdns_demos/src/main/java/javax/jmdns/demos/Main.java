package javax.jmdns.demos;

import javax.jmdns.demos.NetworkTopology;
import javax.jmdns.demos.RegisterServices;


public class Main {

	static void usage() {
		System.out.println("");
		System.out.println("Usage: java -jar jmdns_demos.jar [demo]");
		System.out.println("  where 'demo' is one of:");
		System.out.println("    --network_topology");
		System.out.println("    --register_services");
		System.out.println("");
	}
	
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		if ( args.length != 1 ) {
			Main.usage();
			return;
		}
		String demo_type = args[0];
		if ( demo_type.equals("--network_topology") ) {
			NetworkTopology.main(args);
		} else if ( demo_type.equals("--register_services") ) {
			RegisterServices.main(args);
		} else if ( demo_type.equals("--discover_services") ) {
			DiscoverServices.main(args);
		} else {
			Main.usage();
			System.out.printf("Done *%s*\n",demo_type);
		}
	}
}
