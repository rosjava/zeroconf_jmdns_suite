package yujin.sandbox;

import java.net.URI;
import java.net.URISyntaxException;
import org.ros.address.InetAddressFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.DefaultNodeFactory;
//import org.ros.message.std_msgs.String;

public class Main {
	/**
	 * @param args
	 */
	public static void main(String[] args) {
		System.out.println("Hello Dudes");
		org.ros.message.std_msgs.String s = new org.ros.message.std_msgs.String();
		s.data = "dude";
		String masterUri = "http://localhost:11311";
		//System.out.println(System.getProperty("java.class.path"));
		try {
			URI uri = new URI(masterUri);
			String host = InetAddressFactory.newNonLoopback().getHostName();
			NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, uri);
			String defaultNodeName = "dude";
			
			DefaultNodeFactory factory = new DefaultNodeFactory();
			Node node = factory.newNode(defaultNodeName, nodeConfiguration);
			
			System.out.println("Shutting down");
			node.shutdown();
		} catch (URISyntaxException e) {
			System.out.println("URI syntax exception");
		}
		System.out.println("We're done.");
	}
}
