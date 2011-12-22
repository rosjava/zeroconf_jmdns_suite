package ros.zeroconf.jmdns;

import javax.jmdns.ServiceInfo;

/**
 * @brief Interface for a user listener.
 * 
 * Subclass this to create your own listener callbacks and then pass to the zeroconf class
 * when you call the addListener command.
 */
public interface ZeroconfListener {
	public void serviceAdded(ServiceInfo service);
	public void serviceRemoved(ServiceInfo service);
	public void serviceResolved(ServiceInfo service);
}
