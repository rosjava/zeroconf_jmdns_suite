package ros.zeroconf.jmdns;

import javax.jmdns.ServiceInfo;

/**
 * @brief Interface for a user listener.
 * 
 * Subclass this to create your own listener callbacks and then pass to the zeroconf class
 * when you call the addListener command.
 * 
 * This needs a bit of work on the user's end - jmdns often provides alot of callbacks
 * especially if you are listening on multiple interfaces. You'll need a bit of processing
 * at the serviceResolved callback in particular.
 * 
 * Keep in mind that service info's come often come in with multiple addresses, or even 
 * addresses that have already been resolved on previous callbacks. So when processing
 * resolved services, check the name, port and hostname haven't already been added to your
 * stored entries and if they have, simply append whatever new addresses you find to the 
 * existing entry. 
 * 
 */
public interface ZeroconfListener {
	public void serviceAdded(ServiceInfo service);
	public void serviceRemoved(ServiceInfo service);
	public void serviceResolved(ServiceInfo service);
}
