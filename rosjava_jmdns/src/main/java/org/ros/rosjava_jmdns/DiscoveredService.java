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
package org.ros.rosjava_jmdns;

import java.util.ArrayList;

/*
 * This class effectively mirrors zeroconf_msgs/DiscoveredService.msg.
 * We don't actually have a node implementation for zeroconf in rosjava yet,
 * so we're not bringing in the msg dependency. If we do write a node
 * implementation (like zeroconf_avahi), then we'll do so.
 */
public class DiscoveredService {
	public String name;
	public String type;
	public String domain;
	public String description;
	public String hostname;
	public ArrayList<String> ipv4_addresses = new ArrayList<String>();
	public ArrayList<String> ipv6_addresses = new ArrayList<String>();
	public int port;
}
