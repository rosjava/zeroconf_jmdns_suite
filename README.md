See [rosjava_core](https://github.com/rosjava/rosjava_core) readme.

# Zeroconf JmDNS #

This repository packages android friendly sources from the [jmdns](http://jmdns.sourceforge.net/) project
and provides some tutorial demos.

## The JmDNS Library ##

### About ###

JmDNS works a little differently from avahi, although it does the same job. You can separate its
functionality into two parts:

1) Zeroconf services (publishing, discovery and resolution) on a fixed, specified interface.
2) Network topology aware zeroconf services (in the author's language - multi-homed).

What the latter means, is that it will be automatically detect what network interfaces you have
and also be aware of those interfaces going up and down whilst providing a dynamic awareness of
the zeroconf services that appear and disappear with those interfaces.

The network topology part of jmdns is still fairly experimental.

### The Experimental Part ###

The network topology aware part of zeroconf is quite important. On android in particular it saves
you having to worry about detecting the ip of your interface, which is rather handy. This part is
quite reliable after having helped with some patches upstream.

It does have problems however handling multiple network interfaces and can also be confusing with the
way it handles ipv6. I have some patches here in this code set which get it working, but either aren't
upstream yet, or probably not the proper solution yet.

Hence we have a snapshot of the sources here.

### Publishing ###

Publishing is relatively straightforward. For ros environments, we typically only ever want or need to
publish the ros master, in which case you'd publish it under something like:

```
_ros-master._tcp 11311
```

### Discovery ###

There are two methods in the jmdns library for discovery. The first is via *polling*, which will always be
the far more reliable method (especially if your interface is up and down alot and likely to get
jmdns confused). The second is via event based *listeners* which give you callbacks to instantly
react on a zeroconf service coming up or down.

## The ROS Wrapper ##

I've got a couple of classes set up to more easily handle the jmdns library for our purposes (jmdns
can be quite messy to program with). It's not a full node, but it wouldn't be a stretch to
convert this into a full node's functionality using zeroconf_msgs as the basis for handles that work
in a similar way to the ros zeroconf_avahi package.

## Tutorials ##

Instructions can be found in the readme in the jmdns_tutorials package.
