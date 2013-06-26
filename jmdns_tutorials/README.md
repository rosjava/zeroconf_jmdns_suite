# Jmnds Tutorials

There are three demo programs. First build the jmdns_tutorials package and then:

**Publishing**

Start listening in one shell:

```
> avahi-browse _ros-master._tcp
```

And execute the publisher in another shell:

```
./build/install/jmdns_tutorials/bin/jmdns_tutorials --publisher
```

**Discovery - Polling**

Start publishing in one shell:

```
> avahi-publish -s DudeMaster _ros-master._tcp 8882
```

And execute polling discovery in another (it will print out the discovered list every second):

```
./build/install/jmdns_tutorials/bin/jmdns_tutorials --polling
```

**Discovery - Callback**

As above, but start the discovery handler:

```
./build/install/jmdns_tutorials/bin/jmdns_tutorials --polling
```


