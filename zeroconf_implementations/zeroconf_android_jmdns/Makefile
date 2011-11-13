include $(shell rospack find rosjava_bootstrap)/rosjava.mk

ROSPACK_DIR:=$(shell rospack find zeroconf_jmdns)

# Target to copy across all the files from the zeroconf_jmdns package (argh - fugly)
update: update_jmdns update_zeroconf

update_jmdns:
	cp -r $(ROSPACK_DIR)/src/main/java/javax/* ./src/javax

update_zeroconf:
	sed -e 's/ros\.zeroconf/ros\.android\.zeroconf/g' $(ROSPACK_DIR)/src/main/java/ros/zeroconf/jmdns/Zeroconf.java > ./src/ros/android/zeroconf/jmdns/Zeroconf.java
