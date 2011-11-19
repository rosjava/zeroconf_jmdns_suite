# This isn't exactly stable yet
# Biggest problem is it won't generate a local.properties automagically for us
#include $(shell rospack find rosjava_bootstrap)/rosjava.mk

PACKAGE_NAME=$(shell basename $(PWD))

.PHONY: all clean test

all:
	@if [ ! -f local.properties ]; then \
		android update project --path .; \
	fi
	rosrun rosjava_bootstrap make.py $(PACKAGE_NAME)

clean:
	-rosrun rosjava_bootstrap make.py $(PACKAGE_NAME) clean

test: all
	rosrun rosjava_bootstrap make.py $(PACKAGE_NAME) test


