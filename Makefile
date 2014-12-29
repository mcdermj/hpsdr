export ARCH := arm
export CROSS_COMPILE := armv7l-unknown-linux-gnueabihf-
KDIR ?= ../linux-socfpga

upload: default
	scp hpsdr.ko 198.178.136.63:

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

help:
	$(MAKE) -C $(KDIR) M=$$PWD help

modules:
	$(MAKE) -C $(KDIR) M=$$PWD modules

modules_install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

