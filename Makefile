KDIR ?= /home/mcdermj/yocto/build/tmp/work/socfpga_cyclone5-poky-linux-gnueabi/linux-altera-dist-1.0-r1/linux-socfpga/

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
