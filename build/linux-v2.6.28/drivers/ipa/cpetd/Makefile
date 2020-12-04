################################################################################
# 
# Copyright (c) 2009 ip.access Ltd.
#
#  This program is free software; you can redistribute  it and/or modify it
#  under  the terms of  the GNU General  Public License as published by the
#  Free Software Foundation;  either version 2 of the  License, or (at your
#  option) any later version.
#
# 9 Apr 2008 Created by Simon D Hughes.
#
# CPE Timer Driver
#  x86 support should be built with 3gap_setup.sh i386-redhat-linux environment
##############################################################################
# Filename: 
##############################################################################

#
# main kernel makefile requirments
#
obj-$(CONFIG_IPACCESS_CPETD) += cpetd.o
cpetd-objs := cpetd_main.o


# 
# development support
#

APP=cpetd_test
BOARD_VARIANTS?=xbplus xc

# Comment/uncomment the following line to enable/disable debugging
#DEBUG = y

ifeq ($(DEBUG),y)
  DEBFLAGS = -O -g -DCPETD_DEBUG # "-O" is needed to expand inlines
else
  DEBFLAGS = -O2
endif

EXTRA_CFLAGS += $(DEBFLAGS)
#CFLAGS += -DCPETD_TEST

ifeq ($(ARCH),arm)
  EXTRA_CFLAGS += -DCPETD_ON_ARM
endif

ifeq ($(ARCH),i386)
  export CONFIG_IPACCESS_CPETD=m
  KSRC?= /lib/modules/$(shell uname -r)/build
  CPPFLAGS += -I$(KERNEL_SRC)/include
  EXTRA_CFLAGS += -D$(ARCH) 
endif


PWD    := $(shell pwd)

all: all_$(ARCH)
	@echo " Making $@..."

all_arm: 
	@echo " Making $@"
	@echo " module is built from top level kernel make file using make modules"

all_i386: test cpetd_i386
	@echo " Making $@"

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers
	rm -f $(APP)


##############################################################################
### target for building x86 module for local host.
# IPAcess IT supported redhat EL machines have kernel sources installed as detailed below
cpetd_i386:
	@echo " Making $@..."
	$(MAKE) -C $(KSRC) V=1 KSRC=$(KSRC) SUBDIRS=$(PWD) modules

test:
	$(CROSS_COMPILE)gcc cpetd_test.c $(CFLAGS) -L. -I. -I$(KERNEL_SRC)/include -o $(APP)







