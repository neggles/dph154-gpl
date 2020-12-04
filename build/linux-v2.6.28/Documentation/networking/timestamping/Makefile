# BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
#
# Makefile for pc302 hardware timestamp testing
#
# Make sure the arm cross development environment is setup in the 
# terminal window, then issue a 'make' command.
#
# The resulting 'timestamping' executable can then be copied to the 
# root filesystem of the cpe20x platform and run.
#

CC=$(CROSS_COMPILE)gcc
AR=$(CROSS_COMPILE)ar
LD=$(CROSS_COMPILE)ld

CFLAGS=-Wall -g
CPPFLAGS=-I../../../include

ARCH := $(shell $(CC) -dumpmachine)

ifeq ("arm-none-linux-gnueabi", "$(TARGET_NAME)")
    CPPFLAGS += -DARCH_ARM
endif

TARGET=timestamping

$(TARGET): timestamping.o

all:	$(TARGET)

clean:
	rm -f $(TARGET)
