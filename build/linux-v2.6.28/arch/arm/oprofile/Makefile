################################################################################
# 
# Copyright (c) 2009 ip.access Ltd.
#
#  This program is free software; you can redistribute  it and/or modify it
#  under  the terms of  the GNU General  Public License as published by the
#  Free Software Foundation;  either version 2 of the  License, or (at your
#  option) any later version.
#
# 22 Apr 2009 Created by Simon D Hughes.
##############################################################################
# Filename: 
##############################################################################
obj-$(CONFIG_OPROFILE) += oprofile.o

DRIVER_OBJS = $(addprefix ../../../drivers/oprofile/, \
		oprof.o cpu_buffer.o buffer_sync.o \
		event_buffer.o oprofile_files.o \
		oprofilefs.o oprofile_stats.o \
		timer_int.o )

oprofile-y				:= $(DRIVER_OBJS) common.o backtrace.o
oprofile-$(CONFIG_CPU_XSCALE)		+= op_model_xscale.o
oprofile-$(CONFIG_OPROFILE_ARM11_CORE)	+= op_model_arm11_core.o
oprofile-$(CONFIG_OPROFILE_ARMV6)	+= op_model_v6.o
oprofile-$(CONFIG_OPROFILE_MPCORE)	+= op_model_mpcore.o
oprofile-$(CONFIG_OPROFILE_ARMV7)	+= op_model_v7.o
oprofile-$(CONFIG_FIRECRACKER_ASYNC_OPROF)	+= op_model_firecracker.o
oprofile-$(CONFIG_PC302_ASYNC_OPROF)	+= op_model_pc302.o
