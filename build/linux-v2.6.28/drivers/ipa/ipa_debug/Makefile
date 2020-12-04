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
# Makefile for the ipa_debug
# 
##############################################################################
# Filename: 
##############################################################################


obj-$(CONFIG_IPACCESS_DEBUG) += ipa_debug.o
ipa_debug-objs := ipa_debug_main.o


lib:
	$(CROSS_COMPILE)gcc -c ipa_debug_card.c -o ipa_debug_card.o
	$(CROSS_COMPILE)ar r libipa_debug_card.a ipa_debug_card.o

test:
	$(CROSS_COMPILE)gcc ipa_debug_card_test.c -L. -I. -lipa_debug_card -o ipa_debug_card_test
