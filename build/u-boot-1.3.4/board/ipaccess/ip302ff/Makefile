#/*****************************************************************************
# * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
# *****************************************************************************/

#/*!
#* \file Makefile
#* \brief Used during the build process.
#*
#* Copyright (c) 2006-2009 picoChip Designs Ltd
#*
#* This program is free software; you can redistribute it and/or modify
#* it under the terms of the GNU General Public License version 2 as
#* published by the Free Software Foundation.
#*
#* All enquiries to support@picochip.com
#*/

include $(TOPDIR)/config.mk

LIB	= lib$(BOARD).a

OBJS	:= pc7302.o mt29f2g08aadwp.o cmd_ip302ff.o
SOBJS	:= lowlevel_init.o

$(LIB):	$(OBJS) $(SOBJS)
	$(AR) crv $@ $^

clean:
	rm -f $(SOBJS) $(OBJS)

distclean:	clean
	rm -f $(LIB) core *.bak .depend

#########################################################################

.depend:	Makefile $(SOBJS:.o=.S) $(OBJS:.o=.c)
		$(CC) -M $(CPPFLAGS) $(SOBJS:.o=.S) $(OBJS:.o=.c) > $@

-include .depend

#########################################################################
