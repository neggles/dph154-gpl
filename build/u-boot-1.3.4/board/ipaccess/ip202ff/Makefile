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

OBJS	:= cpe20x.o cmd_ip202ff.o
SOBJS	:= lowlevel_init.o

$(LIB):	$(OBJS) $(SOBJS)
	$(AR) crv $@ $^

# (uncomment these lines if want assembler for cmd_ip202ff - will break the build!)
#cmd_ip202ff.o: cmd_ip202ff.c
#	$(CC) $(CPPFLAGS) -S cmd_ip202ff.c
	
clean:
	rm -f $(SOBJS) $(OBJS)

distclean:	clean
	rm -f $(LIB) core *.bak .depend

#########################################################################

.depend:	Makefile $(SOBJS:.o=.S) $(OBJS:.o=.c)
		$(CC) -M $(CPPFLAGS) $(SOBJS:.o=.S) $(OBJS:.o=.c) > $@

-include .depend

#########################################################################
