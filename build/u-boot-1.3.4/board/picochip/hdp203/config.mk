#
# $picoChipHeaderSubst$
#

#
# (C) Copyright 2004
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# Configured for picoChip HDP target (www.picochip.com)
#     by david.warman@zetetica.co.uk
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

#
# picoChip HDP203 board
#

# If we want a RAM boot....
#ifeq ($(ramsym),1)
#TEXT_BASE = 0x07FD0000
#CFG_RAMBOOT = 1
#else
# Reserve 512K for boot at top of flash; first 256K is environment
TEXT_BASE = 0xFFFC0000
#endif

PLATFORM_CPPFLAGS += -DCONFIG_MPC85xx=1
PLATFORM_CPPFLAGS += -DCONFIG_MPC8560=1
PLATFORM_CPPFLAGS += -DCONFIG_E500=1

ifeq ($(debug),1)
PLATFORM_CPPFLAGS += -DDEBUG
endif

ifdef BSP_DIR
include $(BSP_DIR)/config
PLATFORM_CPPFLAGS += -DPICOCHIP_PLATFORM_VERSION=\"$(RELEASE_VERSION)\"
endif

#ifeq ($(dbcr),1)
#PLATFORM_CPPFLAGS += -DCFG_INIT_DBCR=0x8cff0000
#endif
