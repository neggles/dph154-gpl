#/*****************************************************************************
# * $picoChipHeaderSubst$
# *****************************************************************************/

#/*!
#* \file config.mk
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

TEXT_BASE = 0x05000000

ifdef BSP_ARCH
ifeq ($(CPE20X_2BANKS), Y)
PLATFORM_CPPFLAGS += -DCONFIG_PC20X_2_DDR_RAM_BANKS
endif
endif

ifdef BSP_DIR
include $(BSP_DIR)/config
PLATFORM_CPPFLAGS += -DPICOCHIP_PLATFORM_VERSION=\"$(RELEASE_VERSION)\"
endif

ifeq ($(CPE20X_RUN_FROM_RAM), Y)
PLATFORM_CPPFLAGS += -DCONFIG_RUN_FROM_RAM
endif
