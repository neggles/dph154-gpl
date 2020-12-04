#/*****************************************************************************
# * $picoChipHeaderSubst$
# *****************************************************************************/

#/*!
#* \file config.mk
#* \brief Used during the build process.
#*
#* Copyright (c) 2006-2008 picoChip Designs Ltd
#*
#* This program is free software; you can redistribute it and/or modify
#* it under the terms of the GNU General Public License version 2 as
#* published by the Free Software Foundation.
#*
#* All enquiries to support@picochip.com
#*/

# The SVB302 U-Boot image should be re-located to address...
TEXT_BASE = 0x00020000

ifdef BSP_DIR
include $(BSP_DIR)/config
PLATFORM_CPPFLAGS += -DPICOCHIP_PLATFORM_VERSION=\"$(RELEASE_VERSION)\"
endif

ifeq ($(SVB302_RUN_FROM_RAM), Y)
PLATFORM_CPPFLAGS += -DCONFIG_RUN_FROM_RAM
endif
