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

# The PC7203 U-Boot image should be re-located to address...
TEXT_BASE = 0x06000000

ifdef BSP_DIR
include $(BSP_DIR)/config
PLATFORM_CPPFLAGS += -DPICOCHIP_PLATFORM_VERSION=\"$(RELEASE_VERSION)\"
endif

ifeq ($(PC7302_RUN_FROM_RAM), Y)
PLATFORM_CPPFLAGS += -DCONFIG_RUN_FROM_RAM
endif

#/* Include support / commands for NAND Flash
# *
# * Note: Please read the comments in file
# *       board/picochip/pc7302/mt29f2g08aadwp.c about gpio pins used
# *       and PC302 booting modes before defining CONFIG_CMD_NAND
# */
ifeq ($(PC7302_NAND_SUPPORT), Y)
PLATFORM_CPPFLAGS += -DCONFIG_CMD_NAND
endif

ifeq ($(PC7302_UBIFS), Y)
PLATFORM_CPPFLAGS += -DPICO_USE_UBIFS
endif

ifeq ($(PC7302_THUMB), Y)
PLATFORM_CPPFLAGS += -DBUILD_FOR_THUMB

endif
