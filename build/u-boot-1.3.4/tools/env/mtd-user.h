/*
 * $Id: PC202/uboot1.x/u-boot-1.3.4-ipa-3.2.4-patch 1.4.1.1 2011/04/26 12:44:40BST Ian Hamilton (ih3) Exp  $
 *
 * MTD ABI header for use by user space only.
 */

#ifndef __MTD_USER_H__
#define __MTD_USER_H__

#include <stdint.h>

/* This file is blessed for inclusion by userspace */
#include <mtd/mtd-abi.h>

typedef struct mtd_info_user mtd_info_t;
typedef struct erase_info_user erase_info_t;
typedef struct region_info_user region_info_t;
typedef struct nand_oobinfo nand_oobinfo_t;

#endif /* __MTD_USER_H__ */
