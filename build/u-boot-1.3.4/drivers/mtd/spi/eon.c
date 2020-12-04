/*****************************************************************************
 * BSP Version: 3.2.4, RevisionID: ac30f57, Date: 20100223 17:42:05
 *****************************************************************************/

/*!
* \file eon.c
* \brief Support for Eon Silicon Solutions spi flash devices.
*
* Note: This file is based very heavily on 'stmicro.c'
*
* Copyright (c) 2006-2009 picoChip Designs Ltd
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* All enquiries to support@picochip.com
*/

#include <common.h>
#include <malloc.h>
#include <spi_flash.h>

#include "spi_flash_internal.h"

/* EN25Pxx-specific commands */
#define CMD_EN25PXX_WREN	0x06	/* Write Enable */
#define CMD_EN25PXX_WRDI	0x04	/* Write Disable */
#define CMD_EN25PXX_RDSR        0x05    /* Read Status Register */
#define CMD_EN25PXX_WRSR	0x01	/* Write Status Register */
#define CMD_EN25PXX_READ	0x03	/* Read Data Bytes */
#define CMD_EN25PXX_FAST_READ	0x0b	/* Read Data Bytes at Higher Speed */
#define CMD_EN25PXX_PP		0x02	/* Page Program */
#define CMD_EN25PXX_SE		0xd8	/* Sector Erase */
#define CMD_EN25PXX_BE		0xc7	/* Bulk Erase */
#define CMD_EN25PXX_DP		0xb9	/* Deep Power-down */
#define CMD_EN25PXX_RES		0xab	/* Release from DP, and Read Signature */

#define EON_ID_EN25F05          0x10

#define EON_SR_WIP		(1 << 0)	/* Write-in-Progress */

struct eon_spi_flash_params {
	u8 idcode1;
	u16 page_size;
	u16 pages_per_sector;
	u16 nr_sectors;
	const char *name;
};

/* spi_flash needs to be first so upper layers can free() it */
struct eon_spi_flash {
	struct spi_flash flash;
	const struct eon_spi_flash_params *params;
};

static inline struct eon_spi_flash *to_eon_spi_flash(struct spi_flash *flash)
{
	return container_of(flash, struct eon_spi_flash, flash);
}

static const struct eon_spi_flash_params eon_spi_flash_table[] = {
	{
		.idcode1 = EON_ID_EN25F05,
		.page_size = 256,
		.pages_per_sector = 16,
		.nr_sectors = 16,
		.name = "EN25F05",
	},
};

static int eon_wait_ready(struct spi_flash *flash, unsigned long timeout)
{
	struct spi_slave *spi = flash->spi;
	unsigned long timebase;
	int ret;
	u8 status;

	timebase = get_timer(0);
	do {
		ret = spi_flash_cmd(spi, CMD_EN25PXX_RDSR, &status, sizeof(status));
		if (ret)
			return -1;

		if ((status & EON_SR_WIP) == 0)
			break;

	} while (get_timer(timebase) < timeout);


	if ((status & EON_SR_WIP) == 0)
		return 0;

	/* Timed out */
	return -1;
}

static int eon_read_fast(struct spi_flash *flash,
	    	         u32 offset, size_t len, void *buf)
{
	struct eon_spi_flash *eon = to_eon_spi_flash(flash);
	unsigned long page_addr;
	unsigned long page_size;
	u8 cmd[5];

	page_size = eon->params->page_size;
	page_addr = offset / page_size;

	cmd[0] = CMD_READ_ARRAY_FAST;
	cmd[1] = page_addr >> 8;
	cmd[2] = page_addr;
	cmd[3] = offset % page_size;
	cmd[4] = 0x00;

	return spi_flash_read_common(flash, cmd, sizeof(cmd), buf, len);
}

static int eon_write(struct spi_flash *flash,
		     u32 offset, size_t len, const void *buf)
{
	struct eon_spi_flash *eon = to_eon_spi_flash(flash);
	unsigned long page_addr;
	unsigned long byte_addr;
	unsigned long page_size;
	size_t chunk_len;
	size_t actual;
	int ret;
	u8 cmd[4];

	page_size = eon->params->page_size;
	page_addr = offset / page_size;
	byte_addr = offset % page_size;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		debug("SF: Unable to claim SPI bus\n");
		return ret;
	}

	ret = 0;
	for (actual = 0; actual < len; actual += chunk_len) {
		chunk_len = min(len - actual, page_size - byte_addr);

		cmd[0] = CMD_EN25PXX_PP;
		cmd[1] = page_addr >> 8;
		cmd[2] = page_addr;
		cmd[3] = byte_addr;

		debug
		    ("PP: 0x%p => cmd = { 0x%02x 0x%02x%02x%02x } chunk_len = %d\n",
		     buf + actual, cmd[0], cmd[1], cmd[2], cmd[3], chunk_len);

		ret = spi_flash_cmd(flash->spi, CMD_EN25PXX_WREN, NULL, 0);
		if (ret < 0) {
			debug("SF: Enabling Write failed\n");
			break;
		}

		ret = spi_flash_cmd_write(flash->spi, cmd, 4,
					  buf + actual, chunk_len);
		if (ret < 0) {
			debug("SF: EON Page Program failed\n");
			break;
		}

		ret = eon_wait_ready(flash, SPI_FLASH_PROG_TIMEOUT);
		if (ret < 0) {
			debug("SF: EON page programming timed out\n");
			break;
		}

		page_addr++;
		byte_addr = 0;
	}

	debug("SF: EON: Successfully programmed %u bytes @ 0x%x\n",
	      len, offset);

	spi_release_bus(flash->spi);
	return ret;
}

int eon_erase(struct spi_flash *flash, u32 offset, size_t len)
{
	struct eon_spi_flash *eon = to_eon_spi_flash(flash);
	unsigned long sector_size;
	size_t actual;
	int ret;
	u8 cmd[4];

	/*
	 * This function currently uses sector erase only.
	 * probably speed things up by using bulk erase
	 * when possible.
	 */

	sector_size = eon->params->page_size * eon->params->pages_per_sector;

	if (offset % sector_size || len % sector_size) {
		debug("SF: Erase offset/length not multiple of sector size\n");
		return -1;
	}

	len /= sector_size;
	cmd[0] = CMD_EN25PXX_SE;
	cmd[2] = 0x00;
	cmd[3] = 0x00;

	ret = spi_claim_bus(flash->spi);
	if (ret) {
		debug("SF: Unable to claim SPI bus\n");
		return ret;
	}

	ret = 0;
	for (actual = 0; actual < len; actual++) {
		cmd[1] = (offset / sector_size) + actual;

		ret = spi_flash_cmd(flash->spi, CMD_EN25PXX_WREN, NULL, 0);
		if (ret < 0) {
			debug("SF: Enabling Write failed\n");
			break;
		}

		ret = spi_flash_cmd_write(flash->spi, cmd, 4, NULL, 0);
		if (ret < 0) {
			debug("SF: EON page erase failed\n");
			break;
		}

		ret = eon_wait_ready(flash, SPI_FLASH_PAGE_ERASE_TIMEOUT);
		if (ret < 0) {
			debug("SF: EON page erase timed out\n");
			break;
		}
	}

	debug("SF: EON: Successfully erased %u bytes @ 0x%x\n",
	      len * sector_size, offset);

	spi_release_bus(flash->spi);
	return ret;
}

struct spi_flash *spi_flash_probe_eon(struct spi_slave *spi, u8 * idcode)
{
	const struct eon_spi_flash_params *params;
	struct eon_spi_flash *eon;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(eon_spi_flash_table); i++) {
		params = &eon_spi_flash_table[i];
		if (params->idcode1 == idcode[2]) {
			break;
		}
	}

	if (i == ARRAY_SIZE(eon_spi_flash_table)) {
		debug("SF: Unsupported EON ID %02x\n", idcode[1]);
		return NULL;
	}

	eon = malloc(sizeof(struct eon_spi_flash));
	if (!eon) {
		debug("SF: Failed to allocate memory\n");
		return NULL;
	}

	eon->params = params;
	eon->flash.spi = spi;
	eon->flash.name = params->name;

	eon->flash.write = eon_write;
	eon->flash.erase = eon_erase;
	eon->flash.read = eon_read_fast;
	eon->flash.size = (params->page_size *
                           params->pages_per_sector *
	                   params->nr_sectors);

	debug("SF: Detected %s with page size %u, total %u bytes\n",
	      params->name, params->page_size, eon->flash.size);

	return &eon->flash;
}
