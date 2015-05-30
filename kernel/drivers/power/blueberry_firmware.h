/*
 * BlueBerry firmware update driver integrated into battery driver.
 *
 * Copyright (C) 2013-2013 Paul Ma <magf@bitland.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#ifndef __BLUEBERRY_FW_H
#define __BLUEBERRY_FW_H

#define EC_XFER_LEN		32
#define EC_READ_STEP	256

/* flash command id */
#define CMD_RDID		0xab		/* Read Manufacturer and Product ID */
#define CMD_JEDEC_ID	0x9f		/* Read Manufacturer and Product ID by JEDEC ID command */
#define CMD_RDMDID		0x90		/* Read Manufacturer and Device ID */
#define CMD_WREN		0x06		/* Write Enable */
#define CMD_WRDI		0x04		/* Write Disable */
#define CMD_RDSR		0x05		/* Read Status Register */
#define CMD_WRSR		0x01		/* Write status Register */
#define CMD_READ		0x03		/* Read data bytes from memory at normal read mode */
#define CMD_FAST_READ	0x0b		/* Read data bytes from memory at fast read mode */
#define CMD_FRDO		0x3b		/* Read data dual ouput */
#define CMD_PAGE_PROG	0x02		/* Page program data bytes into memory */
#define CMD_SECTOR_ER	0x07		/* Sector Erase */
#define CMD_BLOCK_ER	0x08		/* Block Erase */
#define CMD_CHIP_ER		0xc7		/* Chip Erase */

/* CMD_RDSR */
#define RDSR_WIP		BIT(0)		/* Write In Progress bit, when 0, device is ready for a write status register, program or erase,otherwise busy */
#define RDSR_WEL		BIT(1)		/* Write Enable Latch, when 1, write operations allow, other wise not */
#define RDSR_BP0		BIT(2)
#define RDSR_BP1		BIT(3)
#define RDSR_BP2		BIT(4)
#define RDSR_SRWD		BIT(7)		/* when SRWD is 1 and WP# is low, SRWD, BP2, BP1, BP0 become read-only, WRSR will be ignored. */

#define CMD_GET_VERSION           0x10
#define CMD_UN_PROTECT            0x20
#define CMD_GET_ECFW_VERSION      0x30
#define CMD_FW_UPGRADE            0x40

#define SHORT_RETRY_COUNT	0x0a
#define LONG_RETRY_COUNT	500		/* in fact it is 500 * 10 ms = 5 seconds */

int blueberry_fw_driver_init(struct blueberry_bat *bbbat);
int blueberry_fw_driver_uninit(struct blueberry_bat *bbbat);
int blueberry_fw_get_version(struct blueberry_bat *bbbat, int *ver);
int blueberry_fw_read_firmware(struct blueberry_bat *bbbat, unsigned char *buf, int len);

#if defined(CONFIG_BLUEBERRY_FW_KERNEL_UPGRADE)
int blueberry_fw_kernel_upgrade(struct blueberry_bat *bbbat);
#endif

#endif
