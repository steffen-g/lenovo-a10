/*
 * BlueBerry battery driver based on EC
 *
 * Copyright (C) 2013-2013 Paul Ma <magf@bitland.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#define pr_fmt(fmt) KBUILD_BASENAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <asm/unaligned.h>
#include <linux/idr.h>
#include <asm/atomic.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <mach/board.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/cdev.h>
#include <linux/vmalloc.h>
#include "blueberry_batfwi2c.h"
#include "blueberry_firmware.h"

#define blueberry_fw_debug(fmt, ...)                         \
    do {                                                     \
        if (bbbat->fw_debug)                          		 \
            pr_info(pr_fmt(fmt), ##__VA_ARGS__);             \
    } while (0)

#define blueberry_fw_assert_mode(mode)																						\
	do {																													\
		if(atomic_read(&bbbat->working_mode) != mode)	{																	\
			pr_err("mode assert failed, working_mode=%d, assert_mode = %d\n", atomic_read(&bbbat->working_mode), mode);		\
		}																													\
	} while (0)


int blueberry_fw_get_version(struct blueberry_bat *bbbat, int *ver)
{
	int ret;
	unsigned char buf[2];
	
	blueberry_fw_assert_mode(NORMAL_MODE);

	/* to prevent to use i2c bus at the same time */
	mutex_lock(&bbbat->xfer_lock);
	ret = blueberry_bat_smbus_read_word_data(bbbat->client, 0xec, buf, 2);
	mutex_unlock(&bbbat->xfer_lock);
	if(ret){
		blueberry_fw_debug("read ec firmware version failed.\n");
		return ret;
	}

	*ver = (buf[0] << 8) | buf[1];
	blueberry_fw_debug("ec version read, buf[0]=0x%x, buf[1]=0x%x, ver=0x%x\n", buf[0], buf[1], *ver);

	return 0;
}

static int blueberry_fw_exit_flash_mode(struct blueberry_bat *bbbat)
{	
	int ret;

	blueberry_fw_assert_mode(FLASH_MODE);

	ret = blueberry_bat_smbus_write_byte_no_data(bbbat->client, 0xef);
	if(ret) {
		blueberry_fw_debug("exit flash mode write byte failed.\n");
		return ret;
	}
	mdelay(1);

	atomic_set(&bbbat->working_mode, NORMAL_MODE);

	return ret;
}

static int blueberry_fw_enter_flash_mode(struct blueberry_bat *bbbat)
{
	int ret;

	blueberry_fw_assert_mode(NORMAL_MODE);

	mutex_lock(&bbbat->xfer_lock);
	ret = blueberry_bat_smbus_write_byte_no_data(bbbat->client, 0xef);
	if(ret) {
		blueberry_fw_debug("enter flash mode write byte failed.\n");
		mutex_unlock(&bbbat->xfer_lock);
		return ret;
	}
	atomic_set(&bbbat->working_mode, FLASH_MODE);
	mutex_unlock(&bbbat->xfer_lock);

	mdelay(10);

	return ret;
}

static int blueberry_fw_read_flash_status(struct blueberry_bat *bbbat, unsigned char *status)
{
	int ret;

	blueberry_fw_assert_mode(FLASH_MODE);

	ret = blueberry_bat_smbus_read_byte(bbbat->client, CMD_RDSR, status);
	if(ret) {
		blueberry_fw_debug("read flash status failed.\n");
		return ret;
	}

	return ret;
}

/* wait until device is free for an 'write status register', 'program'
	or 'erase' operation. */
static int blueberry_fw_wait_flash_free(struct blueberry_bat *bbbat)
{
	int ret;
	unsigned char status;
	int count = 0;

	blueberry_fw_assert_mode(FLASH_MODE);
	
	while(count < SHORT_RETRY_COUNT) {
		ret = blueberry_fw_read_flash_status(bbbat, &status);
		if(ret) {
			blueberry_fw_debug("wait_flash_free exit because of read_falsh_status error, count=%d\n", count);
			return ret;
		}
		count++;
		if(!(status & RDSR_WIP)) {
			break;
		}
		mdelay(1);
	}

	if(count >= SHORT_RETRY_COUNT ) {
		blueberry_fw_debug("wait_flash_free failed because of timeout.\n");
		return -EBUSY;
	}

	return 0;
}

/* wait until device is free for an 'write status register', 'program'
    or 'erase' operation. after send erase chip command, it will cost up
	to 3 seconds to complete the opertaion, we use a 4 seconds loop to
	wait the operation to complete. */
static int blueberry_fw_wait_flash_free_long_timeout(struct blueberry_bat *bbbat)
{
    int ret;
    unsigned char status;
	int count = LONG_RETRY_COUNT; /* 500 times * 10ms/times = 500 ms = 5 seconds */

    blueberry_fw_assert_mode(FLASH_MODE);

    while(count--) {
        ret = blueberry_fw_read_flash_status(bbbat, &status);
        if(ret) {
            blueberry_fw_debug("wait_flash_free exit because fw_read_flash_status failed.\n");
            return ret;
        }
        if(!(status & RDSR_WIP)) {
            break;
        }
        mdelay(10);
		/* to try a better value */
		/* blueberry_fw_debug("current count=%d\n", count); */
    }

	if(count <= 0) {
		pr_err("blueberry_fw: timeout while fw_wait_falsh_free_long_timeout.\n");
		return -1;
	}

    return 0;
}


static int blueberry_fw_write_flash_enable(struct blueberry_bat *bbbat)
{
	int ret;
	int count = 0;
	unsigned char status;
	
	blueberry_fw_assert_mode(FLASH_MODE);

	ret = blueberry_fw_wait_flash_free(bbbat);
	if(ret) {
		return ret;
	}

	ret = blueberry_bat_smbus_write_byte_no_data(bbbat->client, CMD_WREN);
	if(ret) {
		blueberry_fw_debug("blueberry_bat_smbus_write_byte_no_data for CMD_WREN failed.\n");
		return ret;
	}
	
	while(count < SHORT_RETRY_COUNT) {
		ret = blueberry_fw_read_flash_status(bbbat, &status);
		if(ret) {
			blueberry_fw_debug("write flash enable read status failed.\n");
			return ret;
		}
		count++;
		if((status & RDSR_WEL) == RDSR_WEL)  /* if((status & 0x03) == 0x02) */
			break;

		mdelay(1);
	}

	if(count >= SHORT_RETRY_COUNT) {
		blueberry_fw_debug("write flash enable timeout.\n");
		return -EBUSY;
	}

	return 0;
}

/* before send 'write status register' command, should wait WIP to 0. 
	the WEL bit must be write enabled before any write operation, including sector, block erase,
	page program and write status register operation. The WEL bit will be reset to the write-
	protect state automatically upon completion of a write operation. The WREN instruction is 
	required before any above operation is executed. */
static int blueberry_fw_write_flash_status(struct blueberry_bat *bbbat, unsigned char status)
{
    int ret;

	blueberry_fw_assert_mode(FLASH_MODE);

	ret = blueberry_fw_write_flash_enable(bbbat);
	if(ret) {
		return ret;
	}

    ret = blueberry_bat_smbus_write_byte(bbbat->client, CMD_WRSR, &status);
    if(ret) {
        blueberry_fw_debug("write flash status failed.\n");
        return ret;
    }

    return ret;
}

static int blueberry_fw_erase_flash_all(struct blueberry_bat *bbbat)
{
	int ret;

	blueberry_fw_assert_mode(FLASH_MODE);
	
	ret = blueberry_fw_write_flash_status(bbbat, 0);
	if(ret) {
		blueberry_fw_debug("write flash status failed.\n");
		return ret;
	}
	
	ret = blueberry_fw_wait_flash_free(bbbat);
	if(ret) {
		blueberry_fw_debug("after write falsh status, wait_flash free failed.\n");
		return ret;
	}

	ret = blueberry_fw_write_flash_enable(bbbat);
	if(ret) {
		blueberry_fw_debug("write flash enable before erase chip command failed.\n");
		return ret;
	}

	ret = blueberry_bat_smbus_write_byte_no_data(bbbat->client, CMD_CHIP_ER);
	if(ret) {
		blueberry_fw_debug("erase chip command failed.\n");
		return ret;
	}

	mdelay(100);
	ret = blueberry_fw_wait_flash_free_long_timeout(bbbat);
	if(ret) {
		blueberry_fw_debug("failed to wait flash free after CMD_CHIP_ER.\n");
		return ret;
	}

	return 0;
}

static int blueberry_fw_reset_rom_addr(struct blueberry_bat *bbbat)
{
	int ret;

	blueberry_fw_assert_mode(FLASH_MODE);

	ret = blueberry_bat_smbus_write_byte_no_data(bbbat->client, 0xad);
	if(ret) {
		blueberry_fw_debug("failed to reset rom addr.\n");
		return ret;
	}

	return 0;
}

static int blueberry_fw_write_flash_disable(struct blueberry_bat *bbbat)
{
	int ret;
	int count = 0;
	unsigned char status;

	blueberry_fw_assert_mode(FLASH_MODE);

	ret = blueberry_fw_wait_flash_free(bbbat);
	if(ret) {
		blueberry_fw_debug("failed to wait flash free for write disable.\n");
		return ret;
	}
	
	ret = blueberry_bat_smbus_write_byte_no_data(bbbat->client, CMD_WRDI);
	if(ret) {
		blueberry_fw_debug("failed to write CMD_WRDI.\n");
		return ret;
	}

	while(count < SHORT_RETRY_COUNT) {
		ret = blueberry_fw_read_flash_status(bbbat, &status);
		if(ret) {
			return ret;
		}

		count++;
		if((status & RDSR_WEL) == 0x00)
			break;

		mdelay(1);
	}

	if(count >= SHORT_RETRY_COUNT) {
		blueberry_fw_debug("after write disable, no proper status, timeout.\n");
		return -EBUSY;
	}

	return 0;
}

/* program 32 bytes to flash */
static int blueberry_fw_flash_page_prog_32bytes(struct blueberry_bat *bbbat, unsigned char *buf, int size)
{
	int ret;

	blueberry_fw_assert_mode(FLASH_MODE);
	
	if(size != EC_XFER_LEN)
		return -EINVAL;
	
	ret = blueberry_fw_write_flash_enable(bbbat);
	if(ret) {
		blueberry_fw_debug("failed write enable before page_prog.\n");
		return ret;
	}

	ret = blueberry_bat_smbus_write_byte_block(bbbat->client, CMD_PAGE_PROG, buf, size);
	if(ret) {
		blueberry_fw_debug("wirte_byte_block failed while flash_page_prog.\n");
		return ret;
	}

	return 0;
}

/* read 32 bytes from flash */
static int blueberry_fw_read_code_32bytes(struct blueberry_bat *bbbat, unsigned char *buf, int len)
{
	int ret;

	blueberry_fw_assert_mode(FLASH_MODE);

	if(len != EC_XFER_LEN) {
		blueberry_fw_debug("invalid len for fw_read_code_32bytes.\n");
		return -EINVAL;
	}

	ret = blueberry_bat_smbus_read_byte_block(bbbat->client, CMD_FAST_READ, buf, len);
	if(ret) {
		blueberry_fw_debug("read byte block failed while read code from ec.\n");
		return ret;
	}

	return ret;
}

/* write 256 bytes to flash */
static int blueberry_fw_flash_page_prog(struct blueberry_bat *bbbat, unsigned char *buf, int size)
{
	int ret;
	int i, count;

	blueberry_fw_assert_mode(FLASH_MODE);

	if(size != EC_READ_STEP){
		blueberry_fw_debug("invalide buf size.\n");
		return -EINVAL;
	}

	count = EC_READ_STEP / EC_XFER_LEN;
	
	for(i=0; i<count; i++) {
		ret = blueberry_fw_flash_page_prog_32bytes(bbbat, buf+i*EC_XFER_LEN, EC_XFER_LEN);
		if(ret) 
			return ret;
		ret = blueberry_fw_wait_flash_free(bbbat);
		if(ret)
			return ret;
	}

	return 0;
}

/* read 256 bytes from flash */
static int blueberry_fw_read_code_page(struct blueberry_bat *bbbat, unsigned char *buf, int size)
{
	int ret;
	int i, count;

	blueberry_fw_assert_mode(FLASH_MODE);

	if(size != EC_READ_STEP) {
		blueberry_fw_debug("invalid buf size for read_rom_step.\n");
		return -EINVAL;
	}

	count = EC_READ_STEP / EC_XFER_LEN;	/* 256 bytes: 32 bytes for 8 times */

	for(i=0; i<count; i++) {
		ret = blueberry_fw_read_code_32bytes(bbbat, buf+i*EC_XFER_LEN, EC_XFER_LEN);
		if(ret) {
			blueberry_fw_debug("read code from ec failed.\n");
			return ret;
		}
	}

	return 0;
}

/* write 64K bytes to flash */
static int blueberry_fw_flash_all(struct blueberry_bat *bbbat, unsigned char *buf, int len)
{
	int ret;
	int i;
	int count;

	blueberry_fw_assert_mode(FLASH_MODE);
	
	if(buf == NULL)
		return -EINVAL;

	if(len != 64 * 1024) {
		blueberry_fw_debug("invalid len for fw_flash_all.\n");
		return -EINVAL;
	}
	
	count = len / EC_READ_STEP;

	blueberry_fw_reset_rom_addr(bbbat);

	mdelay(1);

	for(i=0; i<count; i++) {
		ret = blueberry_fw_flash_page_prog(bbbat, buf+i*EC_READ_STEP, EC_READ_STEP);
		if(ret) {
			blueberry_fw_debug("failed to write data.\n");
			return ret;
		}
		udelay(200);
	}

	blueberry_fw_write_flash_disable(bbbat);
	
	return 0;
}

static int blueberry_fw_read_all(struct blueberry_bat *bbbat, unsigned char *buf, int len)
{
	int ret;
	int i;

	int count;

	if(len != 64 * 1024) {
		blueberry_fw_debug("invalid parameter for fw_read_all.\n");
		return -EINVAL;
	}

	blueberry_fw_assert_mode(FLASH_MODE);

	count = len / EC_READ_STEP;	
	ret = blueberry_fw_write_flash_disable(bbbat);
	if(ret) {
		blueberry_fw_debug("fw_read_all write_flash_disable failed.\n");
		return ret;
	}

	ret = blueberry_fw_wait_flash_free(bbbat);
	if(ret) {
		blueberry_fw_debug("wait_flash_free failed.\n");
		return ret;
	}

	ret = blueberry_fw_reset_rom_addr(bbbat);
	if(ret) {
		blueberry_fw_debug("fw_reset_rom_addr failed for read_all.\n");
		return ret;
	}
	
	memset(buf, 0, 64 * 1024);
	for(i=0; i<count; i++) {
		ret = blueberry_fw_read_code_page(bbbat, buf+i*EC_READ_STEP, EC_READ_STEP);
		if(ret) {
			blueberry_fw_debug("read_code_page failed.\n");
			return ret;
		}
	}

	ret = blueberry_fw_wait_flash_free(bbbat);
	if(ret) {
		blueberry_fw_debug("after read all, wait_flash_free failed.\n");
		return ret;
	}

	blueberry_fw_debug("read flash all success.\n");

	return ret;
}
		
int blueberry_fw_read_firmware(struct blueberry_bat *bbbat, unsigned char *buf, int len)
{
	int ret;

	blueberry_fw_assert_mode(NORMAL_MODE);

	ret = blueberry_fw_enter_flash_mode(bbbat);
	if(ret) {
		blueberry_fw_debug("read_firmware enter flash mode failed.\n");
		return ret;
	}

	ret = blueberry_fw_read_all(bbbat, buf, len);
	if(ret){
		blueberry_fw_debug("read all failed.\n");
	}

	ret = blueberry_fw_exit_flash_mode(bbbat);
	if(ret) {
		blueberry_fw_debug("failed to exit flash mode.\n");
		return ret;
	}
	blueberry_fw_debug("blueberry_fw_read_firmware ret=%s\n", ret ? "failed" : "successed");

	return ret;
}

static int blueberry_fw_verify_ff(struct blueberry_bat *bbbat)
{
	int ret;
	unsigned char *fw;
	int i;
	int size = 64 * 1024;

	fw = vmalloc(size);
	if(fw == NULL) {
		blueberry_fw_debug("blueberry_fw_verify_ff failed.\n");
		return -ENOMEM;
	}

	ret = blueberry_fw_read_all(bbbat, fw, size);
	if(ret) {
		blueberry_fw_debug("bluebeery_fw_read_all failed.\n");
		return ret;
	}

	for(i=0; i<size; i++) {
		if(*(fw+i) != 0xff) {
			blueberry_fw_debug("fw_verify_ff failed.\n");
			break;
		}else{
			continue;
		}
	}
	vfree(fw);

	if(i >= size) {
		blueberry_fw_debug("verify_ff succ, erase done.\n");
		return 0;
	}else{
		blueberry_fw_debug("verify_ff failed, erase fail.\n");
		return -1;
	}	
}

static int blueberry_fw_verify(struct blueberry_bat *bbbat, unsigned char *src, int len)
{
	int ret;
	int size = 64 * 1024;
	unsigned char *fw;
	int i;

	if(len != size){
		return -EINVAL;
	}

	fw = vmalloc(size);
	if(fw == NULL) {
		blueberry_fw_debug("fw_verify failed because of no memory.\n");
		return -ENOMEM;
	}

    ret = blueberry_fw_read_all(bbbat, fw, size);
    if(ret) {
        blueberry_fw_debug("bluebeery_fw_read_all failed.\n");
        return ret;
    }

    for(i=0; i<size; i++) {
        if(*(fw+i) != *(src+i)) {
            blueberry_fw_debug("fw_verify failed.\n");
            break;
        }else{
            continue;
        }
    }
    vfree(fw);

    if(i >= size) {
        blueberry_fw_debug("verify succ, flash the chip done.\n");
        return 0;
    }else{
        blueberry_fw_debug("verify failed, flash the chip fail.\n");
        return -1;
    }
}	
	
static int blueberry_fw_update(struct blueberry_bat *bbbat, unsigned char *fw, int len)
{
	int ret;
	int retry;

	blueberry_fw_assert_mode(NORMAL_MODE);

	ret = blueberry_fw_enter_flash_mode(bbbat);
	if(ret) {
		blueberry_fw_debug("can't enter flash mode.\n");
		return ret;
	}
	
	retry = 3;
	while(retry--) {
		ret = blueberry_fw_erase_flash_all(bbbat);
		if(ret) {
			blueberry_fw_debug("can't do erase all.\n");
			continue;
		}
		ret = blueberry_fw_verify_ff(bbbat);
		if(ret) {
			blueberry_fw_debug("erase all succ but can't verify all 0xff.\n");
			continue;
		}
		ret = blueberry_fw_flash_all(bbbat, fw, len);
		if(ret) {
			blueberry_fw_debug("flash all failed, redo it.\n");
			continue;
		}
		ret = blueberry_fw_verify(bbbat, fw, len);
		if(ret) {
			blueberry_fw_debug("verify failed after write.\n");
			continue;
		}else{
			blueberry_fw_debug("update fw success now.\n");
			break;
		}
	}
	if(ret) {
		blueberry_fw_debug("failed to update firmware.\n");
	}

	ret = blueberry_fw_exit_flash_mode(bbbat);
	if(ret) {
		blueberry_fw_debug("failed to exit flash mode.\n");
	}

	return ret;
}

#if defined(CONFIG_BLUEBERRY_FW_KERNEL_UPGRADE)
extern unsigned char ecfw[];
int blueberry_fw_kernel_upgrade(struct blueberry_bat *bbbat)
{
	int ret = 0;
	int ecfw_ver;
	int curr_ver;
	
	ecfw_ver = (ecfw[0xe800] << 8) | ecfw[0xe801];
	
	ret = blueberry_fw_get_version(bbbat, &curr_ver);
	if(ret) {
		pr_err("blueberry_fw: error while read currently running ec firmware version.\n");
		return -ENODEV;
	}

	pr_info("blueberry_fw: running ec ver: 0x%x, kernel packaged ec ver: 0x%x\n", curr_ver, ecfw_ver);
	/*
	if(curr_ver > ecfw_ver) {
		pr_info("blueberry_fw: running latest ec firmware, do not do upgrade.\n");
		return 0;
	}
	*/

	ret = blueberry_fw_update(bbbat, ecfw, 64 * 1024);
	if(ret) {
		pr_err("blueberry_fw: fatal error, upgrade ec firmware in kernel driver failed.\n");
	}
	pr_info("blueberry_fw: kernel update blueberry ec firmware success.\n");

	return ret;
}
#endif

static int blueberry_fw_open(struct inode *inode, struct file *file)
{
	struct blueberry_bat *bbbat;
	
	bbbat = (struct blueberry_bat *)container_of(inode->i_cdev, struct blueberry_bat, fw_cdev);

	nonseekable_open(inode, file);
	file->private_data = bbbat;
	
	blueberry_fw_debug("blueberry_fw_open.\n");
	return 0;
}

static int blueberry_fw_release(struct inode *inode, struct file *file)
{
	struct blueberry_bat *bbbat;
	
	bbbat = file->private_data;
	file->private_data = NULL;
	
	blueberry_fw_debug("blueberry_fw_release.\n");
	return 0;
}

static ssize_t blueberry_fw_read(struct file *file, char __user *buf,
						size_t count, loff_t *offset)
{
	size_t size = 64 * 1024;
	int ret;
	struct blueberry_bat *bbbat;
	unsigned char *fw = NULL;

	mutex_lock(&bbbat->fw_rwlock);
	
	bbbat = file->private_data;

	blueberry_fw_debug("blueberry_fw_read enter. count=%d.\n", count);

	if(count < size) {
		blueberry_fw_debug("count < 64K, invalide parameter.\n");
		ret = -EINVAL;
		goto err_release_lock;
	}

	memset(buf, 0, count);

	fw = vmalloc(size);
	if(!fw) {
		blueberry_fw_debug("can't alloc fw buffer.\n");
		ret = -ENOMEM;
		goto err_release_lock;
	}
	
	ret = blueberry_fw_read_firmware(bbbat, fw, size);
	if(ret) {
		goto err_release_lock;
	}
	
	if(copy_to_user(buf, fw, size)) {
		blueberry_fw_debug("copy firmware to user failed.\n");
		ret = -EIO;
		goto err_release_lock;
	}
	blueberry_fw_debug("read firmware succ.\n");
	ret = size;									/* when succ, return size, when failed , return a value < 0 */	

err_release_lock:
	if(fw) vfree(fw);
	mutex_unlock(&bbbat->fw_rwlock);

	return ret;
}

static ssize_t blueberry_fw_write(struct file *file, const char __user *buf,
						size_t count, loff_t *offset)
{
	int ret = 0;
	int size = 64 * 1024;
	struct blueberry_bat *bbbat;
	unsigned char *fw = NULL;

	bbbat = file->private_data;

	if(bbbat->fw_protect != 0 ) {
		blueberry_fw_debug("flash is at protect status, please cetificate and re-do.\n");
		return -EIO;
	}

	mutex_lock(&bbbat->fw_rwlock);

	if(count != size) {
		blueberry_fw_debug("count=%d, not 64K, return.\n",count);
		ret = -EINVAL;
		goto err_release_lock;
	}
	fw = vmalloc(size);
	if(!fw) {
		blueberry_fw_debug("alloc fw buffer for write failed.\n");
		ret = -ENOMEM;
		goto err_release_lock;
	}

	ret = copy_from_user(fw, buf, size);

	ret = blueberry_fw_update(bbbat, fw, size);
	if(ret) {
		blueberry_fw_debug("failed to update firmware.\n");
		ret = -EIO;
		goto err_release_lock;
	}
	blueberry_fw_debug("update firmware succ.\n");
	ret = size;

err_release_lock:
	if(fw) vfree(fw);
	mutex_unlock(&bbbat->fw_rwlock);
	bbbat->fw_protect = 1;

	return ret;
}

static long blueberry_fw_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret;
	unsigned int version;
	struct blueberry_bat *bbbat;
	unsigned char magic[128];
	unsigned short ver;
	unsigned int ecfw_ver;

	void __user *argp = (void __user *) arg;
	bbbat = file->private_data;

	blueberry_fw_assert_mode(NORMAL_MODE);
	switch(cmd) {

		case CMD_GET_VERSION:
		{
			ret = blueberry_fw_get_version(bbbat, &version);
			if(ret) {
				return ret;
			}
			ver = (unsigned short)version;
			ret = copy_to_user(argp, &ver, sizeof(unsigned short));
			break;
		}

#if defined(CONFIG_BLUEBERRY_FW_KERNEL_UPGRADE)
		case CMD_GET_ECFW_VERSION:
		{
			ecfw_ver = (ecfw[0xe800] << 8) | ecfw[0xe801];
			ver = (unsigned short)ecfw_ver;
			ret = copy_to_user(argp, &ver, sizeof(unsigned short));
			break;
		}
#endif

		case CMD_UN_PROTECT:
		{
			memset(magic, 0, 128);
			ret = copy_from_user(magic, argp, strlen("bitland.ec.unprotect"));
			if(!strcmp(magic, "bitland.ec.unprotect")) {
				bbbat->fw_protect = 0;
				ret = 0;
			}else
				ret = -EINVAL;		

			break;	
		}
		
#if defined(CONFIG_BLUEBERRY_FW_KERNEL_UPGRADE)
		case CMD_FW_UPGRADE:
		{
    		ret = blueberry_fw_kernel_upgrade(bbbat);
    		if(ret) {
        		blueberry_fw_debug("failed to upgrade ec fw in kernel.\n");
    		}
			break;
		}
#endif

		default:
			ret = -ENOTTY;
			blueberry_fw_debug("unknown command.\n");
			break;
	}

	return ret;
}

static struct file_operations blueberry_fops = {
	.owner			= THIS_MODULE,
	.open 			= blueberry_fw_open,
	.release		= blueberry_fw_release,
	.read 			= blueberry_fw_read,
	.write			= blueberry_fw_write,
	.unlocked_ioctl = blueberry_fw_ioctl,
};	
	
static struct class *blueberry_fw_class;

int blueberry_fw_driver_init(struct blueberry_bat *bbbat)
{
	int ret;
	dev_t dev;

	ret = alloc_chrdev_region(&dev, 0, 1, "ite8561");
	if(ret < 0) {
		blueberry_fw_debug("can't allocate major number\n");
		return -ENODEV;
	}
	blueberry_fw_class = class_create(THIS_MODULE, "ite8561");
	bbbat->fw_dev = dev;

	cdev_init(&bbbat->fw_cdev, &blueberry_fops);
	if(cdev_add(&bbbat->fw_cdev, dev, 1)) {
		blueberry_fw_debug("cdev_add failed.\n");
		unregister_chrdev_region(dev, 1);
		return -ENODEV;
	}	

	device_create(blueberry_fw_class, NULL, dev, NULL, "ite8561");
	blueberry_fw_debug("blueberry_fw_driver_init success.\n");

	mutex_init(&bbbat->fw_rwlock);
	bbbat->fw_protect = 1;
	
	return 0;
}


int blueberry_fw_driver_uninit(struct blueberry_bat *bbbat)
{

	mutex_destroy(&bbbat->fw_rwlock);
	class_destroy(blueberry_fw_class);
	cdev_del(&bbbat->fw_cdev);
	unregister_chrdev_region(bbbat->fw_dev, 1);
	
	return 0;
}
