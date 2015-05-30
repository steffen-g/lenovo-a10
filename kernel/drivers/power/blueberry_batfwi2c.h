/*
 * BlueBerry battery & firmware i2c/smbus operation functions.
 *
 * Copyright (C) 2013-2013 Paul Ma <magf@bitland.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#ifndef __BLUEBERRY_BATFWI2C_H
#define __BLUEBERRY_BATFWI2C_H

#define FLASH_MODE  0
#define NORMAL_MODE 1

#define FLASH_MODE_ADDR 0x0d
#define NORMAL_MODE_ADDR 0x62

#define CONNECT_USB	0
#define CONNECT_ADAPTER 1
#define CONNECT_NONE 2

#define CONNECT_USB_COMMAND	0xf0
#define CONNECT_ADAPTER_COMMAND 0xf1

struct blueberry_bat_reg_cache {
    int temperature;
    int current_now;
    int status;
    int voltage;
    int energy;
    int health;
    int present;
    int capacity;
    int tte;
    int ttecp;
    int ttf;
    int nac;
    int lmd;
    int ilmd;
    int cyct;
};

struct blueberry_bat {
    struct device *dev;
    struct i2c_client *client;
    int id;

    struct power_supply bat;
    struct power_supply ac;

	/* protect get_property not do i2c operation at the same time */
	struct mutex xfer_lock;

    unsigned int interval;
    struct delayed_work work;
    struct work_struct dcwakeup_work;
    struct work_struct lowpower_work;

    unsigned int dc_det_pin;
    unsigned int bat_det_adc_channel;
    unsigned int bat_low_det_pin;

    int power_down;

    /* when not zero, output debug info */
    unsigned int debug;

    /* firmware update */
    atomic_t working_mode;
    unsigned int fw_debug;
    struct cdev fw_cdev;
    struct mutex fw_rwlock;
    int fw_protect;
    dev_t   fw_dev;
    struct blueberry_bat_reg_cache cache;

	/* for adc input to judge bat in/out */
	struct adc_client *adclient;
	int adcval;

#ifdef CONFIG_CHARGER_INPUT_SELECT
	/* for charger adapter/usb input select */
	struct workqueue_struct *usb_ac_detect_workqueue;
	unsigned int usb_ac_detect_interval;
	struct delayed_work usb_ac_detect_work;
	atomic_t connect;
#endif
};

int blueberry_bat_smbus_read_byte(struct i2c_client *client,
                        unsigned char reg_addr, unsigned char *data);
int blueberry_bat_smbus_write_byte(struct i2c_client *client,
                        unsigned char reg_addr, unsigned char *data);
int blueberry_bat_smbus_write_byte_no_data(struct i2c_client *client,
                        unsigned char reg_addr);

int blueberry_bat_smbus_read_word_data(struct i2c_client *client,
    unsigned char reg_addr, unsigned char *data, unsigned char length);
int blueberry_bat_smbus_write_word_data(struct i2c_client *client,
    unsigned char reg_addr, unsigned char *data, unsigned char length);

int blueberry_bat_smbus_read_byte_block(struct i2c_client *client,
                unsigned char reg_addr, unsigned char *data, unsigned char len);
int blueberry_bat_smbus_write_byte_block(struct i2c_client *client,
                unsigned char reg_addr, unsigned char *data, unsigned char len);

int blueberry_bat_i2c_read_data(struct i2c_client *client, char *buf, int length);
int blueberry_bat_i2c_write_data(struct i2c_client *client, char *buf, int length);

#endif
