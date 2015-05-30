/*
 * BlueBerry tp i2c/smbus operation functions.
 *
 * Copyright (C) 2013-2013 Paul Ma <magf@bitland.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#ifndef __BLUEBERRY_TP_I2C_H
#define __BLUEBERRY_TP_I2C_H

int blueberry_tp_smbus_read_byte(struct i2c_client *client,
                        unsigned char reg_addr, unsigned char *data);
int blueberry_tp_smbus_write_byte(struct i2c_client *client,
                        unsigned char reg_addr, unsigned char *data);
int blueberry_tp_smbus_write_byte_no_data(struct i2c_client *client,
                        unsigned char reg_addr);

int blueberry_tp_smbus_read_word_data(struct i2c_client *client,
    unsigned char reg_addr, unsigned char *data, unsigned char length);
int blueberry_tp_smbus_write_word_data(struct i2c_client *client,
    unsigned char reg_addr, unsigned char *data, unsigned char length);

int blueberry_tp_smbus_read_byte_block(struct i2c_client *client,
                unsigned char reg_addr, unsigned char *data, unsigned char len);
int blueberry_tp_smbus_write_byte_block(struct i2c_client *client,
                unsigned char reg_addr, unsigned char *data, unsigned char len);

int blueberry_tp_i2c_read_data(struct i2c_client *client, char *buf, int length);
int blueberry_tp_i2c_write_data(struct i2c_client *client, char *buf, int length);

#endif
