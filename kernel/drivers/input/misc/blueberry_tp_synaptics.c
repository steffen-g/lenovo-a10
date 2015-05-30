/*
 * Derived from Synaptics TouchPad PS/2 mouse driver
 * 
 * Blueberry Synaptics Touch pad driver.
 * Copyright by Bitland 2013-2013 Paul Ma <magf@bitland.com.cn>.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
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
#include <asm/atomic.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "blueberry_tp_i2c.h"
#include "blueberry_tp_synaptics.h"

/*
 * The x/y limits are taken from the Synaptics TouchPad interfacing Guide,
 * section 2.3.2, which says that they should be valid regardless of the
 * actual size of the sensor.
 * Note that newer firmware allows querying device for maximum useable
 * coordinates.
 */
#define XMIN 0
#define XMAX 6143
#define YMIN 0
#define YMAX 6143
#define XMIN_NOMINAL 1472
#define XMAX_NOMINAL 5472
#define YMIN_NOMINAL 1408
#define YMAX_NOMINAL 4448

/* Size in bits of absolute position values reported by the hardware */
#define ABS_POS_BITS 13

/*
 * These values should represent the absolute maximum value that will
 * be reported for a positive position value. Some Synaptics firmware
 * uses this value to indicate a finger near the edge of the touchpad
 * whose precise position cannot be determined.
 *
 * At least one touchpad is known to report positions in excess of this
 * value which are actually negative values truncated to the 13-bit
 * reporting range. These values have never been observed to be lower
 * than 8184 (i.e. -8), so we treat all values greater than 8176 as
 * negative and any other value as positive.
 */
#define X_MAX_POSITIVE 8176
#define Y_MAX_POSITIVE 8176

#define blueberry_tp_synaptics_debug(fmt, ...)              \
    do {                                                    \
        if (synaptics->debug)                               \
            pr_info(pr_fmt(fmt), ##__VA_ARGS__);            \
    } while (0)

/* some function use 'struct synaptics_data *priv' */
#define blueberry_tp_priv_debug(fmt, ...) 		            \
    do {                                                    \
        if (priv->debug)                                    \
            pr_info(pr_fmt(fmt), ##__VA_ARGS__);            \
    } while (0)

/*****************************************************************************
 *	Stuff we need even when we do not want native Synaptics support
 ****************************************************************************/

static int blueberry_tp_synaptics_mode_cmd(struct synaptics_data *synaptics, unsigned char mode)
{
	int ret = -1;
	unsigned char cmd[3] = {0x0f, 0x00, 0x00};

	cmd[2] = mode;
	ret = blueberry_tp_i2c_write_data(synaptics->client, cmd, 3);
	if(ret) {
		pr_err("blueberry_tp_synaptics_mode_cmd failed.\n");
		return ret;
	}
	/* wait 50ms to let EC write mode to touchpad. */
	mdelay(100);
	blueberry_tp_synaptics_debug("blueberry_tp_synaptics_mode_cmd 0x%x set succ.\n", mode);

	return 0;
}

/*****************************************************************************
 *	Synaptics communications functions
 ****************************************************************************/

/*
 * Synaptics touchpads report the y coordinate from bottom to top, which is
 * opposite from what userspace expects.
 * This function is used to invert y before reporting.
 */
static int blueberry_tp_synaptics_invert_y(int y)
{
	return YMAX_NOMINAL + YMIN_NOMINAL - y;
}

/*
 * Read the model-id bytes from the touchpad
 * see also SYN_MODEL_* macros
 */
static int blueberry_tp_synaptics_model_id(struct synaptics_data *synaptics)
{
	synaptics->model_id = (0x1<<16) | (0xe2<<8) | (0xb1);			/* 01, e2, b1 */
	return 0;
}

/*
 * Read the board id from the touchpad
 * The board id is encoded in the "QUERY MODES" response
 */
static int blueberry_tp_synaptics_board_id(struct synaptics_data *synaptics)
{
	unsigned char bid[3] = {0x2c, 0x55, 0x40};						/* 2c, 55, 40 */
	synaptics->board_id = ((bid[0] & 0xfc) << 6) | bid[1];
	return 0;
}

/*
 * Read the firmware id from the touchpad
 */
static int blueberry_tp_synaptics_firmware_id(struct synaptics_data *synaptics)
{
	unsigned char fwid[3] = {0x16, 0xaa, 0x2b};						/* 16, aa, 2b */
	synaptics->firmware_id = (fwid[0] << 16) | (fwid[1] << 8) |fwid[2];
	return 0;
}

/*
 * Read the capability-bits from the touchpad
 * see also the SYN_CAP_* macros
 */
static int blueberry_tp_synaptics_capability(struct synaptics_data *synaptics)
{
	synaptics->capabilities = (0xd0 << 16) | (0x01 << 8) | (0x23);	/* d0, 01, 23 */
	synaptics->ext_cap = (0x84 << 16) | (0x03 << 8) | (0x00);		/* 84, 03, 00 */
	/*
	 * if nExtBtn is greater than 8 it should be considered
	 * invalid and treated as 0
	 */
	if (SYN_CAP_MULTI_BUTTON_NO(synaptics->ext_cap) > 8)
		synaptics->ext_cap &= 0xff0fff;

	synaptics->ext_cap_0c = (0x12 << 16) | (0x6c << 8) | (0x00);	/* 12, 6c, 00 */

	return 0;
}

/*
 * Identify Touchpad
 * See also the SYN_ID_* macros
 */
static int blueberry_tp_synaptics_identify(struct synaptics_data *synaptics)
{
	synaptics->identity = (0x01 << 16) | (0x47 << 8) | (0x18);		/* 01, 47, 18 */
	if (SYN_ID_IS_SYNAPTICS(synaptics->identity))
		return 0;
	return -1;
}

/*
 * Read touchpad resolution and maximum reported coordinates
 * Resolution is left zero if touchpad does not support the query
 */
static int blueberry_tp_synaptics_resolution(struct synaptics_data *synaptics)
{
	unsigned char resp[3];

	if (SYN_ID_MAJOR(synaptics->identity) < 4)
		return 0;

	synaptics->x_res = 0x53;				/* command return 53, 80, 9b */
	synaptics->y_res = 0x9b;	

	if (SYN_EXT_CAP_REQUESTS(synaptics->capabilities) >= 5 &&
	    SYN_CAP_MAX_DIMENSIONS(synaptics->ext_cap_0c)) {
			resp[0] = 0xb6;					/* command return b6, dc, a0 */
			resp[1] = 0xdc;
			resp[2] = 0xa0;
			synaptics->x_max = (resp[0] << 5) | ((resp[1] & 0x0f) << 1);
			synaptics->y_max = (resp[2] << 5) | ((resp[1] & 0xf0) >> 3);
	}

	if (SYN_EXT_CAP_REQUESTS(synaptics->capabilities) >= 7 &&
	    SYN_CAP_MIN_DIMENSIONS(synaptics->ext_cap_0c)) {
			blueberry_tp_synaptics_debug("this can't be executed.\n");
			resp[0] = 0x22;
			resp[1] = 0x12;
			resp[2] = 0x16;
			synaptics->x_min = (resp[0] << 5) | ((resp[1] & 0x0f) << 1);
			synaptics->y_min = (resp[2] << 5) | ((resp[1] & 0xf0) >> 3);
	}else if(SYN_CAP_MIN_DIMENSIONS(synaptics->ext_cap_0c)) {
			// may be the following datas are wrong because SYN_EXT_CAP_REQUEST(synaptics->capabilities) is 0x101.
			// from interfacing guide, only if SYN_CAP_MIN_DIMENSIONS set, then this can be read.
            resp[0] = 0x22;
            resp[1] = 0x12;
            resp[2] = 0x16;
            synaptics->x_min = (resp[0] << 5) | ((resp[1] & 0x0f) << 1);
            synaptics->y_min = (resp[2] << 5) | ((resp[1] & 0xf0) >> 3);			
			blueberry_tp_synaptics_debug("synaptica->x_min=%d and synaptics->y_min=%d maybe wrong. Pay attension here if not work\n",  \
															synaptics->x_min, synaptics->y_min);
	} 

	return 0;
}

static int blueberry_tp_synaptics_query_hardware(struct synaptics_data *synaptics)
{
	if (blueberry_tp_synaptics_identify(synaptics))
		return -1;
	if (blueberry_tp_synaptics_model_id(synaptics))
		return -1;
	if (blueberry_tp_synaptics_firmware_id(synaptics))
		return -1;
	if (blueberry_tp_synaptics_board_id(synaptics))
		return -1;
	if (blueberry_tp_synaptics_capability(synaptics))
		return -1;
	if (blueberry_tp_synaptics_resolution(synaptics))
		return -1;

	return 0;
}

#ifdef USE_SYNAPTICS_SET_RATE
static int blueberry_tp_synaptics_set_rate(struct synaptics_data *synaptics, unsigned char rate)
{
    int ret = -1;
    unsigned char cmd[3] = {0x10, 0x00, 0x00};

    cmd[2] = rate;
    ret = blueberry_tp_i2c_write_data(synaptics->client, cmd, 3);
    if(ret) {
        pr_err("blueberry_tp_synaptics_set_rate failed.\n");
        return ret;
    }
    blueberry_tp_synaptics_debug("blueberry_tp_synaptics_set_rate 0x%x set succ.\n", rate);

    return 0;
}
#endif

static int blueberry_tp_synaptics_enter_advanced_mode(struct synaptics_data * synaptics)
{
	int ret = -1;
	unsigned char cmd[3] = {0x11, 0x00, 0xc8};
	ret = blueberry_tp_i2c_write_data(synaptics->client, cmd, 3);
	if(ret) {
		pr_err("blueberry_tp_synaptics_enter_andvanced_mode failed.\n");
		return ret;
	}
	blueberry_tp_synaptics_debug("blueberry_tp_synaptics_enter_advanced_mode succ.\n");

	/* after this function executed, should delay 50 ms to let EC sendout complete command string */

	return 0;
}

static int blueberry_tp_synaptics_sendout_query_cmd(struct synaptics_data *synaptics, 
	unsigned char query, unsigned char *prompt)
{
	int ret = -1;
	unsigned char cmd[3] = {0x12, 0x00, 0x00};
	cmd[2] = query;

	ret = blueberry_tp_i2c_write_data(synaptics->client, cmd, 3);
	if(ret) {
		pr_err("blueberry_tp_synaptics_execute_query(%s) failed.\n", prompt);
		return ret;
	}
	blueberry_tp_synaptics_debug("blueberry_tp_synaptics_execute_query(%s) send command(0x%x) succ.\n", \
																prompt, query);
	
	return 0;
}

static int blueberry_tp_synaptics_get_query_data(struct synaptics_data *synaptics, 
	unsigned char *data, unsigned char *prompt)
{
	int ret = -1;
	
	ret = blueberry_tp_smbus_read_byte_block(synaptics->client, 0x13, data, 3);
	if(ret) {
		pr_err("blueberry_tp_synaptics_get_query_data(%s) failed.\n", prompt);
		return ret;
	}
	blueberry_tp_synaptics_debug("blueberry_tp_synaptics_get_query_data(%s) = 0x%x, 0x%x, 0x%x\n", \
																prompt, data[0], data[1], data[2]);

	return 0;
}

static int blueberry_tp_synaptics_execute_query(struct synaptics_data *synaptics, 
	unsigned char query, unsigned char *data, unsigned char *prompt)
{
	int ret = -1;
	
	ret = blueberry_tp_synaptics_sendout_query_cmd(synaptics, query, prompt);
	if(ret) {
		return -1;
	}
	mdelay(50);
	ret = blueberry_tp_synaptics_get_query_data(synaptics, data, prompt);
	if(ret) {
		return -1;
	}

	return 0;
}

static int blueberry_tp_synaptics_set_advanced_gesture_mode(struct synaptics_data *synaptics)
{
	if (!(SYN_CAP_ADV_GESTURE(synaptics->ext_cap_0c) ||
	      SYN_CAP_IMAGE_SENSOR(synaptics->ext_cap_0c))){
		pr_err("not ADV_GESTURE or IMAGE_SENSOR. return 0 directly.\n");
		return 0;
	}

	if(blueberry_tp_synaptics_enter_advanced_mode(synaptics)) {
		return -1;
	}
	mdelay(50);

	blueberry_tp_synaptics_debug("enter advanced gesture mode.\n");
	/* Advanced gesture mode also sends multi finger data */
	synaptics->capabilities |= BIT(1);

	return 0;
}

static int blueberry_tp_synaptics_set_mode(struct synaptics_data *synaptics)
{
	synaptics->mode = 0;
	if (synaptics->absolute_mode)
		synaptics->mode |= SYN_BIT_ABSOLUTE_MODE;
	if (synaptics->disable_gesture)
		synaptics->mode |= SYN_BIT_DISABLE_GESTURE;
	if (synaptics->rate >= 80)							/* need define a rate variable in synaptics_data */
		synaptics->mode |= SYN_BIT_HIGH_RATE;
	if (SYN_CAP_EXTENDED(synaptics->capabilities))
		synaptics->mode |= SYN_BIT_W_MODE;

	if (blueberry_tp_synaptics_mode_cmd(synaptics, synaptics->mode)) {
		pr_err("set synaptics mode(0x%x) failed.\n", synaptics->mode);
		return -1;
	}

	if (synaptics->absolute_mode &&
	    blueberry_tp_synaptics_set_advanced_gesture_mode(synaptics)) {
		pr_err("Advanced gesture mode init failed.\n");
		return -1;
	}

	return 0;
}

/*****************************************************************************
 *	Functions to interpret the absolute mode packets
 ****************************************************************************/

static void blueberry_tp_synaptics_mt_state_set(struct synaptics_mt_state *state, int count,
				   int sgm, int agm)
{
	state->count = count;
	state->sgm = sgm;
	state->agm = agm;
}

static void blueberry_tp_synaptics_parse_agm(const unsigned char buf[],
				struct synaptics_data *priv,
				struct synaptics_hw_state *hw)
{
	struct synaptics_hw_state *agm = &priv->agm;
	int agm_packet_type;

	agm_packet_type = (buf[5] & 0x30) >> 4;
	switch (agm_packet_type) {
	case 1:
		/* Gesture packet: (x, y, z) half resolution */
		agm->w = hw->w;
		agm->x = (((buf[4] & 0x0f) << 8) | buf[1]) << 1;
		agm->y = (((buf[4] & 0xf0) << 4) | buf[2]) << 1;
		agm->z = ((buf[3] & 0x30) | (buf[5] & 0x0f)) << 1;
		break;

	case 2:
		/* AGM-CONTACT packet: (count, sgm, agm) */
		blueberry_tp_synaptics_mt_state_set(&agm->mt_state, buf[1], buf[2], buf[4]);
		break;

	default:
		break;
	}

	/* Record that at least one AGM has been received since last SGM */
	priv->agm_pending = true;
}

static int blueberry_tp_synaptics_parse_hw_state(const unsigned char buf[],
				    struct synaptics_data *priv,
				    struct synaptics_hw_state *hw)
{
	memset(hw, 0, sizeof(struct synaptics_hw_state));

	if (SYN_MODEL_NEWABS(priv->model_id)) {
		hw->w = (((buf[0] & 0x30) >> 2) |
			 ((buf[0] & 0x04) >> 1) |
			 ((buf[3] & 0x04) >> 2));

		hw->left  = (buf[0] & 0x01) ? 1 : 0;
		hw->right = (buf[0] & 0x02) ? 1 : 0;

		if (SYN_CAP_CLICKPAD(priv->ext_cap_0c)) {
			/*
			 * Clickpad's button is transmitted as middle button,
			 * however, since it is primary button, we will report
			 * it as BTN_LEFT.
			 */
			hw->left = ((buf[0] ^ buf[3]) & 0x01) ? 1 : 0;

		} else if (SYN_CAP_MIDDLE_BUTTON(priv->capabilities)) {
			hw->middle = ((buf[0] ^ buf[3]) & 0x01) ? 1 : 0;
			if (hw->w == 2)
				hw->scroll = (signed char)(buf[1]);
		}

		if (SYN_CAP_FOUR_BUTTON(priv->capabilities)) {
			hw->up   = ((buf[0] ^ buf[3]) & 0x01) ? 1 : 0;
			hw->down = ((buf[0] ^ buf[3]) & 0x02) ? 1 : 0;
		}

		if ((SYN_CAP_ADV_GESTURE(priv->ext_cap_0c) ||
			SYN_CAP_IMAGE_SENSOR(priv->ext_cap_0c)) &&
		    hw->w == 2) {
			blueberry_tp_synaptics_parse_agm(buf, priv, hw);
			return 1;
		}

		hw->x = (((buf[3] & 0x10) << 8) |
			 ((buf[1] & 0x0f) << 8) |
			 buf[4]);
		hw->y = (((buf[3] & 0x20) << 7) |
			 ((buf[1] & 0xf0) << 4) |
			 buf[5]);
		hw->z = buf[2];

		if (SYN_CAP_MULTI_BUTTON_NO(priv->ext_cap) &&
		    ((buf[0] ^ buf[3]) & 0x02)) {
			switch (SYN_CAP_MULTI_BUTTON_NO(priv->ext_cap) & ~0x01) {
			default:
				/*
				 * if nExtBtn is greater than 8 it should be
				 * considered invalid and treated as 0
				 */
				break;
			case 8:
				hw->ext_buttons |= ((buf[5] & 0x08)) ? 0x80 : 0;
				hw->ext_buttons |= ((buf[4] & 0x08)) ? 0x40 : 0;
			case 6:
				hw->ext_buttons |= ((buf[5] & 0x04)) ? 0x20 : 0;
				hw->ext_buttons |= ((buf[4] & 0x04)) ? 0x10 : 0;
			case 4:
				hw->ext_buttons |= ((buf[5] & 0x02)) ? 0x08 : 0;
				hw->ext_buttons |= ((buf[4] & 0x02)) ? 0x04 : 0;
			case 2:
				hw->ext_buttons |= ((buf[5] & 0x01)) ? 0x02 : 0;
				hw->ext_buttons |= ((buf[4] & 0x01)) ? 0x01 : 0;
			}
		}
	} else {
		hw->x = (((buf[1] & 0x1f) << 8) | buf[2]);
		hw->y = (((buf[4] & 0x1f) << 8) | buf[5]);

		hw->z = (((buf[0] & 0x30) << 2) | (buf[3] & 0x3F));
		hw->w = (((buf[1] & 0x80) >> 4) | ((buf[0] & 0x04) >> 1));

		hw->left  = (buf[0] & 0x01) ? 1 : 0;
		hw->right = (buf[0] & 0x02) ? 1 : 0;
	}

	/*
	 * Convert wrap-around values to negative. (X|Y)_MAX_POSITIVE
	 * is used by some firmware to indicate a finger at the edge of
	 * the touchpad whose precise position cannot be determined, so
	 * convert these values to the maximum axis value.
	 */
	if (hw->x > X_MAX_POSITIVE)
		hw->x -= 1 << ABS_POS_BITS;
	else if (hw->x == X_MAX_POSITIVE)
		hw->x = XMAX;

	if (hw->y > Y_MAX_POSITIVE)
		hw->y -= 1 << ABS_POS_BITS;
	else if (hw->y == Y_MAX_POSITIVE)
		hw->y = YMAX;

	return 0;
}

static void blueberry_tp_synaptics_report_semi_mt_slot(struct input_dev *dev, int slot,
					  bool active, int x, int y)
{
	input_mt_slot(dev, slot);
	input_mt_report_slot_state(dev, MT_TOOL_FINGER, active);
	if (active) {
		input_report_abs(dev, ABS_MT_POSITION_X, x);
		input_report_abs(dev, ABS_MT_POSITION_Y, blueberry_tp_synaptics_invert_y(y));
	}
}

static void blueberry_tp_synaptics_report_semi_mt_data(struct input_dev *dev,
					  const struct synaptics_hw_state *a,
					  const struct synaptics_hw_state *b,
					  int num_fingers)
{
	if (num_fingers >= 2) {
		blueberry_tp_synaptics_report_semi_mt_slot(dev, 0, true, min(a->x, b->x),
					      min(a->y, b->y));
		blueberry_tp_synaptics_report_semi_mt_slot(dev, 1, true, max(a->x, b->x),
					      max(a->y, b->y));
	} else if (num_fingers == 1) {
		blueberry_tp_synaptics_report_semi_mt_slot(dev, 0, true, a->x, a->y);
		blueberry_tp_synaptics_report_semi_mt_slot(dev, 1, false, 0, 0);
	} else {
		blueberry_tp_synaptics_report_semi_mt_slot(dev, 0, false, 0, 0);
		blueberry_tp_synaptics_report_semi_mt_slot(dev, 1, false, 0, 0);
	}
}

static void blueberry_tp_synaptics_report_buttons(struct synaptics_data *priv,
				     const struct synaptics_hw_state *hw)
{
	struct input_dev *dev = priv->abs_dev;
	int i;

	input_report_key(dev, BTN_LEFT, hw->left);
	input_report_key(dev, BTN_RIGHT, hw->right);

	if (SYN_CAP_MIDDLE_BUTTON(priv->capabilities))
		input_report_key(dev, BTN_MIDDLE, hw->middle);

	if (SYN_CAP_FOUR_BUTTON(priv->capabilities)) {
		input_report_key(dev, BTN_FORWARD, hw->up);
		input_report_key(dev, BTN_BACK, hw->down);
	}

	for (i = 0; i < SYN_CAP_MULTI_BUTTON_NO(priv->ext_cap); i++)
		input_report_key(dev, BTN_0 + i, hw->ext_buttons & (1 << i));
}

static void blueberry_tp_synaptics_report_slot(struct input_dev *dev, int slot,
				  const struct synaptics_hw_state *hw)
{
	input_mt_slot(dev, slot);
	input_mt_report_slot_state(dev, MT_TOOL_FINGER, (hw != NULL));
	if (!hw)
		return;

	input_report_abs(dev, ABS_MT_POSITION_X, hw->x);
	input_report_abs(dev, ABS_MT_POSITION_Y, blueberry_tp_synaptics_invert_y(hw->y));
	input_report_abs(dev, ABS_MT_PRESSURE, hw->z);
}

static void blueberry_tp_synaptics_report_mt_data(struct synaptics_data *priv,
				     struct synaptics_mt_state *mt_state,
				     const struct synaptics_hw_state *sgm)
{
	struct input_dev *dev = priv->abs_dev;
	struct synaptics_hw_state *agm = &priv->agm;
	struct synaptics_mt_state *old = &priv->mt_state;

	switch (mt_state->count) {
	case 0:
		blueberry_tp_synaptics_report_slot(dev, 0, NULL);
		blueberry_tp_synaptics_report_slot(dev, 1, NULL);
		break;
	case 1:
		if (mt_state->sgm == -1) {
			blueberry_tp_synaptics_report_slot(dev, 0, NULL);
			blueberry_tp_synaptics_report_slot(dev, 1, NULL);
		} else if (mt_state->sgm == 0) {
			blueberry_tp_synaptics_report_slot(dev, 0, sgm);
			blueberry_tp_synaptics_report_slot(dev, 1, NULL);
		} else {
			blueberry_tp_synaptics_report_slot(dev, 0, NULL);
			blueberry_tp_synaptics_report_slot(dev, 1, sgm);
		}
		break;
	default:
		/*
		 * If the finger slot contained in SGM is valid, and either
		 * hasn't changed, or is new, or the old SGM has now moved to
		 * AGM, then report SGM in MTB slot 0.
		 * Otherwise, empty MTB slot 0.
		 */
		if (mt_state->sgm != -1 &&
		    (mt_state->sgm == old->sgm ||
		     old->sgm == -1 || mt_state->agm == old->sgm))
			blueberry_tp_synaptics_report_slot(dev, 0, sgm);
		else
			blueberry_tp_synaptics_report_slot(dev, 0, NULL);

		/*
		 * If the finger slot contained in AGM is valid, and either
		 * hasn't changed, or is new, then report AGM in MTB slot 1.
		 * Otherwise, empty MTB slot 1.
		 *
		 * However, in the case where the AGM is new, make sure that
		 * that it is either the same as the old SGM, or there was no
		 * SGM.
		 *
		 * Otherwise, if the SGM was just 1, and the new AGM is 2, then
		 * the new AGM will keep the old SGM's tracking ID, which can
		 * cause apparent drumroll.  This happens if in the following
		 * valid finger sequence:
		 *
		 *  Action                 SGM  AGM (MTB slot:Contact)
		 *  1. Touch contact 0    (0:0)
		 *  2. Touch contact 1    (0:0, 1:1)
		 *  3. Lift  contact 0    (1:1)
		 *  4. Touch contacts 2,3 (0:2, 1:3)
		 *
		 * In step 4, contact 3, in AGM must not be given the same
		 * tracking ID as contact 1 had in step 3.  To avoid this,
		 * the first agm with contact 3 is dropped and slot 1 is
		 * invalidated (tracking ID = -1).
		 */
		if (mt_state->agm != -1 &&
		    (mt_state->agm == old->agm ||
		     (old->agm == -1 &&
		      (old->sgm == -1 || mt_state->agm == old->sgm))))
			blueberry_tp_synaptics_report_slot(dev, 1, agm);
		else
			blueberry_tp_synaptics_report_slot(dev, 1, NULL);
		break;
	}

	/* Don't use active slot count to generate BTN_TOOL events. */
	input_mt_report_pointer_emulation(dev, false);

	/* Send the number of fingers reported by touchpad itself. */
	input_mt_report_finger_count(dev, mt_state->count);

	blueberry_tp_synaptics_report_buttons(priv, sgm);

	input_sync(dev);
}

/* Handle case where mt_state->count = 0 */
static void blueberry_tp_synaptics_image_sensor_0f(struct synaptics_data *priv,
				      struct synaptics_mt_state *mt_state)
{
	blueberry_tp_synaptics_mt_state_set(mt_state, 0, -1, -1);
	priv->mt_state_lost = false;
}

/* Handle case where mt_state->count = 1 */
static void blueberry_tp_synaptics_image_sensor_1f(struct synaptics_data *priv,
				      struct synaptics_mt_state *mt_state)
{
	struct synaptics_hw_state *agm = &priv->agm;
	struct synaptics_mt_state *old = &priv->mt_state;

	/*
	 * If the last AGM was (0,0,0), and there is only one finger left,
	 * then we absolutely know that SGM contains slot 0, and all other
	 * fingers have been removed.
	 */
	if (priv->agm_pending && agm->z == 0) {
		blueberry_tp_synaptics_mt_state_set(mt_state, 1, 0, -1);
		priv->mt_state_lost = false;
		return;
	}

	switch (old->count) {
	case 0:
		blueberry_tp_synaptics_mt_state_set(mt_state, 1, 0, -1);
		break;
	case 1:
		/*
		 * If mt_state_lost, then the previous transition was 3->1,
		 * and SGM now contains either slot 0 or 1, but we don't know
		 * which.  So, we just assume that the SGM now contains slot 1.
		 *
		 * If pending AGM and either:
		 *   (a) the previous SGM slot contains slot 0, or
		 *   (b) there was no SGM slot
		 * then, the SGM now contains slot 1
		 *
		 * Case (a) happens with very rapid "drum roll" gestures, where
		 * slot 0 finger is lifted and a new slot 1 finger touches
		 * within one reporting interval.
		 *
		 * Case (b) happens if initially two or more fingers tap
		 * briefly, and all but one lift before the end of the first
		 * reporting interval.
		 *
		 * (In both these cases, slot 0 will becomes empty, so SGM
		 * contains slot 1 with the new finger)
		 *
		 * Else, if there was no previous SGM, it now contains slot 0.
		 *
		 * Otherwise, SGM still contains the same slot.
		 */
		if (priv->mt_state_lost ||
		    (priv->agm_pending && old->sgm <= 0))
			blueberry_tp_synaptics_mt_state_set(mt_state, 1, 1, -1);
		else if (old->sgm == -1)
			blueberry_tp_synaptics_mt_state_set(mt_state, 1, 0, -1);
		break;
	case 2:
		/*
		 * If mt_state_lost, we don't know which finger SGM contains.
		 *
		 * So, report 1 finger, but with both slots empty.
		 * We will use slot 1 on subsequent 1->1
		 */
		if (priv->mt_state_lost) {
			blueberry_tp_synaptics_mt_state_set(mt_state, 1, -1, -1);
			break;
		}
		/*
		 * Since the last AGM was NOT (0,0,0), it was the finger in
		 * slot 0 that has been removed.
		 * So, SGM now contains previous AGM's slot, and AGM is now
		 * empty.
		 */
		blueberry_tp_synaptics_mt_state_set(mt_state, 1, old->agm, -1);
		break;
	case 3:
		/*
		 * Since last AGM was not (0,0,0), we don't know which finger
		 * is left.
		 *
		 * So, report 1 finger, but with both slots empty.
		 * We will use slot 1 on subsequent 1->1
		 */
		blueberry_tp_synaptics_mt_state_set(mt_state, 1, -1, -1);
		priv->mt_state_lost = true;
		break;
	case 4:
	case 5:
		/* mt_state was updated by AGM-CONTACT packet */
		break;
	}
}

/* Handle case where mt_state->count = 2 */
static void blueberry_tp_synaptics_image_sensor_2f(struct synaptics_data *priv,
				      struct synaptics_mt_state *mt_state)
{
	struct synaptics_mt_state *old = &priv->mt_state;

	switch (old->count) {
	case 0:
		blueberry_tp_synaptics_mt_state_set(mt_state, 2, 0, 1);
		break;
	case 1:
		/*
		 * If previous SGM contained slot 1 or higher, SGM now contains
		 * slot 0 (the newly touching finger) and AGM contains SGM's
		 * previous slot.
		 *
		 * Otherwise, SGM still contains slot 0 and AGM now contains
		 * slot 1.
		 */
		if (old->sgm >= 1)
			blueberry_tp_synaptics_mt_state_set(mt_state, 2, 0, old->sgm);
		else
			blueberry_tp_synaptics_mt_state_set(mt_state, 2, 0, 1);
		break;
	case 2:
		/*
		 * If mt_state_lost, SGM now contains either finger 1 or 2, but
		 * we don't know which.
		 * So, we just assume that the SGM contains slot 0 and AGM 1.
		 */
		if (priv->mt_state_lost)
			blueberry_tp_synaptics_mt_state_set(mt_state, 2, 0, 1);
		/*
		 * Otherwise, use the same mt_state, since it either hasn't
		 * changed, or was updated by a recently received AGM-CONTACT
		 * packet.
		 */
		break;
	case 3:
		/*
		 * 3->2 transitions have two unsolvable problems:
		 *  1) no indication is given which finger was removed
		 *  2) no way to tell if agm packet was for finger 3
		 *     before 3->2, or finger 2 after 3->2.
		 *
		 * So, report 2 fingers, but empty all slots.
		 * We will guess slots [0,1] on subsequent 2->2.
		 */
		blueberry_tp_synaptics_mt_state_set(mt_state, 2, -1, -1);
		priv->mt_state_lost = true;
		break;
	case 4:
	case 5:
		/* mt_state was updated by AGM-CONTACT packet */
		break;
	}
}

/* Handle case where mt_state->count = 3 */
static void blueberry_tp_synaptics_image_sensor_3f(struct synaptics_data *priv,
				      struct synaptics_mt_state *mt_state)
{
	struct synaptics_mt_state *old = &priv->mt_state;

	switch (old->count) {
	case 0:
		blueberry_tp_synaptics_mt_state_set(mt_state, 3, 0, 2);
		break;
	case 1:
		/*
		 * If previous SGM contained slot 2 or higher, SGM now contains
		 * slot 0 (one of the newly touching fingers) and AGM contains
		 * SGM's previous slot.
		 *
		 * Otherwise, SGM now contains slot 0 and AGM contains slot 2.
		 */
		if (old->sgm >= 2)
			blueberry_tp_synaptics_mt_state_set(mt_state, 3, 0, old->sgm);
		else
			blueberry_tp_synaptics_mt_state_set(mt_state, 3, 0, 2);
		break;
	case 2:
		/*
		 * If the AGM previously contained slot 3 or higher, then the
		 * newly touching finger is in the lowest available slot.
		 *
		 * If SGM was previously 1 or higher, then the new SGM is
		 * now slot 0 (with a new finger), otherwise, the new finger
		 * is now in a hidden slot between 0 and AGM's slot.
		 *
		 * In all such cases, the SGM now contains slot 0, and the AGM
		 * continues to contain the same slot as before.
		 */
		if (old->agm >= 3) {
			blueberry_tp_synaptics_mt_state_set(mt_state, 3, 0, old->agm);
			break;
		}

		/*
		 * After some 3->1 and all 3->2 transitions, we lose track
		 * of which slot is reported by SGM and AGM.
		 *
		 * For 2->3 in this state, report 3 fingers, but empty all
		 * slots, and we will guess (0,2) on a subsequent 0->3.
		 *
		 * To userspace, the resulting transition will look like:
		 *    2:[0,1] -> 3:[-1,-1] -> 3:[0,2]
		 */
		if (priv->mt_state_lost) {
			blueberry_tp_synaptics_mt_state_set(mt_state, 3, -1, -1);
			break;
		}

		/*
		 * If the (SGM,AGM) really previously contained slots (0, 1),
		 * then we cannot know what slot was just reported by the AGM,
		 * because the 2->3 transition can occur either before or after
		 * the AGM packet. Thus, this most recent AGM could contain
		 * either the same old slot 1 or the new slot 2.
		 * Subsequent AGMs will be reporting slot 2.
		 *
		 * To userspace, the resulting transition will look like:
		 *    2:[0,1] -> 3:[0,-1] -> 3:[0,2]
		 */
		blueberry_tp_synaptics_mt_state_set(mt_state, 3, 0, -1);
		break;
	case 3:
		/*
		 * If, for whatever reason, the previous agm was invalid,
		 * Assume SGM now contains slot 0, AGM now contains slot 2.
		 */
		if (old->agm <= 2)
			blueberry_tp_synaptics_mt_state_set(mt_state, 3, 0, 2);
		/*
		 * mt_state either hasn't changed, or was updated by a recently
		 * received AGM-CONTACT packet.
		 */
		break;

	case 4:
	case 5:
		/* mt_state was updated by AGM-CONTACT packet */
		break;
	}
}

/* Handle case where mt_state->count = 4, or = 5 */
static void blueberry_tp_synaptics_image_sensor_45f(struct synaptics_data *priv,
				       struct synaptics_mt_state *mt_state)
{
	/* mt_state was updated correctly by AGM-CONTACT packet */
	priv->mt_state_lost = false;
}

static void blueberry_tp_synaptics_image_sensor_process(struct synaptics_data *priv,
					   struct synaptics_hw_state *sgm)
{
	struct synaptics_hw_state *agm = &priv->agm;
	struct synaptics_mt_state mt_state;

	/* Initialize using current mt_state (as updated by last agm) */
	mt_state = agm->mt_state;

	/*
	 * Update mt_state using the new finger count and current mt_state.
	 */
	if (sgm->z == 0)
		blueberry_tp_synaptics_image_sensor_0f(priv, &mt_state);
	else if (sgm->w >= 4)
		blueberry_tp_synaptics_image_sensor_1f(priv, &mt_state);
	else if (sgm->w == 0)
		blueberry_tp_synaptics_image_sensor_2f(priv, &mt_state);
	else if (sgm->w == 1 && mt_state.count <= 3)
		blueberry_tp_synaptics_image_sensor_3f(priv, &mt_state);
	else
		blueberry_tp_synaptics_image_sensor_45f(priv, &mt_state);

	/* Send resulting input events to user space */
	blueberry_tp_synaptics_report_mt_data(priv, &mt_state, sgm);

	/* Store updated mt_state */
	priv->mt_state = agm->mt_state = mt_state;
	priv->agm_pending = false;
}

/*
 *  called for each full received packet from the touchpad
 */
static void blueberry_tp_synaptics_process_packet(struct synaptics_data *priv)
{
	struct input_dev *dev = priv->abs_dev;
	struct synaptics_hw_state hw;
	int num_fingers;
	int finger_width;

	if (blueberry_tp_synaptics_parse_hw_state(priv->packet, priv, &hw)) 
		return;

	if (SYN_CAP_IMAGE_SENSOR(priv->ext_cap_0c)) {
		blueberry_tp_synaptics_image_sensor_process(priv, &hw);
		return;
	}

	if (hw.scroll) {
		priv->scroll += hw.scroll;

		while (priv->scroll >= 4) {
			input_report_key(dev, BTN_BACK, !hw.down);
			input_sync(dev);
			input_report_key(dev, BTN_BACK, hw.down);
			input_sync(dev);
			priv->scroll -= 4;
		}
		while (priv->scroll <= -4) {
			input_report_key(dev, BTN_FORWARD, !hw.up);
			input_sync(dev);
			input_report_key(dev, BTN_FORWARD, hw.up);
			input_sync(dev);
			priv->scroll += 4;
		}
		return;
	}

	if (hw.z > 0 && hw.x > 1) {
		num_fingers = 1;
		finger_width = 5;
		if (SYN_CAP_EXTENDED(priv->capabilities)) {
			switch (hw.w) {
			case 0 ... 1:
				if (SYN_CAP_MULTIFINGER(priv->capabilities))
					num_fingers = hw.w + 2;
				break;
			case 2:
				if (SYN_MODEL_PEN(priv->model_id))
					;   /* Nothing, treat a pen as a single finger */
				break;
			case 4 ... 15:
				if (SYN_CAP_PALMDETECT(priv->capabilities))
					finger_width = hw.w;
				break;
			}
		}
	} else {
		num_fingers = 0;
		finger_width = 0;
	}

	if (SYN_CAP_ADV_GESTURE(priv->ext_cap_0c))
		blueberry_tp_synaptics_report_semi_mt_data(dev, &hw, &priv->agm,
					      num_fingers);

	/* Post events
	 * BTN_TOUCH has to be first as mousedev relies on it when doing
	 * absolute -> relative conversion
	 */
	if (hw.z > 30) input_report_key(dev, BTN_TOUCH, 1);
	if (hw.z < 25) input_report_key(dev, BTN_TOUCH, 0);

	if (num_fingers > 0) {
		input_report_abs(dev, ABS_X, hw.x);
		input_report_abs(dev, ABS_Y, blueberry_tp_synaptics_invert_y(hw.y));
	}
	input_report_abs(dev, ABS_PRESSURE, hw.z);

	if (SYN_CAP_PALMDETECT(priv->capabilities))
		input_report_abs(dev, ABS_TOOL_WIDTH, finger_width);

	input_report_key(dev, BTN_TOOL_FINGER, num_fingers == 1);
	if (SYN_CAP_MULTIFINGER(priv->capabilities)) {
		input_report_key(dev, BTN_TOOL_DOUBLETAP, num_fingers == 2);
		input_report_key(dev, BTN_TOOL_TRIPLETAP, num_fingers == 3);
	}

	blueberry_tp_synaptics_report_buttons(priv, &hw);

	input_sync(dev);
}

static int blueberry_tp_synaptics_validate_byte(struct synaptics_data *priv,
				   int idx, unsigned char pkt_type)
{
	static const unsigned char newabs_mask[]	= { 0xC8, 0x00, 0x00, 0xC8, 0x00 };
	static const unsigned char newabs_rel_mask[]	= { 0xC0, 0x00, 0x00, 0xC0, 0x00 };
	static const unsigned char newabs_rslt[]	= { 0x80, 0x00, 0x00, 0xC0, 0x00 };
	static const unsigned char oldabs_mask[]	= { 0xC0, 0x60, 0x00, 0xC0, 0x60 };
	static const unsigned char oldabs_rslt[]	= { 0xC0, 0x00, 0x00, 0x80, 0x00 };
	const char *packet = priv->packet;

	if (idx < 0 || idx > 4)
		return 0;

	switch (pkt_type) {

	case SYN_NEWABS:
	case SYN_NEWABS_RELAXED:
		return (packet[idx] & newabs_rel_mask[idx]) == newabs_rslt[idx];

	case SYN_NEWABS_STRICT:
		return (packet[idx] & newabs_mask[idx]) == newabs_rslt[idx];

	case SYN_OLDABS:
		return (packet[idx] & oldabs_mask[idx]) == oldabs_rslt[idx];

	default:
		pr_err("unknown packet type %d\n", pkt_type);
		return 0;
	}
}

static unsigned char blueberry_tp_synaptics_detect_pkt_type(struct synaptics_data * priv)
{
	int i;

	for (i = 0; i < 5; i++)
		if (!blueberry_tp_synaptics_validate_byte(priv, i, SYN_NEWABS_STRICT)) {
			blueberry_tp_priv_debug("using relaxed packet validation\n");
			return SYN_NEWABS_RELAXED;
		}

	return SYN_NEWABS_STRICT;
}

#ifdef SYNAPTICS_PROCESS_BYTE
static psmouse_ret_t synaptics_process_byte(struct psmouse *psmouse)
{
	struct synaptics_data *priv = psmouse->private;

	if (psmouse->pktcnt >= 6) { /* Full packet received */
		if (unlikely(priv->pkt_type == SYN_NEWABS))
			priv->pkt_type = synaptics_detect_pkt_type(psmouse);

		if (SYN_CAP_PASS_THROUGH(priv->capabilities) &&
		    synaptics_is_pt_packet(psmouse->packet)) {
			if (priv->pt_port)
				synaptics_pass_pt_packet(priv->pt_port, psmouse->packet);
		} else
			synaptics_process_packet(psmouse);

		return PSMOUSE_FULL_PACKET;
	}

	return synaptics_validate_byte(psmouse, psmouse->pktcnt - 1, priv->pkt_type) ?
		PSMOUSE_GOOD_DATA : PSMOUSE_BAD_DATA;
}
#endif

/*****************************************************************************
 *	Driver initialization/cleanup functions
 ****************************************************************************/
static void blueberry_tp_synaptics_set_abs_position_params(struct input_dev *dev,
				    struct synaptics_data *priv, int x_code,
				    int y_code)
{
	int x_min = priv->x_min ?: XMIN_NOMINAL;
	int x_max = priv->x_max ?: XMAX_NOMINAL;
	int y_min = priv->y_min ?: YMIN_NOMINAL;
	int y_max = priv->y_max ?: YMAX_NOMINAL;
	int fuzz = SYN_CAP_REDUCED_FILTERING(priv->ext_cap_0c) ?
			SYN_REDUCED_FILTER_FUZZ : 0;

	input_set_abs_params(dev, x_code, x_min, x_max, fuzz, 0);
	input_set_abs_params(dev, y_code, y_min, y_max, fuzz, 0);
	input_abs_set_res(dev, x_code, priv->x_res);
	input_abs_set_res(dev, y_code, priv->y_res);
}

static void blueberry_tp_synaptics_set_input_params(struct input_dev *dev, struct synaptics_data *priv)
{
	int i;

	/* Things that apply to both modes */
	__set_bit(INPUT_PROP_POINTER, dev->propbit);
	__set_bit(EV_KEY, dev->evbit);
	__set_bit(BTN_LEFT, dev->keybit);
	__set_bit(BTN_RIGHT, dev->keybit);

	if (SYN_CAP_MIDDLE_BUTTON(priv->capabilities))
		__set_bit(BTN_MIDDLE, dev->keybit);

	if (!priv->absolute_mode) {
		/* Relative mode */
		__set_bit(EV_REL, dev->evbit);
		__set_bit(REL_X, dev->relbit);
		__set_bit(REL_Y, dev->relbit);
		return;
	}

	/* Absolute mode */
	__set_bit(EV_ABS, dev->evbit);
	blueberry_tp_synaptics_set_abs_position_params(dev, priv, ABS_X, ABS_Y);
	input_set_abs_params(dev, ABS_PRESSURE, 0, 255, 0, 0);

	if (SYN_CAP_IMAGE_SENSOR(priv->ext_cap_0c)) {
		blueberry_tp_synaptics_set_abs_position_params(dev, priv, ABS_MT_POSITION_X,
					ABS_MT_POSITION_Y);
		/* Image sensors can report per-contact pressure */
		input_set_abs_params(dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
		/* remove INPUT_MT_POINTER because of kernel version  --  by magf */
		input_mt_init_slots(dev, 2); //, INPUT_MT_POINTER);		

		/* Image sensors can signal 4 and 5 finger clicks */
		__set_bit(BTN_TOOL_QUADTAP, dev->keybit);

		/* we use the model only supporting four fingers, so can disable this -- by magf */
		/* __set_bit(BTN_TOOL_QUINTTAP, dev->keybit); */
	} else if (SYN_CAP_ADV_GESTURE(priv->ext_cap_0c)) {
		/* Non-image sensors with AGM use semi-mt */
		__set_bit(INPUT_PROP_SEMI_MT, dev->propbit);
		/* remove the third parameter because of kernel version -- by magf */
		input_mt_init_slots(dev, 2); //, 0);
		blueberry_tp_synaptics_set_abs_position_params(dev, priv, ABS_MT_POSITION_X,
					ABS_MT_POSITION_Y);
	}

	if (SYN_CAP_PALMDETECT(priv->capabilities))
		input_set_abs_params(dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	__set_bit(BTN_TOUCH, dev->keybit);
	__set_bit(BTN_TOOL_FINGER, dev->keybit);

	if (SYN_CAP_MULTIFINGER(priv->capabilities)) {
		__set_bit(BTN_TOOL_DOUBLETAP, dev->keybit);
		__set_bit(BTN_TOOL_TRIPLETAP, dev->keybit);
	}

	if (SYN_CAP_FOUR_BUTTON(priv->capabilities) ||
	    SYN_CAP_MIDDLE_BUTTON(priv->capabilities)) {
		__set_bit(BTN_FORWARD, dev->keybit);
		__set_bit(BTN_BACK, dev->keybit);
	}

	for (i = 0; i < SYN_CAP_MULTI_BUTTON_NO(priv->ext_cap); i++)
		__set_bit(BTN_0 + i, dev->keybit);

	__clear_bit(EV_REL, dev->evbit);
	__clear_bit(REL_X, dev->relbit);
	__clear_bit(REL_Y, dev->relbit);

	if (SYN_CAP_CLICKPAD(priv->ext_cap_0c)) {
		__set_bit(INPUT_PROP_BUTTONPAD, dev->propbit);
		/* Clickpads report only left button */
		__clear_bit(BTN_RIGHT, dev->keybit);
		__clear_bit(BTN_MIDDLE, dev->keybit);
	}
}

static void blueberry_tp_synaptics_set_relative_input_params(struct synaptics_data *priv)
{
	struct input_dev *dev = priv->rel_dev;

    /* Things that apply to both modes */
    __set_bit(INPUT_PROP_POINTER, dev->propbit);
    __set_bit(EV_KEY, dev->evbit);
    __set_bit(BTN_LEFT, dev->keybit);
    __set_bit(BTN_RIGHT, dev->keybit);

    /* Relative mode */
    __set_bit(EV_REL, dev->evbit);
    __set_bit(REL_X, dev->relbit);
    __set_bit(REL_Y, dev->relbit);

    return;
}

/* 
 * 0x0d:0x01 -> enable touchpad, 0x0d:0x00 -> disable touchpad 
 */
static int blueberry_tp_synaptics_enable(struct synaptics_data *priv)
{
    int ret;
    unsigned char cmd[3] = {0x0d, 0x00, 0x01};

    ret = blueberry_tp_i2c_write_data(priv->client, cmd, 3);
    if(ret) {
        pr_err("blueberry_tp_synaptics: enable touchpad failed.\n");
        return ret;
    }
    blueberry_tp_priv_debug("enable touchpad success.\n");

    enable_irq(priv->client->irq);
    priv->enable = BLUEBERRY_TP_ENABLE;

    return 0;
}

static int blueberry_tp_synaptics_disable(struct synaptics_data *priv)
{
    int ret;
    unsigned char cmd[3] = {0x0d, 0x00, 0x00};

    ret = blueberry_tp_i2c_write_data(priv->client, cmd, 3);
    if(ret) {
        pr_err("blueberry_tp_synaptics: disable touchpad failed.\n");
        return ret;
    }
    blueberry_tp_priv_debug("disable touchpad success.\n");

    disable_irq(priv->client->irq);
    priv->enable = BLUEBERRY_TP_DISABLE;

    return 0;
}

static int blueberry_tp_synaptics_set_relative_mode(struct synaptics_data *priv)
{
	int ret;

	if(priv->working_mode == BLUEBERRY_TP_REL_MODE) {
		/* already relative mode */
		return 0;
	}
	/* when write new mode byte to ec, ec should clear its internal data buffer. */
	ret = blueberry_tp_synaptics_mode_cmd(priv, 0x40);
	if(ret) {
		pr_err("blueberry_tp_synaptics: set touchpad to relative mode failed.\n");
		return ret;
	}
	blueberry_tp_priv_debug("synaptics touchpad relative mode enabled.\n");

    priv->working_mode = BLUEBERRY_TP_REL_MODE;

    return ret;
}

static int blueberry_tp_synaptics_set_absolute_mode(struct synaptics_data *priv)
{
	int ret;
	if(priv->working_mode == BLUEBERRY_TP_ABS_MODE) {
		/* already absolute mode */
		return 0;
	}
	ret = blueberry_tp_synaptics_set_mode(priv);
	if(ret) {
		pr_err("synaptics set to absolute mode failed.\n");
		return ret;
	}
	blueberry_tp_priv_debug("set absolute mode succ.\n");
	priv->working_mode = BLUEBERRY_TP_ABS_MODE;

	return 0;
}

static ssize_t blueberry_tp_synaptics_enable_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    if (*buf == 'E') {

        blueberry_tp_priv_debug("user enable touchpad by sysfs.\n");
        if(priv->enable == BLUEBERRY_TP_DISABLE)
            blueberry_tp_synaptics_enable(priv);

    }else if(*buf == 'D') {

        blueberry_tp_priv_debug("user disable touchpad by sysfs.\n");
        if(priv->enable == BLUEBERRY_TP_ENABLE)
            blueberry_tp_synaptics_disable(priv);

    }

    return len;
};

static ssize_t blueberry_tp_synaptics_enable_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", priv->enable > 0 ? 1 : 0);
}

static ssize_t blueberry_tp_synaptics_working_mode_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    if (*buf == 'M') {

        blueberry_tp_priv_debug("user switch to mutil-touch mode by sysfs.\n");
        //blueberry_tp_synaptics_disable(bbtp);
        //msleep(1);
        blueberry_tp_synaptics_set_absolute_mode(priv);
        //msleep(1);
        //blueberry_tp_synaptics_enable(bbtp);

    }else if(*buf == 'S') {

        blueberry_tp_priv_debug("user switch to single-touch mode by sysfs.\n");
        //blueberry_tp_synaptics_disable(bbtp);
        //msleep(1);
        blueberry_tp_synaptics_set_relative_mode(priv);
        //msleep(1);
        //blueberry_tp_synaptics_enable(bbtp);

    }

    return len;
};

static ssize_t blueberry_tp_synaptics_working_mode_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", priv->working_mode > 0 ? 1 : 0);
}

static ssize_t blueberry_tp_synaptics_show_disable_gesture(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_data *priv = i2c_get_clientdata(client);

	return sprintf(buf, "%c\n", priv->disable_gesture ? '1' : '0');
}

static ssize_t blueberry_tp_synaptics_set_disable_gesture(struct device *dev,
					     struct device_attribute *attr, const char *buf,
					     size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct synaptics_data *priv = i2c_get_clientdata(client);
	unsigned int value;
	int err;

	err = kstrtouint(buf, 10, &value);
	if (err)
		return err;

	if (value > 1)
		return -EINVAL;

	if (value == priv->disable_gesture)
		return len;

	priv->disable_gesture = value;
	if (value)
		priv->mode |= SYN_BIT_DISABLE_GESTURE;
	else
		priv->mode &= ~SYN_BIT_DISABLE_GESTURE;

	if (blueberry_tp_synaptics_mode_cmd(priv, priv->mode))
		return -EIO;

	return len;
}

static ssize_t blueberry_tp_synaptics_show_query(struct device *dev,
                struct device_attribute *attr, char *buf)
{
//    struct i2c_client *client = to_i2c_client(dev);
//    struct synaptics_data *priv = i2c_get_clientdata(client);

    return sprintf(buf, "%s\n", "just a place holder");
}

static ssize_t blueberry_tp_synaptics_set_query(struct device *dev,
                         struct device_attribute *attr, const char *buf,
                         size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);
    unsigned int value;
	unsigned char data[3] = {0x00, 0x00, 0x00};
	char prompt[20];
    int err;

    err = kstrtouint(buf, 10, &value);
    if (err)
        return err;

    if (value > 0x0f)
        return -EINVAL;
	
	sprintf(prompt, "cmd 0x%x", (unsigned char)value);

	err = blueberry_tp_synaptics_execute_query(priv, (unsigned char)value, data, prompt);
	if(err) {
		pr_err("query %s failed\n", prompt);
		return -EIO;
	}
	pr_err("query %s result: 0x%x, 0x%x, 0x%x\n", prompt, data[0], data[1], data[2]);

    return len;
}

static ssize_t blueberry_tp_synaptics_rel_threshold1_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);
    unsigned int thres;

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    thres = simple_strtol(buf, NULL, 0);
    if(thres > 15) {
        thres = 15;
    }
    blueberry_tp_priv_debug("user set threshold1 = %d\n", thres);
    priv->rel_threshold1 = thres;

    return len;
};

static ssize_t blueberry_tp_synaptics_rel_threshold1_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", priv->rel_threshold1);
}

static ssize_t blueberry_tp_synaptics_rel_threshold2_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);
    unsigned int thres;

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    thres = simple_strtol(buf, NULL, 0);
    if(thres > 15) {
        thres = 15;
    }
    blueberry_tp_priv_debug("user set threshold2 = %d\n", thres);
    priv->rel_threshold2 = thres;

    return len;
};

static ssize_t blueberry_tp_synaptics_rel_threshold2_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", priv->rel_threshold2);
}

static ssize_t blueberry_tp_synaptics_rel_speed_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);
    unsigned int speed;

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    speed = simple_strtol(buf, NULL, 0);
    if(speed > 2) {
        speed = 2;
    }
    blueberry_tp_priv_debug("user set speed = %d\n", speed);
    priv->rel_pointer_speed = speed;

    return len;
};

static ssize_t blueberry_tp_synaptics_rel_speed_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", priv->rel_pointer_speed);
}

static int calculate_sensitivity_factor(int sensitivity);
static ssize_t blueberry_tp_synaptics_sensitivity_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);
    unsigned int sensitivity;

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    sensitivity = simple_strtol(buf, NULL, 0);
    if(sensitivity > 20) {
        sensitivity = 20;
    }
    if(sensitivity < 1) {
        sensitivity = 1;
    }
    blueberry_tp_priv_debug("user set sensitivity = %d\n", sensitivity);
    priv->sensitivity = sensitivity;
    priv->sensitivity_factor = calculate_sensitivity_factor(priv->sensitivity);
    blueberry_tp_priv_debug("sensitivity_fator = %d\n", priv->sensitivity_factor);

    return len;
};

static ssize_t blueberry_tp_synaptics_sensitivity_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", priv->sensitivity);
}

static ssize_t blueberry_tp_vendor_id_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct synaptics_data *priv = i2c_get_clientdata(client);

    blueberry_tp_priv_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "0x%x", 0x02);
}

static DEVICE_ATTR(vendor_id, 0660, blueberry_tp_vendor_id_show, NULL);
static DEVICE_ATTR(enable, 0660, blueberry_tp_synaptics_enable_show, blueberry_tp_synaptics_enable_store);
static DEVICE_ATTR(mode, 0660, blueberry_tp_synaptics_working_mode_show, blueberry_tp_synaptics_working_mode_store);
static DEVICE_ATTR(disable_gesture, 0660, blueberry_tp_synaptics_show_disable_gesture, blueberry_tp_synaptics_set_disable_gesture);
static DEVICE_ATTR(query, 0660, blueberry_tp_synaptics_show_query, blueberry_tp_synaptics_set_query);
static DEVICE_ATTR(rel_threshold1, 0660, blueberry_tp_synaptics_rel_threshold1_show, blueberry_tp_synaptics_rel_threshold1_store);
static DEVICE_ATTR(rel_threshold2, 0660, blueberry_tp_synaptics_rel_threshold2_show, blueberry_tp_synaptics_rel_threshold2_store);
static DEVICE_ATTR(rel_speed, 0660, blueberry_tp_synaptics_rel_speed_show, blueberry_tp_synaptics_rel_speed_store);
static DEVICE_ATTR(sensitivity, 0660, blueberry_tp_synaptics_sensitivity_show, blueberry_tp_synaptics_sensitivity_store);

static struct attribute *blueberry_tp_synaptics_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_mode.attr,
    &dev_attr_disable_gesture.attr,
	&dev_attr_query.attr,
    &dev_attr_rel_threshold1.attr,
    &dev_attr_rel_threshold2.attr,
    &dev_attr_rel_speed.attr,
	&dev_attr_sensitivity.attr,
	&dev_attr_vendor_id.attr,
    NULL
};

static struct attribute_group blueberry_tp_synaptics_attribute_group = {
        .attrs = blueberry_tp_synaptics_attributes
};

static int blueberry_tp_synaptics_resume(struct i2c_client *client)
{
    /* struct synaptics_data *priv = i2c_get_clientdata(client); */
    printk("%s() enter, enable touchpad power\n", __func__);
    gpio_set_value(RK30_PIN1_PB5, GPIO_HIGH);								/* bad usage, should improve */
    return 0;
}

static int blueberry_tp_synaptics_suspend(struct i2c_client *client, pm_message_t mesg)
{
    /* struct synaptics *priv = i2c_get_clientdata(client); */
    printk("%s() enter, disable touchpad power\n", __func__);
    gpio_set_value(RK30_PIN1_PB5, GPIO_LOW);								/* bad usage, should improve */
    return 0;
}

static void blueberry_tp_synaptics_early_suspend(struct early_suspend *handler)
{
    struct synaptics_data *priv = (struct synaptics_data *) container_of(handler, struct synaptics_data, early_drv);
    unsigned char cmd[3] = {0x0d, 0x00, 0x00};
    int ret;

    blueberry_tp_priv_debug("enter blueberry_tp_synaptics_early_suspend.\n");
    if(!priv){
        pr_err("early_suspend, priv is NULL\n");
        return;
    }
    /* 0x0d : 0x01 -- enable touchpad, 0x0d : 0x00 -- disable touchpad */
    if(priv->enable == BLUEBERRY_TP_ENABLE) {
        ret = blueberry_tp_i2c_write_data(priv->client, cmd, 3);
        if(ret) {
            pr_err("blueberry_tp_synaptics: disable touchpad failed.\n");
        }
        blueberry_tp_priv_debug("before suspend, status enable, disable touchpad success.\n");
    }else{
        blueberry_tp_priv_debug("before suspend, status disable, do nothing.\n");
    }

    disable_irq(priv->client->irq);
    return;
}

static void blueberry_tp_synaptics_early_resume(struct early_suspend *handler)
{
    struct synaptics_data *priv = (struct synaptics_data *) container_of(handler, struct synaptics_data, early_drv);
    unsigned char cmd[3] = {0x0d, 0x00, 0x01};
    int ret;
    
	blueberry_tp_priv_debug("enter blueberry_tp_synaptics_early_resume.\n");
    if(!priv) {
        pr_err("early_resume, fatal error, priv is NULL\n");
        return;
    }

    blueberry_tp_priv_debug("here, touchpad must be disabled.\n");

    /* 0x0d : 0x01 -- enable touchpad, 0x0d : 0x00 -- disable touchpad */
    if(priv->enable == BLUEBERRY_TP_ENABLE) {
        ret = blueberry_tp_i2c_write_data(priv->client, cmd, 3);
        if(ret) {
            pr_err("blueberry_tp_synaptics: early_resume, fatal error, enable failed.\n");
            return;
        }
        blueberry_tp_priv_debug("when suspend, status enable, enable it success.\n");
    }else{
        blueberry_tp_priv_debug("when suspend, status disable, do nothing.\n");
    }

    enable_irq(priv->client->irq);
    return;
}

static irqreturn_t blueberry_tp_synaptics_interrupt(int irq, void *dev_id)
{
    struct synaptics_data *priv = dev_id;

    disable_irq_nosync(priv->client->irq);
    schedule_work(&priv->input_work);
    
    return IRQ_HANDLED;
}   

static void blueberry_tp_synaptics_packet_dump(struct synaptics_data *priv)
{
    int i;

    printk(KERN_ERR "blueberry_tp_syn packet [");
    for (i = 0; i < priv->pktsize; i++)
        printk("%s0x%02x ", i ? ", " : " ", priv->packet[i]);
    printk("]\n");
}

static int calculate_sensitivity_factor(int sensitivity)
{
    int factor;

    if(sensitivity <= 2)
       factor = sensitivity*256/32 ;
    else if((sensitivity >= 3) && (sensitivity <= 10 ))
       factor = (sensitivity-2)*256/8 ;
    else
       factor = (sensitivity-6)*256/4 ;

    return factor;
}

static int blueberry_tp_synaptics_do_mouse_accel(struct synaptics_data *priv, int delta)
{
    int new_delta = delta;

    if(priv->rel_pointer_speed != 0) {
        if(abs(delta) > priv->rel_threshold1) {
            new_delta *= 2;

            if((abs(delta) > priv->rel_threshold2) && (priv->rel_pointer_speed == 2))
                new_delta *= 2;
        }
    }

    return new_delta;
}

static void blueberry_tp_synaptics_report_relative(struct synaptics_data *priv)
{
    int x_sign, y_sign, left_btn, right_btn, delta_x, delta_y;
    int accel_delta_x, accel_delta_y;
	int numerator;
    unsigned char *packet = priv->packet;
    struct input_dev *dev = priv->rel_dev;

    x_sign = (packet[0] & X_SIGN_MASK) ? 1:0;
    y_sign = (packet[0] & Y_SIGN_MASK) ? 1:0;

    if(packet[2] == 0) {
        y_sign = 0;
    }

    left_btn = (packet[0] & LEFT_BTN_MASK) ? 1:0;
    right_btn = (packet[0] & RIGHT_BTN_MASK) ? 1:0;
    delta_x = x_sign ? (packet[1] - 0xff) : packet[1];
    delta_y = y_sign ? (packet[2] - 0xff) : packet[2];

    blueberry_tp_priv_debug("packet[1]=0x%x, x_sign=0x%x, delta_x=0x%x, packet[2]=0x%x, y_sign=0x%x, delta_y=0x%x\n", \
                    packet[1], x_sign, delta_x, packet[2], y_sign, delta_y);

    delta_y = -1 * delta_y;

    /* accel mouse speed */
    accel_delta_x = blueberry_tp_synaptics_do_mouse_accel(priv, delta_x);
    accel_delta_y = blueberry_tp_synaptics_do_mouse_accel(priv, delta_y);
   
    if(priv->sensitivity != MOUSE_SENSITIVITY_DEFAULT) {
        if(accel_delta_x) {
            numerator = accel_delta_x * priv->sensitivity_factor + priv->dx_remainder;
            accel_delta_x = numerator / 256;
            priv->dx_remainder = numerator % 256;
            if((numerator < 0) && (priv->dx_remainder > 0)) {
                accel_delta_x ++;
                priv->dx_remainder -= 256;
            }
        }
        if(accel_delta_y) {
            numerator = accel_delta_y * priv->sensitivity_factor + priv->dy_remainder;
            accel_delta_y = numerator / 256;
            priv->dy_remainder = numerator % 256;
            if((numerator < 0) && (priv->dy_remainder > 0)) {
                accel_delta_y ++;
                priv->dy_remainder -=256;
            }
        }
    }
 
	input_report_rel(dev, REL_X, accel_delta_x);  /* delta_x); */
    input_report_rel(dev, REL_Y, accel_delta_y);  /* delta_y); */
    input_report_key(dev, BTN_LEFT, left_btn);
    input_report_key(dev, BTN_RIGHT, right_btn);
    input_sync(dev);
}

static void blueberry_tp_synaptics_input_work(struct work_struct *work)
{   
    struct synaptics_data *priv = (struct synaptics_data *)container_of(work, struct synaptics_data, input_work);
    int ret;

    /* 0x03 is read 6bytes packet command, 0x08 is packet array size, 6 is multi-touch packet len */
    memset(priv->packet, 0, 0x08);

    if(priv->working_mode == BLUEBERRY_TP_ABS_MODE) {

        ret = blueberry_tp_smbus_read_byte_block(priv->client, 0x03, priv->packet, 6);
        if(ret) {
            pr_err("read 6 bytes packet failed.\n");
            goto enable_irq;
        }
        if(priv->debug)
            blueberry_tp_synaptics_packet_dump(priv);

		if(unlikely(priv->pkt_type == SYN_NEWABS)) 
			priv->pkt_type = blueberry_tp_synaptics_detect_pkt_type(priv);

		blueberry_tp_synaptics_process_packet(priv);

    }else{

        ret = blueberry_tp_smbus_read_byte_block(priv->client, 0x03, priv->packet, 3);
        if(ret) {
            pr_err("read 3 bytes packet failed.\n");
            goto enable_irq;
        }

        blueberry_tp_synaptics_report_relative(priv);

    }

enable_irq:
    enable_irq(priv->client->irq);
    return;
}

static int blueberry_tp_who_am_i(struct i2c_client *client)
{
	int ret;
	unsigned char id;	/* 0x01 -- elan, 0x02 -- synaptics */
	
	ret = blueberry_tp_smbus_read_byte(client, 0x0e, &id);
	if(ret) {
		pr_err("error while read touchpad id, exit.\n");
		return ret;
	}
	pr_info("blueberry_tp_who_am_i: id=0x%x (%s).\n", id, id==1 ? "elan":"synaptics");
	
	if(id != 0x02)
		return -1;

	return 0;
}

static int blueberry_tp_synaptics_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_data *priv = NULL;
    int ret;

    pr_info("enter blueberry_tp_probe\n");

	if(blueberry_tp_who_am_i(client))
		return -ENODEV;
	
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
    if(!priv) {
        pr_err("can't alloc priv of synaptics_data, no memory.\n");
        ret = -ENOMEM;
        goto kfree_dev;
    }

	priv->rel_threshold1 = 6;
	priv->rel_threshold2 = 10;
	priv->rel_pointer_speed = 1;

    priv->sensitivity = 10;
    priv->sensitivity_factor = 256;
    priv->dx_remainder = 0;
    priv->dy_remainder = 0;
    priv->sensitivity_factor = calculate_sensitivity_factor(priv->sensitivity);
    pr_err("calculated sensitivity_factor=%d from sensitivity=%d\n", priv->sensitivity_factor, priv->sensitivity);

	priv->client = client;
	priv->pktsize = 6;								/* packet size of absolute mode */
	priv->debug = 1;

    priv->enable = BLUEBERRY_TP_ENABLE;				/* initially touchpad is enabled. */
    priv->working_mode = BLUEBERRY_TP_REL_MODE; 	/* initially we use REL_MODE, rather than BLUEBERRY_TP_ABS_MODE; */
//	priv->working_mode = BLUEBERRY_TP_ABS_MODE;

    INIT_WORK(&priv->input_work, blueberry_tp_synaptics_input_work);
    i2c_set_clientdata(client, priv);

	if(blueberry_tp_synaptics_query_hardware(priv)) {
		pr_err("Unable to query device.\n");
		ret = -ENODEV;
		goto kfree_dev;
	}

	priv->absolute_mode = true;						/* only act as a flag generator, do not indicate relative/absolute mode */
	if(SYN_ID_DISGEST_SUPPORTED(priv->identity)) {
		blueberry_tp_priv_debug("disable_gesture is true. in absolute mode and Wmode=1, it is EWmode\n");
		priv->disable_gesture = true;				/* for absolute mode, it only indicate that EWmode is 1 */
	}												/* relative mode do not use this flag, it diffect use mode byte 0x40 */

	priv->rate = 200;	/* 0xc8 */					/* only as a flag generator, we do not touch touchpad's sample rate */
													/* EC use 80 as a default value */

	if(priv->working_mode == BLUEBERRY_TP_ABS_MODE) {
		if(blueberry_tp_synaptics_set_mode(priv)) {		
			pr_err("Unable to initialize device.\n");
			ret = -ENODEV;
			goto kfree_dev;
		}
	}else{											/* for relative mode, use mode byte 0x40 directly */
		if(blueberry_tp_synaptics_mode_cmd(priv, 0x40)) {
			pr_err("Unable to initialize device to single finger mode.\n");
			ret = -ENODEV;
			goto kfree_dev;
		}
	}
	
	priv->pkt_type = SYN_MODEL_NEWABS(priv->model_id) ? SYN_NEWABS : SYN_OLDABS;
	blueberry_tp_priv_debug("priv->pkt_type is %s\n", SYN_MODEL_NEWABS(priv->model_id) ? "SYN_NEWABS" : "SYN_OLDABS");

    blueberry_tp_priv_debug("Touchpad model: %ld, fw: %ld.%ld, id: %#lx, caps: %#lx/%#lx/%#lx, board id: %lu, fw id: %lu\n",
             SYN_ID_MODEL(priv->identity),
             SYN_ID_MAJOR(priv->identity), SYN_ID_MINOR(priv->identity),
             priv->model_id,
             priv->capabilities, priv->ext_cap, priv->ext_cap_0c,
             priv->board_id, priv->firmware_id);


    /* the following code setup interrupt */
    ret = gpio_request(client->irq, "synaptics_irq");
    if(ret){
        pr_err("Failed to request blueberry_tp irq GPIO!\n");
        goto kfree_dev;
    }
    ret = gpio_direction_input(client->irq);
    if(ret){
        pr_err("failed to set blueberry_tp irq gpio input\n");
        goto kfree_dev;
    }
    gpio_pull_updown(client->irq, GPIOPullUp);

    ret = request_irq(client->irq, blueberry_tp_synaptics_interrupt, IRQ_TYPE_LEVEL_LOW, "synaptics_irq", priv);
    if(ret){
        pr_err("request irq failed\n");
        goto kfree_dev;
    }

    /* alloc abs input device and set its params */
    priv->abs_dev = input_allocate_device();
    if(!priv->abs_dev) {
        ret = -ENOMEM;
        goto err_free_irq;
    }
    blueberry_tp_priv_debug("abs input device allocated.\n");
    priv->abs_dev->name = "blueberry_syn_abs";
    priv->abs_dev->phys = priv->client->adapter->name;
    priv->abs_dev->id.bustype = BUS_I2C;
    priv->abs_dev->dev.parent = &client->dev;

    blueberry_tp_synaptics_set_input_params(priv->abs_dev, priv);
    blueberry_tp_priv_debug("set_absolute_input_params succ.\n");

    /* alloc rel input device and set its params */
    priv->rel_dev = input_allocate_device();
    if(!priv->rel_dev) {
        ret = -ENOMEM;
        goto err_abs_input_free;
    }
    blueberry_tp_priv_debug("rel input device allocated.\n");
    priv->rel_dev->name = "blueberry_syn_rel";
    priv->rel_dev->phys = priv->client->adapter->name;
    priv->rel_dev->id.bustype = BUS_I2C;
    priv->rel_dev->dev.parent = &client->dev;

    blueberry_tp_synaptics_set_relative_input_params(priv);
	blueberry_tp_priv_debug("set_relative_input_params succ.\n");

    /* register both abs and rel input device */
    ret = input_register_device(priv->abs_dev);
    if(ret){
        pr_err("blueberry_tp_synaptics input_register_abs_device failed.\n");
        goto err_rel_input_free;
    }
    blueberry_tp_priv_debug("input_register_abs_device succ.\n");

    ret = input_register_device(priv->rel_dev);
    if(ret) {
        pr_err("blueberry_tp_synaptics input_register_rel_device failed.\n");
        goto err_reg_rel;
    }
    blueberry_tp_priv_debug("input_register_rel_device succ.\n");

#if defined(CONFIG_HAS_EARLYSUSPEND)
    priv->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
    priv->early_drv.suspend  = blueberry_tp_synaptics_early_suspend,
    priv->early_drv.resume   = blueberry_tp_synaptics_early_resume,
    register_early_suspend(&priv->early_drv);
#endif

    ret = sysfs_create_group(&client->dev.kobj, &blueberry_tp_synaptics_attribute_group);
    if (ret) {
        pr_err("%s: sysfs_create_group returned err = %d. Abort.\n", __func__, ret);
    }
    blueberry_tp_priv_debug("sysfs_create_group OK \n");
    blueberry_tp_priv_debug("%s() OK\n", __func__);

    return 0;

err_reg_rel:
    input_unregister_device(priv->abs_dev);
err_rel_input_free:
    if(priv->abs_dev) input_free_device(priv->abs_dev);
err_abs_input_free:
    if(priv->rel_dev) input_free_device(priv->rel_dev);
err_free_irq:
    free_irq(client->irq, priv);
    gpio_free(client->irq);
kfree_dev:
	if(priv) kfree(priv);

    return ret;
}

static int blueberry_tp_synaptics_remove(struct i2c_client *client)
{
	struct synaptics_data *priv;

    priv = i2c_get_clientdata(client);

    blueberry_tp_priv_debug("enter blueberry_tp_synaptics_remove.\n");
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&priv->early_drv);
#endif
    sysfs_remove_group(&client->dev.kobj, &blueberry_tp_synaptics_attribute_group);

    input_unregister_device(priv->abs_dev);
    input_unregister_device(priv->rel_dev);

    cancel_work_sync(&priv->input_work);
    free_irq(client->irq, priv);

    kfree(priv);
    pr_info("blueberry_tp successfully removed.");

    return 0;
}

static const struct i2c_device_id blueberry_tp_synaptics_i2c_id[] = {
    {"a10_tp_syn", 0},
    { }
};

static struct i2c_driver blueberry_tp_synaptics_driver = {
    .probe      = blueberry_tp_synaptics_probe,
    .remove     = __devexit_p(blueberry_tp_synaptics_remove),
    .suspend    = blueberry_tp_synaptics_suspend,
    .resume     = blueberry_tp_synaptics_resume,
    .driver = {
        .owner  = THIS_MODULE,
        .name   = "a10_tp_syn",
    },
    .id_table   = blueberry_tp_synaptics_i2c_id,
};

static int __init blueberry_tp_synaptics_init(void)
{
    pr_info("enter blueberry_tp_synaptics_init.\n");
    return i2c_add_driver(&blueberry_tp_synaptics_driver);
}

static void __exit blueberry_tp_synaptics_exit(void)
{
    pr_info("enter blueberry_tp_synaptics_exit.\n");
    i2c_del_driver(&blueberry_tp_synaptics_driver);
}

module_init(blueberry_tp_synaptics_init);
module_exit(blueberry_tp_synaptics_exit);

MODULE_AUTHOR("Paul Ma");
MODULE_DESCRIPTION("blueberry Synaptics touchpad driver");
MODULE_LICENSE("GPL");
