/*
 * EC based touchpad driver, support single finger and multiple fingers.
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
#include <asm/atomic.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "blueberry_tp_i2c.h"
#include "blueberry_tp.h"

struct blueberry_tp {
	struct i2c_client *client;
	struct input_dev *abs_dev;
	struct input_dev *rel_dev;
	struct work_struct input_work;
	struct workqueue_struct *touchpad_wq;
	struct blueberry_tp_data *private;

#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif

	unsigned char packet[8];
	unsigned char pktsize;

	unsigned int enable;
	unsigned int mode;

	/* accel the mouse movement speed */
	unsigned int rel_threshold1;
	unsigned int rel_threshold2;
	unsigned int rel_pointer_speed;
	int sensitivity;
	int sensitivity_factor;
	int dx_remainder;
	int dy_remainder;	
};

static bool probe_ctrl_flag = false;
#define blueberry_tp_debug(fmt, ...)                   		\
    do {                                					\
        if (bbtd->debug)                     				\
            pr_info(pr_fmt(fmt), ##__VA_ARGS__);  			\
    } while (0)

/*
 * Dump a complete mouse movement packet to the syslog
 */
static void blueberry_tp_packet_dump(struct blueberry_tp *bbtp)
{
    int i;

    printk(KERN_ERR "blueberry_tp packet [");
    for (i = 0; i < bbtp->pktsize; i++)
        printk("%s0x%02x ", i ? ", " : " ", bbtp->packet[i]);
    printk("]\n");
}

static void blueberry_tp_input_sync_v4(struct blueberry_tp *bbtp)
{
    struct input_dev *dev = bbtp->abs_dev;
    unsigned char *packet = bbtp->packet;

    input_report_key(dev, BTN_LEFT, packet[0] & 0x01);
    input_mt_report_pointer_emulation(dev, true);
    input_sync(dev);
}

static void blueberry_tp_process_packet_status_v4(struct blueberry_tp *bbtp)
{
    struct input_dev *dev = bbtp->abs_dev;
    unsigned char *packet = bbtp->packet;
    unsigned fingers;
    int i;

    /* notify finger state change */
    fingers = packet[1] & 0x1f;
    for (i = 0; i < ETP_MAX_FINGERS; i++) {
        if ((fingers & (1 << i)) == 0) {
            input_mt_slot(dev, i);
            input_mt_report_slot_state(dev, MT_TOOL_FINGER, false);
        }
    }

    blueberry_tp_input_sync_v4(bbtp);
}

static void blueberry_tp_process_packet_head_v4(struct blueberry_tp *bbtp)
{
    struct input_dev *dev = bbtp->abs_dev;
    struct blueberry_tp_data *bbtd = bbtp->private;
    unsigned char *packet = bbtp->packet;
    int id = ((packet[3] & 0xe0) >> 5) - 1;
    int pres, traces;

    if (id < 0)
        return;

    bbtd->mt[id].x = ((packet[1] & 0x0f) << 8) | packet[2];
    bbtd->mt[id].y = bbtd->y_max - (((packet[4] & 0x0f) << 8) | packet[5]);
    pres = (packet[1] & 0xf0) | ((packet[4] & 0xf0) >> 4);
    traces = (packet[0] & 0xf0) >> 4;

    input_mt_slot(dev, id);
    input_mt_report_slot_state(dev, MT_TOOL_FINGER, true);

    input_report_abs(dev, ABS_MT_POSITION_X, bbtd->mt[id].x);
    input_report_abs(dev, ABS_MT_POSITION_Y, bbtd->mt[id].y);
    input_report_abs(dev, ABS_MT_PRESSURE, pres);
    input_report_abs(dev, ABS_MT_TOUCH_MAJOR, traces * bbtd->width);
    /* report this for backwards compatibility */
    input_report_abs(dev, ABS_TOOL_WIDTH, traces);

    blueberry_tp_input_sync_v4(bbtp);
}

static void blueberry_tp_process_packet_motion_v4(struct blueberry_tp *bbtp)
{
    struct input_dev *dev = bbtp->abs_dev;
    struct blueberry_tp_data *bbtd = bbtp->private;
    unsigned char *packet = bbtp->packet;
    int weight, delta_x1 = 0, delta_y1 = 0, delta_x2 = 0, delta_y2 = 0;
    int id, sid;

    id = ((packet[0] & 0xe0) >> 5) - 1;
    if (id < 0)
        return;

    sid = ((packet[3] & 0xe0) >> 5) - 1;
    weight = (packet[0] & 0x10) ? ETP_WEIGHT_VALUE : 1;
    /*
     * Motion packets give us the delta of x, y values of specific fingers,
     * but in two's complement. Let the compiler do the conversion for us.
     * Also _enlarge_ the numbers to int, in case of overflow.
     */
    delta_x1 = (signed char)packet[1];
    delta_y1 = (signed char)packet[2];
    delta_x2 = (signed char)packet[4];
    delta_y2 = (signed char)packet[5];

    bbtd->mt[id].x += delta_x1 * weight;
    bbtd->mt[id].y -= delta_y1 * weight;
    input_mt_slot(dev, id);
    input_report_abs(dev, ABS_MT_POSITION_X, bbtd->mt[id].x);
    input_report_abs(dev, ABS_MT_POSITION_Y, bbtd->mt[id].y);

    if (sid >= 0) {
        bbtd->mt[sid].x += delta_x2 * weight;
        bbtd->mt[sid].y -= delta_y2 * weight;
        input_mt_slot(dev, sid);
        input_report_abs(dev, ABS_MT_POSITION_X, bbtd->mt[sid].x);
        input_report_abs(dev, ABS_MT_POSITION_Y, bbtd->mt[sid].y);
    }

    blueberry_tp_input_sync_v4(bbtp);
}

static void blueberry_tp_report_absolute_v4(struct blueberry_tp *bbtp,
                    int packet_type)
{
    switch (packet_type) {
    case PACKET_V4_STATUS:
        blueberry_tp_process_packet_status_v4(bbtp);
        break;

    case PACKET_V4_HEAD:
        blueberry_tp_process_packet_head_v4(bbtp);
        break;

    case PACKET_V4_MOTION:
        blueberry_tp_process_packet_motion_v4(bbtp);
        break;

    case PACKET_UNKNOWN:
    default:
        /* impossible to get here */
        break;
    }
}

static int blueberry_tp_packet_check_v4(struct blueberry_tp *bbtp)
{
    unsigned char *packet = bbtp->packet;

    if ((packet[0] & 0x0c) == 0x04 &&
        (packet[3] & 0x1f) == 0x11)
        return PACKET_V4_HEAD;

    if ((packet[0] & 0x0c) == 0x04 &&
        (packet[3] & 0x1f) == 0x12)
        return PACKET_V4_MOTION;

    if ((packet[0] & 0x0c) == 0x04 &&
        (packet[3] & 0x1f) == 0x10)
        return PACKET_V4_STATUS;

    return PACKET_UNKNOWN;
}

#ifdef BLUEBERRY_TP_PROCESS_BYTE
/*
 * Process byte stream from mouse and handle complete packets
 */
static blueberry_tp_ret_t blueberry_tp_process_byte(struct blueberry_tp *bbtp)
{
    struct blueberry_tp_data *bbtd = bbtp->private;
    int packet_type;

    if (bbtp->pktcnt < bbtp->pktsize)
        return BBTP_GOOD_DATA;

    if (bbtd->debug > 1)
        blueberry_tp_packet_dump(bbtp);

    switch (bbtd->hw_version) {
    case 1:
    case 2:
    case 3:
	default:
		pr_err("blueberry_tp_process_byte(): do not support ver 1/2/3 elan protocol.\n");
        break;

    case 4:
        packet_type = blueberry_tp_packet_check_v4(bbtp);
        if (packet_type == PACKET_UNKNOWN)
            return BBTP_BAD_DATA;

        blueberry_tp_report_absolute_v4(bbtp, packet_type);
        break;
    }

    return BBTP_FULL_PACKET;
}
#endif

static int blueberry_tp_do_mouse_accel(struct blueberry_tp *bbtp, int delta)
{
	int new_delta = delta;

	if(bbtp->rel_pointer_speed != 0) {
		if(abs(delta) > bbtp->rel_threshold1) {
			new_delta *= 2;
		
			if((abs(delta) > bbtp->rel_threshold2) && (bbtp->rel_pointer_speed == 2))
				new_delta *= 2;
		}
	}

	return new_delta;
}

static void blueberry_tp_report_relative(struct blueberry_tp *bbtp)
{
	struct blueberry_tp_data *bbtd = bbtp->private;
	int x_sign, y_sign, left_btn, right_btn, delta_x, delta_y;
	int accel_delta_x, accel_delta_y;
	int numerator;
	unsigned char *packet = bbtp->packet;
	struct input_dev *dev = bbtp->rel_dev;

	x_sign = (packet[0] & X_SIGN_MASK) ? 1:0;
	y_sign = (packet[0] & Y_SIGN_MASK) ? 1:0;
	
	if(packet[2] == 0) {
		y_sign = 0;
	}

	left_btn = (packet[0] & LEFT_BTN_MASK) ? 1:0;
	right_btn = (packet[0] & RIGHT_BTN_MASK) ? 1:0;
	delta_x = x_sign ? (packet[1] | 0xffffff00) : packet[1];
	delta_y = y_sign ? (packet[2] | 0xffffff00) : packet[2];

	blueberry_tp_debug("0x%02x 0x%02x 0x%02x\n]", packet[0], packet[1], packet[2]);
//	blueberry_tp_debug("x_sign=0x%x, delta_x=0x%x, y_sign=0x%x, delta_y=0x%x, left_btn=0x%x, right_btn=0x%x\n", \
					x_sign, delta_x, y_sign, delta_y, left_btn, right_btn);

	delta_y = -1 * delta_y;
	
	/* accel mouse speed */
	//accel_delta_x = blueberry_tp_do_mouse_accel(bbtp, delta_x);
	accel_delta_x = blueberry_tp_do_mouse_accel(bbtp, delta_x);
	accel_delta_y = blueberry_tp_do_mouse_accel(bbtp, delta_y);

	if(bbtp->sensitivity != MOUSE_SENSITIVITY_DEFAULT) {
		if(accel_delta_x) {
			numerator = accel_delta_x * bbtp->sensitivity_factor + bbtp->dx_remainder;
			accel_delta_x = numerator / 256;
			bbtp->dx_remainder = numerator % 256;
			if((numerator < 0) && (bbtp->dx_remainder > 0)) {
				accel_delta_x ++;
				bbtp->dx_remainder -= 256;
			}
		}
		if(accel_delta_y) {
			numerator = accel_delta_y * bbtp->sensitivity_factor + bbtp->dy_remainder;
			accel_delta_y = numerator / 256;
			bbtp->dy_remainder = numerator % 256;
			if((numerator < 0) && (bbtp->dy_remainder > 0)) {
				accel_delta_y ++;
				bbtp->dy_remainder -=256;
			}
		}
	}		

	input_report_rel(dev, REL_X, accel_delta_x);  /* delta_x); */
	input_report_rel(dev, REL_Y, accel_delta_y);  /* delta_y); */
	input_report_key(dev, BTN_LEFT, left_btn);
	input_report_key(dev, BTN_RIGHT, right_btn);
	input_sync(dev);
}

/* 
 * 0x0d:0x01 -> enable touchpad, 0x0d:0x00 -> disable touchpad 
 */
static int blueberry_tp_enable(struct blueberry_tp *bbtp)
{
    struct blueberry_tp_data *bbtd = bbtp->private;
    int ret;
    unsigned char cmd[3] = {0x0d, 0x00, 0x01};

	ret = blueberry_tp_i2c_write_data(bbtp->client, cmd, 3);
	if(ret) {
		pr_err("blueberry_tp: enable touchpad failed.\n");
		return ret;
	}		

	blueberry_tp_debug("enable touchpad success.\n");

	enable_irq(bbtp->client->irq);
	bbtp->enable = BLUEBERRY_TP_ENABLE;

	return 0;
}

static int blueberry_tp_disable(struct blueberry_tp * bbtp)
{
    struct blueberry_tp_data *bbtd = bbtp->private;
    int ret;
    unsigned char cmd[3] = {0x0d, 0x00, 0x00};

	ret = blueberry_tp_i2c_write_data(bbtp->client, cmd, 3);
	if(ret) {
		pr_err("blueberry_tp: disable touchpad failed.\n");
		return ret;
	}

	blueberry_tp_debug("disable touchpad success.\n");
		
    disable_irq(bbtp->client->irq);
    bbtp->enable = BLUEBERRY_TP_DISABLE;

	return 0;
}

/*
 * Put the touchpad into relative mode
 */
static int blueberry_tp_set_relative_mode(struct blueberry_tp *bbtp)
{
    struct blueberry_tp_data *bbtd = bbtp->private;
    int ret = -1;
    unsigned char cmd[3] = {0x07, 0x00, 0x00};

    switch (bbtd->hw_version) {
    case 1:
    case 2:
    case 3:
    default:
        pr_err("blueberry do not support v1/2/3 elan tp hw.\n");
        break;

    case 4:
		ret = blueberry_tp_i2c_write_data(bbtp->client, cmd, 3);
		if(ret){
			pr_err("blueberry_tp: set touchpad to relative mode failed.\n");
			return ret;
		}
		blueberry_tp_debug("set touchpad to relative mode success.\n");
        break;
    }

    bbtp->mode = BLUEBERRY_TP_REL_MODE;

    return ret;
}

/*
 * Put the touchpad into absolute mode
 */
static int blueberry_tp_set_absolute_mode(struct blueberry_tp *bbtp)
{
    struct blueberry_tp_data *bbtd = bbtp->private;
    int ret = -1;
    unsigned char cmd[3] = {0x07, 0x00, 0x01};

    switch (bbtd->hw_version) {
    case 1:
    case 2:
    case 3:
    default:
        pr_err("blueberry do not support v1/2/3 elan tp hw.\n");
        break;

    case 4:
        ret = blueberry_tp_i2c_write_data(bbtp->client, cmd, 3);
        if(ret){
            pr_err("blueberry_tp: set touchpad to absolute mode failed.\n");
            return ret;
        }
        blueberry_tp_debug("set touchpad to absolute mode success.\n");
        break;
    }

    bbtp->mode = BLUEBERRY_TP_ABS_MODE;

    return 0;
}

#ifdef DO_QUERY_MYSELF
static int blueberry_tp_send_cmd(struct blueberry_tp *bbtp,
				  unsigned int cmd, unsigned char *param)
{
	int ret;

	/* for EC, FW_ID_QUERY=0x08, FW_VERSION_QUERY=0x09, .... */		
	ret = blueberry_tp_smbus_read_byte_block(bbtp->client,
								cmd+0x08, param, 3);
	if(ret) {
		pr_err("blueberry_tp_send_cmd failed, cmd=0x%d\n", cmd);
		return ret;
	}

	return 0;	
}
#endif

static int blueberry_tp_set_range(struct blueberry_tp *bbtp,
                  unsigned int *x_min, unsigned int *y_min,
                  unsigned int *x_max, unsigned int *y_max,
                  unsigned int *width)
{
    struct blueberry_tp_data *bbtd = bbtp->private;
    unsigned char param[3];
    unsigned char traces;
	unsigned char fwid[3] = {0x36, 0xb4, 0x0c};

    switch (bbtd->hw_version) {
    case 1:
    case 2:
    case 3:
	default:
		pr_err("blueberry_tp: we do not support elan v1/2/3 hw version.\n");
        break;

    case 4:
		/* use constant value */
        /* if (blueberry_tp_send_cmd(bbtp, ETP_FW_ID_QUERY, param))
            return -1;
		*/
		param[0] = fwid[0];
		param[1] = fwid[1];
		param[2] = fwid[2];

		blueberry_tp_debug("ETP_FW_ID_QUERY = 0x%x, 0x%x, 0x%x\n", param[0], param[1], param[2]);

        *x_max = (0x0f & param[0]) << 8 | param[1];
        *y_max = (0xf0 & param[0]) << 4 | param[2];
        traces = bbtd->capabilities[1];
        if ((traces < 2) || (traces > *x_max)) {
			pr_err("blueberry_tp: failed to calculate width. traces=0x%X, x_max=0x%x.\n", traces, *x_max);
            return -1;
		}

        *width = *x_max / (traces - 1);
        break;
    }

    return 0;
}

/*
 * (value from firmware) * 10 + 790 = dpi
 * we also have to convert dpi to dots/mm (*10/254 to avoid floating point)
 */
static unsigned int blueberry_tp_convert_res(unsigned int val)
{
    return (val * 10 + 790) * 10 / 254;
}

static int blueberry_tp_get_resolution_v4(struct blueberry_tp *bbtp,
                      unsigned int *x_res,
                      unsigned int *y_res)
{
    unsigned char param[3];
	unsigned char resolution[3] = {0x06, 0x24, 0x00};
	struct blueberry_tp_data *bbtd = bbtp->private;

	/* use constant value */
	/*
    if (blueberry_tp_send_cmd(bbtp, ETP_RESOLUTION_QUERY, param))
        return -1;
	*/
	param[0] = resolution[0];
	param[1] = resolution[1];
	param[2] = resolution[2];

	blueberry_tp_debug("ETP_RESOLUTION_QUERY=0x%x, 0x%x, 0x%x\n", param[0], param[1], param[2]);

    *x_res = blueberry_tp_convert_res(param[1] & 0x0f);
    *y_res = blueberry_tp_convert_res((param[1] & 0xf0) >> 4);

    return 0;
}

static int blueberry_tp_set_relative_input_params(struct blueberry_tp *bbtp)
{
	struct blueberry_tp_data *bbtd = bbtp->private;
	struct input_dev *dev = bbtp->rel_dev;

	blueberry_tp_debug("enter set_relative_input_params.\n");
	
    __set_bit(EV_REL, dev->evbit);
    __set_bit(REL_X, dev->relbit);
    __set_bit(REL_Y, dev->relbit);

	__set_bit(EV_KEY, dev->evbit);
    __set_bit(BTN_LEFT, dev->keybit);
    __set_bit(BTN_RIGHT, dev->keybit);

	return 0;
}

/*
 * Set the appropriate event bits for the input subsystem
 */
static int blueberry_tp_set_absolute_input_params(struct blueberry_tp *bbtp)
{
    struct input_dev *dev = bbtp->abs_dev;
    struct blueberry_tp_data *bbtd = bbtp->private;
    unsigned int x_min = 0, y_min = 0, x_max = 0, y_max = 0, width = 0;
    unsigned int x_res = 0, y_res = 0;

    if (blueberry_tp_set_range(bbtp, &x_min, &y_min, &x_max, &y_max, &width))
        return -1;

    __set_bit(INPUT_PROP_POINTER, dev->propbit);
    __set_bit(EV_KEY, dev->evbit);
    __set_bit(EV_ABS, dev->evbit);
    __clear_bit(EV_REL, dev->evbit);

    __set_bit(BTN_LEFT, dev->keybit);
    __set_bit(BTN_RIGHT, dev->keybit);

    __set_bit(BTN_TOUCH, dev->keybit);
    __set_bit(BTN_TOOL_FINGER, dev->keybit);
    __set_bit(BTN_TOOL_DOUBLETAP, dev->keybit);
    __set_bit(BTN_TOOL_TRIPLETAP, dev->keybit);

    switch (bbtd->hw_version) {
    case 1:
    case 2:
    case 3:
		pr_err("blueberry_tp: we do not support v1/2/3 hw.\n");
        break;
    case 4:
        if (blueberry_tp_get_resolution_v4(bbtp, &x_res, &y_res)) {
            /*
             * if query failed, print a warning and leave the values
             * zero to resemble synaptics.c behavior.
             */
            pr_err("blueberry_tp: couldn't query resolution data.\n");
        }
        /* v4 is clickpad, with only one button. */
        __set_bit(INPUT_PROP_BUTTONPAD, dev->propbit);
        __clear_bit(BTN_RIGHT, dev->keybit);
        __set_bit(BTN_TOOL_QUADTAP, dev->keybit);
        /* For X to recognize me as touchpad. */
        input_set_abs_params(dev, ABS_X, x_min, x_max, 0, 0);
        input_set_abs_params(dev, ABS_Y, y_min, y_max, 0, 0);
        input_abs_set_res(dev, ABS_X, x_res);
        input_abs_set_res(dev, ABS_Y, y_res);
        /*
         * range of pressure and width is the same as v2,
         * report ABS_PRESSURE, ABS_TOOL_WIDTH for compatibility.
         */
        input_set_abs_params(dev, ABS_PRESSURE, ETP_PMIN_V2,
                     ETP_PMAX_V2, 0, 0);
        input_set_abs_params(dev, ABS_TOOL_WIDTH, ETP_WMIN_V2,
                     ETP_WMAX_V2, 0, 0);
        /* Multitouch capable pad, up to 5 fingers. */
        input_mt_init_slots(dev, ETP_MAX_FINGERS); // magf , 0);
        input_set_abs_params(dev, ABS_MT_POSITION_X, x_min, x_max, 0, 0);
        input_set_abs_params(dev, ABS_MT_POSITION_Y, y_min, y_max, 0, 0);
        input_abs_set_res(dev, ABS_MT_POSITION_X, x_res);
        input_abs_set_res(dev, ABS_MT_POSITION_Y, y_res);
        input_set_abs_params(dev, ABS_MT_PRESSURE, ETP_PMIN_V2,
                     ETP_PMAX_V2, 0, 0);
        /*
         * The firmware reports how many trace lines the finger spans,
         * convert to surface unit as Protocol-B requires.
         */
        input_set_abs_params(dev, ABS_MT_TOUCH_MAJOR, 0,
                     ETP_WMAX_V2 * width, 0, 0);
        break;
    }

    bbtd->y_max = y_max;
    bbtd->width = width;

    return 0;
}

static void blueberry_tp_early_suspend(struct early_suspend *handler)
{
	struct blueberry_tp *bbtp = (struct blueberry_tp *) container_of(handler, struct blueberry_tp, early_drv);
	struct blueberry_tp_data *bbtd = bbtp->private;
	unsigned char cmd[3] = {0x0d, 0x00, 0x00};
	int ret;

	blueberry_tp_debug("enter blueberry_tp_early_suspend.\n");

	if(!bbtp){
		pr_err("blueberry_tp: early_suspend, bbtp is NULL\n");
		return;
	}

	/* 0x0d : 0x01 -- enable touchpad, 0x0d : 0x00 -- disable touchpad */
/*	if(bbtp->enable == BLUEBERRY_TP_ENABLE) {
		ret = blueberry_tp_i2c_write_data(bbtp->client, cmd, 3);	
		if(ret) {
			pr_err("blueberry_tp: disable touchpad failed.\n");
		}		 
		blueberry_tp_debug("before suspend, status enable, disable touchpad success.\n");
	}else{
		blueberry_tp_debug("before suspend, status disable, do nothing.\n");
	}
	*/
	flush_workqueue(bbtp->touchpad_wq);
	disable_irq(bbtp->client->irq);
    return;
}

static void blueberry_tp_early_resume(struct early_suspend *handler)
{
	struct blueberry_tp *bbtp = (struct blueberry_tp *) container_of(handler, struct blueberry_tp, early_drv);
	struct blueberry_tp_data *bbtd = bbtp->private;
	unsigned char cmd[3] = {0x0d, 0x00, 0x01};
	int ret;

    blueberry_tp_debug("enter blueberry_tp_early_resume.\n");

	if(!bbtp) {
		pr_err("blueberry_tp: early_resume, fatal error, bbtp is NULL\n");
		return;
	}

	blueberry_tp_debug("here, touchpad must be disabled.\n");

	/* 0x0d : 0x01 -- enable touchpad, 0x0d : 0x00 -- disable touchpad */
/*	if(bbtp->enable == BLUEBERRY_TP_ENABLE) {
		ret = blueberry_tp_i2c_write_data(bbtp->client, cmd, 3);
		if(ret) {
			pr_err("blueberry_tp: early_resume, fatal error, enable failed.\n");
			return;
		}
		blueberry_tp_debug("when suspend, status enable, enable it success.\n");
	}else{
		blueberry_tp_debug("when suspend, status disable, do nothing.\n");
	}
*/	
	enable_irq(bbtp->client->irq);

	return;
}

static irqreturn_t blueberry_tp_interrupt(int irq, void *dev_id)
{
	struct blueberry_tp *bbtp = dev_id;
	
	disable_irq_nosync(bbtp->client->irq);
	//schedule_work(&bbtp->input_work);
	queue_work(bbtp->touchpad_wq, &bbtp->input_work);

	return IRQ_HANDLED;
}

static void blueberry_tp_input_work(struct work_struct *work)
{
	struct blueberry_tp *bbtp = (struct blueberry_tp *)container_of(work, struct blueberry_tp, input_work);
	struct blueberry_tp_data *bbtd = bbtp->private;
	int packet_type;
	int ret;

	/* 0x03 is read 6bytes packet command, 0x08 is packet array size, 6 is multi-touch packet len */
	memset(bbtp->packet, 0, 0x08);

	if(bbtp->mode == BLUEBERRY_TP_ABS_MODE) {

		ret = blueberry_tp_smbus_read_byte_block(bbtp->client, 0x03, bbtp->packet, 6);
		if(ret) {
			pr_err("blueberry_tp: read 6 bytes packet failed.\n");
			goto enable_irq;
		}
		if(bbtd->debug)
			blueberry_tp_packet_dump(bbtp);

    	packet_type = blueberry_tp_packet_check_v4(bbtp);
    	if (packet_type == PACKET_UNKNOWN){
			pr_err("blueberry_tp: unknown packet, trasfer error!!!\n");
			pr_err("blueberry_tp: maybe we need let EC sync the packet.\n");
			goto enable_irq;
		}       

    	blueberry_tp_report_absolute_v4(bbtp, packet_type);	
	}else{
		
		ret = blueberry_tp_smbus_read_byte_block(bbtp->client, 0x03, bbtp->packet, 3);
		if(ret) {
			pr_err("blueberry_tp: read 3 bytes packet failed.\n");
			goto enable_irq;
		}
	
		blueberry_tp_report_relative(bbtp);
	}
	
enable_irq:
	enable_irq(bbtp->client->irq);
	return;
}

static ssize_t blueberry_tp_enable_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
	struct blueberry_tp_data *bbtd = bbtp->private;

    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    if (*buf == 'E') {

        blueberry_tp_debug("user enable touchpad by sysfs.\n");
		if(bbtp->enable == BLUEBERRY_TP_DISABLE)
        	blueberry_tp_enable(bbtp);

    }else if(*buf == 'D') {

        blueberry_tp_debug("user disable touchpad by sysfs.\n");
		if(bbtp->enable == BLUEBERRY_TP_ENABLE)
        	blueberry_tp_disable(bbtp);

    }

    return len;
};

static ssize_t blueberry_tp_enable_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
	struct blueberry_tp_data *bbtd = bbtp->private;

    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", bbtp->enable > 0 ? 1 : 0);
}

static ssize_t blueberry_tp_mode_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
	struct blueberry_tp_data *bbtd = bbtp->private;

    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    if (*buf == 'M') {

        blueberry_tp_debug("user switch to mutil-touch mode by sysfs.\n");
        //blueberry_tp_disable(bbtp);
        //msleep(1);
        blueberry_tp_set_absolute_mode(bbtp);
        //msleep(1);
        //blueberry_tp_enable(bbtp);

    }else if(*buf == 'S') {

        blueberry_tp_debug("user switch to single-touch mode by sysfs.\n");
        //blueberry_tp_disable(bbtp);
        //msleep(1);
        blueberry_tp_set_relative_mode(bbtp);
        //msleep(1);
        //blueberry_tp_enable(bbtp);

    }

    return len;
};

static ssize_t blueberry_tp_mode_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
	struct blueberry_tp_data *bbtd = bbtp->private;

    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", bbtp->mode > 0 ? 1 : 0);
}

static ssize_t blueberry_tp_rel_threshold1_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
    struct blueberry_tp_data *bbtd = bbtp->private;
	unsigned int thres;

    blueberry_tp_debug("enter %s\n",__FUNCTION__);
	
	thres = simple_strtol(buf, NULL, 0);
	if(thres > 15) {
		thres = 15;
	}
	blueberry_tp_debug("user set threshold1 = %d\n", thres);
	bbtp->rel_threshold1 = thres;

    return len;
};

static ssize_t blueberry_tp_rel_threshold1_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
    struct blueberry_tp_data *bbtd = bbtp->private;

    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", bbtp->rel_threshold1);
}

static ssize_t blueberry_tp_rel_threshold2_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
    struct blueberry_tp_data *bbtd = bbtp->private;
    unsigned int thres;

    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    thres = simple_strtol(buf, NULL, 0);
    if(thres > 15) {
        thres = 15;
    }
    blueberry_tp_debug("user set threshold2 = %d\n", thres);
    bbtp->rel_threshold2 = thres;

    return len;
};

static ssize_t blueberry_tp_rel_threshold2_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
    struct blueberry_tp_data *bbtd = bbtp->private;
    
    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", bbtp->rel_threshold2);
}

static ssize_t blueberry_tp_rel_speed_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
    struct blueberry_tp_data *bbtd = bbtp->private;
    unsigned int speed;

    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    speed = simple_strtol(buf, NULL, 0);
    if(speed > 2) {
        speed = 2;
    }
    blueberry_tp_debug("user set speed = %d\n", speed);
    bbtp->rel_pointer_speed = speed;

    return len;
};

static ssize_t blueberry_tp_rel_speed_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
    struct blueberry_tp_data *bbtd = bbtp->private;
    
    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", bbtp->rel_pointer_speed);
}

static ssize_t blueberry_tp_debug_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
    struct blueberry_tp_data *bbtd = bbtp->private;

	if(buf[0] == 'E') {
		pr_info("User enable debug by sysfs.\n");
		bbtd->debug = 1;
	}else if(buf[0] == 'D') {
		pr_info("User disable debug by sysfs.\n");
		bbtd->debug = 0;
	}

    return len;
};

static ssize_t blueberry_tp_debug_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
    struct blueberry_tp_data *bbtd = bbtp->private;

    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", bbtd->debug);
}

static int calculate_sensitivity_factor(int sensitivity);
static ssize_t blueberry_tp_sensitivity_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
    struct blueberry_tp_data *bbtd = bbtp->private;
    unsigned int sensitivity;

    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    sensitivity = simple_strtol(buf, NULL, 0);
    if(sensitivity > 20) {
        sensitivity = 20;
    }
	if(sensitivity < 1) {
		sensitivity = 1;
	}
    blueberry_tp_debug("user set sensitivity = %d\n", sensitivity);
    bbtp->sensitivity = sensitivity;
	bbtp->sensitivity_factor = calculate_sensitivity_factor(bbtp->sensitivity);
	blueberry_tp_debug("sensitivity_fator = %d\n", bbtp->sensitivity_factor);

    return len;
};

static ssize_t blueberry_tp_sensitivity_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
    struct blueberry_tp_data *bbtd = bbtp->private;

    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "%d\n", bbtp->sensitivity);
}

static ssize_t blueberry_tp_vendor_id_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct blueberry_tp *bbtp = i2c_get_clientdata(client);
    struct blueberry_tp_data *bbtd = bbtp->private;

    blueberry_tp_debug("enter %s\n",__FUNCTION__);

    return sprintf(buf, "0x%x", 0x01);
}

static DEVICE_ATTR(vendor_id, 0660, blueberry_tp_vendor_id_show, NULL);
static DEVICE_ATTR(enable, 0660, blueberry_tp_enable_show, blueberry_tp_enable_store);
static DEVICE_ATTR(mode, 0660, blueberry_tp_mode_show, blueberry_tp_mode_store);
static DEVICE_ATTR(rel_threshold1, 0660, blueberry_tp_rel_threshold1_show, blueberry_tp_rel_threshold1_store);
static DEVICE_ATTR(rel_threshold2, 0660, blueberry_tp_rel_threshold2_show, blueberry_tp_rel_threshold2_store);
static DEVICE_ATTR(rel_speed, 0660, blueberry_tp_rel_speed_show, blueberry_tp_rel_speed_store);
static DEVICE_ATTR(debug, 0660, blueberry_tp_debug_show, blueberry_tp_debug_store);
static DEVICE_ATTR(sensitivity, 0660, blueberry_tp_sensitivity_show, blueberry_tp_sensitivity_store);

static struct attribute *blueberry_tp_attributes[] = {
    &dev_attr_vendor_id.attr,
    &dev_attr_mode.attr,
    &dev_attr_enable.attr,
	&dev_attr_rel_threshold1.attr,
	&dev_attr_rel_threshold2.attr,
	&dev_attr_rel_speed.attr,
	&dev_attr_debug.attr,
	&dev_attr_sensitivity.attr,
    NULL
};

static struct attribute_group blueberry_tp_attribute_group = {
        .attrs = blueberry_tp_attributes
};

static int blueberry_tp_resume(struct i2c_client *client)
{
    /* struct blueberry_tp *bbtp = i2c_get_clientdata(client); */
    printk("%s() enter, enable touchpad power\n", __func__);
    gpio_set_value(RK30_PIN1_PB5, GPIO_HIGH);
    return 0;
}

static int blueberry_tp_suspend(struct i2c_client *client, pm_message_t mesg)
{
    /* struct blueberry_tp *bbtp = i2c_get_clientdata(client); */
    printk("%s() enter, disable touchpad power\n", __func__);
    gpio_set_value(RK30_PIN1_PB5, GPIO_LOW);
    return 0;
}

static int blueberry_tp_who_am_i(struct i2c_client *client)
{
    int ret;
    unsigned char id;   /* 0x01 -- elan, 0x02 -- synaptics */

    ret = blueberry_tp_smbus_read_byte(client, 0x0e, &id);
    if(ret) {
        pr_err("error while read touchpad id, exit.\n");
        return ret;
    }
    pr_info("blueberry_tp_who_am_i: id=0x%x (%s).\n", id, id==1 ? "elan":"synaptics");

	//default elan
	if(id == 0x00)
		return 0;

    if(id != 0x01)
        return -1;

    return 0;
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


static int blueberry_tp_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct blueberry_tp *bbtp;
	struct blueberry_tp_data *bbtd;
	unsigned char param[3];
	unsigned char ver;
	int ret;
	unsigned char version[3]={0x48, 0x1f, 0x00};
	unsigned char capability[3]={0x41, 0x0c, 0x06};
	
	bbtp = NULL;
	bbtd = NULL;

	pr_info("enter blueberry_tp_probe\n");
if(!probe_ctrl_flag){

	if(blueberry_tp_who_am_i(client))
		return -ENODEV;

	bbtp = kzalloc(sizeof(*bbtp), GFP_KERNEL);
	if(!bbtp) {
		pr_err("blueberry_tp: can't alloc bbtp, no memory.\n");
		ret = -ENOMEM;
		goto kfree_dev;
	}
	
	bbtd = kzalloc(sizeof(*bbtd), GFP_KERNEL);
	if(!bbtd) {
		pr_err("blueberry_tp: can't alloc bbtd, no memory.\n");
		ret = -ENOMEM;
		goto kfree_dev;
	}

	bbtp->touchpad_wq = create_singlethread_workqueue("touchpad_wq");
	if(!(bbtp->touchpad_wq))
	{
		ret = -ENOMEM;
		goto kfree_dev;
	}

	bbtp->private = bbtd;
	bbtp->client = client;
	bbtp->pktsize = 6;
	
	bbtd->debug = 0;
	bbtd->hw_version = 4;

	/* 6/10/1 is a good combination for synaptics. 0/5/2 is a good 
		combination for elan. */
	bbtp->rel_threshold1 = 0; //6;
	bbtp->rel_threshold2 = 10;
	bbtp->rel_pointer_speed = 0;

	bbtp->sensitivity = 10;
	bbtp->sensitivity_factor = 256;
	bbtp->dx_remainder = 0;
	bbtp->dy_remainder = 0;
	bbtp->sensitivity_factor = calculate_sensitivity_factor(bbtp->sensitivity);
	pr_err("calculated sensitivity_factor=%d from sensitivity=%d\n", bbtp->sensitivity_factor, bbtp->sensitivity);
	
	bbtp->enable = BLUEBERRY_TP_ENABLE;
	bbtp->mode = BLUEBERRY_TP_REL_MODE;

	INIT_WORK(&bbtp->input_work, blueberry_tp_input_work);

	/* use constant value */
	/* ret = blueberry_tp_send_cmd(bbtp, ETP_FW_VERSION_QUERY, param); */
	ret = 0;
	param[0] = version[0];
	param[1] = version[1];
	param[2] = version[2];
	if(ret) {
		pr_err("blueberry_tp: ETP_FW_VERSION_QUERY failed.\n");
		ret = -ENODEV;
		goto kfree_dev;
	}else{
		bbtd->fw_version = (param[0] << 16) | (param[1] << 8) | param[2];
		blueberry_tp_debug("param[0]=0x%x, param[1]=0x%x, param[2]=0x%x\n", param[0], param[1], param[2]);
		blueberry_tp_debug("bbtd fw_version=0x%x\n",bbtd->fw_version);
	}
	ver = (bbtd->fw_version & 0x0f0000) >> 16;
	if(ver < 6) {
		pr_err("we only support v4 protocol for blueberry tp.\n");
		ret = -ENODEV;
		goto kfree_dev;
	}

	/* use constant value */
	/* ret = blueberry_tp_send_cmd(bbtp, ETP_CAPABILITIES_QUERY, bbtd->capabilities); */
	ret = 0;
	bbtd->capabilities[0] = capability[0];
	bbtd->capabilities[1] = capability[1];
	bbtd->capabilities[2] = capability[2];
	if(ret) {
		pr_err("blueberry_tp: ETP_FW_CAPABILITIES_QUERY failed.\n");
		ret = -ENODEV;
		goto kfree_dev;
	}else{
		blueberry_tp_debug("cap[0]=0x%x, cap[1]=0x%x, cap[2]=0x%x\n", \
				 bbtd->capabilities[0], bbtd->capabilities[1], bbtd->capabilities[2]);
	}

    i2c_set_clientdata(client, bbtp);
/*
	ret = blueberry_tp_set_absolute_mode(bbtp);
	if(ret) {
		pr_err("blueberry_tp set_absolute mode failed.\n");
		ret = -ENODEV;
		goto kfree_dev;
	}
	pr_info("blueberry_tp now in absolute mode.\n");
*/

    /* the following code setup interrupt */
    ret = gpio_request(client->irq, "bbtp_irq");
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

    ret = request_irq(client->irq, blueberry_tp_interrupt, IRQ_TYPE_LEVEL_LOW, "bbtp_irq", bbtp);
    if(ret){
        pr_err("blueberry_tp: request irq failed\n");
        goto kfree_dev;
    }

	/* alloc abs input device and set its params */
	bbtp->abs_dev = input_allocate_device();
	if(!bbtp->abs_dev) {
		ret = -ENOMEM;
		goto err_free_irq;
	}
	blueberry_tp_debug("abs input device allocated.\n");
	bbtp->abs_dev->name = "blueberry_tp_abs";
	bbtp->abs_dev->phys = bbtp->client->adapter->name;
	bbtp->abs_dev->id.bustype = BUS_I2C;
	bbtp->abs_dev->dev.parent = &client->dev;	

	ret = blueberry_tp_set_absolute_input_params(bbtp);
	if(ret){
		pr_err("blueberry_tp set absolute input params failed.\n");
		ret = -ENODEV;
		goto err_abs_input_free;
	}
	blueberry_tp_debug("set_absolute_input_params succ.\n");

	/* alloc rel input device and set its params */
	bbtp->rel_dev = input_allocate_device();
	if(!bbtp->rel_dev) {
		ret = -ENOMEM;
		goto err_abs_input_free;
	}
	blueberry_tp_debug("rel input device allocated.\n");
	bbtp->rel_dev->name = "blueberry_tp_rel";
	bbtp->rel_dev->phys = bbtp->client->adapter->name;
	bbtp->rel_dev->id.bustype = BUS_I2C;
	bbtp->rel_dev->dev.parent = &client->dev;

	ret = blueberry_tp_set_relative_input_params(bbtp);
	if(ret) {
		pr_err("blueberry_tp set relative input params failed.\n");
		ret = -ENODEV;
		goto err_rel_input_free;
	}

	/* register both abs and rel input device */
	ret = input_register_device(bbtp->abs_dev);
	if(ret){
		pr_err("blueberry_tp input_register_abs_device failed.\n");
		goto err_rel_input_free;
	}
	blueberry_tp_debug("input_register_abs_device succ.\n");

	ret = input_register_device(bbtp->rel_dev);
	if(ret) {
		pr_err("blueberry_tp input_register_rel_device failed.\n");
		goto err_reg_rel;
	}
	blueberry_tp_debug("input_register_rel_device succ.\n");

#if defined(CONFIG_HAS_EARLYSUSPEND)
    bbtp->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
    bbtp->early_drv.suspend  = blueberry_tp_early_suspend,
    bbtp->early_drv.resume   = blueberry_tp_early_resume,
    register_early_suspend(&bbtp->early_drv);
#endif

    ret = sysfs_create_group(&client->dev.kobj, &blueberry_tp_attribute_group);
    if (ret) {
        pr_err("%s: sysfs_create_group returned err = %d. Abort.\n", __func__, ret);
    }
    blueberry_tp_debug("sysfs_create_group OK \n");
    blueberry_tp_debug("%s() OK\n", __func__);
}
else{
	return -ENODEV;
}
probe_ctrl_flag = true;

	return 0;

err_reg_rel:
	input_unregister_device(bbtp->abs_dev);
	//goto err_abs_input_free;
err_rel_input_free:
	if(bbtp->abs_dev) input_free_device(bbtp->abs_dev);
err_abs_input_free:
	if(bbtp->rel_dev) input_free_device(bbtp->rel_dev);
err_free_irq:
	free_irq(client->irq, bbtp);
	gpio_free(client->irq);
kfree_dev:
	if(bbtp) kfree(bbtp);
	if(bbtd) kfree(bbtd);

	return ret;	
}

static int blueberry_tp_remove(struct i2c_client *client)
{
	probe_ctrl_flag = false;;
	struct blueberry_tp *bbtp;
	struct blueberry_tp_data *bbtd;

	bbtp = i2c_get_clientdata(client);
	bbtd = bbtp->private;

    blueberry_tp_debug("enter blueberry_tp_remove.\n");
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&bbtp->early_drv);
#endif
    sysfs_remove_group(&client->dev.kobj, &blueberry_tp_attribute_group);

    input_unregister_device(bbtp->abs_dev); 
	input_unregister_device(bbtp->rel_dev);

    cancel_work_sync(&bbtp->input_work);

	flush_workqueue(bbtp->touchpad_wq);
	destroy_workqueue(bbtp->touchpad_wq);

	free_irq(client->irq, bbtp);

    kfree(bbtp);
	kfree(bbtd);
    pr_info("blueberry_tp successfully removed.");

    return 0;
}

static const struct i2c_device_id blueberry_tp_i2c_id[] = {
    {"blueberry_tp", 0},
    { }
};

static struct i2c_driver blueberry_tp_driver = {
    .probe      = blueberry_tp_probe,
    .remove     = __devexit_p(blueberry_tp_remove),
	.suspend	= blueberry_tp_suspend,
	.resume		= blueberry_tp_resume,
    .driver = {
        .owner  = THIS_MODULE,
        .name   = "blueberry_tp",
    },
    .id_table   = blueberry_tp_i2c_id,
};

static int __init blueberry_tp_init(void)
{
    pr_info("enter blueberry_tp_init.\n");
    return i2c_add_driver(&blueberry_tp_driver);
}

static void __exit blueberry_tp_exit(void)
{
    pr_info("enter blueberry_tp_exit.\n");
    i2c_del_driver(&blueberry_tp_driver);
}

module_init(blueberry_tp_init);
module_exit(blueberry_tp_exit);

MODULE_AUTHOR("Paul Ma");
MODULE_DESCRIPTION("blueberry touchpad driver");
MODULE_LICENSE("GPL");

