/*
 * BlueBerry touchpad driver
 *
 * Copyright (C) 2013-2013 Paul Ma <magf@bitland.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#ifndef _BLUEBERRY_TP_H
#define _BLUEBERRY_TP_H

/*
 * Command values for Synaptics style queries
 */
#define ETP_FW_ID_QUERY			0x00
#define ETP_FW_VERSION_QUERY		0x01
#define ETP_CAPABILITIES_QUERY		0x02
#define ETP_SAMPLE_QUERY		0x03
#define ETP_RESOLUTION_QUERY		0x04

#define ETP_PMIN_V2			0
#define ETP_PMAX_V2			255
#define ETP_WMIN_V2			0
#define ETP_WMAX_V2			15

/*
 * v4 hardware has 3 kind of packet.
 */
#define PACKET_UNKNOWN			0x01
#define PACKET_V4_HEAD			0x05
#define PACKET_V4_MOTION		0x06
#define PACKET_V4_STATUS		0x07

/*
 * track up to 5 fingers for v4 hardware
 */
#define ETP_MAX_FINGERS			5

/*
 * weight value for v4 hardware
 */
#define ETP_WEIGHT_VALUE		5

/*
 * tp abs and rel mode switch and enable 
 * and disable definition.
 */
#define BLUEBERRY_TP_ABS_MODE   1
#define BLUEBERRY_TP_REL_MODE   0
#define BLUEBERRY_TP_ENABLE     1
#define BLUEBERRY_TP_DISABLE    0

/*
 * for touchpad relative mode
 */
/* relative mode packet formate */
#define Y_OVERFLOW_MASK         0x80
#define X_OVERFLOW_MASK         0x40
#define Y_SIGN_MASK             0x20
#define X_SIGN_MASK             0x10
#define RIGHT_BTN_MASK          0x2
#define LEFT_BTN_MASK           0x1

#define MOUSE_SENSITIVITY_MIN 1
#define MOUSE_SENSITIVITY_DEFAULT 10
#define MOUSE_SENSITIVITY_MAX 20

/*
 * return value for process_byte
 */
typedef enum {
	BBTP_BAD_DATA,
	BBTP_GOOD_DATA,
	BBTP_FULL_PACKET
}blueberry_tp_ret_t;

/*
 * The base position for one finger, v4 hardware
 */
struct finger_pos {
	unsigned int x;
	unsigned int y;
};

struct blueberry_tp_data {
	unsigned char reg_07;
	unsigned char debug;
	unsigned char capabilities[3];
	unsigned char hw_version;
	unsigned int fw_version;
	unsigned int y_max;
	unsigned int width;
	struct finger_pos mt[ETP_MAX_FINGERS];
};

#endif
