#ifndef __LINUX_EKTF2k_H
#define __LINUX_EKTF2k_H

#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>

#define TOUCH_MAX_HEIGHT  2496
#define TOUCH_MAX_WIDTH   1472

#define SCREEN_MAX_HEIGHT 1368
#define SCREEN_MAX_WIDTH  768

#define EKTF2K_MULTI_TOUCH 10
#define EKTF2K_TEN_FINGER 0x62

struct ektf2k_ts_data {
    struct i2c_client *client;
    struct input_dev *input_dev;

    unsigned irq_pin;
    unsigned reset_pin;
	
	unsigned irq;

	struct workqueue_struct *ektf2k_wq;
    struct work_struct work;
    
    struct early_suspend early_suspend;
    int (*power)(struct ektf2k_ts_data *ts, int on);

	/*For Firmware update*/
	int fw_ver;
	int fw_id;
	struct miscdevice firmware;
};

struct ektf2k_i2c_platform_data {
    unsigned reset_pin;
    unsigned irq_pin;
};

#endif
