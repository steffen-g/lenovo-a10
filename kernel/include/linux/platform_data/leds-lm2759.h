/*
* Copyright (C) 2012 Texas Instruments
*
* License Terms: GNU General Public License v2
*
* Simple driver for Texas Instruments LM3642 LED driver chip
*
* Author: G.Shark Jeong <gshark.jeong@gmail.com>
*         Daniel Jeong <daniel.jeong@ti.com>
*/

#ifndef __LINUX_LM3642_H
#define __LINUX_LM3642_H

int leds_flash_torch(int mode);
enum lm2759_torch_pin_enable {
	LM3642_TORCH_PIN_DISABLE = 0x00,
	LM3642_TORCH_PIN_ENABLE = 0x10,
};

enum lm2759_strobe_pin_enable {
	LM3642_STROBE_PIN_DISABLE = 0x00,
	LM3642_STROBE_PIN_ENABLE = 0x20,
};

enum lm2759_tx_pin_enable {
	LM3642_TX_PIN_DISABLE = 0x00,
	LM3642_TX_PIN_ENABLE = 0x40,
};

struct lm2759_platform_data {
	enum lm2759_torch_pin_enable torch_pin;
	enum lm2759_strobe_pin_enable strobe_pin;
	enum lm2759_tx_pin_enable tx_pin;
	unsigned int torch_current_level;
	unsigned int flash_current_level;

	unsigned int strobe_active; //active level
	unsigned int torch_active;
	unsigned int gpio_strobe;
	unsigned int gpio_torch;
};

#endif /* __LINUX_LM3642_H */

