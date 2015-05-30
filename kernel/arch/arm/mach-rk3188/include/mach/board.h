#ifndef __MACH_BOARD_H
#define __MACH_BOARD_H

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <asm/setup.h>
#include <plat/board.h>
#include <mach/sram.h>
#include <linux/i2c-gpio.h>


void __init rk30_map_common_io(void);
void __init rk30_init_irq(void);
void __init rk30_map_io(void);
struct machine_desc;
void __init rk30_fixup(struct machine_desc *desc, struct tag *tags, char **cmdline, struct meminfo *mi);
void __init rk30_clock_data_init(unsigned long gpll,unsigned long cpll,u32 flags);

#ifdef CONFIG_RK30_PWM_REGULATOR
void  rk30_pwm_suspend_voltage_set(void);
void  rk30_pwm_resume_voltage_set(void);
void __sramfunc rk30_pwm_logic_suspend_voltage(void);
 void __sramfunc rk30_pwm_logic_resume_voltage(void);
#endif

extern struct sys_timer rk30_timer;

#if defined (CONFIG_CHARGER_CW2015)
struct cw2015_platform_data{
	int dc_det_pin;
	int batt_low_pin;
	int dc_det_level;
	int batt_low_level;
};
#endif

#if defined(CONFIG_BATTERY_BLUEBERRY)
struct blueberry_bat_platform_data{
	int dc_det_pin;						/* AC insert detection pin, can wake up the machine */
	int bat_adc_channel;				/* battery insert detection pin, it's a adc pin */
	int bat_low_det_pin;				/* battery low/charge inh interrupt pin, can wake up the machine */
};
#endif

#if defined(CONFIG_TOUCHSCREEN_EKTF2K)
struct elan_ktf2k_i2c_platform_data {
	uint16_t	version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int reset_gpio;
};
#endif

enum _periph_pll {
	periph_pll_1485mhz = 148500000,
	periph_pll_297mhz = 297000000,
	periph_pll_300mhz = 300000000,
	periph_pll_384mhz = 384000000,
	periph_pll_768mhz = 768000000,
	periph_pll_594mhz = 594000000,
	periph_pll_1188mhz = 1188000000, /* for box*/
};
enum _codec_pll {
	codec_pll_360mhz = 360000000, /* for HDMI */
	codec_pll_408mhz = 408000000,
	codec_pll_456mhz = 456000000,
	codec_pll_504mhz = 504000000,
	codec_pll_552mhz = 552000000, /* for HDMI */
	codec_pll_594mhz = 594000000, /* for HDMI */
	codec_pll_600mhz = 600000000,
	codec_pll_742_5khz = 742500000,
	codec_pll_768mhz = 768000000,
	codec_pll_798mhz = 798000000,
	codec_pll_1188mhz = 1188000000,
	codec_pll_1200mhz = 1200000000,
};

struct bq27541_platform_data {
    int (*init_dc_check_pin)(void);
    unsigned int dc_check_pin;
    unsigned int bat_check_pin;
    unsigned int chgok_check_pin;
    unsigned int bat_num;
};

//has extern 27mhz
#define CLK_FLG_EXT_27MHZ 			(1<<0)
//max i2s rate
#define CLK_FLG_MAX_I2S_12288KHZ 	(1<<1)
#define CLK_FLG_MAX_I2S_22579_2KHZ 	(1<<2)
#define CLK_FLG_MAX_I2S_24576KHZ 	(1<<3)
#define CLK_FLG_MAX_I2S_49152KHZ 	(1<<4)
//uart 1m\3m
#define CLK_FLG_UART_1_3M			(1<<5)
#define CLK_CPU_HPCLK_11				(1<<6)


#ifdef CONFIG_RK29_VMAC

#define RK30_CLOCKS_DEFAULT_FLAGS (CLK_FLG_MAX_I2S_12288KHZ/*|CLK_FLG_EXT_27MHZ*/)
#define periph_pll_default periph_pll_300mhz
#define codec_pll_default codec_pll_1188mhz

#else


#define RK30_CLOCKS_DEFAULT_FLAGS (CLK_FLG_MAX_I2S_12288KHZ/*|CLK_FLG_EXT_27MHZ*/)

#define codec_pll_default codec_pll_594mhz
#define periph_pll_default periph_pll_768mhz

//#define codec_pll_default codec_pll_798mhz
//#define periph_pll_default periph_pll_594mhz

#endif






#endif
