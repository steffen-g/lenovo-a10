#include <linux/fb.h>
#include <linux/delay.h>
#include "../../rk29_fb.h"
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include "screen.h"

#include <linux/board-id.h>

#define HD_SCREEN_SIZE 1920UL*1200UL*4*3

struct bid_lcd_size_data{
	int type;
	int bid;
	char *name;
	int h_vd;
	int v_vd;
};


static struct bid_lcd_size_data lcd_size[] = 
{
	{DEVICE_TYPE_LCD, LCD_ID_NULL, "no_lcd", 1024, 768},
	{DEVICE_TYPE_LCD, LCD_ID_IVO_M101_NWN8, "IVO_M101_NWN8", 1368, 768},	
	{DEVICE_TYPE_LCD, LCD_ID_EDID_I2C, "lcd_edid_i2c", 1368, 768},
		
};


static char vendor0block[512] = {0,};
extern void set_lcd_info_ivo_m101nwn8(struct rk29fb_screen *screen, struct rk29lcd_info *lcd_info );
extern void set_lcd_info_edid_i2c(struct rk29fb_screen *screen, struct rk29lcd_info *lcd_info );

void set_lcd_info(struct rk29fb_screen *screen, struct rk29lcd_info *lcd_info )
{
	unsigned char lcd_id = board_id_get(DEVICE_TYPE_LCD);
	if (lcd_id > 0) {
		if (lcd_id == LCD_ID_IVO_M101_NWN8)
			return set_lcd_info_ivo_m101nwn8(screen, lcd_info);

		if (lcd_id == LCD_ID_EDID_I2C)
			return set_lcd_info_edid_i2c(screen, lcd_info);
	}
	
	return set_lcd_info_ivo_m101nwn8(screen, lcd_info);
}


size_t get_fb_size_from_bid(int lcd_id)
{	
	size_t size = 0;

	if(lcd_id >= ARRAY_SIZE(lcd_size))
	size = HD_SCREEN_SIZE;
	else
	{
#if defined(CONFIG_THREE_FB_BUFFER)
		size = ((lcd_size[lcd_id].h_vd)*(lcd_size[lcd_id].v_vd)<<2)* 3; //three buffer
#else
		size = ((lcd_size[lcd_id].h_vd)*(lcd_size[lcd_id].v_vd)<<2)<<1; //two buffer
#endif
	}
	return ALIGN(size,SZ_1M);
}


size_t get_fb_size(void)
{
	int ret = 0;
	unsigned char lcd_id = 0;
	
	board_id_get_from_flash(vendor0block, BID_MM_IOMEM);	
	lcd_id = vendor0block[DEVICE_TYPE_LCD];
	
	return get_fb_size_from_bid(lcd_id);		
}


