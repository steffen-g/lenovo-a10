/***
    Copyright by Bitland Information Technology Co.,Ltd for V80 project, 2013.2
    Author: Paul Ma, magf@bitland.com.cn
***/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/kthread.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include "rpr400.h"

#define rpr400_DEV_NAME          "rpr400"
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)                

static struct PS_ALS_DATA *g_rpr400_ptr = NULL;

typedef struct {
    unsigned long long data;
    unsigned long long data0;
    unsigned long long data1;
    unsigned char      gain_data0;
    unsigned char      gain_data1;
    unsigned long      dev_unit;
    unsigned char      als_time;
    unsigned short     als_data0;
    unsigned short     als_data1;
} CALC_DATA;

typedef struct {
    unsigned long positive;
    unsigned long decimal;
} CALC_ANS;

typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

struct rpr400_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr; 
    u8  ps_thd;     /*PS INT threshold*/
};

/* we must provide als_level_num, als_value_num, als_level, als_value.
   Orignal design use:
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
   why do they have different size?
*/
#define C_CUST_ALS_LEVEL 10
#define ALS_LEVEL_NUM    C_CUST_ALS_LEVEL  /* ALS_LEVEL_NUM and ALS_VALUE_NUM now must have the save count */ 
#define ALS_VALUE_NUM    C_CUST_ALS_LEVEL  /* ALS_LEVEL_NUM and ALS_VALUE_NUM now must have the same count */
static u32 g_als_level[ALS_LEVEL_NUM] = {40, 70, 100, 150, 250, 350, 450, 600, 750, 1000};
static u32 g_als_value[ALS_VALUE_NUM] = {0,   1,   2,   3,   4,   5,   6,   7,   8,    9};

#define PS_THD_VALUE     40

/* if you want to test als and ps working in kernel, 
   enable the following definition, otherwise comment it.
   in working env, please comment it.
 */
/* #define AUTO_TEST_MODE */

#ifndef AUTO_TEST_MODE
#define PS_POLLING_MODE  0
#else
#define PS_POLLING_MODE  1
#endif


struct PS_ALS_DATA {
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct rpr400_i2c_addr  addr;
   
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;		/* this field should be removed */
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away, this field should be removed*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;

    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[ALS_LEVEL_NUM];
    u32         als_value[ALS_VALUE_NUM];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif    

	u32					polling_mode_ps;
	struct input_dev	*input;
};

static int rpr400_enable_als(struct i2c_client *client, int enable)
{
	struct PS_ALS_DATA *obj = i2c_get_clientdata(client);
	u8 databuf[2];	 
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];	
	u8 power_state, power_set;
	PWR_ST  pwr_st;
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	
	buffer[0]= REG_MODECONTROL;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}

	power_state = reg_value[0] & 0xF;
    if (MCTL_TABLE[power_state].PS == 0){
       	pwr_st.ps_state = CTL_STANDBY;
    }else{
        pwr_st.ps_state = CTL_STANDALONE;
    }
		
	if(enable){
		if (pwr_st.ps_state == CTL_STANDALONE){
			power_set = PWRON_PS_ALS;
			databuf[0] = REG_MODECONTROL;	
			databuf[1] = power_set | (reg_value[0] & CLR_LOW4BIT);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}	
		}else if (pwr_st.ps_state == CTL_STANDBY){
			power_set = PWRON_ONLY_ALS;
			databuf[0] = REG_MODECONTROL;	
			databuf[1] = power_set | (reg_value[0] & CLR_LOW4BIT);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}
		}
		/*modify ps to ALS below two lines */
		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));				
	}else{
		if (pwr_st.ps_state == CTL_STANDALONE){
			power_set = PWRON_ONLY_PS;
			databuf[0] = REG_MODECONTROL;	
			databuf[1] = power_set | (reg_value[0] & CLR_LOW4BIT);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}
		}else if (pwr_st.ps_state == CTL_STANDBY){
			power_set = PWRON_STANDBY;
			databuf[0] = REG_MODECONTROL;	
			databuf[1] = power_set | (reg_value[0] & CLR_LOW4BIT);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}			
		}		
		atomic_set(&obj->als_deb_on, 0);
	}
	return 0;
		
	EXIT_ERR:
		APS_ERR("rpr400_enable_als fail\n");
		return res;
}

static int rpr400_enable_ps(struct i2c_client *client, int enable)
{
	struct PS_ALS_DATA *obj = i2c_get_clientdata(client);
	u8 databuf[2];   
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];
	u8 power_state, power_set;
	PWR_ST  pwr_st;	

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]= REG_MODECONTROL;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}

	power_state = reg_value[0] & 0xF;
    if(MCTL_TABLE[power_state].ALS == 0){
       	pwr_st.als_state = CTL_STANDBY;
    }else{
      	pwr_st.als_state = CTL_STANDALONE;
    }
		
	if(enable){
		if (pwr_st.als_state == CTL_STANDALONE){
			power_set = PWRON_PS_ALS;
			databuf[0] = REG_MODECONTROL;	
			databuf[1] = power_set | (reg_value[0] & CLR_LOW4BIT);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}
		}else if(pwr_st.als_state == CTL_STANDBY){
			power_set = PWRON_ONLY_PS;
			databuf[0] = REG_MODECONTROL;	
			databuf[1] = power_set | (reg_value[0] & CLR_LOW4BIT);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}
		}
		
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		
		if(0 == obj->polling_mode_ps)				
		{			
			databuf[0] = REG_PSTL_LSB;	
			databuf[1] = (u8)(PS_ALS_SET_PS_TL & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}
			databuf[0] = REG_PSTL_MBS;	
			databuf[1] = (u8)((PS_ALS_SET_PS_TL & 0xFF00) >> 8);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}
			databuf[0] = REG_PSTH_LSB;	
			databuf[1] = (u8)(PS_ALS_SET_ALS_TH & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}
			databuf[0] = REG_PSTH_MBS;
			databuf[1] = (u8)((PS_ALS_SET_ALS_TH & 0xFF00) >> 8);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}

			buffer[0]= REG_INTERRUPT;
			res = i2c_master_send(client, buffer, 0x1);
			if(res <= 0){
				goto EXIT_ERR;
			}
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0){
				goto EXIT_ERR;
			}

			databuf[0] = REG_INTERRUPT;
			databuf[1] = reg_value[0]|MODE_PROXIMITY;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}
			APS_LOG("rpr400_enable_ps: ps enabled.");
		}
			
	}else{
		if (pwr_st.als_state == CTL_STANDALONE){
			power_set = PWRON_ONLY_ALS;
			databuf[0] = REG_MODECONTROL;	
			databuf[1] = power_set | (reg_value[0] & CLR_LOW4BIT);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}
		}else if (pwr_st.als_state == CTL_STANDBY){
			power_set = PWRON_STANDBY;
			databuf[0] = REG_MODECONTROL;	
			databuf[1] = power_set | (reg_value[0] & CLR_LOW4BIT);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0){
				goto EXIT_ERR;
			}			
		}
		
		atomic_set(&obj->ps_deb_on, 0);
		APS_LOG("rpr400_enable_ps: ps disabled.");
	}
	
	return 0;
	
EXIT_ERR:
	APS_ERR("rpr400_enable_ps fail\n");
	return res;
}

static int rpr400_enable(struct i2c_client *client, int enable)
{
	u8 databuf[2];   
	int res = 0;

	if(client == NULL){
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	if(!enable){
		//disable
		databuf[0] = REG_MODECONTROL;	
		databuf[1] = PWRON_STANDBY;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0){
			goto EXIT_ERR;
		}
	}else{
		;	//please enable the ps and als seperately.
	}
	
	return 0;
	
EXIT_ERR:
	APS_ERR("rpr400_enable fail\n");
	return res;
}

/* for interrup work mode support. */
static int rpr400_check_and_clear_intr(struct i2c_client *client)
{
	int res;
	u8 buffer[2], int_status[1];
	
	buffer[0] = REG_INTERRUPT;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0){
		return -1;
	}
	res = i2c_master_recv(client, int_status, 0x1);
	if(res <= 0){
		return -1;
	}

	return int_status[0];

}

static irqreturn_t rpr400_interrupt(int irq,void *dev_id)
{
	struct PS_ALS_DATA *obj = g_rpr400_ptr;

    disable_irq_nosync(obj->client->irq);
    schedule_work(&obj->eint_work);

    return IRQ_HANDLED;
}

static int rpr400_init_client(struct i2c_client *client)
{
	struct PS_ALS_DATA *obj = i2c_get_clientdata(client);
	u8 databuf[2];   
	int res = 0;

	databuf[0] = REG_SYSTEMCONTROL;   
	databuf[1] = REG_SW_RESET | REG_INT_RESET;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0){
		goto EXIT_ERR;
		return rpr400_ERR_I2C;
	}	
	
	databuf[0] = REG_MODECONTROL;   
	databuf[1] = PS_ALS_SET_MODE_CONTROL|PWRON_PS_ALS;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0){
		goto EXIT_ERR;
		return rpr400_ERR_I2C;
	}
	
	databuf[0] = REG_ALSPSCONTROL;   
	databuf[1] = PS_ALS_SET_ALSPS_CONTROL;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0){
		goto EXIT_ERR;
		return rpr400_ERR_I2C;
	}

	databuf[0] = REG_PERSISTENCE;   
	databuf[1] = PS_ALS_SET_INTR_PERSIST;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0){
		goto EXIT_ERR;
		return rpr400_ERR_I2C;
	}

	databuf[0] = REG_INTERRUPT;
	databuf[1] = PS_ALS_SET_INTR;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0){
		goto EXIT_ERR;
		return rpr400_ERR_I2C;
	}
	
	/*for interrup work mode support */
	if(0 == obj->polling_mode_ps){				
		databuf[0] = REG_PSTL_LSB;	
		databuf[1] = (u8)(PS_ALS_SET_PS_TL & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0){
			goto EXIT_ERR;
			return rpr400_ERR_I2C;
		}
			
		databuf[0] = REG_PSTL_MBS;	
		databuf[1] = (u8)((PS_ALS_SET_PS_TL & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0){
			goto EXIT_ERR;
			return rpr400_ERR_I2C;
		}
			
		databuf[0] = REG_PSTH_LSB;	
		databuf[1] = (u8)( PS_ALS_SET_PS_TH & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0){
			goto EXIT_ERR;
			return rpr400_ERR_I2C;
		}
			
		databuf[0] = REG_PSTH_MBS;	
		databuf[1] = (u8)(( PS_ALS_SET_PS_TH & 0xFF00) >> 8);;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0){
			goto EXIT_ERR;
			return rpr400_ERR_I2C;
		}

		databuf[0] = REG_INTERRUPT;
		databuf[1] = PS_ALS_SET_INTR | MODE_PROXIMITY;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0){
			goto EXIT_ERR;
			return rpr400_ERR_I2C;
		}
	}

	databuf[0] = REG_ALSDATA0TH_LSB;   
	databuf[1] = PS_ALS_SET_ALS_TH & 0x00FF ;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0){
		goto EXIT_ERR;
		return rpr400_ERR_I2C;
	}
     
	databuf[0] = REG_ALSDATA0TH_MBS;   
	databuf[1] = (PS_ALS_SET_ALS_TH& 0xFF00) >> 8;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0){
		goto EXIT_ERR;
		return rpr400_ERR_I2C;
	}

	databuf[0] = REG_ALSDATA0TL_LSB;   
	databuf[1] = PS_ALS_SET_ALS_TL & 0x00FF ;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0){
		goto EXIT_ERR;
		return rpr400_ERR_I2C;
	}

	databuf[0] = REG_ALSDATA0TL_MBS;   
	databuf[1] = (PS_ALS_SET_ALS_TL& 0xFF00) >> 8;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0){
		goto EXIT_ERR;
		return rpr400_ERR_I2C;
	}
	
	return 0;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/* reset RPR400 register. */
static int rpr400_ps_als_driver_reset(struct i2c_client *client)
{
	u8 databuf[2];   
	int res = 0;
  
	databuf[0] = REG_SYSTEMCONTROL;   
	databuf[1] = REG_SW_RESET | REG_INT_RESET;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0){
		goto EXIT_ERR;
		return rpr400_ERR_I2C;
	}	

   	return 0;	 	
	
EXIT_ERR:
	APS_ERR("rpr400 reset fail\n");
	return res;
}


/* calc divider of unsigned long long int or unsgined long.*/ 
static void long_long_divider(unsigned long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile unsigned long long divier;
    volatile unsigned long      unit_sft;

    if ((long long)data < 0){	// . If data MSB is 1, it may go to endless loop.
		*answer = 0;
		*overplus = 0;
		return;		//Theorically, if data is negative, the program will have been returned CALC_ERROR earlier.
    }
    divier = base_divier;
    if (data > MASK_LONG) {
        unit_sft = 0;
        while (data > divier) {
            unit_sft++;
            divier = divier << 1;
        }
        while (data > base_divier) {
            if (data > divier) {
                *answer += 1 << unit_sft;
                data    -= divier;
            }
            unit_sft--;
            divier = divier >> 1;
        }
        *overplus = data;
    } else {
        *answer = (unsigned long)(data & MASK_LONG) / base_divier;
        /* calculate over plus and shift 16bit */
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }
}

/* calculate illuminance data for RPR400
   final_data is 1000 times, which is defined as CUT_UNIT, of the actual lux value.
*/
static int calc_rohm_als_data(READ_DATA_BUF data, DEVICE_VAL dev_val)
{
#define DECIMAL_BIT      (15)
#define JUDGE_FIXED_COEF (100)
#define MAX_OUTRANGE     (11357)
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)
#define CUT_UNIT         10

	int                final_data, mid_data;
	CALC_DATA          calc_data;
	CALC_ANS           calc_ans;
	unsigned long      calc_judge;
	unsigned char      set_case;
	unsigned long      div_answer;
	unsigned long long div_overplus;
	unsigned long long overplus;
	unsigned long      max_range;

	/* set the value of measured als data */
	calc_data.als_data0  = data.als_data0;
	calc_data.als_data1  = data.als_data1;
	calc_data.gain_data0 = GAIN_TABLE[dev_val.gain].DATA0;

	/* set max range */
	if (calc_data.gain_data0 == 0){
		/* issue error value when gain is 0 */
		return (CALC_ERROR);
	}else{
		max_range = MAX_OUTRANGE / calc_data.gain_data0;
	}
	
	/* calculate data */
	if (calc_data.als_data0 == MAXRANGE_NMODE){
		calc_ans.positive = max_range;
		calc_ans.decimal  = 0;
	}else{
		/* get the value which is measured from power table */
		calc_data.als_time = MCTL_TABLE[dev_val.time].ALS;
		if (calc_data.als_time == 0){
			/* issue error value when time is 0 */
			return (CALC_ERROR);
		}

		calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
		if (calc_judge < (calc_data.als_data0 * judge_coefficient[0])){
			set_case = 0;
		}else if (calc_judge < (data.als_data0 * judge_coefficient[1])){
			set_case = 1;
		}else if (calc_judge < (data.als_data0 * judge_coefficient[2])){
			set_case = 2;
		}else if (calc_judge < (data.als_data0 * judge_coefficient[3])){
			 set_case = 3;
		}else{
			set_case = MAXSET_CASE;
		}
		calc_ans.positive = 0;
		if (set_case >= MAXSET_CASE){
			calc_ans.decimal = 0;	//which means that lux output is 0
		}else{
			calc_data.gain_data1 = GAIN_TABLE[dev_val.gain].DATA1;
			if (calc_data.gain_data1 == 0){
				/* issue error value when gain is 0 */
				return (CALC_ERROR);
			}
			calc_data.data0      = (unsigned long long )(data0_coefficient[set_case] * calc_data.als_data0) * calc_data.gain_data1;
			calc_data.data1      = (unsigned long long )(data1_coefficient[set_case] * calc_data.als_data1) * calc_data.gain_data0;
			if (calc_data.data0 < calc_data.data1){
				/* issue error value when data is negtive */
				return (CALC_ERROR);
			}
			calc_data.data       = (calc_data.data0 - calc_data.data1);
			calc_data.dev_unit   = calc_data.gain_data0 * calc_data.gain_data1 * calc_data.als_time * 10;
			if (calc_data.dev_unit == 0){
				/* issue error value when dev_unit is 0 */
				return (CALC_ERROR);
			}

			/* calculate a positive number */
			div_answer   = 0;
			div_overplus = 0;
			long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus);
			calc_ans.positive = div_answer;
			/* calculate a decimal number */
			calc_ans.decimal = 0;
			overplus         = div_overplus;
			if (calc_ans.positive < max_range){
				if (overplus != 0){
					overplus     = overplus << DECIMAL_BIT;
					div_answer   = 0;
					div_overplus = 0;
					long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
					calc_ans.decimal = div_answer;
				}
			}else{
				calc_ans.positive = max_range;
			}
		}
	}
	
	mid_data = (calc_ans.positive << DECIMAL_BIT) + calc_ans.decimal;
	final_data = calc_ans.positive * CUT_UNIT + ((calc_ans.decimal * CUT_UNIT) >> DECIMAL_BIT);
					
	return (final_data);

#undef CUT_UNIT
#undef DECIMAL_BIT
#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
}

/* periodically reads the data from sensor(thread of work) 
   from its function, now it is ready.
*/
static int get_from_device(DEVICE_VAL *dev_val, struct i2c_client *client)
{
#define LEDBIT_MASK   (3)
#define GAIN_VAL_MASK (0xF)

	int res = 0;
#if 0
	struct PS_ALS_DATA *obj = i2c_get_clientdata(client);	
	u8 buffer[1];
	int res = 0;
    	unsigned char alsps_ctl[1], read_time[1];

   	 /* initalize the returning value */
    	dev_val->time        = 6;
    	dev_val->gain        = (PS_ALS_SET_ALSPS_CONTROL >> 2) & GAIN_VAL_MASK;
    	dev_val->led_current = PS_ALS_SET_ALSPS_CONTROL & LEDBIT_MASK;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=REG_MODECONTROL;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, read_time, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	dev_val->time = read_time[0] & 0xF;

	buffer[0]=REG_ALSPSCONTROL;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, alsps_ctl, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

    	dev_val->led_current = alsps_ctl[0] & LEDBIT_MASK;
    	dev_val->gain        = (alsps_ctl[0] >> 2) & GAIN_VAL_MASK;

#else
    	dev_val->time        = 6;
    	dev_val->gain        = (PS_ALS_SET_ALSPS_CONTROL >> 2) & GAIN_VAL_MASK;
    	dev_val->led_current = PS_ALS_SET_ALSPS_CONTROL & LEDBIT_MASK;
#endif

    return (0);
#if 0           /* to get rid of a warning */		
EXIT_ERR:
#endif
	APS_ERR("rpr400_read_ps fail\n");
	return res;

#undef LEDBIT_MASK
#undef GAIN_VAL_MASK
}

int rpr400_read_als(struct i2c_client *client, u16 *data)
{
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	u16 prev_als_value = *data;	//Keep previous value
	int res = 0;
	READ_DATA_BUF   als_data;
	DEVICE_VAL  dev_val;
	
	if(client == NULL){
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	
    //get adc channel 0 value
	buffer[0]=REG_ALSDATA0_LSB;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
	
	buffer[0]=REG_ALSDATA0_MBS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0){
		goto EXIT_ERR;
	}
	
	als_data.als_data0 = als_value_low[0] | (als_value_high[0]<<8);
	printk(KERN_INFO "\nROHM RPR400 read als: als_data0 = %d\n", als_data.als_data0);
	
    //get adc channel 1 value
	buffer[0]=REG_ALSDATA1_LSB;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
		
	buffer[0]=REG_ALSDATA1_MBS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0){
		goto EXIT_ERR;
	}
		
	als_data.als_data1 = als_value_low[0] | (als_value_high[0]<<8);
	printk(KERN_INFO "\nROHM RPR400 read als: als_data1 = %d\n", als_data.als_data1);

	get_from_device(&dev_val, client);

	*data = calc_rohm_als_data(als_data, dev_val);
	if(*data == CALC_ERROR){
		*data = prev_als_value;	//Report same value as previous.
		return 0;
	}
	if(*data == 0)
		*data = *data + 1;

	return 0;	
	
EXIT_ERR:
	APS_ERR("rpr400_read_als fail\n");
	return res;
}

int rpr400_read_als_ch0(struct i2c_client *client, u16 *data)
{
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	int res = 0;
	
	if(client == NULL){
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
    
    //get adc channel 0 value
	buffer[0]=REG_ALSDATA0_LSB;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
	
	buffer[0]=REG_ALSDATA0_MBS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0){
		goto EXIT_ERR;
	}
	
	*data = als_value_low[0] | (als_value_high[0]<<8);

	return 0;	
	
EXIT_ERR:
	APS_ERR("rpr400_read_als fail\n");
	return res;
}

static int rpr400_get_als_value(struct PS_ALS_DATA *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++){
		if(als < obj->als_level[idx]){
			break;
		}
	}
	
	if(idx >= obj->als_value_num){
		APS_ERR("exceed range\n");
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on)){
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt)){
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on)){
			invalid = 1;
		}
	}

	if(!invalid){
		return obj->als_value[idx];
	}else{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->als_value[idx]);   
		return -1;
	}
}

int rpr400_read_ps(struct i2c_client *client, u16 *data)
{
	u8 ps_value_low[1], ps_value_high[1];
	u8 buffer[1];
	int res = 0;

	if(client == NULL){
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=REG_PSDATA_LSB;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_low, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}

	buffer[0]=REG_PSDATA_MBS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0){
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_high, 0x01);
	if(res <= 0){
		goto EXIT_ERR;
	}

	*data = ps_value_low[0] | (ps_value_high[0]<<8);
	return 0;   

EXIT_ERR:
	APS_ERR("rpr400_read_ps fail\n");
	return res;
}

static int rpr400_get_ps_value(struct PS_ALS_DATA *obj, u16 ps)
{
	int val; //, mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	static int val_temp=1;
	u16 temp_ps[1];
	
    mdelay(160);
	rpr400_read_ps(obj->client,temp_ps);
	if((ps > atomic_read(&obj->ps_thd_val))&&(temp_ps[0]  > atomic_read(&obj->ps_thd_val))){
		val = 0;  /*close*/
		val_temp = 0;
	}else if((ps < atomic_read(&obj->ps_thd_val))&&(temp_ps[0]  < atomic_read(&obj->ps_thd_val))){
		val = 1;  /*far away*/
		val_temp = 1;
	}else
		val = val_temp;	
				
	if(atomic_read(&obj->ps_suspend)){
		invalid = 1;
	}else if(1 == atomic_read(&obj->ps_deb_on)){
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt)){
			atomic_set(&obj->ps_deb_on, 0);
		}		
		if (1 == atomic_read(&obj->ps_deb_on)){
			invalid = 1;
		}
	}
	else if (obj->als > 45000){
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid){
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}else{
		return -1;
	}	
}

/* for interrup work mode support.
   still do not now how to report value to up-layer, need work to do.
   need add input subsystem report ---> by magf.
*/
static void rpr400_eint_work(struct work_struct *work)
{
	struct PS_ALS_DATA *obj = (struct PS_ALS_DATA *)container_of(work, struct PS_ALS_DATA, eint_work);
	//int err;
//	hwm_sensor_data sensor_data;

	//int res, int_status;
	//u8 buffer[2];
	
	int result = 0;
	int status;   
	//READ_DATA_BUF read_data_buf;

	//DEVICE_VAL    dev_val;
	//long          get_timer;
	//long          wait_sec;
	//unsigned long wait_nsec;

	if(0 == obj->polling_mode_ps){
		result = rpr400_check_and_clear_intr(obj->client);
		if(result < 0){
			APS_DBG("ERROR! read interrupt status. \n");
		}else{
			status = result;
			if(status & PS_INT_MASK){
				rpr400_read_ps(obj->client, &obj->ps);
			}
			if(status & ALS_INT_MASK){ // 2 kinds of interrupt may occur at same time
				rpr400_read_als_ch0(obj->client, &obj->als);
			}
			if(!((status & ALS_INT_MASK) || (status & PS_INT_MASK))){
				APS_DBG( "Unknown interrupt source.\n");
			}
			
//			sensor_data.values[0] = rpr400_get_ps_value(obj, obj->ps);
//			sensor_data.value_divide = 1;
//			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			

			//let up layer to know
//			if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data))){
//		 		 APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
//			}
		}		
		enable_irq(obj->client->irq);
	}else{
		rpr400_read_ps(obj->client, &obj->ps);
		//mdelay(160);
		rpr400_read_als_ch0(obj->client, &obj->als);
		APS_DBG("rpr400_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
//		sensor_data.values[0] = rpr400_get_ps_value(obj, obj->ps);
//		sensor_data.value_divide = 1;
//		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			

		//let up layer to know
//		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data))){
//		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
//		}			
	}
		
//	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);     //==============> must be changed to nowadays mechanism, by magf
}

static int rpr400_open(struct inode *inode, struct file *file)
{
	file->private_data = g_rpr400_ptr->client;

	if (!file->private_data){
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}

static int rpr400_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long rpr400_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct PS_ALS_DATA *obj = i2c_get_clientdata(client); 
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;

	switch (cmd){
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable))){
				err = -EFAULT;
				goto err_out;
			}
			if(enable){
				err = rpr400_enable_ps(obj->client, 1);
				if(err){
					APS_ERR("enable ps fail: %d\n", err);
					goto err_out;
				}				
				set_bit(CMC_BIT_PS, &obj->enable);
			}else{
				err = rpr400_enable_ps(obj->client, 0);
				if(err){
					APS_ERR("disable ps fail: %d\n", err);
					goto err_out;
				}				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable))){
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:   
			err = rpr400_read_ps(obj->client, &obj->ps);
			if(err){
				goto err_out;
			}			
			dat = rpr400_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat))){
				err = -EFAULT;
				goto err_out;
			} 
			break;

		case ALSPS_GET_PS_RAW_DATA:   
			err = rpr400_read_ps(obj->client, &obj->ps);
			if(err){
				goto err_out;
			}			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat))){
				err = -EFAULT;
				goto err_out;
			} 
			break;             

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable))){
				err = -EFAULT;
				goto err_out;
			}
			if(enable){
				err = rpr400_enable_als(obj->client, 1);
				if(err){
					APS_ERR("enable als fail: %d\n", err);
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}else{
				err = rpr400_enable_als(obj->client, 0);
				if(err){
					APS_ERR("disable als fail: %d\n", err);
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable))){
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA:
			err = rpr400_read_als(obj->client, &obj->als);
			if(err){
				goto err_out;
			}
			dat = rpr400_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat))){
				err = -EFAULT;
				goto err_out;
			}             
			break;

		case ALSPS_GET_ALS_RAW_DATA:   
			err = rpr400_read_als(obj->client, &obj->als);
			if(err){
				goto err_out;
			}
			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat))){
				err = -EFAULT;
				goto err_out;
			}             
			break;

		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;   
}

static struct file_operations rpr400_fops = {
	.owner = THIS_MODULE,
	.open = rpr400_open,
	.release = rpr400_release,	
	.unlocked_ioctl = rpr400_ioctl,  
};

static struct miscdevice rpr400_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &rpr400_fops,
};

static void rpr400_early_suspend(struct early_suspend *handler)
{
	struct PS_ALS_DATA *obj = g_rpr400_ptr;
	int err;

	APS_FUN();

	if(!obj){
		APS_ERR("null pointer!!\n");
		return;
	}
		
	atomic_set(&obj->als_suspend, 1);
	err = rpr400_enable_als(obj->client, 0);
	if(err){
		APS_ERR("disable als: %d\n", err);
		return;
	}

	if(0 == obj->polling_mode_ps){
		disable_irq(obj->client->irq);
	}

	atomic_set(&obj->ps_suspend, 1);
	err = rpr400_enable_ps(obj->client, 0);
	if(err){
		APS_ERR("disable ps:  %d\n", err);
		return;
	}

	return;
}

static void rpr400_early_resume(struct early_suspend *handler)
{
	struct PS_ALS_DATA *obj = g_rpr400_ptr;
	int err;

	APS_FUN();

	if(!obj){
		APS_ERR("null pointer!!\n");
		return;
	}

	/* it seem that this is not neccessary, disable it first. */
/*	if(err = rpr400_init_client(obj->client)){									// ======> pay attention, please. by magf
		APS_ERR("initialize client fail!!\n");
		return err;       
	}
*/	
	atomic_set(&obj->als_suspend, 0);

	if(test_bit(CMC_BIT_ALS, &obj->enable)){
		err = rpr400_enable_als(obj->client, 1);
		if(err){
			APS_ERR("enable als fail: %d\n", err);       
			return;
		}
	}

	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable)){
		err = rpr400_enable_ps(obj->client, 1);
		if(err){				
			APS_ERR("enable ps fail: %d\n", err);               
			return;
		}
	}
	
	if(0 == obj->polling_mode_ps) {
		enable_irq(obj->client->irq);
	}

	return;
}

#ifdef AUTO_TEST_MODE
static int alsps_test_read(void)
{
    struct PS_ALS_DATA *obj = g_rpr400_ptr;
    int err = 0;
    int dat;

    err = rpr400_read_als(obj->client, &obj->als);
    if(err) {
        APS_ERR("rpr400 auto test, read als failed.");
        return -1;
    }else{
        APS_LOG("rpr400 raw_als = %d", obj->als);
    }

    dat = rpr400_get_als_value(obj, obj->als);
    APS_LOG("rpr400 converted als = %d", dat);

    err = rpr400_read_ps(obj->client, &obj->ps);
    if(err) {
        APS_ERR("rpr400 auto test, read ps failed.");
        return -1;
    }else{
        APS_LOG("rpr400 raw_ps = %d\n", obj->ps);
    }

    dat = rpr400_get_ps_value(obj, obj->ps);
    APS_LOG("rpr400 converted ps = %d\n", dat);

    return 0;
}

static int auto_test_read(void *unused)
{
        while(1){
                alsps_test_read();
                msleep(200);
        }
        return 0;
}
#endif

static int rpr400_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
#ifdef AUTO_TEST_MODE
	struct task_struct *thread;
#endif
	struct PS_ALS_DATA *obj;
	int    err = 0;	

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));

	/*for interrup work mode support */
	INIT_WORK(&obj->eint_work, rpr400_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	atomic_set(&obj->ps_thd_val,  PS_THD_VALUE);	
	obj->polling_mode_ps = PS_POLLING_MODE;
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->als_level)/sizeof(obj->als_level[0]);
	obj->als_value_num = sizeof(obj->als_value)/sizeof(obj->als_value[0]); 
	
	obj->als_modulus = (400*100*40)/(1*1500);	//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										
	memcpy(obj->als_level, g_als_level, sizeof(obj->als_level));
	memcpy(obj->als_value, g_als_value, sizeof(obj->als_value));

	/* this is useless, will be removed */
	atomic_set(&obj->i2c_retry, 3);					

	g_rpr400_ptr = obj;

	err = rpr400_init_client(client);
	if(err){
		goto exit_kfree;
	}

	/* interrupt setup must be after rpr400_init_client 
	   because interrupt mode must be setup before in the device.
	*/
	APS_LOG("rpr400_probe, polling_mode_ps = %d\n", obj->polling_mode_ps);
    if(0 == obj->polling_mode_ps){
        /* the following code setup interrupt */
        err = gpio_request(client->irq, "rpr400_irq");
        if(err){
            APS_ERR( "Failed to request rpr400 ps irq GPIO!\n");
            goto exit_kfree;
        }
        err = gpio_direction_input(client->irq);
        if(err){
            APS_ERR("failed to set rpr400 irq gpio input\n");
            goto exit_kfree;
        }
        gpio_pull_updown(client->irq, GPIOPullUp);

		/* our interrupt is Low stable if newer measurement result is 
		   also interrupt active, if not work, need change to IRQ_TYPE_LEVEL_LOW.
		*/
        err = request_irq(client->irq, rpr400_interrupt, IRQ_TYPE_EDGE_FALLING, "rpr400_ps", g_rpr400_ptr);   // =====> maybe LOW is correct, by magf
        if(err){
            printk(KERN_ERR "rpr400: request irq failed\n");
            goto exit_kfree;
        }
    }

	APS_LOG("rpr400_init_client() OK!\n");

	err = misc_register(&rpr400_device);
	if(err){
		APS_ERR("rpr400_device register failed\n");
		goto exit_kfree;
	}

	/* register input device */	
    obj->input = input_allocate_device();
    if (!obj->input) {
        err = -ENOMEM;
        APS_ERR("rpr400: Failed to allocate input device\n");
        goto exit_input_allocate_device_failed;
    }
    set_bit(EV_ABS, obj->input->evbit);									// ==========> we need add proximity event bit. by magf
    input_set_abs_params(obj->input, ABS_MISC, 0, 0x1ff, 0, 0);
    obj->input->name = "rpr400-input";

    err = input_register_device(obj->input);
    if (err < 0) {
        APS_ERR("rpr400: Unable to register input device: %s\n", obj->input->name);
        goto exit_input_register_device_failed;
    }

#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = rpr400_early_suspend,
	obj->early_drv.resume   = rpr400_early_resume,   
	register_early_suspend(&obj->early_drv);
#endif

#ifdef AUTO_TEST_MODE
    err = rpr400_enable_als(obj->client, 1);
    if(err){
        APS_ERR("enable als fail: %d\n", err);
    }
    set_bit(CMC_BIT_ALS, &obj->enable);
	
	err = rpr400_enable_ps(obj->client, 1);
	if(err){
		APS_ERR("enable ps fail: %d\n", err);
	}
	set_bit(CMC_BIT_PS, &obj->enable);

    thread=kthread_run(auto_test_read, NULL, "rpr400_read_test");
	if(thread) APS_LOG("rpr400 auto test thread started.");
#endif

	APS_LOG("%s: rpr400 i2c device create OK\n", __func__);
	return 0;


exit_input_register_device_failed:
	input_free_device(obj->input);
exit_input_allocate_device_failed:
	misc_deregister(&rpr400_device);
exit_kfree:
	kfree(obj);
exit:
	g_rpr400_ptr = NULL;
	APS_ERR("%s: rpr400 i2c device probe failed, err = %d\n", __func__, err);
	return err;
}

static int rpr400_remove(struct i2c_client *client)
{
	APS_FUN();
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&g_rpr400_ptr->early_drv);
#endif
	misc_deregister(&rpr400_device);
	input_unregister_device(g_rpr400_ptr->input);
	cancel_work_sync(&g_rpr400_ptr->eint_work);
	kfree(g_rpr400_ptr);
	g_rpr400_ptr = NULL;
	APS_LOG("rpr400 successfully removed.");

	return 0;
}

static const struct i2c_device_id rpr400_i2c_id[] = {
	{rpr400_DEV_NAME, 0},
	{ }
};

static struct i2c_driver rpr400_driver = {
    .probe      = rpr400_probe,
    .remove     = __devexit_p(rpr400_remove),
    .driver = {
        .owner  = THIS_MODULE,
        .name   = rpr400_DEV_NAME,
    },
    .id_table   = rpr400_i2c_id,
};

static int __init rpr400_init(void)
{
	APS_FUN();
	return i2c_add_driver(&rpr400_driver);
}

static void __exit rpr400_exit(void)
{
	APS_FUN();
	i2c_del_driver(&rpr400_driver);
}

module_init(rpr400_init);
module_exit(rpr400_exit);

MODULE_AUTHOR("Paul Ma");
MODULE_DESCRIPTION("rpr400 driver");
MODULE_LICENSE("GPL");
