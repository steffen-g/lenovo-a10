/*****************************************************************************
*
*	Copyright(c) O2Micro, 2012. All rights reserved.
*	
*	Description:
*		
*   Qualification Level: 
*		Sample Code Release (Sample Code Release | Production Release)
*   
*	Release Purpose: 
*		Reference design for OZ8806 access
*   
*	Release Target: O2MICRO Customer
*   Re-distribution Allowed: 
*				No (Yes, under agreement | No)
*   Author: Eason.yuan
*	$Source: /data/code/CVS
*	$Revision: 3.00.00 $
*
*****************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include "parameter.h"

/*****************************************************************************
* Define section
* add all #define here
*****************************************************************************/
//#define OCV_DATA_NUM  11
#define OCV_DATA_NUM  51
#define TEMPERATURE_DATA_NUM 28

/****************************************************************************
* extern variable declaration section
****************************************************************************/


/*****************************************************************************
* Global variables section - Exported
* add declaration of global variables that will be exported here
* e.g.
*	int8_t foo;
****************************************************************************/

/*
one_latitude_data_t ocv_data[OCV_DATA_NUM] = {
				{3590, 00},{3650, 10},{3690, 20},
				{3740, 30},{3790, 40},{3830, 50}, 
				{3870, 60},{3910, 70},{3950, 80}, 
				{3990, 90},{4100, 100},
};
*/

one_latitude_data_t ocv_data[OCV_DATA_NUM] = {
	{3570, 00},{3580, 2},{3592, 4},{3605, 6},{3607, 8},{3617, 10},{3625, 12},
	{3632, 14},{3640, 16},{3647, 18},{3650, 20},{3653, 22},{3657, 24},{3662, 26},
	{3664, 28},{3665, 30},{3672, 32},{3680, 34},{3682, 36},{3687, 38},{3695, 40},
	{3702, 42},{3705, 44},{3715, 46},{3722, 48},{3727, 50},{3735, 52},{3740, 54},
	{3755, 56},{3765, 58},{3777, 60},{3785, 62},{3800, 64},{3815, 66},{3827, 68},
	{3840, 70},{3850, 72},{3862, 74},{3882, 76},{3900, 78},{3915, 80},{3932, 82},
	{3945, 84},{3955, 86},{3977, 88},{3992, 90},{4012, 92},{4030, 94},{4040, 96},
	{4045, 98},{4050, 100},
};

one_latitude_data_t			cell_temp_data[TEMPERATURE_DATA_NUM] = {                                                                                                               
			{681,   115}, {766,   113}, {865,   105},
			{980,   100}, {1113,   95}, {1266,   90},
			{1451,   85}, {1668,   80}, {1924,   75},
			{2228,   70}, {2588,   65}, {3020,   60},
			{3536,   55}, {4160,   50}, {4911,   45},
			{5827,   40}, {6940,   35}, {8313,   30},
			{10000,  25}, {12090,  20}, {14690,  15},
			{17960,  10}, {22050,   5},	{27280,   0},
			{33900,  -5}, {42470, -10}, {53410, -15},
			{67770, -20},
};


config_data_t config_data = {20,232000,3300,5,781,250,5500,4200,100,3500,0,0};

/*
	int32_t		fRsense;		//= 20;			//Rsense value of chip, in mini ohm
	int32_t     temp_pull_up;  //230000;
	int32_t     temp_ref_voltage; //1800;1.8v
	int32_t		dbCARLSB;		//= 5.0;		//LSB of CAR, comes from spec
	int32_t		dbCurrLSB;		//781 (3.90625*100);	//LSB of Current, comes from spec
	int32_t		fVoltLSB;		//250 (2.5*100);	//LSB of Voltage, comes from spec

	int32_t		design_capacity;	//= 7000;		//design capacity of the battery
 	int32_t		charge_cv_voltage;	//= 4200;		//CV Voltage at fully charged
	int32_t		charge_end_current;	//= 100;		//the current threshold of End of Charged
	int32_t		discharge_end_voltage;	//= 3550;		//mV
	int32_t     board_offset;			//0; 				//mA, not more than caculate data
	uint8_t     debug;                                          // enable or disable O2MICRO debug information
}
*/
parameter_data_t parameter_customer;

/*****************************************************************************
 * Description:
 *		bmu_init_chip
 * Parameters:
 *		description for each argument, new argument starts at new line
 * Return:
 *		what does this function returned?
 *****************************************************************************/
void bmu_init_parameter(struct i2c_client *client)
{
	parameter_customer.config = &config_data;
	parameter_customer.ocv = &ocv_data;
	parameter_customer.temperature = &cell_temp_data;
	parameter_customer.client = client;
	parameter_customer.ocv_data_num = OCV_DATA_NUM;
	parameter_customer.cell_temp_num = TEMPERATURE_DATA_NUM;
	parameter_customer.charge_pursue_step = 10;		
 	parameter_customer.discharge_pursue_step = 5;		
	parameter_customer.discharge_pursue_th = 5;
	parameter_customer.wait_method = 2;
	
}










