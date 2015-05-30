#ifndef __RK_NTC_H__
#define __RK_NTC_H__

struct ntc_res_temp_tbl{
	int temperature;
	int resistance;
};
struct rk_ntc_platform_data{
	struct ntc_res_temp_tbl  res_to_temp_tbl[40];
	int adc_chn;

};

#define TAIL_OF_TABLE	-1
#define ADC_SAMPLE_TIME				500
#define ADC_REF_VOLTAGE				1800

#endif

