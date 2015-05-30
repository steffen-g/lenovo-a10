#ifndef BQ24195_CHARGER_H
#define BQ24195_CHARGER_H

struct bq24195_info{
	unsigned int chg_en_pin;        
	unsigned int chg_det_pin;       
	unsigned int chg_stat_pin;	
	unsigned int chg_susp_pin;
    unsigned int max_current;       
    bool    otg_power_form_smb;
};

#endif
