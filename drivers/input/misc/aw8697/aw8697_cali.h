#ifndef __AW8697_CALI_H__
#define __AW8697_CALI_H__

#define AW_CALI_STORE_EXAMPLE
//#define AW_F0_BRINGUP_CALI

enum aw86xx_channel_mode { 
	AW8697_CHANNLE = 0,
	AW8697X_CHANNLE = 1,
};

extern int aw8697_set_cali_lra_to_nvram(char cali_lra, int channel);
extern int aw8697_get_cali_lra_from_nvram(char *cali_lra, int channel);

#endif