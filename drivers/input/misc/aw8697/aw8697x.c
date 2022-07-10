/*
 * aw8697x.c   aw8697x haptic module
 *
 * Version: v1.4.9
 *
 * Copyright (c) 2018 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/power_supply.h>
#include <linux/pm_qos.h>
#include <linux/input/touch_event_notify.h>
#include <linux/aw8697_config.h>
#include <linux/aw8697_reg.h>
#include <linux/aw8697.h>
//#include <linux/leds.h>
#include "aw8697_cali.h"
/******************************************************
 *
 * Marco
 *
 ******************************************************/
#define AW8697X_I2C_NAME "aw8697x_haptic"
#define AW8697X_HAPTIC_NAME "aw8697x_haptic"

#define AW8697X_VERSION "v1.4.9"

#define AWINIC_RAM_UPDATE_DELAY

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 2
#define AW_READ_CHIPID_RETRIES 5
#define AW_READ_CHIPID_RETRY_DELAY 2
#define AW8697_MAX_DSP_START_TRY_COUNT    10
#define AWINIC_READ_BIN_FLEXBALLY

#define AW8697_MAX_FIRMWARE_LOAD_CNT 20
#define OSC_CALIBRATION_T_LENGTH 5100000
#define PM_QOS_VALUE_VB 400
unsigned char g_aw8697x_rtp_going = 0;
struct pm_qos_request pm_qos_req_vb_x;
/******************************************************
 *
 * variable
 *
 ******************************************************/
//#define AW8697_RTP_NAME_MAX        64
static char *aw8697x_ram_name = "aw8697x_haptic.bin";
/*static char aw8697x_rtp_name[][AW8697_RTP_NAME_MAX] = {
	{"aw8697x_osc_rtp_24K_5s.bin"},
	{"aw8697x_rtp.bin"},
	{"aw8697x_rtp_lighthouse.bin"},
	{"aw8697x_rtp_silk.bin"},
};*/

struct aw8697_container *aw8697x_rtp;
struct aw8697 *g_aw8697x = NULL;

const unsigned char aw8697x_reg_access[AW8697_REG_MAX] = {
	[AW8697_REG_ID] = REG_RD_ACCESS,
	[AW8697_REG_SYSST] = REG_RD_ACCESS,
	[AW8697_REG_SYSINT] = REG_RD_ACCESS,
	[AW8697_REG_SYSINTM] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_SYSCTRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_GO] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_RTP_DATA] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAVSEQ1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAVSEQ2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAVSEQ3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAVSEQ4] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAVSEQ5] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAVSEQ6] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAVSEQ7] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAVSEQ8] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAVLOOP1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAVLOOP2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAVLOOP3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAVLOOP4] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_MAIN_LOOP] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TRG1_WAV_P] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TRG2_WAV_P] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TRG3_WAV_P] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TRG1_WAV_N] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TRG2_WAV_N] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TRG3_WAV_N] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TRG_PRIO] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TRG_CFG1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TRG_CFG2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_DBGCTRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BASE_ADDRH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BASE_ADDRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_FIFO_AEH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_FIFO_AEL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_FIFO_AFH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_FIFO_AFL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAKE_DLY] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_START_DLY] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_END_DLY_H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_END_DLY_L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_DATCTRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_PWMDEL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_PWMPRC] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_PWMDBG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_LDOCTRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_DBGSTAT] = REG_RD_ACCESS,
	[AW8697_REG_BSTDBG1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BSTDBG2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BSTDBG3] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BSTCFG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_ANADBG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_ANACTRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_CPDBG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_GLBDBG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_DATDBG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BSTDBG4] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BSTDBG5] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BSTDBG6] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_HDRVDBG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_PRLVL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_PRTIME] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_RAMADDRH] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_RAMADDRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_RAMDATA] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_GLB_STATE] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BST_AUTO] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_CONT_CTRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_F_PRE_H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_F_PRE_L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TD_H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TD_L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TSET] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TRIM_LRA] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_R_SPARE] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_D2SCFG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_DETCTRL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_RLDET] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_OSDET] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_VBATDET] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TESTDET] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_DETLO] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BEMFDBG] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_ADCTEST] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BEMFTEST] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_F_LRA_F0_H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_F_LRA_F0_L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_F_LRA_CONT_H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_F_LRA_CONT_L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAIT_VOL_MP] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_WAIT_VOL_MN] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BEMF_VOL_H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BEMF_VOL_L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_ZC_THRSH_H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_ZC_THRSH_L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BEMF_VTHH_H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BEMF_VTHH_L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BEMF_VTHL_H] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BEMF_VTHL_L] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_BEMF_NUM] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_DRV_TIME] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_TIME_NZC] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_DRV_LVL] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_DRV_LVL_OV] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_NUM_F0_1] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_NUM_F0_2] = REG_RD_ACCESS | REG_WR_ACCESS,
	[AW8697_REG_NUM_F0_3] = REG_RD_ACCESS | REG_WR_ACCESS,
};
/******************************************************
 *
 * functions
 *
 ******************************************************/
void aw8697x_interrupt_clear(struct aw8697 *aw8697x);
static int aw8697x_haptic_trig_enable_config(struct aw8697 *aw8697x);

 /******************************************************
 *
 * aw8697x i2c write/read
 *
 ******************************************************/
static int aw8697x_i2c_write(struct aw8697 *aw8697x,
			    unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret =
		    i2c_smbus_write_byte_data(aw8697x->i2c, reg_addr, reg_data);
		if (ret < 0) {
			pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000,
			     AW_I2C_RETRY_DELAY * 1000 + 500);
	}

	return ret;
}

static int aw8697x_i2c_read(struct aw8697 *aw8697x,
			   unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw8697x->i2c, reg_addr);
		if (ret < 0) {
			pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt,
			       ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		usleep_range(AW_I2C_RETRY_DELAY * 1000,
			     AW_I2C_RETRY_DELAY * 1000 + 500);
	}

	return ret;
}

static int aw8697x_i2c_write_bits(struct aw8697 *aw8697x,
				 unsigned char reg_addr, unsigned int mask,
				 unsigned char reg_data)
{
	unsigned char reg_val = 0;

	aw8697x_i2c_read(aw8697x, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= reg_data;
	aw8697x_i2c_write(aw8697x, reg_addr, reg_val);

	return 0;
}

static int aw8697x_i2c_writes(struct aw8697 *aw8697x,
			     unsigned char reg_addr, unsigned char *buf,
			     unsigned int len)
{
	int ret = -1;
	unsigned char *data;

	data = kmalloc(len + 1, GFP_KERNEL);
	if (data == NULL) {
		pr_err("%s: can not allocate memory\n", __func__);
		return -ENOMEM;
	}

	data[0] = reg_addr;
	memcpy(&data[1], buf, len);

	ret = i2c_master_send(aw8697x->i2c, data, len + 1);
	if (ret < 0)
		pr_err("%s: i2c master send error\n", __func__);

	kfree(data);

	return ret;
}

static void aw8697x_haptic_boost(struct aw8697 *aw8697x)
{
	if (aw8697x->index ==  2 || aw8697x->index == 10)
		aw8697x->boost_en = false;
	else
		aw8697x->boost_en = true;
}
/*****************************************************
 *
 * ram update
 *
 *****************************************************/
static void aw8697x_rtp_loaded(const struct firmware *cont, void *context)
{
	struct aw8697 *aw8697x = context;

	pr_debug("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__,
		       aw8697x_rtp_name[aw8697x->rtp_file_num]);
		release_firmware(cont);
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__,
		aw8697x_rtp_name[aw8697x->rtp_file_num], cont ? cont->size : 0);

	/* aw8697x rtp update */
	mutex_lock(&aw8697x->rtp_lock);
	aw8697x_rtp = vmalloc(cont->size + sizeof(int));
	if (!aw8697x_rtp) {
		release_firmware(cont);
		mutex_unlock(&aw8697x->rtp_lock);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw8697x_rtp->len = cont->size;
	pr_debug("%s: rtp size = %d\n", __func__, aw8697x_rtp->len);
	memcpy(aw8697x_rtp->data, cont->data, cont->size);
	release_firmware(cont);
	mutex_unlock(&aw8697x->rtp_lock);

	aw8697x->rtp_init = 1;
	pr_debug("%s: rtp update complete\n", __func__);
}

static int aw8697x_rtp_update(struct aw8697 *aw8697x)
{
	pr_debug("%s enter\n", __func__);

	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw8697x_rtp_name[aw8697x->rtp_file_num],
				       aw8697x->dev, GFP_KERNEL, aw8697x,
				       aw8697x_rtp_loaded);
}

static void aw8697x_container_update(struct aw8697 *aw8697x,
				    struct aw8697_container *aw8697x_cont)
{
	int i = 0;
	unsigned int shift = 0;

	pr_debug("%s enter\n", __func__);

	mutex_lock(&aw8697x->lock);

	aw8697x->ram.baseaddr_shift = 2;
	aw8697x->ram.ram_shift = 4;

	/* RAMINIT Enable */
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_EN);

	/* base addr */
	shift = aw8697x->ram.baseaddr_shift;
	aw8697x->ram.base_addr =
	    (unsigned int)((aw8697x_cont->data[0 + shift] << 8) |
			   (aw8697x_cont->data[1 + shift]));
	pr_debug("%s: base_addr=0x%4x\n", __func__, aw8697x->ram.base_addr);

	aw8697x_i2c_write(aw8697x, AW8697_REG_BASE_ADDRH,
			 aw8697x_cont->data[0 + shift]);
	aw8697x_i2c_write(aw8697x, AW8697_REG_BASE_ADDRL,
			 aw8697x_cont->data[1 + shift]);
	/*1/2 FIFO */
	aw8697x_i2c_write(aw8697x, AW8697_REG_FIFO_AEH,
			(unsigned char)((aw8697x->ram.base_addr >> 1) >> 8));
	/*1/2 FIFO */
	aw8697x_i2c_write(aw8697x, AW8697_REG_FIFO_AEL,
			(unsigned char)((aw8697x->ram.base_addr >> 1) & 0x00FF));
	aw8697x_i2c_write(aw8697x, AW8697_REG_FIFO_AFH,
			 (unsigned
			  char)((aw8697x->ram.base_addr -
				 (aw8697x->ram.base_addr >> 2)) >> 8));
	aw8697x_i2c_write(aw8697x, AW8697_REG_FIFO_AFL,
			 (unsigned
			  char)((aw8697x->ram.base_addr -
				 (aw8697x->ram.base_addr >> 2)) & 0x00FF));

	/* ram */
	shift = aw8697x->ram.baseaddr_shift;
	aw8697x_i2c_write(aw8697x, AW8697_REG_RAMADDRH,
			 aw8697x_cont->data[0 + shift]);
	aw8697x_i2c_write(aw8697x, AW8697_REG_RAMADDRL,
			 aw8697x_cont->data[1 + shift]);
	shift = aw8697x->ram.ram_shift;
	for (i = shift; i < aw8697x_cont->len; i++) {
		aw8697x->ramupdate_flag =
		    aw8697x_i2c_write(aw8697x, AW8697_REG_RAMDATA,
				     aw8697x_cont->data[i]);
	}

#if 0
	/* ram check */
	shift = aw8697x->ram.baseaddr_shift;
	aw8697x_i2c_write(aw8697x, AW8697_REG_RAMADDRH,
			 aw8697x_cont->data[0 + shift]);
	aw8697x_i2c_write(aw8697x, AW8697_REG_RAMADDRL,
			 aw8697x_cont->data[1 + shift]);
	shift = aw8697x->ram.ram_shift;
	for (i = shift; i < aw8697x_cont->len; i++) {
		aw8697x_i2c_read(aw8697x, AW8697_REG_RAMDATA, &reg_val);
		if (reg_val != aw8697x_cont->data[i]) {
			pr_err
			("%s: ram check error addr=0x%04x, file_data=0x%02x, ram_data=0x%02x\n",
			__func__, i, aw8697x_cont->data[i], reg_val);
			return;
		}
	}
#endif

	/* RAMINIT Disable */
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_OFF);

	mutex_unlock(&aw8697x->lock);

	pr_debug("%s exit\n", __func__);
}

static void aw8697x_ram_loaded(const struct firmware *cont, void *context)
{
	struct aw8697 *aw8697x = context;
	struct aw8697_container *aw8697x_fw;
	int i = 0;
	unsigned short check_sum = 0;

#ifdef AWINIC_READ_BIN_FLEXBALLY
	static unsigned char load_cont;
	int ram_timer_val = 1000;

	load_cont++;
#endif
	pr_debug("%s enter\n", __func__);

	if (!cont) {
		pr_err("%s: failed to read %s\n", __func__, aw8697x_ram_name);
		release_firmware(cont);
#ifdef AWINIC_READ_BIN_FLEXBALLY
		if (load_cont <= 20) {
			schedule_delayed_work(&aw8697x->ram_work,
					      msecs_to_jiffies(ram_timer_val));
			pr_info("%s:start hrtimer:load_cont%d\n", __func__,
				load_cont);
		}
#endif
		return;
	}

	pr_info("%s: loaded %s - size: %zu\n", __func__, aw8697x_ram_name,
		cont ? cont->size : 0);
/*
	for(i=0; i<cont->size; i++) {
		pr_info("%s: addr:0x%04x, data:0x%02x\n", __func__, i, *(cont->data+i));
	}
*/

	/* check sum */
	for (i = 2; i < cont->size; i++)
		check_sum += cont->data[i];
	if (check_sum !=
	    (unsigned short)((cont->data[0] << 8) | (cont->data[1]))) {
		pr_err("%s: check sum err: check_sum=0x%04x\n", __func__,
		       check_sum);
		return;
	} else {
		pr_info("%s: check sum pass : 0x%04x\n", __func__, check_sum);
		aw8697x->ram.check_sum = check_sum;
	}

	/* aw8697x ram update */
	aw8697x_fw = kzalloc(cont->size + sizeof(int), GFP_KERNEL);
	if (!aw8697x_fw) {
		release_firmware(cont);
		pr_err("%s: Error allocating memory\n", __func__);
		return;
	}
	aw8697x_fw->len = cont->size;
	memcpy(aw8697x_fw->data, cont->data, cont->size);
	release_firmware(cont);

	aw8697x_container_update(aw8697x, aw8697x_fw);

	aw8697x->ram.len = aw8697x_fw->len;

	kfree(aw8697x_fw);

	aw8697x->ram_init = 1;
	pr_debug("%s: fw update complete\n", __func__);

	aw8697x_haptic_trig_enable_config(aw8697x);

	aw8697x_rtp_update(aw8697x);
}

static int aw8697x_ram_update(struct aw8697 *aw8697x)
{
	aw8697x->ram_init = 0;
	aw8697x->rtp_init = 0;
	return request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
				       aw8697x_ram_name, aw8697x->dev, GFP_KERNEL,
				       aw8697x, aw8697x_ram_loaded);
}

#ifdef AWINIC_RAM_UPDATE_DELAY
static void aw8697x_ram_work_routine(struct work_struct *work)
{
	struct aw8697 *aw8697x =
	    container_of(work, struct aw8697, ram_work.work);

	pr_debug("%s enter\n", __func__);

	aw8697x_ram_update(aw8697x);

}
#endif

static int aw8697x_ram_init(struct aw8697 *aw8697x)
{
#ifdef AWINIC_RAM_UPDATE_DELAY
	int ram_timer_val = 5000;

	INIT_DELAYED_WORK(&aw8697x->ram_work, aw8697x_ram_work_routine);
	schedule_delayed_work(&aw8697x->ram_work,
			      msecs_to_jiffies(ram_timer_val));
#else
	aw8697x_ram_update(aw8697x);
#endif
	return 0;
}

/*****************************************************
 *
 * haptic control
 *
 *****************************************************/
static int aw8697x_haptic_softreset(struct aw8697 *aw8697x)
{
	pr_debug("%s enter\n", __func__);

	aw8697x_i2c_write(aw8697x, AW8697_REG_ID, 0xAA);
	usleep_range(3000, 3500);
	return 0;
}

static int aw8697x_haptic_active(struct aw8697 *aw8697x)
{
	pr_debug("%s enter\n", __func__);

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
			      AW8697_BIT_SYSCTRL_ACTIVE);
	aw8697x_interrupt_clear(aw8697x);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_UVLO_MASK,
			      AW8697_BIT_SYSINTM_UVLO_EN);
	return 0;
}

int aw8697x_haptic_play_mode(struct aw8697 *aw8697x,
				   unsigned char play_mode)
{
	pr_debug("%s enter\n", __func__);

	aw8697x_haptic_boost(aw8697x);
	switch (play_mode) {
	case AW8697_HAPTIC_STANDBY_MODE:
		aw8697x->play_mode = AW8697_HAPTIC_STANDBY_MODE;
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
				      AW8697_BIT_SYSINTM_UVLO_MASK,
				      AW8697_BIT_SYSINTM_UVLO_OFF);
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
				      AW8697_BIT_SYSCTRL_STANDBY);
		break;
	case AW8697_HAPTIC_RAM_MODE:
		aw8697x->play_mode = AW8697_HAPTIC_RAM_MODE;
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
		/* aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
				      AW8697_BIT_SYSCTRL_BST_MODE_BYPASS); */
		if (aw8697x->auto_boost) {
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8697x_haptic_active(aw8697x);
		if (aw8697x->auto_boost) {
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RAM_ENABLE);
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
					      AW8697_BIT_SYSCTRL_BST_MODE_MASK &
					      AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
					      AW8697_BIT_SYSCTRL_BST_MODE_BOOST
					      | AW8697_BIT_SYSCTRL_STANDBY);
			aw8697x_haptic_active(aw8697x);
		} else {
			if (aw8697x->boost_en == true) {
				aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
						AW8697_BIT_SYSCTRL_BST_MODE_MASK,
						AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
			}
		}
		usleep_range(2000, 2500);
		break;
	case AW8697_HAPTIC_RAM_LOOP_MODE:
		aw8697x->play_mode = AW8697_HAPTIC_RAM_LOOP_MODE;
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
				      AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8697x->auto_boost) {
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8697x_haptic_active(aw8697x);
		break;
	case AW8697_HAPTIC_RTP_MODE:
		aw8697x->play_mode = AW8697_HAPTIC_RTP_MODE;
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_RTP);
		/* aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
				      AW8697_BIT_SYSCTRL_BST_MODE_BYPASS); */
		if (aw8697x->auto_boost) {
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RTP_DISABLE);
		}
		aw8697x_haptic_active(aw8697x);
		if (aw8697x->auto_boost) {
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RTP_ENABLE);
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
					      AW8697_BIT_SYSCTRL_BST_MODE_MASK &
					      AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
					      AW8697_BIT_SYSCTRL_BST_MODE_BOOST
					      | AW8697_BIT_SYSCTRL_STANDBY);
			aw8697x_haptic_active(aw8697x);
		} else {
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
					AW8697_BIT_SYSCTRL_BST_MODE_MASK,
					AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		usleep_range(2000, 2500);
		break;
	case AW8697_HAPTIC_TRIG_MODE:
		aw8697x->play_mode = AW8697_HAPTIC_TRIG_MODE;
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
		/* aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
				      AW8697_BIT_SYSCTRL_BST_MODE_BYPASS); */
		if (aw8697x->auto_boost) {
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8697x_haptic_active(aw8697x);
		if (aw8697x->auto_boost) {
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RAM_ENABLE);
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
					      AW8697_BIT_SYSCTRL_BST_MODE_MASK &
					      AW8697_BIT_SYSCTRL_WORK_MODE_MASK,
					      AW8697_BIT_SYSCTRL_BST_MODE_BOOST
					      | AW8697_BIT_SYSCTRL_STANDBY);
			aw8697x_haptic_active(aw8697x);
		} else {
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
					AW8697_BIT_SYSCTRL_BST_MODE_MASK,
					AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
		}
		usleep_range(2000, 2500);
		break;
	case AW8697_HAPTIC_CONT_MODE:
		aw8697x->play_mode = AW8697_HAPTIC_CONT_MODE;
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_MASK,
				      AW8697_BIT_SYSCTRL_PLAY_MODE_CONT);
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
				      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
				      AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
		if (aw8697x->auto_boost) {
			aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BST_AUTO,
					AW8697_BIT_BST_AUTO_BST_RAM_MASK,
					AW8697_BIT_BST_AUTO_BST_RAM_DISABLE);
		}
		aw8697x_haptic_active(aw8697x);
		break;
	default:
		dev_err(aw8697x->dev, "%s: play mode %d err",
			__func__, play_mode);
		break;
	}
	return 0;
}

int aw8697x_haptic_juge_RTP_is_going_on(struct aw8697 *aw8697x)
{
	unsigned char reg_val = 0;
	unsigned char rtp_state = 0;
	static unsigned char pre_reg_val;

	aw8697x_i2c_read(aw8697x, AW8697_REG_SYSCTRL, &reg_val);
	if ((reg_val & AW8697_BIT_SYSCTRL_PLAY_MODE_RTP) &&
		(!(reg_val & AW8697_BIT_SYSCTRL_STANDBY)))
		rtp_state = 1;	/*is going on */
	if (pre_reg_val != reg_val)
		pr_info("%s AW8697_REG_SYSCTRL 0x04==%02x rtp_state=%d\n",
			__func__, reg_val, rtp_state);
	pre_reg_val = reg_val;
	if (aw8697x->rtp_routine_on) {
		pr_info("%s:rtp_routine_on\n", __func__);
		rtp_state = 1;	/*is going on */
	}
	return rtp_state;
}

int aw8697x_haptic_play_go(struct aw8697 *aw8697x, bool flag)
{
	pr_debug("%s enter\n", __func__);
	if (flag == true) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_GO,
			AW8697_BIT_GO_MASK, AW8697_BIT_GO_ENABLE);
		mdelay(2);
	} else {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_GO,
			AW8697_BIT_GO_MASK,AW8697_BIT_GO_DISABLE);
	}
	return 0;
}

static int aw8697x_haptic_stop_delay(struct aw8697 *aw8697x)
{
	unsigned char reg_val = 0;
	unsigned int cnt = 100;

	while (cnt--) {
		aw8697x_i2c_read(aw8697x, AW8697_REG_GLB_STATE, &reg_val);
		if ((reg_val & 0x0f) == 0x00)
			return 0;
		usleep_range(2000, 2500);
		pr_debug("%s wait for standby, reg glb_state=0x%02x\n",
			 __func__, reg_val);
	}
	pr_err("%s do not enter standby automatically\n", __func__);

	return 0;
}

int aw8697x_haptic_stop(struct aw8697 *aw8697x)
{
	unsigned char reg_val = 0;
	unsigned int cnt = 60;
	unsigned int go_disable_times = 0;

	pr_debug("%s enter\n", __func__);
	while (cnt--) {
		aw8697x_i2c_read(aw8697x, AW8697_REG_GLB_STATE, &reg_val);
		if (reg_val == 0){//standby
			aw8697x_haptic_play_mode(aw8697x, AW8697_HAPTIC_STANDBY_MODE);
			return 0;
		} else if (reg_val <= 3){//bringup
			usleep_range(2000, 2500);
			pr_debug("%s bringup, reg glb_state=0x%02x\n",__func__, reg_val);
		} else {//playing
			if (go_disable_times == 0){//go->0 just set one time
				aw8697x_haptic_play_go(aw8697x, false);
				go_disable_times = 1;
			}
			usleep_range(2000, 2500);
			pr_debug("%s playing, reg glb_state=0x%02x\n",__func__, reg_val);
		}
	}

	return 0;
}

static int aw8697x_haptic_start(struct aw8697 *aw8697x)
{
	pr_debug("%s enter\n", __func__);

	aw8697x_haptic_play_go(aw8697x, true);

	return 0;
}

static int aw8697x_haptic_set_wav_seq(struct aw8697 *aw8697x,
				     unsigned char wav, unsigned char seq)
{
	pr_debug("%s, seq = %d\n", __func__, seq);
	aw8697x_i2c_write(aw8697x, AW8697_REG_WAVSEQ1 + wav, seq);
	return 0;
}

static int aw8697x_haptic_set_wav_loop(struct aw8697 *aw8697x,
				      unsigned char wav, unsigned char loop)
{
	unsigned char tmp = 0;

	if (wav % 2) {
		tmp = loop << 0;
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_WAVLOOP1 + (wav / 2),
				      AW8697_BIT_WAVLOOP_SEQNP1_MASK, tmp);
	} else {
		tmp = loop << 4;
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_WAVLOOP1 + (wav / 2),
				      AW8697_BIT_WAVLOOP_SEQN_MASK, tmp);
	}

	return 0;
}

/*
static int aw8697x_haptic_set_main_loop(struct aw8697 *aw8697x,
				       unsigned char loop)
{
	aw8697x_i2c_write(aw8697x, AW8697_REG_MAIN_LOOP, loop);
	return 0;
}
*/

static int aw8697x_haptic_set_repeat_wav_seq(struct aw8697 *aw8697x,
					    unsigned char seq)
{
	aw8697x_haptic_set_wav_seq(aw8697x, 0x00, seq);
	aw8697x_haptic_set_wav_loop(aw8697x, 0x00,
				   AW8697_BIT_WAVLOOP_INIFINITELY);

	return 0;
}

static int aw8697x_haptic_set_bst_vol(struct aw8697 *aw8697x,
				     unsigned char bst_vol)
{
	if (bst_vol & 0xe0)
		bst_vol = 0x1f;
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BSTDBG4,
			      AW8697_BIT_BSTDBG4_BSTVOL_MASK, (bst_vol << 1));
	return 0;
}

static int aw8697x_haptic_set_bst_peak_cur(struct aw8697 *aw8697x,
					  unsigned char peak_cur)
{
	peak_cur &= AW8697_BSTCFG_PEAKCUR_LIMIT;
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BSTCFG,
			      AW8697_BIT_BSTCFG_PEAKCUR_MASK, peak_cur);
	return 0;
}

static int aw8697x_haptic_set_gain(struct aw8697 *aw8697x, unsigned char gain)
{
	pr_debug("%s, gain = 0x%x\n", __func__, gain);
	aw8697x_i2c_write(aw8697x, AW8697_REG_DATDBG, gain);
	return 0;
}

static int aw8697x_haptic_set_pwm(struct aw8697 *aw8697x, unsigned char mode)
{
	switch (mode) {
	case AW8697_PWM_48K:
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_PWMDBG,
				      AW8697_BIT_PWMDBG_PWM_MODE_MASK,
				      AW8697_BIT_PWMDBG_PWM_48K);
		break;
	case AW8697_PWM_24K:
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_PWMDBG,
				      AW8697_BIT_PWMDBG_PWM_MODE_MASK,
				      AW8697_BIT_PWMDBG_PWM_24K);
		break;
	case AW8697_PWM_12K:
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_PWMDBG,
				      AW8697_BIT_PWMDBG_PWM_MODE_MASK,
				      AW8697_BIT_PWMDBG_PWM_12K);
		break;
	default:
		break;
	}
	return 0;
}

static int aw8697x_haptic_play_wav_seq(struct aw8697 *aw8697x, unsigned char flag)
{
	pr_debug("%s enter\n", __func__);

	if (flag) {
		aw8697x_haptic_play_mode(aw8697x, AW8697_HAPTIC_RAM_MODE);
		aw8697x_haptic_start(aw8697x);
	}
	return 0;
}

static int aw8697x_haptic_play_repeat_seq(struct aw8697 *aw8697x,
					 unsigned char flag)
{
	pr_debug("%s enter\n", __func__);

	if (flag) {
		aw8697x_haptic_play_mode(aw8697x, AW8697_HAPTIC_RAM_LOOP_MODE);
		aw8697x_haptic_start(aw8697x);
	}

	return 0;
}

/*****************************************************
 *
 * motor protect
 *
 *****************************************************/
static int aw8697x_haptic_swicth_motorprotect_config(struct aw8697 *aw8697x,
						    unsigned char addr,
						    unsigned char val)
{
	pr_debug("%s enter\n", __func__);
	if (addr == 1) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_DETCTRL,
				      AW8697_BIT_DETCTRL_PROTECT_MASK,
				      AW8697_BIT_DETCTRL_PROTECT_SHUTDOWN);
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_PWMPRC,
				      AW8697_BIT_PWMPRC_PRC_MASK,
				      AW8697_BIT_PWMPRC_PRC_ENABLE);
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_PRLVL,
				      AW8697_BIT_PRLVL_PR_MASK,
				      AW8697_BIT_PRLVL_PR_ENABLE);
	} else if (addr == 0) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_DETCTRL,
				      AW8697_BIT_DETCTRL_PROTECT_MASK,
				      AW8697_BIT_DETCTRL_PROTECT_NO_ACTION);
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_PWMPRC,
				      AW8697_BIT_PWMPRC_PRC_MASK,
				      AW8697_BIT_PWMPRC_PRC_DISABLE);
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_PRLVL,
				      AW8697_BIT_PRLVL_PR_MASK,
				      AW8697_BIT_PRLVL_PR_DISABLE);
	} else if (addr == 0x2d) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_PWMPRC,
				      AW8697_BIT_PWMPRC_PRCTIME_MASK, val);
	} else if (addr == 0x3e) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_PRLVL,
				      AW8697_BIT_PRLVL_PRLVL_MASK, val);
	} else if (addr == 0x3f) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_PRTIME,
				      AW8697_BIT_PRTIME_PRTIME_MASK, val);
	} else {
		/*nothing to do; */
	}
	return 0;
}

/*****************************************************
 *
 * offset calibration
 *
 *****************************************************/
static int aw8697x_haptic_offset_calibration(struct aw8697 *aw8697x)
{
	unsigned int cont = 2000;
	unsigned char reg_val = 0;

	pr_debug("%s enter\n", __func__);

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_EN);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_DETCTRL,
			      AW8697_BIT_DETCTRL_DIAG_GO_MASK,
			      AW8697_BIT_DETCTRL_DIAG_GO_ENABLE);
	while (1) {
		aw8697x_i2c_read(aw8697x, AW8697_REG_DETCTRL, &reg_val);
		if ((reg_val & 0x01) == 0 || cont == 0)
			break;
		cont--;
	}
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_OFF);

	return 0;
}

/*****************************************************
 *
 * trig config
 *
 *****************************************************/
static int aw8697x_haptic_trig_param_init(struct aw8697 *aw8697x)
{
	pr_debug("%s enter\n", __func__);

	aw8697x->trig[0].enable = AW8697_TRG1_ENABLE;
	aw8697x->trig[0].default_level = AW8697_TRG1_DEFAULT_LEVEL;
	aw8697x->trig[0].dual_edge = AW8697_TRG1_DUAL_EDGE;
	aw8697x->trig[0].frist_seq = AW8697_TRG1_FIRST_EDGE_SEQ;
	aw8697x->trig[0].second_seq = AW8697_TRG1_SECOND_EDGE_SEQ;

	aw8697x->trig[1].enable = AW8697_TRG2_ENABLE;
	aw8697x->trig[1].default_level = AW8697_TRG2_DEFAULT_LEVEL;
	aw8697x->trig[1].dual_edge = AW8697_TRG2_DUAL_EDGE;
	aw8697x->trig[1].frist_seq = AW8697_TRG2_FIRST_EDGE_SEQ;
	aw8697x->trig[1].second_seq = AW8697_TRG2_SECOND_EDGE_SEQ;

	aw8697x->trig[2].enable = AW8697_TRG3_ENABLE;
	aw8697x->trig[2].default_level = AW8697_TRG3_DEFAULT_LEVEL;
	aw8697x->trig[2].dual_edge = AW8697_TRG3_DUAL_EDGE;
	aw8697x->trig[2].frist_seq = AW8697_TRG3_FIRST_EDGE_SEQ;
	aw8697x->trig[2].second_seq = AW8697_TRG3_SECOND_EDGE_SEQ;

	return 0;
}

static int aw8697x_haptic_trig_param_config(struct aw8697 *aw8697x)
{
	pr_debug("%s enter\n", __func__);

	if (aw8697x->trig[0].default_level) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG1_POLAR_MASK,
				      AW8697_BIT_TRGCFG1_TRG1_POLAR_NEG);
	} else {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG1_POLAR_MASK,
				      AW8697_BIT_TRGCFG1_TRG1_POLAR_POS);
	}
	if (aw8697x->trig[1].default_level) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG2_POLAR_MASK,
				      AW8697_BIT_TRGCFG1_TRG2_POLAR_NEG);
	} else {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG2_POLAR_MASK,
				      AW8697_BIT_TRGCFG1_TRG2_POLAR_POS);
	}
	if (aw8697x->trig[2].default_level) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG3_POLAR_MASK,
				      AW8697_BIT_TRGCFG1_TRG3_POLAR_NEG);
	} else {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG3_POLAR_MASK,
				      AW8697_BIT_TRGCFG1_TRG3_POLAR_POS);
	}

	if (aw8697x->trig[0].dual_edge) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG1_EDGE_MASK,
				      AW8697_BIT_TRGCFG1_TRG1_EDGE_POS_NEG);
	} else {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG1_EDGE_MASK,
				      AW8697_BIT_TRGCFG1_TRG1_EDGE_POS);
	}
	if (aw8697x->trig[1].dual_edge) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG2_EDGE_MASK,
				      AW8697_BIT_TRGCFG1_TRG2_EDGE_POS_NEG);
	} else {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG2_EDGE_MASK,
				      AW8697_BIT_TRGCFG1_TRG2_EDGE_POS);
	}
	if (aw8697x->trig[2].dual_edge) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG3_EDGE_MASK,
				      AW8697_BIT_TRGCFG1_TRG3_EDGE_POS_NEG);
	} else {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG1,
				      AW8697_BIT_TRGCFG1_TRG3_EDGE_MASK,
				      AW8697_BIT_TRGCFG1_TRG3_EDGE_POS);
	}

	if (aw8697x->trig[0].frist_seq) {
		aw8697x_i2c_write(aw8697x, AW8697_REG_TRG1_WAV_P,
				 aw8697x->trig[0].frist_seq);
	}
	if (aw8697x->trig[0].second_seq && aw8697x->trig[0].dual_edge) {
		aw8697x_i2c_write(aw8697x, AW8697_REG_TRG1_WAV_N,
				 aw8697x->trig[0].second_seq);
	}
	if (aw8697x->trig[1].frist_seq) {
		aw8697x_i2c_write(aw8697x, AW8697_REG_TRG2_WAV_P,
				 aw8697x->trig[1].frist_seq);
	}
	if (aw8697x->trig[1].second_seq && aw8697x->trig[1].dual_edge) {
		aw8697x_i2c_write(aw8697x, AW8697_REG_TRG2_WAV_N,
				 aw8697x->trig[1].second_seq);
	}
	if (aw8697x->trig[2].frist_seq) {
		aw8697x_i2c_write(aw8697x, AW8697_REG_TRG3_WAV_P,
				 aw8697x->trig[1].frist_seq);
	}
	if (aw8697x->trig[2].second_seq && aw8697x->trig[2].dual_edge) {
		aw8697x_i2c_write(aw8697x, AW8697_REG_TRG3_WAV_N,
				 aw8697x->trig[1].second_seq);
	}

	return 0;
}

static int aw8697x_haptic_trig_enable_config(struct aw8697 *aw8697x)
{
	pr_debug("%s enter\n", __func__);

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG2,
			      AW8697_BIT_TRGCFG2_TRG1_ENABLE_MASK,
			      aw8697x->trig[0].enable);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG2,
			      AW8697_BIT_TRGCFG2_TRG2_ENABLE_MASK,
			      aw8697x->trig[1].enable);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_TRG_CFG2,
			      AW8697_BIT_TRGCFG2_TRG3_ENABLE_MASK,
			      aw8697x->trig[2].enable);

	return 0;
}

static int aw8697x_haptic_auto_boost_config(struct aw8697 *aw8697x,
					   unsigned char flag)
{
	aw8697x->auto_boost = flag;
	if (flag) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BST_AUTO,
				      AW8697_BIT_BST_AUTO_BST_AUTOSW_MASK,
				      AW8697_BIT_BST_AUTO_BST_AUTOMATIC_BOOST);
	} else {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BST_AUTO,
				      AW8697_BIT_BST_AUTO_BST_AUTOSW_MASK,
				      AW8697_BIT_BST_AUTO_BST_MANUAL_BOOST);
	}
	return 0;
}

/*****************************************************
 *
 * vbat mode
 *
 *****************************************************/
static int aw8697x_haptic_cont_vbat_mode(struct aw8697 *aw8697x,
					unsigned char flag)
{
	if (flag == AW8697_HAPTIC_CONT_VBAT_HW_COMP_MODE) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_ADCTEST,
				      AW8697_BIT_ADCTEST_VBAT_MODE_MASK,
				      AW8697_BIT_ADCTEST_VBAT_HW_COMP);
	} else {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_ADCTEST,
				      AW8697_BIT_ADCTEST_VBAT_MODE_MASK,
				      AW8697_BIT_ADCTEST_VBAT_SW_COMP);
	}
	return 0;
}

static int aw8697x_haptic_get_vbat(struct aw8697 *aw8697x)
{
	unsigned char reg_val = 0;
	unsigned int cont = 2000;

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_EN);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_DETCTRL,
			      AW8697_BIT_DETCTRL_VBAT_GO_MASK,
			      AW8697_BIT_DETCTRL_VABT_GO_ENABLE);

	while (1) {
		aw8697x_i2c_read(aw8697x, AW8697_REG_DETCTRL, &reg_val);
		if ((reg_val & 0x02) == 0 || cont == 0)
			break;
		cont--;
	}

	aw8697x_i2c_read(aw8697x, AW8697_REG_VBATDET, &reg_val);
	aw8697x->vbat = 6100 * reg_val / 256;
	if (aw8697x->vbat > AW8697_VBAT_MAX) {
		aw8697x->vbat = AW8697_VBAT_MAX;
		pr_debug("%s vbat max limit = %dmV\n", __func__, aw8697x->vbat);
	}
	if (aw8697x->vbat < AW8697_VBAT_MIN) {
		aw8697x->vbat = AW8697_VBAT_MIN;
		pr_debug("%s vbat min limit = %dmV\n", __func__, aw8697x->vbat);
	}

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_OFF);

	return 0;
}

int aw8697x_haptic_ram_vbat_comp(struct aw8697 *aw8697x, bool flag)
{
	int temp_gain = 0;

	if (flag) {
		if
		(aw8697x->ram_vbat_comp == AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE) {
			aw8697x_haptic_get_vbat(aw8697x);
			temp_gain =
			    aw8697x->gain * AW8697_VBAT_REFER / aw8697x->vbat;
			if (temp_gain >
			    (128 * AW8697_VBAT_REFER / AW8697_VBAT_MIN)) {
				temp_gain =
				    128 * AW8697_VBAT_REFER / AW8697_VBAT_MIN;
				pr_debug("%s gain limit=%d\n", __func__,
					 temp_gain);
			}
			aw8697x_haptic_set_gain(aw8697x, temp_gain);
		} else {
			aw8697x_haptic_set_gain(aw8697x, aw8697x->gain);
		}
	} else {
		aw8697x_haptic_set_gain(aw8697x, aw8697x->gain);
	}

	return 0;
}

/*****************************************************
 *
 * f0
 *
 *****************************************************/
static int aw8697x_haptic_set_f0_preset(struct aw8697 *aw8697x)
{
	unsigned int f0_reg = 0;

	pr_debug("%s enter\n", __func__);

	f0_reg = 1000000000 / (aw8697x->info.f0_pre * aw8697x->info.f0_coeff);
	aw8697x_i2c_write(aw8697x, AW8697_REG_F_PRE_H,
			 (unsigned char)((f0_reg >> 8) & 0xff));
	aw8697x_i2c_write(aw8697x, AW8697_REG_F_PRE_L,
			 (unsigned char)((f0_reg >> 0) & 0xff));

	return 0;
}

static int aw8697x_haptic_read_f0(struct aw8697 *aw8697x)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	pr_debug("%s enter\n", __func__);

	ret = aw8697x_i2c_read(aw8697x, AW8697_REG_F_LRA_F0_H, &reg_val);
	f0_reg = (reg_val << 8);
	ret = aw8697x_i2c_read(aw8697x, AW8697_REG_F_LRA_F0_L, &reg_val);
	f0_reg |= (reg_val << 0);
	if (!f0_reg) {
		pr_info("%s not get f0 because f0_reg value is 0!\n", __func__);
		return 0;
	}
	f0_tmp = 1000000000 / (f0_reg * aw8697x->info.f0_coeff);
	aw8697x->f0 = (unsigned int)f0_tmp;
	pr_debug("%s f0=%d\n", __func__, aw8697x->f0);

	return 0;
}

static int aw8697x_haptic_read_cont_f0(struct aw8697 *aw8697x)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_reg = 0;
	unsigned long f0_tmp = 0;

	pr_debug("%s enter\n", __func__);

	ret = aw8697x_i2c_read(aw8697x, AW8697_REG_F_LRA_CONT_H, &reg_val);
	f0_reg = (reg_val << 8);
	ret = aw8697x_i2c_read(aw8697x, AW8697_REG_F_LRA_CONT_L, &reg_val);
	f0_reg |= (reg_val << 0);
	f0_tmp = 1000000000 / (f0_reg * aw8697x->info.f0_coeff);
	aw8697x->cont_f0 = (unsigned int)f0_tmp;
	pr_debug("%s f0=%d\n", __func__, aw8697x->cont_f0);

	return 0;
}

static int aw8697x_haptic_read_beme(struct aw8697 *aw8697x)
{
	int ret = 0;
	unsigned char reg_val = 0;

	ret = aw8697x_i2c_read(aw8697x, AW8697_REG_WAIT_VOL_MP, &reg_val);
	aw8697x->max_pos_beme = (reg_val << 0);
	ret = aw8697x_i2c_read(aw8697x, AW8697_REG_WAIT_VOL_MN, &reg_val);
	aw8697x->max_neg_beme = (reg_val << 0);

	pr_debug("%s max_pos_beme=%d\n", __func__, aw8697x->max_pos_beme);
	pr_debug("%s max_neg_beme=%d\n", __func__, aw8697x->max_neg_beme);

	return 0;
}

/*****************************************************
 *
 * rtp
 *
 *****************************************************/
void aw8697x_haptic_set_rtp_aei(struct aw8697 *aw8697x, bool flag)
{
	if (flag) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
				      AW8697_BIT_SYSINTM_FF_AE_MASK,
				      AW8697_BIT_SYSINTM_FF_AE_EN);
	} else {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
				      AW8697_BIT_SYSINTM_FF_AE_MASK,
				      AW8697_BIT_SYSINTM_FF_AE_OFF);
	}
}

/*
static void aw8697x_haptic_set_rtp_afi(struct aw8697 *aw8697x, bool flag)
{
	if(flag) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
		AW8697_BIT_SYSINTM_FF_AF_MASK, AW8697_BIT_SYSINTM_FF_AF_EN);
	} else {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
		AW8697_BIT_SYSINTM_FF_AF_MASK, AW8697_BIT_SYSINTM_FF_AF_OFF);
	}
}
*/

/*
static unsigned char aw8697x_haptic_rtp_get_fifo_aei(struct aw8697 *aw8697x)
{
	unsigned char ret;
	unsigned char reg_val;

	aw8697x_i2c_read(aw8697x, AW8697_REG_SYSINT, &reg_val);
	reg_val &= AW8697_BIT_SYSINT_FF_AEI;
	ret = reg_val>>4;

	return ret;
}
*/

/*
static unsigned char aw8697x_haptic_rtp_get_fifo_aes(struct aw8697 *aw8697x)
{
	unsigned char ret;
	unsigned char reg_val;

	aw8697x_i2c_read(aw8697x, AW8697_REG_SYSST, &reg_val);
	reg_val &= AW8697_BIT_SYSST_FF_AES;
	ret = reg_val>>4;

	return ret;
}
*/

static unsigned char aw8697x_haptic_rtp_get_fifo_afi(struct aw8697 *aw8697x)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;

	if (aw8697x->osc_cali_flag == 1) {
		aw8697x_i2c_read(aw8697x, AW8697_REG_SYSST, &reg_val);
		reg_val &= AW8697_BIT_SYSST_FF_AFS;
		ret = reg_val >> 3;
	} else {
		aw8697x_i2c_read(aw8697x, AW8697_REG_SYSINT, &reg_val);
		reg_val &= AW8697_BIT_SYSINT_FF_AFI;
		ret = reg_val >> 3;
	}
	return ret;
}

static unsigned char aw8697x_haptic_rtp_get_fifo_afs(struct aw8697 *aw8697x)
{
	unsigned char ret = 0;
	unsigned char reg_val = 0;

	aw8697x_i2c_read(aw8697x, AW8697_REG_SYSST, &reg_val);
	reg_val &= AW8697_BIT_SYSST_FF_AFS;
	ret = reg_val>>3;

	return ret;
}

/*****************************************************
 *
 * rtp
 *
 *****************************************************/
static int aw8697x_haptic_rtp_init(struct aw8697 *aw8697x)
{
	unsigned int buf_len = 0;
	unsigned char reg_val = 0;

	pr_debug("%s enter\n", __func__);
	pm_qos_add_request(&pm_qos_req_vb_x, PM_QOS_CPU_DMA_LATENCY, PM_QOS_VALUE_VB);
	aw8697x->rtp_cnt = 0;
	mutex_lock(&aw8697x->rtp_lock);
	while ((!aw8697x_haptic_rtp_get_fifo_afs(aw8697x)) &&
	       (aw8697x->play_mode == AW8697_HAPTIC_RTP_MODE) && g_aw8697x_rtp_going) {
		/* pr_debug("%s rtp cnt = %d\n", __func__, aw8697x->rtp_cnt); */
		if (!aw8697x_rtp) {
			pr_info("%s:aw8697x_rtp is null break\n", __func__);
			mutex_unlock(&aw8697x->rtp_lock);
			break;
		}
		if ((aw8697x->rtp_cnt < aw8697x->ram.base_addr)) {
			if((aw8697x_rtp->len-aw8697x->rtp_cnt) < (aw8697x->ram.base_addr)) {
				buf_len = aw8697x_rtp->len-aw8697x->rtp_cnt;
			} else {
				buf_len = (aw8697x->ram.base_addr);
			}
		} else if ((aw8697x_rtp->len - aw8697x->rtp_cnt) < (aw8697x->ram.base_addr >> 2)) {
			buf_len = aw8697x_rtp->len - aw8697x->rtp_cnt;
		} else {
			buf_len = (aw8697x->ram.base_addr >> 2);
		}

		aw8697x_i2c_writes(aw8697x, AW8697_REG_RTP_DATA,
				  &aw8697x_rtp->data[aw8697x->rtp_cnt], buf_len);
		aw8697x->rtp_cnt += buf_len;

		aw8697x_i2c_read(aw8697x, AW8697_REG_GLB_STATE, &reg_val);
              if ((aw8697x->rtp_cnt == aw8697x_rtp->len) || ((reg_val & 0x0f) == 0x00)) {
			pr_info("%s: rtp update complete\n", __func__);
			aw8697x->rtp_cnt = 0;
			mutex_unlock(&aw8697x->rtp_lock);
			pm_qos_remove_request(&pm_qos_req_vb_x);
			return 0;
		}
	}
	mutex_unlock(&aw8697x->rtp_lock);

	if (aw8697x->play_mode == AW8697_HAPTIC_RTP_MODE)
		aw8697x_haptic_set_rtp_aei(aw8697x, true);

	pr_debug("%s exit\n", __func__);
	pm_qos_remove_request(&pm_qos_req_vb_x);
	return 0;
}

void aw8697x_haptic_upload_lra(struct aw8697 *aw8697x, unsigned int flag)
{
	switch (flag) {
	case 1:
		pr_debug("%s f0_cali_lra=%d\n", __func__, aw8697x->f0_calib_data);
		aw8697x_i2c_write(aw8697x, AW8697_REG_TRIM_LRA,
				 (char)aw8697x->f0_calib_data);
		break;
	case 2:
		pr_debug("%s rtp_cali_lra=%d\n", __func__, aw8697x->lra_calib_data);
		aw8697x_i2c_write(aw8697x, AW8697_REG_TRIM_LRA,
				 (char)aw8697x->lra_calib_data);
		break;
	default:
		break;
	}
}

static int aw8697x_clock_OSC_trim_calibration(unsigned long int theory_time, unsigned long int real_time)
{
	unsigned int real_code = 0;
	unsigned int LRA_TRIM_CODE = 0;
	unsigned int DFT_LRA_TRIM_CODE = 0;
	unsigned int Not_need_cali_threshold = 10;/*0.1 percent not need calibrate*/

	if (theory_time == real_time) {
		pr_info("aw_osctheory_time == real_time:%ld  theory_time = %ld not need to cali\n", real_time, theory_time);
		return 0;
	} else if (theory_time < real_time) {
		if ((real_time - theory_time) > (theory_time / 25)) {
			pr_info("aw_osc(real_time - theory_time) > (theory_time/50) not to cali\n");
			return DFT_LRA_TRIM_CODE;
		}

		if ((real_time - theory_time) < (Not_need_cali_threshold*theory_time/10000)) {
			pr_info("aw_oscmicrosecond:%ld  theory_time = %ld not need to cali\n", real_time, theory_time);
			return DFT_LRA_TRIM_CODE;
		}

		real_code = ((real_time - theory_time) * 4000) / theory_time;
		real_code = ((real_code%10 < 5) ? 0 : 1) + real_code/10;
		real_code = 32 + real_code;
	} else if (theory_time > real_time) {
		if ((theory_time - real_time) > (theory_time / 25)) {
			pr_info("aw_osc((theory_time - real_time) > (theory_time / 50)) not to cali\n");
			return DFT_LRA_TRIM_CODE;
		}
		if ((theory_time - real_time) < (Not_need_cali_threshold * theory_time/10000)) {
			pr_info("aw_oscmicrosecond:%ld  theory_time = %ld not need to cali\n", real_time, theory_time);
			return DFT_LRA_TRIM_CODE;
		}
		real_code = ((theory_time - real_time) * 4000) / theory_time;
		real_code = ((real_code%10 < 5) ? 0 : 1) + real_code/10;
		real_code = 32 - real_code;
	}
	if (real_code > 31)
		LRA_TRIM_CODE = real_code - 32;
	else
		LRA_TRIM_CODE = real_code + 32;
	pr_info("aw_oscmicrosecond:%ld  theory_time = %ld real_code =0X%02X LRA_TRIM_CODE 0X%02X\n", real_time, theory_time, real_code, LRA_TRIM_CODE);

	return LRA_TRIM_CODE;
}

static int aw8697x_rtp_trim_lra_calibration(struct aw8697 *aw8697x)
{
	unsigned char reg_val = 0;
	unsigned int fre_val = 0;
	unsigned int theory_time = 0;
	unsigned int lra_rtim_code = 0;

	aw8697x_i2c_read(aw8697x, AW8697_REG_PWMDBG, &reg_val);
	fre_val = (reg_val & 0x006f) >> 5;

	if (fre_val == 3)
		theory_time = (aw8697x->rtp_len / 12000) * 1000000; /*12K */
	if (fre_val == 2)
		theory_time = (aw8697x->rtp_len / 24000) * 1000000; /*24K */
	if (fre_val == 1 || fre_val == 0)
		theory_time = (aw8697x->rtp_len / 48000) * 1000000; /*48K */

	printk("microsecond:%ld  theory_time = %d\n", aw8697x->microsecond, theory_time);

	lra_rtim_code = aw8697x_clock_OSC_trim_calibration(theory_time, aw8697x->microsecond);
	if (lra_rtim_code >= 0) {
		aw8697x->lra_calib_data = lra_rtim_code;
		aw8697x_i2c_write(aw8697x, AW8697_REG_TRIM_LRA, (char)lra_rtim_code);
	}
	return 0;
}
static unsigned char aw8697x_haptic_osc_read_int(struct aw8697 *aw8697x)
{
	unsigned char reg_val = 0;

	aw8697x_i2c_read(aw8697x, AW8697_REG_DBGSTAT, &reg_val);
	return reg_val;
}

static int aw8697x_rtp_osc_calibration(struct aw8697 *aw8697x)
{
	const struct firmware *rtp_file;
	int ret = -1;
	unsigned int buf_len = 0;
	unsigned char osc_int_state = 0;
	aw8697x->rtp_cnt = 0;
	aw8697x->timeval_flags = 1;
	aw8697x->osc_cali_flag = 1;

	pr_debug("%s enter\n", __func__);
	/* fw loaded */
	ret = request_firmware(&rtp_file,
			       aw8697x_rtp_name[0],/*aw8697x->rtp_file_num */
			       aw8697x->dev);
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__,
		       aw8697x_rtp_name[0]);/*aw8697x->rtp_file_num */
		return ret;
	}
	/*awinic add stop,for irq interrupt during calibrate*/
	aw8697x_haptic_stop(aw8697x);
	aw8697x->rtp_init = 0;
	mutex_lock(&aw8697x->rtp_lock);
	vfree(aw8697x_rtp);
	aw8697x_rtp = vmalloc(rtp_file->size + sizeof(int));
	if (!aw8697x_rtp) {
		release_firmware(rtp_file);
		mutex_unlock(&aw8697x->rtp_lock);
		pr_err("%s: error allocating memory\n", __func__);
		return -ENOMEM;
	}
	aw8697x_rtp->len = rtp_file->size;
	aw8697x->rtp_len = rtp_file->size;
	pr_info("%s: rtp file [%s] size = %d\n", __func__,
		aw8697x_rtp_name[0], aw8697x_rtp->len);/*aw8697x->rtp_file_num */
	memcpy(aw8697x_rtp->data, rtp_file->data, rtp_file->size);
	release_firmware(rtp_file);
	mutex_unlock(&aw8697x->rtp_lock);

	/* gain */
	aw8697x_haptic_ram_vbat_comp(aw8697x, false);

	/* rtp mode config */
	aw8697x_haptic_play_mode(aw8697x, AW8697_HAPTIC_RTP_MODE);

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_DBGCTRL,
			      AW8697_BIT_DBGCTRL_INT_MODE_MASK,
			      AW8697_BIT_DBGCTRL_INT_MODE_EDGE);
	disable_irq(gpio_to_irq(aw8697x->irq_gpio));
	/* haptic start */
	aw8697x_haptic_start(aw8697x);
	pm_qos_add_request(&pm_qos_req_vb_x, PM_QOS_CPU_DMA_LATENCY, PM_QOS_VALUE_VB);
	while (1) {
		if (!aw8697x_haptic_rtp_get_fifo_afi(aw8697x)) {
			pr_info("%s !aw8697x_haptic_rtp_get_fifo_afi done aw8697x->rtp_cnt= %d \n", __func__, aw8697x->rtp_cnt);
			mutex_lock(&aw8697x->rtp_lock);
			if ((aw8697x_rtp->len - aw8697x->rtp_cnt) <
			    (aw8697x->ram.base_addr >> 2))
				buf_len = aw8697x_rtp->len - aw8697x->rtp_cnt;
			else
				buf_len = (aw8697x->ram.base_addr >> 2);

			if (aw8697x->rtp_cnt != aw8697x_rtp->len) {
				if (aw8697x->timeval_flags == 1) {
					do_gettimeofday(&aw8697x->start);
					aw8697x->timeval_flags = 0;
				}
				aw8697x->rtpupdate_flag =
				    aw8697x_i2c_writes(aw8697x,
						AW8697_REG_RTP_DATA,
						&aw8697x_rtp->data[aw8697x->
						rtp_cnt], buf_len);
				aw8697x->rtp_cnt += buf_len;
			}
			mutex_unlock(&aw8697x->rtp_lock);
		}

		osc_int_state = aw8697x_haptic_osc_read_int(aw8697x);
		if (osc_int_state&AW8697_BIT_SYSINT_DONEI) {
			do_gettimeofday(&aw8697x->end);
			pr_info("%s vincent playback done aw8697x->rtp_cnt= %d \n", __func__, aw8697x->rtp_cnt);
			break;
		}

		do_gettimeofday(&aw8697x->end);
		aw8697x->microsecond = (aw8697x->end.tv_sec - aw8697x->start.tv_sec)*1000000 +
					(aw8697x->end.tv_usec - aw8697x->start.tv_usec);
		if (aw8697x->microsecond > OSC_CALIBRATION_T_LENGTH) {
			pr_info("%s vincent time out aw8697x->rtp_cnt %d osc_int_state %02x\n", __func__, aw8697x->rtp_cnt, osc_int_state);
			break;
		}
	}
	pm_qos_remove_request(&pm_qos_req_vb_x);
	enable_irq(gpio_to_irq(aw8697x->irq_gpio));

	aw8697x->osc_cali_flag = 0;
	aw8697x->microsecond = (aw8697x->end.tv_sec - aw8697x->start.tv_sec)*1000000 +
				(aw8697x->end.tv_usec - aw8697x->start.tv_usec);
	/*calibration osc*/
	pr_info("%s awinic_microsecond:%ld \n", __func__, aw8697x->microsecond);
	pr_info("%s exit\n", __func__);
	return 0;
}

void aw8697x_op_clean_status(struct aw8697 *aw8697x)
{
	aw8697x->audio_ready = false;
	aw8697x->haptic_ready = false;
	aw8697x->pre_haptic_number = false;
	aw8697x->rtp_routine_on = 0;
	pr_debug("%s enter\n", __func__);
}
static void aw8697x_rtp_work_routine(struct work_struct *work)
{
	const struct firmware *rtp_file;
	int ret = -1;
	struct aw8697 *aw8697x = container_of(work, struct aw8697, rtp_work);

	pr_debug("%s enter\n", __func__);
	mutex_lock(&aw8697x->rtp_lock);
	if(aw8697x->rtp_routine_on != 1) {
		mutex_unlock(&aw8697x->rtp_lock);
		return;
	}
	/* fw loaded */
	ret = request_firmware(&rtp_file,
		aw8697x_rtp_name[aw8697x->rtp_file_num],
		aw8697x->dev);
	if (ret < 0) {
		pr_err("%s: failed to read %s\n", __func__,
			aw8697x_rtp_name[aw8697x->rtp_file_num]);
		mutex_unlock(&aw8697x->rtp_lock);
		return;
	}
	aw8697x->rtp_init = 0;
	vfree(aw8697x_rtp);
	aw8697x_rtp = vmalloc(rtp_file->size+sizeof(int));
	if (!aw8697x_rtp) {
		release_firmware(rtp_file);
		pr_err("%s: error allocating memory\n", __func__);
		mutex_unlock(&aw8697x->rtp_lock);
		return;
	}
	aw8697x_rtp->len = rtp_file->size;
	pr_info("%s: rtp file [%s] size = %d\n", __func__,
		aw8697x_rtp_name[aw8697x->rtp_file_num], aw8697x_rtp->len);
	memcpy(aw8697x_rtp->data, rtp_file->data, rtp_file->size);
	mutex_unlock(&aw8697x->rtp_lock);
	release_firmware(rtp_file);
	mutex_lock(&aw8697x->lock);
	aw8697x->rtp_init = 1;

	aw8697x_haptic_upload_lra(aw8697x, AW8697_HAPTIC_RTP_CALI_LRA);

	/* gain */
	aw8697x_haptic_ram_vbat_comp(aw8697x, false);

	/* rtp mode config */
	aw8697x_haptic_play_mode(aw8697x, AW8697_HAPTIC_RTP_MODE);

	/* haptic start */
	aw8697x_haptic_start(aw8697x);

	aw8697x_haptic_rtp_init(aw8697x);

	mutex_unlock(&aw8697x->lock);
}


/*****************************************************
 *
 * haptic - audio
 *
 *****************************************************/
static int aw8697x_fb_notifier_callback_tp(struct notifier_block *self, unsigned long event, void *data)
{
	struct aw8697 *aw8697x = container_of(self, struct aw8697, fb_notif);
	struct tp *tp = &(aw8697x->haptic_audio.tp);
	struct tp_point_event *tp_event = (struct tp_point_event *)data;
	struct shake_point *record_point = tp_event->record_point;
	int i = 0;
	i= tp_event->i;
	pr_debug("%s: tp event = %ld, blank = %d\n", __func__, event, i);

	if (event == 11) {
		pr_debug("%s: tp down\n", __func__);
			pr_debug("%s: tp record_point_down[%d].status = %d\n", __func__, i, record_point[i].status);
			if((AW8697_HAPTIC_TP_ST_PRESS == record_point[i].status) &&
				(record_point[i].status != tp->id[i].pt_info.status)) {
				tp->id[i].pt_info.status = record_point[i].status;
				tp->id[i].tp_flag = AW8697_HAPTIC_TP_PRESS;
				tp->id[i].press_flag = AW8697_HAPTIC_TP_PRESS;
				tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
				tp->id[i].no_play_cnt = 0;
				do_gettimeofday(&tp->id[i].t_press);
				pr_debug("%s: tp_press_release: status=%d, flag=%d", __func__, tp->id[i].pt_info.status, tp->id[i].tp_flag);
			}
	}
	if (event == 10) {
		pr_debug("%s: tp up\n", __func__);
			pr_debug("%s: tp record_point_up[%d].status = %d\n", __func__, i, record_point[i].status);
			if((AW8697_HAPTIC_TP_ST_RELEASE == record_point[i].status) &&
				(record_point[i].status != tp->id[i].pt_info.status)) {
				tp->id[i].pt_info.status = record_point[i].status;
				tp->id[i].tp_flag = AW8697_HAPTIC_TP_RELEASE;
				tp->id[i].release_flag = AW8697_HAPTIC_TP_RELEASE;
				do_gettimeofday(&tp->id[i].t_release);
				pr_debug("%s: tp_press_release: status=%d, flag=%d", __func__, tp->id[i].pt_info.status, tp->id[i].tp_flag);
			}
	}
	if (event == 0) {
		pr_debug("%s: tp all up\n", __func__);
			pr_debug("%s: tp record_point_up[%d].status = %d\n", __func__, i, record_point[i].status);
			if((AW8697_HAPTIC_TP_ST_RELEASE == record_point[i].status) &&
				(record_point[i].status != tp->id[i].pt_info.status)) {
				tp->id[i].pt_info.status = record_point[i].status;
				tp->id[i].tp_flag = AW8697_HAPTIC_TP_RELEASE;
				tp->id[i].release_flag = AW8697_HAPTIC_TP_RELEASE;
				do_gettimeofday(&tp->id[i].t_release);
				pr_debug("%s: tp_press_release: status=%d, flag=%d", __func__, tp->id[i].pt_info.status, tp->id[i].tp_flag);
			}
	}

	return 0;
}

static int aw8697x_haptic_audio_ctr_list_insert(struct haptic_audio
				*haptic_audio, struct haptic_ctr *haptic_ctr)
{
	struct haptic_ctr *p_new = NULL;

	p_new =
	    (struct haptic_ctr *)kzalloc(sizeof(struct haptic_ctr), GFP_KERNEL);
	if (p_new == NULL) {
		pr_err("%s: kzalloc memory fail\n", __func__);
		return -1;
	}
	/* update new list info */
	p_new->cnt = haptic_ctr->cnt;
	p_new->cmd = haptic_ctr->cmd;
	p_new->play = haptic_ctr->play;
	p_new->wavseq = haptic_ctr->wavseq;
	p_new->loop = haptic_ctr->loop;
	p_new->gain = haptic_ctr->gain;

	INIT_LIST_HEAD(&(p_new->list));
	list_add(&(p_new->list), &(haptic_audio->ctr_list));

	return 0;
}

static int aw8697x_haptic_audio_ctr_list_clear(struct haptic_audio *haptic_audio)
{
	struct haptic_ctr *p_ctr = NULL;
	struct haptic_ctr *p_ctr_bak = NULL;

	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
		list_del(&p_ctr->list);
		kfree(p_ctr);
	}

	return 0;
}

static int aw8697x_haptic_audio_init(struct aw8697 *aw8697x)
{

	/*   pr_debug("%s enter\n", __func__); */

	aw8697x_haptic_set_wav_seq(aw8697x, 0x01, 0x00);
	/* aw8697x->haptic_audio.ori_gain = reg_val & 0xFF; */

	return 0;
}

static int aw8697x_haptic_audio_stop(struct aw8697 *aw8697x)
{
    pr_debug("%s enter\n", __func__);


    aw8697x_haptic_play_go(aw8697x, false);
    aw8697x_haptic_stop_delay(aw8697x);
    aw8697x_haptic_play_mode(aw8697x, AW8697_HAPTIC_STANDBY_MODE);

    return 0;
}

static int aw8697x_haptic_audio_play_cfg(struct aw8697 *aw8697x)
{
	pr_debug("%s enter\n", __func__);

	aw8697x->play_mode = AW8697_HAPTIC_RAM_MODE;
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_PLAY_MODE_MASK, AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
			AW8697_BIT_SYSINTM_UVLO_MASK, AW8697_BIT_SYSINTM_UVLO_OFF);

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_WORK_MODE_MASK, AW8697_BIT_SYSCTRL_ACTIVE);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			AW8697_BIT_SYSCTRL_BST_MODE_MASK, AW8697_BIT_SYSCTRL_BST_MODE_BOOST);

    return 0;
}

static int aw8697x_haptic_audio_off(struct aw8697 *aw8697x)
{
	pr_debug("%s enter\n", __func__);

	mutex_lock(&aw8697x->lock);
	aw8697x_haptic_set_gain(aw8697x, 0x80);
	aw8697x_haptic_stop(aw8697x);
	aw8697x_haptic_audio_ctr_list_clear(&aw8697x->haptic_audio);
	mutex_unlock(&aw8697x->lock);

	return 0;
}

static enum hrtimer_restart aw8697x_haptic_audio_timer_func(struct hrtimer
							   *timer)
{
	struct aw8697 *aw8697x =
	    container_of(timer, struct aw8697, haptic_audio.timer);

	pr_debug("%s enter\n", __func__);
	schedule_work(&aw8697x->haptic_audio.work);

	hrtimer_start(&aw8697x->haptic_audio.timer,
		      ktime_set(aw8697x->haptic_audio.timer_val / 1000000,
				(aw8697x->haptic_audio.timer_val % 1000000) *
				1000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static void aw8697x_haptic_audio_work_routine(struct work_struct *work)
{
	struct aw8697 *aw8697x =
	    container_of(work, struct aw8697, haptic_audio.work);
	struct haptic_audio *haptic_audio = NULL;
	struct haptic_ctr *p_ctr = NULL;
	struct haptic_ctr *p_ctr_bak = NULL;
	unsigned int ctr_list_flag = 0;
	unsigned int ctr_list_input_cnt = 0;
	unsigned int ctr_list_output_cnt = 0;
	unsigned int ctr_list_diff_cnt = 0;
	unsigned int ctr_list_del_cnt = 0;

	int rtp_is_going_on = 0;
	struct tp *tp = &(aw8697x->haptic_audio.tp);
	struct timeval tmp_time;
	unsigned int press_delay = 0;
	unsigned int release_delay = 0;
	unsigned int i = 0;
	pr_debug("%s enter\n", __func__);

	haptic_audio = &(aw8697x->haptic_audio);
	mutex_lock(&aw8697x->haptic_audio.lock);
	memset(&aw8697x->haptic_audio.ctr, 0, sizeof(struct haptic_ctr));
	ctr_list_flag = 0;
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
		ctr_list_flag = 1;
		break;
	}
	if (ctr_list_flag == 0)
		pr_info("%s: ctr list empty\n", __func__);
	if (ctr_list_flag == 1) {
		list_for_each_entry_safe(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
			ctr_list_input_cnt = p_ctr->cnt;
			break;
		}
		list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
						 &(haptic_audio->ctr_list),
						 list) {
			ctr_list_output_cnt = p_ctr->cnt;
			break;
		}
		if (ctr_list_input_cnt > ctr_list_output_cnt) {
			ctr_list_diff_cnt =
			    ctr_list_input_cnt - ctr_list_output_cnt;
		}
		if (ctr_list_input_cnt < ctr_list_output_cnt) {
			ctr_list_diff_cnt =
			    32 + ctr_list_input_cnt - ctr_list_output_cnt;
		}
		if (ctr_list_diff_cnt > 2) {
			list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
							 &(haptic_audio->
							   ctr_list), list) {
				if ((p_ctr->play == 0)
				    && (AW8697_HAPTIC_CMD_ENABLE ==
					(AW8697_HAPTIC_CMD_HAPTIC & p_ctr->
					 cmd))) {
					list_del(&p_ctr->list);
					kfree(p_ctr);
					ctr_list_del_cnt++;
				}
				if (ctr_list_del_cnt == ctr_list_diff_cnt)
					break;
			}
		}

	}

	/* get the last data from list */
	list_for_each_entry_safe_reverse(p_ctr, p_ctr_bak,
					 &(haptic_audio->ctr_list), list) {
		aw8697x->haptic_audio.ctr.cnt = p_ctr->cnt;
		aw8697x->haptic_audio.ctr.cmd = p_ctr->cmd;
		aw8697x->haptic_audio.ctr.play = p_ctr->play;
		aw8697x->haptic_audio.ctr.wavseq = p_ctr->wavseq;
		aw8697x->haptic_audio.ctr.loop = p_ctr->loop;
		aw8697x->haptic_audio.ctr.gain = p_ctr->gain;
		list_del(&p_ctr->list);
		kfree(p_ctr);
		break;
	}
	if (aw8697x->haptic_audio.ctr.play) {
		pr_info
		("%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
		 __func__, aw8697x->haptic_audio.ctr.cnt,
		 aw8697x->haptic_audio.ctr.cmd,
		 aw8697x->haptic_audio.ctr.play,
		 aw8697x->haptic_audio.ctr.wavseq,
		 aw8697x->haptic_audio.ctr.loop,
		 aw8697x->haptic_audio.ctr.gain);
	}
	if(AW8697_HAPTIC_CMD_TP ==
			(aw8697x->haptic_audio.ctr.cmd & AW8697_HAPTIC_CMD_SYS)) {
		do_gettimeofday(&tmp_time);
		for(i=0; i<AW8697_HAPTIC_TP_ID_MAX; i++) {
			press_delay = 0;
			release_delay = 0;
			if(AW8697_HAPTIC_TP_PRESS == tp->id[i].press_flag) {
				press_delay = (tmp_time.tv_sec-tp->id[i].t_press.tv_sec)*1000000 +
					(tmp_time.tv_usec-tp->id[i].t_press.tv_usec);
			}
			if(AW8697_HAPTIC_TP_RELEASE == tp->id[i].release_flag) {
				release_delay = (tmp_time.tv_sec-tp->id[i].t_release.tv_sec)*1000000 +
					(tmp_time.tv_usec-tp->id[i].t_release.tv_usec);
			}
			if(tp->id[i].press_flag || tp->id[i].release_flag) {
				pr_info("%s: id[%d]: press_flag=%d, press_delay=%dus, release_flag=%d, release_delaly=%dus\n",
					__func__, i, tp->id[i].press_flag, press_delay, tp->id[i].release_flag, release_delay);
			}
			if(AW8697_HAPTIC_PLAY_ENABLE == aw8697x->haptic_audio.ctr.play) {
				if(AW8697_HAPTIC_TP_PRESS == tp->id[i].press_flag) {
					if(AW8697_HAPTIC_TP_PLAY_NULL == tp->id[i].play_flag) {
						if(press_delay > tp->press_delay_max) {
							/* no play time > tp-play delay max time */
							tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NOMORE;
							tp->id[i].press_flag = AW8697_HAPTIC_TP_NULL;
							tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
						} else {
							if(press_delay > tp->press_delay_min) {
								/* tp-play delay min time < no play time < tp-play delay max time */
								tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_ENABLE;
							} else {
								/* no play time < tp-play delay min time */
								tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NOMORE;
							}
						}
					} else if(AW8697_HAPTIC_TP_PLAY_ENABLE == tp->id[i].play_flag) {
					}
					pr_info("%s: %d: id[%d]:play_flag=%d", __func__, __LINE__, i, tp->id[i].play_flag);
				}
				if(AW8697_HAPTIC_TP_RELEASE == tp->id[i].release_flag) {
					if(release_delay > tp->release_delay_max) {
						/* tp release time > 3-continue-time play time*/
						if(AW8697_HAPTIC_TP_PLAY_ENABLE == tp->id[i].play_flag) {
							tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NOMORE;
							tp->id[i].press_flag = AW8697_HAPTIC_TP_NULL;
							tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
						} else {
						}
					}
					pr_info("%s: %d: id[%d]:play_flag=%d", __func__, __LINE__, i, tp->id[i].play_flag);
				}
				tp->id[i].no_play_cnt = 0;
			} else if(AW8697_HAPTIC_PLAY_NULL == aw8697x->haptic_audio.ctr.play) {
				if(AW8697_HAPTIC_TP_PRESS == tp->id[i].press_flag) {
					if(AW8697_HAPTIC_TP_PLAY_ENABLE == tp->id[i].play_flag) {
						tp->id[i].no_play_cnt ++;
						if(tp->id[i].no_play_cnt > tp->no_play_cnt_max) {
							/* no play cnt > play interal time */
							tp->id[i].no_play_cnt = tp->no_play_cnt_max;
							tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NULL;
							tp->id[i].press_flag = AW8697_HAPTIC_TP_NULL;
							tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
						} else {
						}
					} else {
						tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NULL;
					}
					pr_info("%s: %d: id[%d]:play_flag=%d", __func__, __LINE__, i, tp->id[i].play_flag);
				}
				if(AW8697_HAPTIC_TP_RELEASE == tp->id[i].release_flag) {
					/* tp release time > 3-continue-time play time*/
					if(release_delay > tp->release_delay_max) {
						tp->id[i].play_flag = AW8697_HAPTIC_TP_PLAY_NULL;
						tp->id[i].press_flag = AW8697_HAPTIC_TP_NULL;
						tp->id[i].release_flag = AW8697_HAPTIC_TP_NULL;
					} else {
					}
					pr_info("%s: %d: id[%d]:play_flag=%d", __func__, __LINE__, i, tp->id[i].play_flag);
				}
			} else {
			}
		}
		/* adjust tp play enable*/
		tp->play_flag = AW8697_HAPTIC_TP_PLAY_NULL;
		for(i=0; i<AW8697_HAPTIC_TP_ID_MAX; i++) {
			if(AW8697_HAPTIC_TP_PLAY_ENABLE ==
				tp->id[i].play_flag) {
					tp->play_flag = AW8697_HAPTIC_TP_PLAY_ENABLE;
				break;
			}
		}
		if(tp->play_flag) {
			pr_info("%s: tp.play_flag=%d\n", __func__, tp->play_flag);
		}
		for(i=0; i<AW8697_HAPTIC_TP_ID_MAX; i++) {
			tp->id[i].tp_flag = AW8697_HAPTIC_TP_NULL;
		}
	} else {
		for(i=0; i<AW8697_HAPTIC_TP_ID_MAX; i++) {
			tp->id[i].tp_flag = AW8697_HAPTIC_TP_NULL;
		}
	}
	/* rtp mode jump */
	rtp_is_going_on = aw8697x_haptic_juge_RTP_is_going_on(aw8697x);
	if (rtp_is_going_on) {
		mutex_unlock(&aw8697x->haptic_audio.lock);
		return;
	}
	mutex_unlock(&aw8697x->haptic_audio.lock);
	if(AW8697_HAPTIC_CMD_ENABLE ==
		(AW8697_HAPTIC_CMD_HAPTIC & aw8697x->haptic_audio.ctr.cmd)) {
		if (aw8697x->haptic_audio.ctr.play == AW8697_HAPTIC_PLAY_ENABLE) {
			pr_info("%s: haptic_audio_play_start\n", __func__);
           if(AW8697_HAPTIC_CMD_TP == (AW8697_HAPTIC_CMD_SYS & aw8697x->haptic_audio.ctr.cmd) &&
				(AW8697_HAPTIC_TP_PLAY_ENABLE != tp->play_flag)) {
				pr_info("%s: cancel haptic with tp event\n", __func__);
			} else {
				pr_info("%s: normal haptic with tp event\n", __func__);
			mutex_lock(&aw8697x->lock);
			aw8697x_haptic_audio_stop(aw8697x);
			aw8697x_haptic_audio_play_cfg(aw8697x);

			aw8697x_haptic_set_wav_seq(aw8697x, 0x00,
						  aw8697x->haptic_audio.ctr.
						  wavseq);
			aw8697x_haptic_set_wav_seq(aw8697x, 0x01, 0x00);

			aw8697x_haptic_set_wav_loop(aw8697x, 0x00,
						   aw8697x->haptic_audio.ctr.
						   loop);

			aw8697x_haptic_set_gain(aw8697x,
					       aw8697x->haptic_audio.ctr.gain);

			aw8697x_haptic_start(aw8697x);
			mutex_unlock(&aw8697x->lock);
		}
		} else if (AW8697_HAPTIC_PLAY_STOP ==
			   aw8697x->haptic_audio.ctr.play) {
			mutex_lock(&aw8697x->lock);
			aw8697x_haptic_stop(aw8697x);
			mutex_unlock(&aw8697x->lock);
		} else if (AW8697_HAPTIC_PLAY_GAIN ==
			   aw8697x->haptic_audio.ctr.play) {
			mutex_lock(&aw8697x->lock);
			aw8697x_haptic_set_gain(aw8697x,
					       aw8697x->haptic_audio.ctr.gain);
			mutex_unlock(&aw8697x->lock);
		}
	}
}

/*****************************************************
 *
 * haptic cont
 *
 *****************************************************/
int aw8697x_haptic_cont(struct aw8697 *aw8697x)
{
	pr_debug("%s enter\n", __func__);

	/* work mode */
	aw8697x_haptic_play_mode(aw8697x, AW8697_HAPTIC_CONT_MODE);

	/* preset f0 */
	aw8697x_haptic_set_f0_preset(aw8697x);

	/* lpf */
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_DATCTRL,
			      AW8697_BIT_DATCTRL_FC_MASK,
			      AW8697_BIT_DATCTRL_FC_1000HZ);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_DATCTRL,
			      AW8697_BIT_DATCTRL_LPF_ENABLE_MASK,
			      AW8697_BIT_DATCTRL_LPF_ENABLE);

	/* cont config */
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_ZC_DETEC_MASK,
			      AW8697_BIT_CONT_CTRL_ZC_DETEC_ENABLE);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_WAIT_PERIOD_MASK,
			      AW8697_BIT_CONT_CTRL_WAIT_1PERIOD);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_MODE_MASK,
			      AW8697_BIT_CONT_CTRL_BY_GO_SIGNAL);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK,
			      AW8697_CONT_PLAYBACK_MODE);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_F0_DETECT_MASK,
			      AW8697_BIT_CONT_CTRL_F0_DETECT_DISABLE);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_O2C_MASK,
			      AW8697_BIT_CONT_CTRL_O2C_DISABLE);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_AUTO_BRK_MASK,
			      AW8697_BIT_CONT_CTRL_AUTO_BRK_ENABLE);

	/* TD time */
	aw8697x_i2c_write(aw8697x, AW8697_REG_TD_H,
			 (unsigned char)(aw8697x->info.cont_td >> 8));
	aw8697x_i2c_write(aw8697x, AW8697_REG_TD_L,
			 (unsigned char)(aw8697x->info.cont_td >> 0));
	aw8697x_i2c_write(aw8697x, AW8697_REG_TSET, aw8697x->info.tset);

	/* zero cross */
	aw8697x_i2c_write(aw8697x, AW8697_REG_ZC_THRSH_H,
			 (unsigned char)(aw8697x->info.cont_zc_thr >> 8));
	aw8697x_i2c_write(aw8697x, AW8697_REG_ZC_THRSH_L,
			 (unsigned char)(aw8697x->info.cont_zc_thr >> 0));

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BEMF_NUM,
			      AW8697_BIT_BEMF_NUM_BRK_MASK,
			      aw8697x->info.cont_num_brk);
	/*  35*171us=5.985ms */
	aw8697x_i2c_write(aw8697x, AW8697_REG_TIME_NZC, 0x23);
	/* f0 driver level */
	aw8697x_i2c_write(aw8697x, AW8697_REG_DRV_LVL, aw8697x->info.cont_drv_lvl);
	aw8697x_i2c_write(aw8697x, AW8697_REG_DRV_LVL_OV,
			 aw8697x->info.cont_drv_lvl_ov);

	/* cont play go */
	aw8697x_haptic_play_go(aw8697x, true);

	return 0;
}

/*****************************************************
 *
 * haptic f0 cali
 *
 *****************************************************/
static int aw8697x_haptic_get_f0(struct aw8697 *aw8697x)
{
	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	unsigned char f0_pre_num = 0;
	unsigned char f0_wait_num = 0;
	unsigned char f0_repeat_num = 0;
	unsigned char f0_trace_num = 0;
	unsigned int t_f0_ms = 0;
	unsigned int t_f0_trace_ms = 0;
	unsigned int f0_cali_cnt = 50;

	pr_debug("%s enter\n", __func__);

	aw8697x->f0 = aw8697x->info.f0_pre;

	/* f0 calibrate work mode */
	aw8697x_haptic_stop(aw8697x);
	aw8697x_haptic_play_mode(aw8697x, AW8697_HAPTIC_CONT_MODE);

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK,
			      AW8697_BIT_CONT_CTRL_OPEN_PLAYBACK);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_F0_DETECT_MASK,
			      AW8697_BIT_CONT_CTRL_F0_DETECT_ENABLE);

	/* LPF */
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_DATCTRL,
			      AW8697_BIT_DATCTRL_FC_MASK,
			      AW8697_BIT_DATCTRL_FC_1000HZ);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_DATCTRL,
			      AW8697_BIT_DATCTRL_LPF_ENABLE_MASK,
			      AW8697_BIT_DATCTRL_LPF_ENABLE);

	/* LRA OSC Source */
	if (aw8697x->f0_cali_flag == AW8697_HAPTIC_CALI_F0) {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_ANACTRL,
				      AW8697_BIT_ANACTRL_LRA_SRC_MASK,
				      AW8697_BIT_ANACTRL_LRA_SRC_REG);
	} else {
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_ANACTRL,
				      AW8697_BIT_ANACTRL_LRA_SRC_MASK,
				      AW8697_BIT_ANACTRL_LRA_SRC_EFUSE);
	}

	/* preset f0 */
	aw8697x_haptic_set_f0_preset(aw8697x);

	/* f0 driver level */
	aw8697x_i2c_write(aw8697x, AW8697_REG_DRV_LVL, aw8697x->info.cont_drv_lvl);

	/* f0 trace parameter */
	f0_pre_num = aw8697x->info.f0_trace_parameter[0];
	f0_wait_num = aw8697x->info.f0_trace_parameter[1];
	f0_repeat_num = aw8697x->info.f0_trace_parameter[2];
	f0_trace_num = aw8697x->info.f0_trace_parameter[3];
	aw8697x_i2c_write(aw8697x, AW8697_REG_NUM_F0_1,
			 (f0_pre_num << 4) | (f0_wait_num << 0));
	aw8697x_i2c_write(aw8697x, AW8697_REG_NUM_F0_2, (f0_repeat_num << 0));
	aw8697x_i2c_write(aw8697x, AW8697_REG_NUM_F0_3, (f0_trace_num << 0));

	/* clear aw8697x interrupt */
	ret = aw8697x_i2c_read(aw8697x, AW8697_REG_SYSINT, &reg_val);

	/* play go and start f0 calibration */
	aw8697x_haptic_play_go(aw8697x, true);

	/* f0 trace time */
	t_f0_ms = 1000 * 10 / aw8697x->info.f0_pre;
	t_f0_trace_ms =
	    t_f0_ms * (f0_pre_num + f0_wait_num +
		       (f0_trace_num + f0_wait_num) * (f0_repeat_num - 1));
	usleep_range(t_f0_trace_ms * 1000, t_f0_trace_ms * 1000 + 500);

	for (i = 0; i < f0_cali_cnt; i++) {
		usleep_range(2000, 2500);
		ret = aw8697x_i2c_read(aw8697x, AW8697_REG_GLB_STATE, &reg_val);
		/* f0 calibrate done */
		if ((reg_val & 0xf) == 0) {
			aw8697x_haptic_read_f0(aw8697x);
			aw8697x_haptic_read_beme(aw8697x);
			break;
		}
		usleep_range(10000, 10500);
		pr_debug("%s f0 cali sleep 10ms\n", __func__);
	}

	if (i == f0_cali_cnt) {
		pr_info("%s f0 cali failed, i = %d, f0_cali_cnt =%d\n",
			__func__, i, f0_cali_cnt);
		ret = -1;
	} else
		ret = 0;

	/* restore default config */
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_EN_CLOSE_MASK,
			      AW8697_CONT_PLAYBACK_MODE);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_CONT_CTRL,
			      AW8697_BIT_CONT_CTRL_F0_DETECT_MASK,
			      AW8697_BIT_CONT_CTRL_F0_DETECT_DISABLE);

	return ret;
}

static int aw8697x_haptic_f0_calibration(struct aw8697 *aw8697x)
{
	int ret = 0;
	unsigned char reg_val = 0;
	unsigned int f0_limit = 0;
	char f0_cali_lra = 0;
	int f0_cali_step = 0;

	pr_debug("%s enter\n", __func__);

	aw8697x->f0_cali_flag = AW8697_HAPTIC_CALI_F0;

	aw8697x_i2c_write(aw8697x, AW8697_REG_TRIM_LRA, 0x00);
	if (aw8697x_haptic_get_f0(aw8697x)) {
		pr_err("%s get f0 error, user defafult f0\n", __func__);
	} else {
		/* max and min limit */
		f0_limit = aw8697x->f0;
		if (aw8697x->f0 * 100 <
		    aw8697x->info.f0_pre * (100 - aw8697x->info.f0_cali_percen)) {
			f0_limit = aw8697x->info.f0_pre;
		}
		if (aw8697x->f0 * 100 >
		    aw8697x->info.f0_pre * (100 + aw8697x->info.f0_cali_percen)) {
			f0_limit = aw8697x->info.f0_pre;
		}

		/* calculate cali step */
		f0_cali_step =
		    100000 * ((int)f0_limit -
			      (int)aw8697x->info.f0_pre) / ((int)f0_limit * 25);
		pr_debug("%s  line=%d f0_cali_step=%d\n", __func__, __LINE__,
		       f0_cali_step);
		pr_debug("%s line=%d  f0_limit=%d\n", __func__, __LINE__,
		       (int)f0_limit);
		pr_debug("%s line=%d  aw8697x->info.f0_pre=%d\n", __func__,
		       __LINE__, (int)aw8697x->info.f0_pre);

		if (f0_cali_step >= 0) {	/*f0_cali_step >= 0 */
			if (f0_cali_step % 10 >= 5)
				f0_cali_step = f0_cali_step / 10 + 1 + 32;
			else
				f0_cali_step = f0_cali_step / 10 + 32;
		} else {	/*f0_cali_step < 0 */
			if (f0_cali_step % 10 <= -5)
				f0_cali_step = 32 + (f0_cali_step / 10 - 1);
			else
				f0_cali_step = 32 + f0_cali_step / 10;
		}

		if (f0_cali_step > 31)
			f0_cali_lra = (char)f0_cali_step - 32;
		else
			f0_cali_lra = (char)f0_cali_step + 32;
		pr_debug("%s f0_cali_lra=%d\n", __func__, (int)f0_cali_lra);

		aw8697x->f0_calib_data = (int)f0_cali_lra;
		pr_debug("%s f0_cali_lra=%d\n", __func__, (int)f0_cali_lra);
#ifndef AW_F0_BRINGUP_CALI
		aw8697x->cali_lra = (char)f0_cali_lra;
		aw8697_set_cali_lra_to_nvram(aw8697x->cali_lra, AW8697X_CHANNLE);
#endif
		/* update cali step */
		aw8697x_i2c_write(aw8697x, AW8697_REG_TRIM_LRA,
				 (char)f0_cali_lra);
		aw8697x_i2c_read(aw8697x, AW8697_REG_TRIM_LRA, &reg_val);
		pr_debug("%s final trim_lra=0x%02x\n", __func__, reg_val);
	}

	/* restore default work mode */
	aw8697x_haptic_play_mode(aw8697x, AW8697_HAPTIC_STANDBY_MODE);
	aw8697x->play_mode = AW8697_HAPTIC_RAM_MODE;
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_PLAY_MODE_MASK,
			      AW8697_BIT_SYSCTRL_PLAY_MODE_RAM);
	aw8697x_haptic_stop(aw8697x);

	return ret;
}

/*****************************************************
 *
 * haptic fops
 *
 *****************************************************/
static int aw8697x_file_open(struct inode *inode, struct file *file)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	file->private_data = (void *)g_aw8697x;

	return 0;
}

static int aw8697x_file_release(struct inode *inode, struct file *file)
{
	file->private_data = (void *)NULL;

	module_put(THIS_MODULE);

	return 0;
}

static long aw8697x_file_unlocked_ioctl(struct file *file, unsigned int cmd,
				       unsigned long arg)
{
	struct aw8697 *aw8697x = (struct aw8697 *)file->private_data;

	int ret = 0;

	dev_info(aw8697x->dev, "%s: cmd=0x%x, arg=0x%lx\n", __func__, cmd, arg);

	mutex_lock(&aw8697x->lock);

	if (_IOC_TYPE(cmd) != AW8697_HAPTIC_IOCTL_MAGIC) {
		dev_err(aw8697x->dev, "%s: cmd magic err\n", __func__);
		return -EINVAL;
	}

	switch (cmd) {
	default:
		dev_err(aw8697x->dev, "%s, unknown cmd\n", __func__);
		break;
	}

	mutex_unlock(&aw8697x->lock);

	return ret;
}

static ssize_t aw8697x_file_read(struct file *filp, char *buff, size_t len,
				loff_t *offset)
{
	struct aw8697 *aw8697x = (struct aw8697 *)filp->private_data;
	int ret = 0;
	int i = 0;
	unsigned char reg_val = 0;
	unsigned char *pbuff = NULL;

	mutex_lock(&aw8697x->lock);

	dev_dbg(aw8697x->dev, "%s: len=%zu\n", __func__, len);

	switch (aw8697x->fileops.cmd) {
	case AW8697_HAPTIC_CMD_READ_REG:
		pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
		if (pbuff != NULL) {
			for (i = 0; i < len; i++) {
				aw8697x_i2c_read(aw8697x, aw8697x->fileops.reg + i,
						&reg_val);
				pbuff[i] = reg_val;
			}
			for (i = 0; i < len; i++) {
				dev_dbg(aw8697x->dev, "%s: pbuff[%d]=0x%02x\n",
					 __func__, i, pbuff[i]);
			}
			ret = copy_to_user(buff, pbuff, len);
			if (ret) {
				dev_err(aw8697x->dev, "%s: copy to user fail\n",
					__func__);
			}
			kfree(pbuff);
		} else {
			dev_err(aw8697x->dev, "%s: alloc memory fail\n",
				__func__);
		}
		break;
	default:
		dev_err(aw8697x->dev, "%s, unknown cmd %d\n", __func__,
			aw8697x->fileops.cmd);
		break;
	}

	mutex_unlock(&aw8697x->lock);

	return len;
}

static ssize_t aw8697x_file_write(struct file *filp, const char *buff,
				 size_t len, loff_t *off)
{
	struct aw8697 *aw8697x = (struct aw8697 *)filp->private_data;
	int i = 0;
	int ret = 0;
	unsigned char *pbuff = NULL;

	pbuff = (unsigned char *)kzalloc(len, GFP_KERNEL);
	if (pbuff == NULL) {
		dev_err(aw8697x->dev, "%s: alloc memory fail\n", __func__);
		return len;
	}
	ret = copy_from_user(pbuff, buff, len);
	if (ret) {
		kfree(pbuff);
		dev_err(aw8697x->dev, "%s: copy from user fail\n", __func__);
		return len;
	}

	for (i = 0; i < len; i++) {
		dev_dbg(aw8697x->dev, "%s: pbuff[%d]=0x%02x\n",
			 __func__, i, pbuff[i]);
	}

	mutex_lock(&aw8697x->lock);

	aw8697x->fileops.cmd = pbuff[0];

	switch (aw8697x->fileops.cmd) {
	case AW8697_HAPTIC_CMD_READ_REG:
		if (len == 2) {
			aw8697x->fileops.reg = pbuff[1];
		} else {
			dev_err(aw8697x->dev, "%s: read cmd len %zu err\n",
				__func__, len);
		}
		break;
	case AW8697_HAPTIC_CMD_WRITE_REG:
		if (len > 2) {
			for (i = 0; i < len - 2; i++) {
				dev_info(aw8697x->dev,
					 "%s: write reg0x%02x=0x%02x\n",
					 __func__, pbuff[1] + i, pbuff[i + 2]);
				aw8697x_i2c_write(aw8697x, pbuff[1] + i,
						 pbuff[2 + i]);
			}
		} else {
			dev_err(aw8697x->dev, "%s: write cmd len %zu err\n",
				__func__, len);
		}
		break;
	default:
		dev_err(aw8697x->dev, "%s, unknown cmd %d\n", __func__,
			aw8697x->fileops.cmd);
		break;
	}

	mutex_unlock(&aw8697x->lock);

	if (pbuff != NULL)
		kfree(pbuff);
	return len;
}

static const struct file_operations fops = {
	.owner = THIS_MODULE,
	.read = aw8697x_file_read,
	.write = aw8697x_file_write,
	.unlocked_ioctl = aw8697x_file_unlocked_ioctl,
	.open = aw8697x_file_open,
	.release = aw8697x_file_release,
};

static struct miscdevice aw8697x_haptic_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AW8697X_HAPTIC_NAME,
	.fops = &fops,
};

static int aw8697x_haptic_init(struct aw8697 *aw8697x)
{
	int ret = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;
	unsigned char bemf_config = 0;
	struct tp *tp = &(aw8697x->haptic_audio.tp);

	pr_debug("%s enter\n", __func__);

	ret = misc_register(&aw8697x_haptic_misc);
	if (ret) {
		dev_err(aw8697x->dev, "%s: misc fail: %d\n", __func__, ret);
		return ret;
	}

	/* haptic audio */
	aw8697x->haptic_audio.delay_val = 1;
	aw8697x->haptic_audio.timer_val = 21318;
	INIT_LIST_HEAD(&(aw8697x->haptic_audio.ctr_list));

	hrtimer_init(&aw8697x->haptic_audio.timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	aw8697x->haptic_audio.timer.function = aw8697x_haptic_audio_timer_func;
	INIT_WORK(&aw8697x->haptic_audio.work, aw8697x_haptic_audio_work_routine);

	mutex_init(&aw8697x->haptic_audio.lock);

	INIT_LIST_HEAD(&(aw8697x->haptic_audio.list));
	aw8697x_op_clean_status(aw8697x);
/* Haptic TP event notify start */
	aw8697x->haptic_audio.tp.press_delay_min = 1;
	aw8697x->haptic_audio.tp.press_delay_max = 400000;
	aw8697x->haptic_audio.tp.release_delay_max = 500000;
	aw8697x->haptic_audio.tp.no_play_cnt_max = 15;
	for(i=0; i<AW8697_HAPTIC_TP_ID_MAX; i++) {
		tp->id[i].pt_info.status = 0;
	}
/* Haptic TP event notify end */
	/* haptic init */
	mutex_lock(&aw8697x->lock);

	aw8697x->activate_mode = aw8697x->info.mode;

	ret = aw8697x_i2c_read(aw8697x, AW8697_REG_WAVSEQ1, &reg_val);
	aw8697x->index = reg_val & 0x7F;
	ret = aw8697x_i2c_read(aw8697x, AW8697_REG_DATDBG, &reg_val);
	aw8697x->gain = reg_val & 0xFF;
	ret = aw8697x_i2c_read(aw8697x, AW8697_REG_BSTDBG4, &reg_val);
	aw8697x->vmax = (reg_val >> 1) & 0x1F;
	for (i = 0; i < AW8697_SEQUENCER_SIZE; i++) {
		ret = aw8697x_i2c_read(aw8697x, AW8697_REG_WAVSEQ1 + i, &reg_val);
		aw8697x->seq[i] = reg_val;
	}

	aw8697x_haptic_play_mode(aw8697x, AW8697_HAPTIC_STANDBY_MODE);

	aw8697x_haptic_set_pwm(aw8697x, AW8697_PWM_24K);

	aw8697x_i2c_write(aw8697x, AW8697_REG_BSTDBG1, aw8697x->info.bstdbg[0]);
	aw8697x_i2c_write(aw8697x, AW8697_REG_BSTDBG2, aw8697x->info.bstdbg[1]);
	aw8697x_i2c_write(aw8697x, AW8697_REG_BSTDBG3, aw8697x->info.bstdbg[2]);
	aw8697x_i2c_write(aw8697x, AW8697_REG_TSET, aw8697x->info.tset);
	aw8697x_i2c_write(aw8697x, AW8697_REG_R_SPARE, aw8697x->info.r_spare);

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_ANADBG,
			      AW8697_BIT_ANADBG_IOC_MASK,
			      AW8697_BIT_ANADBG_IOC_4P65A);

	aw8697x_haptic_set_bst_peak_cur(aw8697x, AW8697_DEFAULT_PEAKCUR);

	aw8697x_haptic_swicth_motorprotect_config(aw8697x, 0x00, 0x00);

	aw8697x_haptic_auto_boost_config(aw8697x, false);

	aw8697x_haptic_trig_param_init(aw8697x);
	aw8697x_haptic_trig_param_config(aw8697x);

	aw8697x_haptic_offset_calibration(aw8697x);

	/* vbat compensation */
	aw8697x_haptic_cont_vbat_mode(aw8697x,
				     AW8697_HAPTIC_CONT_VBAT_HW_COMP_MODE);
	aw8697x->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE;

	mutex_unlock(&aw8697x->lock);

#ifdef AW_F0_BRINGUP_CALI
	/* f0 calibration */
	mutex_lock(&aw8697x->lock);

	aw8697x_haptic_f0_calibration(aw8697x);
	mutex_unlock(&aw8697x->lock);
#endif

	/* beme config */
	bemf_config = aw8697x->info.bemf_config[0];
	aw8697x_i2c_write(aw8697x, AW8697_REG_BEMF_VTHH_H, bemf_config);
	bemf_config = aw8697x->info.bemf_config[1];
	aw8697x_i2c_write(aw8697x, AW8697_REG_BEMF_VTHH_L, bemf_config);
	bemf_config = aw8697x->info.bemf_config[2];
	aw8697x_i2c_write(aw8697x, AW8697_REG_BEMF_VTHL_H, bemf_config);
	bemf_config = aw8697x->info.bemf_config[3];
	aw8697x_i2c_write(aw8697x, AW8697_REG_BEMF_VTHL_L, bemf_config);
	return ret;
}

/*****************************************************
 *
 * vibrator
 *
 *****************************************************/
#ifdef TIMED_OUTPUT
static int aw8697x_vibrator_get_time(struct timed_output_dev *dev)
{
	struct aw8697 *aw8697x = container_of(dev, struct aw8697, to_dev);

	if (hrtimer_active(&aw8697x->timer)) {
		ktime_t r = hrtimer_get_remaining(&aw8697x->timer);

		return ktime_to_ms(r);
	}

	return 0;
}

static void aw8697x_vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct aw8697 *aw8697x = container_of(dev, struct aw8697, to_dev);

	mutex_lock(&aw8697x->lock);

	pr_debug("%s enter\n", __func__);

	aw8697x_haptic_stop(aw8697x);

	if (value > 0) {
		aw8697x_haptic_ram_vbat_comp(aw8697x, false);
		aw8697x_haptic_play_wav_seq(aw8697x, value);
	}

	mutex_unlock(&aw8697x->lock);

	pr_debug("%s exit\n", __func__);
}

#else
static enum led_brightness aw8697x_haptic_brightness_get(struct led_classdev
							*cdev)
{
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);

	return aw8697x->amplitude;
}

static void aw8697x_haptic_brightness_set(struct led_classdev *cdev,
					 enum led_brightness level)
{
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);

	if (!aw8697x->ram_init)
		return;
	if (aw8697x->ramupdate_flag < 0)
		return;
	aw8697x->amplitude = level;

	mutex_lock(&aw8697x->lock);

	aw8697x_haptic_stop(aw8697x);
	if (aw8697x->amplitude > 0) {
		aw8697x_haptic_ram_vbat_comp(aw8697x, false);
		aw8697x_haptic_play_wav_seq(aw8697x, aw8697x->amplitude);
	}

	mutex_unlock(&aw8697x->lock);

}
#endif

static ssize_t aw8697x_state_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "%d\n", aw8697x->state);
}

static ssize_t aw8697x_state_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8697x_duration_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ktime_t time_rem;
	s64 time_ms = 0;

	if (hrtimer_active(&aw8697x->timer)) {
		time_rem = hrtimer_get_remaining(&aw8697x->timer);
		time_ms = ktime_to_ms(time_rem);
	}

	return snprintf(buf, PAGE_SIZE, "%lld\n", time_ms);
}

static ssize_t aw8697x_duration_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	/* setting 0 on duration is NOP for now */
	if (val <= 0)
		return count;

	aw8697x->duration = val;

	return count;
}

static ssize_t aw8697x_activate_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif

	/* For now nothing to show */
	return snprintf(buf, PAGE_SIZE, "%d\n", aw8697x->state);
}

static ssize_t aw8697x_activate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8697x->lock);
	hrtimer_cancel(&aw8697x->timer);

	aw8697x->state = val;

	mutex_unlock(&aw8697x->lock);
	schedule_work(&aw8697x->vibrator_work);

	return count;
}

static ssize_t aw8697x_activate_mode_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "activate_mode=%d\n",
			aw8697x->activate_mode);
}

static ssize_t aw8697x_activate_mode_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw8697x->lock);
	aw8697x->activate_mode = val;
	mutex_unlock(&aw8697x->lock);
	return count;
}

static ssize_t aw8697x_index_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned char reg_val = 0;

	aw8697x_i2c_read(aw8697x, AW8697_REG_WAVSEQ1, &reg_val);
	aw8697x->index = reg_val;

	return snprintf(buf, PAGE_SIZE, "%d\n", aw8697x->index);
}

static ssize_t aw8697x_index_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8697x->lock);
	aw8697x->index = val;
	aw8697x_haptic_set_repeat_wav_seq(aw8697x, aw8697x->index);
	mutex_unlock(&aw8697x->lock);
	return count;
}

static ssize_t aw8697x_vmax_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw8697x->vmax);
}

static ssize_t aw8697x_vmax_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8697x->lock);
	aw8697x->vmax = val;
	aw8697x_haptic_set_bst_vol(aw8697x, aw8697x->vmax);
	mutex_unlock(&aw8697x->lock);
	return count;
}

static ssize_t aw8697x_gain_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	return snprintf(buf, PAGE_SIZE, "0x%02x\n", aw8697x->gain);
}

static ssize_t aw8697x_gain_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	pr_debug("%s: value=%d\n", __func__, val);

	mutex_lock(&aw8697x->lock);
	aw8697x->gain = val;
	aw8697x_haptic_set_gain(aw8697x, aw8697x->gain);
	mutex_unlock(&aw8697x->lock);
	return count;
}

static ssize_t aw8697x_seq_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW8697_SEQUENCER_SIZE; i++) {
		aw8697x_i2c_read(aw8697x, AW8697_REG_WAVSEQ1 + i, &reg_val);
		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d: 0x%02x\n", i + 1, reg_val);
		aw8697x->seq[i] |= reg_val;
	}
	return count;
}

static ssize_t aw8697x_seq_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		pr_debug("%s: seq%d=0x%x\n", __func__, databuf[0],
			 databuf[1]);
		mutex_lock(&aw8697x->lock);
		aw8697x->seq[databuf[0]] = (unsigned char)databuf[1];
		aw8697x_haptic_set_wav_seq(aw8697x, (unsigned char)databuf[0],
					  aw8697x->seq[databuf[0]]);
		mutex_unlock(&aw8697x->lock);
	}
	return count;
}

static ssize_t aw8697x_loop_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	size_t count = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW8697_SEQUENCER_LOOP_SIZE; i++) {
		aw8697x_i2c_read(aw8697x, AW8697_REG_WAVLOOP1 + i, &reg_val);
		aw8697x->loop[i * 2 + 0] = (reg_val >> 4) & 0x0F;
		aw8697x->loop[i * 2 + 1] = (reg_val >> 0) & 0x0F;

		count += snprintf(buf + count, PAGE_SIZE - count,
				  "seq%d loop: 0x%02x\n", i * 2 + 1,
				  aw8697x->loop[i * 2 + 0]);
		count +=
		    snprintf(buf + count, PAGE_SIZE - count,
			     "seq%d loop: 0x%02x\n", i * 2 + 2,
			     aw8697x->loop[i * 2 + 1]);
	}
	return count;
}

static ssize_t aw8697x_loop_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		pr_debug("%s: seq%d loop=0x%x\n", __func__, databuf[0],
			 databuf[1]);
		mutex_lock(&aw8697x->lock);
		aw8697x->loop[databuf[0]] = (unsigned char)databuf[1];
		aw8697x_haptic_set_wav_loop(aw8697x, (unsigned char)databuf[0],
					   aw8697x->loop[databuf[0]]);
		mutex_unlock(&aw8697x->lock);
	}

	return count;
}

static ssize_t aw8697x_reg_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW8697_REG_MAX; i++) {
		if (!(aw8697x_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw8697x_i2c_read(aw8697x, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}
	return len;
}

static ssize_t aw8697x_reg_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw8697x_i2c_write(aw8697x, (unsigned char)databuf[0],
				 (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw8697x_rtp_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "rtp play: %d\n",
			aw8697x->rtp_cnt);

	return len;
}

#define AUDIO_READY_STATUS  1024
#define FACTORY_MODE_NORMAL_RTP_NUMBER  73
#define FACTORY_MODE_HIGH_TEMP_RTP_NUMBER  72

static ssize_t aw8697x_rtp_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
		pr_info("%s: kstrtouint fail\n", __func__);
		return rc;
	}
	pr_info("%s: rtp[%d]\n", __func__, val);

	mutex_lock(&aw8697x->lock);
	aw8697x_haptic_stop(aw8697x);

	aw8697x_haptic_set_rtp_aei(aw8697x, false);
	aw8697x_interrupt_clear(aw8697x);
	if (val < (sizeof(aw8697x_rtp_name)/AW8697_RTP_NAME_MAX)) {
		aw8697x->rtp_file_num = val;

		if (val)
			schedule_work(&aw8697x->rtp_work);
		else
			pr_err("%s: rtp_file_num 0x%02x over max value\n", __func__,
		       aw8697x->rtp_file_num);
	}
	mutex_unlock(&aw8697x->lock);
	return count;
}

static ssize_t aw8697x_ram_update_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	/* struct timed_output_dev *to_dev = dev_get_drvdata(dev); */
	/*struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);*/
#else
	/* struct led_classdev *cdev = dev_get_drvdata(dev); */
	/* struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev); */
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "sram update mode\n");
	return len;
}

static ssize_t aw8697x_ram_update_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val)
		aw8697x_ram_update(aw8697x);
	return count;
}

static ssize_t aw8697x_f0_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	mutex_lock(&aw8697x->lock);
	aw8697x->f0_cali_flag = AW8697_HAPTIC_LRA_F0;
	aw8697x_haptic_get_f0(aw8697x);
	mutex_unlock(&aw8697x->lock);
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "aw8697x lra f0 = %d\n",
		     aw8697x->f0);
	return len;
}

static ssize_t aw8697x_f0_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
#ifdef TIMED_OUTPUT
	/* struct timed_output_dev *to_dev = dev_get_drvdata(dev); */
	/*struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);*/
#else
	/* struct led_classdev *cdev = dev_get_drvdata(dev); */
	/* struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev); */
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	return count;
}

static ssize_t aw8697x_cali_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	mutex_lock(&aw8697x->lock);
	aw8697x->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
	aw8697x_haptic_get_f0(aw8697x);
	mutex_unlock(&aw8697x->lock);
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "aw8697x cali f0 = %d\n",
		     aw8697x->f0);
	return len;
}

static ssize_t aw8697x_cali_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	if (val) {
		mutex_lock(&aw8697x->lock);
		aw8697x_haptic_f0_calibration(aw8697x);
		mutex_unlock(&aw8697x->lock);
	}
	return count;
}

static ssize_t aw8697x_cali_lra_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	//struct led_classdev *cdev = dev_get_drvdata(dev);
	//struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	char cali_lar_temp = 0;

	if (aw8697_get_cali_lra_from_nvram(&cali_lar_temp, AW8697X_CHANNLE) == 0) {
		if (cali_lar_temp != 0xff)
			len += snprintf(buf + len, PAGE_SIZE - len, "OK\n");
		else
			len += snprintf(buf + len, PAGE_SIZE - len, "failed\n");
		/*aw8697x->cali_lra = cali_lar_temp;
		aw8697x_i2c_write(aw8697x, AW8697_REG_TRIM_LRA,
		aw8697x->cali_lra);
		mutex_lock(&aw8697x->lock);
		aw8697x->f0_cali_flag = AW8697_HAPTIC_CALI_F0;
		aw8697x_haptic_get_f0(aw8697x);
		mutex_unlock(&aw8697x->lock);*/
	} else
		len += snprintf(buf + len, PAGE_SIZE - len, "failed\n");

	return len;
}

static ssize_t aw8697x_cali_lra_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;
	char cali_lar_temp = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	rc = aw8697_get_cali_lra_from_nvram(&cali_lar_temp, AW8697X_CHANNLE);
	if (rc == 0 && cali_lar_temp != 0xff) {
		aw8697x->cali_lra = cali_lar_temp;
		aw8697x->f0_calib_data = cali_lar_temp;
		aw8697x_i2c_write(aw8697x, AW8697_REG_TRIM_LRA,
		aw8697x->cali_lra);
	} else { //not calibration f0
		mutex_lock(&aw8697x->lock);
		aw8697x_haptic_f0_calibration(aw8697x);
		mutex_unlock(&aw8697x->lock);
	}

	return count;
}

static ssize_t aw8697x_cont_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	aw8697x_haptic_read_cont_f0(aw8697x);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw8697x cont f0 = %d\n",
			aw8697x->cont_f0);
	return len;
}

static ssize_t aw8697x_cont_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	aw8697x_haptic_stop(aw8697x);
	if (val)
		aw8697x_haptic_cont(aw8697x);
	return count;
}

static ssize_t aw8697x_cont_td_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw8697x cont delay time = 0x%04x\n",
			aw8697x->info.cont_td);
	return len;
}

static ssize_t aw8697x_cont_td_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		aw8697x->info.cont_td = databuf[0];
		aw8697x_i2c_write(aw8697x, AW8697_REG_TD_H,
				 (unsigned char)(databuf[0] >> 8));
		aw8697x_i2c_write(aw8697x, AW8697_REG_TD_L,
				 (unsigned char)(databuf[0] >> 0));
	}
	return count;
}

static ssize_t aw8697x_cont_drv_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw8697x cont drv level = %d\n",
			aw8697x->info.cont_drv_lvl);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw8697x cont drv level overdrive= %d\n",
			aw8697x->info.cont_drv_lvl_ov);
	return len;
}

static ssize_t aw8697x_cont_drv_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%d %d", &databuf[0], &databuf[1]) == 2) {
		aw8697x->info.cont_drv_lvl = databuf[0];
		aw8697x_i2c_write(aw8697x, AW8697_REG_DRV_LVL,
				 aw8697x->info.cont_drv_lvl);
		aw8697x->info.cont_drv_lvl_ov = databuf[1];
		aw8697x_i2c_write(aw8697x, AW8697_REG_DRV_LVL_OV,
				 aw8697x->info.cont_drv_lvl_ov);
	}
	return count;
}

static ssize_t aw8697x_cont_num_brk_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw8697x cont break num = %d\n",
			aw8697x->info.cont_num_brk);
	return len;
}

static ssize_t aw8697x_cont_num_brk_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%d", &databuf[0]) == 1) {
		aw8697x->info.cont_num_brk = databuf[0];
		if (aw8697x->info.cont_num_brk > 7)
			aw8697x->info.cont_num_brk = 7;
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_BEMF_NUM,
				      AW8697_BIT_BEMF_NUM_BRK_MASK,
				      aw8697x->info.cont_num_brk);
	}
	return count;
}

static ssize_t aw8697x_cont_zc_thr_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"aw8697x cont zero cross thr = 0x%04x\n",
			aw8697x->info.cont_zc_thr);
	return len;
}

static ssize_t aw8697x_cont_zc_thr_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		aw8697x->info.cont_zc_thr = databuf[0];
		aw8697x_i2c_write(aw8697x, AW8697_REG_ZC_THRSH_H,
				 (unsigned char)(databuf[0] >> 8));
		aw8697x_i2c_write(aw8697x, AW8697_REG_ZC_THRSH_L,
				 (unsigned char)(databuf[0] >> 0));
	}
	return count;
}

static ssize_t aw8697x_vbat_monitor_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	mutex_lock(&aw8697x->lock);
	aw8697x_haptic_stop(aw8697x);
	aw8697x_haptic_get_vbat(aw8697x);
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "vbat=%dmV\n", aw8697x->vbat);
	mutex_unlock(&aw8697x->lock);

	return len;
}

static ssize_t aw8697x_vbat_monitor_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8697x_lra_resistance_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	unsigned char reg_val = 0;

	mutex_lock(&aw8697x->lock);
	aw8697x_haptic_stop(aw8697x);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_EN);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
			      AW8697_BIT_SYSCTRL_BST_MODE_BYPASS);

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_ANACTRL,
			      AW8697_BIT_ANACTRL_HD_PD_MASK,
			      AW8697_BIT_ANACTRL_HD_HZ_EN);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_D2SCFG,
			      AW8697_BIT_D2SCFG_CLK_ADC_MASK,
			      AW8697_BIT_D2SCFG_CLK_ASC_1P5MHZ);

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_DETCTRL,
			      AW8697_BIT_DETCTRL_RL_OS_MASK,
			      AW8697_BIT_DETCTRL_RL_DETECT);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_DETCTRL,
			      AW8697_BIT_DETCTRL_DIAG_GO_MASK,
			      AW8697_BIT_DETCTRL_DIAG_GO_ENABLE);
	usleep_range(3000, 3500);
	aw8697x_i2c_read(aw8697x, AW8697_REG_RLDET, &reg_val);
	aw8697x->lra = 298 * reg_val;
	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "r_lra=%dmohm\n", aw8697x->lra);

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_ANACTRL,
			      AW8697_BIT_ANACTRL_HD_PD_MASK,
			      AW8697_BIT_ANACTRL_HD_PD_EN);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_D2SCFG,
			      AW8697_BIT_D2SCFG_CLK_ADC_MASK,
			      AW8697_BIT_D2SCFG_CLK_ASC_6MHZ);

	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_OFF);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_BST_MODE_MASK,
			      AW8697_BIT_SYSCTRL_BST_MODE_BOOST);
	mutex_unlock(&aw8697x->lock);

	return len;
}

static ssize_t aw8697x_lra_resistance_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	return count;
}

static ssize_t aw8697x_auto_boost_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "auto_boost=%d\n",
		     aw8697x->auto_boost);

	return len;
}

static ssize_t aw8697x_auto_boost_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw8697x->lock);
	aw8697x_haptic_stop(aw8697x);
	aw8697x_haptic_auto_boost_config(aw8697x, val);
	mutex_unlock(&aw8697x->lock);

	return count;
}

static ssize_t aw8697x_prctmode_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	unsigned char reg_val = 0;

	aw8697x_i2c_read(aw8697x, AW8697_REG_RLDET, &reg_val);

	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "prctmode=%d\n",
		     reg_val & 0x20);
	return len;
}

static ssize_t aw8697x_prctmode_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = { 0, 0 };
	unsigned int addr = 0;
	unsigned int val = 0;

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		addr = databuf[0];
		val = databuf[1];
		mutex_lock(&aw8697x->lock);
		aw8697x_haptic_swicth_motorprotect_config(aw8697x, addr, val);
		mutex_unlock(&aw8697x->lock);
	}
	return count;
}

static ssize_t aw8697x_trig_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;
	unsigned char i = 0;

	for (i = 0; i < AW8697_TRIG_NUM; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len,
				"trig%d: enable=%d, default_level=%d, dual_edge=%d, frist_seq=%d, second_seq=%d\n",
				i + 1, aw8697x->trig[i].enable,
				aw8697x->trig[i].default_level,
				aw8697x->trig[i].dual_edge,
				aw8697x->trig[i].frist_seq,
				aw8697x->trig[i].second_seq);
	}

	return len;
}

static ssize_t aw8697x_trig_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[6] = { 0 };

	if (sscanf(buf, "%d %d %d %d %d %d",
		   &databuf[0], &databuf[1], &databuf[2], &databuf[3],
		   &databuf[4], &databuf[5])) {
		pr_debug("%s: %d, %d, %d, %d, %d, %d\n", __func__, databuf[0],
			 databuf[1], databuf[2], databuf[3], databuf[4],
			 databuf[5]);
		if (databuf[0] > 3)
			databuf[0] = 3;
		if (databuf[0] > 0)
			databuf[0] -= 1;
		aw8697x->trig[databuf[0]].enable = databuf[1];
		aw8697x->trig[databuf[0]].default_level = databuf[2];
		aw8697x->trig[databuf[0]].dual_edge = databuf[3];
		aw8697x->trig[databuf[0]].frist_seq = databuf[4];
		aw8697x->trig[databuf[0]].second_seq = databuf[5];
		mutex_lock(&aw8697x->lock);
		aw8697x_haptic_trig_param_config(aw8697x);
		aw8697x_haptic_trig_enable_config(aw8697x);
		mutex_unlock(&aw8697x->lock);
	}
	return count;
}

static ssize_t aw8697x_ram_vbat_comp_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "ram_vbat_comp=%d\n",
		     aw8697x->ram_vbat_comp);

	return len;
}

static ssize_t aw8697x_ram_vbat_comp_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;
	int rc = 0;

	rc = kstrtouint(buf, 0, &val);
	if (rc < 0)
		return rc;

	mutex_lock(&aw8697x->lock);
	if (val)
		aw8697x->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_ENABLE;
	else
		aw8697x->ram_vbat_comp = AW8697_HAPTIC_RAM_VBAT_COMP_DISABLE;
	mutex_unlock(&aw8697x->lock);

	return count;
}

static ssize_t aw8697x_osc_cali_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct aw8697 *aw8697x = dev_get_drvdata(dev);
	ssize_t len = 0;

	len +=
	    snprintf(buf + len, PAGE_SIZE - len, "lra_calib_data=%d\n",
		     aw8697x->lra_calib_data);

	return len;
}
static ssize_t aw8697x_osc_cali_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int val = 0;

	int rc = 0;
	rc = kstrtouint(buf, 0, &val);
	if (rc < 0) {
	return rc;
	}
	mutex_lock(&aw8697x->lock);
	/* osc calibration flag start,Other behaviors are forbidden */
	aw8697x->osc_cali_run = 1;
	if (val == 1) {
		aw8697x_rtp_osc_calibration(aw8697x);
		aw8697x_rtp_trim_lra_calibration(aw8697x);
	}
	aw8697x->osc_cali_run = 0;
	/* osc calibration flag end,Other behaviors are permitted */
	mutex_unlock(&aw8697x->lock);

	return count;
}

static ssize_t aw8697x_haptic_audio_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "%d\n",
			aw8697x->haptic_audio.ctr.cnt);
	return len;
}

static ssize_t aw8697x_haptic_audio_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[6] = { 0 };
	struct haptic_ctr *hap_ctr = NULL;

	if (!aw8697x->ram_init)
		return count;
	if (6 ==
	    sscanf(buf, "%d %d %d %d %d %d", &databuf[0], &databuf[1],
		   &databuf[2], &databuf[3], &databuf[4], &databuf[5])) {
		if (databuf[2]) {
			pr_info
			("%s: cnt=%d, cmd=%d, play=%d, wavseq=%d, loop=%d, gain=%d\n",
			 __func__, databuf[0], databuf[1], databuf[2],
			 databuf[3], databuf[4], databuf[5]);
		}

		hap_ctr =
		    (struct haptic_ctr *)kzalloc(sizeof(struct haptic_ctr),
						 GFP_KERNEL);
		if (hap_ctr == NULL) {
			pr_err("%s: kzalloc memory fail\n", __func__);
			return count;
		}
		mutex_lock(&aw8697x->haptic_audio.lock);
		hap_ctr->cnt = (unsigned char)databuf[0];
		hap_ctr->cmd = (unsigned char)databuf[1];
		hap_ctr->play = (unsigned char)databuf[2];
		hap_ctr->wavseq = (unsigned char)databuf[3];
		hap_ctr->loop = (unsigned char)databuf[4];
		hap_ctr->gain = (unsigned char)databuf[5];
		aw8697x_haptic_audio_ctr_list_insert(&aw8697x->haptic_audio,
						    hap_ctr);

		if (hap_ctr->cmd == 0xff) {
			pr_info("%s: haptic_audio stop\n", __func__);
			if (hrtimer_active(&aw8697x->haptic_audio.timer)) {
				pr_info("%s: cancel haptic_audio_timer\n",
					__func__);
				hrtimer_cancel(&aw8697x->haptic_audio.timer);
				aw8697x->haptic_audio.ctr.cnt = 0;
				aw8697x_haptic_audio_off(aw8697x);
			}
		} else {
			if (hrtimer_active(&aw8697x->haptic_audio.timer)) {
			} else {
				pr_info("%s: start haptic_audio_timer\n",
					__func__);
				aw8697x_haptic_audio_init(aw8697x);
				hrtimer_start(&aw8697x->haptic_audio.timer,
					      ktime_set(aw8697x->haptic_audio.
							delay_val / 1000000,
							(aw8697x->haptic_audio.
							 delay_val % 1000000) *
							1000),
					      HRTIMER_MODE_REL);
			}
		}
		mutex_unlock(&aw8697x->haptic_audio.lock);
		kfree(hap_ctr);
	}
	return count;
}

static ssize_t aw8697x_haptic_audio_time_show(struct device *dev,
					     struct device_attribute *attr,
					     char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	ssize_t len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len,
			"haptic_audio.delay_val=%dus\n",
			aw8697x->haptic_audio.delay_val);
	len += snprintf(buf + len, PAGE_SIZE - len,
			"haptic_audio.timer_val=%dus\n",
			aw8697x->haptic_audio.timer_val);
	return len;
}

static ssize_t aw8697x_haptic_audio_time_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[2] = { 0 };

	if (sscanf(buf, "%d %d", &databuf[0], &databuf[1]) == 2) {
		aw8697x->haptic_audio.delay_val = databuf[0];
		aw8697x->haptic_audio.timer_val = databuf[1];
	}
	return count;
}

static ssize_t aw8697x_haptic_audio_tp_time_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	struct tp *tp = &(aw8697x->haptic_audio.tp);
	ssize_t len = 0;
	len += snprintf(buf+len, PAGE_SIZE-len, "tp->press_delay_min=%dus\n", tp->press_delay_min);
	len += snprintf(buf+len, PAGE_SIZE-len, "tp->press_delay_max=%dus\n", tp->press_delay_max);
	len += snprintf(buf+len, PAGE_SIZE-len, "tp->release_delay_max=%dus\n", tp->release_delay_max);
	len += snprintf(buf+len, PAGE_SIZE-len, "tp->no_play_cnt_max=%d\n", tp->no_play_cnt_max);
	return len;
}

static ssize_t aw8697x_haptic_audio_tp_time_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
#ifdef TIMED_OUTPUT
	struct timed_output_dev *to_dev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(to_dev, struct aw8697, to_dev);
#else
	struct led_classdev *cdev = dev_get_drvdata(dev);
	struct aw8697 *aw8697x = container_of(cdev, struct aw8697, cdev);
#endif
	unsigned int databuf[4] = {0};
	struct tp *tp = &(aw8697x->haptic_audio.tp);

	if(4 == sscanf(buf, "%d %d %d %d", &databuf[0], &databuf[1], &databuf[2], &databuf[3])) {
		tp->press_delay_min = databuf[0];
		tp->press_delay_max = databuf[1];
		tp->release_delay_max = databuf[2];
		tp->no_play_cnt_max= databuf[3];
	}
	return count;
}


static DEVICE_ATTR(state, S_IWUSR | S_IRUGO, aw8697x_state_show,
		   aw8697x_state_store);
static DEVICE_ATTR(duration, S_IWUSR | S_IRUGO, aw8697x_duration_show,
		   aw8697x_duration_store);
static DEVICE_ATTR(activate, S_IWUSR | S_IRUGO, aw8697x_activate_show,
		   aw8697x_activate_store);
static DEVICE_ATTR(activate_mode, S_IWUSR | S_IRUGO, aw8697x_activate_mode_show,
		   aw8697x_activate_mode_store);
static DEVICE_ATTR(index, S_IWUSR | S_IRUGO, aw8697x_index_show,
		   aw8697x_index_store);
static DEVICE_ATTR(vmax, S_IWUSR | S_IRUGO, aw8697x_vmax_show,
		   aw8697x_vmax_store);
static DEVICE_ATTR(gain, S_IWUSR | S_IRUGO, aw8697x_gain_show,
		   aw8697x_gain_store);
static DEVICE_ATTR(seq, S_IWUSR | S_IRUGO, aw8697x_seq_show, aw8697x_seq_store);
static DEVICE_ATTR(loop, S_IWUSR | S_IRUGO, aw8697x_loop_show,
		   aw8697x_loop_store);
static DEVICE_ATTR(register, S_IWUSR | S_IRUGO, aw8697x_reg_show,
		   aw8697x_reg_store);
static DEVICE_ATTR(rtp, S_IWUSR | S_IRUGO, aw8697x_rtp_show, aw8697x_rtp_store);
static DEVICE_ATTR(ram_update, S_IWUSR | S_IRUGO, aw8697x_ram_update_show,
		   aw8697x_ram_update_store);
static DEVICE_ATTR(f0, S_IWUSR | S_IRUGO, aw8697x_f0_show, aw8697x_f0_store);
static DEVICE_ATTR(cali, S_IWUSR | S_IRUGO, aw8697x_cali_show,
		   aw8697x_cali_store);
static DEVICE_ATTR(cali_lra, S_IWUSR | S_IRUGO, aw8697x_cali_lra_show,
				   aw8697x_cali_lra_store);
static DEVICE_ATTR(cont, S_IWUSR | S_IRUGO, aw8697x_cont_show,
		   aw8697x_cont_store);
static DEVICE_ATTR(cont_td, S_IWUSR | S_IRUGO, aw8697x_cont_td_show,
		   aw8697x_cont_td_store);
static DEVICE_ATTR(cont_drv, S_IWUSR | S_IRUGO, aw8697x_cont_drv_show,
		   aw8697x_cont_drv_store);
static DEVICE_ATTR(cont_num_brk, S_IWUSR | S_IRUGO, aw8697x_cont_num_brk_show,
		   aw8697x_cont_num_brk_store);
static DEVICE_ATTR(cont_zc_thr, S_IWUSR | S_IRUGO, aw8697x_cont_zc_thr_show,
		   aw8697x_cont_zc_thr_store);
static DEVICE_ATTR(vbat_monitor, S_IWUSR | S_IRUGO, aw8697x_vbat_monitor_show,
		   aw8697x_vbat_monitor_store);
static DEVICE_ATTR(lra_resistance, S_IWUSR | S_IRUGO,
		   aw8697x_lra_resistance_show, aw8697x_lra_resistance_store);
static DEVICE_ATTR(auto_boost, S_IWUSR | S_IRUGO, aw8697x_auto_boost_show,
		   aw8697x_auto_boost_store);
static DEVICE_ATTR(prctmode, S_IWUSR | S_IRUGO, aw8697x_prctmode_show,
		   aw8697x_prctmode_store);
static DEVICE_ATTR(trig, S_IWUSR | S_IRUGO, aw8697x_trig_show,
		   aw8697x_trig_store);
static DEVICE_ATTR(ram_vbat_comp, S_IWUSR | S_IRUGO, aw8697x_ram_vbat_comp_show,
		   aw8697x_ram_vbat_comp_store);
static DEVICE_ATTR(osc_cali, S_IWUSR | S_IRUGO, aw8697x_osc_cali_show, aw8697x_osc_cali_store);
static DEVICE_ATTR(haptic_audio, S_IWUSR | S_IRUGO, aw8697x_haptic_audio_show,
		   aw8697x_haptic_audio_store);
static DEVICE_ATTR(haptic_audio_time, S_IWUSR | S_IRUGO,
		   aw8697x_haptic_audio_time_show,
		   aw8697x_haptic_audio_time_store);
static DEVICE_ATTR(haptic_audio_tp_time, S_IWUSR | S_IRUGO, aw8697x_haptic_audio_tp_time_show, aw8697x_haptic_audio_tp_time_store);

static struct attribute *aw8697x_vibrator_attributes[] = {
	&dev_attr_state.attr,
	&dev_attr_duration.attr,
	&dev_attr_activate.attr,
	&dev_attr_activate_mode.attr,
	&dev_attr_index.attr,
	&dev_attr_vmax.attr,
	&dev_attr_gain.attr,
	&dev_attr_seq.attr,
	&dev_attr_loop.attr,
	&dev_attr_register.attr,
	&dev_attr_rtp.attr,
	&dev_attr_ram_update.attr,
	&dev_attr_f0.attr,
	&dev_attr_cali.attr,
	&dev_attr_cali_lra.attr,
	&dev_attr_cont.attr,
	&dev_attr_cont_td.attr,
	&dev_attr_cont_drv.attr,
	&dev_attr_cont_num_brk.attr,
	&dev_attr_cont_zc_thr.attr,
	&dev_attr_vbat_monitor.attr,
	&dev_attr_lra_resistance.attr,
	&dev_attr_auto_boost.attr,
	&dev_attr_prctmode.attr,
	&dev_attr_trig.attr,
	&dev_attr_ram_vbat_comp.attr,
	&dev_attr_osc_cali.attr,
	&dev_attr_haptic_audio.attr,
	&dev_attr_haptic_audio_time.attr,
	&dev_attr_haptic_audio_tp_time.attr,
	NULL
};

static struct attribute_group aw8697x_vibrator_attribute_group = {
	.attrs = aw8697x_vibrator_attributes
};

static enum hrtimer_restart aw8697x_vibrator_timer_func(struct hrtimer *timer)
{
	struct aw8697 *aw8697x = container_of(timer, struct aw8697, timer);

	pr_debug("%s enter\n", __func__);
	aw8697x->state = 0;
	schedule_work(&aw8697x->vibrator_work);

	return HRTIMER_NORESTART;
}

static void aw8697x_vibrator_work_routine(struct work_struct *work)
{
	struct aw8697 *aw8697x =
	    container_of(work, struct aw8697, vibrator_work);

	pr_debug("%s enter\n", __func__);

	mutex_lock(&aw8697x->lock);
	if (aw8697x->ram_vib_type == AW_RAM_SHORT_VIB && aw8697x->state) {
		aw8697_double_ram_right();
		mutex_unlock(&aw8697x->lock);
		return;
	}

	aw8697x_haptic_stop(aw8697x);
	aw8697x_haptic_upload_lra(aw8697x, AW8697_HAPTIC_F0_CALI_LRA);
	if (aw8697x->state) {
		if (aw8697x->activate_mode == AW8697_HAPTIC_ACTIVATE_RAM_MODE) {
			aw8697x_haptic_set_repeat_wav_seq(aw8697x, aw8697x->index);
			/* disable gian compensation for all seq */
			aw8697x_haptic_ram_vbat_comp(aw8697x, false);
			aw8697x_haptic_play_repeat_seq(aw8697x, true);
		} else if (aw8697x->activate_mode ==
			   AW8697_HAPTIC_ACTIVATE_CONT_MODE) {
			aw8697x_haptic_cont(aw8697x);
		} else {
		}
		/* run ms timer */
		hrtimer_start(&aw8697x->timer,
			      ktime_set(aw8697x->duration / 1000,
					(aw8697x->duration % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}
	mutex_unlock(&aw8697x->lock);
}

static int aw8697x_vibrator_init(struct aw8697 *aw8697x)
{
	int ret = 0;

	pr_debug("%s enter\n", __func__);

#ifdef TIMED_OUTPUT
	aw8697x->to_dev.name = "vibrator_aw8697x";
	aw8697x->to_dev.get_time = aw8697x_vibrator_get_time;
	aw8697x->to_dev.enable = aw8697x_vibrator_enable;

	ret = timed_output_dev_register(&(aw8697x->to_dev));
	if (ret < 0) {
		dev_err(aw8697x->dev, "%s: fail to create timed output dev\n",
			__func__);
		return ret;
	}
	ret =
	    sysfs_create_group(&aw8697x->to_dev.dev->kobj,
			       &aw8697x_vibrator_attribute_group);
	if (ret < 0) {
		dev_err(aw8697x->dev, "%s error creating sysfs attr files\n",
			__func__);
		return ret;
	}
#else
	aw8697x->cdev.name = "vibrator_aw8697x";
	aw8697x->cdev.brightness_get = aw8697x_haptic_brightness_get;
	aw8697x->cdev.brightness_set = aw8697x_haptic_brightness_set;

	ret = devm_led_classdev_register(&aw8697x->i2c->dev, &aw8697x->cdev);
	//ret = led_classdev_register(&aw8697x->i2c->dev, &aw8697x->cdev);
	if (ret < 0) {
		dev_err(aw8697x->dev, "%s: fail to create led dev\n", __func__);
		return ret;
	}
	ret =
	    sysfs_create_group(&aw8697x->cdev.dev->kobj,
			       &aw8697x_vibrator_attribute_group);
	if (ret < 0) {
		dev_err(aw8697x->dev, "%s error creating sysfs attr files\n",
			__func__);
		return ret;
	}
#endif
	hrtimer_init(&aw8697x->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aw8697x->timer.function = aw8697x_vibrator_timer_func;
	INIT_WORK(&aw8697x->vibrator_work, aw8697x_vibrator_work_routine);

	INIT_WORK(&aw8697x->rtp_work, aw8697x_rtp_work_routine);

	mutex_init(&aw8697x->lock);
	mutex_init(&aw8697x->rtp_lock);

	return 0;
}

/******************************************************
 *
 * irq
 *
 ******************************************************/
void aw8697x_interrupt_clear(struct aw8697 *aw8697x)
{
	unsigned char reg_val = 0;

	pr_debug("%s enter\n", __func__);
	aw8697x_i2c_read(aw8697x, AW8697_REG_SYSINT, &reg_val);
	pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
}

static void aw8697x_interrupt_setup(struct aw8697 *aw8697x)
{
	unsigned char reg_val = 0;

	pr_debug("%s enter\n", __func__);

	aw8697x_i2c_read(aw8697x, AW8697_REG_SYSINT, &reg_val);
	pr_info("%s: reg SYSINT=0x%x\n", __func__, reg_val);

	/* edge int mode */
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_DBGCTRL,
			      AW8697_BIT_DBGCTRL_INT_MODE_MASK,
			      AW8697_BIT_DBGCTRL_INT_MODE_EDGE);

	/* int enable */
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_BSTERR_MASK,
			      AW8697_BIT_SYSINTM_BSTERR_OFF);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_OV_MASK,
			      AW8697_BIT_SYSINTM_OV_EN);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_UVLO_MASK,
			      AW8697_BIT_SYSINTM_UVLO_EN);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_OCD_MASK,
			      AW8697_BIT_SYSINTM_OCD_EN);
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_OT_MASK,
			      AW8697_BIT_SYSINTM_OT_EN);
}

static irqreturn_t aw8697x_irq(int irq, void *data)
{
	struct aw8697 *aw8697x = data;
	unsigned char reg_val = 0;
	unsigned char dbg_val = 0;
	unsigned int buf_len = 0;

	pr_debug("%s enter\n", __func__);

	aw8697x_i2c_read(aw8697x, AW8697_REG_SYSINT, &reg_val);
	aw8697x_i2c_read(aw8697x, AW8697_REG_DBGSTAT, &dbg_val);

	if (reg_val & AW8697_BIT_SYSINT_OVI) {
		aw8697x_op_clean_status(aw8697x);
		pr_err("%s chip ov int error\n", __func__);
	}
	if (reg_val & AW8697_BIT_SYSINT_UVLI) {
		aw8697x_op_clean_status(aw8697x);
		pr_err("%s chip uvlo int error\n", __func__);
	}
	if (reg_val & AW8697_BIT_SYSINT_OCDI) {
		aw8697x_op_clean_status(aw8697x);
		pr_err("%s chip over current int error\n", __func__);
	}
	if (reg_val & AW8697_BIT_SYSINT_OTI) {
		aw8697x_op_clean_status(aw8697x);
		pr_err("%s chip over temperature int error\n", __func__);
	}
	if (reg_val & AW8697_BIT_SYSINT_DONEI) {
		aw8697x_op_clean_status(aw8697x);
		aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSINTM,
			      AW8697_BIT_SYSINTM_UVLO_MASK,
			      AW8697_BIT_SYSINTM_UVLO_OFF);
		pr_info("%s chip playback done\n", __func__);
	}

	if (reg_val & AW8697_BIT_SYSINT_FF_AEI) {
		if (aw8697x->rtp_init) {
			while ((!aw8697x_haptic_rtp_get_fifo_afs(aw8697x)) &&
			       (aw8697x->play_mode == AW8697_HAPTIC_RTP_MODE) && g_aw8697x_rtp_going) {
				mutex_lock(&aw8697x->rtp_lock);
				if (!aw8697x_rtp) {
					pr_info("%s:aw8697x_rtp is null break\n",
						__func__);
					mutex_unlock(&aw8697x->rtp_lock);
					break;
				}
				if ((aw8697x_rtp->len - aw8697x->rtp_cnt) <
				    (aw8697x->ram.base_addr >> 2)) {
					buf_len =
					    aw8697x_rtp->len - aw8697x->rtp_cnt;
				} else {
					buf_len = (aw8697x->ram.base_addr >> 2);
				}
				aw8697x->rtpupdate_flag = aw8697x_i2c_writes(
					aw8697x, AW8697_REG_RTP_DATA,
					&aw8697x_rtp->data[aw8697x->rtp_cnt],
					buf_len);
				aw8697x->rtp_cnt += buf_len;
				aw8697x_i2c_read(aw8697x, AW8697_REG_GLB_STATE, &reg_val);
				if ((aw8697x->rtp_cnt == aw8697x_rtp->len) || ((reg_val & 0x0f) == 0x00)) {
					aw8697x_op_clean_status(aw8697x);
					pr_info("%s: rtp update complete\n",
					       __func__);
					aw8697x_haptic_set_rtp_aei(aw8697x,
								  false);
					aw8697x->rtp_cnt = 0;
					aw8697x->rtp_init = 0;
					mutex_unlock(&aw8697x->rtp_lock);
					return IRQ_HANDLED;
				}
				mutex_unlock(&aw8697x->rtp_lock);
			}
		} else {
			pr_info("%s: aw8697x rtp init = %d, init error\n",
			       __func__, aw8697x->rtp_init);
		}
	}

	if (reg_val & AW8697_BIT_SYSINT_FF_AFI)
		pr_debug("%s: aw8697x rtp mode fifo full empty\n", __func__);

	if (aw8697x->play_mode != AW8697_HAPTIC_RTP_MODE)
		aw8697x_haptic_set_rtp_aei(aw8697x, false);

	/*
	aw8697x_i2c_read(aw8697x, AW8697_REG_SYSINT, &reg_val);
	pr_debug("%s: reg SYSINT=0x%x\n", __func__, reg_val);
	aw8697x_i2c_read(aw8697x, AW8697_REG_SYSST, &reg_val);
	pr_debug("%s: reg SYSST=0x%x\n", __func__, reg_val);
	*/

	pr_debug("%s exit\n", __func__);

	return IRQ_HANDLED;
}

/*****************************************************
 *
 * device tree
 *
 *****************************************************/
static int aw8697x_parse_dt(struct device *dev, struct aw8697 *aw8697x,
			   struct device_node *np)
{
	unsigned int val = 0;
	unsigned int bstdbg[6];
	unsigned int f0_trace_parameter[4];
	unsigned int bemf_config[4];

	aw8697x->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (aw8697x->reset_gpio < 0) {
		dev_err(dev,
			"%s: no reset gpio provided, will not HW reset device\n",
			__func__);
		return -1;
	} else {
		dev_dbg(dev, "%s: reset gpio provided ok\n", __func__);
	}
	aw8697x->irq_gpio = of_get_named_gpio(np, "irq-gpio", 0);
	if (aw8697x->irq_gpio < 0)
		dev_err(dev, "%s: no irq gpio provided.\n", __func__);
	else
		dev_dbg(dev, "%s: irq gpio provided ok.\n", __func__);

	val = of_property_read_u32(np, "vib_mode", &aw8697x->info.mode);
	if (val != 0)
		pr_info("%s vib_mode not found\n", __func__);
	val = of_property_read_u32(np, "vib_f0_pre", &aw8697x->info.f0_pre);
	if (val != 0)
		pr_info("%s vib_f0_pre not found\n", __func__);
	val =
	    of_property_read_u32(np, "vib_f0_cali_percen",
				 &aw8697x->info.f0_cali_percen);
	if (val != 0)
		pr_info("%s vib_f0_cali_percen not found\n", __func__);
	val =
	    of_property_read_u32(np, "vib_cont_drv_lev",
				 &aw8697x->info.cont_drv_lvl);
	if (val != 0)
		pr_info("%s vib_cont_drv_lev not found\n", __func__);
	val =
	    of_property_read_u32(np, "vib_cont_drv_lvl_ov",
				 &aw8697x->info.cont_drv_lvl_ov);
	if (val != 0)
		pr_info("%s vib_cont_drv_lvl_ov not found\n", __func__);
	val = of_property_read_u32(np, "vib_cont_td", &aw8697x->info.cont_td);
	if (val != 0)
		pr_info("%s vib_cont_td not found\n", __func__);
	val =
	    of_property_read_u32(np, "vib_cont_zc_thr",
				 &aw8697x->info.cont_zc_thr);
	if (val != 0)
		pr_info("%s vib_cont_zc_thr not found\n", __func__);
	val =
	    of_property_read_u32(np, "vib_cont_num_brk",
				 &aw8697x->info.cont_num_brk);
	if (val != 0)
		pr_info("%s vib_cont_num_brk not found\n", __func__);
	val = of_property_read_u32(np, "vib_f0_coeff", &aw8697x->info.f0_coeff);
	if (val != 0)
		pr_info("%s vib_f0_coeff not found\n", __func__);

	val = of_property_read_u32(np, "vib_tset", &aw8697x->info.tset);
	if (val != 0)
		pr_info("%s vib_tset not found\n", __func__);
	val = of_property_read_u32(np, "vib_r_spare", &aw8697x->info.r_spare);
	if (val != 0)
		pr_info("%s vib_r_spare not found\n", __func__);
	val = of_property_read_u32_array(np, "vib_bstdbg",
					 bstdbg, ARRAY_SIZE(bstdbg));
	if (val != 0)
		pr_info("%s vib_bstdbg not found\n", __func__);
	memcpy(aw8697x->info.bstdbg, bstdbg, sizeof(bstdbg));
	val = of_property_read_u32_array(np, "vib_f0_trace_parameter",
					 f0_trace_parameter,
					 ARRAY_SIZE(f0_trace_parameter));
	if (val != 0)
		pr_info("%s vib_f0_trace_parameter not found\n", __func__);
	memcpy(aw8697x->info.f0_trace_parameter, f0_trace_parameter,
	       sizeof(f0_trace_parameter));
	val =
	    of_property_read_u32_array(np, "vib_bemf_config", bemf_config,
				       ARRAY_SIZE(bemf_config));
	if (val != 0)
		pr_info("%s vib_bemf_config not found\n", __func__);
	memcpy(aw8697x->info.bemf_config, bemf_config, sizeof(bemf_config));

	return 0;
}

static int aw8697x_hw_reset(struct aw8697 *aw8697x)
{
	pr_debug("%s enter\n", __func__);

	if (aw8697x && gpio_is_valid(aw8697x->reset_gpio)) {
		gpio_set_value_cansleep(aw8697x->reset_gpio, 0);
		usleep_range(1000, 2000);
		gpio_set_value_cansleep(aw8697x->reset_gpio, 1);
		usleep_range(3500, 4000);
	} else {
		dev_err(aw8697x->dev, "%s:  failed\n", __func__);
	}
	return 0;
}

/*****************************************************
 *
 * check chip id
 *
 *****************************************************/
static int aw8697x_read_chipid(struct aw8697 *aw8697x)
{
	int ret = -1;
	unsigned char cnt = 0;
	unsigned char reg = 0;

	while (cnt < AW_READ_CHIPID_RETRIES) {
		/* hardware reset */
		aw8697x_hw_reset(aw8697x);

		ret = aw8697x_i2c_read(aw8697x, AW8697_REG_ID, &reg);
		if (ret < 0) {
			dev_err(aw8697x->dev,
				"%s: failed to read register AW8697_REG_ID: %d\n",
				__func__, ret);
		}
		switch (reg) {
		case AW8697_CHIPID:
			pr_info("%s aw8697x detected\n", __func__);
			aw8697x->chipid = AW8697_CHIPID;
			/* aw8697x->flags |= AW8697_FLAG_SKIP_INTERRUPTS; */
			aw8697x_haptic_softreset(aw8697x);
			return 0;
		default:
			pr_info("%s unsupported device revision (0x%x)\n",
				__func__, reg);
			break;
		}
		cnt++;

		usleep_range(AW_READ_CHIPID_RETRY_DELAY * 1000,
			     AW_READ_CHIPID_RETRY_DELAY * 1000 + 500);
	}

	return -EINVAL;
}

/******************************************************
 *
 * sys group attribute: reg
 *
 ******************************************************/
static ssize_t aw8697x_i2c_reg_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct aw8697 *aw8697x = dev_get_drvdata(dev);

	unsigned int databuf[2] = { 0, 0 };

	if (sscanf(buf, "%x %x", &databuf[0], &databuf[1]) == 2) {
		aw8697x_i2c_write(aw8697x, (unsigned char)databuf[0],
				 (unsigned char)databuf[1]);
	}

	return count;
}

static ssize_t aw8697x_i2c_reg_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct aw8697 *aw8697x = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned char i = 0;
	unsigned char reg_val = 0;

	for (i = 0; i < AW8697_REG_MAX; i++) {
		if (!(aw8697x_reg_access[i] & REG_RD_ACCESS))
			continue;
		aw8697x_i2c_read(aw8697x, i, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len,
				"reg:0x%02x=0x%02x\n", i, reg_val);
	}
	return len;
}

static ssize_t aw8697x_i2c_ram_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct aw8697 *aw8697x = dev_get_drvdata(dev);

	unsigned int databuf[1] = { 0 };

	if (sscanf(buf, "%x", &databuf[0]) == 1) {
		if (databuf[0] == 1)
			aw8697x_ram_update(aw8697x);
	}

	return count;
}

static ssize_t aw8697x_i2c_ram_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct aw8697 *aw8697x = dev_get_drvdata(dev);
	ssize_t len = 0;
	unsigned int i = 0;
	unsigned char reg_val = 0;

	aw8697x_haptic_stop(aw8697x);
	/* RAMINIT Enable */
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_EN);

	aw8697x_i2c_write(aw8697x, AW8697_REG_RAMADDRH,
			 (unsigned char)(aw8697x->ram.base_addr >> 8));
	aw8697x_i2c_write(aw8697x, AW8697_REG_RAMADDRL,
			 (unsigned char)(aw8697x->ram.base_addr & 0x00ff));
	len += snprintf(buf + len, PAGE_SIZE - len, "aw8697x_haptic_ram:\n");
	for (i = 0; i < aw8697x->ram.len; i++) {
		aw8697x_i2c_read(aw8697x, AW8697_REG_RAMDATA, &reg_val);
		len += snprintf(buf + len, PAGE_SIZE - len, "0x%02x,", reg_val);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	/* RAMINIT Disable */
	aw8697x_i2c_write_bits(aw8697x, AW8697_REG_SYSCTRL,
			      AW8697_BIT_SYSCTRL_RAMINIT_MASK,
			      AW8697_BIT_SYSCTRL_RAMINIT_OFF);

	return len;
}

#define HAPTIC2_RESET_ACTIVE    "pmx_haptic2_reset_active"
#define HAPTIC2_INT   "pmx_haptic2_int"

static int aw8697x_i2c_pinctrl_init(struct aw8697 *aw8697x)
{
	int retval;

	/* Get pinctrl if target uses pinctrl */
	aw8697x->active_pinctrl = devm_pinctrl_get(&aw8697x->i2c->dev);
	if (IS_ERR_OR_NULL(aw8697x->active_pinctrl)) {
		retval = PTR_ERR(aw8697x->active_pinctrl);
		pr_info("haptic2 does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	aw8697x->pinctrl_reset_state
		= pinctrl_lookup_state(aw8697x->active_pinctrl, HAPTIC2_RESET_ACTIVE);
	if (IS_ERR_OR_NULL(aw8697x->pinctrl_reset_state)) {
		retval = PTR_ERR(aw8697x->pinctrl_reset_state);
		pr_info("Can not lookup %s pinstate %d\n",
					HAPTIC2_RESET_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	aw8697x->pinctrl_int_state
		= pinctrl_lookup_state(aw8697x->active_pinctrl, HAPTIC2_INT);
	if (IS_ERR_OR_NULL(aw8697x->pinctrl_int_state)) {
		retval = PTR_ERR(aw8697x->pinctrl_int_state);
		pr_info("Can not lookup %s pinstate %d\n",
					HAPTIC2_INT, retval);
		goto err_pinctrl_lookup;
	}

	if (aw8697x->active_pinctrl) {
		/*
		 * Pinctrl handle is optional. If pinctrl handle is found
		 * let pins to be configured in active state. If not
		 * found continue further without error.
		 */
		retval = pinctrl_select_state(aw8697x->active_pinctrl,
						aw8697x->pinctrl_reset_state);
		if (retval < 0) {
			pr_info("%s: Failed to select %s pinstate %d\n",
				__func__, HAPTIC2_RESET_ACTIVE, retval);
		}
		retval = pinctrl_select_state(aw8697x->active_pinctrl,
						aw8697x->pinctrl_int_state);
		if (retval < 0) {
			pr_info("%s: Failed to select %s pinstate %d\n",
				__func__, HAPTIC2_INT, retval);
		}
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(aw8697x->active_pinctrl);
err_pinctrl_get:
	aw8697x->active_pinctrl = NULL;
	return retval;
}

static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO, aw8697x_i2c_reg_show,
		   aw8697x_i2c_reg_store);
static DEVICE_ATTR(ram, S_IWUSR | S_IRUGO, aw8697x_i2c_ram_show,
		   aw8697x_i2c_ram_store);

static struct attribute *aw8697x_attributes[] = {
	&dev_attr_reg.attr,
	&dev_attr_ram.attr,
	NULL
};

static struct attribute_group aw8697x_attribute_group = {
	.attrs = aw8697x_attributes
};

/******************************************************
 *
 * i2c driver
 *
 ******************************************************/
static int aw8697x_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct aw8697 *aw8697x;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags = 0;
	int ret = -1;

	pr_debug("%s enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	aw8697x = devm_kzalloc(&i2c->dev, sizeof(struct aw8697), GFP_KERNEL);
	if (aw8697x == NULL)
		return -ENOMEM;

	aw8697x->dev = &i2c->dev;
	aw8697x->i2c = i2c;

	i2c_set_clientdata(i2c, aw8697x);

	/* aw8697x rst & int */
	if (np) {
		ret = aw8697x_parse_dt(&i2c->dev, aw8697x, np);
		if (ret) {
			dev_err(&i2c->dev,
				"%s: failed to parse device tree node\n",
				__func__);
			goto err_parse_dt;
		}
	} else {
		aw8697x->reset_gpio = -1;
		aw8697x->irq_gpio = -1;
	}

	ret = aw8697x_i2c_pinctrl_init(aw8697x);
	if (ret) {
		dev_err(&i2c->dev,
			"%s: failed to pinctrl init\n",
			__func__);
	}

	if (gpio_is_valid(aw8697x->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw8697x->reset_gpio,
					    GPIOF_OUT_INIT_LOW, "aw8697x_rst");
		if (ret) {
			dev_err(&i2c->dev, "%s: rst request failed\n",
				__func__);
			goto err_reset_gpio_request;
		}
	}

	if (gpio_is_valid(aw8697x->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, aw8697x->irq_gpio,
					    GPIOF_DIR_IN, "aw8697x_int");
		if (ret) {
			dev_err(&i2c->dev, "%s: int request failed\n",
				__func__);
			goto err_irq_gpio_request;
		}
	}

	/* aw8697x chip id */
	ret = aw8697x_read_chipid(aw8697x);
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: aw8697x_read_chipid failed ret=%d\n",
			__func__, ret);
		goto err_id;
	}

	/* aw8697x irq */
	if (gpio_is_valid(aw8697x->irq_gpio) &&
	    !(aw8697x->flags & AW8697_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		aw8697x_interrupt_setup(aw8697x);
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
						gpio_to_irq(aw8697x->irq_gpio),
						NULL, aw8697x_irq, irq_flags,
						"aw8697x", aw8697x);
		if (ret != 0) {
			dev_err(&i2c->dev, "%s: failed to request IRQ %d: %d\n",
				__func__, gpio_to_irq(aw8697x->irq_gpio), ret);
			goto err_irq;
		}
	} else {
		dev_info(&i2c->dev, "%s skipping IRQ registration\n", __func__);
		/* disable feature support if gpio was invalid */
		aw8697x->flags |= AW8697_FLAG_SKIP_INTERRUPTS;
	}

	dev_set_drvdata(&i2c->dev, aw8697x);

	ret = sysfs_create_group(&i2c->dev.kobj, &aw8697x_attribute_group);
	if (ret < 0) {
		dev_info(&i2c->dev, "%s error creating sysfs attr files\n",
			 __func__);
		goto err_sysfs;
	}

	g_aw8697x = aw8697x;

	aw8697x_vibrator_init(aw8697x);

	aw8697x_haptic_init(aw8697x);

	aw8697x_ram_init(aw8697x);
	/* add haptic audio tp mask */
	aw8697x->fb_notif.notifier_call = aw8697x_fb_notifier_callback_tp;
#if 0
	ret = touch_event_register_notifier(&aw8697x->fb_notif);
	if (ret)
		pr_info("Unable to register x fb_notifier: %d\n", ret);
#endif
	/* add haptic audio tp mask end */
	pr_info("%s probe completed successfully!\n", __func__);

	return 0;

 err_sysfs:
	devm_free_irq(&i2c->dev, gpio_to_irq(aw8697x->irq_gpio), aw8697x);
 err_irq:
 err_id:
	if (gpio_is_valid(aw8697x->irq_gpio))
		devm_gpio_free(&i2c->dev, aw8697x->irq_gpio);
 err_irq_gpio_request:
	if (gpio_is_valid(aw8697x->reset_gpio))
		devm_gpio_free(&i2c->dev, aw8697x->reset_gpio);
 err_reset_gpio_request:
 err_parse_dt:
	devm_kfree(&i2c->dev, aw8697x);
	aw8697x = NULL;
	return ret;
}

static int aw8697x_i2c_remove(struct i2c_client *i2c)
{
	struct aw8697 *aw8697x = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);

	sysfs_remove_group(&i2c->dev.kobj, &aw8697x_attribute_group);

	devm_free_irq(&i2c->dev, gpio_to_irq(aw8697x->irq_gpio), aw8697x);

	if (gpio_is_valid(aw8697x->irq_gpio))
		devm_gpio_free(&i2c->dev, aw8697x->irq_gpio);
	if (gpio_is_valid(aw8697x->reset_gpio))
		devm_gpio_free(&i2c->dev, aw8697x->reset_gpio);

	devm_kfree(&i2c->dev, aw8697x);
	aw8697x = NULL;

	return 0;
}

static int aw8697x_suspend(struct device *dev)
{
	int ret = 0;
	struct aw8697 *aw8697x = dev_get_drvdata(dev);

	mutex_lock(&aw8697x->lock);
	aw8697x_haptic_stop(aw8697x);
	mutex_unlock(&aw8697x->lock);

	return ret;
}

static int aw8697x_resume(struct device *dev)
{
	int ret = 0;
	return ret;
}

static SIMPLE_DEV_PM_OPS(aw8697x_pm_ops, aw8697x_suspend, aw8697x_resume);

static const struct i2c_device_id aw8697x_i2c_id[] = {
	{AW8697X_I2C_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, aw8697x_i2c_id);

static const struct of_device_id aw8697x_dt_match[] = {
	{.compatible = "awinic,aw8697x_haptic"},
	{},
};

static struct i2c_driver aw8697x_i2c_driver = {
	.driver = {
		   .name = AW8697X_I2C_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(aw8697x_dt_match),
#ifdef CONFIG_PM_SLEEP
		   .pm = &aw8697x_pm_ops,
#endif
		   },
	.probe = aw8697x_i2c_probe,
	.remove = aw8697x_i2c_remove,
	.id_table = aw8697x_i2c_id,
};

static int __init aw8697x_i2c_init(void)
{
	int ret = 0;

	pr_debug("aw8697x driver version %s\n", AW8697X_VERSION);

	ret = i2c_add_driver(&aw8697x_i2c_driver);
	if (ret) {
		pr_err("fail to add aw8697x device into i2c\n");
		return ret;
	}

	return 0;
}

/* late_initcall(aw8697x_i2c_init); */
module_init(aw8697x_i2c_init);

static void __exit aw8697x_i2c_exit(void)
{
	i2c_del_driver(&aw8697x_i2c_driver);
}

module_exit(aw8697x_i2c_exit);

MODULE_DESCRIPTION("AW8697X Haptic Driver");
MODULE_LICENSE("GPL v2");
