// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2020 The Linux Foundation. All rights reserved.
 */

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/regulator/driver.h>
#include <linux/qpnp/qpnp-revid.h>
#include <linux/irq.h>
#include <linux/iio/consumer.h>
#include <linux/pmic-voter.h>
#include <linux/of_batterydata.h>
#include <linux/ktime.h>
#include "smb5-lib.h"
#include "smb5-reg.h"
#include "schgm-flash.h"
#include "step-chg-jeita.h"
#include "storm-watch.h"
#include "schgm-flash.h"
#ifdef CONFIG_PRODUCT_MOBA
#include <linux/gpio.h>
#include <linux/kernel.h>
#endif
#define smblib_err(chg, fmt, ...)		\
	pr_err("%s: %s: " fmt, chg->name,	\
		__func__, ##__VA_ARGS__)	\

#define smblib_dbg(chg, reason, fmt, ...)			\
	do {							\
		if (*chg->debug_mask & (reason))		\
			pr_info("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
		else						\
			pr_debug("%s: %s: " fmt, chg->name,	\
				__func__, ##__VA_ARGS__);	\
	} while (0)

#define typec_rp_med_high(chg, typec_mode)			\
	((typec_mode == POWER_SUPPLY_TYPEC_SOURCE_MEDIUM	\
	|| typec_mode == POWER_SUPPLY_TYPEC_SOURCE_HIGH)	\
	&& (!chg->typec_legacy || chg->typec_legacy_use_rp_icl))

static void update_sw_icl_max(struct smb_charger *chg, int pst);
static int smblib_get_prop_typec_mode(struct smb_charger *chg);

#ifdef CONFIG_PRODUCT_MOBA
extern bool sec_port_online;
bool first_port_online;
#define LENOVO_WORK_MS 2000
#define LENOVO_FFC_CTRL_WORK_MS 500
int pd1_max_current = 5000;
int pd2_max_current = 5000;
int batt1_ffc_voltage;
int batt2_ffc_voltage;
int batt1_ffc_temp;
int batt2_ffc_temp;
bool is_batt1_stop;
bool is_batt2_stop;
bool is_batt1_thermal_stop;
bool is_batt2_thermal_stop;
bool is_batt1_full;
bool is_batt2_full;
extern int get_batt1_vol;
extern int get_batt2_vol;
int qcom_pd_enable;
extern bool fusb_pd_enable;
extern bool is_protect_data;
/*thermal control pd max current*/
int thermal_ctrl_batt1_current = 0;
int thermal_ctrl_batt2_current = 0;
extern int is_pd1_done;
extern int is_pd2_done;

extern bool fusb_orient_en;
bool qcom_orient_en;
extern bool fc_exit_en;
int usb1_otg_en=0;
extern int usb2_otg_en;
int otg1_charge2_en;
int connect_device_type=0;

extern int last_batt1_soc;
extern int last_batt2_soc;
extern int last_batt1_real_soc;
extern int last_batt2_real_soc;
extern int last_user_soc;
extern int realpd1_request_curr;
extern int realpd2_request_curr;

#define LENOVO_BATTERY_MONITOR_DEALY_MS				50
#define LENOVO_THERMAL_MONITOR_DEALY_MS				4000
#define QCOM_PLUG_IN_CHECK_NUM					(400 / LENOVO_BATTERY_MONITOR_DEALY_MS)
#define QCOM_PLUG_OUT_CHECK_NUM					(1000 / LENOVO_BATTERY_MONITOR_DEALY_MS)
#define LENOVO_MONITOR_EVENT_CHECK_NUM				(10000 / LENOVO_BATTERY_MONITOR_DEALY_MS)
#define CHARGER_IN_DELAY_MS					6000
#define BATTERY_FAKE_TEMP					-400
#define SEC_CHARGE_VBUS						4600000
#define	THERMAL_CHECK_MAX_NUM					30

#define BAT1_STEP_VOL	4250000
#define BAT1_STEP_CURR	2500

#define THERMAL_T0		400
#define THERMAL_T1		410
#define THERMAL_T2		420
#define THERMAL_T3		430
#define THERMAL_T4		440
#define THERMAL_T5		450
#define THERMAL_T6		460

#define THERMAL_VOL_4100MV		4100000
#define THERMAL_VOL_4150MV		4150000
#define THERMAL_VOL_4200MV		4200000
#define THERMAL_VOL_4250MV		4250000
#define THERMAL_VOL_4300MV		4300000
#define THERMAL_VOL_4470MV		4470000

#define FFC_STEP_VOLT	4250000
#define FFC_PD_MAX_CURRENT_2P5A		2500
#define FFC_PD_MAX_CURRENT_3A		3000
#define FFC_PD_MAX_CURRENT_3P5A		3500
#define FFC_PD_MAX_CURRENT_3P75A	3750
#define FFC_PD_MAX_CURRENT_4A		4000
#define FFC_PD_MAX_CURRENT_4P1A		4100
#define FFC_PD_MAX_CURRENT_4P5A		4500
#define FFC_PD_MAX_CURRENT_4P8A		4800
#define FFC_PD_MAX_CURRENT_5A		5000
#define FFC_PD_MAX_CURRENT_5P5A		5500
#define FFC_PD_MAX_CURRENT_5P7A		5700
#define FFC_PD_MAX_CURRENT_6A		6000
#define FFC_PD_MAX_CURRENT_6P5A		6500
#define FFC_PD_MAX_CURRENT_7A		7000
#define FFC_PD_MAX_CURRENT_7P5A		7500
#define FFC_PD_MAX_CURRENT_8A		8000

extern int audio_switch_state;

int qcom_charger_online;
u8 g_chg_en_dev = 0;
static u8 last_g_chg_en_dev = 0;
static u8 last_snk_stat = -1;
static u8 plug_in_count;
static u8 plug_out_count;
static u8 monitor_count = 0;

static bool last_fusb_orient_en = -1;
static bool last_qcom_orient_en = -1;
static int last_usb1_otg_en = -1;
static int last_usb2_otg_en = -1;
static int batt1_status = -1;
static int last_batt1_status = -1;
static int batt2_status = -1;
static int last_batt2_status = -1;
static int charger1_type = -1;
static int last_charger1_type = -1;
static int charger2_type = -1;
static int last_charger2_type = -1;
static int usb1_audio_en = 0;
static int last_usb1_audio_en = 0;
static int usb2_audio_en = 0;
static int last_usb2_audio_en = 0;
//static u8 g_last_chg_en_dev;

u8 first_typec_attached = 0;
u8 insert_delay_done = 0;
u8 bootup_delay_done = 0;
static void lenovo_charge_monitor_work(struct work_struct *work);
void charger_insert_remove_process(struct smb_charger *chg, int online);
extern int bq2589x_dynamic_update_charging_profile(int chg_mv, int chg_ma, int icl_ma, int ivl_mv);
extern int get_charger_max_power(void);
extern int get_charger1_5v_obj_current(void);
extern int get_charger1_9v_obj_current(void);
extern int get_charger2_max_power(void);
extern int get_charger2_5v_obj_current(void);
extern int get_charger2_9v_obj_current(void);
extern int get_charger2a_max_power(void);
extern int get_charger2a_5v_obj_current(void);
extern int get_charger2a_9v_obj_current(void);
extern void reset_charger1_pd_power_data(void);
extern int lenovo_get_sensor_temp(char *name, int *temp);
static int therm_check_total = 0;
static int therm_level[4] = {0, 0, 0, 0};
static int skin_temp = 0;
extern int force_usb_suspend;

enum pd_therm_level {
        PD_THERM_UNKNOWN,
        PD_THERM_NORMAL,
        PD_THERM_WARM,
        PD_THERM_HOT,
        PD_THERM_PERF,
        PD_SINGLE_PORT_DISABLED,
        PD_SMALL_POWER,
};

#define SINGLE_PD_THERM_LOW_TEMP		44000
#define SINGLE_PD_THERM_HIGH_TEMP		47000
#define DUAL_PD_THERM_LOW_TEMP			53000
#define DUAL_PD_THERM_HIGH_TEMP			56000
#define SINGLE_PD_THERM_USER_HIGH_TEMP		46000
#define DUAL_PD_THERM_USER_HIGH_TEMP		46000
#define	PD_THERM_WARM_FCC			3000
#endif

int smblib_read(struct smb_charger *chg, u16 addr, u8 *val)
{
	unsigned int value;
	int rc = 0;

	rc = regmap_read(chg->regmap, addr, &value);
	if (rc >= 0)
		*val = (u8)value;

	return rc;
}

int smblib_batch_read(struct smb_charger *chg, u16 addr, u8 *val,
			int count)
{
	return regmap_bulk_read(chg->regmap, addr, val, count);
}

int smblib_write(struct smb_charger *chg, u16 addr, u8 val)
{
	return regmap_write(chg->regmap, addr, val);
}

int smblib_batch_write(struct smb_charger *chg, u16 addr, u8 *val,
			int count)
{
	return regmap_bulk_write(chg->regmap, addr, val, count);
}

int smblib_masked_write(struct smb_charger *chg, u16 addr, u8 mask, u8 val)
{
	return regmap_update_bits(chg->regmap, addr, mask, val);
}

int smblib_get_iio_channel(struct smb_charger *chg, const char *propname,
					struct iio_channel **chan)
{
	int rc = 0;

	rc = of_property_match_string(chg->dev->of_node,
					"io-channel-names", propname);
	if (rc < 0)
		return 0;

	*chan = iio_channel_get(chg->dev, propname);
	if (IS_ERR(*chan)) {
		rc = PTR_ERR(*chan);
		if (rc != -EPROBE_DEFER)
			smblib_err(chg, "%s channel unavailable, %d\n",
							propname, rc);
		*chan = NULL;
	}

	return rc;
}

#define DIV_FACTOR_MICRO_V_I	1
#define DIV_FACTOR_MILI_V_I	1000
#define DIV_FACTOR_DECIDEGC	100
int smblib_read_iio_channel(struct smb_charger *chg, struct iio_channel *chan,
							int div, int *data)
{
	int rc = 0;
	*data = -ENODATA;

	if (chan) {
		rc = iio_read_channel_processed(chan, data);
		if (rc < 0) {
			smblib_err(chg, "Error in reading IIO channel data, rc=%d\n",
					rc);
			return rc;
		}

		if (div != 0)
			*data /= div;
	}

	return rc;
}

int smblib_get_jeita_cc_delta(struct smb_charger *chg, int *cc_delta_ua)
{
	int rc, cc_minus_ua;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_7_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}

	if (stat & BAT_TEMP_STATUS_HOT_SOFT_BIT) {
		rc = smblib_get_charge_param(chg, &chg->param.jeita_cc_comp_hot,
					&cc_minus_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get jeita cc minus rc=%d\n",
					rc);
			return rc;
		}
	} else if (stat & BAT_TEMP_STATUS_COLD_SOFT_BIT) {
		rc = smblib_get_charge_param(chg,
					&chg->param.jeita_cc_comp_cold,
					&cc_minus_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get jeita cc minus rc=%d\n",
					rc);
			return rc;
		}
	} else {
		cc_minus_ua = 0;
	}

	*cc_delta_ua = -cc_minus_ua;

	return 0;
}

int smblib_icl_override(struct smb_charger *chg, enum icl_override_mode  mode)
{
	int rc;
	u8 usb51_mode, icl_override, apsd_override;

	switch (mode) {
	case SW_OVERRIDE_USB51_MODE:
		usb51_mode = 0;
		icl_override = ICL_OVERRIDE_BIT;
		apsd_override = 0;
		break;
	case SW_OVERRIDE_HC_MODE:
		usb51_mode = USBIN_MODE_CHG_BIT;
#ifdef CONFIG_PRODUCT_MOBA
		icl_override = 1;
#else
		icl_override = 0;
#endif
		apsd_override = ICL_OVERRIDE_AFTER_APSD_BIT;
		break;
	case HW_AUTO_MODE:
	default:
		usb51_mode = USBIN_MODE_CHG_BIT;
		icl_override = 0;
		apsd_override = 0;
		break;
	}

	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
				USBIN_MODE_CHG_BIT, usb51_mode);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set USBIN_ICL_OPTIONS rc=%d\n", rc);
		return rc;
	}

	rc = smblib_masked_write(chg, CMD_ICL_OVERRIDE_REG,
				ICL_OVERRIDE_BIT, icl_override);
	if (rc < 0) {
		smblib_err(chg, "Couldn't override ICL rc=%d\n", rc);
		return rc;
	}

	rc = smblib_masked_write(chg, USBIN_LOAD_CFG_REG,
				ICL_OVERRIDE_AFTER_APSD_BIT, apsd_override);
	if (rc < 0) {
		smblib_err(chg, "Couldn't override ICL_AFTER_APSD rc=%d\n", rc);
		return rc;
	}

	return rc;
}

/*
 * This function does smb_en pin access, which is lock protected.
 * It should be called with smb_lock held.
 */
static int smblib_select_sec_charger_locked(struct smb_charger *chg,
					int sec_chg)
{
	int rc = 0;

	switch (sec_chg) {
	case POWER_SUPPLY_CHARGER_SEC_CP:
		vote(chg->pl_disable_votable, PL_SMB_EN_VOTER, true, 0);

		/* select Charge Pump instead of slave charger */
		rc = smblib_masked_write(chg, MISC_SMB_CFG_REG,
					SMB_EN_SEL_BIT, SMB_EN_SEL_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't select SMB charger rc=%d\n",
				rc);
			return rc;
		}
		/* Enable Charge Pump, under HW control */
		rc = smblib_masked_write(chg, MISC_SMB_EN_CMD_REG,
					EN_CP_CMD_BIT, EN_CP_CMD_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't enable SMB charger rc=%d\n",
						rc);
			return rc;
		}
		vote(chg->smb_override_votable, PL_SMB_EN_VOTER, false, 0);
		break;
	case POWER_SUPPLY_CHARGER_SEC_PL:
		/* select slave charger instead of Charge Pump */
		rc = smblib_masked_write(chg, MISC_SMB_CFG_REG,
					SMB_EN_SEL_BIT, 0);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't select SMB charger rc=%d\n",
				rc);
			return rc;
		}
		/* Enable slave charger, under HW control */
		rc = smblib_masked_write(chg, MISC_SMB_EN_CMD_REG,
					EN_STAT_CMD_BIT, EN_STAT_CMD_BIT);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't enable SMB charger rc=%d\n",
						rc);
			return rc;
		}
		vote(chg->smb_override_votable, PL_SMB_EN_VOTER, false, 0);

		vote(chg->pl_disable_votable, PL_SMB_EN_VOTER, false, 0);

		break;
	case POWER_SUPPLY_CHARGER_SEC_NONE:
	default:
		vote(chg->pl_disable_votable, PL_SMB_EN_VOTER, true, 0);

		/* SW override, disabling secondary charger(s) */
		vote(chg->smb_override_votable, PL_SMB_EN_VOTER, true, 0);
		break;
	}

	return rc;
}

static int smblib_select_sec_charger(struct smb_charger *chg, int sec_chg,
					int reason, bool toggle)
{
	int rc;

	mutex_lock(&chg->smb_lock);

	if (toggle && sec_chg == POWER_SUPPLY_CHARGER_SEC_CP) {
		rc = smblib_select_sec_charger_locked(chg,
					POWER_SUPPLY_CHARGER_SEC_NONE);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't disable secondary charger rc=%d\n",
				rc);
			goto unlock_out;
		}

		/*
		 * A minimum of 20us delay is expected before switching on STAT
		 * pin.
		 */
		usleep_range(20, 30);
	}

	rc = smblib_select_sec_charger_locked(chg, sec_chg);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't switch secondary charger rc=%d\n",
			rc);
		goto unlock_out;
	}

	chg->sec_chg_selected = sec_chg;
	chg->cp_reason = reason;

unlock_out:
	mutex_unlock(&chg->smb_lock);

	return rc;
}

static void smblib_notify_extcon_props(struct smb_charger *chg, int id)
{
	union extcon_property_value val;
	union power_supply_propval prop_val;

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_TYPEC) {
		smblib_get_prop_typec_cc_orientation(chg, &prop_val);
		val.intval = ((prop_val.intval == 2) ? 1 : 0);
		extcon_set_property(chg->extcon, id,
				EXTCON_PROP_USB_TYPEC_POLARITY, val);
		val.intval = true;
		extcon_set_property(chg->extcon, id,
				EXTCON_PROP_USB_SS, val);
	} else if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB) {
		val.intval = false;
		extcon_set_property(chg->extcon, id,
				EXTCON_PROP_USB_SS, val);
	}
}

static void smblib_notify_device_mode(struct smb_charger *chg, bool enable)
{
	if (enable)
		smblib_notify_extcon_props(chg, EXTCON_USB);

	extcon_set_state_sync(chg->extcon, EXTCON_USB, enable);
}

#ifdef CONFIG_PRODUCT_MOBA
static void smblib_notify_usb_host(struct smb_charger *chg, bool enable)
{
	if (enable) {
		smblib_dbg(chg, PR_OTG, "enabling VBUS in OTG mode\n");
		gpio_set_value(chg->charge_1t1_gpio, 1);
		gpio_set_value(chg->charge_1t2_gpio, 1);
		gpio_set_value(chg->otg_gpio, 1);
		smblib_notify_extcon_props(chg, EXTCON_USB_HOST);
	} else {
		smblib_dbg(chg, PR_OTG, "disabling VBUS in OTG mode\n");
		gpio_set_value(chg->otg_gpio, 0);
		gpio_set_value(chg->charge_1t1_gpio, 0);
		gpio_set_value(chg->charge_1t2_gpio, 0);
	}

	extcon_set_state_sync(chg->extcon, EXTCON_USB_HOST, enable);
}
#else
static void smblib_notify_usb_host(struct smb_charger *chg, bool enable)
{
	int rc = 0;

	if (enable) {
		smblib_dbg(chg, PR_OTG, "enabling VBUS in OTG mode\n");
		rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG,
					OTG_EN_BIT, OTG_EN_BIT);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't enable VBUS in OTG mode rc=%d\n", rc);
			return;
		}

		smblib_notify_extcon_props(chg, EXTCON_USB_HOST);
	} else {
		smblib_dbg(chg, PR_OTG, "disabling VBUS in OTG mode\n");
		rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG,
					OTG_EN_BIT, 0);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't disable VBUS in OTG mode rc=%d\n",
				rc);
			return;
		}
	}

	extcon_set_state_sync(chg->extcon, EXTCON_USB_HOST, enable);
}
#endif

/********************
 * REGISTER GETTERS *
 ********************/

int smblib_get_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int *val_u)
{
	int rc = 0;
	u8 val_raw;

	rc = smblib_read(chg, param->reg, &val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't read from 0x%04x rc=%d\n",
			param->name, param->reg, rc);
		return rc;
	}

	if (param->get_proc)
		*val_u = param->get_proc(param, val_raw);
	else
		*val_u = val_raw * param->step_u + param->min_u;
	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, *val_u, val_raw);

	return rc;
}

int smblib_get_usb_suspend(struct smb_charger *chg, int *suspend)
{
	int rc = 0;
	u8 temp;

	rc = smblib_read(chg, USBIN_CMD_IL_REG, &temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_CMD_IL rc=%d\n", rc);
		return rc;
	}
	*suspend = temp & USBIN_SUSPEND_BIT;

	return rc;
}


static const s16 therm_lookup_table[] = {
	/* Index -30C~85C, ADC raw code */
	0x6C92, 0x6C43, 0x6BF0, 0x6B98, 0x6B3A, 0x6AD8, 0x6A70, 0x6A03,
	0x6990, 0x6916, 0x6897, 0x6811, 0x6785, 0x66F2, 0x6658, 0x65B7,
	0x650F, 0x6460, 0x63AA, 0x62EC, 0x6226, 0x6159, 0x6084, 0x5FA8,
	0x5EC3, 0x5DD8, 0x5CE4, 0x5BE9, 0x5AE7, 0x59DD, 0x58CD, 0x57B5,
	0x5696, 0x5571, 0x5446, 0x5314, 0x51DD, 0x50A0, 0x4F5E, 0x4E17,
	0x4CCC, 0x4B7D, 0x4A2A, 0x48D4, 0x477C, 0x4621, 0x44C4, 0x4365,
	0x4206, 0x40A6, 0x3F45, 0x3DE6, 0x3C86, 0x3B28, 0x39CC, 0x3872,
	0x3719, 0x35C4, 0x3471, 0x3322, 0x31D7, 0x308F, 0x2F4C, 0x2E0D,
	0x2CD3, 0x2B9E, 0x2A6E, 0x2943, 0x281D, 0x26FE, 0x25E3, 0x24CF,
	0x23C0, 0x22B8, 0x21B5, 0x20B8, 0x1FC2, 0x1ED1, 0x1DE6, 0x1D01,
	0x1C22, 0x1B49, 0x1A75, 0x19A8, 0x18E0, 0x181D, 0x1761, 0x16A9,
	0x15F7, 0x154A, 0x14A2, 0x13FF, 0x1361, 0x12C8, 0x1234, 0x11A4,
	0x1119, 0x1091, 0x100F, 0x0F90, 0x0F15, 0x0E9E, 0x0E2B, 0x0DBC,
	0x0D50, 0x0CE8, 0x0C83, 0x0C21, 0x0BC3, 0x0B67, 0x0B0F, 0x0AB9,
	0x0A66, 0x0A16, 0x09C9, 0x097E,
};

int smblib_get_thermal_threshold(struct smb_charger *chg, u16 addr, int *val)
{
	u8 buff[2];
	s16 temp;
	int rc = 0;
	int i, lower, upper;

	rc = smblib_batch_read(chg, addr, buff, 2);
	if (rc < 0) {
		pr_err("failed to write to 0x%04X, rc=%d\n", addr, rc);
		return rc;
	}

	temp = buff[1] | buff[0] << 8;

	lower = 0;
	upper = ARRAY_SIZE(therm_lookup_table) - 1;
	while (lower <= upper) {
		i = (upper + lower) / 2;
		if (therm_lookup_table[i] < temp)
			upper = i - 1;
		else if (therm_lookup_table[i] > temp)
			lower = i + 1;
		else
			break;
	}

	/* index 0 corresonds to -30C */
	*val = (i - 30) * 10;

	return rc;
}

struct apsd_result {
	const char * const name;
	const u8 bit;
	const enum power_supply_type pst;
};

enum {
	UNKNOWN,
	SDP,
	CDP,
	DCP,
	OCP,
	FLOAT,
	HVDCP2,
	HVDCP3,
	MAX_TYPES
};

static const struct apsd_result smblib_apsd_results[] = {
	[UNKNOWN] = {
		.name	= "UNKNOWN",
		.bit	= 0,
		.pst	= POWER_SUPPLY_TYPE_UNKNOWN
	},
	[SDP] = {
		.name	= "SDP",
		.bit	= SDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB
	},
	[CDP] = {
		.name	= "CDP",
		.bit	= CDP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_CDP
	},
	[DCP] = {
		.name	= "DCP",
		.bit	= DCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[OCP] = {
		.name	= "OCP",
		.bit	= OCP_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_DCP
	},
	[FLOAT] = {
		.name	= "FLOAT",
		.bit	= FLOAT_CHARGER_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_FLOAT
	},
	[HVDCP2] = {
		.name	= "HVDCP2",
		.bit	= DCP_CHARGER_BIT | QC_2P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP
	},
	[HVDCP3] = {
		.name	= "HVDCP3",
		.bit	= DCP_CHARGER_BIT | QC_3P0_BIT,
		.pst	= POWER_SUPPLY_TYPE_USB_HVDCP_3,
	},
};

static const struct apsd_result *smblib_get_apsd_result(struct smb_charger *chg)
{
	int rc, i;
	u8 apsd_stat, stat;
	const struct apsd_result *result = &smblib_apsd_results[UNKNOWN];

	rc = smblib_read(chg, APSD_STATUS_REG, &apsd_stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return result;
	}
	smblib_dbg(chg, PR_REGISTER, "APSD_STATUS = 0x%02x\n", apsd_stat);

	if (!(apsd_stat & APSD_DTC_STATUS_DONE_BIT))
		return result;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_RESULT_STATUS rc=%d\n",
			rc);
		return result;
	}
	stat &= APSD_RESULT_STATUS_MASK;

	for (i = 0; i < ARRAY_SIZE(smblib_apsd_results); i++) {
		if (smblib_apsd_results[i].bit == stat)
			result = &smblib_apsd_results[i];
	}

	if (apsd_stat & QC_CHARGER_BIT) {
		/* since its a qc_charger, either return HVDCP3 or HVDCP2 */
		if (result != &smblib_apsd_results[HVDCP3])
			result = &smblib_apsd_results[HVDCP2];
	}
	return result;
}

#define INPUT_NOT_PRESENT	0
#define INPUT_PRESENT_USB	BIT(1)
#define INPUT_PRESENT_DC	BIT(2)
static int smblib_is_input_present(struct smb_charger *chg,
				   int *present)
{
	int rc;
	union power_supply_propval pval = {0, };

	*present = INPUT_NOT_PRESENT;

	rc = smblib_get_prop_usb_present(chg, &pval);
	if (rc < 0) {
		pr_err("Couldn't get usb presence status rc=%d\n", rc);
		return rc;
	}
	*present |= pval.intval ? INPUT_PRESENT_USB : INPUT_NOT_PRESENT;

	rc = smblib_get_prop_dc_present(chg, &pval);
	if (rc < 0) {
		pr_err("Couldn't get dc presence status rc=%d\n", rc);
		return rc;
	}
	*present |= pval.intval ? INPUT_PRESENT_DC : INPUT_NOT_PRESENT;

	return 0;
}

#define AICL_RANGE2_MIN_MV		5600
#define AICL_RANGE2_STEP_DELTA_MV	200
#define AICL_RANGE2_OFFSET		16
int smblib_get_aicl_cont_threshold(struct smb_chg_param *param, u8 val_raw)
{
	int base = param->min_u;
	u8 reg = val_raw;
	int step = param->step_u;


	if (val_raw >= AICL_RANGE2_OFFSET) {
		reg = val_raw - AICL_RANGE2_OFFSET;
		base = AICL_RANGE2_MIN_MV;
		step = AICL_RANGE2_STEP_DELTA_MV;
	}

	return base + (reg * step);
}

/********************
 * REGISTER SETTERS *
 ********************/
static const struct buck_boost_freq chg_freq_list[] = {
	[0] = {
		.freq_khz	= 2400,
		.val		= 7,
	},
	[1] = {
		.freq_khz	= 2100,
		.val		= 8,
	},
	[2] = {
		.freq_khz	= 1600,
		.val		= 11,
	},
	[3] = {
		.freq_khz	= 1200,
		.val		= 15,
	},
};

int smblib_set_chg_freq(struct smb_chg_param *param,
				int val_u, u8 *val_raw)
{
	u8 i;

	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	/* Charger FSW is the configured freqency / 2 */
	val_u *= 2;
	for (i = 0; i < ARRAY_SIZE(chg_freq_list); i++) {
		if (chg_freq_list[i].freq_khz == val_u)
			break;
	}
	if (i == ARRAY_SIZE(chg_freq_list)) {
		pr_err("Invalid frequency %d Hz\n", val_u / 2);
		return -EINVAL;
	}

	*val_raw = chg_freq_list[i].val;

	return 0;
}

int smblib_set_opt_switcher_freq(struct smb_charger *chg, int fsw_khz)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher, fsw_khz);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_buck rc=%d\n", rc);

	if (chg->mode == PARALLEL_MASTER && chg->pl.psy) {
		pval.intval = fsw_khz;
		/*
		 * Some parallel charging implementations may not have
		 * PROP_BUCK_FREQ property - they could be running
		 * with a fixed frequency
		 */
		power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_BUCK_FREQ, &pval);
	}

	return rc;
}

int smblib_set_charge_param(struct smb_charger *chg,
			    struct smb_chg_param *param, int val_u)
{
	int rc = 0;
	u8 val_raw;

	if (param->set_proc) {
		rc = param->set_proc(param, val_u, &val_raw);
		if (rc < 0)
			return -EINVAL;
	} else {
		if (val_u > param->max_u || val_u < param->min_u)
			smblib_dbg(chg, PR_MISC,
				"%s: %d is out of range [%d, %d]\n",
				param->name, val_u, param->min_u, param->max_u);

		if (val_u > param->max_u)
			val_u = param->max_u;
		if (val_u < param->min_u)
			val_u = param->min_u;

		val_raw = (val_u - param->min_u) / param->step_u;
	}

	rc = smblib_write(chg, param->reg, val_raw);
	if (rc < 0) {
		smblib_err(chg, "%s: Couldn't write 0x%02x to 0x%04x rc=%d\n",
			param->name, val_raw, param->reg, rc);
		return rc;
	}

	smblib_dbg(chg, PR_REGISTER, "%s = %d (0x%02x)\n",
		   param->name, val_u, val_raw);

	return rc;
}

int smblib_set_usb_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;
#ifdef CONFIG_PRODUCT_MOBA
	if (force_usb_suspend)
		return rc;
#endif
	if (suspend)
		vote(chg->icl_irq_disable_votable, USB_SUSPEND_VOTER,
				true, 0);

	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT,
				 suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to USBIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	if (!suspend)
		vote(chg->icl_irq_disable_votable, USB_SUSPEND_VOTER,
				false, 0);

	return rc;
}

#ifdef CONFIG_PRODUCT_MOBA
int smblib_set_usb_suspend_factory_only(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	if (suspend)
		vote(chg->icl_irq_disable_votable, USB_SUSPEND_VOTER,
				true, 0);

	rc = smblib_masked_write(chg, USBIN_CMD_IL_REG, USBIN_SUSPEND_BIT,
				 suspend ? USBIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to USBIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	if (!suspend)
		vote(chg->icl_irq_disable_votable, USB_SUSPEND_VOTER,
				false, 0);

	return rc;
}
#endif

int smblib_set_dc_suspend(struct smb_charger *chg, bool suspend)
{
	int rc = 0;

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_SUSPEND_BIT,
				 suspend ? DCIN_SUSPEND_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't write %s to DCIN_SUSPEND_BIT rc=%d\n",
			suspend ? "suspend" : "resume", rc);

	return rc;
}

static int smblib_usb_pd_adapter_allowance_override(struct smb_charger *chg,
					u8 allowed_voltage)
{
	int rc = 0;

	if (chg->chg_param.smb_version == PMI632_SUBTYPE)
		return 0;

	rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_OVERRIDE_REG,
						allowed_voltage);
	if (rc < 0)
		smblib_err(chg, "Couldn't write 0x%02x to USBIN_ADAPTER_ALLOW_OVERRIDE_REG rc=%d\n",
			allowed_voltage, rc);

	smblib_dbg(chg, PR_MISC, "set USBIN_ALLOW_OVERRIDE: %d\n",
			allowed_voltage);
	return rc;
}

#define MICRO_5V	5000000
#define MICRO_9V	9000000
#define MICRO_12V	12000000
static int smblib_set_usb_pd_fsw(struct smb_charger *chg, int voltage)
{
	int rc = 0;

	if (voltage == MICRO_5V)
		rc = smblib_set_opt_switcher_freq(chg, chg->chg_freq.freq_5V);
	else if (voltage > MICRO_5V && voltage < MICRO_9V)
		rc = smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_6V_8V);
	else if (voltage >= MICRO_9V && voltage < MICRO_12V)
		rc = smblib_set_opt_switcher_freq(chg, chg->chg_freq.freq_9V);
	else if (voltage == MICRO_12V)
		rc = smblib_set_opt_switcher_freq(chg, chg->chg_freq.freq_12V);
	else {
		smblib_err(chg, "Couldn't set Fsw: invalid voltage %d\n",
				voltage);
		return -EINVAL;
	}

	return rc;
}

#define CONT_AICL_HEADROOM_MV		1000
#define AICL_THRESHOLD_MV_IN_CC		5000
static int smblib_set_usb_pd_allowed_voltage(struct smb_charger *chg,
					int min_allowed_uv, int max_allowed_uv)
{
	int rc, aicl_threshold;
	u8 vbus_allowance;

	if (chg->chg_param.smb_version == PMI632_SUBTYPE)
		return 0;

	if (chg->pd_active == POWER_SUPPLY_PD_PPS_ACTIVE) {
		vbus_allowance = CONTINUOUS;
	} else if (min_allowed_uv == MICRO_5V && max_allowed_uv == MICRO_5V) {
		vbus_allowance = FORCE_5V;
	} else if (min_allowed_uv == MICRO_9V && max_allowed_uv == MICRO_9V) {
		vbus_allowance = FORCE_9V;
	} else if (min_allowed_uv == MICRO_12V && max_allowed_uv == MICRO_12V) {
		vbus_allowance = FORCE_12V;
	} else if (min_allowed_uv < MICRO_12V && max_allowed_uv <= MICRO_12V) {
		vbus_allowance = CONTINUOUS;
	} else {
		smblib_err(chg, "invalid allowed voltage [%d, %d]\n",
				min_allowed_uv, max_allowed_uv);
		return -EINVAL;
	}

	rc = smblib_usb_pd_adapter_allowance_override(chg, vbus_allowance);
	if (rc < 0) {
		smblib_err(chg, "set CONTINUOUS allowance failed, rc=%d\n",
				rc);
		return rc;
	}

	if (vbus_allowance != CONTINUOUS)
		return 0;

	aicl_threshold = min_allowed_uv / 1000 - CONT_AICL_HEADROOM_MV;
	if (chg->adapter_cc_mode)
		aicl_threshold = min(aicl_threshold, AICL_THRESHOLD_MV_IN_CC);

	rc = smblib_set_charge_param(chg, &chg->param.aicl_cont_threshold,
							aicl_threshold);
	if (rc < 0) {
		smblib_err(chg, "set CONT_AICL_THRESHOLD failed, rc=%d\n",
							rc);
		return rc;
	}

	return rc;
}

int smblib_set_aicl_cont_threshold(struct smb_chg_param *param,
				int val_u, u8 *val_raw)
{
	int base = param->min_u;
	int offset = 0;
	int step = param->step_u;

	if (val_u > param->max_u)
		val_u = param->max_u;
	if (val_u < param->min_u)
		val_u = param->min_u;

	if (val_u >= AICL_RANGE2_MIN_MV) {
		base = AICL_RANGE2_MIN_MV;
		step = AICL_RANGE2_STEP_DELTA_MV;
		offset = AICL_RANGE2_OFFSET;
	};

	*val_raw = ((val_u - base) / step) + offset;

	return 0;
}

/********************
 * HELPER FUNCTIONS *
 ********************/
static bool is_cp_available(struct smb_charger *chg)
{
	if (!chg->cp_psy)
		chg->cp_psy = power_supply_get_by_name("charge_pump_master");

	return !!chg->cp_psy;
}

static bool is_cp_topo_vbatt(struct smb_charger *chg)
{
	int rc;
	bool is_vbatt;
	union power_supply_propval pval;

	if (!is_cp_available(chg))
		return false;

	rc = power_supply_get_property(chg->cp_psy,
				POWER_SUPPLY_PROP_PARALLEL_OUTPUT_MODE, &pval);
	if (rc < 0)
		return false;

	is_vbatt = (pval.intval == POWER_SUPPLY_PL_OUTPUT_VBAT);

	smblib_dbg(chg, PR_WLS, "%s\n", is_vbatt ? "true" : "false");

	return is_vbatt;
}

#define CP_TO_MAIN_ICL_OFFSET_PC		10
int smblib_get_qc3_main_icl_offset(struct smb_charger *chg, int *offset_ua)
{
	union power_supply_propval pval = {0, };
	int rc;

	/*
	 * Apply ILIM offset to main charger's FCC if all of the following
	 * conditions are met:
	 * - HVDCP3 adapter with CP as parallel charger
	 * - Output connection topology is VBAT
	 */
	if (!is_cp_topo_vbatt(chg) || chg->hvdcp3_standalone_config
		|| ((chg->real_charger_type != POWER_SUPPLY_TYPE_USB_HVDCP_3)
		&& chg->real_charger_type != POWER_SUPPLY_TYPE_USB_HVDCP_3P5))
		return -EINVAL;

	rc = power_supply_get_property(chg->cp_psy, POWER_SUPPLY_PROP_CP_ENABLE,
					&pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get CP ENABLE rc=%d\n", rc);
		return rc;
	}

	if (!pval.intval)
		return -EINVAL;

	rc = power_supply_get_property(chg->cp_psy, POWER_SUPPLY_PROP_CP_ILIM,
					&pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get CP ILIM rc=%d\n", rc);
		return rc;
	}

	*offset_ua = (pval.intval * CP_TO_MAIN_ICL_OFFSET_PC * 2) / 100;

	return 0;
}

int smblib_get_prop_from_bms(struct smb_charger *chg,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int rc;

	if (!chg->bms_psy)
		return -EINVAL;

	rc = power_supply_get_property(chg->bms_psy, psp, val);

	return rc;
}

void smblib_apsd_enable(struct smb_charger *chg, bool enable)
{
	int rc;

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				BC1P2_SRC_DETECT_BIT,
				enable ? BC1P2_SRC_DETECT_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "failed to write USBIN_OPTIONS_1_CFG rc=%d\n",
				rc);
}

void smblib_hvdcp_detect_enable(struct smb_charger *chg, bool enable)
{
	int rc;
	u8 mask;
	pr_info("smblib_hvdcp_detect_enable:chg->hvdcp_disable=%d\n",chg->hvdcp_disable);
	if (chg->hvdcp_disable)
		return;
	mask = HVDCP_AUTH_ALG_EN_CFG_BIT | HVDCP_EN_BIT;
	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG, mask,
						enable ? mask : 0);
	if (rc < 0)
		smblib_err(chg, "failed to write USBIN_OPTIONS_1_CFG rc=%d\n",
				rc);
}

static void smblib_hvdcp_detect_try_enable(struct smb_charger *chg, bool enable)
{
	if (chg->hvdcp_disable || chg->pd_not_supported)
		return;
	smblib_hvdcp_detect_enable(chg, enable);
}

void smblib_hvdcp_hw_inov_enable(struct smb_charger *chg, bool enable)
{
	int rc;

	rc = smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT,
				enable ? HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "failed to write USBIN_OPTIONS_1_CFG rc=%d\n",
				rc);
}

void smblib_hvdcp_exit_config(struct smb_charger *chg)
{
	u8 stat;
	int rc;

	rc = smblib_read(chg, APSD_RESULT_STATUS_REG, &stat);
	if (rc < 0)
		return;

	if (stat & (QC_3P0_BIT | QC_2P0_BIT)) {
		/* force HVDCP to 5V */
		smblib_masked_write(chg, USBIN_OPTIONS_1_CFG_REG,
				HVDCP_AUTONOMOUS_MODE_EN_CFG_BIT, 0);
		smblib_write(chg, CMD_HVDCP_2_REG, FORCE_5V_BIT);

		/* rerun APSD */
		smblib_masked_write(chg, CMD_APSD_REG, APSD_RERUN_BIT,
				APSD_RERUN_BIT);
	}
}

static int smblib_request_dpdm(struct smb_charger *chg, bool enable)
{
	int rc = 0;
#ifdef CONFIG_PRODUCT_MOBA
	pr_info("smb5-lib:smblib_request_dpdm,otg1_charge2_en=%d\n",otg1_charge2_en);
	if(otg1_charge2_en==1){
		pr_info("smb5-lib:smblib_request_dpdm,return\n");
              return 0;
	}
#endif
	if (chg->pr_swap_in_progress)
		return 0;

	/* fetch the DPDM regulator */
	if (!chg->dpdm_reg && of_get_property(chg->dev->of_node,
				"dpdm-supply", NULL)) {
		chg->dpdm_reg = devm_regulator_get(chg->dev, "dpdm");
		if (IS_ERR(chg->dpdm_reg)) {
			rc = PTR_ERR(chg->dpdm_reg);
			smblib_err(chg, "Couldn't get dpdm regulator rc=%d\n",
					rc);
			chg->dpdm_reg = NULL;
			return rc;
		}
	}

	mutex_lock(&chg->dpdm_lock);
	if (enable) {
		if (chg->dpdm_reg && !chg->dpdm_enabled) {
			smblib_dbg(chg, PR_MISC, "enabling DPDM regulator\n");
			rc = regulator_enable(chg->dpdm_reg);
			if (rc < 0)
				smblib_err(chg,
					"Couldn't enable dpdm regulator rc=%d\n",
					rc);
			else
				chg->dpdm_enabled = true;
		}
	} else {
		if (chg->dpdm_reg && chg->dpdm_enabled) {
			smblib_dbg(chg, PR_MISC, "disabling DPDM regulator\n");
			rc = regulator_disable(chg->dpdm_reg);
			if (rc < 0)
				smblib_err(chg,
					"Couldn't disable dpdm regulator rc=%d\n",
					rc);
			else
				chg->dpdm_enabled = false;
		}
	}
	mutex_unlock(&chg->dpdm_lock);

	return rc;
}

void smblib_rerun_apsd(struct smb_charger *chg)
{
	int rc;

	smblib_dbg(chg, PR_MISC, "re-running APSD\n");

	rc = smblib_masked_write(chg, CMD_APSD_REG,
				APSD_RERUN_BIT, APSD_RERUN_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't re-run APSD rc=%d\n", rc);
}

static const struct apsd_result *smblib_update_usb_type(struct smb_charger *chg)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);

	/* if PD is active, APSD is disabled so won't have a valid result */
	if (chg->pd_active) {
		chg->real_charger_type = POWER_SUPPLY_TYPE_USB_PD;
	} else if (chg->qc3p5_detected) {
		chg->real_charger_type = POWER_SUPPLY_TYPE_USB_HVDCP_3P5;
	} else {
		/*
		 * Update real charger type only if its not FLOAT
		 * detected as as SDP
		 */
		if (!(apsd_result->pst == POWER_SUPPLY_TYPE_USB_FLOAT &&
			chg->real_charger_type == POWER_SUPPLY_TYPE_USB))
			chg->real_charger_type = apsd_result->pst;
	}

	smblib_dbg(chg, PR_MISC, "APSD=%s PD=%d QC3P5=%d\n",
			apsd_result->name, chg->pd_active, chg->qc3p5_detected);
	return apsd_result;
}

static int smblib_notifier_call(struct notifier_block *nb,
		unsigned long ev, void *v)
{
	struct power_supply *psy = v;
	struct smb_charger *chg = container_of(nb, struct smb_charger, nb);

	if (!strcmp(psy->desc->name, "bms")) {
		if (!chg->bms_psy)
			chg->bms_psy = psy;
		if (ev == PSY_EVENT_PROP_CHANGED)
			schedule_work(&chg->bms_update_work);
	}

	if (chg->jeita_configured == JEITA_CFG_NONE)
		schedule_work(&chg->jeita_update_work);

	if (chg->sec_pl_present && !chg->pl.psy
		&& !strcmp(psy->desc->name, "parallel")) {
		chg->pl.psy = psy;
		schedule_work(&chg->pl_update_work);
	}

	if (!strcmp(psy->desc->name, "charge_pump_master")) {
		pm_stay_awake(chg->dev);
		schedule_work(&chg->cp_status_change_work);
	}

	return NOTIFY_OK;
}

static int smblib_register_notifier(struct smb_charger *chg)
{
	int rc;

	chg->nb.notifier_call = smblib_notifier_call;
	rc = power_supply_reg_notifier(&chg->nb);
	if (rc < 0) {
		smblib_err(chg, "Couldn't register psy notifier rc = %d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_mapping_soc_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u)
		return -EINVAL;

	*val_raw = val_u << 1;

	return 0;
}

int smblib_mapping_cc_delta_to_field_value(struct smb_chg_param *param,
					   u8 val_raw)
{
	int val_u  = val_raw * param->step_u + param->min_u;

	if (val_u > param->max_u)
		val_u -= param->max_u * 2;

	return val_u;
}

int smblib_mapping_cc_delta_from_field_value(struct smb_chg_param *param,
					     int val_u, u8 *val_raw)
{
	if (val_u > param->max_u || val_u < param->min_u - param->max_u)
		return -EINVAL;

	val_u += param->max_u * 2 - param->min_u;
	val_u %= param->max_u * 2;
	*val_raw = val_u / param->step_u;

	return 0;
}

static void smblib_uusb_removal(struct smb_charger *chg)
{
	int rc;
	struct smb_irq_data *data;
	struct storm_watch *wdata;
	int sec_charger;

	sec_charger = chg->sec_pl_present ? POWER_SUPPLY_CHARGER_SEC_PL :
				POWER_SUPPLY_CHARGER_SEC_NONE;
	smblib_select_sec_charger(chg, sec_charger, POWER_SUPPLY_CP_NONE,
					false);

	cancel_delayed_work_sync(&chg->pl_enable_work);

	if (chg->wa_flags & BOOST_BACK_WA) {
		data = chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data;
		if (data) {
			wdata = &data->storm_data;
			update_storm_count(wdata, WEAK_CHG_STORM_COUNT);
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
			vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
					false, 0);
		}
	}
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);

	/* reset both usbin current and voltage votes */
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
			is_flash_active(chg) ? SDP_CURRENT_UA : SDP_100_MA);
	vote(chg->usb_icl_votable, SW_QC3_VOTER, false, 0);
	vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER, false, 0);
	vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
	vote(chg->usb_icl_votable, THERMAL_THROTTLE_VOTER, false, 0);
	vote(chg->limited_irq_disable_votable, CHARGER_TYPE_VOTER,
			true, 0);
	vote(chg->hdc_irq_disable_votable, CHARGER_TYPE_VOTER, true, 0);
	vote(chg->hdc_irq_disable_votable, HDC_IRQ_VOTER, false, 0);

	/* Remove SW thermal regulation WA votes */
	vote(chg->usb_icl_votable, SW_THERM_REGULATION_VOTER, false, 0);
	vote(chg->pl_disable_votable, SW_THERM_REGULATION_VOTER, false, 0);
	vote(chg->dc_suspend_votable, SW_THERM_REGULATION_VOTER, false, 0);
	if (chg->cp_disable_votable)
		vote(chg->cp_disable_votable, SW_THERM_REGULATION_VOTER,
								false, 0);

	/* reset USBOV votes and cancel work */
	cancel_delayed_work_sync(&chg->usbov_dbc_work);
	vote(chg->awake_votable, USBOV_DBC_VOTER, false, 0);
	chg->dbc_usbov = false;

	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;
	chg->usbin_forced_max_uv = 0;
	chg->usb_icl_delta_ua = 0;
	chg->pulse_cnt = 0;
	chg->uusb_apsd_rerun_done = false;
	chg->chg_param.forced_main_fcc = 0;

	del_timer_sync(&chg->apsd_timer);
	chg->apsd_ext_timeout = false;

	/* write back the default FLOAT charger configuration */
	rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
				(u8)FLOAT_OPTIONS_MASK, chg->float_cfg);
	if (rc < 0)
		smblib_err(chg, "Couldn't write float charger options rc=%d\n",
			rc);

	/* clear USB ICL vote for USB_PSY_VOTER */
	rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't un-vote for USB ICL rc=%d\n", rc);

	/* clear USB ICL vote for DCP_VOTER */
	rc = vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	if (rc < 0)
		smblib_err(chg,
			"Couldn't un-vote DCP from USB ICL rc=%d\n", rc);

	/*
	 * if non-compliant charger caused UV, restore original max pulses
	 * and turn SUSPEND_ON_COLLAPSE_USBIN_BIT back on.
	 */
	if (chg->qc2_unsupported_voltage) {
		rc = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
				HVDCP_PULSE_COUNT_MAX_QC2_MASK,
				chg->qc2_max_pulses);
		if (rc < 0)
			smblib_err(chg, "Couldn't restore max pulses rc=%d\n",
					rc);

		rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
				SUSPEND_ON_COLLAPSE_USBIN_BIT,
				SUSPEND_ON_COLLAPSE_USBIN_BIT);
		if (rc < 0)
			smblib_err(chg, "Couldn't turn on SUSPEND_ON_COLLAPSE_USBIN_BIT rc=%d\n",
					rc);

		chg->qc2_unsupported_voltage = QC2_COMPLIANT;
	}

	chg->qc3p5_detected = false;
	smblib_update_usb_type(chg);
}

void smblib_suspend_on_debug_battery(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval val;

	rc = smblib_get_prop_from_bms(chg,
			POWER_SUPPLY_PROP_DEBUG_BATTERY, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get debug battery prop rc=%d\n", rc);
		return;
	}
	if (chg->suspend_input_on_debug_batt) {
		vote(chg->usb_icl_votable, DEBUG_BOARD_VOTER, val.intval, 0);
		vote(chg->dc_suspend_votable, DEBUG_BOARD_VOTER, val.intval, 0);
		if (val.intval)
			pr_info("Input suspended: Fake battery\n");
	} else {
		vote(chg->chg_disable_votable, DEBUG_BOARD_VOTER,
					val.intval, 0);
	}
}

int smblib_rerun_apsd_if_required(struct smb_charger *chg)
{
	union power_supply_propval val;
	int rc;

	rc = smblib_get_prop_usb_present(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb present rc = %d\n", rc);
		return rc;
	}

	if (!val.intval)
		return 0;

	rc = smblib_request_dpdm(chg, true);
	if (rc < 0)
		smblib_err(chg, "Couldn't to enable DPDM rc=%d\n", rc);

	chg->uusb_apsd_rerun_done = true;
	smblib_rerun_apsd(chg);

	return 0;
}

static int smblib_get_pulse_cnt(struct smb_charger *chg, int *count)
{
	*count = chg->pulse_cnt;
	return 0;
}

#define USBIN_25MA	25000
#define USBIN_100MA	100000
#define USBIN_150MA	150000
#define USBIN_500MA	500000
#define USBIN_900MA	900000
static int set_sdp_current(struct smb_charger *chg, int icl_ua)
{
	int rc;
	u8 icl_options;
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);

	pr_info("set icl : %d\n", icl_ua);
	/* power source is SDP */
	switch (icl_ua) {
	case USBIN_100MA:
		/* USB 2.0 100mA */
		icl_options = 0;
		break;
	case USBIN_150MA:
		/* USB 3.0 150mA */
		icl_options = CFG_USB3P0_SEL_BIT;
		break;
	case USBIN_500MA:
		/* USB 2.0 500mA */
		icl_options = USB51_MODE_BIT;
		break;
	case USBIN_900MA:
		/* USB 3.0 900mA */
		icl_options = CFG_USB3P0_SEL_BIT | USB51_MODE_BIT;
		break;
	default:
		return -EINVAL;
	}

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB &&
		apsd_result->pst == POWER_SUPPLY_TYPE_USB_FLOAT) {
		/*
		 * change the float charger configuration to SDP, if this
		 * is the case of SDP being detected as FLOAT
		 */
		rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
			FORCE_FLOAT_SDP_CFG_BIT, FORCE_FLOAT_SDP_CFG_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set float ICL options rc=%d\n",
						rc);
			return rc;
		}
	}

	rc = smblib_masked_write(chg, USBIN_ICL_OPTIONS_REG,
			CFG_USB3P0_SEL_BIT | USB51_MODE_BIT, icl_options);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL options rc=%d\n", rc);
		return rc;
	}

	rc = smblib_icl_override(chg, SW_OVERRIDE_USB51_MODE);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL override rc=%d\n", rc);
		return rc;
	}

	return rc;
}

int smblib_set_icl_current(struct smb_charger *chg, int icl_ua)
{
	int rc = 0;
	enum icl_override_mode icl_override = HW_AUTO_MODE;
	/* suspend if 25mA or less is requested */
	bool suspend = (icl_ua <= USBIN_25MA);

	if (chg->chg_param.smb_version == PMI632_SUBTYPE)
		schgm_flash_torch_priority(chg, suspend ? TORCH_BOOST_MODE :
					TORCH_BUCK_MODE);

	/* Do not configure ICL from SW for DAM cables */
	if (smblib_get_prop_typec_mode(chg) ==
			    POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY)
		return 0;

	if (suspend)
		return smblib_set_usb_suspend(chg, true);

	if (icl_ua == INT_MAX)
		goto set_mode;

	/* configure current */
	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB
		&& (chg->typec_legacy
		|| chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT
		|| chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)) {
		rc = set_sdp_current(chg, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set SDP ICL rc=%d\n", rc);
			goto out;
		}
	} else {
		/*
		 * Try USB 2.0/3,0 option first on USB path when maximum input
		 * current limit is 500mA or below for better accuracy; in case
		 * of error, proceed to use USB high-current mode.
		 */
		if (icl_ua <= USBIN_500MA) {
			rc = set_sdp_current(chg, icl_ua);
			if (rc >= 0)
				goto unsuspend;
		}

		rc = smblib_set_charge_param(chg, &chg->param.usb_icl, icl_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set HC ICL rc=%d\n", rc);
			goto out;
		}
		icl_override = SW_OVERRIDE_HC_MODE;
	}

set_mode:
	rc = smblib_icl_override(chg, icl_override);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set ICL override rc=%d\n", rc);
		goto out;
	}

unsuspend:
	/* unsuspend after configuring current and override */
	rc = smblib_set_usb_suspend(chg, false);
	if (rc < 0) {
		smblib_err(chg, "Couldn't resume input rc=%d\n", rc);
		goto out;
	}

	/* Re-run AICL */
	if (icl_override != SW_OVERRIDE_HC_MODE)
		rc = smblib_run_aicl(chg, RERUN_AICL);
out:
	return rc;
}

int smblib_get_icl_current(struct smb_charger *chg, int *icl_ua)
{
	int rc;

	rc = smblib_get_charge_param(chg, &chg->param.icl_max_stat, icl_ua);
	if (rc < 0)
		smblib_err(chg, "Couldn't get HC ICL rc=%d\n", rc);

	return rc;
}

int smblib_toggle_smb_en(struct smb_charger *chg, int toggle)
{
	int rc = 0;

	if (!toggle)
		return rc;

	rc = smblib_select_sec_charger(chg, chg->sec_chg_selected,
				chg->cp_reason, true);

	return rc;
}

int smblib_get_irq_status(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 reg;

	if (chg->wa_flags & SKIP_MISC_PBS_IRQ_WA) {
		val->intval = 0;
		return 0;
	}

	mutex_lock(&chg->irq_status_lock);
	/* Report and clear cached status */
	val->intval = chg->irq_status;
	chg->irq_status = 0;

	/* get real time status of pulse skip irq */
	rc = smblib_read(chg, MISC_PBS_RT_STS_REG, &reg);
	if (rc < 0)
		smblib_err(chg, "Couldn't read MISC_PBS_RT_STS_REG rc=%d\n",
				rc);
	else
		val->intval |= (reg & PULSE_SKIP_IRQ_BIT);
	mutex_unlock(&chg->irq_status_lock);

	return rc;
}

/****************************
 * uUSB Moisture Protection *
 ****************************/
#define MICRO_USB_DETECTION_ON_TIME_20_MS 0x08
#define MICRO_USB_DETECTION_PERIOD_X_100 0x03
#define U_USB_STATUS_WATER_PRESENT 0x00
static int smblib_set_moisture_protection(struct smb_charger *chg,
				bool enable)
{
	int rc = 0;

	if (chg->moisture_present == enable) {
		smblib_dbg(chg, PR_MISC, "No change in moisture protection status\n");
		return rc;
	}

	if (enable) {
		chg->moisture_present = true;

		/* Disable uUSB factory mode detection */
		rc = smblib_masked_write(chg, TYPEC_U_USB_CFG_REG,
					EN_MICRO_USB_FACTORY_MODE_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable uUSB factory mode detection rc=%d\n",
				rc);
			return rc;
		}

		/* Disable moisture detection and uUSB state change interrupt */
		rc = smblib_masked_write(chg, TYPE_C_INTERRUPT_EN_CFG_2_REG,
					TYPEC_WATER_DETECTION_INT_EN_BIT |
					MICRO_USB_STATE_CHANGE_INT_EN_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable moisture detection interrupt rc=%d\n",
			rc);
			return rc;
		}

		/* Set 1% duty cycle on ID detection */
		rc = smblib_masked_write(chg,
				((chg->chg_param.smb_version == PMI632_SUBTYPE)
				? PMI632_TYPEC_U_USB_WATER_PROTECTION_CFG_REG :
				TYPEC_U_USB_WATER_PROTECTION_CFG_REG),
				EN_MICRO_USB_WATER_PROTECTION_BIT |
				MICRO_USB_DETECTION_ON_TIME_CFG_MASK |
				MICRO_USB_DETECTION_PERIOD_CFG_MASK,
				EN_MICRO_USB_WATER_PROTECTION_BIT |
				MICRO_USB_DETECTION_ON_TIME_20_MS |
				MICRO_USB_DETECTION_PERIOD_X_100);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set 1 percent CC_ID duty cycle rc=%d\n",
				rc);
			return rc;
		}

		vote(chg->usb_icl_votable, MOISTURE_VOTER, true, 0);
	} else {
		chg->moisture_present = false;
		vote(chg->usb_icl_votable, MOISTURE_VOTER, false, 0);

		/* Enable moisture detection and uUSB state change interrupt */
		rc = smblib_masked_write(chg, TYPE_C_INTERRUPT_EN_CFG_2_REG,
					TYPEC_WATER_DETECTION_INT_EN_BIT |
					MICRO_USB_STATE_CHANGE_INT_EN_BIT,
					TYPEC_WATER_DETECTION_INT_EN_BIT |
					MICRO_USB_STATE_CHANGE_INT_EN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable moisture detection and uUSB state change interrupt rc=%d\n",
				rc);
			return rc;
		}

		/* Disable periodic monitoring of CC_ID pin */
		rc = smblib_write(chg,
				((chg->chg_param.smb_version == PMI632_SUBTYPE)
				? PMI632_TYPEC_U_USB_WATER_PROTECTION_CFG_REG :
				TYPEC_U_USB_WATER_PROTECTION_CFG_REG), 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable 1 percent CC_ID duty cycle rc=%d\n",
				rc);
			return rc;
		}

		/* Enable uUSB factory mode detection */
		rc = smblib_masked_write(chg, TYPEC_U_USB_CFG_REG,
					EN_MICRO_USB_FACTORY_MODE_BIT,
					EN_MICRO_USB_FACTORY_MODE_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable uUSB factory mode detection rc=%d\n",
				rc);
			return rc;
		}
	}

	smblib_dbg(chg, PR_MISC, "Moisture protection %s\n",
			chg->moisture_present ? "enabled" : "disabled");
	return rc;
}

/*********************
 * VOTABLE CALLBACKS *
 *********************/
static int smblib_smb_disable_override_vote_callback(struct votable *votable,
			void *data, int disable_smb, const char *client)
{
	struct smb_charger *chg = data;
	int rc = 0;

	/* Enable/disable SMB_EN pin */
	rc = smblib_masked_write(chg, MISC_SMB_EN_CMD_REG,
			SMB_EN_OVERRIDE_BIT | SMB_EN_OVERRIDE_VALUE_BIT,
			disable_smb ? SMB_EN_OVERRIDE_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't configure SMB_EN, rc=%d\n", rc);

	return rc;
}

static int smblib_dc_suspend_vote_callback(struct votable *votable, void *data,
			int suspend, const char *client)
{
	struct smb_charger *chg = data;

	if (chg->chg_param.smb_version == PMI632_SUBTYPE)
		return 0;

	/* resume input if suspend is invalid */
	if (suspend < 0)
		suspend = 0;

	return smblib_set_dc_suspend(chg, (bool)suspend);
}

static int smblib_awake_vote_callback(struct votable *votable, void *data,
			int awake, const char *client)
{
	struct smb_charger *chg = data;

	if (awake) {
		pr_info("batt_sys: pm_stay_awake [%s]\n", client);
		pm_stay_awake(chg->dev);
	} else {
		pr_info("batt_sys: pm_relax [vote] [%s]\n", client);
		pm_relax(chg->dev);
	}

	return 0;
}

static int smblib_chg_disable_vote_callback(struct votable *votable, void *data,
			int chg_disable, const char *client)
{
	struct smb_charger *chg = data;
	int rc;

	rc = smblib_masked_write(chg, CHARGING_ENABLE_CMD_REG,
				 CHARGING_ENABLE_CMD_BIT,
				 chg_disable ? 0 : CHARGING_ENABLE_CMD_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't %s charging rc=%d\n",
			chg_disable ? "disable" : "enable", rc);
		return rc;
	}

	return 0;
}

static int smblib_hdc_irq_disable_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq)
		return 0;

	if (chg->irq_info[HIGH_DUTY_CYCLE_IRQ].enabled) {
		if (disable)
			disable_irq_nosync(
				chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq);
	} else {
		if (!disable)
			enable_irq(chg->irq_info[HIGH_DUTY_CYCLE_IRQ].irq);
	}

	chg->irq_info[HIGH_DUTY_CYCLE_IRQ].enabled = !disable;

	return 0;
}

static int smblib_limited_irq_disable_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].irq)
		return 0;

	if (chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].enabled) {
		if (disable)
			disable_irq_nosync(
				chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].irq);
	} else {
		if (!disable)
			enable_irq(
				chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].irq);
	}

	chg->irq_info[INPUT_CURRENT_LIMITING_IRQ].enabled = !disable;

	return 0;
}

static int smblib_icl_irq_disable_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq)
		return 0;

	if (chg->irq_info[USBIN_ICL_CHANGE_IRQ].enabled) {
		if (disable)
			disable_irq_nosync(
				chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);
	} else {
		if (!disable)
			enable_irq(chg->irq_info[USBIN_ICL_CHANGE_IRQ].irq);
	}

	chg->irq_info[USBIN_ICL_CHANGE_IRQ].enabled = !disable;

	return 0;
}

static int smblib_temp_change_irq_disable_vote_callback(struct votable *votable,
				void *data, int disable, const char *client)
{
	struct smb_charger *chg = data;

	if (!chg->irq_info[TEMP_CHANGE_IRQ].irq)
		return 0;

	if (chg->irq_info[TEMP_CHANGE_IRQ].enabled && disable) {
		if (chg->irq_info[TEMP_CHANGE_IRQ].wake)
			disable_irq_wake(chg->irq_info[TEMP_CHANGE_IRQ].irq);
		disable_irq_nosync(chg->irq_info[TEMP_CHANGE_IRQ].irq);
	} else if (!chg->irq_info[TEMP_CHANGE_IRQ].enabled && !disable) {
		enable_irq(chg->irq_info[TEMP_CHANGE_IRQ].irq);
		if (chg->irq_info[TEMP_CHANGE_IRQ].wake)
			enable_irq_wake(chg->irq_info[TEMP_CHANGE_IRQ].irq);
	}

	chg->irq_info[TEMP_CHANGE_IRQ].enabled = !disable;

	return 0;
}

/*******************
 * VCONN REGULATOR *
 * *****************/

int smblib_vconn_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 stat, orientation;

	smblib_dbg(chg, PR_OTG, "enabling VCONN\n");

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}

	/* VCONN orientation is opposite to that of CC */
	orientation =
		stat & TYPEC_CCOUT_VALUE_BIT ? 0 : VCONN_EN_ORIENTATION_BIT;
	rc = smblib_masked_write(chg, TYPE_C_VCONN_CONTROL_REG,
				VCONN_EN_VALUE_BIT | VCONN_EN_ORIENTATION_BIT,
				VCONN_EN_VALUE_BIT | orientation);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_CCOUT_CONTROL_REG rc=%d\n",
			rc);
		return rc;
	}

	return 0;
}

int smblib_vconn_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;

	smblib_dbg(chg, PR_OTG, "disabling VCONN\n");
	rc = smblib_masked_write(chg, TYPE_C_VCONN_CONTROL_REG,
				 VCONN_EN_VALUE_BIT, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't disable vconn regulator rc=%d\n", rc);

	return 0;
}

int smblib_vconn_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;
	u8 cmd;

	rc = smblib_read(chg, TYPE_C_VCONN_CONTROL_REG, &cmd);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			rc);
		return rc;
	}

	return (cmd & VCONN_EN_VALUE_BIT) ? 1 : 0;
}

/*****************
 * OTG REGULATOR *
 *****************/
#ifdef CONFIG_PRODUCT_MOBA
int smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);

	pr_info("enabling OTG\n");
	usb1_otg_en=1;
	connect_device_type=1;
	gpio_set_value(chg->charge_1t1_gpio, 1);
	gpio_set_value(chg->charge_1t2_gpio, 1);
	gpio_set_value(chg->otg_gpio, 1);
	return 0;
}

int smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);

	pr_info("disabling OTG\n");
	usb1_otg_en=0;
	gpio_set_value(chg->otg_gpio, 0);
	gpio_set_value(chg->charge_1t1_gpio, 0);
	gpio_set_value(chg->charge_1t2_gpio, 0);
	return 0;
}

int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);

	return !!gpio_get_value(chg->otg_gpio);
}
#else
int smblib_vbus_regulator_enable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;

	smblib_dbg(chg, PR_OTG, "enabling OTG\n");

	rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG, OTG_EN_BIT, OTG_EN_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable OTG rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_vbus_regulator_disable(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc;

	smblib_dbg(chg, PR_OTG, "disabling OTG\n");

	rc = smblib_masked_write(chg, DCDC_CMD_OTG_REG, OTG_EN_BIT, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable OTG regulator rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_vbus_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct smb_charger *chg = rdev_get_drvdata(rdev);
	int rc = 0;
	u8 cmd;

	rc = smblib_read(chg, DCDC_CMD_OTG_REG, &cmd);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CMD_OTG rc=%d", rc);
		return rc;
	}

	return (cmd & OTG_EN_BIT) ? 1 : 0;
}
#endif

/********************
 * BATT PSY GETTERS *
 ********************/

int smblib_get_prop_input_suspend(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	val->intval
		= (get_client_vote(chg->usb_icl_votable, USER_VOTER) == 0)
		 && get_client_vote(chg->dc_suspend_votable, USER_VOTER);
	return 0;
}

int smblib_get_prop_batt_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATIF_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATIF_INT_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = !(stat & (BAT_THERM_OR_ID_MISSING_RT_STS_BIT
					| BAT_TERMINAL_MISSING_RT_STS_BIT));

	return rc;
}

int smblib_get_prop_batt_capacity(struct smb_charger *chg,
				  union power_supply_propval *val)
{
	int rc = -EINVAL;

	if (chg->fake_capacity >= 0) {
		val->intval = chg->fake_capacity;
		return 0;
	}

	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CAPACITY, val);

	return rc;
}

static bool is_charging_paused(struct smb_charger *chg)
{
	int rc;
	u8 val;

	rc = smblib_read(chg, CHARGING_PAUSE_CMD_REG, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CHARGING_PAUSE_CMD rc=%d\n", rc);
		return false;
	}

	return val & CHARGING_PAUSE_CMD_BIT;
}

int smblib_get_prop_batt_status(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	bool usb_online, dc_online;
	u8 stat;
	int rc, suspend = 0;

	if (chg->fake_chg_status_on_debug_batt) {
		rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_DEBUG_BATTERY, &pval);
		if (rc < 0) {
			pr_err_ratelimited("Couldn't get debug battery prop rc=%d\n",
					rc);
		} else if (pval.intval == 1) {
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			return 0;
		}
	}

	rc = smblib_get_prop_batt_health(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get batt health rc=%d\n", rc);
		return rc;
	}
	/*
	 * The charger status register shows charging even though the battery
	 * is discharging when the over voltage condition is hit. Report power
	 * supply state as NOT_CHARGING when the battery health reports
	 * over voltage.
	 */
	if (pval.intval == POWER_SUPPLY_HEALTH_OVERVOLTAGE) {
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	if (chg->dbc_usbov) {
		rc = smblib_get_prop_usb_present(chg, &pval);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't get usb present prop rc=%d\n", rc);
			return rc;
		}

		rc = smblib_get_usb_suspend(chg, &suspend);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't get usb suspend rc=%d\n", rc);
			return rc;
		}

		/*
		 * Report charging as long as USBOV is not debounced and
		 * charging path is un-suspended.
		 */
		if (pval.intval && !suspend) {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			return 0;
		}
	}

	rc = smblib_get_prop_usb_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb online property rc=%d\n",
			rc);
		return rc;
	}
	usb_online = (bool)pval.intval;

	rc = smblib_get_prop_dc_online(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get dc online property rc=%d\n",
			rc);
		return rc;
	}
	dc_online = (bool)pval.intval;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}
	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (!usb_online && !dc_online) {
		switch (stat) {
		case TERMINATE_CHARGE:
		case INHIBIT_CHARGE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}
		return rc;
	}

	switch (stat) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
	case FULLON_CHARGE:
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case TERMINATE_CHARGE:
	case INHIBIT_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	case DISABLE_CHARGE:
	case PAUSE_CHARGE:
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	if (is_charging_paused(chg)) {
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	/*
	 * If charge termination WA is active and has suspended charging, then
	 * continue reporting charging status as FULL.
	 */
	if (is_client_vote_enabled_locked(chg->usb_icl_votable,
						CHG_TERMINATION_VOTER)) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	if (val->intval != POWER_SUPPLY_STATUS_CHARGING)
		return 0;

	if (!usb_online && dc_online
		&& chg->fake_batt_status == POWER_SUPPLY_STATUS_FULL) {
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_5_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
				rc);
			return rc;
	}

	stat &= ENABLE_TRICKLE_BIT | ENABLE_PRE_CHARGING_BIT |
						ENABLE_FULLON_MODE_BIT;

	if (!stat)
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

int smblib_get_prop_batt_charge_type(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	switch (stat & BATTERY_CHARGER_STATUS_MASK) {
	case TRICKLE_CHARGE:
	case PRE_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case FULLON_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case TAPER_CHARGE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TAPER;
		break;
	default:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	return rc;
}

int smblib_get_prop_batt_health(struct smb_charger *chg,
				union power_supply_propval *val)
{
	union power_supply_propval pval;
	int rc;
	int effective_fv_uv;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_2_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "BATTERY_CHARGER_STATUS_2 = 0x%02x\n",
		   stat);

	if (stat & CHARGER_ERROR_STATUS_BAT_OV_BIT) {
		rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
		if (!rc) {
			/*
			 * If Vbatt is within 40mV above Vfloat, then don't
			 * treat it as overvoltage.
			 */
			effective_fv_uv = get_effective_result_locked(
							chg->fv_votable);
			if (pval.intval >= effective_fv_uv + 40000) {
				val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
				smblib_err(chg, "battery over-voltage vbat_fg = %duV, fv = %duV\n",
						pval.intval, effective_fv_uv);
				goto done;
			}
		}
	}

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_7_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_2 rc=%d\n",
			rc);
		return rc;
	}
	if (stat & BAT_TEMP_STATUS_TOO_COLD_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COLD;
	else if (stat & BAT_TEMP_STATUS_TOO_HOT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (stat & BAT_TEMP_STATUS_COLD_SOFT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_COOL;
	else if (stat & BAT_TEMP_STATUS_HOT_SOFT_BIT)
		val->intval = POWER_SUPPLY_HEALTH_WARM;
	else
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

done:
	return rc;
}

int smblib_get_prop_system_temp_level(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->system_temp_level;
	return 0;
}

#ifdef CONFIG_PRODUCT_MOBA
int smblib_get_prop_charge_thermal_status(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->charge_thermal_status;
	return 0;
}

int smblib_set_prop_charge_thermal_status(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (val->intval != chg->charge_thermal_status) {
		pr_info("batt_sys: charge_thermal_status %d to %d\n",
				chg->charge_thermal_status, val->intval);
		chg->charge_thermal_status = val->intval;
	}

	return 0;
}
#endif

int smblib_get_prop_system_temp_level_max(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_levels;
	return 0;
}

#ifdef CONFIG_PRODUCT_MOBA
int smblib_get_prop_therm_display_rate_level_max(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_levels_display_rate;
	return 0;
}

int smblib_get_prop_therm_display_rate_level(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_display_rate_level;
	return 0;
}

int smblib_set_prop_therm_display_rate_level(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels_display_rate <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels_display_rate)
		return -EINVAL;

	if (val->intval >= chg->thermal_levels_display_rate)
		chg->thermal_display_rate_level = chg->thermal_levels_display_rate - 1;
	else
		chg->thermal_display_rate_level = val->intval;

	return 0;
}

int smblib_get_prop_therm_display_rate_limit(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_mitigation_display_rate[chg->thermal_display_rate_level];
	return 0;
}

int smblib_get_prop_therm_display_rate(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_current_display_rate;
	return 0;
}

int smblib_set_prop_therm_display_rate(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	chg->thermal_current_display_rate = val->intval;
	return 0;
}

int smblib_get_prop_therm_speaker_level_max(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_levels_speaker;
	return 0;
}

int smblib_get_prop_therm_speaker_level(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_speaker_level;
	return 0;
}

int smblib_set_prop_therm_speaker_level(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels_speaker <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels_speaker)
		return -EINVAL;

	if (val->intval >= chg->thermal_levels_speaker)
		chg->thermal_speaker_level = chg->thermal_levels_speaker - 1;
	else
		chg->thermal_speaker_level = val->intval;

	return 0;
}

int smblib_get_prop_therm_speaker_limit(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_mitigation_speaker[chg->thermal_speaker_level];
	return 0;
}

int smblib_get_prop_therm_speaker(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_current_speaker;
	return 0;
}

int smblib_set_prop_therm_speaker(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	chg->thermal_current_speaker = val->intval;
	return 0;
}

int smblib_get_prop_therm_modem_5g_level_max(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_levels_modem_5g;
	return 0;
}

int smblib_get_prop_therm_modem_5g_level(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_modem_5g_level;
	return 0;
}

int smblib_set_prop_therm_modem_5g_level(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels_modem_5g <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels_modem_5g)
		return -EINVAL;

	if (val->intval >= chg->thermal_levels_modem_5g)
		chg->thermal_modem_5g_level = chg->thermal_levels_modem_5g - 1;
	else
		chg->thermal_modem_5g_level = val->intval;

	return 0;
}

int smblib_get_prop_therm_modem_5g_limit(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_mitigation_modem_5g[chg->thermal_modem_5g_level];
	return 0;
}

int smblib_get_prop_therm_modem_5g(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_current_modem_5g;
	return 0;
}

int smblib_set_prop_therm_modem_5g(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	chg->thermal_current_modem_5g = val->intval;
	return 0;
}

int smblib_get_prop_therm_camera_level_max(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_levels_camera;
	return 0;
}

int smblib_get_prop_therm_camera_level(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_camera_level;
	return 0;
}

int smblib_set_prop_therm_camera_level(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels_camera <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels_camera)
		return -EINVAL;

	if (val->intval >= chg->thermal_levels_camera)
		chg->thermal_camera_level = chg->thermal_levels_camera - 1;
	else
		chg->thermal_camera_level = val->intval;

	return 0;
}

int smblib_get_prop_therm_camera_limit(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_mitigation_camera[chg->thermal_camera_level];
	return 0;
}

int smblib_get_prop_therm_camera(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->thermal_current_camera;
	return 0;
}

int smblib_set_prop_therm_camera(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	chg->thermal_current_camera = val->intval;
	return 0;
}

#endif

int smblib_get_prop_input_current_limited(struct smb_charger *chg,
				union power_supply_propval *val)
{
	u8 stat;
	int rc;

	if (chg->fake_input_current_limited >= 0) {
		val->intval = chg->fake_input_current_limited;
		return 0;
	}

	rc = smblib_read(chg, AICL_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n", rc);
		return rc;
	}
	val->intval = (stat & SOFT_ILIMIT_BIT) || chg->is_hdc;
	return 0;
}

int smblib_get_prop_batt_iterm(struct smb_charger *chg,
		union power_supply_propval *val)
{
	int rc, temp;
	u8 stat, buf[2];

	/*
	 * Currently, only ADC comparator-based termination is supported,
	 * hence read only the threshold corresponding to ADC source.
	 * Proceed only if CHGR_ITERM_USE_ANALOG_BIT is 0.
	 */
	rc = smblib_read(chg, CHGR_ENG_CHARGING_CFG_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CHGR_ENG_CHARGING_CFG_REG rc=%d\n",
				rc);
		return rc;
	}

	if (stat & CHGR_ITERM_USE_ANALOG_BIT) {
		val->intval = -EINVAL;
		return 0;
	}

	rc = smblib_batch_read(chg, CHGR_ADC_ITERM_UP_THD_MSB_REG, buf, 2);

	if (rc < 0) {
		smblib_err(chg, "Couldn't read CHGR_ADC_ITERM_UP_THD_MSB_REG rc=%d\n",
				rc);
		return rc;
	}

	temp = buf[1] | (buf[0] << 8);
	temp = sign_extend32(temp, 15);

	if (chg->chg_param.smb_version == PMI632_SUBTYPE)
		temp = DIV_ROUND_CLOSEST(temp * ITERM_LIMITS_PMI632_MA,
					ADC_CHG_ITERM_MASK);
	else
		temp = DIV_ROUND_CLOSEST(temp * ITERM_LIMITS_PM8150B_MA,
					ADC_CHG_ITERM_MASK);

	val->intval = temp;

	return rc;
}

int smblib_get_prop_batt_charge_done(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
			rc);
		return rc;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;
	val->intval = (stat == TERMINATE_CHARGE);
	return 0;
}

int smblib_get_batt_current_now(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;

	rc = smblib_get_prop_from_bms(chg,
			POWER_SUPPLY_PROP_CURRENT_NOW, val);
	if (!rc)
		val->intval *= (-1);

	return rc;
}

/***********************
 * BATTERY PSY SETTERS *
 ***********************/

int smblib_set_prop_input_suspend(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	int rc;

#ifdef CONFIG_PRODUCT_MOBA
	if (force_usb_suspend)
		return rc;
#endif
	/* vote 0mA when suspended */
	rc = vote(chg->usb_icl_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s USB rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, USER_VOTER, (bool)val->intval, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't vote to %s DC rc=%d\n",
			(bool)val->intval ? "suspend" : "resume", rc);
		return rc;
	}

	power_supply_changed(chg->batt_psy);
	return rc;
}

int smblib_set_prop_batt_capacity(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	chg->fake_capacity = val->intval;

	power_supply_changed(chg->batt_psy);

	return 0;
}

int smblib_set_prop_batt_status(struct smb_charger *chg,
				  const union power_supply_propval *val)
{
	/* Faking battery full */
	if (val->intval == POWER_SUPPLY_STATUS_FULL)
		chg->fake_batt_status = val->intval;
	else
		chg->fake_batt_status = -EINVAL;

	power_supply_changed(chg->batt_psy);

	return 0;
}

int smblib_set_prop_system_temp_level(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	if (val->intval < 0)
		return -EINVAL;

	if (chg->thermal_levels <= 0)
		return -EINVAL;

	if (val->intval > chg->thermal_levels)
		return -EINVAL;

	if (val->intval >= chg->thermal_levels)
		chg->system_temp_level = chg->thermal_levels - 1;
	else
		chg->system_temp_level = val->intval;
#if 0

	if (chg->system_temp_level == chg->thermal_levels)
		return vote(chg->chg_disable_votable,
			THERMAL_DAEMON_VOTER, true, 0);

	vote(chg->chg_disable_votable, THERMAL_DAEMON_VOTER, false, 0);
	if (chg->system_temp_level == 0)
		return vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, false, 0);

	vote(chg->fcc_votable, THERMAL_DAEMON_VOTER, true,
			chg->thermal_mitigation[chg->system_temp_level]);
#endif
	return 0;
}

int smblib_set_prop_input_current_limited(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	chg->fake_input_current_limited = val->intval;
	return 0;
}

int smblib_set_prop_rechg_soc_thresh(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;
	u8 new_thr = DIV_ROUND_CLOSEST(val->intval * 255, 100);

	rc = smblib_write(chg, CHARGE_RCHG_SOC_THRESHOLD_CFG_REG,
			new_thr);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write to RCHG_SOC_THRESHOLD_CFG_REG rc=%d\n",
				rc);
		return rc;
	}

	chg->auto_recharge_soc = val->intval;

	return rc;
}

int smblib_run_aicl(struct smb_charger *chg, int type)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
								rc);
		return rc;
	}

	/* USB is suspended so skip re-running AICL */
	if (stat & USBIN_SUSPEND_STS_BIT)
		return rc;

	smblib_dbg(chg, PR_MISC, "re-running AICL\n");

	stat = (type == RERUN_AICL) ? RERUN_AICL_BIT : RESTART_AICL_BIT;
	rc = smblib_masked_write(chg, AICL_CMD_REG, stat, stat);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to AICL_CMD_REG rc=%d\n",
				rc);
	return 0;
}

static int smblib_dp_pulse(struct smb_charger *chg)
{
	int rc;

	/* QC 3.0 increment */
	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, SINGLE_INCREMENT_BIT,
			SINGLE_INCREMENT_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

static int smblib_dm_pulse(struct smb_charger *chg)
{
	int rc;

	/* QC 3.0 decrement */
	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, SINGLE_DECREMENT_BIT,
			SINGLE_DECREMENT_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

int smblib_force_vbus_voltage(struct smb_charger *chg, u8 val)
{
	int rc;

	rc = smblib_masked_write(chg, CMD_HVDCP_2_REG, val, val);
	if (rc < 0)
		smblib_err(chg, "Couldn't write to CMD_HVDCP_2_REG rc=%d\n",
				rc);

	return rc;
}

static void smblib_hvdcp_set_fsw(struct smb_charger *chg, int bit)
{
	switch (bit) {
	case QC_5V_BIT:
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_5V);
		break;
	case QC_9V_BIT:
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_9V);
		break;
	case QC_12V_BIT:
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_12V);
		break;
	default:
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_removal);
		break;
	}
}

#define QC3_PULSES_FOR_6V	5
#define QC3_PULSES_FOR_9V	20
#define QC3_PULSES_FOR_12V	35
static int smblib_hvdcp3_set_fsw(struct smb_charger *chg)
{
	int pulse_count, rc;

	rc = smblib_get_pulse_cnt(chg, &pulse_count);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
		return rc;
	}

	if (pulse_count < QC3_PULSES_FOR_6V)
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_5V);
	else if (pulse_count < QC3_PULSES_FOR_9V)
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_6V_8V);
	else if (pulse_count < QC3_PULSES_FOR_12V)
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_9V);
	else
		smblib_set_opt_switcher_freq(chg,
				chg->chg_freq.freq_12V);

	return 0;
}

static void smblib_hvdcp_adaptive_voltage_change(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP) {
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_CHANGE_STATUS rc=%d\n", rc);
			return;
		}

		smblib_hvdcp_set_fsw(chg, stat & QC_2P0_STATUS_MASK);
		vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER, false, 0);
	}

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3
		|| chg->real_charger_type == POWER_SUPPLY_TYPE_USB_HVDCP_3P5) {
		rc = smblib_hvdcp3_set_fsw(chg);
		if (rc < 0)
			smblib_err(chg, "Couldn't set QC3.0 Fsw rc=%d\n", rc);
	}

	power_supply_changed(chg->usb_main_psy);
}

int smblib_dp_dm(struct smb_charger *chg, int val)
{
	int target_icl_ua, rc = 0;
	union power_supply_propval pval;
	u8 stat;

	switch (val) {
	case POWER_SUPPLY_DP_DM_DP_PULSE:
		/*
		 * Pre-emptively increment pulse count to enable the setting
		 * of FSW prior to increasing voltage.
		 */
		chg->pulse_cnt++;

		rc = smblib_hvdcp3_set_fsw(chg);
		if (rc < 0)
			smblib_err(chg, "Couldn't set QC3.0 Fsw rc=%d\n", rc);

		rc = smblib_dp_pulse(chg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't increase pulse count rc=%d\n",
				rc);
			/*
			 * Increment pulse count failed;
			 * reset to former value.
			 */
			chg->pulse_cnt--;
		}

		smblib_dbg(chg, PR_PARALLEL, "DP_DM_DP_PULSE rc=%d cnt=%d\n",
				rc, chg->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_DM_PULSE:
		rc = smblib_dm_pulse(chg);
		if (!rc && chg->pulse_cnt)
			chg->pulse_cnt--;
		smblib_dbg(chg, PR_PARALLEL, "DP_DM_DM_PULSE rc=%d cnt=%d\n",
				rc, chg->pulse_cnt);
		break;
	case POWER_SUPPLY_DP_DM_ICL_DOWN:
		target_icl_ua = get_effective_result(chg->usb_icl_votable);
		if (target_icl_ua < 0) {
			/* no client vote, get the ICL from charger */
			rc = power_supply_get_property(chg->usb_psy,
					POWER_SUPPLY_PROP_HW_CURRENT_MAX,
					&pval);
			if (rc < 0) {
				smblib_err(chg, "Couldn't get max curr rc=%d\n",
					rc);
				return rc;
			}
			target_icl_ua = pval.intval;
		}

		/*
		 * Check if any other voter voted on USB_ICL in case of
		 * voter other than SW_QC3_VOTER reset and restart reduction
		 * again.
		 */
		if (target_icl_ua != get_client_vote(chg->usb_icl_votable,
							SW_QC3_VOTER))
			chg->usb_icl_delta_ua = 0;

		chg->usb_icl_delta_ua += 100000;
		vote(chg->usb_icl_votable, SW_QC3_VOTER, true,
						target_icl_ua - 100000);
		smblib_dbg(chg, PR_PARALLEL, "ICL DOWN ICL=%d reduction=%d\n",
				target_icl_ua, chg->usb_icl_delta_ua);
		break;
	case POWER_SUPPLY_DP_DM_FORCE_5V:
		rc = smblib_force_vbus_voltage(chg, FORCE_5V_BIT);
		if (rc < 0)
			pr_err("Failed to force 5V\n");
		break;
	case POWER_SUPPLY_DP_DM_FORCE_9V:
		if (chg->qc2_unsupported_voltage == QC2_NON_COMPLIANT_9V) {
			smblib_err(chg, "Couldn't set 9V: unsupported\n");
			return -EINVAL;
		}

		/* If we are increasing voltage to get to 9V, set FSW first */
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read QC_CHANGE_STATUS_REG rc=%d\n",
					rc);
			break;
		}

		if (stat & QC_5V_BIT) {
			/* Force 1A ICL before requesting higher voltage */
			vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER,
					true, 1000000);
			smblib_hvdcp_set_fsw(chg, QC_9V_BIT);
		}

		rc = smblib_force_vbus_voltage(chg, FORCE_9V_BIT);
		if (rc < 0)
			pr_err("Failed to force 9V\n");
		break;
	case POWER_SUPPLY_DP_DM_FORCE_12V:
		if (chg->qc2_unsupported_voltage == QC2_NON_COMPLIANT_12V) {
			smblib_err(chg, "Couldn't set 12V: unsupported\n");
			return -EINVAL;
		}

		/* If we are increasing voltage to get to 12V, set FSW first */
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read QC_CHANGE_STATUS_REG rc=%d\n",
					rc);
			break;
		}

		if ((stat & QC_9V_BIT) || (stat & QC_5V_BIT)) {
			/* Force 1A ICL before requesting higher voltage */
			vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER,
					true, 1000000);
			smblib_hvdcp_set_fsw(chg, QC_12V_BIT);
		}

		rc = smblib_force_vbus_voltage(chg, FORCE_12V_BIT);
		if (rc < 0)
			pr_err("Failed to force 12V\n");
		break;
	case POWER_SUPPLY_DP_DM_CONFIRMED_HVDCP3P5:
		chg->qc3p5_detected = true;
		smblib_update_usb_type(chg);
		break;
	case POWER_SUPPLY_DP_DM_ICL_UP:
	default:
		break;
	}

	return rc;
}

int smblib_disable_hw_jeita(struct smb_charger *chg, bool disable)
{
	int rc;
	u8 mask;

	/*
	 * Disable h/w base JEITA compensation if s/w JEITA is enabled
	 */
	mask = JEITA_EN_COLD_SL_FCV_BIT
		| JEITA_EN_HOT_SL_FCV_BIT
		| JEITA_EN_HOT_SL_CCC_BIT
		| JEITA_EN_COLD_SL_CCC_BIT,
#ifdef CONFIG_PRODUCT_MOBA
	rc = smblib_masked_write(chg, JEITA_EN_CFG_REG, 0xff,
			0);
#else
	rc = smblib_masked_write(chg, JEITA_EN_CFG_REG, mask,
			disable ? 0 : mask);
#endif
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't configure s/w jeita rc=%d\n",
				rc);
		return rc;
	}

	return 0;
}

static int smblib_set_sw_thermal_regulation(struct smb_charger *chg,
						bool enable)
{
	int rc = 0;

	if (!(chg->wa_flags & SW_THERM_REGULATION_WA))
		return rc;

	if (enable) {
		/*
		 * Configure min time to quickly address thermal
		 * condition.
		 */
		rc = smblib_masked_write(chg, SNARL_BARK_BITE_WD_CFG_REG,
			SNARL_WDOG_TIMEOUT_MASK, SNARL_WDOG_TMOUT_62P5MS);
		if (rc < 0) {
			smblib_err(chg, "Couldn't configure snarl wdog tmout, rc=%d\n",
					rc);
			return rc;
		}

		/*
		 * Schedule SW_THERM_REGULATION_WORK directly if USB input
		 * is suspended due to SW thermal regulation WA since WDOG
		 * IRQ won't trigger with input suspended.
		 */
		if (is_client_vote_enabled(chg->usb_icl_votable,
						SW_THERM_REGULATION_VOTER)) {
			vote(chg->awake_votable, SW_THERM_REGULATION_VOTER,
								true, 0);
			schedule_delayed_work(&chg->thermal_regulation_work, 0);
		}
	} else {
		cancel_delayed_work_sync(&chg->thermal_regulation_work);
		vote(chg->awake_votable, SW_THERM_REGULATION_VOTER, false, 0);
	}

	smblib_dbg(chg, PR_MISC, "WDOG SNARL INT %s\n",
				enable ? "Enabled" : "Disabled");

	return rc;
}

static int smblib_update_thermal_readings(struct smb_charger *chg)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	if (!chg->pl.psy)
		chg->pl.psy = power_supply_get_by_name("parallel");

	rc = smblib_read_iio_channel(chg, chg->iio.die_temp_chan,
				DIV_FACTOR_DECIDEGC, &chg->die_temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DIE TEMP channel, rc=%d\n", rc);
		return rc;
	}

	rc = smblib_read_iio_channel(chg, chg->iio.connector_temp_chan,
				DIV_FACTOR_DECIDEGC, &chg->connector_temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CONN TEMP channel, rc=%d\n", rc);
		return rc;
	}

	rc = smblib_read_iio_channel(chg, chg->iio.skin_temp_chan,
				DIV_FACTOR_DECIDEGC, &chg->skin_temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SKIN TEMP channel, rc=%d\n", rc);
		return rc;
	}

	if (chg->sec_chg_selected == POWER_SUPPLY_CHARGER_SEC_CP) {
		if (is_cp_available(chg)) {
			rc = power_supply_get_property(chg->cp_psy,
				POWER_SUPPLY_PROP_CP_DIE_TEMP, &pval);
			if (rc < 0) {
				smblib_err(chg, "Couldn't get smb1390 charger temp, rc=%d\n",
					rc);
				return rc;
			}
			chg->smb_temp = pval.intval;
		} else {
			smblib_dbg(chg, PR_MISC, "Coudln't find cp_psy\n");
			chg->smb_temp = -ENODATA;
		}
	} else if (chg->pl.psy && chg->sec_chg_selected ==
					POWER_SUPPLY_CHARGER_SEC_PL) {
		rc = power_supply_get_property(chg->pl.psy,
				POWER_SUPPLY_PROP_CHARGER_TEMP, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get smb1355 charger temp, rc=%d\n",
					rc);
			return rc;
		}
		chg->smb_temp = pval.intval;
	} else {
		chg->smb_temp = -ENODATA;
	}

	return rc;
}

/* SW thermal regulation thresholds in deciDegC */
#define DIE_TEMP_RST_THRESH		1000
#define DIE_TEMP_REG_H_THRESH		800
#define DIE_TEMP_REG_L_THRESH		600

#define CONNECTOR_TEMP_SHDN_THRESH	700
#define CONNECTOR_TEMP_RST_THRESH	600
#define CONNECTOR_TEMP_REG_H_THRESH	550
#define CONNECTOR_TEMP_REG_L_THRESH	500

#define SMB_TEMP_SHDN_THRESH		1400
#define SMB_TEMP_RST_THRESH		900
#define SMB_TEMP_REG_H_THRESH		800
#define SMB_TEMP_REG_L_THRESH		600

#define SKIN_TEMP_SHDN_THRESH		700
#define SKIN_TEMP_RST_THRESH		600
#define SKIN_TEMP_REG_H_THRESH		550
#define SKIN_TEMP_REG_L_THRESH		500

#define THERM_REG_RECHECK_DELAY_1S	1000	/* 1 sec */
#define THERM_REG_RECHECK_DELAY_8S	8000	/* 8 sec */
static int smblib_process_thermal_readings(struct smb_charger *chg)
{
	int rc = 0, wdog_timeout = SNARL_WDOG_TMOUT_8S;
	u32 thermal_status = TEMP_BELOW_RANGE;
	bool suspend_input = false, disable_smb = false;

	/*
	 * Following is the SW thermal regulation flow:
	 *
	 * TEMP_SHUT_DOWN_LEVEL: If either connector temp or skin temp
	 * exceeds their respective SHDN threshold. Need to suspend input
	 * and secondary charger.
	 *
	 * TEMP_SHUT_DOWN_SMB_LEVEL: If smb temp exceed its SHDN threshold
	 * but connector and skin temp are below it. Need to suspend SMB.
	 *
	 * TEMP_ALERT_LEVEL: If die, connector, smb or skin temp exceeds it's
	 * respective RST threshold. Stay put and monitor temperature closely.
	 *
	 * TEMP_ABOVE_RANGE or TEMP_WITHIN_RANGE or TEMP_BELOW_RANGE: If die,
	 * connector, smb or skin temp exceeds it's respective REG_H or REG_L
	 * threshold. Unsuspend input and SMB.
	 */
	if (chg->connector_temp > CONNECTOR_TEMP_SHDN_THRESH ||
		chg->skin_temp > SKIN_TEMP_SHDN_THRESH) {
		thermal_status = TEMP_SHUT_DOWN;
		wdog_timeout = SNARL_WDOG_TMOUT_1S;
		suspend_input = true;
		disable_smb = true;
		goto out;
	}

	if (chg->smb_temp > SMB_TEMP_SHDN_THRESH) {
		thermal_status = TEMP_SHUT_DOWN_SMB;
		wdog_timeout = SNARL_WDOG_TMOUT_1S;
		disable_smb = true;
		goto out;
	}

	if (chg->connector_temp > CONNECTOR_TEMP_RST_THRESH ||
			chg->skin_temp > SKIN_TEMP_RST_THRESH ||
			chg->smb_temp > SMB_TEMP_RST_THRESH ||
			chg->die_temp > DIE_TEMP_RST_THRESH) {
		thermal_status = TEMP_ALERT_LEVEL;
		wdog_timeout = SNARL_WDOG_TMOUT_1S;
		goto out;
	}

	if (chg->connector_temp > CONNECTOR_TEMP_REG_H_THRESH ||
			chg->skin_temp > SKIN_TEMP_REG_H_THRESH ||
			chg->smb_temp > SMB_TEMP_REG_H_THRESH ||
			chg->die_temp > DIE_TEMP_REG_H_THRESH) {
		thermal_status = TEMP_ABOVE_RANGE;
		wdog_timeout = SNARL_WDOG_TMOUT_1S;
		goto out;
	}

	if (chg->connector_temp > CONNECTOR_TEMP_REG_L_THRESH ||
			chg->skin_temp > SKIN_TEMP_REG_L_THRESH ||
			chg->smb_temp > SMB_TEMP_REG_L_THRESH ||
			chg->die_temp > DIE_TEMP_REG_L_THRESH) {
		thermal_status = TEMP_WITHIN_RANGE;
		wdog_timeout = SNARL_WDOG_TMOUT_8S;
	}
out:
	smblib_dbg(chg, PR_MISC, "Current temperatures: \tDIE_TEMP: %d,\tCONN_TEMP: %d,\tSMB_TEMP: %d,\tSKIN_TEMP: %d\nTHERMAL_STATUS: %d\n",
			chg->die_temp, chg->connector_temp, chg->smb_temp,
			chg->skin_temp, thermal_status);

	if (thermal_status != chg->thermal_status) {
		chg->thermal_status = thermal_status;
		/*
		 * If thermal level changes to TEMP ALERT LEVEL, don't
		 * enable/disable main/parallel charging.
		 */
		if (chg->thermal_status == TEMP_ALERT_LEVEL)
			goto exit;

		vote(chg->smb_override_votable, SW_THERM_REGULATION_VOTER,
				disable_smb, 0);

		/*
		 * Enable/disable secondary charger through votables to ensure
		 * that if SMB_EN pin get's toggled somehow, secondary charger
		 * remains enabled/disabled according to SW thermal regulation.
		 */
		if (!chg->cp_disable_votable)
			chg->cp_disable_votable = find_votable("CP_DISABLE");
		if (chg->cp_disable_votable)
			vote(chg->cp_disable_votable, SW_THERM_REGULATION_VOTER,
							disable_smb, 0);

		vote(chg->pl_disable_votable, SW_THERM_REGULATION_VOTER,
							disable_smb, 0);
		smblib_dbg(chg, PR_MISC, "Parallel %s as per SW thermal regulation\n",
				disable_smb ? "disabled" : "enabled");

		/*
		 * If thermal level changes to TEMP_SHUT_DOWN_SMB, don't
		 * enable/disable main charger.
		 */
		if (chg->thermal_status == TEMP_SHUT_DOWN_SMB)
			goto exit;

		/* Suspend input if SHDN threshold reached */
		vote(chg->dc_suspend_votable, SW_THERM_REGULATION_VOTER,
							suspend_input, 0);
		vote(chg->usb_icl_votable, SW_THERM_REGULATION_VOTER,
							suspend_input, 0);
		smblib_dbg(chg, PR_MISC, "USB/DC %s as per SW thermal regulation\n",
				suspend_input ? "suspended" : "unsuspended");
	}
exit:
	/*
	 * On USB suspend, WDOG IRQ stops triggering. To continue thermal
	 * monitoring and regulation until USB is plugged out, reschedule
	 * the SW thermal regulation work without releasing the wake lock.
	 */
	if (is_client_vote_enabled(chg->usb_icl_votable,
					SW_THERM_REGULATION_VOTER)) {
		schedule_delayed_work(&chg->thermal_regulation_work,
				msecs_to_jiffies(THERM_REG_RECHECK_DELAY_1S));
		return 0;
	}

	rc = smblib_masked_write(chg, SNARL_BARK_BITE_WD_CFG_REG,
			SNARL_WDOG_TIMEOUT_MASK, wdog_timeout);
	if (rc < 0)
		smblib_err(chg, "Couldn't set WD SNARL timer, rc=%d\n", rc);

	vote(chg->awake_votable, SW_THERM_REGULATION_VOTER, false, 0);
	return rc;
}

/*******************
 * DC PSY GETTERS *
 *******************/

int smblib_get_prop_voltage_wls_output(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->wls_psy) {
		chg->wls_psy = power_supply_get_by_name("wireless");
		if (!chg->wls_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->wls_psy,
				POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get POWER_SUPPLY_PROP_VOLTAGE_REGULATION, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_prop_dc_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	if (chg->chg_param.smb_version == PMI632_SUBTYPE) {
		val->intval = 0;
		return 0;
	}

	rc = smblib_read(chg, DCIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DCIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & DCIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_dc_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (chg->chg_param.smb_version == PMI632_SUBTYPE) {
		val->intval = 0;
		return 0;
	}

	if (get_client_vote(chg->dc_suspend_votable, USER_VOTER)) {
		val->intval = false;
		return rc;
	}

	if (is_client_vote_enabled(chg->dc_suspend_votable,
						CHG_TERMINATION_VOTER)) {
		rc = smblib_get_prop_dc_present(chg, val);
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_DCIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_STS_BIT);

	return rc;
}

int smblib_get_prop_dc_current_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	return smblib_get_charge_param(chg, &chg->param.dc_icl, &val->intval);
}

int smblib_get_prop_dc_voltage_max(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;
	val->intval = MICRO_12V;

	if (!chg->wls_psy)
		chg->wls_psy = power_supply_get_by_name("wireless");

	if (chg->wls_psy) {
		rc = power_supply_get_property(chg->wls_psy,
				POWER_SUPPLY_PROP_VOLTAGE_MAX,
				val);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't get VOLTAGE_MAX, rc=%d\n",
					rc);
			return rc;
		}
	}

	return 0;
}

int smblib_get_prop_dc_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->wls_psy) {
		chg->wls_psy = power_supply_get_by_name("wireless");
		if (!chg->wls_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->wls_psy,
				POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
				val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get POWER_SUPPLY_PROP_VOLTAGE_REGULATION, rc=%d\n",
				rc);
		return rc;
	}

	return rc;
}

/*******************
 * DC PSY SETTERS *
 *******************/

int smblib_set_prop_dc_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	chg->dcin_icl_user_set = true;
	return smblib_set_charge_param(chg, &chg->param.dc_icl, val->intval);
}

#define DCIN_AICL_RERUN_DELAY_MS	5000
int smblib_set_prop_voltage_wls_output(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	if (!chg->wls_psy) {
		chg->wls_psy = power_supply_get_by_name("wireless");
		if (!chg->wls_psy)
			return -ENODEV;
	}

	rc = power_supply_set_property(chg->wls_psy,
				POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set POWER_SUPPLY_PROP_VOLTAGE_REGULATION, rc=%d\n",
				rc);

	smblib_dbg(chg, PR_WLS, "%d\n", val->intval);

	/*
	 * When WLS VOUT goes down, the power-constrained adaptor may be able
	 * to supply more current, so allow it to do so - unless userspace has
	 * changed DCIN ICL value already due to thermal considerations.
	 */
	if (!chg->dcin_icl_user_set && (val->intval > 0) &&
			(val->intval < chg->last_wls_vout)) {
		alarm_start_relative(&chg->dcin_aicl_alarm,
				ms_to_ktime(DCIN_AICL_RERUN_DELAY_MS));
	}

	chg->last_wls_vout = val->intval;

	return rc;
}

int smblib_set_prop_dc_reset(struct smb_charger *chg)
{
	int rc;

	rc = vote(chg->dc_suspend_votable, VOUT_VOTER, true, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't suspend DC rc=%d\n", rc);
		return rc;
	}

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_EN_MASK,
				DCIN_EN_OVERRIDE_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set DCIN_EN_OVERRIDE_BIT rc=%d\n",
			rc);
		return rc;
	}

	rc = smblib_write(chg, DCIN_CMD_PON_REG, DCIN_PON_BIT | MID_CHG_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write %d to DCIN_CMD_PON_REG rc=%d\n",
			DCIN_PON_BIT | MID_CHG_BIT, rc);
		return rc;
	}

	/* Wait for 10ms to allow the charge to get drained */
	usleep_range(10000, 10010);

	rc = smblib_write(chg, DCIN_CMD_PON_REG, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't clear DCIN_CMD_PON_REG rc=%d\n", rc);
		return rc;
	}

	rc = smblib_masked_write(chg, DCIN_CMD_IL_REG, DCIN_EN_MASK, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't clear DCIN_EN_OVERRIDE_BIT rc=%d\n",
			rc);
		return rc;
	}

	rc = vote(chg->dc_suspend_votable, VOUT_VOTER, false, 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't unsuspend  DC rc=%d\n", rc);
		return rc;
	}

	smblib_dbg(chg, PR_MISC, "Wireless charger removal detection successful\n");
	return rc;
}

/*******************
 * USB PSY GETTERS *
 *******************/

int smblib_get_prop_usb_present(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN_RT_STS rc=%d\n", rc);
		return rc;
	}

	val->intval = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	return 0;
}

int smblib_get_prop_usb_online(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	if (get_client_vote_locked(chg->usb_icl_votable, USER_VOTER) == 0) {
		val->intval = false;
		return rc;
	}

	if (is_client_vote_enabled_locked(chg->usb_icl_votable,
					CHG_TERMINATION_VOTER)) {
		rc = smblib_get_prop_usb_present(chg, val);
		return rc;
	}

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "POWER_PATH_STATUS = 0x%02x\n",
		   stat);

	val->intval = (stat & USE_USBIN_BIT) &&
		      (stat & VALID_INPUT_POWER_SOURCE_STS_BIT);
	return rc;
}

int smblib_get_usb_online(struct smb_charger *chg,
			union power_supply_propval *val)
{
	int rc;

	rc = smblib_get_prop_usb_online(chg, val);
	if (!val->intval)
		goto exit;

	if (((chg->typec_mode == POWER_SUPPLY_TYPEC_SOURCE_DEFAULT) ||
		(chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB))
		&& (chg->real_charger_type == POWER_SUPPLY_TYPE_USB))
		val->intval = 0;
	else
		val->intval = 1;

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_UNKNOWN)
		val->intval = 0;

exit:
	return rc;
}

int smblib_get_prop_usb_voltage_max_design(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		if (chg->qc2_unsupported_voltage == QC2_NON_COMPLIANT_9V) {
			val->intval = MICRO_5V;
			break;
		} else if (chg->qc2_unsupported_voltage ==
				QC2_NON_COMPLIANT_12V) {
			val->intval = MICRO_9V;
			break;
		}
		/* else, fallthrough */
	case POWER_SUPPLY_TYPE_USB_HVDCP_3P5:
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
	case POWER_SUPPLY_TYPE_USB_PD:
		if (chg->chg_param.smb_version == PMI632_SUBTYPE)
			val->intval = MICRO_9V;
		else
			val->intval = MICRO_12V;
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

int smblib_get_prop_usb_voltage_max(struct smb_charger *chg,
					union power_supply_propval *val)
{
	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		if (chg->qc2_unsupported_voltage == QC2_NON_COMPLIANT_9V) {
			val->intval = MICRO_5V;
			break;
		} else if (chg->qc2_unsupported_voltage ==
				QC2_NON_COMPLIANT_12V) {
			val->intval = MICRO_9V;
			break;
		}
		/* else, fallthrough */
	case POWER_SUPPLY_TYPE_USB_HVDCP_3P5:
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		if (chg->chg_param.smb_version == PMI632_SUBTYPE)
			val->intval = MICRO_9V;
		else
			val->intval = MICRO_12V;
		break;
	case POWER_SUPPLY_TYPE_USB_PD:
		val->intval = chg->voltage_max_uv;
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

#define HVDCP3_STEP_UV	200000
#define HVDCP3P5_STEP_UV	20000
static int smblib_estimate_adaptor_voltage(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	int step_uv = HVDCP3_STEP_UV;

	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		val->intval = MICRO_12V;
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP_3P5:
		step_uv = HVDCP3P5_STEP_UV;
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		val->intval = MICRO_5V + (step_uv * chg->pulse_cnt);
		break;
	case POWER_SUPPLY_TYPE_USB_PD:
		/* Take the average of min and max values */
		val->intval = chg->voltage_min_uv +
			((chg->voltage_max_uv - chg->voltage_min_uv) / 2);
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

static int smblib_read_mid_voltage_chan(struct smb_charger *chg,
					union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.mid_chan)
		return -ENODATA;

	rc = iio_read_channel_processed(chg->iio.mid_chan, &val->intval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read MID channel rc=%d\n", rc);
		return rc;
	}

	/*
	 * If MID voltage < 1V, it is unreliable.
	 * Figure out voltage from registers and calculations.
	 */
	if (val->intval < 1000000)
		return smblib_estimate_adaptor_voltage(chg, val);

	return 0;
}

static int smblib_read_usbin_voltage_chan(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.usbin_v_chan)
		return -ENODATA;

	rc = iio_read_channel_processed(chg->iio.usbin_v_chan, &val->intval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USBIN channel rc=%d\n", rc);
		return rc;
	}

	return 0;
}

int smblib_get_prop_usb_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	int rc, ret = 0;
	u8 reg, adc_ch_reg;

	mutex_lock(&chg->adc_lock);

	if (chg->wa_flags & USBIN_ADC_WA) {
		/* Store ADC channel config in order to restore later */
		rc = smblib_read(chg, BATIF_ADC_CHANNEL_EN_REG, &adc_ch_reg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read ADC config rc=%d\n", rc);
			ret = rc;
			goto unlock;
		}

		/* Disable all ADC channels except IBAT channel */
		rc = smblib_write(chg, BATIF_ADC_CHANNEL_EN_REG,
						IBATT_CHANNEL_EN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't disable ADC channels rc=%d\n",
						rc);
			ret = rc;
			goto unlock;
		}
	}

	rc = smblib_get_prop_usb_present(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get usb presence status rc=%d\n", rc);
		ret = -ENODATA;
		goto restore_adc_config;
	}

	/*
	 * Skip reading voltage only if USB is not present and we are not in
	 * OTG mode.
	 */
	if (!pval.intval) {
		rc = smblib_read(chg, DCDC_CMD_OTG_REG, &reg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read CMD_OTG rc=%d", rc);
			goto restore_adc_config;
		}

		if (!(reg & OTG_EN_BIT))
			goto restore_adc_config;
	}

	/*
	 * For PM8150B, use MID_CHG ADC channel because overvoltage is observed
	 * to occur randomly in the USBIN channel, particularly at high
	 * voltages.
	 */
	if (chg->chg_param.smb_version == PM8150B_SUBTYPE)
		rc = smblib_read_mid_voltage_chan(chg, val);
	else
		rc = smblib_read_usbin_voltage_chan(chg, val);
	if (rc < 0) {
		smblib_err(chg, "Failed to read USBIN over vadc, rc=%d\n", rc);
		ret = rc;
	}

restore_adc_config:
	 /* Restore ADC channel config */
	if (chg->wa_flags & USBIN_ADC_WA) {
		rc = smblib_write(chg, BATIF_ADC_CHANNEL_EN_REG, adc_ch_reg);
		if (rc < 0)
			smblib_err(chg, "Couldn't write ADC config rc=%d\n",
						rc);
	}

unlock:
	mutex_unlock(&chg->adc_lock);

	return ret;
}

int smblib_get_prop_vph_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->iio.vph_v_chan)
		return -ENODATA;

	rc = iio_read_channel_processed(chg->iio.vph_v_chan, &val->intval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read vph channel rc=%d\n", rc);
		return rc;
	}

	return 0;
}

bool smblib_rsbux_low(struct smb_charger *chg, int r_thr)
{
	int r_sbu1, r_sbu2;
	bool ret = false;
	int rc;

	if (!chg->iio.sbux_chan)
		return false;

	/* disable crude sensors */
	rc = smblib_masked_write(chg, TYPE_C_CRUDE_SENSOR_CFG_REG,
			EN_SRC_CRUDE_SENSOR_BIT | EN_SNK_CRUDE_SENSOR_BIT,
			0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable crude sensor rc=%d\n", rc);
		return false;
	}

	/* select SBU1 as current source */
	rc = smblib_write(chg, TYPE_C_SBU_CFG_REG, SEL_SBU1_ISRC_VAL);
	if (rc < 0) {
		smblib_err(chg, "Couldn't select SBU1 rc=%d\n", rc);
		goto cleanup;
	}

	rc = iio_read_channel_processed(chg->iio.sbux_chan, &r_sbu1);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SBU1 rc=%d\n", rc);
		goto cleanup;
	}

	if (r_sbu1 < r_thr) {
		ret = true;
		goto cleanup;
	}

	/* select SBU2 as current source */
	rc = smblib_write(chg, TYPE_C_SBU_CFG_REG, SEL_SBU2_ISRC_VAL);
	if (rc < 0) {
		smblib_err(chg, "Couldn't select SBU1 rc=%d\n", rc);
		goto cleanup;
	}

	rc = iio_read_channel_processed(chg->iio.sbux_chan, &r_sbu2);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SBU1 rc=%d\n", rc);
		goto cleanup;
	}

	if (r_sbu2 < r_thr)
		ret = true;
cleanup:
	/* enable crude sensors */
	rc = smblib_masked_write(chg, TYPE_C_CRUDE_SENSOR_CFG_REG,
			EN_SRC_CRUDE_SENSOR_BIT | EN_SNK_CRUDE_SENSOR_BIT,
			EN_SRC_CRUDE_SENSOR_BIT | EN_SNK_CRUDE_SENSOR_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't enable crude sensor rc=%d\n", rc);

	/* disable current source */
	rc = smblib_write(chg, TYPE_C_SBU_CFG_REG, 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't select SBU1 rc=%d\n", rc);

	return ret;
}

int smblib_get_prop_charger_temp(struct smb_charger *chg,
				 union power_supply_propval *val)
{
	int temp, rc;
	int input_present;

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return rc;

	if (input_present == INPUT_NOT_PRESENT)
		return -ENODATA;

	if (chg->iio.temp_chan) {
		rc = iio_read_channel_processed(chg->iio.temp_chan,
				&temp);
		if (rc < 0) {
			pr_err("Error in reading temp channel, rc=%d\n", rc);
			return rc;
		}
		val->intval = temp / 100;
	} else {
		return -ENODATA;
	}

	return rc;
}

int smblib_get_prop_typec_cc_orientation(struct smb_charger *chg,
					 union power_supply_propval *val)
{
	int rc = 0;
	u8 stat;

	val->intval = 0;

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		return 0;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n", rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_4 = 0x%02x\n", stat);

	if (stat & CC_ATTACHED_BIT)
		val->intval = (bool)(stat & CC_ORIENTATION_BIT) + 1;

	return rc;
}

static const char * const smblib_typec_mode_name[] = {
	[POWER_SUPPLY_TYPEC_NONE]		  = "NONE",
	[POWER_SUPPLY_TYPEC_SOURCE_DEFAULT]	  = "SOURCE_DEFAULT",
	[POWER_SUPPLY_TYPEC_SOURCE_MEDIUM]	  = "SOURCE_MEDIUM",
	[POWER_SUPPLY_TYPEC_SOURCE_HIGH]	  = "SOURCE_HIGH",
	[POWER_SUPPLY_TYPEC_NON_COMPLIANT]	  = "NON_COMPLIANT",
	[POWER_SUPPLY_TYPEC_SINK]		  = "SINK",
	[POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE]   = "SINK_POWERED_CABLE",
	[POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY] = "SINK_DEBUG_ACCESSORY",
	[POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER]   = "SINK_AUDIO_ADAPTER",
	[POWER_SUPPLY_TYPEC_POWERED_CABLE_ONLY]   = "POWERED_CABLE_ONLY",
};

static int smblib_get_prop_ufp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_SNK_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_1 rc=%d\n", rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_STATUS_1 = 0x%02x\n", stat);

	switch (stat & DETECTED_SRC_TYPE_MASK) {
	case SNK_RP_STD_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_DEFAULT;
	case SNK_RP_1P5_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_MEDIUM;
	case SNK_RP_3P0_BIT:
		return POWER_SUPPLY_TYPEC_SOURCE_HIGH;
	case SNK_RP_SHORT_BIT:
		return POWER_SUPPLY_TYPEC_NON_COMPLIANT;
	case SNK_DAM_500MA_BIT:
	case SNK_DAM_1500MA_BIT:
	case SNK_DAM_3000MA_BIT:
		return POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

static int smblib_get_prop_dfp_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	if (chg->lpd_stage == LPD_STAGE_COMMIT)
		return POWER_SUPPLY_TYPEC_NONE;

	rc = smblib_read(chg, TYPE_C_SRC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_SRC_STATUS_REG rc=%d\n",
				rc);
		return POWER_SUPPLY_TYPEC_NONE;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_SRC_STATUS_REG = 0x%02x\n", stat);

	switch (stat & DETECTED_SNK_TYPE_MASK) {
	case AUDIO_ACCESS_RA_RA_BIT:
		return POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER;
	case SRC_DEBUG_ACCESS_BIT:
		return POWER_SUPPLY_TYPEC_SINK_DEBUG_ACCESSORY;
	case SRC_RD_RA_VCONN_BIT:
		return POWER_SUPPLY_TYPEC_SINK_POWERED_CABLE;
	case SRC_RD_OPEN_BIT:
		return POWER_SUPPLY_TYPEC_SINK;
	default:
		break;
	}

	return POWER_SUPPLY_TYPEC_NONE;
}

static int smblib_get_prop_typec_mode(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n",
				rc);
		return 0;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_MISC_STATUS_REG = 0x%02x\n", stat);

	if (stat & SNK_SRC_MODE_BIT)
		return smblib_get_prop_dfp_mode(chg);
	else
		return smblib_get_prop_ufp_mode(chg);
}

inline int smblib_get_usb_prop_typec_mode(struct smb_charger *chg,
				union power_supply_propval *val)
{
	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		val->intval = POWER_SUPPLY_TYPEC_NONE;
	else
		val->intval = chg->typec_mode;

	return 0;
}

int smblib_get_prop_typec_power_role(struct smb_charger *chg,
				     union power_supply_propval *val)
{
	int rc = 0;
	u8 ctrl;

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB) {
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		return 0;
	}

	rc = smblib_read(chg, TYPE_C_MODE_CFG_REG, &ctrl);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_MODE_CFG_REG rc=%d\n",
			rc);
		return rc;
	}
	smblib_dbg(chg, PR_REGISTER, "TYPE_C_MODE_CFG_REG = 0x%02x\n",
		   ctrl);

	if (ctrl & TYPEC_DISABLE_CMD_BIT) {
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		return rc;
	}

	switch (ctrl & (EN_SRC_ONLY_BIT | EN_SNK_ONLY_BIT)) {
	case 0:
		val->intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		break;
	case EN_SRC_ONLY_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
		break;
	case EN_SNK_ONLY_BIT:
		val->intval = POWER_SUPPLY_TYPEC_PR_SINK;
		break;
	default:
		val->intval = POWER_SUPPLY_TYPEC_PR_NONE;
		smblib_err(chg, "unsupported power role 0x%02lx\n",
			ctrl & (EN_SRC_ONLY_BIT | EN_SNK_ONLY_BIT));
		return -EINVAL;
	}

	chg->power_role = val->intval;
	return rc;
}

static inline bool typec_in_src_mode(struct smb_charger *chg)
{
	return (chg->typec_mode > POWER_SUPPLY_TYPEC_NONE &&
		chg->typec_mode < POWER_SUPPLY_TYPEC_SOURCE_DEFAULT);
}

int smblib_get_prop_typec_select_rp(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc, rp;
	u8 stat;

	if (!typec_in_src_mode(chg))
		return -ENODATA;

	rc = smblib_read(chg, TYPE_C_CURRSRC_CFG_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_CURRSRC_CFG_REG rc=%d\n",
				rc);
		return rc;
	}

	switch (stat & TYPEC_SRC_RP_SEL_MASK) {
	case TYPEC_SRC_RP_STD:
		rp = POWER_SUPPLY_TYPEC_SRC_RP_STD;
		break;
	case TYPEC_SRC_RP_1P5A:
		rp = POWER_SUPPLY_TYPEC_SRC_RP_1P5A;
		break;
	case TYPEC_SRC_RP_3A:
	case TYPEC_SRC_RP_3A_DUPLICATE:
		rp = POWER_SUPPLY_TYPEC_SRC_RP_3A;
		break;
	default:
		return -EINVAL;
	}

	val->intval = rp;

	return 0;
}

int smblib_get_prop_usb_current_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };
	int rc = 0, buck_scale = 1, boost_scale = 1;

	if (chg->iio.usbin_i_chan) {
		rc = iio_read_channel_processed(chg->iio.usbin_i_chan,
				&val->intval);
		if (rc < 0) {
			pr_err("Error in reading USBIN_I channel, rc=%d\n", rc);
			return rc;
		}

		/*
		 * For PM8150B, scaling factor = reciprocal of
		 * 0.2V/A in Buck mode, 0.4V/A in Boost mode.
		 * For PMI632, scaling factor = reciprocal of
		 * 0.4V/A in Buck mode, 0.8V/A in Boost mode.
		 */
		switch (chg->chg_param.smb_version) {
		case PMI632_SUBTYPE:
			buck_scale = 40;
			boost_scale = 80;
			break;
		default:
			buck_scale = 20;
			boost_scale = 40;
			break;
		}

		if (chg->otg_present || smblib_get_prop_dfp_mode(chg) !=
				POWER_SUPPLY_TYPEC_NONE) {
			val->intval = DIV_ROUND_CLOSEST(val->intval * 100,
								boost_scale);
			return rc;
		}

		rc = smblib_get_prop_usb_present(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get usb present status,rc=%d\n",
				rc);
			return -ENODATA;
		}

		/* If USB is not present, return 0 */
		if (!pval.intval)
			val->intval = 0;
		else
			val->intval = DIV_ROUND_CLOSEST(val->intval * 100,
								buck_scale);
	} else {
		val->intval = 0;
		rc = -ENODATA;
	}

	return rc;
}

int smblib_get_prop_low_power(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	int rc;
	u8 stat;

	if (chg->sink_src_mode != SRC_MODE)
		return -ENODATA;

	rc = smblib_read(chg, TYPE_C_SRC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_SRC_STATUS_REG rc=%d\n",
				rc);
		return rc;
	}

	val->intval = !(stat & SRC_HIGH_BATT_BIT);

	return 0;
}

int smblib_get_prop_input_current_settled(struct smb_charger *chg,
					  union power_supply_propval *val)
{
	return smblib_get_charge_param(chg, &chg->param.icl_stat, &val->intval);
}

int smblib_get_prop_input_voltage_settled(struct smb_charger *chg,
						union power_supply_propval *val)
{
	int rc, pulses;
	int step_uv = HVDCP3_STEP_UV;

	switch (chg->real_charger_type) {
	case POWER_SUPPLY_TYPE_USB_HVDCP_3P5:
		step_uv = HVDCP3P5_STEP_UV;
	case POWER_SUPPLY_TYPE_USB_HVDCP_3:
		rc = smblib_get_pulse_cnt(chg, &pulses);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't read QC_PULSE_COUNT rc=%d\n", rc);
			return 0;
		}
		val->intval = MICRO_5V + step_uv * pulses;
		break;
	case POWER_SUPPLY_TYPE_USB_PD:
		val->intval = chg->voltage_min_uv;
		break;
	default:
		val->intval = MICRO_5V;
		break;
	}

	return 0;
}

int smblib_get_prop_pd_in_hard_reset(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	val->intval = chg->pd_hard_reset;
	return 0;
}

int smblib_get_pe_start(struct smb_charger *chg,
			       union power_supply_propval *val)
{
	val->intval = chg->ok_to_pd;
	return 0;
}

int smblib_get_prop_smb_health(struct smb_charger *chg)
{
	int rc;
	int input_present;
	union power_supply_propval prop = {0, };

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return rc;

	if ((input_present == INPUT_NOT_PRESENT) || (!is_cp_available(chg)))
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	rc = power_supply_get_property(chg->cp_psy,
				POWER_SUPPLY_PROP_CP_DIE_TEMP, &prop);
	if (rc < 0)
		return rc;

	if (prop.intval > SMB_TEMP_RST_THRESH)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (prop.intval > SMB_TEMP_REG_H_THRESH)
		return POWER_SUPPLY_HEALTH_HOT;

	if (prop.intval > SMB_TEMP_REG_L_THRESH)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_COOL;
}

int smblib_get_prop_die_health(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	int input_present;

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return rc;

	if (input_present == INPUT_NOT_PRESENT)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	if (chg->wa_flags & SW_THERM_REGULATION_WA) {
		if (chg->die_temp == -ENODATA)
			return POWER_SUPPLY_HEALTH_UNKNOWN;

		if (chg->die_temp > DIE_TEMP_RST_THRESH)
			return POWER_SUPPLY_HEALTH_OVERHEAT;

		if (chg->die_temp > DIE_TEMP_REG_H_THRESH)
			return POWER_SUPPLY_HEALTH_HOT;

		if (chg->die_temp > DIE_TEMP_REG_L_THRESH)
			return POWER_SUPPLY_HEALTH_WARM;

		return POWER_SUPPLY_HEALTH_COOL;
	}

	rc = smblib_read(chg, DIE_TEMP_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DIE_TEMP_STATUS_REG, rc=%d\n",
				rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (stat & DIE_TEMP_RST_BIT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (stat & DIE_TEMP_UB_BIT)
		return POWER_SUPPLY_HEALTH_HOT;

	if (stat & DIE_TEMP_LB_BIT)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_COOL;
}

int smblib_get_die_health(struct smb_charger *chg,
			union power_supply_propval *val)
{
	if (chg->die_health == -EINVAL)
		val->intval = smblib_get_prop_die_health(chg);
	else
		val->intval = chg->die_health;

	return 0;
}

int smblib_get_prop_scope(struct smb_charger *chg,
			union power_supply_propval *val)
{
	int rc;
	union power_supply_propval pval;

	val->intval = POWER_SUPPLY_SCOPE_UNKNOWN;
	rc = smblib_get_prop_usb_present(chg, &pval);
	if (rc < 0)
		return rc;

	val->intval = pval.intval ? POWER_SUPPLY_SCOPE_DEVICE
		: chg->otg_present ? POWER_SUPPLY_SCOPE_SYSTEM
		: POWER_SUPPLY_SCOPE_UNKNOWN;

	return 0;
}

static int smblib_get_typec_connector_temp_status(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	if (chg->connector_health != -EINVAL)
		return chg->connector_health;

	if (chg->wa_flags & SW_THERM_REGULATION_WA) {
		if (chg->connector_temp == -ENODATA)
			return POWER_SUPPLY_HEALTH_UNKNOWN;

		if (chg->connector_temp > CONNECTOR_TEMP_RST_THRESH)
			return POWER_SUPPLY_HEALTH_OVERHEAT;

		if (chg->connector_temp > CONNECTOR_TEMP_REG_H_THRESH)
			return POWER_SUPPLY_HEALTH_HOT;

		if (chg->connector_temp > CONNECTOR_TEMP_REG_L_THRESH)
			return POWER_SUPPLY_HEALTH_WARM;

		return POWER_SUPPLY_HEALTH_COOL;
	}

	rc = smblib_read(chg, CONNECTOR_TEMP_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read CONNECTOR_TEMP_STATUS_REG, rc=%d\n",
				rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (stat & CONNECTOR_TEMP_RST_BIT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (stat & CONNECTOR_TEMP_UB_BIT)
		return POWER_SUPPLY_HEALTH_HOT;

	if (stat & CONNECTOR_TEMP_LB_BIT)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_COOL;
}

int smblib_get_skin_temp_status(struct smb_charger *chg)
{
	int rc;
	u8 stat;

	if (!chg->en_skin_therm_mitigation)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	rc = smblib_read(chg, SKIN_TEMP_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SKIN_TEMP_STATUS_REG, rc=%d\n",
				rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	}

	if (stat & SKIN_TEMP_RST_BIT)
		return POWER_SUPPLY_HEALTH_OVERHEAT;

	if (stat & SKIN_TEMP_UB_BIT)
		return POWER_SUPPLY_HEALTH_HOT;

	if (stat & SKIN_TEMP_LB_BIT)
		return POWER_SUPPLY_HEALTH_WARM;

	return POWER_SUPPLY_HEALTH_COOL;
}

int smblib_get_prop_connector_health(struct smb_charger *chg)
{
	bool dc_present, usb_present;
	int input_present;
	int rc;

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	dc_present = input_present & INPUT_PRESENT_DC;
	usb_present = input_present & INPUT_PRESENT_USB;

	if (usb_present)
		return smblib_get_typec_connector_temp_status(chg);

	/*
	 * In PM8150B, SKIN channel measures Wireless charger receiver
	 * temp, used to regulate DC ICL.
	 */
	if (chg->chg_param.smb_version == PM8150B_SUBTYPE && dc_present)
		return smblib_get_skin_temp_status(chg);

	return POWER_SUPPLY_HEALTH_COOL;
}

static int get_rp_based_dcp_current(struct smb_charger *chg, int typec_mode)
{
	int rp_ua;

	switch (typec_mode) {
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		rp_ua = TYPEC_HIGH_CURRENT_UA;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
	/* fall through */
	default:
		rp_ua = DCP_CURRENT_UA;
	}

	return rp_ua;
}

/*******************
 * USB PSY SETTERS *
 * *****************/

int smblib_set_prop_pd_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, icl;

	if (chg->pd_active) {
		icl = get_client_vote(chg->usb_icl_votable, PD_VOTER);
		rc = vote(chg->usb_icl_votable, PD_VOTER, true, val->intval);
		if (val->intval != icl)
			power_supply_changed(chg->usb_psy);
	} else {
		rc = -EPERM;
	}

	return rc;
}

static int smblib_handle_usb_current(struct smb_charger *chg,
					int usb_current)
{
	int rc = 0, rp_ua, typec_mode;
	union power_supply_propval val = {0, };

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB_FLOAT) {
		if (usb_current == -ETIMEDOUT) {
			if ((chg->float_cfg & FLOAT_OPTIONS_MASK)
						== FORCE_FLOAT_SDP_CFG_BIT) {
				/*
				 * Confiugure USB500 mode if Float charger is
				 * configured for SDP mode.
				 */
				rc = vote(chg->usb_icl_votable,
					SW_ICL_MAX_VOTER, true, USBIN_500MA);
				if (rc < 0)
					smblib_err(chg,
						"Couldn't set SDP ICL rc=%d\n",
						rc);
				return rc;
			}

			if (chg->connector_type ==
					POWER_SUPPLY_CONNECTOR_TYPEC) {
				/*
				 * Valid FLOAT charger, report the current
				 * based of Rp.
				 */
				typec_mode = smblib_get_prop_typec_mode(chg);
				rp_ua = get_rp_based_dcp_current(chg,
								typec_mode);
				rc = vote(chg->usb_icl_votable,
						SW_ICL_MAX_VOTER, true, rp_ua);
				if (rc < 0)
					return rc;
			} else {
				rc = vote(chg->usb_icl_votable,
					SW_ICL_MAX_VOTER, true, DCP_CURRENT_UA);
				if (rc < 0)
					return rc;
			}
		} else {
			/*
			 * FLOAT charger detected as SDP by USB driver,
			 * charge with the requested current and update the
			 * real_charger_type
			 */
			chg->real_charger_type = POWER_SUPPLY_TYPE_USB;
			rc = vote(chg->usb_icl_votable, USB_PSY_VOTER,
						true, usb_current);
			if (rc < 0)
				return rc;
			rc = vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER,
							false, 0);
			if (rc < 0)
				return rc;
		}
	} else {
		rc = smblib_get_prop_usb_present(chg, &val);
		if (!rc && !val.intval)
			return 0;

		/* if flash is active force 500mA */
		if ((usb_current < SDP_CURRENT_UA) && is_flash_active(chg))
			usb_current = SDP_CURRENT_UA;

		rc = vote(chg->usb_icl_votable, USB_PSY_VOTER, true,
							usb_current);
		if (rc < 0) {
			pr_err("Couldn't vote ICL USB_PSY_VOTER rc=%d\n", rc);
			return rc;
		}

		rc = vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
		if (rc < 0) {
			pr_err("Couldn't remove SW_ICL_MAX vote rc=%d\n", rc);
			return rc;
		}

	}

	return 0;
}

int smblib_set_prop_sdp_current_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	union power_supply_propval pval;
	int rc = 0;
	int usb_t_curr = 500000;

	if (!chg->pd_active) {
		rc = smblib_get_prop_usb_present(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get usb present rc = %d\n",
						rc);
			return rc;
		}

		/* handle the request only when USB is present */
		if (pval.intval){
			if (val->intval > usb_t_curr)
				rc = smblib_handle_usb_current(chg, usb_t_curr);
			else
				rc = smblib_handle_usb_current(chg, val->intval);
			}
	} else if (chg->system_suspend_supported) {
		if (val->intval <= USBIN_25MA)
			rc = vote(chg->usb_icl_votable,
				PD_SUSPEND_SUPPORTED_VOTER, true, val->intval);
		else
			rc = vote(chg->usb_icl_votable,
				PD_SUSPEND_SUPPORTED_VOTER, false, 0);
	}
	return rc;
}

int smblib_set_prop_boost_current(struct smb_charger *chg,
					const union power_supply_propval *val)
{
	int rc = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher,
				val->intval <= chg->boost_threshold_ua ?
				chg->chg_freq.freq_below_otg_threshold :
				chg->chg_freq.freq_above_otg_threshold);
	if (rc < 0) {
		dev_err(chg->dev, "Error in setting freq_boost rc=%d\n", rc);
		return rc;
	}

	chg->boost_current_ua = val->intval;
	return rc;
}

int smblib_set_prop_usb_voltage_max_limit(struct smb_charger *chg,
					const union power_supply_propval *val)
{
	union power_supply_propval pval = {0, };

	/* Exit if same value is re-configured */
	if (val->intval == chg->usbin_forced_max_uv)
		return 0;

	smblib_get_prop_usb_voltage_max_design(chg, &pval);

	if (val->intval >= MICRO_5V && val->intval <= pval.intval) {
		chg->usbin_forced_max_uv = val->intval;
		smblib_dbg(chg, PR_MISC, "Max VBUS limit changed to: %d\n",
				val->intval);
	} else if (chg->usbin_forced_max_uv) {
		chg->usbin_forced_max_uv = 0;
	} else {
		return 0;
	}

	power_supply_changed(chg->usb_psy);

	return 0;
}

void smblib_typec_irq_config(struct smb_charger *chg, bool en)
{
	if (en == chg->typec_irq_en)
		return;

	if (en) {
		enable_irq(
			chg->irq_info[TYPEC_ATTACH_DETACH_IRQ].irq);
		enable_irq(
			chg->irq_info[TYPEC_CC_STATE_CHANGE_IRQ].irq);
		enable_irq(
			chg->irq_info[TYPEC_OR_RID_DETECTION_CHANGE_IRQ].irq);
	} else {
		disable_irq_nosync(
			chg->irq_info[TYPEC_ATTACH_DETACH_IRQ].irq);
		disable_irq_nosync(
			chg->irq_info[TYPEC_CC_STATE_CHANGE_IRQ].irq);
		disable_irq_nosync(
			chg->irq_info[TYPEC_OR_RID_DETECTION_CHANGE_IRQ].irq);
	}

	chg->typec_irq_en = en;
}

#define PR_LOCK_TIMEOUT_MS	1000
int smblib_set_prop_typec_power_role(struct smb_charger *chg,
				     const union power_supply_propval *val)
{
	int rc = 0;
	u8 power_role;
	enum power_supply_typec_mode typec_mode;
	bool snk_attached = false, src_attached = false, is_pr_lock = false;

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		return 0;

	smblib_dbg(chg, PR_MISC, "power role change: %d --> %d!",
			chg->power_role, val->intval);

	if (chg->power_role == val->intval) {
		smblib_dbg(chg, PR_MISC, "power role already in %d, ignore!",
				chg->power_role);
		return 0;
	}

	typec_mode = smblib_get_prop_typec_mode(chg);
	if (typec_mode >= POWER_SUPPLY_TYPEC_SINK &&
			typec_mode <= POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER)
		snk_attached = true;
	else if (typec_mode >= POWER_SUPPLY_TYPEC_SOURCE_DEFAULT &&
			typec_mode <= POWER_SUPPLY_TYPEC_SOURCE_HIGH)
		src_attached = true;

	/*
	 * If current power role is in DRP, and type-c is already in the
	 * mode (source or sink) that's being requested, it means this is
	 * a power role locking request from USBPD driver. Disable type-c
	 * related interrupts for locking power role to avoid the redundant
	 * notifications.
	 */
	if ((chg->power_role == POWER_SUPPLY_TYPEC_PR_DUAL) &&
		((src_attached && val->intval == POWER_SUPPLY_TYPEC_PR_SINK) ||
		(snk_attached && val->intval == POWER_SUPPLY_TYPEC_PR_SOURCE)))
		is_pr_lock = true;

	smblib_dbg(chg, PR_MISC, "snk_attached = %d, src_attached = %d, is_pr_lock = %d\n",
			snk_attached, src_attached, is_pr_lock);
	cancel_delayed_work(&chg->pr_lock_clear_work);
	spin_lock(&chg->typec_pr_lock);
	if (!chg->pr_lock_in_progress && is_pr_lock) {
		smblib_dbg(chg, PR_MISC, "disable type-c interrupts for power role locking\n");
		smblib_typec_irq_config(chg, false);
		schedule_delayed_work(&chg->pr_lock_clear_work,
					msecs_to_jiffies(PR_LOCK_TIMEOUT_MS));
	} else if (chg->pr_lock_in_progress && !is_pr_lock) {
		smblib_dbg(chg, PR_MISC, "restore type-c interrupts after exit power role locking\n");
		smblib_typec_irq_config(chg, true);
	}

	chg->pr_lock_in_progress = is_pr_lock;
	spin_unlock(&chg->typec_pr_lock);

	switch (val->intval) {
	case POWER_SUPPLY_TYPEC_PR_NONE:
		power_role = TYPEC_DISABLE_CMD_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_DUAL:
		power_role = chg->typec_try_mode;
		break;
	case POWER_SUPPLY_TYPEC_PR_SINK:
		power_role = EN_SNK_ONLY_BIT;
		break;
	case POWER_SUPPLY_TYPEC_PR_SOURCE:
		power_role = EN_SRC_ONLY_BIT;
		break;
	default:
		smblib_err(chg, "power role %d not supported\n", val->intval);
		return -EINVAL;
	}

	rc = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
				TYPEC_POWER_ROLE_CMD_MASK | TYPEC_TRY_MODE_MASK,
				power_role);
	if (rc < 0) {
		smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
			power_role, rc);
		return rc;
	}

	chg->power_role = val->intval;
	return rc;
}

int smblib_set_prop_typec_select_rp(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc;

	if (!typec_in_src_mode(chg)) {
		smblib_err(chg, "Couldn't set curr src: not in SRC mode\n");
		return -EINVAL;
	}

	if (val->intval < TYPEC_SRC_RP_MAX_ELEMENTS) {
		rc = smblib_masked_write(chg, TYPE_C_CURRSRC_CFG_REG,
				TYPEC_SRC_RP_SEL_MASK,
				val->intval);
		if (rc < 0)
			smblib_err(chg, "Couldn't write to TYPE_C_CURRSRC_CFG rc=%d\n",
					rc);
		return rc;
	}

	return -EINVAL;
}

int smblib_set_prop_pd_voltage_min(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, min_uv;

	min_uv = min(val->intval, chg->voltage_max_uv);
	if (chg->voltage_min_uv == min_uv)
		return 0;

	rc = smblib_set_usb_pd_allowed_voltage(chg, min_uv,
					       chg->voltage_max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid min voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_min_uv = min_uv;
	power_supply_changed(chg->usb_main_psy);

	return rc;
}

int smblib_set_prop_pd_voltage_max(struct smb_charger *chg,
				    const union power_supply_propval *val)
{
	int rc, max_uv;

	max_uv = max(val->intval, chg->voltage_min_uv);
	if (chg->voltage_max_uv == max_uv)
		return 0;

	rc = smblib_set_usb_pd_fsw(chg, max_uv);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set FSW for voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	rc = smblib_set_usb_pd_allowed_voltage(chg, chg->voltage_min_uv,
					       max_uv);
	if (rc < 0) {
		smblib_err(chg, "invalid max voltage %duV rc=%d\n",
			val->intval, rc);
		return rc;
	}

	chg->voltage_max_uv = max_uv;
	power_supply_changed(chg->usb_main_psy);

	return rc;
}

int smblib_set_prop_pd_active(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	const struct apsd_result *apsd = smblib_get_apsd_result(chg);

	int rc = 0;
	int sec_charger, typec_mode;

	/*
	 * Ignore repetitive notification while PD is active, which
	 * is caused by hard reset.
	 */
	if (chg->pd_active && chg->pd_active == val->intval)
		return 0;

	chg->pd_active = val->intval;

	smblib_apsd_enable(chg, !chg->pd_active);
#ifdef CONFIG_PRODUCT_MOBA
	 if (!chg->pd) {
               chg->pd = devm_usbpd_get_by_phandle(chg->dev,
                               "qcom,usbpd-phandle");
               if (IS_ERR_OR_NULL(chg->pd))
                       pr_err("Failed to get pd handle %ld\n",
                                       PTR_ERR(chg->pd));
	}
#endif
	update_sw_icl_max(chg, apsd->pst);

	if (chg->pd_active) {
		vote(chg->limited_irq_disable_votable, CHARGER_TYPE_VOTER,
				false, 0);
		vote(chg->hdc_irq_disable_votable, CHARGER_TYPE_VOTER,
				false, 0);

		/*
		 * Enforce 100mA for PD until the real vote comes in later.
		 * It is guaranteed that pd_active is set prior to
		 * pd_current_max
		 */
		vote(chg->usb_icl_votable, PD_VOTER, true, USBIN_100MA);
		vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);

		/*
		 * For PPS, Charge Pump is preferred over parallel charger if
		 * present.
		 */
		if (chg->pd_active == POWER_SUPPLY_PD_PPS_ACTIVE
						&& chg->sec_cp_present) {
			rc = smblib_select_sec_charger(chg,
						POWER_SUPPLY_CHARGER_SEC_CP,
						POWER_SUPPLY_CP_PPS, false);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't enable secondary charger rc=%d\n",
					rc);
		}
	} else {
		vote(chg->usb_icl_votable, PD_VOTER, false, 0);
		vote(chg->limited_irq_disable_votable, CHARGER_TYPE_VOTER,
				true, 0);
		vote(chg->hdc_irq_disable_votable, CHARGER_TYPE_VOTER,
				true, 0);

		sec_charger = chg->sec_pl_present ?
						POWER_SUPPLY_CHARGER_SEC_PL :
						POWER_SUPPLY_CHARGER_SEC_NONE;
		rc = smblib_select_sec_charger(chg, sec_charger,
						POWER_SUPPLY_CP_NONE, false);
		if (rc < 0)
			dev_err(chg->dev,
				"Couldn't enable secondary charger rc=%d\n",
					rc);

		/* PD hard resets failed, proceed to detect QC2/3 */
		if (chg->ok_to_pd) {
			chg->ok_to_pd = false;
			smblib_hvdcp_detect_try_enable(chg, true);
		}
	}

	smblib_usb_pd_adapter_allowance_override(chg,
			!!chg->pd_active ? FORCE_5V : FORCE_NULL);
	smblib_update_usb_type(chg);

	if (chg->real_charger_type == POWER_SUPPLY_TYPE_USB &&
			!chg->ok_to_pd) {
		typec_mode = smblib_get_prop_typec_mode(chg);
		if (typec_rp_med_high(chg, typec_mode))
			vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	}

	power_supply_changed(chg->usb_psy);
	return rc;
}

int smblib_set_prop_ship_mode(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;

	smblib_dbg(chg, PR_MISC, "Set ship mode: %d!!\n", !!val->intval);

	rc = smblib_masked_write(chg, SHIP_MODE_REG, SHIP_MODE_EN_BIT,
			!!val->intval ? SHIP_MODE_EN_BIT : 0);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't %s ship mode, rc=%d\n",
				!!val->intval ? "enable" : "disable", rc);

	return rc;
}

int smblib_set_prop_pd_in_hard_reset(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc = 0;

	if (chg->pd_hard_reset == val->intval)
		return rc;

	chg->pd_hard_reset = val->intval;
	rc = smblib_masked_write(chg, TYPE_C_EXIT_STATE_CFG_REG,
			EXIT_SNK_BASED_ON_CC_BIT,
			(chg->pd_hard_reset) ? EXIT_SNK_BASED_ON_CC_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't set EXIT_SNK_BASED_ON_CC rc=%d\n",
				rc);

	return rc;
}

#define JEITA_SOFT			0
#define JEITA_HARD			1
static int smblib_update_jeita(struct smb_charger *chg, u32 *thresholds,
								int type)
{
	int rc;
	u16 temp, base;

	base = CHGR_JEITA_THRESHOLD_BASE_REG(type);

	temp = thresholds[1] & 0xFFFF;
	temp = ((temp & 0xFF00) >> 8) | ((temp & 0xFF) << 8);
	rc = smblib_batch_write(chg, base, (u8 *)&temp, 2);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't configure Jeita %s hot threshold rc=%d\n",
			(type == JEITA_SOFT) ? "Soft" : "Hard", rc);
		return rc;
	}

	temp = thresholds[0] & 0xFFFF;
	temp = ((temp & 0xFF00) >> 8) | ((temp & 0xFF) << 8);
	rc = smblib_batch_write(chg, base + 2, (u8 *)&temp, 2);
	if (rc < 0) {
		smblib_err(chg,
			"Couldn't configure Jeita %s cold threshold rc=%d\n",
			(type == JEITA_SOFT) ? "Soft" : "Hard", rc);
		return rc;
	}

	smblib_dbg(chg, PR_MISC, "%s Jeita threshold configured\n",
				(type == JEITA_SOFT) ? "Soft" : "Hard");

	return 0;
}

static int smblib_charge_inhibit_en(struct smb_charger *chg, bool enable)
{
	int rc;

	rc = smblib_masked_write(chg, CHGR_CFG2_REG,
					CHARGER_INHIBIT_BIT,
					enable ? CHARGER_INHIBIT_BIT : 0);
	return rc;
}

static int smblib_soft_jeita_arb_wa(struct smb_charger *chg)
{
	union power_supply_propval pval;
	int rc = 0;
	bool soft_jeita;

	rc = smblib_get_prop_batt_health(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get battery health rc=%d\n", rc);
		return rc;
	}

	/* Do nothing on entering hard JEITA condition */
	if (pval.intval == POWER_SUPPLY_HEALTH_COLD ||
		pval.intval == POWER_SUPPLY_HEALTH_HOT)
		return 0;

	if (chg->jeita_soft_fcc[0] < 0 || chg->jeita_soft_fcc[1] < 0 ||
		chg->jeita_soft_fv[0] < 0 || chg->jeita_soft_fv[1] < 0)
		return 0;

	soft_jeita = (pval.intval == POWER_SUPPLY_HEALTH_COOL) ||
			(pval.intval == POWER_SUPPLY_HEALTH_WARM);

	/* Do nothing on entering soft JEITA from hard JEITA */
	if (chg->jeita_arb_flag && soft_jeita)
		return 0;

	/* Do nothing, initial to health condition */
	if (!chg->jeita_arb_flag && !soft_jeita)
		return 0;

	/* Entering soft JEITA from normal state */
	if (!chg->jeita_arb_flag && soft_jeita) {
		vote(chg->chg_disable_votable, JEITA_ARB_VOTER, true, 0);

		rc = smblib_charge_inhibit_en(chg, true);
		if (rc < 0)
			smblib_err(chg, "Couldn't enable charge inhibit rc=%d\n",
					rc);

		rc = smblib_update_jeita(chg, chg->jeita_soft_hys_thlds,
					JEITA_SOFT);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't configure Jeita soft threshold rc=%d\n",
				rc);

		if (pval.intval == POWER_SUPPLY_HEALTH_COOL) {
			vote(chg->fcc_votable, JEITA_ARB_VOTER, true,
						chg->jeita_soft_fcc[0]);
			vote(chg->fv_votable, JEITA_ARB_VOTER, true,
						chg->jeita_soft_fv[0]);
		} else {
			vote(chg->fcc_votable, JEITA_ARB_VOTER, true,
						chg->jeita_soft_fcc[1]);
			vote(chg->fv_votable, JEITA_ARB_VOTER, true,
						chg->jeita_soft_fv[1]);
		}

		vote(chg->chg_disable_votable, JEITA_ARB_VOTER, false, 0);
		chg->jeita_arb_flag = true;
	} else if (chg->jeita_arb_flag && !soft_jeita) {
		/* Exit to health state from soft JEITA */

		vote(chg->chg_disable_votable, JEITA_ARB_VOTER, true, 0);

		rc = smblib_charge_inhibit_en(chg, false);
		if (rc < 0)
			smblib_err(chg, "Couldn't disable charge inhibit rc=%d\n",
					rc);

		rc = smblib_update_jeita(chg, chg->jeita_soft_thlds,
							JEITA_SOFT);
		if (rc < 0)
			smblib_err(chg, "Couldn't configure Jeita soft threshold rc=%d\n",
				rc);

		vote(chg->fcc_votable, JEITA_ARB_VOTER, false, 0);
		vote(chg->fv_votable, JEITA_ARB_VOTER, false, 0);
		vote(chg->chg_disable_votable, JEITA_ARB_VOTER, false, 0);
		chg->jeita_arb_flag = false;
	}

	smblib_dbg(chg, PR_MISC, "JEITA ARB status %d, soft JEITA status %d\n",
			chg->jeita_arb_flag, soft_jeita);
	return rc;
}

/************************
 * USB MAIN PSY GETTERS *
 ************************/
int smblib_get_prop_fcc_delta(struct smb_charger *chg,
				union power_supply_propval *val)
{
	int rc, jeita_cc_delta_ua = 0;

	if (chg->sw_jeita_enabled) {
		val->intval = 0;
		return 0;
	}

	rc = smblib_get_jeita_cc_delta(chg, &jeita_cc_delta_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get jeita cc delta rc=%d\n", rc);
		jeita_cc_delta_ua = 0;
	}

	val->intval = jeita_cc_delta_ua;
	return 0;
}

/************************
 * USB MAIN PSY SETTERS *
 ************************/
int smblib_get_charge_current(struct smb_charger *chg,
				int *total_current_ua)
{
	const struct apsd_result *apsd_result = smblib_get_apsd_result(chg);
	union power_supply_propval val = {0, };
	int rc = 0, typec_source_rd, current_ua;
	bool non_compliant;
	u8 stat;

	if (chg->pd_active) {
		*total_current_ua =
			get_client_vote_locked(chg->usb_icl_votable, PD_VOTER);
		return rc;
	}

	rc = smblib_read(chg, LEGACY_CABLE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_5 rc=%d\n", rc);
		return rc;
	}
	non_compliant = stat & TYPEC_NONCOMP_LEGACY_CABLE_STATUS_BIT;

	/* get settled ICL */
	rc = smblib_get_prop_input_current_settled(chg, &val);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get settled ICL rc=%d\n", rc);
		return rc;
	}

	typec_source_rd = smblib_get_prop_ufp_mode(chg);

	/* QC 2.0/3.0 adapter */
	if (apsd_result->bit & (QC_3P0_BIT | QC_2P0_BIT)) {
		*total_current_ua = HVDCP_CURRENT_UA;
		return 0;
	}

	if (non_compliant && !chg->typec_legacy_use_rp_icl) {
		switch (apsd_result->bit) {
		case CDP_CHARGER_BIT:
			current_ua = CDP_CURRENT_UA;
			break;
		case DCP_CHARGER_BIT:
		case OCP_CHARGER_BIT:
		case FLOAT_CHARGER_BIT:
			current_ua = DCP_CURRENT_UA;
			break;
		default:
			current_ua = 0;
			break;
		}

		*total_current_ua = max(current_ua, val.intval);
		return 0;
	}

	switch (typec_source_rd) {
	case POWER_SUPPLY_TYPEC_SOURCE_DEFAULT:
		switch (apsd_result->bit) {
		case CDP_CHARGER_BIT:
			current_ua = CDP_CURRENT_UA;
			break;
		case DCP_CHARGER_BIT:
		case OCP_CHARGER_BIT:
		case FLOAT_CHARGER_BIT:
			current_ua = chg->default_icl_ua;
			break;
		default:
			current_ua = 0;
			break;
		}
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_MEDIUM:
		current_ua = TYPEC_MEDIUM_CURRENT_UA;
		break;
	case POWER_SUPPLY_TYPEC_SOURCE_HIGH:
		current_ua = TYPEC_HIGH_CURRENT_UA;
		break;
	case POWER_SUPPLY_TYPEC_NON_COMPLIANT:
	case POWER_SUPPLY_TYPEC_NONE:
	default:
		current_ua = 0;
		break;
	}

	*total_current_ua = max(current_ua, val.intval);
	return 0;
}

#define IADP_OVERHEAT_UA	500000
int smblib_set_prop_thermal_overheat(struct smb_charger *chg,
						int therm_overheat)
{
	int icl_ua = 0;

	if (chg->thermal_overheat == !!therm_overheat)
		return 0;

	/* Configure ICL to 500mA in case system health is Overheat */
	if (therm_overheat)
		icl_ua = IADP_OVERHEAT_UA;

	if (!chg->cp_disable_votable)
		chg->cp_disable_votable = find_votable("CP_DISABLE");

	if (chg->cp_disable_votable) {
		vote(chg->cp_disable_votable, OVERHEAT_LIMIT_VOTER,
							therm_overheat, 0);
		vote(chg->usb_icl_votable, OVERHEAT_LIMIT_VOTER,
							therm_overheat, icl_ua);
	}

	chg->thermal_overheat = !!therm_overheat;
	return 0;
}

/**********************
 * INTERRUPT HANDLERS *
 **********************/

irqreturn_t default_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	if (strcmp(irq_data->name, "input-current-limiting"))
		smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	return IRQ_HANDLED;
}

irqreturn_t smb_en_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc, input_present;

	if (!chg->cp_disable_votable) {
		chg->cp_disable_votable = find_votable("CP_DISABLE");
		if (!chg->cp_disable_votable)
			return IRQ_HANDLED;
	}

	if (chg->pd_hard_reset) {
		vote(chg->cp_disable_votable, BOOST_BACK_VOTER, true, 0);
		return IRQ_HANDLED;
	}

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0) {
		pr_err("Couldn't get usb presence status rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	if (input_present) {
		/*
		 * Add some delay to enable SMB1390 switcher after SMB_EN
		 * pin goes high
		 */
		usleep_range(1000, 1100);
		vote(chg->cp_disable_votable, BOOST_BACK_VOTER, false, 0);
	}

	return IRQ_HANDLED;
}

irqreturn_t sdam_sts_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	mutex_lock(&chg->irq_status_lock);
	chg->irq_status |= PULSE_SKIP_IRQ_BIT;
	mutex_unlock(&chg->irq_status_lock);

	power_supply_changed(chg->usb_main_psy);

	return IRQ_HANDLED;
}

#define CHG_TERM_WA_ENTRY_DELAY_MS		300000		/* 5 min */
#define CHG_TERM_WA_EXIT_DELAY_MS		60000		/* 1 min */
static void smblib_eval_chg_termination(struct smb_charger *chg, u8 batt_status)
{
	union power_supply_propval pval = {0, };
	int rc = 0;

	rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_REAL_CAPACITY, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read SOC value, rc=%d\n", rc);
		return;
	}

	/*
	 * Post charge termination, switch to BSM mode triggers the risk of
	 * over charging as BATFET opening may take some time post the necessity
	 * of staying in supplemental mode, leading to unintended charging of
	 * battery. Trigger the charge termination WA once charging is completed
	 * to prevent overcharing.
	 */
	if ((batt_status == TERMINATE_CHARGE) && (pval.intval == 100)) {
		chg->cc_soc_ref = 0;
		chg->last_cc_soc = 0;
		chg->term_vbat_uv = 0;
		alarm_start_relative(&chg->chg_termination_alarm,
				ms_to_ktime(CHG_TERM_WA_ENTRY_DELAY_MS));
	} else if (pval.intval < 100) {
		/*
		 * Reset CC_SOC reference value for charge termination WA once
		 * we exit the TERMINATE_CHARGE state and soc drops below 100%
		 */
		chg->cc_soc_ref = 0;
		chg->last_cc_soc = 0;
		chg->term_vbat_uv = 0;
	}
}

irqreturn_t chg_state_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_read(chg, BATTERY_CHARGER_STATUS_1_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read BATTERY_CHARGER_STATUS_1 rc=%d\n",
				rc);
		return IRQ_HANDLED;
	}

	stat = stat & BATTERY_CHARGER_STATUS_MASK;

	if (chg->wa_flags & CHG_TERMINATION_WA)
		smblib_eval_chg_termination(chg, stat);

	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

irqreturn_t batt_temp_changed_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->jeita_configured != JEITA_CFG_COMPLETE)
		return IRQ_HANDLED;

	rc = smblib_soft_jeita_arb_wa(chg);
	if (rc < 0) {
		smblib_err(chg, "Couldn't fix soft jeita arb rc=%d\n",
				rc);
		return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

irqreturn_t batt_psy_changed_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
	power_supply_changed(chg->batt_psy);
	return IRQ_HANDLED;
}

#define AICL_STEP_MV		200
#define MAX_AICL_THRESHOLD_MV	4800
irqreturn_t usbin_uv_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct storm_watch *wdata;
	const struct apsd_result *apsd = smblib_get_apsd_result(chg);
	int rc;
	u8 stat = 0, max_pulses = 0;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if ((chg->wa_flags & WEAK_ADAPTER_WA)
			&& is_storming(&irq_data->storm_data)) {

		if (chg->aicl_max_reached) {
			smblib_dbg(chg, PR_MISC,
					"USBIN_UV storm at max AICL threshold\n");
			return IRQ_HANDLED;
		}

		smblib_dbg(chg, PR_MISC, "USBIN_UV storm at threshold %d\n",
				chg->aicl_5v_threshold_mv);

		/* suspend USBIN before updating AICL threshold */
		vote(chg->usb_icl_votable, AICL_THRESHOLD_VOTER, true, 0);

		/* delay for VASHDN deglitch */
		msleep(20);

		if (chg->aicl_5v_threshold_mv > MAX_AICL_THRESHOLD_MV) {
			/* reached max AICL threshold */
			chg->aicl_max_reached = true;
			goto unsuspend_input;
		}

		/* Increase AICL threshold by 200mV */
		rc = smblib_set_charge_param(chg, &chg->param.aicl_5v_threshold,
				chg->aicl_5v_threshold_mv + AICL_STEP_MV);
		if (rc < 0)
			dev_err(chg->dev,
				"Error in setting AICL threshold rc=%d\n", rc);
		else
			chg->aicl_5v_threshold_mv += AICL_STEP_MV;

		rc = smblib_set_charge_param(chg,
				&chg->param.aicl_cont_threshold,
				chg->aicl_cont_threshold_mv + AICL_STEP_MV);
		if (rc < 0)
			dev_err(chg->dev,
				"Error in setting AICL threshold rc=%d\n", rc);
		else
			chg->aicl_cont_threshold_mv += AICL_STEP_MV;

unsuspend_input:
		/* Force torch in boost mode to ensure it works with low ICL */
		if (chg->chg_param.smb_version == PMI632_SUBTYPE)
			schgm_flash_torch_priority(chg, TORCH_BOOST_MODE);

		if (chg->aicl_max_reached) {
			smblib_dbg(chg, PR_MISC,
				"Reached max AICL threshold resctricting ICL to 100mA\n");
			vote(chg->usb_icl_votable, AICL_THRESHOLD_VOTER,
					true, USBIN_100MA);
			smblib_run_aicl(chg, RESTART_AICL);
		} else {
			smblib_run_aicl(chg, RESTART_AICL);
			vote(chg->usb_icl_votable, AICL_THRESHOLD_VOTER,
					false, 0);
		}

		wdata = &chg->irq_info[USBIN_UV_IRQ].irq_data->storm_data;
		reset_storm_count(wdata);
	}

	if (!chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data)
		return IRQ_HANDLED;

	wdata = &chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data->storm_data;
	reset_storm_count(wdata);

	/* Workaround for non-QC2.0-compliant chargers follows */
	if (!chg->qc2_unsupported_voltage &&
			apsd->pst == POWER_SUPPLY_TYPE_USB_HVDCP) {
		rc = smblib_read(chg, QC_CHANGE_STATUS_REG, &stat);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't read CHANGE_STATUS_REG rc=%d\n", rc);

		if (stat & QC_5V_BIT)
			return IRQ_HANDLED;

		rc = smblib_read(chg, HVDCP_PULSE_COUNT_MAX_REG, &max_pulses);
		if (rc < 0)
			smblib_err(chg,
				"Couldn't read QC2 max pulses rc=%d\n", rc);

		chg->qc2_max_pulses = (max_pulses &
				HVDCP_PULSE_COUNT_MAX_QC2_MASK);

		if (stat & QC_12V_BIT) {
			chg->qc2_unsupported_voltage = QC2_NON_COMPLIANT_12V;
			rc = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
					HVDCP_PULSE_COUNT_MAX_QC2_MASK,
					HVDCP_PULSE_COUNT_MAX_QC2_9V);
			if (rc < 0)
				smblib_err(chg, "Couldn't force max pulses to 9V rc=%d\n",
						rc);

		} else if (stat & QC_9V_BIT) {
			chg->qc2_unsupported_voltage = QC2_NON_COMPLIANT_9V;
			rc = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
					HVDCP_PULSE_COUNT_MAX_QC2_MASK,
					HVDCP_PULSE_COUNT_MAX_QC2_5V);
			if (rc < 0)
				smblib_err(chg, "Couldn't force max pulses to 5V rc=%d\n",
						rc);

		}

		rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
				SUSPEND_ON_COLLAPSE_USBIN_BIT,
				0);
		if (rc < 0)
			smblib_err(chg, "Couldn't turn off SUSPEND_ON_COLLAPSE_USBIN_BIT rc=%d\n",
					rc);

		smblib_rerun_apsd(chg);
	}

	return IRQ_HANDLED;
}

#define USB_WEAK_INPUT_UA	1400000
#define ICL_CHANGE_DELAY_MS	1000
irqreturn_t icl_change_irq_handler(int irq, void *data)
{
	u8 stat;
	int rc, settled_ua, delay = ICL_CHANGE_DELAY_MS;
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	if (chg->mode == PARALLEL_MASTER) {
		/*
		 * Ignore if change in ICL is due to DIE temp mitigation.
		 * This is to prevent any further ICL split.
		 */
		if (chg->hw_die_temp_mitigation) {
			rc = smblib_read(chg, DIE_TEMP_STATUS_REG, &stat);
			if (rc < 0) {
				smblib_err(chg,
					"Couldn't read DIE_TEMP rc=%d\n", rc);
				return IRQ_HANDLED;
			}
			if (stat & (DIE_TEMP_UB_BIT | DIE_TEMP_LB_BIT)) {
				smblib_dbg(chg, PR_PARALLEL,
					"skip ICL change DIE_TEMP %x\n", stat);
				return IRQ_HANDLED;
			}
		}

		rc = smblib_read(chg, AICL_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read AICL_STATUS rc=%d\n",
					rc);
			return IRQ_HANDLED;
		}

		rc = smblib_get_charge_param(chg, &chg->param.icl_stat,
					&settled_ua);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
			return IRQ_HANDLED;
		}

		/* If AICL settled then schedule work now */
		if (settled_ua == get_effective_result(chg->usb_icl_votable))
			delay = 0;

		cancel_delayed_work_sync(&chg->icl_change_work);
		schedule_delayed_work(&chg->icl_change_work,
						msecs_to_jiffies(delay));
	}

	return IRQ_HANDLED;
}

static void smblib_micro_usb_plugin(struct smb_charger *chg, bool vbus_rising)
{
	if (!vbus_rising) {
		smblib_update_usb_type(chg);
		smblib_notify_device_mode(chg, false);
		smblib_uusb_removal(chg);
	}
}

void smblib_usb_plugin_hard_reset_locked(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	bool vbus_rising;
	struct smb_irq_data *data;
	struct storm_watch *wdata;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	if (vbus_rising) {
		/* Remove FCC_STEPPER 1.5A init vote to allow FCC ramp up */
		if (chg->fcc_stepper_enable)
			vote(chg->fcc_votable, FCC_STEPPER_VOTER, false, 0);
	} else {
		if (chg->wa_flags & BOOST_BACK_WA) {
			data = chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data;
			if (data) {
				wdata = &data->storm_data;
				update_storm_count(wdata,
						WEAK_CHG_STORM_COUNT);
				vote(chg->usb_icl_votable, BOOST_BACK_VOTER,
						false, 0);
				vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
						false, 0);
			}
		}

		/* Force 1500mA FCC on USB removal if fcc stepper is enabled */
		if (chg->fcc_stepper_enable)
			vote(chg->fcc_votable, FCC_STEPPER_VOTER,
							true, 1500000);
	}

	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: usbin-plugin %s\n",
					vbus_rising ? "attached" : "detached");
}

#define PL_DELAY_MS	30000
void smblib_usb_plugin_locked(struct smb_charger *chg)
{
	int rc;
	u8 stat;
	bool vbus_rising;
	struct smb_irq_data *data;
	struct storm_watch *wdata;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return;
	}

	vbus_rising = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);
	smblib_set_opt_switcher_freq(chg, vbus_rising ? chg->chg_freq.freq_5V :
						chg->chg_freq.freq_removal);

#ifdef CONFIG_PRODUCT_MOBA
	first_port_online = vbus_rising;

if(usb1_otg_en==1&&vbus_rising==1){
	otg1_charge2_en=1;
	pr_info("smb5:smblib_usb_plugin_locked,otg1_charge2_enable\n");
}else{
	otg1_charge2_en=0;
	pr_info("smb5:smblib_usb_plugin_locked,otg1_charge2_disable\n");
}
#endif

	if (vbus_rising) {
		cancel_delayed_work_sync(&chg->pr_swap_detach_work);
		vote(chg->awake_votable, DETACH_DETECT_VOTER, false, 0);
		rc = smblib_request_dpdm(chg, true);
		if (rc < 0)
			smblib_err(chg, "Couldn't to enable DPDM rc=%d\n", rc);

		/* Enable SW Thermal regulation */
		rc = smblib_set_sw_thermal_regulation(chg, true);
		if (rc < 0)
			smblib_err(chg, "Couldn't start SW thermal regulation WA, rc=%d\n",
				rc);

		/* Remove FCC_STEPPER 1.5A init vote to allow FCC ramp up */
		if (chg->fcc_stepper_enable)
			vote(chg->fcc_votable, FCC_STEPPER_VOTER, false, 0);

		/* Schedule work to enable parallel charger */
		vote(chg->awake_votable, PL_DELAY_VOTER, true, 0);
		schedule_delayed_work(&chg->pl_enable_work,
					msecs_to_jiffies(PL_DELAY_MS));
	} else {
		/* Disable SW Thermal Regulation */
		rc = smblib_set_sw_thermal_regulation(chg, false);
		if (rc < 0)
			smblib_err(chg, "Couldn't stop SW thermal regulation WA, rc=%d\n",
				rc);

		if (chg->wa_flags & BOOST_BACK_WA) {
			data = chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data;
			if (data) {
				wdata = &data->storm_data;
				update_storm_count(wdata,
						WEAK_CHG_STORM_COUNT);
				vote(chg->usb_icl_votable, BOOST_BACK_VOTER,
						false, 0);
				vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
						false, 0);
			}
		}

		/* Force 1500mA FCC on removal if fcc stepper is enabled */
		if (chg->fcc_stepper_enable)
			vote(chg->fcc_votable, FCC_STEPPER_VOTER,
							true, 1500000);

		if (chg->wa_flags & WEAK_ADAPTER_WA) {
			chg->aicl_5v_threshold_mv =
					chg->default_aicl_5v_threshold_mv;
			chg->aicl_cont_threshold_mv =
					chg->default_aicl_cont_threshold_mv;

			smblib_set_charge_param(chg,
					&chg->param.aicl_5v_threshold,
					chg->aicl_5v_threshold_mv);
			smblib_set_charge_param(chg,
					&chg->param.aicl_cont_threshold,
					chg->aicl_cont_threshold_mv);
			chg->aicl_max_reached = false;

			if (chg->chg_param.smb_version == PMI632_SUBTYPE)
				schgm_flash_torch_priority(chg,
						TORCH_BUCK_MODE);

			data = chg->irq_info[USBIN_UV_IRQ].irq_data;
			if (data) {
				wdata = &data->storm_data;
				reset_storm_count(wdata);
			}
			vote(chg->usb_icl_votable, AICL_THRESHOLD_VOTER,
					false, 0);
		}

		rc = smblib_request_dpdm(chg, false);
		if (rc < 0)
			smblib_err(chg, "Couldn't disable DPDM rc=%d\n", rc);

		smblib_update_usb_type(chg);
	}

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		smblib_micro_usb_plugin(chg, vbus_rising);

	vote(chg->temp_change_irq_disable_votable, DEFAULT_VOTER,
						!vbus_rising, 0);

	power_supply_changed(chg->usb_psy);
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: usbin-plugin %s\n",
					vbus_rising ? "attached" : "detached");
}

irqreturn_t usb_plugin_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	if (chg->pd_hard_reset)
		smblib_usb_plugin_hard_reset_locked(chg);
	else
		smblib_usb_plugin_locked(chg);

	return IRQ_HANDLED;
}

static void smblib_handle_slow_plugin_timeout(struct smb_charger *chg,
					      bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: slow-plugin-timeout %s\n",
		   rising ? "rising" : "falling");
}

static void smblib_handle_sdp_enumeration_done(struct smb_charger *chg,
					       bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: sdp-enumeration-done %s\n",
		   rising ? "rising" : "falling");
}

#define APSD_EXTENDED_TIMEOUT_MS	400
/* triggers when HVDCP 3.0 authentication has finished */
static void smblib_handle_hvdcp_3p0_auth_done(struct smb_charger *chg,
					      bool rising)
{
	const struct apsd_result *apsd_result;
	int rc;

	if (!rising)
		return;

	if (chg->mode == PARALLEL_MASTER)
		vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, true, 0);

	/* the APSD done handler will set the USB supply type */
	apsd_result = smblib_get_apsd_result(chg);

	if (apsd_result->bit & QC_3P0_BIT) {
		/* for QC3, switch to CP if present */
		if (chg->sec_cp_present) {
			rc = smblib_select_sec_charger(chg,
				POWER_SUPPLY_CHARGER_SEC_CP,
				POWER_SUPPLY_CP_HVDCP3, false);
			if (rc < 0)
				dev_err(chg->dev,
				"Couldn't enable secondary chargers  rc=%d\n",
					rc);
		}

		/* QC3.5 detection timeout */
		if (!chg->apsd_ext_timeout &&
				!timer_pending(&chg->apsd_timer)) {
			smblib_dbg(chg, PR_MISC,
				"APSD Extented timer started at %lld\n",
				jiffies_to_msecs(jiffies));

			mod_timer(&chg->apsd_timer,
				msecs_to_jiffies(APSD_EXTENDED_TIMEOUT_MS)
				+ jiffies);
		}
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-3p0-auth-done rising; %s detected\n",
		   apsd_result->name);
}

static void smblib_handle_hvdcp_check_timeout(struct smb_charger *chg,
					      bool rising, bool qc_charger)
{
	if (rising) {

		if (qc_charger) {
			/* enable HDC and ICL irq for QC2/3 charger */
			vote(chg->limited_irq_disable_votable,
					CHARGER_TYPE_VOTER, false, 0);
			vote(chg->hdc_irq_disable_votable,
					CHARGER_TYPE_VOTER, false, 0);
			vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
				HVDCP_CURRENT_UA);
		} else {
			/* A plain DCP, enforce DCP ICL if specified */
			vote(chg->usb_icl_votable, DCP_VOTER,
				chg->dcp_icl_ua != -EINVAL, chg->dcp_icl_ua);
		}
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s %s\n", __func__,
		   rising ? "rising" : "falling");
}

/* triggers when HVDCP is detected */
static void smblib_handle_hvdcp_detect_done(struct smb_charger *chg,
					    bool rising)
{
	smblib_dbg(chg, PR_INTERRUPT, "IRQ: hvdcp-detect-done %s\n",
		   rising ? "rising" : "falling");
}

static void update_sw_icl_max(struct smb_charger *chg, int pst)
{
	int typec_mode;
	int rp_ua;

	/* while PD is active it should have complete ICL control */
	if (chg->pd_active)
		return;

	if (chg->typec_mode == POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER) {
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, 500000);
		return;
	}

	/*
	 * HVDCP 2/3, handled separately
	 */
	if (pst == POWER_SUPPLY_TYPE_USB_HVDCP
			|| pst == POWER_SUPPLY_TYPE_USB_HVDCP_3)
		return;

	/* TypeC rp med or high, use rp value */
	typec_mode = smblib_get_prop_typec_mode(chg);
	if (typec_rp_med_high(chg, typec_mode)) {
		rp_ua = get_rp_based_dcp_current(chg, typec_mode);
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, rp_ua);
		return;
	}

	/* rp-std or legacy, USB BC 1.2 */
	switch (pst) {
	case POWER_SUPPLY_TYPE_USB:
		/*
		 * USB_PSY will vote to increase the current to 500/900mA once
		 * enumeration is done.
		 */
		if (!is_client_vote_enabled(chg->usb_icl_votable,
						USB_PSY_VOTER)) {
			/* if flash is active force 500mA */
			vote(chg->usb_icl_votable, USB_PSY_VOTER, true,
					is_flash_active(chg) ?
					SDP_CURRENT_UA : SDP_100_MA);
		}
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					CDP_CURRENT_UA);
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		rp_ua = get_rp_based_dcp_current(chg, typec_mode);
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, rp_ua);
		break;
	case POWER_SUPPLY_TYPE_USB_FLOAT:
		/*
		 * limit ICL to 100mA, the USB driver will enumerate to check
		 * if this is a SDP and appropriately set the current
		 */
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					SDP_100_MA);
		break;
	case POWER_SUPPLY_TYPE_UNKNOWN:
	default:
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
					SDP_100_MA);
		break;
	}
}

static void smblib_handle_apsd_done(struct smb_charger *chg, bool rising)
{
	const struct apsd_result *apsd_result;

	if (!rising)
		return;

	apsd_result = smblib_update_usb_type(chg);

	update_sw_icl_max(chg, apsd_result->pst);

	switch (apsd_result->bit) {
	case SDP_CHARGER_BIT:
	case CDP_CHARGER_BIT:
	case FLOAT_CHARGER_BIT:
		if (chg->use_extcon)
			smblib_notify_device_mode(chg, true);
		break;
	case OCP_CHARGER_BIT:
	case DCP_CHARGER_BIT:
		break;
	default:
		break;
	}

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: apsd-done rising; %s detected\n",
		   apsd_result->name);
}

irqreturn_t usb_source_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc = 0;
	u8 stat;

	/* PD session is ongoing, ignore BC1.2 and QC detection */
	if (chg->pd_active)
		return IRQ_HANDLED;

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_INTERRUPT, "APSD_STATUS = 0x%02x\n", stat);

	if ((chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB)
		&& (stat & APSD_DTC_STATUS_DONE_BIT)
		&& !chg->uusb_apsd_rerun_done) {
		/*
		 * Force re-run APSD to handle slow insertion related
		 * charger-mis-detection.
		 */
		chg->uusb_apsd_rerun_done = true;
		smblib_rerun_apsd_if_required(chg);
		return IRQ_HANDLED;
	}

	smblib_handle_apsd_done(chg,
		(bool)(stat & APSD_DTC_STATUS_DONE_BIT));

	smblib_handle_hvdcp_detect_done(chg,
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_check_timeout(chg,
		(bool)(stat & HVDCP_CHECK_TIMEOUT_BIT),
		(bool)(stat & QC_CHARGER_BIT));

	smblib_handle_hvdcp_3p0_auth_done(chg,
		(bool)(stat & QC_AUTH_DONE_STATUS_BIT));

	smblib_handle_sdp_enumeration_done(chg,
		(bool)(stat & ENUMERATION_DONE_BIT));

	smblib_handle_slow_plugin_timeout(chg,
		(bool)(stat & SLOW_PLUGIN_TIMEOUT_BIT));

	smblib_hvdcp_adaptive_voltage_change(chg);

	power_supply_changed(chg->usb_psy);

	rc = smblib_read(chg, APSD_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read APSD_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}
	smblib_dbg(chg, PR_INTERRUPT, "APSD_STATUS = 0x%02x\n", stat);

	return IRQ_HANDLED;
}

enum alarmtimer_restart smblib_lpd_recheck_timer(struct alarm *alarm,
						ktime_t time)
{
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(alarm, struct smb_charger,
							lpd_recheck_timer);
	int rc;

	if (chg->lpd_reason == LPD_MOISTURE_DETECTED) {
		pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		rc = smblib_set_prop_typec_power_role(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
				pval.intval, rc);
			return ALARMTIMER_NORESTART;
		}
		chg->moisture_present = false;
		power_supply_changed(chg->usb_psy);
	} else {
		rc = smblib_masked_write(chg, TYPE_C_INTERRUPT_EN_CFG_2_REG,
					TYPEC_WATER_DETECTION_INT_EN_BIT,
					TYPEC_WATER_DETECTION_INT_EN_BIT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set TYPE_C_INTERRUPT_EN_CFG_2_REG rc=%d\n",
					rc);
			return ALARMTIMER_NORESTART;
		}
	}

	chg->lpd_stage = LPD_STAGE_NONE;
	chg->lpd_reason = LPD_NONE;

	return ALARMTIMER_NORESTART;
}

#define RSBU_K_300K_UV	3000000
static bool smblib_src_lpd(struct smb_charger *chg)
{
	union power_supply_propval pval;
	bool lpd_flag = false;
	u8 stat;
	int rc;

	if (chg->lpd_disabled)
		return false;

	rc = smblib_read(chg, TYPE_C_SRC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_SRC_STATUS_REG rc=%d\n",
				rc);
		return false;
	}

	switch (stat & DETECTED_SNK_TYPE_MASK) {
	case SRC_DEBUG_ACCESS_BIT:
		if (smblib_rsbux_low(chg, RSBU_K_300K_UV))
			lpd_flag = true;
		break;
	case SRC_RD_RA_VCONN_BIT:
	case SRC_RD_OPEN_BIT:
	case AUDIO_ACCESS_RA_RA_BIT:
	default:
		break;
	}

	if (lpd_flag) {
		chg->lpd_stage = LPD_STAGE_COMMIT;
		pval.intval = POWER_SUPPLY_TYPEC_PR_SINK;
		rc = smblib_set_prop_typec_power_role(chg, &pval);
		if (rc < 0)
			smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
				pval.intval, rc);
		chg->lpd_reason = LPD_MOISTURE_DETECTED;
		chg->moisture_present =  true;
		vote(chg->usb_icl_votable, LPD_VOTER, true, 0);
		alarm_start_relative(&chg->lpd_recheck_timer,
						ms_to_ktime(60000));
		power_supply_changed(chg->usb_psy);
	} else {
		chg->lpd_reason = LPD_NONE;
		chg->typec_mode = smblib_get_prop_typec_mode(chg);
	}

	return lpd_flag;
}

static void typec_src_fault_condition_cfg(struct smb_charger *chg, bool src)
{
	int rc;
	u8 mask = USBIN_MID_COMP_FAULT_EN_BIT | USBIN_COLLAPSE_FAULT_EN_BIT;

	rc = smblib_masked_write(chg, OTG_FAULT_CONDITION_CFG_REG, mask,
					src ? 0 : mask);
	if (rc < 0)
		smblib_err(chg, "Couldn't write OTG_FAULT_CONDITION_CFG_REG rc=%d\n",
			rc);
}

static void typec_sink_insertion(struct smb_charger *chg)
{
	int rc;

	typec_src_fault_condition_cfg(chg, true);
	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher,
					chg->chg_freq.freq_above_otg_threshold);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_boost rc=%d\n", rc);

	if (chg->use_extcon) {
		smblib_notify_usb_host(chg, true);
		chg->otg_present = true;
	}

	if (!chg->pr_swap_in_progress)
		chg->ok_to_pd = (!(chg->pd_disabled) || chg->early_usb_attach)
					&& !chg->pd_not_supported;
}

static void typec_src_insertion(struct smb_charger *chg)
{
	int rc = 0;
	u8 stat;

	if (chg->pr_swap_in_progress) {
		vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, false, 0);
		return;
	}

	rc = smblib_read(chg, LEGACY_CABLE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATE_MACHINE_STATUS_REG rc=%d\n",
			rc);
		return;
	}

	chg->typec_legacy = stat & TYPEC_LEGACY_CABLE_STATUS_BIT;
	chg->ok_to_pd = (!(chg->typec_legacy || chg->pd_disabled)
			|| chg->early_usb_attach) && !chg->pd_not_supported;

	/* allow apsd proceed to detect QC2/3 */
	if (!chg->ok_to_pd)
		smblib_hvdcp_detect_try_enable(chg, true);
}

static void typec_ra_ra_insertion(struct smb_charger *chg)
{
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, 500000);
	vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	chg->ok_to_pd = false;
	smblib_hvdcp_detect_enable(chg, true);
}

static void typec_sink_removal(struct smb_charger *chg)
{
	int rc;
#ifdef CONFIG_PRODUCT_MOBA
       usb1_otg_en=0;
       otg1_charge2_en=0;
	pr_info("smb5-lib:typec_sink_removal,otg1_charge2_disable\n");
#endif
	typec_src_fault_condition_cfg(chg, false);
	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher,
					chg->chg_freq.freq_removal);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_removal rc=%d\n", rc);

	if (chg->use_extcon) {
		if (chg->otg_present)
			smblib_notify_usb_host(chg, false);
		chg->otg_present = false;
	}
#ifdef CONFIG_PRODUCT_MOBA
	rc=smblib_masked_write(chg, TYPE_C_EXIT_STATE_CFG_REG,
			BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT,0);
	if (rc < 0)
		pr_err("Couldn't set exit state cfg ret=%d\n", rc);
	rc = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
				EN_TRY_SNK_BIT,0x10);
	if (rc < 0)
		pr_err("Couldn't configure TYPE_C_MODE_CFG_REG ret=%d\n",rc);
#endif
}

static void typec_src_removal(struct smb_charger *chg)
{
	int rc;
	struct smb_irq_data *data;
	struct storm_watch *wdata;
	int sec_charger;
	u8 val[2] = {0};

	sec_charger = chg->sec_pl_present ? POWER_SUPPLY_CHARGER_SEC_PL :
				POWER_SUPPLY_CHARGER_SEC_NONE;

	rc = smblib_select_sec_charger(chg, sec_charger, POWER_SUPPLY_CP_NONE,
					false);
	if (rc < 0)
		dev_err(chg->dev,
			"Couldn't disable secondary charger rc=%d\n", rc);

	chg->qc3p5_detected = false;
	typec_src_fault_condition_cfg(chg, false);
	smblib_hvdcp_detect_try_enable(chg, false);
	smblib_update_usb_type(chg);

	if (chg->wa_flags & BOOST_BACK_WA) {
		data = chg->irq_info[SWITCHER_POWER_OK_IRQ].irq_data;
		if (data) {
			wdata = &data->storm_data;
			update_storm_count(wdata, WEAK_CHG_STORM_COUNT);
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
			vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
					false, 0);
		}
	}

	cancel_delayed_work_sync(&chg->pl_enable_work);

	/* reset input current limit voters */
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true,
			is_flash_active(chg) ? SDP_CURRENT_UA : SDP_100_MA);
	vote(chg->usb_icl_votable, PD_VOTER, false, 0);
	vote(chg->usb_icl_votable, USB_PSY_VOTER, false, 0);
	vote(chg->usb_icl_votable, DCP_VOTER, false, 0);
	vote(chg->usb_icl_votable, SW_QC3_VOTER, false, 0);
	vote(chg->usb_icl_votable, CTM_VOTER, false, 0);
	vote(chg->usb_icl_votable, HVDCP2_ICL_VOTER, false, 0);
	vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
	vote(chg->usb_icl_votable, THERMAL_THROTTLE_VOTER, false, 0);
	vote(chg->usb_icl_votable, LPD_VOTER, false, 0);

	/* reset usb irq voters */
	vote(chg->limited_irq_disable_votable, CHARGER_TYPE_VOTER,
			true, 0);
	vote(chg->hdc_irq_disable_votable, CHARGER_TYPE_VOTER, true, 0);
	vote(chg->hdc_irq_disable_votable, HDC_IRQ_VOTER, false, 0);

	/* reset parallel voters */
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);
	vote(chg->pl_disable_votable, PL_FCC_LOW_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_I_VOTER, false, 0);
	vote(chg->pl_enable_votable_indirect, USBIN_V_VOTER, false, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);

	/* Remove SW thermal regulation WA votes */
	vote(chg->usb_icl_votable, SW_THERM_REGULATION_VOTER, false, 0);
	vote(chg->pl_disable_votable, SW_THERM_REGULATION_VOTER, false, 0);
	vote(chg->dc_suspend_votable, SW_THERM_REGULATION_VOTER, false, 0);
	if (chg->cp_disable_votable)
		vote(chg->cp_disable_votable, SW_THERM_REGULATION_VOTER,
								false, 0);

	/* reset USBOV votes and cancel work */
	cancel_delayed_work_sync(&chg->usbov_dbc_work);
	vote(chg->awake_votable, USBOV_DBC_VOTER, false, 0);
	chg->dbc_usbov = false;

	chg->pulse_cnt = 0;
	chg->usb_icl_delta_ua = 0;
	chg->voltage_min_uv = MICRO_5V;
	chg->voltage_max_uv = MICRO_5V;
	chg->usbin_forced_max_uv = 0;
	chg->chg_param.forced_main_fcc = 0;

	/* Reset all CC mode votes */
	vote(chg->fcc_main_votable, MAIN_FCC_VOTER, false, 0);
	chg->adapter_cc_mode = 0;
	chg->thermal_overheat = 0;
	vote_override(chg->fcc_votable, CC_MODE_VOTER, false, 0);
	vote_override(chg->usb_icl_votable, CC_MODE_VOTER, false, 0);
	vote(chg->cp_disable_votable, OVERHEAT_LIMIT_VOTER, false, 0);
	vote(chg->usb_icl_votable, OVERHEAT_LIMIT_VOTER, false, 0);

	/* write back the default FLOAT charger configuration */
	rc = smblib_masked_write(chg, USBIN_OPTIONS_2_CFG_REG,
				(u8)FLOAT_OPTIONS_MASK, chg->float_cfg);
	if (rc < 0)
		smblib_err(chg, "Couldn't write float charger options rc=%d\n",
			rc);

	if (chg->sdam_base) {
		rc = smblib_write(chg,
			chg->sdam_base + SDAM_QC_DET_STATUS_REG, 0);
		if (rc < 0)
			pr_err("Couldn't clear SDAM QC status rc=%d\n", rc);

		rc = smblib_batch_write(chg,
			chg->sdam_base + SDAM_QC_ADC_LSB_REG, val, 2);
		if (rc < 0)
			pr_err("Couldn't clear SDAM ADC status rc=%d\n", rc);
	}

	if (!chg->pr_swap_in_progress) {
		rc = smblib_usb_pd_adapter_allowance_override(chg, FORCE_NULL);
		if (rc < 0)
			smblib_err(chg, "Couldn't set FORCE_NULL rc=%d\n", rc);

		rc = smblib_set_charge_param(chg,
				&chg->param.aicl_cont_threshold,
				chg->default_aicl_cont_threshold_mv);
		if (rc < 0)
			smblib_err(chg, "Couldn't restore aicl_cont_threshold, rc=%d",
					rc);
	}
	/*
	 * if non-compliant charger caused UV, restore original max pulses
	 * and turn SUSPEND_ON_COLLAPSE_USBIN_BIT back on.
	 */
	if (chg->qc2_unsupported_voltage) {
		rc = smblib_masked_write(chg, HVDCP_PULSE_COUNT_MAX_REG,
				HVDCP_PULSE_COUNT_MAX_QC2_MASK,
				chg->qc2_max_pulses);
		if (rc < 0)
			smblib_err(chg, "Couldn't restore max pulses rc=%d\n",
					rc);

		rc = smblib_masked_write(chg, USBIN_AICL_OPTIONS_CFG_REG,
				SUSPEND_ON_COLLAPSE_USBIN_BIT,
				SUSPEND_ON_COLLAPSE_USBIN_BIT);
		if (rc < 0)
			smblib_err(chg, "Couldn't turn on SUSPEND_ON_COLLAPSE_USBIN_BIT rc=%d\n",
					rc);

		chg->qc2_unsupported_voltage = QC2_COMPLIANT;
	}

	if (chg->use_extcon)
		smblib_notify_device_mode(chg, false);

	chg->typec_legacy = false;

	del_timer_sync(&chg->apsd_timer);
	chg->apsd_ext_timeout = false;
}

static void typec_mode_unattached(struct smb_charger *chg)
{
	vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER, true, USBIN_100MA);
}

static void smblib_handle_rp_change(struct smb_charger *chg, int typec_mode)
{
	const struct apsd_result *apsd = smblib_get_apsd_result(chg);

	/*
	 * We want the ICL vote @ 100mA for a FLOAT charger
	 * until the detection by the USB stack is complete.
	 * Ignore the Rp changes unless there is a
	 * pre-existing valid vote or FLOAT is configured for
	 * SDP current.
	 */
	if (apsd->pst == POWER_SUPPLY_TYPE_USB_FLOAT) {
		if (get_client_vote(chg->usb_icl_votable, SW_ICL_MAX_VOTER)
					<= USBIN_100MA
			|| (chg->float_cfg & FLOAT_OPTIONS_MASK)
					== FORCE_FLOAT_SDP_CFG_BIT)
			return;
	}

	update_sw_icl_max(chg, apsd->pst);

	smblib_dbg(chg, PR_MISC, "CC change old_mode=%d new_mode=%d\n",
						chg->typec_mode, typec_mode);
}

static void smblib_lpd_launch_ra_open_work(struct smb_charger *chg)
{
	u8 stat;
	int rc;

	if (chg->lpd_disabled)
		return;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n",
			rc);
		return;
	}

	if (!(stat & TYPEC_TCCDEBOUNCE_DONE_STATUS_BIT)
			&& chg->lpd_stage == LPD_STAGE_NONE) {
		chg->lpd_stage = LPD_STAGE_FLOAT;
		cancel_delayed_work_sync(&chg->lpd_ra_open_work);
		vote(chg->awake_votable, LPD_VOTER, true, 0);
		schedule_delayed_work(&chg->lpd_ra_open_work,
						msecs_to_jiffies(300));
	}
}

irqreturn_t typec_or_rid_detection_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int ret;
	u8 stat;
	bool usb1_otg;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);
#ifdef CONFIG_PRODUCT_MOBA
       if(fusb_orient_en==1 && usb2_otg_en==0){
	   	ret = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
		if (ret < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n", ret);
		}
		usb1_otg = (stat & SNK_SRC_MODE_BIT);

		if(usb1_otg==1){
			pr_info("smblib_enable_otg\n");
			ret=smblib_masked_write(chg, TYPE_C_EXIT_STATE_CFG_REG,
				BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT,0x8);
			if (ret < 0)
				pr_err("Couldn't set exit state cfg ret=%d\n", ret);
		       ret = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
				EN_TRY_SNK_BIT,0);
			if (ret < 0)
				pr_err("Couldn't configure TYPE_C_MODE_CFG_REG ret=%d\n",ret);
		}
	}
#endif
	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB) {
		if (chg->uusb_moisture_protection_enabled) {
			/*
			 * Adding pm_stay_awake as because pm_relax is called
			 * on exit path from the work routine.
			 */
			pm_stay_awake(chg->dev);
			schedule_work(&chg->moisture_protection_work);
		}

		cancel_delayed_work_sync(&chg->uusb_otg_work);
		/*
		 * Skip OTG enablement if RID interrupt triggers with moisture
		 * protection still enabled.
		 */
		if (!chg->moisture_present) {
			vote(chg->awake_votable, OTG_DELAY_VOTER, true, 0);
			smblib_dbg(chg, PR_INTERRUPT, "Scheduling OTG work\n");
			schedule_delayed_work(&chg->uusb_otg_work,
				msecs_to_jiffies(chg->otg_delay_ms));
		}

		goto out;
	}

	if (chg->pr_swap_in_progress || chg->pd_hard_reset)
		goto out;

	smblib_lpd_launch_ra_open_work(chg);

	if (chg->usb_psy)
		power_supply_changed(chg->usb_psy);

out:
	return IRQ_HANDLED;
}

irqreturn_t typec_state_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int typec_mode;

	if (chg->connector_type == POWER_SUPPLY_CONNECTOR_MICRO_USB) {
		smblib_dbg(chg, PR_INTERRUPT,
				"Ignoring for micro USB\n");
		return IRQ_HANDLED;
	}

	typec_mode = smblib_get_prop_typec_mode(chg);
	if (chg->sink_src_mode != UNATTACHED_MODE
			&& (typec_mode != chg->typec_mode))
		smblib_handle_rp_change(chg, typec_mode);
	chg->typec_mode = typec_mode;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: cc-state-change; Type-C %s detected\n",
				smblib_typec_mode_name[chg->typec_mode]);

	power_supply_changed(chg->usb_psy);

	return IRQ_HANDLED;
}

static void smblib_lpd_clear_ra_open_work(struct smb_charger *chg)
{
	if (chg->lpd_disabled)
		return;

	cancel_delayed_work_sync(&chg->lpd_detach_work);
	chg->lpd_stage = LPD_STAGE_FLOAT_CANCEL;
	cancel_delayed_work_sync(&chg->lpd_ra_open_work);
	vote(chg->awake_votable, LPD_VOTER, false, 0);
}

irqreturn_t typec_attach_detach_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	bool attached = false;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_read(chg, TYPE_C_STATE_MACHINE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATE_MACHINE_STATUS_REG rc=%d\n",
			rc);
		return IRQ_HANDLED;
	}

	attached = !!(stat & TYPEC_ATTACH_DETACH_STATE_BIT);

	if (attached) {
		smblib_lpd_clear_ra_open_work(chg);

		rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n",
				rc);
			return IRQ_HANDLED;
		}

		if (smblib_get_prop_dfp_mode(chg) ==
				POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER) {
			chg->sink_src_mode = AUDIO_ACCESS_MODE;
			typec_ra_ra_insertion(chg);
		} else if (stat & SNK_SRC_MODE_BIT) {
			if (smblib_src_lpd(chg))
				return IRQ_HANDLED;
			chg->sink_src_mode = SRC_MODE;
			typec_sink_insertion(chg);
		} else {
			chg->sink_src_mode = SINK_MODE;
			typec_src_insertion(chg);
		}
#ifdef CONFIG_PRODUCT_MOBA
		first_typec_attached = 1;
		charger_insert_remove_process(chg, 1);
#endif

	} else {
#ifdef CONFIG_PRODUCT_MOBA
		reset_charger1_pd_power_data();
		first_typec_attached = 0;
		charger_insert_remove_process(chg, 0);
#endif
		switch (chg->sink_src_mode) {
		case SRC_MODE:
			typec_sink_removal(chg);
			break;
		case SINK_MODE:
		case AUDIO_ACCESS_MODE:
			typec_src_removal(chg);
			break;
		case UNATTACHED_MODE:
		default:
			typec_mode_unattached(chg);
			break;
		}

		if (!chg->pr_swap_in_progress) {
			chg->ok_to_pd = false;
			chg->sink_src_mode = UNATTACHED_MODE;
			chg->early_usb_attach = false;
			smblib_apsd_enable(chg, true);
		}

		if (chg->lpd_stage == LPD_STAGE_FLOAT_CANCEL)
			schedule_delayed_work(&chg->lpd_detach_work,
					msecs_to_jiffies(1000));
	}

	rc = smblib_masked_write(chg, USB_CMD_PULLDOWN_REG,
			EN_PULLDOWN_USB_IN_BIT,
			attached ?  0 : EN_PULLDOWN_USB_IN_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't configure pulldown on USB_IN rc=%d\n",
				rc);

	power_supply_changed(chg->usb_psy);

	return IRQ_HANDLED;
}

static void dcin_aicl(struct smb_charger *chg)
{
	int rc, icl, icl_save;
	int input_present;
	bool aicl_done = true;

	/*
	 * Hold awake votable to prevent pm_relax being called prior to
	 * completion of this work.
	 */
	vote(chg->awake_votable, DCIN_AICL_VOTER, true, 0);

increment:
	mutex_lock(&chg->dcin_aicl_lock);

	rc = smblib_get_charge_param(chg, &chg->param.dc_icl, &icl);
	if (rc < 0)
		goto err;

	if (icl == chg->wls_icl_ua) {
		/* Upper limit reached; do nothing */
		smblib_dbg(chg, PR_WLS, "hit max ICL: stop\n");

		rc = smblib_is_input_present(chg, &input_present);
		if (rc < 0 || !(input_present & INPUT_PRESENT_DC))
			aicl_done = false;

		goto unlock;
	}

	icl = min(chg->wls_icl_ua, icl + DCIN_ICL_STEP_UA);
	icl_save = icl;

	rc = smblib_set_charge_param(chg, &chg->param.dc_icl, icl);
	if (rc < 0)
		goto err;

	mutex_unlock(&chg->dcin_aicl_lock);

	smblib_dbg(chg, PR_WLS, "icl: %d mA\n", (icl / 1000));

	/* Check to see if DC is still present before and after sleep */
	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0 || !(input_present & INPUT_PRESENT_DC)) {
		aicl_done = false;
		goto unvote;
	}

	/*
	 * Wait awhile to check for any DCIN_UVs (the UV handler reduces the
	 * ICL). If the adaptor collapses, the ICL read after waking up will be
	 * lesser, indicating that the AICL process is complete.
	 */
	msleep(500);

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0 || !(input_present & INPUT_PRESENT_DC)) {
		aicl_done = false;
		goto unvote;
	}

	mutex_lock(&chg->dcin_aicl_lock);

	rc = smblib_get_charge_param(chg, &chg->param.dc_icl, &icl);
	if (rc < 0)
		goto err;

	if (icl < icl_save) {
		smblib_dbg(chg, PR_WLS, "done: icl: %d mA\n", (icl / 1000));
		goto unlock;
	}

	mutex_unlock(&chg->dcin_aicl_lock);

	goto increment;

err:
	aicl_done = false;
unlock:
	mutex_unlock(&chg->dcin_aicl_lock);
unvote:
	vote(chg->awake_votable, DCIN_AICL_VOTER, false, 0);
	chg->dcin_aicl_done = aicl_done;
}

static void dcin_aicl_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						dcin_aicl_work);
	dcin_aicl(chg);
}

static enum alarmtimer_restart dcin_aicl_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
	struct smb_charger *chg = container_of(alarm, struct smb_charger,
					dcin_aicl_alarm);

	smblib_dbg(chg, PR_WLS, "rerunning DCIN AICL\n");

	pm_stay_awake(chg->dev);
	schedule_work(&chg->dcin_aicl_work);

	return ALARMTIMER_NORESTART;
}

static void dcin_icl_decrement(struct smb_charger *chg)
{
	int rc, icl;
	ktime_t now = ktime_get();

	rc = smblib_get_charge_param(chg, &chg->param.dc_icl, &icl);
	if (rc < 0) {
		smblib_err(chg, "reading DCIN ICL failed: %d\n", rc);
		return;
	}

	if (icl == DCIN_ICL_MIN_UA) {
		/* Cannot possibly decrease ICL any further - do nothing */
		smblib_dbg(chg, PR_WLS, "hit min ICL: stop\n");
		return;
	}

	/* Reduce ICL by 100 mA if 3 UVs happen in a row */
	if (ktime_us_delta(now, chg->dcin_uv_last_time) > (200 * 1000)) {
		chg->dcin_uv_count = 0;
	} else if (chg->dcin_uv_count >= 3) {
		icl -= DCIN_ICL_STEP_UA;

		smblib_dbg(chg, PR_WLS, "icl: %d mA\n", (icl / 1000));
		rc = smblib_set_charge_param(chg, &chg->param.dc_icl, icl);
		if (rc < 0) {
			smblib_err(chg, "setting DCIN ICL failed: %d\n", rc);
			return;
		}

		chg->dcin_uv_count = 0;
	}

	chg->dcin_uv_last_time = now;
}

irqreturn_t dcin_uv_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	mutex_lock(&chg->dcin_aicl_lock);

	chg->dcin_uv_count++;
	smblib_dbg(chg, (PR_WLS | PR_INTERRUPT), "DCIN UV count: %d\n",
			chg->dcin_uv_count);
	dcin_icl_decrement(chg);

	mutex_unlock(&chg->dcin_aicl_lock);

	return IRQ_HANDLED;
}

irqreturn_t dc_plugin_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	union power_supply_propval pval;
	int input_present;
	bool dcin_present, vbus_present;
	int rc, wireless_vout = 0, wls_set = 0;
	int sec_charger;

	rc = smblib_get_prop_vph_voltage_now(chg, &pval);
	if (rc < 0)
		return IRQ_HANDLED;

	/* 2*VPH, with a granularity of 100mV */
	wireless_vout = ((pval.intval * 2) / 100000) * 100000;

	rc = smblib_is_input_present(chg, &input_present);
	if (rc < 0)
		return IRQ_HANDLED;

	dcin_present = input_present & INPUT_PRESENT_DC;
	vbus_present = input_present & INPUT_PRESENT_USB;

	if (!chg->cp_ilim_votable)
		chg->cp_ilim_votable = find_votable("CP_ILIM");

	if (dcin_present && !vbus_present) {
		cancel_work_sync(&chg->dcin_aicl_work);

		/* Reset DCIN ICL to 100 mA */
		mutex_lock(&chg->dcin_aicl_lock);
		rc = smblib_set_charge_param(chg, &chg->param.dc_icl,
				DCIN_ICL_MIN_UA);
		mutex_unlock(&chg->dcin_aicl_lock);
		if (rc < 0)
			return IRQ_HANDLED;

		smblib_dbg(chg, (PR_WLS | PR_INTERRUPT), "reset: icl: 100 mA\n");

		/*
		 * Remove USB's CP ILIM vote - inapplicable for wireless
		 * parallel charging.
		 */
		if (chg->cp_ilim_votable)
			vote(chg->cp_ilim_votable, ICL_CHANGE_VOTER, false, 0);

		if (chg->sec_cp_present) {
			/*
			 * If CP output topology is VBATT, limit main charger's
			 * FCC share and let the CPs handle the rest.
			 */
			if (is_cp_topo_vbatt(chg))
				vote(chg->fcc_main_votable,
					WLS_PL_CHARGING_VOTER, true, 800000);

			rc = smblib_get_prop_batt_status(chg, &pval);
			if (rc < 0)
				smblib_err(chg, "Couldn't read batt status rc=%d\n",
						rc);

			wls_set = (pval.intval == POWER_SUPPLY_STATUS_FULL) ?
				MICRO_5V : wireless_vout;

			pval.intval = wls_set;
			rc = smblib_set_prop_voltage_wls_output(chg, &pval);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't set dc voltage to 2*vph  rc=%d\n",
					rc);

			rc = smblib_select_sec_charger(chg,
					POWER_SUPPLY_CHARGER_SEC_CP,
					POWER_SUPPLY_CP_WIRELESS, false);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't enable secondary chargers  rc=%d\n",
					rc);
		} else {
			/*
			 * If no secondary charger is present, commence
			 * wireless charging at 5 V by default.
			 */
			pval.intval = 5000000;
			rc = smblib_set_prop_voltage_wls_output(chg, &pval);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't set dc voltage to 5 V rc=%d\n",
					rc);
		}

		schedule_work(&chg->dcin_aicl_work);
	} else {
		if (chg->cp_reason == POWER_SUPPLY_CP_WIRELESS) {
			sec_charger = chg->sec_pl_present ?
					POWER_SUPPLY_CHARGER_SEC_PL :
					POWER_SUPPLY_CHARGER_SEC_NONE;
			rc = smblib_select_sec_charger(chg, sec_charger,
					POWER_SUPPLY_CP_NONE, false);
			if (rc < 0)
				dev_err(chg->dev, "Couldn't disable secondary charger rc=%d\n",
					rc);
		}

		vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER, false, 0);
		vote(chg->fcc_main_votable, WLS_PL_CHARGING_VOTER, false, 0);

		chg->last_wls_vout = 0;
		chg->dcin_aicl_done = false;
		chg->dcin_icl_user_set = false;
	}

	/*
	 * Vote for 1500mA FCC upon WLS detach and remove vote upon attach if
	 * FCC stepper is enabled.
	 */
	if (chg->fcc_stepper_enable && !vbus_present)
		vote(chg->fcc_votable, FCC_STEPPER_VOTER, !dcin_present,
				dcin_present ? 0 : 1500000);

	if (chg->dc_psy)
		power_supply_changed(chg->dc_psy);

	smblib_dbg(chg, (PR_WLS | PR_INTERRUPT), "dcin_present= %d, usbin_present= %d, cp_reason = %d\n",
			dcin_present, vbus_present, chg->cp_reason);

	return IRQ_HANDLED;
}

irqreturn_t high_duty_cycle_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	chg->is_hdc = true;
	/*
	 * Disable usb IRQs after the flag set and re-enable IRQs after
	 * the flag cleared in the delayed work queue, to avoid any IRQ
	 * storming during the delays
	 */
	vote(chg->hdc_irq_disable_votable, HDC_IRQ_VOTER, true, 0);

	schedule_delayed_work(&chg->clear_hdc_work, msecs_to_jiffies(60));

	return IRQ_HANDLED;
}

static void smblib_bb_removal_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						bb_removal_work.work);

	vote(chg->usb_icl_votable, BOOST_BACK_VOTER, false, 0);
	vote(chg->awake_votable, BOOST_BACK_VOTER, false, 0);
}

#define BOOST_BACK_UNVOTE_DELAY_MS		750
#define BOOST_BACK_STORM_COUNT			3
#define WEAK_CHG_STORM_COUNT			8
irqreturn_t switcher_power_ok_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	struct storm_watch *wdata = &irq_data->storm_data;
	int rc, usb_icl;
	u8 stat;

	if (!(chg->wa_flags & BOOST_BACK_WA))
		return IRQ_HANDLED;

	rc = smblib_read(chg, POWER_PATH_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read POWER_PATH_STATUS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	/* skip suspending input if its already suspended by some other voter */
	usb_icl = get_effective_result(chg->usb_icl_votable);
	if ((stat & USE_USBIN_BIT) && usb_icl >= 0 && usb_icl <= USBIN_25MA)
		return IRQ_HANDLED;

	if (stat & USE_DCIN_BIT)
		return IRQ_HANDLED;

	if (is_storming(&irq_data->storm_data)) {
		/* This could be a weak charger reduce ICL */
		if (!is_client_vote_enabled(chg->usb_icl_votable,
						WEAK_CHARGER_VOTER)) {
			smblib_err(chg,
				"Weak charger detected: voting %dmA ICL\n",
				chg->weak_chg_icl_ua / 1000);
			vote(chg->usb_icl_votable, WEAK_CHARGER_VOTER,
					true, chg->weak_chg_icl_ua);
			/*
			 * reset storm data and set the storm threshold
			 * to 3 for reverse boost detection.
			 */
			update_storm_count(wdata, BOOST_BACK_STORM_COUNT);
		} else {
			smblib_err(chg,
				"Reverse boost detected: voting 0mA to suspend input\n");
			vote(chg->usb_icl_votable, BOOST_BACK_VOTER, true, 0);
			vote(chg->awake_votable, BOOST_BACK_VOTER, true, 0);
			/*
			 * Remove the boost-back vote after a delay, to avoid
			 * permanently suspending the input if the boost-back
			 * condition is unintentionally hit.
			 */
			schedule_delayed_work(&chg->bb_removal_work,
				msecs_to_jiffies(BOOST_BACK_UNVOTE_DELAY_MS));
		}
	}

	return IRQ_HANDLED;
}

irqreturn_t wdog_snarl_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (chg->wa_flags & SW_THERM_REGULATION_WA) {
		cancel_delayed_work_sync(&chg->thermal_regulation_work);
		vote(chg->awake_votable, SW_THERM_REGULATION_VOTER, true, 0);
		schedule_delayed_work(&chg->thermal_regulation_work, 0);
	}

	power_supply_changed(chg->batt_psy);

	return IRQ_HANDLED;
}

irqreturn_t wdog_bark_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	rc = smblib_write(chg, BARK_BITE_WDOG_PET_REG, BARK_BITE_WDOG_PET_BIT);
	if (rc < 0)
		smblib_err(chg, "Couldn't pet the dog rc=%d\n", rc);

	return IRQ_HANDLED;
}

static void smblib_die_rst_icl_regulate(struct smb_charger *chg)
{
	int rc;
	u8 temp;

	rc = smblib_read(chg, DIE_TEMP_STATUS_REG, &temp);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read DIE_TEMP_STATUS_REG rc=%d\n",
				rc);
		return;
	}

	/* Regulate ICL on die temp crossing DIE_RST threshold */
	vote(chg->usb_icl_votable, DIE_TEMP_VOTER,
				temp & DIE_TEMP_RST_BIT, 500000);
}

/*
 * triggered when DIE or SKIN or CONNECTOR temperature across
 * either of the _REG_L, _REG_H, _RST, or _SHDN thresholds
 */
irqreturn_t temp_change_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;

	smblib_die_rst_icl_regulate(chg);

	return IRQ_HANDLED;
}

static void smblib_usbov_dbc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						usbov_dbc_work.work);

	smblib_dbg(chg, PR_MISC, "Resetting USBOV debounce\n");
	chg->dbc_usbov = false;
	vote(chg->awake_votable, USBOV_DBC_VOTER, false, 0);
}

#define USB_OV_DBC_PERIOD_MS		1000
irqreturn_t usbin_ov_irq_handler(int irq, void *data)
{
	struct smb_irq_data *irq_data = data;
	struct smb_charger *chg = irq_data->parent_data;
	u8 stat;
	int rc;

	smblib_dbg(chg, PR_INTERRUPT, "IRQ: %s\n", irq_data->name);

	if (!(chg->wa_flags & USBIN_OV_WA))
		return IRQ_HANDLED;

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		return IRQ_HANDLED;
	}

	/*
	 * On specific PMICs, OV IRQ triggers for very small duration in
	 * interim periods affecting charging status reflection. In order to
	 * differentiate between OV IRQ glitch and real OV_IRQ, add a debounce
	 * period for evaluation.
	 */
	if (stat & USBIN_OV_RT_STS_BIT) {
		chg->dbc_usbov = true;
		vote(chg->awake_votable, USBOV_DBC_VOTER, true, 0);
		schedule_delayed_work(&chg->usbov_dbc_work,
				msecs_to_jiffies(USB_OV_DBC_PERIOD_MS));
	} else {
		cancel_delayed_work_sync(&chg->usbov_dbc_work);
		chg->dbc_usbov = false;
		vote(chg->awake_votable, USBOV_DBC_VOTER, false, 0);
	}

	smblib_dbg(chg, PR_MISC, "USBOV debounce status %d\n",
				chg->dbc_usbov);
	return IRQ_HANDLED;
}

/**************
 * Additional USB PSY getters/setters
 * that call interrupt functions
 ***************/

int smblib_get_prop_pr_swap_in_progress(struct smb_charger *chg,
				union power_supply_propval *val)
{
	val->intval = chg->pr_swap_in_progress;
	return 0;
}

#define DETACH_DETECT_DELAY_MS 20
int smblib_set_prop_pr_swap_in_progress(struct smb_charger *chg,
				const union power_supply_propval *val)
{
	int rc;
	u8 stat = 0, orientation;

	smblib_dbg(chg, PR_MISC, "Requested PR_SWAP %d\n", val->intval);
	chg->pr_swap_in_progress = val->intval;

	/* check for cable removal during pr_swap */
	if (!chg->pr_swap_in_progress) {
		cancel_delayed_work_sync(&chg->pr_swap_detach_work);
		vote(chg->awake_votable, DETACH_DETECT_VOTER, true, 0);
		schedule_delayed_work(&chg->pr_swap_detach_work,
				msecs_to_jiffies(DETACH_DETECT_DELAY_MS));
	}

	rc = smblib_masked_write(chg, TYPE_C_DEBOUNCE_OPTION_REG,
			REDUCE_TCCDEBOUNCE_TO_2MS_BIT,
			val->intval ? REDUCE_TCCDEBOUNCE_TO_2MS_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't set tCC debounce rc=%d\n", rc);

	rc = smblib_masked_write(chg, TYPE_C_EXIT_STATE_CFG_REG,
			BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT,
			val->intval ? BYPASS_VSAFE0V_DURING_ROLE_SWAP_BIT : 0);
	if (rc < 0)
		smblib_err(chg, "Couldn't set exit state cfg rc=%d\n", rc);

	if (chg->pr_swap_in_progress) {
		rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_STATUS_4 rc=%d\n",
				rc);
		}

		orientation =
			stat & CC_ORIENTATION_BIT ? TYPEC_CCOUT_VALUE_BIT : 0;
		rc = smblib_masked_write(chg, TYPE_C_CCOUT_CONTROL_REG,
			TYPEC_CCOUT_SRC_BIT | TYPEC_CCOUT_BUFFER_EN_BIT
					| TYPEC_CCOUT_VALUE_BIT,
			TYPEC_CCOUT_SRC_BIT | TYPEC_CCOUT_BUFFER_EN_BIT
					| orientation);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_CCOUT_CONTROL_REG rc=%d\n",
				rc);
		}
	} else {
		rc = smblib_masked_write(chg, TYPE_C_CCOUT_CONTROL_REG,
			TYPEC_CCOUT_SRC_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read TYPE_C_CCOUT_CONTROL_REG rc=%d\n",
				rc);
			return rc;
		}

		/* enable DRP */
		rc = smblib_masked_write(chg, TYPE_C_MODE_CFG_REG,
				 TYPEC_POWER_ROLE_CMD_MASK, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't enable DRP rc=%d\n", rc);
			return rc;
		}
		chg->power_role = POWER_SUPPLY_TYPEC_PR_DUAL;
		smblib_dbg(chg, PR_MISC, "restore power role: %d\n",
				chg->power_role);
	}

	return 0;
}

/***************
 * Work Queues *
 ***************/
static void smblib_pr_lock_clear_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pr_lock_clear_work.work);

	spin_lock(&chg->typec_pr_lock);
	if (chg->pr_lock_in_progress) {
		smblib_dbg(chg, PR_MISC, "restore type-c interrupts\n");
		smblib_typec_irq_config(chg, true);
		chg->pr_lock_in_progress = false;
	}
	spin_unlock(&chg->typec_pr_lock);
}

static void smblib_pr_swap_detach_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pr_swap_detach_work.work);
	int rc;
	u8 stat;

	rc = smblib_read(chg, TYPE_C_STATE_MACHINE_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read STATE_MACHINE_STS rc=%d\n", rc);
		goto out;
	}
	smblib_dbg(chg, PR_REGISTER, "STATE_MACHINE_STS %x\n", stat);
	if (!(stat & TYPEC_ATTACH_DETACH_STATE_BIT)) {
		rc = smblib_request_dpdm(chg, false);
		if (rc < 0)
			smblib_err(chg, "Couldn't disable DPDM rc=%d\n", rc);
	}
out:
	vote(chg->awake_votable, DETACH_DETECT_VOTER, false, 0);
}

static void smblib_uusb_otg_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						uusb_otg_work.work);
	int rc;
	u8 stat;
	bool otg;

	rc = smblib_read(chg, TYPEC_U_USB_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_STATUS_3 rc=%d\n", rc);
		goto out;
	}
	otg = !!(stat & U_USB_GROUND_NOVBUS_BIT);
	if (chg->otg_present != otg)
		smblib_notify_usb_host(chg, otg);
	else
		goto out;

	chg->otg_present = otg;
	if (!otg)
		chg->boost_current_ua = 0;

	rc = smblib_set_charge_param(chg, &chg->param.freq_switcher,
				otg ? chg->chg_freq.freq_below_otg_threshold
					: chg->chg_freq.freq_removal);
	if (rc < 0)
		dev_err(chg->dev, "Error in setting freq_boost rc=%d\n", rc);

	smblib_dbg(chg, PR_REGISTER, "TYPE_C_U_USB_STATUS = 0x%02x OTG=%d\n",
			stat, otg);
	power_supply_changed(chg->usb_psy);

out:
	vote(chg->awake_votable, OTG_DELAY_VOTER, false, 0);
}

#ifdef CONFIG_PRODUCT_MOBA
int ffc_get_batt1_vol(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval pval = {0, };

	if (!chg->fcc_qcom_batt_psy) {
		chg->fcc_qcom_batt_psy= power_supply_get_by_name("bq27z561-master");
		if (!chg->fcc_qcom_batt_psy)
			return 0;
	}

	rc = power_supply_get_property(chg->fcc_qcom_batt_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);

	return pval.intval;
}

int ffc_get_batt1_curr(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval pval = {0, };

	if (!chg->fcc_qcom_batt_psy) {
		chg->fcc_qcom_batt_psy= power_supply_get_by_name("bq27z561-master");
		if (!chg->fcc_qcom_batt_psy)
			return 0;
	}

	rc = power_supply_get_property(chg->fcc_qcom_batt_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &pval);

	return pval.intval;
}

int ffc_get_batt1_fg(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval pval = {0, };

	if (!chg->fcc_qcom_batt_psy) {
		chg->fcc_qcom_batt_psy= power_supply_get_by_name("bq27z561-master");
		if (!chg->fcc_qcom_batt_psy)
			return 0;
	}

	rc = power_supply_get_property(chg->fcc_qcom_batt_psy,
				POWER_SUPPLY_PROP_CAPACITY, &pval);

	return pval.intval;
}

int ffc_get_batt1_temp(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval pval = {0, };

	if (!chg->fcc_qcom_batt_psy) {
		chg->fcc_qcom_batt_psy= power_supply_get_by_name("bq27z561-master");
		if (!chg->fcc_qcom_batt_psy)
			return 0;
	}

	rc = power_supply_get_property(chg->fcc_qcom_batt_psy,
				POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		pr_err("Couldn't get battery1 for ffc, rc=%d\n", rc);
		return 0;
	}
	/*250*/
	pr_debug("batt 1 temp is %d\n", pval.intval);
	return pval.intval;
}


int ffc_get_batt1_temp_level(struct smb_charger *chg)
{
	int batt1_temp;

	batt1_temp = ffc_get_batt1_temp(chg);
	/*TEMP BELOW 0*/
	if (batt1_temp < FFC_THRESHOLD_0C)
		return T_COLD_THRESHOLD_BL0C;
	/*TEMP 0-10*/
	else if((batt1_temp < FFC_THRESHOLD_10C) && (batt1_temp >= FFC_THRESHOLD_0C))
		return T_COLD_THRESHOLD_0T10C;
	/*TEMP 10-15*/
	else if((batt1_temp < FFC_THRESHOLD_15C) && (batt1_temp >= FFC_THRESHOLD_10C))
		return T_COOL_THRESHOLD_10T15C;
	/*TEMP 15-30*/
	else if((batt1_temp < FFC_THRESHOLD_30C) && (batt1_temp >= FFC_THRESHOLD_15C))
		return T_COOL_THRESHOLD_15T30C;
	/*TEMP 35-45*/
	else if((batt1_temp < FFC_THRESHOLD_46C) && (batt1_temp >= FFC_THRESHOLD_30C))
		return T_GOOD_THRESHOLD_30T46C;
	/*TEMP 45-60*/
	else if((batt1_temp < FFC_THRESHOLD_60C) && (batt1_temp >= FFC_THRESHOLD_46C))
		return T_WARM_THRESHOLD_45T60C;
	/*TEMP UP 60*/
	else
		return T_HOT_THRESHOLD_UP60C;
}

static int ffc_get_batt2_vol(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval pval = {0, };

	if (!chg->fcc_25890h_batt_psy) {
		chg->fcc_25890h_batt_psy= power_supply_get_by_name("bq27z561-slave");
		if (!chg->fcc_25890h_batt_psy)
			return 0;
	}

	rc = power_supply_get_property(chg->fcc_25890h_batt_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);

	return pval.intval;
}

static int ffc_get_batt2_curr(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval pval = {0, };

	if (!chg->fcc_25890h_batt_psy) {
		chg->fcc_25890h_batt_psy= power_supply_get_by_name("bq27z561-slave");
		if (!chg->fcc_25890h_batt_psy)
			return 0;
	}

	rc = power_supply_get_property(chg->fcc_25890h_batt_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &pval);

	return pval.intval;
}

static int ffc_get_batt2_fg(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval pval = {0, };

	if (!chg->fcc_25890h_batt_psy) {
		chg->fcc_25890h_batt_psy= power_supply_get_by_name("bq27z561-slave");
		if (!chg->fcc_25890h_batt_psy)
			return 0;
	}

	rc = power_supply_get_property(chg->fcc_25890h_batt_psy,
			POWER_SUPPLY_PROP_CAPACITY, &pval);

	return pval.intval;
}

int ffc_get_batt2_temp(struct smb_charger *chg)
{
	int rc;
	union power_supply_propval pval = {0, };

	if (!chg->fcc_25890h_batt_psy) {
		chg->fcc_25890h_batt_psy= power_supply_get_by_name("bq27z561-slave");
		if (!chg->fcc_25890h_batt_psy)
			return 0;
	}

	rc = power_supply_get_property(chg->fcc_25890h_batt_psy,
				POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		pr_err("Couldn't get battery2 for ffc, rc=%d\n", rc);
		return 0;
	}
	/*250*/
	pr_debug("batt 2 temp is %d\n", pval.intval);
	return pval.intval;
}
static int ffc_get_batt2_temp_level(struct smb_charger *chg)
{
	int batt2_temp;

	batt2_temp = ffc_get_batt2_temp(chg);
	/*TEMP BELOW 0*/
	if (batt2_temp < FFC_THRESHOLD_0C)
		return T_COLD_THRESHOLD_BL0C;
	/*TEMP 0-10*/
	else if((batt2_temp < FFC_THRESHOLD_10C) && (batt2_temp >= FFC_THRESHOLD_0C))
		return T_COLD_THRESHOLD_0T10C;
	/*TEMP 10-15*/
	else if((batt2_temp < FFC_THRESHOLD_15C) && (batt2_temp >= FFC_THRESHOLD_10C))
		return T_COOL_THRESHOLD_10T15C;
	/*TEMP 15-30*/
	else if((batt2_temp < FFC_THRESHOLD_30C) && (batt2_temp >= FFC_THRESHOLD_15C))
		return T_COOL_THRESHOLD_15T30C;
	/*TEMP 35-45*/
	else if((batt2_temp < FFC_THRESHOLD_46C) && (batt2_temp >= FFC_THRESHOLD_30C))
		return T_GOOD_THRESHOLD_30T46C;
	/*TEMP 45-60*/
	else if((batt2_temp < FFC_THRESHOLD_60C) && (batt2_temp >= FFC_THRESHOLD_46C))
		return T_WARM_THRESHOLD_45T60C;
	/*TEMP UP 60*/
	else
		return T_HOT_THRESHOLD_UP60C;
}

int lenovo_get_adapters_connect_status(void)
{
	if ((!qcom_orient_en) && (!fusb_orient_en))
		return TYPEC_PORT_VALUE_UNKNOWN;
	else if ((qcom_orient_en) && (!fusb_orient_en))
		return TYPEC_PORT_VALUE_FIR;
	else if ((!qcom_orient_en) && (fusb_orient_en))
		return TYPEC_PORT_VALUE_SEC;
	else
		return TYPEC_PORT_VALUE_BOTH;
}
#if 0
static int ffc_port1_enable_cp(struct smb_charger *chg, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!chg->p1_cp_psy) {
		chg->p1_cp_psy = power_supply_get_by_name("bq2597x-master");
		if (!chg->p1_cp_psy)
			pr_err("ffc_bq2597x-master found\n");
	}

	if (!chg->p1_cp_psy)
		return -ENODEV;

	val.intval = enable;
	ret = power_supply_set_property(chg->p1_cp_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);

	return ret;
}

static int ffc_port1_check_cp_enabled(struct smb_charger *chg)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!chg->p1_cp_psy) {
		chg->p1_cp_psy = power_supply_get_by_name("bq2597x-master");
		if (!chg->p1_cp_psy)
			pr_err("ffc_bq2597x-master found\n");
	}

	if (!chg->p1_cp_psy)
		return -ENODEV;

	ret = power_supply_get_property(chg->p1_cp_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		chg->p1_cp_en= !!val.intval;

	return ret;
}
static int ffc_port1_enable_cp_sec(struct smb_charger *chg, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!chg->p1_cp_sec_psy) {
		chg->p1_cp_sec_psy = power_supply_get_by_name("bq2597x-slave");
		if (!chg->p1_cp_sec_psy)
			pr_err("ffc_bq2597x-slave not found\n");
	}

	if (!chg->p1_cp_sec_psy)
		return -ENODEV;

	val.intval = enable;
	ret = power_supply_set_property(chg->p1_cp_sec_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);

	return ret;
}

static int ffc_port1_check_cp_sec_enabled(struct smb_charger *chg)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!chg->p1_cp_sec_psy) {
		chg->p1_cp_sec_psy = power_supply_get_by_name("bq2597x-slave");
		if (!chg->p1_cp_sec_psy)
			pr_err("ffc_bq2597x-slave not found\n");
	}

	if (!chg->p1_cp_sec_psy)
		return -ENODEV;

	ret = power_supply_get_property(chg->p1_cp_sec_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		chg->p1_cp_sec_en= !!val.intval;

	return ret;
}
static int ffc_port2_enable_cp(struct smb_charger *chg, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!chg->p2_cp_psy) {
		chg->p2_cp_psy = power_supply_get_by_name("bq2597x-sec-master");
		if (!chg->p2_cp_psy)
			pr_err("ffc_bq2597x-sec-master not found\n");
	}

	if (!chg->p2_cp_psy)
		return -ENODEV;

	val.intval = enable;
	ret = power_supply_set_property(chg->p2_cp_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);

	return ret;
}

static int ffc_port2_check_cp_enabled(struct smb_charger *chg)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!chg->p2_cp_psy) {
		chg->p2_cp_psy = power_supply_get_by_name("bq2597x-sec-master");
		if (!chg->p2_cp_psy)
			pr_err("ffc_bq2597x-sec-master not found\n");
	}

	if (!chg->p2_cp_psy)
		return -ENODEV;

	ret = power_supply_get_property(chg->p2_cp_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		chg->p2_cp_en = !!val.intval;

	return ret;
}
static int ffc_port2_enable_cp_sec(struct smb_charger *chg, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!chg->p2_cp_sec_psy) {
		chg->p2_cp_sec_psy = power_supply_get_by_name("bq2597x-sec-slave");
		if (!chg->p2_cp_sec_psy)
			pr_err("ffc_bq2597x-sec-slave not found\n");
	}

	if (!chg->p2_cp_sec_psy)
		return -ENODEV;

	val.intval = enable;
	ret = power_supply_set_property(chg->p2_cp_sec_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);

	return ret;
}

static int ffc_port2_check_cp_sec_enabled(struct smb_charger *chg)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!chg->p2_cp_sec_psy) {
		chg->p2_cp_sec_psy = power_supply_get_by_name("bq2597x-sec-slave");
		if (!chg->p2_cp_sec_psy)
			pr_err("ffc_bq2597x-sec-slave not found\n");
	}

	if (!chg->p2_cp_sec_psy)
		return -ENODEV;

	ret = power_supply_get_property(chg->p1_cp_sec_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		chg->p2_cp_sec_en= !!val.intval;

	return ret;
}
#endif

static int ffc_get_flash_charge_max_current(int batt_vol, int temp_lv, int request_curr)
{
	int adapter_status;
	int bq_current = FFC_PD_MAX_CURRENT_7P5A;
	int pre_max_current;

	adapter_status = lenovo_get_adapters_connect_status();

	pre_max_current = min(FFC_PD_MAX_CURRENT_3P75A, request_curr);

	if ((!qcom_pd_enable) && (!fusb_pd_enable))
		goto out;

	switch(temp_lv){
		case T_COOL_THRESHOLD_15T30C:
		case T_GOOD_THRESHOLD_30T46C:
			if (batt_vol <= FFC_STEP_VOLT){
				if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
					if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
						bq_current = 2*pre_max_current;
					else
						bq_current = FFC_PD_MAX_CURRENT_6A;
				}
				else if (adapter_status == TYPEC_PORT_VALUE_BOTH)
					bq_current = 2*pre_max_current;
			}
			else{
				if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
					if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
						bq_current = 2*pre_max_current - 1000;
					else
						bq_current = FFC_PD_MAX_CURRENT_5A;
				}
				else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
					if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
						bq_current = 2*pre_max_current - 1000;
					else
						bq_current = FFC_PD_MAX_CURRENT_5A;
				}
			}
			break;
		default:
			break;
	}
out:
	return bq_current;
}


#define BATT_FFC_TERM_CURRENT1		-900000
#define BATT_FFC_TERM_CURRENT2		-1500000
#define BATT_FFC_TERM_VOL		4470000

bool detect_stop_chg_en(int batt_vol, int batt_curr, int batt_fg, int temp_lvl)
{
	bool chg_en;
	int vol_buff = 70000;

	chg_en = true;
	switch(temp_lvl){
	case T_COOL_THRESHOLD_15T30C:
		if (batt_vol >= BATT_FFC_TERM_VOL)
			chg_en = true;
		else if (((batt_vol <= BATT_FFC_TERM_VOL) && (batt_curr >= (BATT_FFC_TERM_VOL - vol_buff))
				&& (batt_curr >= BATT_FFC_TERM_CURRENT1)) || (batt_fg == 100))
			chg_en = true;
		else
			chg_en = false;
		break;
	case T_GOOD_THRESHOLD_30T46C:
		if (batt_vol > BATT_FFC_TERM_VOL)
			chg_en = true;
		else if (((batt_vol <= BATT_FFC_TERM_VOL) && (batt_curr >= (BATT_FFC_TERM_VOL - vol_buff))
				&& (batt_curr >= BATT_FFC_TERM_CURRENT2)) || (batt_fg == 100))
			chg_en = true;
		else
			chg_en = false;
		break;
	default:
		break;
	}
	return chg_en;
}

int lenovo_ffc1_thermal_set_current(int batt_vol, int batt_temp, int request_curr)
{
	int thermal_curr;
	int adapter_status;

	int pre_max_current;
	int s_power_buff;

	s_power_buff = 500;
	adapter_status = lenovo_get_adapters_connect_status();
	pre_max_current = min(FFC_PD_MAX_CURRENT_3P75A, request_curr);
	thermal_curr = FFC_PD_MAX_CURRENT_2P5A;

	if (batt_temp <= THERMAL_T0){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_7P5A;
		}
	}
	else if ((batt_temp >THERMAL_T0) && (batt_temp <=THERMAL_T1)){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_6A;
		}
	}
	else if ((batt_temp >THERMAL_T1) && (batt_temp <=THERMAL_T2)){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current - s_power_buff;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_5P5A;
		}
	}
	else if ((batt_temp >THERMAL_T2) && (batt_temp <=THERMAL_T3)){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current - 2*s_power_buff;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_5A;
		}
	}
	else if ((batt_temp >THERMAL_T3) && (batt_temp <=THERMAL_T4)){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current - 3*s_power_buff;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_4P5A;
		}
	}
	else if ((batt_temp >THERMAL_T4) && (batt_temp <=THERMAL_T5)){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current - 4*s_power_buff;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_4A;
		}
	}
	else if ((batt_temp >THERMAL_T5) && (batt_temp <=THERMAL_T6)){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current - 5*s_power_buff;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_3P5A;
		}
	}
	return thermal_curr;
}

int lenovo_ffc2_thermal_set_current(int batt_vol, int batt_temp, int request_curr)
{
	int thermal_curr;
	int adapter_status;

	int pre_max_current;
	int temp_buff;
	int s_power_buff;

	temp_buff = 20;
	s_power_buff = 500;

	adapter_status = lenovo_get_adapters_connect_status();
	pre_max_current = min(FFC_PD_MAX_CURRENT_3P75A, request_curr);
	thermal_curr = FFC_PD_MAX_CURRENT_2P5A;

	if (batt_temp <= (THERMAL_T0 + temp_buff)){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_7P5A;
		}
	}
	else if ((batt_temp >(THERMAL_T0 + temp_buff)) && (batt_temp <=(THERMAL_T1 + temp_buff))){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_6A;
		}
	}
	else if ((batt_temp >(THERMAL_T1 + temp_buff)) && (batt_temp <=(THERMAL_T2 + temp_buff))){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current - s_power_buff;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_5P5A;
		}
	}
	else if ((batt_temp >(THERMAL_T2 + temp_buff)) && (batt_temp <=(THERMAL_T3 + temp_buff))){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current- 2*s_power_buff;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_5A;
		}
	}
	else if ((batt_temp >(THERMAL_T3 + temp_buff)) && (batt_temp <=(THERMAL_T4 + temp_buff))){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current - 3*s_power_buff;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_4P5A;
		}
	}
	else if ((batt_temp >(THERMAL_T4 + temp_buff)) && (batt_temp <=(THERMAL_T5 + temp_buff))){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current - 4*s_power_buff;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_4A;
		}
	}
	else if ((batt_temp >(THERMAL_T5 + temp_buff)) && (batt_temp <=(THERMAL_T6 + temp_buff))){
		if ((adapter_status == TYPEC_PORT_VALUE_FIR) ||
					(adapter_status == TYPEC_PORT_VALUE_SEC)){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_8A;
		}
		else if (adapter_status == TYPEC_PORT_VALUE_BOTH){
			if (pre_max_current < FFC_PD_MAX_CURRENT_3P75A)
				thermal_curr = 2*pre_max_current - 5*s_power_buff;
			else
				thermal_curr = FFC_PD_MAX_CURRENT_3P5A;
		}
	}
	return thermal_curr;
}

int ffc_check_batt_full(int batt_vol, int batt_curr, int batt_fg, int temp_lvl)
{
	bool batt_f;
	int vol_buff = 70000;

	batt_f = false;

	switch(temp_lvl){
	case T_COOL_THRESHOLD_15T30C:
		if (((batt_vol <= BATT_FFC_TERM_VOL) && (batt_curr >= (BATT_FFC_TERM_VOL - vol_buff))
				&& (batt_curr >= BATT_FFC_TERM_CURRENT1)) || (batt_fg == 100))
			batt_f = true;
		else
			batt_f = false;
		break;
	case T_GOOD_THRESHOLD_30T46C:
		if (((batt_vol <= BATT_FFC_TERM_VOL) && (batt_curr >= (BATT_FFC_TERM_VOL - vol_buff))
				&& (batt_curr >= BATT_FFC_TERM_CURRENT2)) || (batt_fg == 100))
			batt_f = true;
		else
			batt_f = false;
		break;
	default:
		break;
	}
	return batt_f;
}

bool ffc_pd1_get_flash_chg_ready(void)
{
	pr_info("qcom_pd_enable = %d,is_batt1_stop = %d,is_pd1_done = %d\n",
		qcom_pd_enable, is_batt1_stop, is_pd1_done);

	if ((qcom_pd_enable == 2) && !is_batt1_stop && (is_pd1_done == 1))
		return true;
	else
		return false;
}
bool ffc_pd2_get_flash_chg_ready(void)
{
	pr_info("fusb_pd_enable = %d,is_batt2_stop = %d,is_pd2_done = %d\n",
		fusb_pd_enable, is_batt2_stop, is_pd2_done);

	if (fusb_pd_enable && !is_batt2_stop && (is_pd2_done == 1))
		return true;
	else
		return false;
}

static void lenovo_ffc_charge_algorithm(struct smb_charger *chg)
{
	int batt1_temp_level_now;
	int batt2_temp_level_now;
	int batt1_t;
	int batt2_t;
	int batt1_vol,batt1_curr,batt1_cap;
	int batt2_vol,batt2_curr,batt2_cap;
	int thermal1_lv_curr;
	int thermal2_lv_curr;
	int bq2597x_curr_1;
	int bq2597x_curr_2;

	int batt1_request_curr;
	int batt2_request_curr;
	/*get batt1 and batt2 request current*/
	batt1_request_curr = realpd1_request_curr;
	batt2_request_curr = realpd2_request_curr;
	pr_info("batt1_request_curr = %d,batt2_request_curr = %d\n",
		batt1_request_curr, batt2_request_curr);

	/*get batt1 and batt2 temp lvl*/
	batt1_t = ffc_get_batt1_temp(chg);
	batt2_t = ffc_get_batt2_temp(chg);
	batt1_temp_level_now = ffc_get_batt1_temp_level(chg);
	batt2_temp_level_now = ffc_get_batt2_temp_level(chg);
	pr_debug("batt1 templv = %d, batt2 temp_lv = %d\n", batt1_temp_level_now, batt2_temp_level_now);

	/*get batt1 and batt2 vol*/
	batt1_vol = ffc_get_batt1_vol(chg);
	batt2_vol = ffc_get_batt2_vol(chg);
	pr_debug("batt1_vol = %d, batt2_vol = %d\n", batt1_vol, batt2_vol);

	/*get batt1 and batt2 current*/
	batt1_curr = ffc_get_batt1_curr(chg);
	batt2_curr = ffc_get_batt2_curr(chg);
	pr_debug("batt1_curr = %d, batt2_curr = %d\n", batt1_curr, batt2_curr);

	/*get batt1 and batt2 thermal current*/
	thermal1_lv_curr = lenovo_ffc1_thermal_set_current(batt1_vol, batt1_t, batt1_request_curr);
	thermal2_lv_curr = lenovo_ffc2_thermal_set_current(batt2_vol, batt2_t, batt2_request_curr);
	pr_info("thermal1_lv_curr = %d, thermal2_lv_curr = %d\n", thermal1_lv_curr, thermal2_lv_curr);

	/*get batt1 and batt2 capacity*/
	batt1_cap = ffc_get_batt1_fg(chg);
	batt2_cap = ffc_get_batt2_fg(chg);
	pr_debug("batt1_cap = %d, batt2_cap = %d\n", batt1_cap, batt2_cap);

	qcom_pd_enable = chg->pd_active;
	/*get main and sec ports request max pd current*/
	if (qcom_pd_enable == 2){
		is_batt1_stop = detect_stop_chg_en(batt1_vol, batt1_curr, batt1_cap, batt1_temp_level_now);
		is_batt1_full = ffc_check_batt_full(batt1_vol, batt1_curr, batt1_cap, batt1_temp_level_now);
		pr_info("batt1_stop = %d, batt1_full = %d\n", is_batt1_stop, is_batt1_full);

		if (!is_batt1_stop){
			bq2597x_curr_1 = ffc_get_flash_charge_max_current(batt1_vol, batt1_temp_level_now, batt1_request_curr);
			pd1_max_current = min(thermal1_lv_curr, bq2597x_curr_1);
			if (thermal_ctrl_batt1_current > 0){
				pd1_max_current = thermal_ctrl_batt1_current;
				pr_info("thermal set pd1 max current = %d\n", thermal_ctrl_batt1_current);
			}
			pr_info("pd1_curr = %d\n", pd1_max_current);
		}
	}
	if (fusb_pd_enable){
		is_batt2_stop = detect_stop_chg_en(batt2_vol, batt2_curr, batt2_cap, batt2_temp_level_now);
		is_batt2_full = ffc_check_batt_full(batt2_vol, batt2_curr, batt2_cap, batt2_temp_level_now);
		pr_debug("batt2_stop = %d, batt2_full = %d\n", is_batt2_stop, is_batt2_full);

		if (!is_batt2_stop){
			bq2597x_curr_2 = ffc_get_flash_charge_max_current(batt2_vol, batt2_temp_level_now, batt2_request_curr);
			pd2_max_current = min(thermal2_lv_curr, bq2597x_curr_2);
			if (thermal_ctrl_batt2_current > 0){
				pd2_max_current = thermal_ctrl_batt2_current;
				pr_info("thermal set pd2 max current = %d\n", thermal_ctrl_batt2_current);
			}
			pr_info("pd2_curr = %d\n", pd2_max_current);
		}
	}
}

static void lenovo_ffc_ctrl_workfunc(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						lenovo_ffc_ctrl_work.work);

	if (lenovo_get_adapters_connect_status() != TYPEC_PORT_VALUE_UNKNOWN){
		lenovo_ffc_charge_algorithm(chg);
		pr_debug("run ffc algorithm\n");
		}
	schedule_delayed_work(&chg->lenovo_ffc_ctrl_work,
			msecs_to_jiffies(LENOVO_FFC_CTRL_WORK_MS));

}
#endif

#ifdef CONFIG_PRODUCT_MOBA
int last_st_val = 0;
int new_st_val = 0;
bool port_status_changed_enable(int status_value)
{
	last_st_val = status_value;
	if (last_st_val != new_st_val)
	{
		new_st_val = last_st_val;
		pr_debug("port online status changed\n");
		return 1;
	}else{
		pr_debug("port online status not changed\n");
		return 0;
	}
}

void lenovo_set_gpios_ctrl(struct smb_charger *chg, bool st_changed)
{
	bool reset_gpio_val;

	reset_gpio_val = st_changed;

	switch(chg->ports_insert_value)
	{
	case TYPEC_PORT_VALUE_FIR:
		if (reset_gpio_val){
			if (!is_protect_data){
				gpio_set_value(chg->charge_1t1_gpio, 1);
				gpio_set_value(chg->charge_1t2_gpio, 1);
				gpio_set_value(chg->charge_2t1_gpio, 1);
				gpio_set_value(chg->charge_2t2_gpio, 1);
				msleep(300);
			}
		}
		if (is_protect_data){
			gpio_set_value(chg->charge_1t1_gpio, 0);
			gpio_set_value(chg->charge_1t2_gpio, 1);
			gpio_set_value(chg->charge_2t1_gpio, 1);
			gpio_set_value(chg->charge_2t2_gpio, 0);
		}
		else {
			gpio_set_value(chg->charge_1t1_gpio, 0);
			gpio_set_value(chg->charge_1t2_gpio, 0);
			gpio_set_value(chg->charge_2t1_gpio, 1);
			gpio_set_value(chg->charge_2t2_gpio, 0);
		}
		pr_info("main port charge two batters\n");
		break;
	case TYPEC_PORT_VALUE_SEC:
		if (reset_gpio_val){
			if (!is_protect_data){
				gpio_set_value(chg->charge_1t1_gpio, 1);
				gpio_set_value(chg->charge_1t2_gpio, 1);
				gpio_set_value(chg->charge_2t1_gpio, 1);
				gpio_set_value(chg->charge_2t2_gpio, 1);
				msleep(300);
			}
		}
		if (is_protect_data){
			gpio_set_value(chg->charge_1t1_gpio, 0);
			gpio_set_value(chg->charge_1t2_gpio, 1);
			gpio_set_value(chg->charge_2t1_gpio, 1);
			gpio_set_value(chg->charge_2t2_gpio, 0);
		}
		else{
			gpio_set_value(chg->charge_1t1_gpio, 0);
			gpio_set_value(chg->charge_1t2_gpio, 1);
			gpio_set_value(chg->charge_2t1_gpio, 0);
			gpio_set_value(chg->charge_2t2_gpio, 0);
		}
		pr_info("sec port charge two batters\n");
		break;
	case TYPEC_PORT_VALUE_BOTH:
		if (reset_gpio_val){
			if (!is_protect_data){
				gpio_set_value(chg->charge_1t1_gpio, 1);
				gpio_set_value(chg->charge_1t2_gpio, 1);
				gpio_set_value(chg->charge_2t1_gpio, 1);
				gpio_set_value(chg->charge_2t2_gpio, 1);
				msleep(450);
			}
		}
		gpio_set_value(chg->charge_1t1_gpio, 0);
		gpio_set_value(chg->charge_1t2_gpio, 1);
		gpio_set_value(chg->charge_2t1_gpio, 1);
		gpio_set_value(chg->charge_2t2_gpio, 0);
		pr_info("two ports charge two batters\n");
		break;
	case TYPEC_PORT_VALUE_UNKNOWN:
		if (reset_gpio_val){
			gpio_set_value(chg->charge_1t1_gpio, 1);
			gpio_set_value(chg->charge_1t2_gpio, 1);
			gpio_set_value(chg->charge_2t1_gpio, 1);
			gpio_set_value(chg->charge_2t2_gpio, 1);
			msleep(450);
		}
		gpio_set_value(chg->charge_1t1_gpio, 0);
		gpio_set_value(chg->charge_1t2_gpio, 1);
		gpio_set_value(chg->charge_2t1_gpio, 1);
		gpio_set_value(chg->charge_2t2_gpio, 0);
		break;
	}
}

void enable_otg_charge(struct smb_charger *chg)
{
          if(usb1_otg_en==1&&usb2_otg_en==0){
		pr_info("enable_usb2_charge\n");
		gpio_set_value(chg->charge_2t1_gpio, 0);
		gpio_set_value(chg->charge_2t2_gpio, 0);
          }
	   else if(usb1_otg_en==0&&usb2_otg_en==1){
	   	pr_info("enable_usb1_charge\n");
		gpio_set_value(chg->charge_2t1_gpio, 1);
		gpio_set_value(chg->charge_2t2_gpio, 1);
		gpio_set_value(chg->charge_1t1_gpio, 0);
		gpio_set_value(chg->charge_1t2_gpio, 0);
          }
}

#ifdef CONFIG_PRODUCT_MOBA
int get_charge_path_status(struct smb_charger *chg, int n)
{
	switch (n) {
	case 1:
		return gpio_get_value(chg->charge_1t1_gpio);
	case 2:
		return gpio_get_value(chg->charge_1t2_gpio);
	case 3:
		return gpio_get_value(chg->charge_2t1_gpio);
	case 4:
		return gpio_get_value(chg->charge_2t2_gpio);
	default:
		return -1;
	}

	return 0;
}

int get_register(struct smb_charger *chg, u16 reg)
{
	int rc;
	u8 stat;

	rc = smblib_read(chg, reg, &stat);

	return (int)stat;
}
#endif

static u8 usb_boot_check = 0;
static bool is_qcom_usb_available(struct smb_charger *chg)
{
	if (!chg->usb_psy)
		chg->usb_psy = power_supply_get_by_name("usb");

	return !!chg->usb_psy;
}

static int is_usb_first_port(struct smb_charger *chg)
{
	int rc;
	bool is_usb;
	union power_supply_propval pval;

	if (!is_qcom_usb_available(chg)) {
		pr_info("qcom usb psy error\n");
		return false;
	}

	rc = power_supply_get_property(chg->usb_psy,
				POWER_SUPPLY_PROP_REAL_TYPE, &pval);
	if (rc < 0) {
		pr_info("get first usb POWER_SUPPLY_PROP_REAL_TYPE error\n");
		return false;
	}

	if ((pval.intval == POWER_SUPPLY_TYPE_USB) || (pval.intval == POWER_SUPPLY_TYPE_USB_CDP))
		is_usb = 1;
	else
		is_usb = 0;

	pr_info("first usb device : %s\n", is_usb ? "true" : "false");
	return is_usb;
}

static int is_usb_sec_port(struct smb_charger *chg)
{
	int rc;
	bool is_usb;
	union power_supply_propval pval;

	if (!chg->fusb302_usb_charge_psy) {
		chg->fusb302_usb_charge_psy = power_supply_get_by_name("bq2589h-charger");
		if (!chg->fusb302_usb_charge_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->fusb302_usb_charge_psy,
				POWER_SUPPLY_PROP_REAL_TYPE, &pval);
	if (rc < 0) {
		pr_info("get second usb POWER_SUPPLY_PROP_REAL_TYPE error\n");
		return false;
	}

	if ((pval.intval == POWER_SUPPLY_TYPE_USB) || (pval.intval == POWER_SUPPLY_TYPE_USB_CDP))
		is_usb = 1;
	else
		is_usb = 0;

	pr_info("second usb device : %s\n", is_usb ? "true" : "false");
	return is_usb;
}

static void lenovo_monitor_ports_status_work(struct work_struct *work)
{
	bool gpio_status_1t1;
	bool gpio_status_1t2;
	bool gpio_status_2t1;
	bool gpio_status_2t2;
	int rc;

	bool insert_val_changed;

	struct smb_charger *chg = container_of(work, struct smb_charger,
			lenovo_monitor_ports_status_work.work);

	gpio_status_1t1 = gpio_get_value(chg->charge_1t1_gpio);
	gpio_status_1t2 = gpio_get_value(chg->charge_1t2_gpio);
	gpio_status_2t1 = gpio_get_value(chg->charge_2t1_gpio);
	gpio_status_2t2 = gpio_get_value(chg->charge_2t2_gpio);
	pr_info("gpio_1t1 is %d, gpio_1t2 is %d, gpio_2t1 is %d, gpio_2t2 is %d\n",
		gpio_status_1t1, gpio_status_1t2, gpio_status_2t1, gpio_status_2t2);

	pr_info("batt_sys: qcom_orient_en = %d, fusb_orient_en = %d, usb1_otg_en = %d, usb2_otg_en = %d, usb_boot_check = %d\n",
				qcom_orient_en, fusb_orient_en, usb1_otg_en, usb2_otg_en, usb_boot_check);

	if (!qcom_orient_en && !fusb_orient_en ){
		chg->ports_insert_value = TYPEC_PORT_VALUE_UNKNOWN;
	}
	else if (qcom_orient_en && !fusb_orient_en){
		chg->ports_insert_value = TYPEC_PORT_VALUE_FIR;
	}
	else if (!qcom_orient_en && fusb_orient_en){
		chg->ports_insert_value = TYPEC_PORT_VALUE_SEC;
		rc = smblib_write(chg, USBIN_ADAPTER_ALLOW_OVERRIDE_REG,
				CONTINUOUS);
		if (rc < 0)
			pr_err("write 0x%02x to USBIN_ADAPTER_ALLOW_OVERRIDE_REG fail for sec port rc=%d\n", rc);
	}
	else{
		chg->ports_insert_value = TYPEC_PORT_VALUE_BOTH;
	}

	insert_val_changed = port_status_changed_enable(chg->ports_insert_value);

	if ((!usb_boot_check) && insert_val_changed && is_usb_first_port(chg)) {
		insert_val_changed = 0;
		usb_boot_check = 1;
	}
	//pr_info("insert_val_changed = %d\n", insert_val_changed);

	if(usb1_otg_en==0&&usb2_otg_en==0){
	lenovo_set_gpios_ctrl(chg, insert_val_changed);
	}else {
	    pr_info("enable_otg_charge\n");
           enable_otg_charge(chg);
	}
	//schedule_work(&chg->bms_update_work);
	schedule_delayed_work(&chg->lenovo_monitor_ports_status_work,
				msecs_to_jiffies(LENOVO_WORK_MS));
}

int smblib_get_master_battery_status(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->master_fg_psy) {
		chg->master_fg_psy = power_supply_get_by_name("bq27z561-master");
		if (!chg->master_fg_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->master_fg_psy,
				POWER_SUPPLY_PROP_STATUS,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get master battery POWER_SUPPLY_PROP_STATUS, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_slave_battery_status(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->slave_fg_psy) {
		chg->slave_fg_psy = power_supply_get_by_name("bq27z561-slave");
		if (!chg->slave_fg_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->slave_fg_psy,
				POWER_SUPPLY_PROP_STATUS,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get slave battery POWER_SUPPLY_PROP_STATUS, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_master_battery_temp(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->master_fg_psy) {
		chg->master_fg_psy = power_supply_get_by_name("bq27z561-master");
		if (!chg->master_fg_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->master_fg_psy,
				POWER_SUPPLY_PROP_TEMP,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get master battery POWER_SUPPLY_PROP_TEMP, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_slave_battery_temp(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->slave_fg_psy) {
		chg->slave_fg_psy = power_supply_get_by_name("bq27z561-slave");
		if (!chg->slave_fg_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->slave_fg_psy,
				POWER_SUPPLY_PROP_TEMP,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get slave battery POWER_SUPPLY_PROP_TEMP, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_master_battery_current_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->master_fg_psy) {
		chg->master_fg_psy = power_supply_get_by_name("bq27z561-master");
		if (!chg->master_fg_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->master_fg_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get master battery POWER_SUPPLY_PROP_CURRENT_NOW, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_slave_battery_current_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->slave_fg_psy) {
		chg->slave_fg_psy = power_supply_get_by_name("bq27z561-slave");
		if (!chg->slave_fg_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->slave_fg_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get slave battery POWER_SUPPLY_PROP_CURRENT_NOW, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_master_battery_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->master_fg_psy) {
		chg->master_fg_psy = power_supply_get_by_name("bq27z561-master");
		if (!chg->master_fg_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->master_fg_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get master battery POWER_SUPPLY_PROP_VOLTAGE_NOW, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_slave_battery_voltage_now(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->slave_fg_psy) {
		chg->slave_fg_psy = power_supply_get_by_name("bq27z561-slave");
		if (!chg->slave_fg_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->slave_fg_psy,
				POWER_SUPPLY_PROP_VOLTAGE_NOW,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get slave battery POWER_SUPPLY_PROP_VOLTAGE_NOW, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_first_real_type(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->usb_psy) {
		chg->usb_psy = power_supply_get_by_name("usb");
		if (!chg->usb_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->usb_psy,
				POWER_SUPPLY_PROP_REAL_TYPE,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get first charger POWER_SUPPLY_PROP_REAL_TYPE, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_second_real_type(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->fusb302_usb_charge_psy) {
		chg->fusb302_usb_charge_psy = power_supply_get_by_name("bq2589h-charger");
		if (!chg->fusb302_usb_charge_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->fusb302_usb_charge_psy,
				POWER_SUPPLY_PROP_REAL_TYPE,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get second charger POWER_SUPPLY_PROP_REAL_TYPE, rc=%d\n",
				rc);

	return rc;
}

int smblib_get_second_usb_voltage(struct smb_charger *chg,
				    union power_supply_propval *val)
{
	int rc;

	if (!chg->fusb302_usb_charge_psy) {
		chg->fusb302_usb_charge_psy = power_supply_get_by_name("bq2589h-charger");
		if (!chg->fusb302_usb_charge_psy)
			return -ENODEV;
	}

	rc = power_supply_get_property(chg->fusb302_usb_charge_psy,
				POWER_SUPPLY_PROP_TI_CHARGER_BUS_VOLTAGE,
				val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get second charger POWER_SUPPLY_PROP_TI_CHARGER_BUS_VOLTAGE, rc=%d\n",
				rc);

	return rc;
}

static int get_bq2597x_1_charging_enabled(struct smb_charger *chg)
{
	int rc = -EINVAL;
	union power_supply_propval val = {0, };

	if (!chg->bq2597x_1_psy) {
		chg->bq2597x_1_psy = power_supply_get_by_name("bq2597x-master");
		if (!chg->bq2597x_1_psy)
			return 0;
	}

	rc = power_supply_get_property(chg->bq2597x_1_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED,
				&val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get bq2597x_1_psy POWER_SUPPLY_PROP_CHARGING_ENABLED, rc=%d\n",
				rc);
		return 0;
	}

	return val.intval;
}

static int get_bq2597x_2_charging_enabled(struct smb_charger *chg)
{
	int rc = -EINVAL;
	union power_supply_propval val = {0, };

	if (!chg->bq2597x_2_psy) {
		chg->bq2597x_2_psy = power_supply_get_by_name("bq2597x-slave");
		if (!chg->bq2597x_2_psy)
			return 0;
	}

	rc = power_supply_get_property(chg->bq2597x_2_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED,
				&val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get bq2597x_2_psy POWER_SUPPLY_PROP_CHARGING_ENABLED, rc=%d\n",
				rc);
		return 0;
	}

	return val.intval;
}

static int get_bq2597x_3_charging_enabled(struct smb_charger *chg)
{
	int rc = -EINVAL;
	union power_supply_propval val = {0, };

	if (!chg->bq2597x_3_psy) {
		chg->bq2597x_3_psy = power_supply_get_by_name("bq2597x-sec-master");
		if (!chg->bq2597x_3_psy)
			return 0;
	}

	rc = power_supply_get_property(chg->bq2597x_3_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED,
				&val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get bq2597x_3_psy POWER_SUPPLY_PROP_CHARGING_ENABLED, rc=%d\n",
				rc);
		return 0;
	}

	return val.intval;
}

static int get_bq2597x_4_charging_enabled(struct smb_charger *chg)
{
	int rc = -EINVAL;
	union power_supply_propval val = {0, };

	if (!chg->bq2597x_4_psy) {
		chg->bq2597x_4_psy = power_supply_get_by_name("bq2597x-sec-slave");
		if (!chg->bq2597x_4_psy)
			return 0;
	}

	rc = power_supply_get_property(chg->bq2597x_4_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED,
				&val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't get bq2597x_4_psy POWER_SUPPLY_PROP_CHARGING_ENABLED, rc=%d\n",
				rc);
		return 0;
	}

	return val.intval;
}

static void lenovo_battery_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			lenovo_battery_monitor_work.work);
	int rc = 0;
	u8 event_ready = 1;
	union power_supply_propval val = {0,};
	u8 stat;
	int value;

	//mutex_lock(&chg->battery_monitor_lock);

	rc = smblib_read(chg, TYPE_C_SNK_STATUS_REG, &stat);
	value = (int)stat;

	if (last_snk_stat != value) {
		plug_in_count = 1;
		plug_out_count = 1;
	} else if (value) {
		if (plug_in_count >= QCOM_PLUG_IN_CHECK_NUM) {
			qcom_orient_en = true;
			plug_in_count = 1;
		} else
			plug_in_count++;
	} else if (!value) {
		if (plug_out_count >= QCOM_PLUG_OUT_CHECK_NUM) {
			qcom_orient_en = false;
			plug_out_count = 1;
		} else
			plug_out_count++;
	}
	last_snk_stat = value;
	//qcom_orient_en |= first_typec_attached;

	rc = smblib_get_master_battery_status(chg, &val);
	if (rc < 0)
		batt1_status = last_batt1_status;
	else
		batt1_status = val.intval;

	rc = smblib_get_slave_battery_status(chg, &val);
	if (rc < 0)
		batt2_status = last_batt2_status;
	else
		batt2_status = val.intval;

	rc = smblib_get_first_real_type(chg, &val);
	if (rc < 0)
		charger1_type = last_charger1_type;
	else
		charger1_type = val.intval;

	rc = smblib_get_second_real_type(chg, &val);
	if (rc < 0)
		charger2_type = last_charger2_type;
	else
		charger2_type = val.intval;

	if (get_bq2597x_1_charging_enabled(chg))
		g_chg_en_dev |= 1 << CHG_EN_CP1;
	else
		g_chg_en_dev &= ~(1 << CHG_EN_CP1);

	if (get_bq2597x_2_charging_enabled(chg))
		g_chg_en_dev |= 1 << CHG_EN_CP2;
	else
		g_chg_en_dev &= ~(1 << CHG_EN_CP2);

	if (get_bq2597x_3_charging_enabled(chg))
		g_chg_en_dev |= 1 << CHG_EN_CP3;
	else
		g_chg_en_dev &= ~(1 << CHG_EN_CP3);

	if (get_bq2597x_4_charging_enabled(chg))
		g_chg_en_dev |= 1 << CHG_EN_CP4;
	else
		g_chg_en_dev &= ~(1 << CHG_EN_CP4);

	rc = smblib_get_usb_prop_typec_mode(chg, &val);
	if (rc < 0)
		usb1_audio_en = last_usb1_audio_en;
	else
		usb1_audio_en = (POWER_SUPPLY_TYPEC_SINK_AUDIO_ADAPTER == val.intval) ? 1:0;

	usb2_audio_en = audio_switch_state;

	if (((last_usb1_audio_en != usb1_audio_en) && (!usb1_audio_en)) ||
			((last_usb1_otg_en != usb1_otg_en) && (!usb1_otg_en)))
		qcom_orient_en = 0;

	qcom_charger_online = qcom_orient_en & (!usb1_otg_en) & (!usb1_audio_en);

#if 0
	pr_info("batt_sys: stat %d:%d, on1 %d:%d, on2 %d:%d, type1: %d:%d, type2: %d:%d, otg1 %d:%d, otg2 %d:%d, en %d:%d, plug %d:%d\n",
			last_snk_stat, value, last_qcom_orient_en, qcom_orient_en, last_fusb_orient_en, fusb_orient_en,
			last_charger1_type, charger1_type, last_charger2_type, charger2_type,
			last_usb1_otg_en, usb1_otg_en, last_usb2_otg_en, usb2_otg_en,
			last_g_chg_en_dev, g_chg_en_dev, plug_in_count, plug_out_count);
#endif

	if (last_qcom_orient_en != qcom_orient_en) {
		last_qcom_orient_en = qcom_orient_en;
		charger_insert_remove_process(chg, qcom_orient_en);
	} else if (last_fusb_orient_en != fusb_orient_en)
		last_fusb_orient_en = fusb_orient_en;
	else if (last_batt1_status != batt1_status)
		last_batt1_status = batt1_status;
	else if (last_batt2_status != batt2_status)
		last_batt2_status = batt2_status;
	else if (last_usb1_otg_en != usb1_otg_en)
		last_usb1_otg_en = usb1_otg_en;
	else if (last_usb2_otg_en != usb2_otg_en)
		last_usb2_otg_en = usb2_otg_en;
	else if (last_charger1_type != charger1_type)
		last_charger1_type = charger1_type;
	else if (last_charger2_type != charger2_type)
		last_charger2_type = charger2_type;
	else if (last_g_chg_en_dev != g_chg_en_dev)
		last_g_chg_en_dev = g_chg_en_dev;
	else if (last_usb1_audio_en != usb1_audio_en)
		last_usb1_audio_en = usb1_audio_en;
	else if (last_usb2_audio_en != usb2_audio_en)
		last_usb2_audio_en = usb2_audio_en;
	else
		event_ready = 0;

	if (qcom_orient_en || fusb_orient_en || usb1_otg_en || usb2_otg_en)
		vote(chg->awake_votable, CHARGER_ATTACHED_WAKE_VOTER, true, 0);
	else
		vote(chg->awake_votable, CHARGER_ATTACHED_WAKE_VOTER, false, 0);

	if (event_ready) {
		pr_info("batt_sys: bat-monitor stat %d, on1 %d, on2 %d, chg1 %d, type1: %d, type2: %d, otg1 %d, otg2 %d, aud1 %d, aud2 %d, en %d, plug %d:%d\n",
				value, qcom_orient_en, fusb_orient_en, qcom_charger_online,
				charger1_type, charger2_type,
				usb1_otg_en, usb2_otg_en,
				usb1_audio_en, usb2_audio_en,
				g_chg_en_dev, plug_in_count, plug_out_count);

		if (chg->batt_psy)
			power_supply_changed(chg->batt_psy);
		monitor_count = 1;
	} else {
		if (monitor_count >= LENOVO_MONITOR_EVENT_CHECK_NUM) {
			if (chg->batt_psy)
				power_supply_changed(chg->batt_psy);
			monitor_count = 1;
		} else
			monitor_count++;
	}
	//mutex_unlock(&chg->battery_monitor_lock);

	schedule_delayed_work(&chg->lenovo_battery_monitor_work,
				msecs_to_jiffies(LENOVO_BATTERY_MONITOR_DEALY_MS));
}

int set_first_main_charge_prop(struct smb_charger *chg, int chg_icl, int chg_fcc, int chg_volt)
{
	int rc;
	union power_supply_propval val = {0, };

	if (!chg->usb_psy) {
		chg->usb_psy = power_supply_get_by_name("usb");
		if (!chg->usb_psy)
			return -ENODEV;
	}

	val.intval = chg_icl;
	rc = power_supply_set_property(chg->usb_psy,
				POWER_SUPPLY_PROP_FORCE_FIRST_MAIN_ICL,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set first charger POWER_SUPPLY_PROP_FORCE_FIRST_MAIN_ICL, rc=%d\n",
			rc);

	val.intval = chg_fcc;
	rc = power_supply_set_property(chg->usb_psy,
				POWER_SUPPLY_PROP_FORCE_FIRST_MAIN_FCC,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set first charger POWER_SUPPLY_PROP_FORCE_FIRST_MAIN_FCC, rc=%d\n",
			rc);


	val.intval = chg_volt;
	rc = power_supply_set_property(chg->usb_psy,
				POWER_SUPPLY_PROP_FORCE_FIRST_MAIN_FV,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set first charger POWER_SUPPLY_PROP_FORCE_FIRST_MAIN_FV, rc=%d\n",
			rc);

	return rc;
}

int set_sec_main_charge_prop(struct smb_charger *chg, int chg_icl, int chg_fcc, int chg_volt)
{
	int rc;

	rc = bq2589x_dynamic_update_charging_profile(chg_volt, chg_fcc, chg_icl, SEC_CHARGE_VBUS);

	return rc;
}

int set_first_main_charge_enable(struct smb_charger *chg, int enable)
{
	int rc;
	union power_supply_propval val = {0, };

	if (!chg->batt_psy) {
		chg->batt_psy = power_supply_get_by_name("battery");
		if (!chg->batt_psy)
			return -ENODEV;
	}

	val.intval = !!enable;
	rc = power_supply_set_property(chg->batt_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set first charge POWER_SUPPLY_PROP_CHARGING_ENABLED, rc=%d\n",
			rc);

	return rc;
}

int set_first_main_usb_input_suspend(struct smb_charger *chg, int suspend)
{
	int rc;
	union power_supply_propval val = {0, };

	if (!chg->batt_psy) {
		chg->batt_psy = power_supply_get_by_name("battery");
		if (!chg->batt_psy)
			return -ENODEV;
	}

	val.intval = !!suspend;
	rc = power_supply_set_property(chg->batt_psy,
				POWER_SUPPLY_PROP_INPUT_SUSPEND,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't set first charge POWER_SUPPLY_PROP_INPUT_SUSPEND, rc=%d\n",
			rc);

	return rc;
}

int set_sec_main_charge_enable(struct smb_charger *chg, int enable)
{
	int rc;
	union power_supply_propval val = {0, };

	if (!chg->fusb302_usb_charge_psy) {
		chg->fusb302_usb_charge_psy = power_supply_get_by_name("bq2589h-charger");
		if (!chg->fusb302_usb_charge_psy)
			return -ENODEV;
	}

	val.intval = !!enable;
	rc = power_supply_set_property(chg->fusb302_usb_charge_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get second charge POWER_SUPPLY_PROP_CHARGING_ENABLED, rc=%d\n",
				rc);

	return rc;
}

int set_sec_main_usb_input_suspend(struct smb_charger *chg, int suspend)
{
	int rc;
	union power_supply_propval val = {0, };

	if (!chg->fusb302_usb_charge_psy) {
		chg->fusb302_usb_charge_psy = power_supply_get_by_name("bq2589h-charger");
		if (!chg->fusb302_usb_charge_psy)
			return -ENODEV;
	}

	val.intval = !!suspend;
	rc = power_supply_set_property(chg->fusb302_usb_charge_psy,
				POWER_SUPPLY_PROP_INPUT_SUSPEND,
				&val);
	if (rc < 0)
		dev_err(chg->dev, "Couldn't get second charge POWER_SUPPLY_PROP_INPUT_SUSPEND, rc=%d\n",
				rc);

	return rc;
}

enum {
	C_UNKNOWN = 0,
	C_NONE,
	C_USB,
	C_DCP,
	C_PD,
};

//#define FORCE_SINGLE_PD_TO_DCP
//#define FORCE_SINGLE_PD_SMALL_POWER

#define PORT_1_DCP_ICL_MAX				3000000
#define PORT_1_DCP_FCC_MAX				2500000

#define PORT_2_DCP_ICL_MAX				3000000
#define PORT_2_DCP_FCC_MAX            			2500000

#define PORT_2_PD_FIR_MAIN_ICL_MAX			1800000
#define PORT_2_PD_FIR_MAIN_FCC_MAX			1800000

#define PORT_2_PD_FIR_MAIN_POWER_ICL_MAX		1800000
#define PORT_2_PD_FIR_MAIN_POWER_FCC_MAX		2500000

#define PORT_1_PD_SEC_MAIN_ICL_MAX			2000000
#define PORT_1_PD_SEC_MAIN_FCC_MAX			2000000

#define PORT_1_PD_SEC_MAIN_POWER_ICL_MAX		1800000
#define PORT_1_PD_SEC_MAIN_POWER_FCC_MAX		2500000

#define DUAL_DCP_ICL_MAX				2800000
#define DUAL_DCP_FCC_MAX				5000000

#define PORT_DCP_ICL_DEFAULT				2000000
#define PORT_DCP_FCC_DEFAULT				2000000

#define USB_CURRENT_MAX					500000
#define PORT_CURRENT_100MA				100000
#define PORT_CURRENT_150MA				150000
#define PORT_CURRENT_350MA				350000

#define ICL_DISABLE					-1
#define ICL_UNKNOWN					-2
#define FCC_DISABLE					-1
#define FCC_UNKNOWN					-2

#define DEFAULT_USB_VOLTAGE				5000000
#define DEFAULT_PD_VOLTAGE				8000000

#define HVDCP_VBUS_LEVEL_0				DEFAULT_USB_VOLTAGE
#define HVDCP_VBUS_LEVEL_1				6000000
#define HVDCP_VBUS_LEVEL_2				7000000
#define HVDCP_VBUS_LEVEL_3				8000000
#define HVDCP_VBUS_LEVEL_4				9000000

struct dual_charge_map {
	int port1_status;
	int port2_status;
	int port1_def_icl;
	int port1_def_fcc;
	int port2_def_icl;
	int port2_def_fcc;
};

static const struct dual_charge_map dual_charge_map[] = {
	{ C_UNKNOWN , C_UNKNOWN , ICL_DISABLE        , FCC_DISABLE        , PORT_CURRENT_100MA , PORT_CURRENT_100MA },
	{ C_NONE    , C_NONE    , ICL_DISABLE        , FCC_DISABLE        , PORT_CURRENT_100MA , PORT_CURRENT_100MA },
	{ C_USB     , C_USB     , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    },
	{ C_USB     , C_DCP     , USB_CURRENT_MAX    , USB_CURRENT_MAX    , PORT_2_DCP_ICL_MAX , PORT_2_DCP_FCC_MAX },
	{ C_USB     , C_PD      , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    },
	{ C_DCP     , C_USB     , PORT_1_DCP_ICL_MAX , PORT_1_DCP_FCC_MAX , USB_CURRENT_MAX    , USB_CURRENT_MAX    },
	{ C_DCP     , C_DCP     , PORT_1_DCP_ICL_MAX , PORT_1_DCP_FCC_MAX , PORT_2_DCP_ICL_MAX , PORT_2_DCP_FCC_MAX },
	{ C_DCP     , C_PD      , PORT_1_DCP_ICL_MAX , PORT_1_DCP_FCC_MAX , USB_CURRENT_MAX    , USB_CURRENT_MAX    },
	{ C_PD      , C_USB     , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    },
	{ C_PD      , C_DCP     , USB_CURRENT_MAX    , USB_CURRENT_MAX    , PORT_2_DCP_ICL_MAX , PORT_2_DCP_FCC_MAX },
	{ C_PD      , C_PD      , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    , USB_CURRENT_MAX    },
	{ C_USB     , C_NONE    , PORT_CURRENT_150MA , PORT_CURRENT_150MA , PORT_CURRENT_350MA , PORT_CURRENT_350MA },
	{ C_DCP     , C_NONE    , ICL_UNKNOWN        , FCC_UNKNOWN        , ICL_UNKNOWN        , FCC_UNKNOWN        },
	{ C_PD      , C_NONE    , USB_CURRENT_MAX    , USB_CURRENT_MAX    , PORT_1_PD_SEC_MAIN_ICL_MAX , PORT_1_PD_SEC_MAIN_FCC_MAX },
	{ C_NONE    , C_USB     , PORT_CURRENT_150MA , PORT_CURRENT_150MA , PORT_CURRENT_350MA , PORT_CURRENT_350MA },
	{ C_NONE    , C_DCP     , ICL_UNKNOWN        , FCC_UNKNOWN        , ICL_UNKNOWN        , FCC_UNKNOWN        },
	{ C_NONE    , C_PD      , PORT_2_PD_FIR_MAIN_ICL_MAX , PORT_2_PD_FIR_MAIN_FCC_MAX , USB_CURRENT_MAX , USB_CURRENT_MAX },
};

struct chg_ffc_table {
	int temp_low;
	int temp_high;
	int chg_volt1;
	int chg_volt2;
	int cutoff_volt;
	int chg_curr;
	int cutoff_curr;
};

static const struct  chg_ffc_table ffc1[] = {
	{   10 , 100 , 2500000, 4400000, 4400000, 1500000, -120000  },
	{  100 , 150 , 2500000, 4400000, 4400000, 3000000, -120000  },
	{  150 , 300 , 2500000, 4470000, 4470000, 3000000, -900000  },
	{  300 , 470 , 2500000, 4470000, 4470000, 3000000, -1510000 },
	{  470 , 510 , 2500000, 4200000, 4200000, 2500000, -120000  },
};
static const struct  chg_ffc_table ffc2[] = {
	{  -10 , 100 , 2500000, 4400000, 4400000, 1500000, -120000  },
	{  100 , 150 , 2500000, 4400000, 4400000, 3000000, -120000  },
	{  150 , 300 , 2500000, 4470000, 4470000, 3000000, -900000  },
	{  300 , 470 , 2500000, 4470000, 4470000, 3000000, -1510000 },
	{  470 , 510 , 2500000, 4200000, 4200000, 2500000, -120000  },
};

static int set_main_charge_prop(struct smb_charger *chg, int fcc1, int icl1, int volt1, int fcc2, int icl2, int volt2)
{
	int rc = 0;

	pr_info("batt_sys: set main prop, [first] fcc=%d icl=%d volt=%d, [sec] curr=%d icl=%d volt=%d\n",
							fcc1, icl1, volt1, fcc2, icl2, volt2);

	if (-1 == icl1) {
		pr_info("batt_sys: first main process [IBUS] [STOP]\n");
		set_first_main_charge_prop(chg, 0, 0, volt1);
		set_first_main_charge_enable(chg, 0);
		set_first_main_usb_input_suspend(chg, 1);
	} else if (-1 == fcc1) {
		pr_info("batt_sys: first main process [CHARGE] [STOP]\n");
		set_first_main_charge_prop(chg, icl1, 0, volt1);
		set_first_main_charge_enable(chg, 0);
		set_first_main_usb_input_suspend(chg, 0);
	} else {
		pr_info("batt_sys: first main process [CHARGE] [RUNNING]\n");
		set_first_main_charge_prop(chg, icl1, fcc1, volt1);
		set_first_main_charge_enable(chg, 1);
		set_first_main_usb_input_suspend(chg, 0);
	}

	if (-1 == icl2) {
		pr_info("batt_sys: second main process [IBUS] [STOP]\n");
		set_sec_main_charge_prop(chg, 0, 100000, volt2);
		set_sec_main_charge_enable(chg, 0);
		set_sec_main_usb_input_suspend(chg, 1);
	} else if (-1 == fcc2) {
		pr_info("batt_sys: second main process [CHARGE] [STOP]\n");
		set_sec_main_charge_prop(chg, icl2, 100000, volt2);
		set_sec_main_charge_enable(chg, 0);
		set_sec_main_usb_input_suspend(chg, 0);
	} else {
		pr_info("batt_sys: second main process [CHARGE] [RUNNING]\n");
		set_sec_main_charge_prop(chg, icl2, fcc2, volt2);
		set_sec_main_charge_enable(chg, 1);
		set_sec_main_usb_input_suspend(chg, 0);
	}

	return rc;
}

static void set_thermal_engine_battery_policy(struct smb_charger *chg, int data)
{
	union power_supply_propval val = {0,};

	pr_info("batt_sys: Thermal-engine battery policy %d\n", data);
	val.intval = data;
	smblib_set_prop_charge_thermal_status(chg, &val);
}

static int get_battery_soc_delta_level(int soc1, int soc2, int *d1, int *d2)
{
	int data1 = 50, data2 = 50;

	if ((abs(soc1 - soc2)) <= 20) {
		data1 = 50;
		data2 = 50;
	} else if ((soc1 - soc2) > 20) {
		data1 = 30;
		data2 = 70;
	} else if ((soc2 - soc1) > 20) {
		data1 = 70;
		data2 = 30;
	}

	*d1 = data1;
	*d2 = data2;

	return 0;
}

static get_port_vbus(int volt)
{
	int vbus_uv;

	if (volt <= HVDCP_VBUS_LEVEL_1)
		vbus_uv= HVDCP_VBUS_LEVEL_0;
	else if (volt <= HVDCP_VBUS_LEVEL_2)
		vbus_uv = HVDCP_VBUS_LEVEL_1;
	else if (volt <= HVDCP_VBUS_LEVEL_3)
		vbus_uv = HVDCP_VBUS_LEVEL_2;
	else if (volt <= HVDCP_VBUS_LEVEL_4)
		vbus_uv = HVDCP_VBUS_LEVEL_3;
	else
		vbus_uv = HVDCP_VBUS_LEVEL_4;

	return vbus_uv;
}

static int set_dual_charge_profile(struct smb_charger *chg, int status1, int status2, int pd_enabled)
{
	int rc = 0;
	u8 ready = 1;
	union power_supply_propval val = {0,};
	int i, j;
	int d1, d2;
	int batt_temp;
	int batt_curr;
	int batt_volt;
	int tmp_volt;
	int tmp_curr;
	int tmp_cutcurr;
	int max_power1 = get_charger_max_power();
	int max_power2 = max(get_charger2_max_power(), get_charger2a_max_power());
	int therm_status = PD_THERM_UNKNOWN;
	int pd1_running = ffc_pd1_get_flash_chg_ready();
	int pd2_running = ffc_pd2_get_flash_chg_ready();
	u8 max_jeita_num1 = ARRAY_SIZE(ffc1) - 1;
	u8 max_jeita_num2 = ARRAY_SIZE(ffc2) - 1;
	int vbus1, vbus2, vbus1_uv, vbus2_uv;
	int total_icl = -1, total_fcc = -1;
	int icl1, icl2, fcc1, fcc2;
	int port1_icl, port2_icl, port1_fcc, port2_fcc;
	int dual_icl;

	get_battery_soc_delta_level(last_batt1_soc, last_batt2_soc, &d1, &d2);

	rc = smblib_get_prop_usb_voltage_now(chg, &val);
	if ((rc < 0) || (val.intval < 4200000))
		vbus1 = DEFAULT_USB_VOLTAGE;
	else
		vbus1 = val.intval;

	rc = smblib_get_second_usb_voltage(chg, &val);
	if ((rc < 0) || (val.intval < 4200))
		vbus2 = DEFAULT_USB_VOLTAGE;
	else
		vbus2 = val.intval * 1000;

	vbus1_uv = get_port_vbus(vbus1);
	vbus2_uv = get_port_vbus(vbus2);
	pr_info("batt_sys: vbus1 %d vbus2 %d vbus1_uv %d vbus2_uv %d\n",
			vbus1, vbus2, vbus1_uv, vbus2_uv);

	/*************************************************************
	* BUS power
	*************************************************************/
	if (max_power1 > 15) {
		chg->port1_bus_5v_icl_ua = get_charger1_5v_obj_current() * 1000;
		chg->port1_bus_9v_icl_ua = get_charger1_9v_obj_current() * 1000;
	} else {
		chg->port1_bus_5v_icl_ua = PORT_DCP_ICL_DEFAULT;
		chg->port1_bus_9v_icl_ua = PORT_DCP_ICL_DEFAULT;
	}

	if (max_power2 > 15) {
		chg->port2_bus_5v_icl_ua = max(get_charger2_5v_obj_current() * 1000, get_charger2a_5v_obj_current() * 1000);
		chg->port2_bus_9v_icl_ua = max(get_charger2_9v_obj_current() * 1000, get_charger2a_9v_obj_current() * 1000);
	} else {
		chg->port2_bus_5v_icl_ua = PORT_DCP_ICL_DEFAULT;
		chg->port2_bus_9v_icl_ua = PORT_DCP_ICL_DEFAULT;
	}

	pr_info("batt_sys: bus1 max power %d, 5v %d, 9v %d\n",
			max_power1,
			get_charger1_5v_obj_current() * 1000, get_charger1_9v_obj_current() * 1000);
	pr_info("batt_sys: bus2 max power %d, 5v %d (%d:%d), 9v %d (%d:%d)\n",
			max_power2,
			chg->port2_bus_5v_icl_ua, get_charger2_5v_obj_current() * 1000, get_charger2a_5v_obj_current() * 1000,
			chg->port2_bus_9v_icl_ua, get_charger2_9v_obj_current() * 1000, get_charger2a_9v_obj_current()) * 1000;

	/*************************************************************
	* FCC Default config
	*************************************************************/
	for (i = 0; i < ARRAY_SIZE(dual_charge_map); i++) {
		if ((status1 == dual_charge_map[i].port1_status) &&
				(status2 == dual_charge_map[i].port2_status)) {
			icl1 = dual_charge_map[i].port1_def_icl;
			fcc1 = dual_charge_map[i].port1_def_fcc;
			icl2 = dual_charge_map[i].port2_def_icl;
			fcc2 = dual_charge_map[i].port2_def_fcc;
			break;
		}
	}

#ifdef FORCE_SINGLE_PD_SMALL_POWER
	if ((status1 == C_PD) && (status2 == C_NONE)) {
		if ((g_chg_en_dev & 1 << CHG_EN_CP1) ||
				(g_chg_en_dev & 1 << CHG_EN_CP2)) {
			icl2 = PORT_2_PD_FIR_MAIN_POWER_ICL_MAX;
			fcc2 = PORT_2_PD_FIR_MAIN_POWER_FCC_MAX;
			pr_info("batt_sys: pd1 only, config second main charger icl %d fcc %d\n", icl2, fcc2);
		}
	}

	if ((status2 == C_PD) && (status1 == C_NONE)) {
		if ((g_chg_en_dev & 1 << CHG_EN_CP3) ||
				(g_chg_en_dev & 1 << CHG_EN_CP4)) {
			icl1 = PORT_1_PD_SEC_MAIN_POWER_ICL_MAX;
			fcc1 = PORT_1_PD_SEC_MAIN_POWER_FCC_MAX;
			pr_info("batt_sys: pd2 only, config first main charger icl %d fcc %d\n", icl1, fcc1);
		}
	}
#endif
	chg->first_main_icl_ua = icl1;
	chg->first_main_fcc_ua = fcc1;
	chg->sec_main_icl_ua = icl2;
	chg->sec_main_fcc_ua = fcc2;
	pr_info("batt_sys: charge [config] icl:fcc  (1) %d:%d  (2) %d:%d\n",
			icl1, fcc1, icl2, fcc2);

	if ((ICL_UNKNOWN == icl1) && (ICL_UNKNOWN == icl2)) {
		if (DEFAULT_USB_VOLTAGE == vbus1_uv)
			dual_icl = max(chg->port1_bus_5v_icl_ua, chg->port2_bus_5v_icl_ua);
		else
			dual_icl = max(chg->port1_bus_9v_icl_ua, chg->port2_bus_9v_icl_ua);
		chg->total_main_icl_ua = min(DUAL_DCP_ICL_MAX, dual_icl);
		chg->total_main_fcc_ua = DUAL_DCP_FCC_MAX;
	} else {
		chg->total_main_icl_ua = ICL_UNKNOWN;
		chg->total_main_fcc_ua = FCC_UNKNOWN;
		if (DEFAULT_USB_VOLTAGE == vbus1_uv)
			chg->first_main_icl_ua = min(chg->port1_bus_5v_icl_ua, chg->first_main_icl_ua);
		else
			chg->first_main_icl_ua = min(chg->port1_bus_9v_icl_ua, chg->first_main_icl_ua);
		if (DEFAULT_USB_VOLTAGE == vbus2_uv)
			chg->sec_main_icl_ua = min(chg->port2_bus_5v_icl_ua, chg->sec_main_icl_ua);
		else
			chg->sec_main_icl_ua = min(chg->port2_bus_9v_icl_ua, chg->sec_main_icl_ua);
	}

	pr_info("batt_sys: charge [default] icl:fcc  (1) %d:%d  (2) %d:%d  (total) %d:%d\n",
			chg->first_main_icl_ua, chg->first_main_fcc_ua,
			chg->sec_main_icl_ua, chg->sec_main_fcc_ua,
			chg->total_main_icl_ua, chg->total_main_fcc_ua);

	/*************************************************************
	* Get Max ICL
	*************************************************************/
#if 0
	//if ((status1 != C_PD) && (status2 != C_PD)) {
	//if ((!pd1_running) && (!pd2_running)) {
	if ((-1 == curr1) && (-1 == curr2)) {
		if (0 == chg->icl_settled_ready) {
			pr_info("batt_sys: BEGIN ICL SETTLED\n");
			chg->icl_settled_ready = 1;
			curr1 = DUAL_DCP_ICL_MAX;
			chg->first_request_icl_ua = curr1 * (DEFAULT_USB_VOLTAGE / 1000000) / (vbus_uv / 1000000) / 100;
			curr2 = 100000;

			schedule_delayed_work(&chg->lenovo_icl_settled_work,
					msecs_to_jiffies(50000));
		} else if (1 == chg->icl_settled_ready) {
			pr_info("batt_sys: WAIT FOR ICL SETTLED\n");
			return 0;
		} else if (2 == chg->icl_settled_ready) {
/*
			curr1 = chg->sin_port_max_power * d1 / (DEFAULT_USB_VOLTAGE / 1000000) / 100;
			curr2 = chg->sin_port_max_power * d2 / (DEFAULT_USB_VOLTAGE / 1000000) / 100;
			chg->first_request_icl_ua = chg->sin_port_max_power * d1 / (vbus_uv / 1000000) / 100;
			chg->sec_request_icl_ua = chg->sin_port_max_power * d2 / (vbus_uv / 1000000) / 100;
*/
/*
			curr1 = PORT_1_DCP_FCC_MAX * / 100;
			curr2 = PORT_2_DCP_FCC_MAX * / 100;
			chg->first_request_icl_ua = PORT_1_DCP_FCC_MAX * (DEFAULT_USB_VOLTAGE / 1000000) / (vbus_uv / 1000000) / 100;
			chg->sec_request_icl_ua = PORT_2_DCP_FCC_MAX * (DEFAULT_USB_VOLTAGE / 1000000) / (vbus_uv / 1000000) / 100;
*/
			curr1 = DUAL_DCP_ICL_MAX / 2;
			curr2 = DUAL_DCP_ICL_MAX / 2;
			chg->first_request_icl_ua = curr1;
			chg->sec_request_icl_ua = curr2;
		}
	} else {
		if (1 == chg->icl_settled_ready)
			cancel_delayed_work_sync(&chg->lenovo_icl_settled_work);
		chg->icl_settled_ready = 0;
		chg->max_icl_settled_ua = 0;
		chg->usb_vbus_uv = 0;
		chg->sin_port_max_power = 0;
	}
#endif
	chg->icl_settled_ready = 0;

	/*************************************************************
	* PD Thermal
	*************************************************************/
	if (lenovo_get_sensor_temp("skin-msm-therm-usr", &skin_temp) < 0) {
		skin_temp = 0;
		pr_info("batt_sys: can not get skin_temp %d\n", skin_temp);
	}
	pr_info("batt_sys: skin_temp %d, power1 %d, power2 %d, d1 %d, d2 %d\n",
			skin_temp, max_power1, max_power2, d1, d2);

	if ((max_power1 > 15) || (max_power2 > 15)) {
		if ((max_power1 > 15) && (max_power2 > 15)) {
			if ((skin_temp >= DUAL_PD_THERM_LOW_TEMP) && (skin_temp <= DUAL_PD_THERM_HIGH_TEMP))
				therm_status = PD_THERM_WARM;
			else if (skin_temp > DUAL_PD_THERM_HIGH_TEMP)
				therm_status = PD_THERM_HOT;
			else
				therm_status = PD_THERM_NORMAL;
		} else {
			if ((skin_temp >= SINGLE_PD_THERM_LOW_TEMP) && (skin_temp <= SINGLE_PD_THERM_HIGH_TEMP))
				therm_status = PD_THERM_WARM;
			else if (skin_temp > SINGLE_PD_THERM_HIGH_TEMP)
				therm_status = PD_THERM_HOT;
			else
				therm_status = PD_THERM_NORMAL;
		}

#ifdef FORCE_SINGLE_PD_TO_DCP
		if (!pd_enabled) {
			therm_status = PD_SINGLE_PORT_DISABLED;
			pr_info("batt_sys: disable pd for single port\n");
		}
#endif
#ifdef FORCE_SINGLE_PD_SMALL_POWER
		if (((status1 == C_PD) && (status2 == C_NONE)) ||
				((status2 == C_PD) && (status1 == C_NONE))) {
			therm_status = PD_SMALL_POWER;
			pr_info("batt_sys: reduce pd fcc for single port\n");
		}
#endif
		if (chg->charge_therm_fcc_ua != -1)
			therm_status = PD_THERM_PERF;

		switch (therm_status) {
		case PD_THERM_WARM:
		case PD_SMALL_POWER:
			thermal_ctrl_batt1_current = PD_THERM_WARM_FCC;
			thermal_ctrl_batt2_current = PD_THERM_WARM_FCC;
			is_batt1_thermal_stop = 0;
			is_batt2_thermal_stop = 0;

			if (0 == chg->restart_pd_status) {
				pr_info("batt_sys: thermal warm, restart pd work\n");
				chg->restart_pd_status = 1;
				schedule_delayed_work(&chg->lenovo_restart_pd_work,
						msecs_to_jiffies(30000));
				schedule_delayed_work(&chg->lenovo_charge_monitor_work,
						msecs_to_jiffies(0));
				return 0;
			}
			break;

		case PD_THERM_HOT:
		case PD_THERM_PERF:
		case PD_SINGLE_PORT_DISABLED:
			if (1 == chg->restart_pd_status)
				cancel_delayed_work_sync(&chg->lenovo_restart_pd_work);
			thermal_ctrl_batt1_current = 0;
			thermal_ctrl_batt2_current = 0;
			is_batt1_thermal_stop = 1;
			is_batt2_thermal_stop = 1;
			chg->restart_pd_status = 0;
			break;

		case PD_THERM_NORMAL:
			thermal_ctrl_batt1_current = 0;
			thermal_ctrl_batt2_current = 0;
			is_batt1_thermal_stop = 0;
			is_batt2_thermal_stop = 0;

			if (0 == chg->restart_pd_status) {
				pr_info("batt_sys: thermal normal, restart pd work\n");
				chg->restart_pd_status = 1;
				schedule_delayed_work(&chg->lenovo_restart_pd_work,
						msecs_to_jiffies(30000));
				schedule_delayed_work(&chg->lenovo_charge_monitor_work,
						msecs_to_jiffies(0));
				return 0;
			}
			break;

		default:
			break;
		}

		pr_info("batt_sys: therm_status %d, pd work %d, therm pd fcc %d:%d, pd stop %d:%d pd_en %d\n",
			therm_status, chg->restart_pd_status,
			thermal_ctrl_batt1_current, thermal_ctrl_batt2_current,
			is_batt1_thermal_stop, is_batt2_thermal_stop, pd_enabled);
	} else {
		if (1 == chg->restart_pd_status)
			cancel_delayed_work_sync(&chg->lenovo_restart_pd_work);
		thermal_ctrl_batt1_current = 0;
		thermal_ctrl_batt2_current = 0;
		is_batt1_thermal_stop = 0;
		is_batt2_thermal_stop = 0;
		chg->restart_pd_status = 0;
	}

	if ((pd1_running) || (pd2_running) || (1 == chg->restart_pd_status)) {
		pr_info("batt_sys: thermal-engine disable [PD running]\n");
		set_thermal_engine_battery_policy(chg, 0);
	} else {
		if ((2 == is_pd1_done) || (2 == is_pd2_done)) {
			if (chg->port1_charge_done && chg->port2_charge_done) {
				pr_info("batt_sys: thermal-engine enable [PD charge done]\n");
				set_thermal_engine_battery_policy(chg, 1);
			} else {
				pr_info("batt_sys: thermal-engine disable [PD to DCP]\n");
				set_thermal_engine_battery_policy(chg, 0);
			}
		} else {
			pr_info("batt_sys: thermal-engine enable [default NOT PD]\n");
			set_thermal_engine_battery_policy(chg, 1);
		}
	}

	/*************************************************************
	* System Thermal
	*************************************************************/

	if (1 != chg->icl_settled_ready) {
		if (chg->charge_therm_fcc_ua != -1)
			pr_info("batt_sys: charge [therm enable] total fcc %d\n", chg->charge_therm_fcc_ua);
		else
			pr_info("batt_sys: charge [therm disable] total fcc %d\n", chg->charge_therm_fcc_ua);
	} else
		pr_info("batt_sys: charge [therm unknown] total fcc %d\n", chg->charge_therm_fcc_ua);

	/*************************************************************
	* Battery Jeita
	*************************************************************/
	for (j = 0; j < 2; j++) {
		if (0 == j) {
			rc = smblib_get_master_battery_temp(chg, &val);
			batt_temp = (rc < 0) ? BATTERY_FAKE_TEMP : val.intval;
			rc = smblib_get_master_battery_current_now(chg, &val);
			batt_curr = (rc < 0) ? 0 : val.intval;
			rc = smblib_get_master_battery_voltage_now(chg, &val);
			batt_volt = (rc < 0) ? 0 : val.intval;
			pr_info("batt_sys: batt1 temp %d, volt %d, curr %d, soc %d, real_soc %d, full %d\n",
					batt_temp, batt_volt, batt_curr, last_batt1_soc, last_batt1_real_soc, chg->batt1_cap_full);

			if (batt_temp <= ffc1[0].temp_low) {
				pr_info("batt_sys: BATT 1 STATUS [COOL]\n");
				tmp_curr = FCC_DISABLE;
				tmp_volt = ffc1[0].cutoff_volt;
			} else if (batt_temp >= ffc1[max_jeita_num1].temp_high) {
				pr_info("batt_sys: BATT 1 STATUS [HOT]\n");
				tmp_curr = FCC_DISABLE;
				tmp_volt = ffc1[max_jeita_num1].cutoff_volt;
			} else {
				for (i = 0; i <= max_jeita_num1; i++) {
					if ((batt_temp > ffc1[i].temp_low) && (batt_temp <= ffc1[i].temp_high)) {
						tmp_curr = ffc1[i].chg_curr;
						tmp_volt = ffc1[i].cutoff_volt;
						tmp_cutcurr = ffc1[i].cutoff_curr;
						break;
					}
				}
			}

			chg->first_batt_max_fv_uv = tmp_volt;

			if (FCC_DISABLE == chg->first_main_fcc_ua) {
				chg->first_batt_max_fcc_ua = FCC_DISABLE;
				pr_info("batt_sys: charger1 [Unknown or None Status]\n");
			} else if (chg->batt1_cap_full) {
				if (last_batt1_real_soc <= 98) {
					chg->batt1_cap_full = 0;
					chg->first_batt_max_fcc_ua = tmp_curr;
					pr_info("batt_sys: batt1 [100] [RE-CHARGING]\n");
				} else {
					chg->first_batt_max_fcc_ua = FCC_DISABLE;
					pr_info("batt_sys: batt1 [100] [DISABLE CHARGE]\n");
				}
			} else if (100 == last_batt1_real_soc) {
				if (batt_curr >= tmp_cutcurr) {
					chg->batt1_cap_full = 1;
					chg->first_batt_max_fcc_ua = FCC_DISABLE;
					pr_info("batt_sys: batt1 [100] [FULL] [%d] [%d]\n", batt_curr, tmp_cutcurr);
				} else {
					chg->first_batt_max_fcc_ua = tmp_curr;
					pr_info("batt_sys: batt1 [100] [CHARGING] [%d] [%d]\n", batt_curr, tmp_cutcurr);
				}
			} else {
				chg->first_batt_max_fcc_ua = tmp_curr;
			}
		} else {
			rc = smblib_get_slave_battery_temp(chg, &val);
			batt_temp = (rc < 0) ? BATTERY_FAKE_TEMP : val.intval;
			rc = smblib_get_slave_battery_current_now(chg, &val);
			batt_curr = (rc < 0) ? 0 : val.intval;
			rc = smblib_get_slave_battery_voltage_now(chg, &val);
			batt_volt = (rc < 0) ? 0 : val.intval;
			pr_info("batt_sys: batt2 temp %d, volt %d, curr %d, soc %d, real_soc %d, full %d\n",
					batt_temp, batt_volt, batt_curr, last_batt2_soc, last_batt2_real_soc, chg->batt2_cap_full);

			if (batt_temp <= ffc2[0].temp_low) {
				pr_info("batt_sys: BATT 2 STATUS [COOL]\n");
				tmp_curr = FCC_DISABLE;
				tmp_volt = ffc2[0].cutoff_volt;
			} else if (batt_temp >= ffc2[max_jeita_num2].temp_high) {
				pr_info("batt_sys: BATT 2 STATUS [HOT]\n");
				tmp_curr = FCC_DISABLE;
				tmp_volt = ffc2[max_jeita_num2].cutoff_volt;
			} else {
				for (i = 0; i <= max_jeita_num2; i++) {
					if ((batt_temp > ffc2[i].temp_low) && (batt_temp <= ffc2[i].temp_high)) {
						tmp_curr = ffc2[i].chg_curr;
						tmp_volt = ffc2[i].cutoff_volt;
						tmp_cutcurr = ffc2[i].cutoff_curr;
						break;
					}
				}
			}

			chg->sec_batt_max_fv_uv = tmp_volt;

			if (FCC_DISABLE == chg->sec_main_fcc_ua) {
				chg->sec_batt_max_fcc_ua = FCC_DISABLE;
				pr_info("batt_sys: charger2 [Unknown or None Status]\n");
			} else if (chg->batt2_cap_full) {
				if (last_batt2_real_soc <= 98) {
					chg->batt2_cap_full = 0;
					chg->sec_batt_max_fcc_ua = tmp_curr;
					pr_info("batt_sys: batt2 [100] [RE-CHARGING]\n");
				} else {
					chg->sec_batt_max_fcc_ua = FCC_DISABLE;
					pr_info("batt_sys: batt2 [100] [DISABLE CHARGE]\n");
				}
			} else if (100 == last_batt2_real_soc) {
				if (batt_curr >= tmp_cutcurr) {
					chg->batt2_cap_full = 1;
					chg->sec_batt_max_fcc_ua = FCC_DISABLE;
					pr_info("batt_sys: batt2 [100] [FULL] [%d] [%d]\n", batt_curr, tmp_cutcurr);
				} else {
					chg->sec_batt_max_fcc_ua = tmp_curr;
					pr_info("batt_sys: batt2 [100] [CHARGING] [%d] [%d]\n", batt_curr, tmp_cutcurr);
				}
			} else {
				chg->sec_batt_max_fcc_ua = tmp_curr;
			}
		}
	}

	/*************************************************************
	* Request ICL and FCC
	*************************************************************/
#if 0
	if (1 == chg->icl_settled_ready) {
		chg->first_request_fcc_ua = chg->first_main_fcc_ua;
		chg->sec_request_fcc_ua = chg->sec_main_fcc_ua;;
		pr_info("batt_sys: aicl setting [1] fcc %d, [2] fcc %d\n",
				chg->first_request_fcc_ua, chg->sec_request_fcc_ua);
	}
#endif

	if ((ICL_UNKNOWN != chg->first_main_icl_ua) && (ICL_UNKNOWN != chg->sec_main_icl_ua)) {
		port1_icl = chg->first_main_icl_ua;
		port1_fcc = min(chg->first_main_fcc_ua, chg->first_batt_max_fcc_ua);
		port2_icl = chg->sec_main_icl_ua;
		port2_fcc = min(chg->sec_main_fcc_ua, chg->sec_batt_max_fcc_ua);
		if (chg->charge_therm_fcc_ua != -1) {
			port1_fcc = min(port1_fcc, chg->charge_therm_fcc_ua / 2);
			port2_fcc = min(port2_fcc, chg->charge_therm_fcc_ua / 2);
		}
	} else {
		total_icl = chg->total_main_icl_ua;
		if (-1 == chg->charge_therm_fcc_ua)
			total_fcc = chg->total_main_fcc_ua;
		else
			total_fcc = min(chg->total_main_fcc_ua, chg->charge_therm_fcc_ua);
		port1_icl = total_icl / 2;
		port1_fcc = min(chg->first_batt_max_fcc_ua, total_fcc / 2);
		port2_icl = total_icl / 2;
		port2_fcc = min(chg->sec_batt_max_fcc_ua, total_fcc / 2);
	}
	pr_info("batt_sys: charge [port] icl:fcc  (1) %d:%d  (2) %d:%d  (total) %d:%d\n",
			port1_icl, port1_fcc, port2_icl, port2_fcc, total_icl, total_fcc);

	//chg->first_request_icl_ua = port1_icl * (DEFAULT_USB_VOLTAGE / 1000000) / (vbus1_uv / 1000000);
	//chg->sec_request_icl_ua = port2_icl * (DEFAULT_USB_VOLTAGE / 1000000) / (vbus2_uv / 1000000);
	chg->first_request_icl_ua = port1_icl;
	chg->sec_request_icl_ua = port2_icl;
	chg->first_request_fcc_ua = port1_fcc;
	chg->sec_request_fcc_ua = port2_fcc;
	chg->first_request_fv_uv = chg->first_batt_max_fv_uv;
	chg->sec_request_fv_uv = chg->sec_batt_max_fv_uv;

	pr_info("batt_sys: charge [request] icl:fcc:fv  (1) %d:%d:%d  (2) %d:%d:%d\n",
		chg->first_request_icl_ua, chg->first_request_fcc_ua, chg->first_request_fv_uv,
		chg->sec_request_icl_ua, chg->sec_request_fcc_ua, chg->sec_request_fv_uv);

#ifdef CONFIG_BATTERY_FAC
	pr_info("batt_sys: [FACTORY] soc [%d]\n", last_user_soc);
	if (last_user_soc >= 60) {
		chg->first_request_icl_ua = -1;
		chg->first_request_fcc_ua = -1;
		chg->sec_request_icl_ua = -1;
		chg->sec_request_fcc_ua = -1;
		is_batt1_thermal_stop = 1;
		is_batt2_thermal_stop = 1;
		pr_info("batt_sys: [FACTORY] soc >= 60, force disable Charge [%d]\n", last_user_soc);
	} else {
		is_batt1_thermal_stop = 0;
		is_batt2_thermal_stop = 0;
		pr_info("batt_sys: [FACTORY] enable charge [%d]\n", last_user_soc);
	}
#endif
	pr_info("batt_sys: profile 1 ready %d, stat %d, def %d jeita %d req %d icl %d fv %d\n",
			ready, status1,
			chg->first_main_fcc_ua, chg->first_batt_max_fcc_ua,
			chg->first_request_fcc_ua, chg->first_request_icl_ua, chg->first_request_fv_uv);
	pr_info("batt_sys: profile 2 ready %d, stat %d, def %d jeita %d req %d icl %d fv %d\n",
			ready, status2,
			chg->sec_main_fcc_ua, chg->sec_batt_max_fcc_ua,
			chg->sec_request_fcc_ua, chg->sec_request_icl_ua, chg->sec_request_fv_uv);
	if (ready) {
		rc = smblib_get_prop_input_current_settled(chg, &val);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get settled ICL rc=%d\n", rc);
		}

		if (val.intval == 100000) {
			if ((C_PD == status1) ||
				(C_DCP == status1) ||
				(C_PD == status2) ||
				(C_DCP == status2))
			pr_info("batt_sys: profile qcom recovery process %d\n", val.intval);

			set_main_charge_prop(chg, 150000, 150000, chg->first_request_fv_uv, 350000, 350000, chg->sec_request_fv_uv);
			msleep(1000);
		}

		set_main_charge_prop(chg, chg->first_request_fcc_ua, chg->first_request_icl_ua, chg->first_request_fv_uv,
					chg->sec_request_fcc_ua, chg->sec_request_icl_ua, chg->sec_request_fv_uv);
	}

	return rc;
}

extern bool ffc_pd1_get_flash_chg_ready(void);
extern bool ffc_pd2_get_flash_chg_ready(void);
static void lenovo_charge_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			lenovo_charge_monitor_work.work);
	int port1_is_charger;
	int port2_is_charger;
	int port1_is_usb;
	int port2_is_usb;
	int status1;
	int status2;
	int pd1_running = ffc_pd1_get_flash_chg_ready();
	int pd2_running = ffc_pd2_get_flash_chg_ready();
	int ready = 1;
	int rc;
	union power_supply_propval val = {0,};
	int max_power1 = get_charger_max_power();
	int max_power2 = max(get_charger2_max_power(), get_charger2a_max_power());
	int pd_en = 1;

	//mutex_lock(&chg->charge_work_lock);
	//port1_is_charger = (qcom_orient_en | first_typec_attached ) & (!usb1_otg_en);
	port1_is_charger = first_typec_attached & (!usb1_otg_en) & (!usb1_audio_en);
	port2_is_charger = fusb_orient_en & (!usb2_otg_en);
	port1_is_usb = is_usb_first_port(chg);
	port2_is_usb = is_usb_sec_port(chg);
	status1 = C_NONE;
	status2 = C_NONE;

	//if ((!g_chg_en_dev) ||
	//		((!port1_is_charger) && (!port2_is_charger)))
	if ((!port1_is_charger) && (!port2_is_charger))
		goto out;

	if (!insert_delay_done) {
		status1 = C_UNKNOWN;
		status2 = C_UNKNOWN;
		goto out;
	}

	if (port1_is_charger) {
		if (port1_is_usb) {
			status1 = C_USB;
			pr_info("batt_sys: port-1 type : [ USB ] Device\n");
		} else if ((max_power1 > 0) && (max_power1 <= 15)) {
			status1 = C_USB;
#ifdef FORCE_SINGLE_PD_TO_DCP
			pd_en = 0;
#endif
			pr_info("batt_sys: port-1 type : [ USB ] WA for weak pd charger [%d]\n", max_power1);
		} else if (is_batt1_thermal_stop) {
			status1 = C_DCP;
#ifdef FORCE_SINGLE_PD_TO_DCP
			pd_en = 0;
#endif
			pr_info("batt_sys: port-1 type : [ DCP ] Request pd stop for thermal\n");
#ifdef FORCE_SINGLE_PD_TO_DCP
		} else if (!port2_is_charger) {
			status1 = C_DCP;
			pd_en = 0;
			pr_info("batt_sys: port-1 type : [ DCP ] change single port pd to dcp\n");
#endif
		} else if (pd1_running) {
			status1 = C_PD;
			pr_info("batt_sys: port-1 type : [ PD ] pd running\n");
		} else if (1 == chg->restart_pd_status) {
			status1 = C_PD;
			pr_info("batt_sys: port-1 type : [ PD ] Request pd restart\n");
		} else {
			status1 = C_DCP;
			pr_info("batt_sys: port-1 type : [ DCP ] Default device\n");
		}
	}

	if (port2_is_charger) {
		rc = smblib_get_second_real_type(chg, &val);
		if (rc < 0) {
			status2 = C_USB;
			pr_info("batt_sys: port-2 type : [ USB ] Can not get charger real type\n");
		} else if (POWER_SUPPLY_TYPE_UNKNOWN == val.intval) {
			status2 = C_USB;
			pr_info("batt_sys: port-2 type : [ USB ] Charger real type Unknown\n");
		} else if (port2_is_usb) {
			status2 = C_USB;
			pr_info("batt_sys: port-2 type : [ USB ] Device\n");
		} else if ((max_power2 > 0) && (max_power2 <= 15)) {
			status2 = C_USB;
#ifdef FORCE_SINGLE_PD_TO_DCP
			pd_en = 0;
#endif
			pr_info("batt_sys: port-2 type : [ USB ] WA for weak pd charger [%d]\n", max_power2);
		} else if (is_batt2_thermal_stop) {
			status2 = C_DCP;
#ifdef FORCE_SINGLE_PD_TO_DCP
			pd_en = 0;
#endif
			pr_info("batt_sys: port-2 type : [ DCP ] Request pd stop for thermal\n");
#ifdef FORCE_SINGLE_PD_TO_DCP
		} else if (!port1_is_charger) {
			status2 = C_DCP;
			pd_en = 0;
			pr_info("batt_sys: port-2 type : [ DCP ] change single port pd to dcp\n");
#endif
		} else if (pd2_running) {
			status2 = C_PD;
			pr_info("batt_sys: port-2 type : [ PD ] pd running\n");
		} else if (1 == chg->restart_pd_status) {
			status2 = C_PD;
			pr_info("batt_sys: port-2 type : [ PD ] Request pd restart\n");
		} else {
			status2 = C_DCP;
			pr_info("batt_sys: port-2 type : [ DCP ] Default device\n");
		}
	}

	if ((!bootup_delay_done) && ((C_USB == status1) || (C_USB == status2))) {
		pr_info("batt_sys: system bootup ...");
		status1 = C_UNKNOWN;
		status2 = C_UNKNOWN;
	}

out:
	pr_info("batt_sys: chg-monitor ready %d, in_delay %d, typec1 %d, charger %d:%d, usb %d:%d, pd %d:%d, status %d:%d, en 0x%02X\n",
			ready, insert_delay_done, first_typec_attached,
			port1_is_charger, port2_is_charger,
			port1_is_usb, port2_is_usb,
			pd1_running, pd2_running,
			status1, status2, g_chg_en_dev);

	if (ready)
		set_dual_charge_profile(chg, status1, status2, pd_en);
	//mutex_unlock(&chg->charge_work_lock);

	schedule_delayed_work(&chg->lenovo_charge_monitor_work,
						msecs_to_jiffies(10000));
}

static void lenovo_icl_settled_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			lenovo_icl_settled_work.work);

	pr_info("batt_sys: icl settled done [%d] [%d] [%d]\n",
			chg->max_icl_settled_ua, chg->usb_vbus_uv, chg->sin_port_max_power);
	if (!chg->max_icl_settled_ua) {
		chg->max_icl_settled_ua = 2000000;
		chg->usb_vbus_uv = DEFAULT_USB_VOLTAGE;
		chg->sin_port_max_power = 10000000;
		pr_info("batt_sys: icl settled default [%d] [%d] [%d]\n",
				 chg->max_icl_settled_ua, chg->usb_vbus_uv, chg->sin_port_max_power);
	}
	chg->icl_settled_ready = 2;
	cancel_delayed_work_sync(&chg->lenovo_charge_monitor_work);
	schedule_delayed_work(&chg->lenovo_charge_monitor_work,
						msecs_to_jiffies(0));
}

static void lenovo_restart_pd_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			lenovo_restart_pd_work.work);

	pr_info("batt_sys: restart pd work done\n");
	chg->restart_pd_status = 2;
}

static void lenovo_insert_delay_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			lenovo_insert_delay_work.work);

	pr_info("batt_sys: charger insert/remove/recheck delay done\n");
	insert_delay_done = 1;
	cancel_delayed_work_sync(&chg->lenovo_charge_monitor_work);
	schedule_delayed_work(&chg->lenovo_charge_monitor_work,
						msecs_to_jiffies(0));
}

static void lenovo_bootup_delay_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			lenovo_bootup_delay_work.work);

	pr_info("batt_sys: charger bootup delay done\n");
	bootup_delay_done = 1;
	cancel_delayed_work_sync(&chg->lenovo_charge_monitor_work);
	schedule_delayed_work(&chg->lenovo_charge_monitor_work,
						msecs_to_jiffies(0));
}

void charger_insert_remove_process(struct smb_charger *chg, int online)
{
	int i;

	pr_info("batt_sys: charger insert/remove/recheck process %d\n", online);

	cancel_delayed_work_sync(&chg->lenovo_insert_delay_work);
	cancel_delayed_work_sync(&chg->lenovo_charge_monitor_work);
	cancel_delayed_work_sync(&chg->lenovo_icl_settled_work);
	cancel_delayed_work_sync(&chg->lenovo_restart_pd_work);
	set_dual_charge_profile(chg, C_UNKNOWN, C_UNKNOWN, 1);
	insert_delay_done = 0;
	chg->icl_settled_ready = 0;
	chg->max_icl_settled_ua = 0;
	chg->usb_vbus_uv = 0;
	chg->sin_port_max_power = 0;
	chg->restart_pd_status = 0;
	chg->batt1_cap_full = 0;
	chg->batt2_cap_full = 0;
	chg->port1_charge_done = 0;
	chg->port2_charge_done = 0;
	chg->charge_therm_fcc_ua = -1;
	therm_check_total = 0;
	for (i = 3; i >= 0; i--)
		therm_level[i] = 0;

	schedule_delayed_work(&chg->lenovo_insert_delay_work,
						msecs_to_jiffies(CHARGER_IN_DELAY_MS));
}

static void lenovo_thermal_monitor_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
			lenovo_thermal_monitor_work.work);
	int max = 0;
	int i;
	int index;
	int user_therm_enable = 0;
	int max_power1 = get_charger_max_power();
	int max_power2 = max(get_charger2_max_power(), get_charger2a_max_power());

	therm_level[chg->system_temp_level]++;
	therm_check_total++;

	if (therm_check_total > THERMAL_CHECK_MAX_NUM) {
		for (i = 3; i >= 0; i--) {
			if (max < therm_level[i]) {
				max = therm_level[i];
				index = i;
			}
		}

		if ((max_power1 > 15) && (max_power2 > 15)) {
			if (skin_temp >= DUAL_PD_THERM_USER_HIGH_TEMP)
				user_therm_enable = 1;
		} else {
			if (skin_temp >= SINGLE_PD_THERM_USER_HIGH_TEMP)
				user_therm_enable = 1;
		}

		if (user_therm_enable) {
			if (0 == index)
				chg->charge_therm_fcc_ua = -1;
			else
				chg->charge_therm_fcc_ua = chg->thermal_mitigation[index];
		} else
			chg->charge_therm_fcc_ua = -1;

		pr_info("batt_sys: thermal user %d, skin temp %d, level %d, count %d %d %d %d, fcc %d\n",
				user_therm_enable, skin_temp, index,
				therm_level[0], therm_level[1], therm_level[2], therm_level[3], chg->charge_therm_fcc_ua);

		therm_check_total = 0;
		for (i = 3; i >= 0; i--)
			therm_level[i] = 0;

	} else
		pr_info("batt_sys: thermal check num %d level %d %d %d %d\n",
				therm_check_total, therm_level[0], therm_level[1], therm_level[2], therm_level[3]);

	schedule_delayed_work(&chg->lenovo_thermal_monitor_work,
				msecs_to_jiffies(LENOVO_THERMAL_MONITOR_DEALY_MS));
}
#endif

static void bms_update_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						bms_update_work);
	smblib_suspend_on_debug_battery(chg);

	if (chg->batt_psy)
		power_supply_changed(chg->batt_psy);
}

static void pl_update_work(struct work_struct *work)
{
	union power_supply_propval prop_val;
	struct smb_charger *chg = container_of(work, struct smb_charger,
						pl_update_work);
	int rc;

	if (chg->smb_temp_max == -EINVAL) {
		rc = smblib_get_thermal_threshold(chg,
					SMB_REG_H_THRESHOLD_MSB_REG,
					&chg->smb_temp_max);
		if (rc < 0) {
			dev_err(chg->dev, "Couldn't get charger_temp_max rc=%d\n",
					rc);
			return;
		}
	}

	prop_val.intval = chg->smb_temp_max;
	rc = power_supply_set_property(chg->pl.psy,
				POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
				&prop_val);
	if (rc < 0) {
		dev_err(chg->dev, "Couldn't set POWER_SUPPLY_PROP_CHARGER_TEMP_MAX rc=%d\n",
				rc);
		return;
	}

	if (chg->sec_chg_selected == POWER_SUPPLY_CHARGER_SEC_CP)
		return;

	smblib_select_sec_charger(chg, POWER_SUPPLY_CHARGER_SEC_PL,
				POWER_SUPPLY_CP_NONE, false);
}

static void clear_hdc_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						clear_hdc_work.work);

	chg->is_hdc = false;
	vote(chg->hdc_irq_disable_votable, HDC_IRQ_VOTER, false, 0);
}

static void smblib_icl_change_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							icl_change_work.work);
	int rc, settled_ua;
	union power_supply_propval val = {0,};
	int vbus_volt;
	int max_power;

	rc = smblib_get_charge_param(chg, &chg->param.icl_stat, &settled_ua);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get ICL status rc=%d\n", rc);
		return;
	}

	power_supply_changed(chg->usb_main_psy);

	smblib_dbg(chg, PR_INTERRUPT, "icl_settled=%d, icl_settled_ready=%d\n",
				settled_ua, chg->icl_settled_ready);

	if (1 == chg->icl_settled_ready) {
		rc = smblib_get_prop_usb_voltage_now(chg, &val);
		if ((rc < 0) || (val.intval < 4200000))
			vbus_volt = DEFAULT_USB_VOLTAGE;
		else
			vbus_volt = val.intval;
		max_power = (settled_ua / 1000) * (vbus_volt / 1000);
		smblib_dbg(chg, PR_INTERRUPT, "icl_settled=%d, vbus=%d, in_power=%d\n",
				settled_ua, vbus_volt, max_power);

		if (chg->sin_port_max_power < max_power) {
			chg->max_icl_settled_ua = settled_ua;
			chg->usb_vbus_uv = vbus_volt;
			chg->sin_port_max_power = max_power;
		}
	}
}

static void smblib_pl_enable_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							pl_enable_work.work);

	smblib_dbg(chg, PR_PARALLEL, "timer expired, enabling parallel\n");
	vote(chg->pl_disable_votable, PL_DELAY_VOTER, false, 0);
	vote(chg->awake_votable, PL_DELAY_VOTER, false, 0);
}

static void smblib_thermal_regulation_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						thermal_regulation_work.work);
	int rc;

	rc = smblib_update_thermal_readings(chg);
	if (rc < 0)
		smblib_err(chg, "Couldn't read current thermal values %d\n",
					rc);

	rc = smblib_process_thermal_readings(chg);
	if (rc < 0)
		smblib_err(chg, "Couldn't run sw thermal regulation %d\n",
					rc);
}

#define MOISTURE_PROTECTION_CHECK_DELAY_MS 300000		/* 5 mins */
static void smblib_moisture_protection_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						moisture_protection_work);
	int rc;
	bool usb_plugged_in;
	u8 stat;

	/*
	 * Hold awake votable to prevent pm_relax being called prior to
	 * completion of this work.
	 */
	vote(chg->awake_votable, MOISTURE_VOTER, true, 0);

	/*
	 * Disable 1% duty cycle on CC_ID pin and enable uUSB factory mode
	 * detection to track any change on RID, as interrupts are disable.
	 */
	rc = smblib_write(chg, ((chg->chg_param.smb_version == PMI632_SUBTYPE) ?
			PMI632_TYPEC_U_USB_WATER_PROTECTION_CFG_REG :
			TYPEC_U_USB_WATER_PROTECTION_CFG_REG), 0);
	if (rc < 0) {
		smblib_err(chg, "Couldn't disable periodic monitoring of CC_ID rc=%d\n",
			rc);
		goto out;
	}

	rc = smblib_masked_write(chg, TYPEC_U_USB_CFG_REG,
					EN_MICRO_USB_FACTORY_MODE_BIT,
					EN_MICRO_USB_FACTORY_MODE_BIT);
	if (rc < 0) {
		smblib_err(chg, "Couldn't enable uUSB factory mode detection rc=%d\n",
			rc);
		goto out;
	}

	/*
	 * Add a delay of 100ms to allow change in rid to reflect on
	 * status registers.
	 */
	msleep(100);

	rc = smblib_read(chg, USBIN_BASE + INT_RT_STS_OFFSET, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read USB_INT_RT_STS rc=%d\n", rc);
		goto out;
	}
	usb_plugged_in = (bool)(stat & USBIN_PLUGIN_RT_STS_BIT);

	/* Check uUSB status for moisture presence */
	rc = smblib_read(chg, TYPEC_U_USB_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_U_USB_STATUS_REG rc=%d\n",
				rc);
		goto out;
	}

	/*
	 * Factory mode detection happens in case of USB plugged-in by using
	 * a different current source of 2uA which can hamper moisture
	 * detection. Since factory mode is not supported in kernel, factory
	 * mode detection can be considered as equivalent to presence of
	 * moisture.
	 */
	if (stat == U_USB_STATUS_WATER_PRESENT || stat == U_USB_FMB1_BIT ||
			stat == U_USB_FMB2_BIT || (usb_plugged_in &&
			stat == U_USB_FLOAT1_BIT)) {
		smblib_set_moisture_protection(chg, true);
		alarm_start_relative(&chg->moisture_protection_alarm,
			ms_to_ktime(MOISTURE_PROTECTION_CHECK_DELAY_MS));
	} else {
		smblib_set_moisture_protection(chg, false);
		rc = alarm_cancel(&chg->moisture_protection_alarm);
		if (rc < 0)
			smblib_err(chg, "Couldn't cancel moisture protection alarm\n");
	}

out:
	vote(chg->awake_votable, MOISTURE_VOTER, false, 0);
}

static enum alarmtimer_restart moisture_protection_alarm_cb(struct alarm *alarm,
							ktime_t now)
{
	struct smb_charger *chg = container_of(alarm, struct smb_charger,
					moisture_protection_alarm);

	smblib_dbg(chg, PR_MISC, "moisture Protection Alarm Triggered %lld\n",
			ktime_to_ms(now));

	/* Atomic context, cannot use voter */
	pm_stay_awake(chg->dev);
	schedule_work(&chg->moisture_protection_work);

	return ALARMTIMER_NORESTART;
}

static void smblib_chg_termination_work(struct work_struct *work)
{
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(work, struct smb_charger,
						chg_termination_work);
	int rc, input_present, delay = CHG_TERM_WA_ENTRY_DELAY_MS;
	int vbat_now_uv, max_fv_uv;

	/*
	 * Hold awake votable to prevent pm_relax being called prior to
	 * completion of this work.
	 */
	vote(chg->awake_votable, CHG_TERMINATION_VOTER, true, 0);

	rc = smblib_is_input_present(chg, &input_present);
	if ((rc < 0) || !input_present)
		goto out;

	rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_REAL_CAPACITY, &pval);
	if ((rc < 0) || (pval.intval < 100)) {
		vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
		vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER, false, 0);
		goto out;
	}

	/* Get the battery float voltage */
	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_VOLTAGE_MAX,
				&pval);
	if (rc < 0)
		goto out;

	max_fv_uv = pval.intval;

	rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CHARGE_FULL,
					&pval);
	if (rc < 0)
		goto out;

	/*
	 * On change in the value of learned capacity, re-initialize the
	 * reference cc_soc value due to change in cc_soc characteristic value
	 * at full capacity. Also, in case cc_soc_ref value is reset,
	 * re-initialize it.
	 */
	if (pval.intval != chg->charge_full_cc || !chg->cc_soc_ref) {
		chg->charge_full_cc = pval.intval;

		rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
		if (rc < 0)
			goto out;

		/*
		 * Store the Vbat at the charge termination to compare with
		 * the current voltage to see if the Vbat is increasing after
		 * charge termination in BSM.
		 */
		chg->term_vbat_uv = pval.intval;
		vbat_now_uv = pval.intval;

		rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CC_SOC,
					&pval);
		if (rc < 0)
			goto out;

		chg->cc_soc_ref = pval.intval;
	} else {
		rc = smblib_get_prop_from_bms(chg,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
		if (rc < 0)
			goto out;

		vbat_now_uv = pval.intval;

		rc = smblib_get_prop_from_bms(chg, POWER_SUPPLY_PROP_CC_SOC,
					&pval);
		if (rc < 0)
			goto out;
	}

	/*
	 * In BSM a sudden jump in CC_SOC is not expected. If seen, its a
	 * good_ocv or updated capacity, reject it.
	 */
	if (chg->last_cc_soc && pval.intval > (chg->last_cc_soc + 100)) {
		/* CC_SOC has increased by 1% from last time */
		chg->cc_soc_ref = pval.intval;
		smblib_dbg(chg, PR_MISC, "cc_soc jumped(%d->%d), reset cc_soc_ref\n",
				chg->last_cc_soc, pval.intval);
	}
	chg->last_cc_soc = pval.intval;

	/*
	 * Suspend/Unsuspend USB input to keep cc_soc within the 0.5% to 0.75%
	 * overshoot range of the cc_soc value at termination and make sure that
	 * vbat is indeed rising above vfloat.
	 */
	if (pval.intval < DIV_ROUND_CLOSEST(chg->cc_soc_ref * 10050, 10000)) {
		vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER, false, 0);
		vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER, false, 0);
		delay = CHG_TERM_WA_ENTRY_DELAY_MS;
	} else if ((pval.intval > DIV_ROUND_CLOSEST(chg->cc_soc_ref * 10075,
								10000))
		  && ((vbat_now_uv > chg->term_vbat_uv) &&
		     (vbat_now_uv > max_fv_uv))) {

		if (input_present & INPUT_PRESENT_USB)
			vote(chg->usb_icl_votable, CHG_TERMINATION_VOTER,
					true, 0);
		if (input_present & INPUT_PRESENT_DC)
			vote(chg->dc_suspend_votable, CHG_TERMINATION_VOTER,
					true, 0);
		delay = CHG_TERM_WA_EXIT_DELAY_MS;
	}

	smblib_dbg(chg, PR_MISC, "Chg Term WA readings: cc_soc: %d, cc_soc_ref: %d, delay: %d vbat_now %d term_vbat %d\n",
			pval.intval, chg->cc_soc_ref, delay, vbat_now_uv,
			chg->term_vbat_uv);
	alarm_start_relative(&chg->chg_termination_alarm, ms_to_ktime(delay));
out:
	vote(chg->awake_votable, CHG_TERMINATION_VOTER, false, 0);
}

static enum alarmtimer_restart chg_termination_alarm_cb(struct alarm *alarm,
								ktime_t now)
{
	struct smb_charger *chg = container_of(alarm, struct smb_charger,
							chg_termination_alarm);

	smblib_dbg(chg, PR_MISC, "Charge termination WA alarm triggered %lld\n",
			ktime_to_ms(now));

	/* Atomic context, cannot use voter */
	pm_stay_awake(chg->dev);
	schedule_work(&chg->chg_termination_work);

	return ALARMTIMER_NORESTART;
}

static void apsd_timer_cb(struct timer_list *tm)
{
	struct smb_charger *chg = container_of(tm, struct smb_charger,
							apsd_timer);

	smblib_dbg(chg, PR_MISC, "APSD Extented timer timeout at %lld\n",
			jiffies_to_msecs(jiffies));

	chg->apsd_ext_timeout = true;
}

#define SOFT_JEITA_HYSTERESIS_OFFSET	0x200
static void jeita_update_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
						jeita_update_work);
	struct device_node *node = chg->dev->of_node;
	struct device_node *batt_node, *pnode;
	union power_supply_propval val;
	int rc, tmp[2], max_fcc_ma, max_fv_uv;
	u32 jeita_hard_thresholds[2];
	u16 addr;
	u8 buff[2];

	batt_node = of_find_node_by_name(node, "qcom,battery-data");
	if (!batt_node) {
		smblib_err(chg, "Batterydata not available\n");
		goto out;
	}

	/* if BMS is not ready, defer the work */
	if (!chg->bms_psy)
		return;

	rc = smblib_get_prop_from_bms(chg,
			POWER_SUPPLY_PROP_RESISTANCE_ID, &val);
	if (rc < 0) {
		smblib_err(chg, "Failed to get batt-id rc=%d\n", rc);
		goto out;
	}

	/* if BMS hasn't read out the batt_id yet, defer the work */
	if (val.intval <= 0)
		return;

	pnode = of_batterydata_get_best_profile(batt_node,
					val.intval / 1000, NULL);
	if (IS_ERR(pnode)) {
		rc = PTR_ERR(pnode);
		smblib_err(chg, "Failed to detect valid battery profile %d\n",
				rc);
		goto out;
	}

	rc = of_property_read_u32_array(pnode, "qcom,jeita-hard-thresholds",
				jeita_hard_thresholds, 2);
	if (!rc) {
		rc = smblib_update_jeita(chg, jeita_hard_thresholds,
					JEITA_HARD);
		if (rc < 0) {
			smblib_err(chg, "Couldn't configure Hard Jeita rc=%d\n",
					rc);
			goto out;
		}
	}

	rc = of_property_read_u32_array(pnode, "qcom,jeita-soft-thresholds",
				chg->jeita_soft_thlds, 2);
	if (!rc) {
		rc = smblib_update_jeita(chg, chg->jeita_soft_thlds,
					JEITA_SOFT);
		if (rc < 0) {
			smblib_err(chg, "Couldn't configure Soft Jeita rc=%d\n",
					rc);
			goto out;
		}

		rc = of_property_read_u32_array(pnode,
					"qcom,jeita-soft-hys-thresholds",
					chg->jeita_soft_hys_thlds, 2);
		if (rc < 0) {
			smblib_err(chg, "Couldn't get Soft Jeita hysteresis thresholds rc=%d\n",
					rc);
			goto out;
		}
	} else {
		/* Populate the jeita-soft-thresholds */
		addr = CHGR_JEITA_THRESHOLD_BASE_REG(JEITA_SOFT);
		rc = smblib_batch_read(chg, addr, buff, 2);
		if (rc < 0) {
			pr_err("failed to read 0x%4X, rc=%d\n", addr, rc);
			goto out;
		}

		chg->jeita_soft_thlds[1] = buff[1] | buff[0] << 8;

		rc = smblib_batch_read(chg, addr + 2, buff, 2);
		if (rc < 0) {
			pr_err("failed to read 0x%4X, rc=%d\n", addr + 2, rc);
			goto out;
		}

		chg->jeita_soft_thlds[0] = buff[1] | buff[0] << 8;

		/*
		 * Update the soft jeita hysteresis 2 DegC less for warm and
		 * 2 DegC more for cool than the soft jeita thresholds to avoid
		 * overwriting the registers with invalid values.
		 */
		chg->jeita_soft_hys_thlds[0] =
			chg->jeita_soft_thlds[0] - SOFT_JEITA_HYSTERESIS_OFFSET;
		chg->jeita_soft_hys_thlds[1] =
			chg->jeita_soft_thlds[1] + SOFT_JEITA_HYSTERESIS_OFFSET;
	}

	chg->jeita_soft_fcc[0] = chg->jeita_soft_fcc[1] = -EINVAL;
	chg->jeita_soft_fv[0] = chg->jeita_soft_fv[1] = -EINVAL;
	max_fcc_ma = max_fv_uv = -EINVAL;

	of_property_read_u32(pnode, "qcom,fastchg-current-ma", &max_fcc_ma);
	of_property_read_u32(pnode, "qcom,max-voltage-uv", &max_fv_uv);

	if (max_fcc_ma <= 0 || max_fv_uv <= 0) {
		smblib_err(chg, "Incorrect fastchg-current-ma or max-voltage-uv\n");
		goto out;
	}

	rc = of_property_read_u32_array(pnode, "qcom,jeita-soft-fcc-ua",
					tmp, 2);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get fcc values for soft JEITA rc=%d\n",
				rc);
		goto out;
	}

	max_fcc_ma *= 1000;
	if (tmp[0] > max_fcc_ma || tmp[1] > max_fcc_ma) {
		smblib_err(chg, "Incorrect FCC value [%d %d] max: %d\n", tmp[0],
			tmp[1], max_fcc_ma);
		goto out;
	}
	chg->jeita_soft_fcc[0] = tmp[0];
	chg->jeita_soft_fcc[1] = tmp[1];

	rc = of_property_read_u32_array(pnode, "qcom,jeita-soft-fv-uv", tmp,
					2);
	if (rc < 0) {
		smblib_err(chg, "Couldn't get fv values for soft JEITA rc=%d\n",
				rc);
		goto out;
	}

	if (tmp[0] > max_fv_uv || tmp[1] > max_fv_uv) {
		smblib_err(chg, "Incorrect FV value [%d %d] max: %d\n", tmp[0],
			tmp[1], max_fv_uv);
		goto out;
	}
	chg->jeita_soft_fv[0] = tmp[0];
	chg->jeita_soft_fv[1] = tmp[1];

	rc = smblib_soft_jeita_arb_wa(chg);
	if (rc < 0) {
		smblib_err(chg, "Couldn't fix soft jeita arb rc=%d\n",
				rc);
		goto out;
	}

	chg->jeita_configured = JEITA_CFG_COMPLETE;
	return;

out:
	chg->jeita_configured = JEITA_CFG_FAILURE;
}

static void smblib_lpd_ra_open_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							lpd_ra_open_work.work);
	union power_supply_propval pval;
	u8 stat;
	int rc;

	if (chg->pr_swap_in_progress || chg->pd_hard_reset) {
		chg->lpd_stage = LPD_STAGE_NONE;
		goto out;
	}

	if (chg->lpd_stage != LPD_STAGE_FLOAT)
		goto out;

	rc = smblib_read(chg, TYPE_C_MISC_STATUS_REG, &stat);
	if (rc < 0) {
		smblib_err(chg, "Couldn't read TYPE_C_MISC_STATUS_REG rc=%d\n",
			rc);
		goto out;
	}

	/* quit if moisture status is gone or in attached state */
	if (!(stat & TYPEC_WATER_DETECTION_STATUS_BIT)
			|| (stat & TYPEC_TCCDEBOUNCE_DONE_STATUS_BIT)) {
		chg->lpd_stage = LPD_STAGE_NONE;
		goto out;
	}

	chg->lpd_stage = LPD_STAGE_COMMIT;

	/* Enable source only mode */
	pval.intval = POWER_SUPPLY_TYPEC_PR_SOURCE;
	rc = smblib_set_prop_typec_power_role(chg, &pval);
	if (rc < 0) {
		smblib_err(chg, "Couldn't set typec source only mode rc=%d\n",
					rc);
		goto out;
	}

	/* Wait 1.5ms to get SBUx ready */
	usleep_range(1500, 1510);

	if (smblib_rsbux_low(chg, RSBU_K_300K_UV)) {
		/* Moisture detected, enable sink only mode */
		pval.intval = POWER_SUPPLY_TYPEC_PR_SINK;
		rc = smblib_set_prop_typec_power_role(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set typec sink only rc=%d\n",
				rc);
			goto out;
		}

		chg->lpd_reason = LPD_MOISTURE_DETECTED;
		chg->moisture_present =  true;
		vote(chg->usb_icl_votable, LPD_VOTER, true, 0);

	} else {
		/* Floating cable, disable water detection irq temporarily */
		rc = smblib_masked_write(chg, TYPE_C_INTERRUPT_EN_CFG_2_REG,
					TYPEC_WATER_DETECTION_INT_EN_BIT, 0);
		if (rc < 0) {
			smblib_err(chg, "Couldn't set TYPE_C_INTERRUPT_EN_CFG_2_REG rc=%d\n",
					rc);
			goto out;
		}

		/* restore DRP mode */
		pval.intval = POWER_SUPPLY_TYPEC_PR_DUAL;
		rc = smblib_set_prop_typec_power_role(chg, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't write 0x%02x to TYPE_C_INTRPT_ENB_SOFTWARE_CTRL rc=%d\n",
				pval.intval, rc);
			goto out;
		}

		chg->lpd_reason = LPD_FLOATING_CABLE;
	}

	/* recheck in 60 seconds */
	alarm_start_relative(&chg->lpd_recheck_timer, ms_to_ktime(60000));
out:
	vote(chg->awake_votable, LPD_VOTER, false, 0);
}

static void smblib_lpd_detach_work(struct work_struct *work)
{
	struct smb_charger *chg = container_of(work, struct smb_charger,
							lpd_detach_work.work);

	if (chg->lpd_stage == LPD_STAGE_FLOAT_CANCEL)
		chg->lpd_stage = LPD_STAGE_NONE;
}

static void smblib_cp_status_change_work(struct work_struct *work)
{
	int rc;
	union power_supply_propval pval;
	struct smb_charger *chg = container_of(work, struct smb_charger,
			cp_status_change_work);

	if (!chg->cp_psy)
		chg->cp_psy = power_supply_get_by_name("charge_pump_master");

	if (!chg->cp_psy)
		goto relax;

	if (chg->cp_topo == -EINVAL) {
		rc = power_supply_get_property(chg->cp_psy,
				POWER_SUPPLY_PROP_PARALLEL_OUTPUT_MODE, &pval);
		if (rc < 0) {
			smblib_err(chg, "Couldn't read cp topo rc=%d\n", rc);
			goto relax;
		}

		chg->cp_topo = pval.intval;

		if (chg->cp_topo == POWER_SUPPLY_PL_OUTPUT_VBAT &&
				chg->cp_reason == POWER_SUPPLY_CP_WIRELESS)
			vote(chg->fcc_main_votable, WLS_PL_CHARGING_VOTER, true,
					800000);
	}
relax:
	pr_info("batt_sys: pm_relax [cp_work]\n");
	pm_relax(chg->dev);
}

static int smblib_create_votables(struct smb_charger *chg)
{
	int rc = 0;

	chg->fcc_votable = find_votable("FCC");
	if (chg->fcc_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find FCC votable rc=%d\n", rc);
		return rc;
	}

	chg->fcc_main_votable = find_votable("FCC_MAIN");
	if (chg->fcc_main_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find FCC Main votable rc=%d\n", rc);
		return rc;
	}

	chg->fv_votable = find_votable("FV");
	if (chg->fv_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find FV votable rc=%d\n", rc);
		return rc;
	}

	chg->usb_icl_votable = find_votable("USB_ICL");
	if (chg->usb_icl_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find USB_ICL votable rc=%d\n", rc);
		return rc;
	}

	chg->pl_disable_votable = find_votable("PL_DISABLE");
	if (chg->pl_disable_votable == NULL) {
		rc = -EINVAL;
		smblib_err(chg, "Couldn't find votable PL_DISABLE rc=%d\n", rc);
		return rc;
	}

	chg->pl_enable_votable_indirect = find_votable("PL_ENABLE_INDIRECT");
	if (chg->pl_enable_votable_indirect == NULL) {
		rc = -EINVAL;
		smblib_err(chg,
			"Couldn't find votable PL_ENABLE_INDIRECT rc=%d\n",
			rc);
		return rc;
	}

	vote(chg->pl_disable_votable, PL_DELAY_VOTER, true, 0);

	chg->smb_override_votable = create_votable("SMB_EN_OVERRIDE",
				VOTE_SET_ANY,
				smblib_smb_disable_override_vote_callback, chg);
	if (IS_ERR(chg->smb_override_votable)) {
		rc = PTR_ERR(chg->smb_override_votable);
		chg->smb_override_votable = NULL;
		return rc;
	}

	chg->dc_suspend_votable = create_votable("DC_SUSPEND", VOTE_SET_ANY,
					smblib_dc_suspend_vote_callback,
					chg);
	if (IS_ERR(chg->dc_suspend_votable)) {
		rc = PTR_ERR(chg->dc_suspend_votable);
		chg->dc_suspend_votable = NULL;
		return rc;
	}

	chg->awake_votable = create_votable("AWAKE", VOTE_SET_ANY,
					smblib_awake_vote_callback,
					chg);
	if (IS_ERR(chg->awake_votable)) {
		rc = PTR_ERR(chg->awake_votable);
		chg->awake_votable = NULL;
		return rc;
	}

	chg->chg_disable_votable = create_votable("CHG_DISABLE", VOTE_SET_ANY,
					smblib_chg_disable_vote_callback,
					chg);
	if (IS_ERR(chg->chg_disable_votable)) {
		rc = PTR_ERR(chg->chg_disable_votable);
		chg->chg_disable_votable = NULL;
		return rc;
	}

	chg->limited_irq_disable_votable = create_votable(
				"USB_LIMITED_IRQ_DISABLE",
				VOTE_SET_ANY,
				smblib_limited_irq_disable_vote_callback,
				chg);
	if (IS_ERR(chg->limited_irq_disable_votable)) {
		rc = PTR_ERR(chg->limited_irq_disable_votable);
		chg->limited_irq_disable_votable = NULL;
		return rc;
	}

	chg->hdc_irq_disable_votable = create_votable("USB_HDC_IRQ_DISABLE",
					VOTE_SET_ANY,
					smblib_hdc_irq_disable_vote_callback,
					chg);
	if (IS_ERR(chg->hdc_irq_disable_votable)) {
		rc = PTR_ERR(chg->hdc_irq_disable_votable);
		chg->hdc_irq_disable_votable = NULL;
		return rc;
	}

	chg->icl_irq_disable_votable = create_votable("USB_ICL_IRQ_DISABLE",
					VOTE_SET_ANY,
					smblib_icl_irq_disable_vote_callback,
					chg);
	if (IS_ERR(chg->icl_irq_disable_votable)) {
		rc = PTR_ERR(chg->icl_irq_disable_votable);
		chg->icl_irq_disable_votable = NULL;
		return rc;
	}

	chg->temp_change_irq_disable_votable = create_votable(
			"TEMP_CHANGE_IRQ_DISABLE", VOTE_SET_ANY,
			smblib_temp_change_irq_disable_vote_callback, chg);
	if (IS_ERR(chg->temp_change_irq_disable_votable)) {
		rc = PTR_ERR(chg->temp_change_irq_disable_votable);
		chg->temp_change_irq_disable_votable = NULL;
		return rc;
	}

	return rc;
}

static void smblib_destroy_votables(struct smb_charger *chg)
{
	if (chg->dc_suspend_votable)
		destroy_votable(chg->dc_suspend_votable);
	if (chg->usb_icl_votable)
		destroy_votable(chg->usb_icl_votable);
	if (chg->awake_votable)
		destroy_votable(chg->awake_votable);
	if (chg->chg_disable_votable)
		destroy_votable(chg->chg_disable_votable);
}

static void smblib_iio_deinit(struct smb_charger *chg)
{
	if (!IS_ERR_OR_NULL(chg->iio.usbin_v_chan))
		iio_channel_release(chg->iio.usbin_v_chan);
	if (!IS_ERR_OR_NULL(chg->iio.usbin_i_chan))
		iio_channel_release(chg->iio.usbin_i_chan);
	if (!IS_ERR_OR_NULL(chg->iio.temp_chan))
		iio_channel_release(chg->iio.temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.sbux_chan))
		iio_channel_release(chg->iio.sbux_chan);
	if (!IS_ERR_OR_NULL(chg->iio.vph_v_chan))
		iio_channel_release(chg->iio.vph_v_chan);
	if (!IS_ERR_OR_NULL(chg->iio.die_temp_chan))
		iio_channel_release(chg->iio.die_temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.connector_temp_chan))
		iio_channel_release(chg->iio.connector_temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.skin_temp_chan))
		iio_channel_release(chg->iio.skin_temp_chan);
	if (!IS_ERR_OR_NULL(chg->iio.smb_temp_chan))
		iio_channel_release(chg->iio.smb_temp_chan);
}

int smblib_init(struct smb_charger *chg)
{
	union power_supply_propval prop_val;
	int rc = 0;

	mutex_init(&chg->smb_lock);
	mutex_init(&chg->irq_status_lock);
	mutex_init(&chg->dcin_aicl_lock);
	mutex_init(&chg->dpdm_lock);
	spin_lock_init(&chg->typec_pr_lock);
	INIT_WORK(&chg->bms_update_work, bms_update_work);
	INIT_WORK(&chg->pl_update_work, pl_update_work);
	INIT_WORK(&chg->jeita_update_work, jeita_update_work);
	INIT_WORK(&chg->dcin_aicl_work, dcin_aicl_work);
	INIT_WORK(&chg->cp_status_change_work, smblib_cp_status_change_work);
	INIT_DELAYED_WORK(&chg->clear_hdc_work, clear_hdc_work);
	INIT_DELAYED_WORK(&chg->icl_change_work, smblib_icl_change_work);
	INIT_DELAYED_WORK(&chg->pl_enable_work, smblib_pl_enable_work);
	INIT_DELAYED_WORK(&chg->uusb_otg_work, smblib_uusb_otg_work);
	INIT_DELAYED_WORK(&chg->bb_removal_work, smblib_bb_removal_work);
	INIT_DELAYED_WORK(&chg->lpd_ra_open_work, smblib_lpd_ra_open_work);
	INIT_DELAYED_WORK(&chg->lpd_detach_work, smblib_lpd_detach_work);
	INIT_DELAYED_WORK(&chg->thermal_regulation_work,
					smblib_thermal_regulation_work);
	INIT_DELAYED_WORK(&chg->usbov_dbc_work, smblib_usbov_dbc_work);
	INIT_DELAYED_WORK(&chg->pr_swap_detach_work,
					smblib_pr_swap_detach_work);
	INIT_DELAYED_WORK(&chg->pr_lock_clear_work,
					smblib_pr_lock_clear_work);
	timer_setup(&chg->apsd_timer, apsd_timer_cb, 0);
#ifdef CONFIG_PRODUCT_MOBA
	INIT_DELAYED_WORK(&chg->lenovo_monitor_ports_status_work,
					lenovo_monitor_ports_status_work);
	INIT_DELAYED_WORK(&chg->lenovo_battery_monitor_work,
					lenovo_battery_monitor_work);
	INIT_DELAYED_WORK(&chg->lenovo_thermal_monitor_work,
					lenovo_thermal_monitor_work);
	INIT_DELAYED_WORK(&chg->lenovo_charge_monitor_work,
					lenovo_charge_monitor_work);
	INIT_DELAYED_WORK(&chg->lenovo_insert_delay_work,
					lenovo_insert_delay_work);
	INIT_DELAYED_WORK(&chg->lenovo_bootup_delay_work,
					lenovo_bootup_delay_work);
	INIT_DELAYED_WORK(&chg->lenovo_ffc_ctrl_work,
					lenovo_ffc_ctrl_workfunc);
	INIT_DELAYED_WORK(&chg->lenovo_icl_settled_work,
					lenovo_icl_settled_work);
	INIT_DELAYED_WORK(&chg->lenovo_restart_pd_work,
					lenovo_restart_pd_work);

	schedule_delayed_work(&chg->lenovo_ffc_ctrl_work,
			msecs_to_jiffies(5000));

#endif

	if (chg->wa_flags & CHG_TERMINATION_WA) {
		INIT_WORK(&chg->chg_termination_work,
					smblib_chg_termination_work);

		if (alarmtimer_get_rtcdev()) {
			alarm_init(&chg->chg_termination_alarm, ALARM_BOOTTIME,
						chg_termination_alarm_cb);
		} else {
			smblib_err(chg, "Couldn't get rtc device\n");
			return -ENODEV;
		}
	}

	if (chg->uusb_moisture_protection_enabled) {
		INIT_WORK(&chg->moisture_protection_work,
					smblib_moisture_protection_work);

		if (alarmtimer_get_rtcdev()) {
			alarm_init(&chg->moisture_protection_alarm,
				ALARM_BOOTTIME, moisture_protection_alarm_cb);
		} else {
			smblib_err(chg, "Failed to initialize moisture protection alarm\n");
			return -ENODEV;
		}
	}

	if (alarmtimer_get_rtcdev()) {
		alarm_init(&chg->dcin_aicl_alarm, ALARM_REALTIME,
				dcin_aicl_alarm_cb);
	} else {
		smblib_err(chg, "Failed to initialize dcin aicl alarm\n");
		return -ENODEV;
	}

	chg->fake_capacity = -EINVAL;
	chg->fake_input_current_limited = -EINVAL;
	chg->fake_batt_status = -EINVAL;
	chg->sink_src_mode = UNATTACHED_MODE;
	chg->jeita_configured = false;
	chg->sec_chg_selected = POWER_SUPPLY_CHARGER_SEC_NONE;
	chg->cp_reason = POWER_SUPPLY_CP_NONE;
	chg->thermal_status = TEMP_BELOW_RANGE;
	chg->typec_irq_en = true;
	chg->cp_topo = -EINVAL;

	switch (chg->mode) {
	case PARALLEL_MASTER:
		rc = qcom_batt_init(&chg->chg_param);
		if (rc < 0) {
			smblib_err(chg, "Couldn't init qcom_batt_init rc=%d\n",
				rc);
			return rc;
		}

		rc = qcom_step_chg_init(chg->dev, chg->step_chg_enabled,
						chg->sw_jeita_enabled, false);
		if (rc < 0) {
			smblib_err(chg, "Couldn't init qcom_step_chg_init rc=%d\n",
				rc);
			return rc;
		}

		rc = smblib_create_votables(chg);
		if (rc < 0) {
			smblib_err(chg, "Couldn't create votables rc=%d\n",
				rc);
			return rc;
		}

		chg->bms_psy = power_supply_get_by_name("bms");

		if (chg->sec_pl_present) {
			chg->pl.psy = power_supply_get_by_name("parallel");
			if (chg->pl.psy) {
				if (chg->sec_chg_selected
					!= POWER_SUPPLY_CHARGER_SEC_CP) {
					rc = smblib_select_sec_charger(chg,
						POWER_SUPPLY_CHARGER_SEC_PL,
						POWER_SUPPLY_CP_NONE, false);
					if (rc < 0)
						smblib_err(chg, "Couldn't config pl charger rc=%d\n",
							rc);
				}

				if (chg->smb_temp_max == -EINVAL) {
					rc = smblib_get_thermal_threshold(chg,
						SMB_REG_H_THRESHOLD_MSB_REG,
						&chg->smb_temp_max);
					if (rc < 0) {
						dev_err(chg->dev, "Couldn't get charger_temp_max rc=%d\n",
								rc);
						return rc;
					}
				}

				prop_val.intval = chg->smb_temp_max;
				rc = power_supply_set_property(chg->pl.psy,
					POWER_SUPPLY_PROP_CHARGER_TEMP_MAX,
					&prop_val);
				if (rc < 0) {
					dev_err(chg->dev, "Couldn't set POWER_SUPPLY_PROP_CHARGER_TEMP_MAX rc=%d\n",
							rc);
					return rc;
				}
			}
		}

		rc = smblib_register_notifier(chg);
		if (rc < 0) {
			smblib_err(chg,
				"Couldn't register notifier rc=%d\n", rc);
			return rc;
		}
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

#ifdef CONFIG_PRODUCT_MOBA
	schedule_delayed_work(&chg->lenovo_battery_monitor_work,
				msecs_to_jiffies(1500));
	schedule_delayed_work(&chg->lenovo_thermal_monitor_work,
				msecs_to_jiffies(2000));
	schedule_delayed_work(&chg->lenovo_charge_monitor_work,
				msecs_to_jiffies(6000));
	schedule_delayed_work(&chg->lenovo_bootup_delay_work,
				msecs_to_jiffies(40000));
#endif
	return rc;
}

int smblib_deinit(struct smb_charger *chg)
{
	switch (chg->mode) {
	case PARALLEL_MASTER:
		if (chg->uusb_moisture_protection_enabled) {
			alarm_cancel(&chg->moisture_protection_alarm);
			cancel_work_sync(&chg->moisture_protection_work);
		}
		if (chg->wa_flags & CHG_TERMINATION_WA) {
			alarm_cancel(&chg->chg_termination_alarm);
			cancel_work_sync(&chg->chg_termination_work);
		}
		del_timer_sync(&chg->apsd_timer);
		cancel_work_sync(&chg->bms_update_work);
		cancel_work_sync(&chg->jeita_update_work);
		cancel_work_sync(&chg->pl_update_work);
		cancel_work_sync(&chg->dcin_aicl_work);
		cancel_work_sync(&chg->cp_status_change_work);
		cancel_delayed_work_sync(&chg->clear_hdc_work);
		cancel_delayed_work_sync(&chg->icl_change_work);
		cancel_delayed_work_sync(&chg->pl_enable_work);
		cancel_delayed_work_sync(&chg->uusb_otg_work);
		cancel_delayed_work_sync(&chg->bb_removal_work);
		cancel_delayed_work_sync(&chg->lpd_ra_open_work);
		cancel_delayed_work_sync(&chg->lpd_detach_work);
		cancel_delayed_work_sync(&chg->thermal_regulation_work);
		cancel_delayed_work_sync(&chg->usbov_dbc_work);
		cancel_delayed_work_sync(&chg->pr_swap_detach_work);
		power_supply_unreg_notifier(&chg->nb);
		smblib_destroy_votables(chg);
		qcom_step_chg_deinit();
		qcom_batt_deinit();
		break;
	case PARALLEL_SLAVE:
		break;
	default:
		smblib_err(chg, "Unsupported mode %d\n", chg->mode);
		return -EINVAL;
	}

	smblib_iio_deinit(chg);

	return 0;
}
