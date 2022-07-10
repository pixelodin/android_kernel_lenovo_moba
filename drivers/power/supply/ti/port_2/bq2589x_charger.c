/*
 * BQ2589x battery charging driver
 *
 * Copyright (C) 2013 Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * This package demos the architecture of bq2589x driver used for linux
 * based system. It also contains the code to tune the output of an adjustable
 * adjust high voltage adapter(AHVDCP) dynamically to achieve better charging efficiency.
 *									[DISCLAIMER]
 * The code is designed for DEMO purpose only, and may be subject to change
 * for any bug fix or improvement without proir notice.
 * TI could offer help to port and debug customer code derived from the demo code,
 * but it is customer's responsibility to assure the code quality and reliability
 * to meet their application requirement.
 */

#define pr_fmt(fmt)	"bq2589x: %s: " fmt, __func__
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/err.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/debugfs.h>
#include <linux/bitops.h>
#include <linux/math64.h>
#include <linux/alarmtimer.h>
#include "bq2589x_reg.h"
#include "bq2589x_temp_map.h"

#define DBG_FS
//#undef	DBG_FS
#ifdef CONFIG_PRODUCT_MOBA
#define SUPPORT_ONSEMI_PDCONTROL
#define BATT_ERROR_READ_TEMP				915
#define BATT_ERROR_DEFAULT_TEMP				-400
#define BATT_NTC_TIMEOUT				30000  // 30s
static int last_temp = BATT_ERROR_DEFAULT_TEMP;
unsigned long ntc_jiffies = -1;
extern u8 g_chg_enabled;
static struct bq2589x *bq2;
static charger_recheck = 0;
extern u8 insert_delay_done;
extern u8 bootup_delay_done;
extern int force_usb_suspend;
extern int fac_chip_read_enabled;
static int fac_charging_enabled;
static int fac_input_suspend;
extern u8 is_fake_battery;
static int is_shutdown = 0;
#endif

#ifdef SUPPORT_ONSEMI_PDCONTROL
#include "pd_policy_manager.h"
extern struct usbpd_pm *bq_usbpd_pm;
bool sec_port_online;

/*USBIN CURRENT */
#define USB_BQ2589X_USBIN_CURRENT		100
#define DCP_BQ2589X_USBIN_CURRENT		100
#define CDP_BQ2589X_USBIN_CURRENT		100
#define HVDCP_BQ2589X_USBIN_CURRENT		100

/*BATT FCC CURRENT*/
#define FCC_BQ2589X_TYPE_USB_CURRENT		250
#define FCC_BQ2589X_TYPE_DCP_CURRENT		250
#define FCC_BQ2589X_TYPE_CDP_CURRENT		250
#define FCC_BQ2589X_TYPE_HVDCP_CURRENT		500
bool type_detect_done;
#endif

enum bq2589x_vbus_type {
	BQ2589X_VBUS_NONE,
	BQ2589X_VBUS_USB_SDP,
	BQ2589X_VBUS_USB_CDP,
	BQ2589X_VBUS_USB_DCP,
	BQ2589X_VBUS_MAXC,
	BQ2589X_VBUS_UNKNOWN,
	BQ2589X_VBUS_NONSTAND,
	BQ2589X_VBUS_OTG,
	BQ2589X_VBUS_TYPE_NUM,
};

enum bq2589x_part_no {
	BQ25890 = 0x03,
};

/* availabe on bq25898D bq25890H
enum bq2589x_dp_volt {
	BQ2589X_DP_HIZ,
	BQ2589X_DP_0MV,
	BQ2589X_DP_600MV,
	BQ2589X_DP_1200MV,
	BQ2589X_DP_2000MV,
	BQ2589X_DP_2700MV,
	BQ2589X_DP_3300MV,
	BQ2589X_DP_SHORT
};
enum bq2589x_dm_volt {
	BQ2589X_DM_HIZ,
	BQ2589X_DM_0MV,
	BQ2589X_DM_600MV,
	BQ2589X_DM_1200MV,
	BQ2589X_DM_2000MV,
	BQ2589X_DM_2700MV,
	BQ2589X_DM_3300MV,
	BQ2589X_DM_3300MV2,
};
*/
enum {
	USER		= BIT(0),
	JEITA		= BIT(1),
	BATT_FC		= BIT(2),
	BATT_PRES	= BIT(3),
	BATT_TUNE	= BIT(4),
	SYS_OFF		= BIT(5),
};

enum wakeup_src {
	WAKEUP_SRC_MONITOR = 0,
	WAKEUP_SRC_JEITA,
	WAKEUP_SRC_MAX,
};

enum bq2589x_charge_state {
	CHARGE_STATE_IDLE = BQ2589X_CHRG_STAT_IDLE,
	CHARGE_STATE_PRECHG = BQ2589X_CHRG_STAT_PRECHG,
	CHARGE_STATE_FASTCHG = BQ2589X_CHRG_STAT_FASTCHG,
	CHARGE_STATE_CHGDONE = BQ2589X_CHRG_STAT_CHGDONE,
};

#define WAKEUP_SRC_MASK (~(~0 << WAKEUP_SRC_MAX))
struct bq2589x_wakeup_source {
	struct wakeup_source source;
	unsigned long enabled_bitmap;
	spinlock_t ws_lock;
};

struct bq2589x_otg_regulator {
	struct regulator_desc	rdesc;
	struct regulator_dev	*rdev;
};

struct bq2589x_config {
	bool enable_auto_dpdm;
	bool enable_12v;
	bool enable_hvdcp;
	bool enable_maxc;

	int	charge_voltage;
	int	charge_current;
	int boost_voltage;
	int boost_current;

	bool enable_term;
	int	term_current;

	bool enable_ico;
	bool use_absolute_vindpm;

	int dpdm_sw_gpio;
	int usb_id_gpio;
};


struct bq2589x {
	struct device *dev;
	struct i2c_client *client;

	enum bq2589x_part_no part_no;
	int revision;

	struct bq2589x_config cfg;


	struct mutex i2c_rw_lock;
	struct mutex charging_disable_lock;
	struct mutex profile_change_lock;
	struct mutex data_lock;
	struct mutex irq_complete;

	struct bq2589x_wakeup_source bq2589x_ws;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	bool usb_present;
	bool charge_enabled;
	bool otg_enabled;

	bool power_good;
	bool vbus_good;

	bool batt_full;
	bool batt_present;

	int	vbus_type;
	int charge_state;
	int charging_disabled_status;

	enum power_supply_type usb_type;

	int chg_ma;
	int chg_mv;
	int icl_ma;
	int ivl_mv;

/* if use software jeita in case of NTC is connected to gauge */
	bool software_jeita_supported;
	bool jeita_active;

	bool batt_hot;
	bool batt_cold;
	bool batt_warm;
	bool batt_cool;

	int batt_hot_degc;
	int batt_warm_degc;
	int batt_cool_degc;
	int batt_cold_degc;
	int hot_temp_hysteresis;
	int cold_temp_hysteresis;

	int batt_cool_ma;
	int batt_warm_ma;
	int batt_cool_mv;
	int batt_warm_mv;

	int batt_temp;

	int jeita_ma;
	int jeita_mv;

	int vbus_volt;
	int vbat_volt;
	int vsys_volt;
	int chg_curr;

	int fault_status;

	int skip_writes;
	int skip_reads;

	struct delayed_work ico_work;
	struct delayed_work pe_work;

	struct delayed_work discharge_jeita_work;
	struct delayed_work charge_jeita_work;
#ifdef CONFIG_PRODUCT_MOBA
	struct delayed_work charger_recheck_work;
#endif

	struct alarm jeita_alarm;

	struct dentry *debug_root;

	struct bq2589x_otg_regulator otg_vreg;

	//struct power_supply batt_psy;
	struct power_supply_desc batt_psy;
	struct power_supply_config batt_cfg;
	struct power_supply *fc_main_psy;
#ifdef CONFIG_PRODUCT_MOBA
	struct power_supply *qcom_usb_psy;
#endif
	const struct regulator_desc *batt_init_desc;

};

struct pe_ctrl {
	bool enable;
	bool tune_up;
	bool tune_down;
	bool tune_done;
	bool tune_fail;
	int  tune_count;
	int  target_volt;
	int	 high_volt_level;/* vbus volt > this threshold means tune up successfully */
	int  low_volt_level; /* vbus volt < this threshold means tune down successfully */
	int  vbat_min_volt;  /* to tune up voltage only when vbat > this threshold */
};

static struct pe_ctrl pe;

static int bq2589x_update_charging_profile(struct bq2589x *bq);

static int __bq2589x_read_byte(struct bq2589x *bq, u8 reg, u8 *data)
{
	int ret;

#ifdef CONFIG_PRODUCT_MOBA
	if (!is_shutdown) {
		if (is_fake_battery && (!fac_chip_read_enabled))
			return 0;
	}
#endif
	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		pr_err("i2c read fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*data = (u8)ret;

	return 0;

}

static int __bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
	int ret;

#ifdef CONFIG_PRODUCT_MOBA
	if (!is_shutdown) {
		if (is_fake_battery && (!fac_chip_read_enabled))
			return 0;
	}
#endif
	ret = i2c_smbus_write_byte_data(bq->client, reg, data);
	if (ret < 0) {
		pr_err("i2c write fail: can't write 0x%02X to reg 0x%02X: %d\n",
				data, reg, ret);
		return ret;
	}
	return 0;

}

static int bq2589x_read_byte(struct bq2589x *bq, u8 *data, u8 reg)
{
	int ret;

#ifdef CONFIG_PRODUCT_MOBA
	if (!is_shutdown) {
		if (is_fake_battery && (!fac_chip_read_enabled))
			return 0;
	}
#endif
	if (bq->skip_reads) {
		*data = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2589x_read_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq2589x_write_byte(struct bq2589x *bq, u8 reg, u8 data)
{
	int ret;

#ifdef CONFIG_PRODUCT_MOBA
	if (!is_shutdown) {
		if (is_fake_battery && (!fac_chip_read_enabled))
			return 0;
	}
#endif
	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq2589x_write_byte(bq, reg, data);
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

static int bq2589x_update_bits(struct bq2589x *bq, u8 reg, u8 mask, u8 data)
{
	int ret;
	u8 tmp;

#ifdef CONFIG_PRODUCT_MOBA
	if (!is_shutdown) {
		if (is_fake_battery && (!fac_chip_read_enabled))
			return 0;
	}
#endif
	if (bq->skip_reads || bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);

	ret = __bq2589x_read_byte(bq, reg, &tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, reg=%d\n", reg, ret);
		goto out;
	}

	tmp &= ~mask;
	tmp |= data & mask;

	ret = __bq2589x_write_byte(bq, reg, tmp);
	if (ret) {
		pr_err("Failed: reg=%02X, reg=%d\n", reg, ret);
	}

out:
	mutex_unlock(&bq->i2c_rw_lock);
	return ret;
}

#if 0
static void bq2589x_stay_awake(struct bq2589x_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);

	if (!__test_and_set_bit(wk_src, &source->enabled_bitmap)) {
		__pm_stay_awake(&source->source);
		pr_debug("enabled source %s, wakeup_src %d\n",
			source->source.name, wk_src);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);
}

static void bq2589x_relax(struct bq2589x_wakeup_source *source,
	enum wakeup_src wk_src)
{
	unsigned long flags;

	spin_lock_irqsave(&source->ws_lock, flags);
	if (__test_and_clear_bit(wk_src, &source->enabled_bitmap) &&
		!(source->enabled_bitmap & WAKEUP_SRC_MASK)) {
		__pm_relax(&source->source);
		pr_debug("disabled source %s\n", source->source.name);
	}
	spin_unlock_irqrestore(&source->ws_lock, flags);

	pr_debug("relax source %s, wakeup_src %d\n",
		source->source.name, wk_src);
}
#endif
static void bq2589x_wakeup_src_init(struct bq2589x *bq)
{
	spin_lock_init(&bq->bq2589x_ws.ws_lock);
	wakeup_source_init(&bq->bq2589x_ws.source, "bq2589x");
}

#if 0
static enum bq2589x_vbus_type bq2589x_get_vbus_type(struct bq2589x *bq)
{
	u8 val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0)
		return 0;
	val &= BQ2589X_VBUS_STAT_MASK;
	val >>= BQ2589X_VBUS_STAT_SHIFT;

	return val;
}
#endif

static int bq2589x_enable_otg(struct bq2589x *bq)
{
	u8 val = BQ2589X_OTG_ENABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
				   BQ2589X_OTG_CONFIG_MASK, val);

}

static int bq2589x_disable_otg(struct bq2589x *bq)
{
	u8 val = BQ2589X_OTG_DISABLE << BQ2589X_OTG_CONFIG_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
				   BQ2589X_OTG_CONFIG_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_disable_otg);

static int bq2589x_set_otg_volt(struct bq2589x *bq, int volt)
{
	u8 val = 0;

	if (volt < BQ2589X_BOOSTV_BASE)
		volt = BQ2589X_BOOSTV_BASE;
	if (volt > BQ2589X_BOOSTV_BASE
			+ (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT)
			* BQ2589X_BOOSTV_LSB)
		volt = BQ2589X_BOOSTV_BASE
			+ (BQ2589X_BOOSTV_MASK >> BQ2589X_BOOSTV_SHIFT)
			* BQ2589X_BOOSTV_LSB;

	val = ((volt - BQ2589X_BOOSTV_BASE) / BQ2589X_BOOSTV_LSB)
			<< BQ2589X_BOOSTV_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_0A,
				BQ2589X_BOOSTV_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_volt);

static int bq2589x_set_otg_current(struct bq2589x *bq, int curr)
{
	u8 temp;

	if (curr == 500)
		temp = BQ2589X_BOOST_LIM_500MA;
	else if (curr == 700)
		temp = BQ2589X_BOOST_LIM_700MA;
	else if (curr == 1100)
		temp = BQ2589X_BOOST_LIM_1100MA;
	else if (curr == 1600)
		temp = BQ2589X_BOOST_LIM_1600MA;
	else if (curr == 1800)
		temp = BQ2589X_BOOST_LIM_1800MA;
	else if (curr == 2100)
		temp = BQ2589X_BOOST_LIM_2100MA;
	else if (curr == 2400)
		temp = BQ2589X_BOOST_LIM_2400MA;
	else
		temp = BQ2589X_BOOST_LIM_1300MA;

	return bq2589x_update_bits(bq, BQ2589X_REG_0A,
				BQ2589X_BOOST_LIM_MASK,
				temp << BQ2589X_BOOST_LIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg_current);


static int bq2589x_enable_hvdcp(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_HVDCP_ENABLE << BQ2589X_HVDCPEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
				BQ2589X_HVDCPEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_hvdcp);

static int bq2589x_disable_hvdcp(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_HVDCP_DISABLE << BQ2589X_HVDCPEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
				BQ2589X_HVDCPEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_hvdcp);

static int bq2589x_enable_maxc(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_MAXC_ENABLE << BQ2589X_MAXCEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
				BQ2589X_MAXCEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_maxc);

static int bq2589x_disable_maxc(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_MAXC_DISABLE << BQ2589X_MAXCEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
				BQ2589X_MAXCEN_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_maxc);


static int bq2589x_enable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_ENABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03,
				BQ2589X_CHG_CONFIG_MASK, val);
	return ret;
}

static int bq2589x_disable_charger(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_CHG_DISABLE << BQ2589X_CHG_CONFIG_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_03,
				BQ2589X_CHG_CONFIG_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_disable_charger);

static int bq2589x_get_charge_enable(struct bq2589x *bq)
{
	int ret;
	u8 val;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_03);
	if (!ret)
		bq->charge_enabled = !!(val & BQ2589X_CHG_CONFIG_MASK);
	return ret;
}

int bq2589x_adc_start(struct bq2589x *bq, bool oneshot)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_02);
	if (ret < 0) {
		dev_err(bq->dev, "%s failed to read register 0x02:%d\n", __func__, ret);
		return ret;
	}

	if (((val & BQ2589X_CONV_RATE_MASK) >> BQ2589X_CONV_RATE_SHIFT) == BQ2589X_ADC_CONTINUE_ENABLE)
		return 0; /*is doing continuous scan*/
	if (oneshot)
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_START_MASK,
					BQ2589X_CONV_START << BQ2589X_CONV_START_SHIFT);
	else
		ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK,
					BQ2589X_ADC_CONTINUE_ENABLE << BQ2589X_CONV_RATE_SHIFT);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_adc_start);

int bq2589x_adc_stop(struct bq2589x *bq)
{
	return bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_CONV_RATE_MASK,
				BQ2589X_ADC_CONTINUE_DISABLE << BQ2589X_CONV_RATE_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_adc_stop);


int bq2589x_adc_read_battery_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0E);
	if (ret < 0) {
		dev_err(bq->dev, "read battery voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ2589X_BATV_BASE + ((val & BQ2589X_BATV_MASK) >> BQ2589X_BATV_SHIFT) * BQ2589X_BATV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_battery_volt);


int bq2589x_adc_read_sys_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0F);
	if (ret < 0) {
		dev_err(bq->dev, "read system voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ2589X_SYSV_BASE + ((val & BQ2589X_SYSV_MASK) >> BQ2589X_SYSV_SHIFT) * BQ2589X_SYSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_sys_volt);

int bq2589x_adc_read_vbus_volt(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_11);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else{
		volt = BQ2589X_VBUSV_BASE + ((val & BQ2589X_VBUSV_MASK) >> BQ2589X_VBUSV_SHIFT) * BQ2589X_VBUSV_LSB ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_vbus_volt);

static int bq_vadc_map_voltage_temp(const struct bq_temp_map *pts,
				      u32 tablesize, s32 input, s64 *output)
{
	bool descending = 1;
	u32 i = 0;

	if (!pts)
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].x < pts[1].x)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending) && (pts[i].x < input)) {
			/* table entry is less than measured*/
			 /* value and table is descending, stop */
			break;
		} else if ((!descending) &&
				(pts[i].x > input)) {
			/* table entry is greater than measured*/
			/*value and table is ascending, stop */
			break;
		}
		i++;
	}

	if (i == 0) {
		*output = pts[0].y;
	} else if (i == tablesize) {
		*output = pts[tablesize - 1].y;
	} else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((s32)((pts[i].y - pts[i - 1].y) *
			(input - pts[i - 1].x)) /
			(pts[i].x - pts[i - 1].x)) +
			pts[i - 1].y);
	}

	return 0;
}

static void bq2589x_dump_reg2(struct bq2589x *bq);
int bq2589x_adc_read_temperature(struct bq2589x *bq)
{
	uint8_t val;
	int temp;
	int ret;
	s64 value = 0, result = 0;

#ifdef CONFIG_PRODUCT_MOBA
	if (!is_shutdown) {
		if (is_fake_battery) {
			dev_err(bq->dev, "fake battery temp is %d\n", 250);
			return 250;
		}
	}
#endif
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_10);
	if (ret < 0) {
		dev_err(bq->dev, "read temperature failed :%d\n", ret);
		return ret;
	} else{
		value = BQ2589X_TSPCT_BASE + ((val & BQ2589X_TSPCT_MASK) >> BQ2589X_TSPCT_SHIFT) * BQ2589X_TSPCT_LSB ;
		value = 21 * 432 * value  * 10000 / (210000000 - 2532 * value);

	        ret = bq_vadc_map_voltage_temp(adcmap_100k_104ef_104fb_1875_vref,
				ARRAY_SIZE(adcmap_100k_104ef_104fb_1875_vref),
				value, &result);
		if (ret) {
			dev_err(bq->dev, "error: vadc_map_temp, reg_10 :0x%02X, value :%lld, ret :%d, temp :%d\n", val, value, ret, result);
			return ret;
		}

		temp = result;
#ifdef CONFIG_PRODUCT_MOBA
		if (temp == BATT_ERROR_READ_TEMP) {
			if (ntc_jiffies == -1) {
				temp = last_temp;
				ntc_jiffies = jiffies;
			} else if (time_before(jiffies, ntc_jiffies + msecs_to_jiffies(BATT_NTC_TIMEOUT))) {
				temp = last_temp;
			} else {
				temp = BATT_ERROR_DEFAULT_TEMP;
			}
		} else {
			last_temp = temp;
			ntc_jiffies = -1;
		}

		if (result == 915) {
			dev_err(bq->dev, "reg_10 :0x%02X, value :%lld, ret :%d, result: %d, temp :%d, last :%d\n",
					val, value, ret, result, temp, last_temp);
			bq2589x_dump_reg2(bq);
		}
#endif
		if (temp <= 100)
			temp = temp - 15;
		else if (temp <= 110)
			temp = temp - 15;
		else if (temp <= 120)
			temp = temp - 14;
		else if (temp <= 130)
			temp = temp - 13;
		else if (temp <= 140)
			temp = temp - 11;
		else if (temp <= 150)
			temp = temp - 11;
		else if (temp <= 160)
			temp = temp - 10;
		else if (temp <= 170)
			temp = temp - 9;
		else if (temp <= 180)
			temp = temp - 7;
		else if (temp <= 200)
			temp = temp - 5;
		else if (temp <= 300)
			temp = temp - 3;
		else if (temp <= 310)
			temp = temp - 2;
		else if (temp <= 450)
			temp = temp + 1;
		else if (temp <= 600)
			temp = temp + 1;

		return temp;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_temperature);

int bq2589x_adc_read_charge_current(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_12);
	if (ret < 0) {
		dev_err(bq->dev, "read charge current failed :%d\n", ret);
		return ret;
	} else{
		volt = (int)(BQ2589X_ICHGR_BASE + ((val & BQ2589X_ICHGR_MASK) >> BQ2589X_ICHGR_SHIFT) * BQ2589X_ICHGR_LSB) ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_adc_read_charge_current);

int bq2589x_read_config_icl(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_00);
	if (ret < 0) {
		dev_err(bq->dev, "read config icl failed :%d\n", ret);
		return ret;
	} else{
		volt = (int)(BQ2589X_IINLIM_BASE + ((val & BQ2589X_IINLIM_MASK) >> BQ2589X_IINLIM_SHIFT) * BQ2589X_IINLIM_LSB) ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_read_config_icl);

int bq2589x_read_config_fcc(struct bq2589x *bq)
{
	uint8_t val;
	int volt;
	int ret;
	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_04);
	if (ret < 0) {
		dev_err(bq->dev, "read config fcc failed :%d\n", ret);
		return ret;
	} else{
		volt = (int)(BQ2589X_ICHG_BASE + ((val & BQ2589X_ICHG_MASK) >> BQ2589X_ICHG_SHIFT) * BQ2589X_ICHG_LSB) ;
		return volt;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_read_config_fcc);

int bq2589x_set_chargecurrent(struct bq2589x *bq, int curr)
{
	u8 ichg;

	ichg = (curr - BQ2589X_ICHG_BASE)/BQ2589X_ICHG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_04,
						BQ2589X_ICHG_MASK, ichg << BQ2589X_ICHG_SHIFT);

}
EXPORT_SYMBOL_GPL(bq2589x_set_chargecurrent);

int bq2589x_set_term_current(struct bq2589x *bq, int curr)
{
	u8 iterm;

	iterm = (curr - BQ2589X_ITERM_BASE) / BQ2589X_ITERM_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05,
						BQ2589X_ITERM_MASK, iterm << BQ2589X_ITERM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_term_current);


int bq2589x_set_prechg_current(struct bq2589x *bq, int curr)
{
	u8 iprechg;

	iprechg = (curr - BQ2589X_IPRECHG_BASE) / BQ2589X_IPRECHG_LSB;

	return bq2589x_update_bits(bq, BQ2589X_REG_05,
						BQ2589X_IPRECHG_MASK, iprechg << BQ2589X_IPRECHG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_prechg_current);

int bq2589x_set_chargevoltage(struct bq2589x *bq, int volt)
{
	u8 val;

	val = (volt - BQ2589X_VREG_BASE)/BQ2589X_VREG_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_06,
						BQ2589X_VREG_MASK, val << BQ2589X_VREG_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_chargevoltage);


int bq2589x_set_input_volt_limit(struct bq2589x *bq, int volt)
{
	u8 val;
	val = (volt - BQ2589X_VINDPM_BASE) / BQ2589X_VINDPM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_0D,
						BQ2589X_VINDPM_MASK, val << BQ2589X_VINDPM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_volt_limit);

int bq2589x_set_input_current_limit(struct bq2589x *bq, int curr)
{
	u8 val;

	val = (curr - BQ2589X_IINLIM_BASE) / BQ2589X_IINLIM_LSB;
	return bq2589x_update_bits(bq, BQ2589X_REG_00, BQ2589X_IINLIM_MASK,
						val << BQ2589X_IINLIM_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_input_current_limit);

/*
int bq2589x_set_vindpm_offset(struct bq2589x *bq, int offset)
{
	u8 val;
	if (offset == 400)
		val = BQ2589X_VINDPMOS_400MV;
	else
		val = BQ2589X_VINDPMOS_600MV;
	return bq2589x_update_bits(bq, BQ2589X_REG_01, BQ2589X_VINDPMOS_MASK,
						val << BQ2589X_VINDPMOS_SHIFT);
}
EXPORT_SYMBOL_GPL(bq2589x_set_vindpm_offset);
*/

int bq2589x_get_charging_status(struct bq2589x *bq)
{
	u8 val = 0;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		pr_err("Failed to read register 0x0b:%d\n",ret);
		return ret;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	return val;
}
EXPORT_SYMBOL_GPL(bq2589x_get_charging_status);

void bq2589x_set_otg(struct bq2589x *bq, bool enable)
{
	int ret;

	if (enable)
		ret = bq2589x_enable_otg(bq);
	else
		ret = bq2589x_disable_otg(bq);

	if (!ret)
		bq->otg_enabled = enable;
	else
		dev_err(bq->dev, "%s:Failed to %s otg:%d\n", __func__,
						enable ? "enable" : "disable", ret);
}
EXPORT_SYMBOL_GPL(bq2589x_set_otg);

int bq2589x_set_watchdog_timer(struct bq2589x *bq, u8 timeout)
{
	u8 val;

	val = (timeout - BQ2589X_WDT_BASE) / BQ2589X_WDT_LSB;
	val <<= BQ2589X_WDT_SHIFT;
	return bq2589x_update_bits(bq, BQ2589X_REG_07,
						BQ2589X_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_set_watchdog_timer);

int bq2589x_disable_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_DISABLE << BQ2589X_WDT_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_07,
						BQ2589X_WDT_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_disable_watchdog_timer);

int bq2589x_reset_watchdog_timer(struct bq2589x *bq)
{
	u8 val = BQ2589X_WDT_RESET << BQ2589X_WDT_RESET_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_03,
						BQ2589X_WDT_RESET_MASK, val);
}
EXPORT_SYMBOL_GPL(bq2589x_reset_watchdog_timer);

int bq2589x_force_dpdm(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_FORCE_DPDM << BQ2589X_FORCE_DPDM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
						BQ2589X_FORCE_DPDM_MASK, val);

	pr_info("Force DPDM %s\n", !ret ? "successfully" : "failed");

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_force_dpdm);

int bq2589x_reset_chip(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_RESET << BQ2589X_RESET_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_14,
						BQ2589X_RESET_MASK, val);
	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_reset_chip);

int bq2589x_enter_ship_mode(struct bq2589x *bq)
{
	int ret;
	u8 val = BQ2589X_BATFET_OFF << BQ2589X_BATFET_DIS_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09,
						BQ2589X_BATFET_DIS_MASK, val);
	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enter_ship_mode);

int bq2589x_enter_hiz_mode(struct bq2589x *bq)
{
	u8 val = BQ2589X_HIZ_ENABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00,
						BQ2589X_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_enter_hiz_mode);

int bq2589x_exit_hiz_mode(struct bq2589x *bq)
{

	u8 val = BQ2589X_HIZ_DISABLE << BQ2589X_ENHIZ_SHIFT;

	return bq2589x_update_bits(bq, BQ2589X_REG_00,
						BQ2589X_ENHIZ_MASK, val);

}
EXPORT_SYMBOL_GPL(bq2589x_exit_hiz_mode);

int bq2589x_get_hiz_mode(struct bq2589x *bq, u8 *state)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_00);
	if (ret)
		return ret;
	*state = (val & BQ2589X_ENHIZ_MASK) >> BQ2589X_ENHIZ_SHIFT;

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_get_hiz_mode);

int bq2589x_pumpx_enable(struct bq2589x *bq, int enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_PUMPX_ENABLE << BQ2589X_EN_PUMPX_SHIFT;
	else
		val = BQ2589X_PUMPX_DISABLE << BQ2589X_EN_PUMPX_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_04,
						BQ2589X_EN_PUMPX_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_enable);

int bq2589x_pumpx_increase_volt(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_PUMPX_UP << BQ2589X_PUMPX_UP_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09,
						BQ2589X_PUMPX_UP_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt);

int bq2589x_pumpx_increase_volt_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_UP_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx up finished*/

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_increase_volt_done);

int bq2589x_pumpx_decrease_volt(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_PUMPX_DOWN << BQ2589X_PUMPX_DOWN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09,
						BQ2589X_PUMPX_DOWN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt);

int bq2589x_pumpx_decrease_volt_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_09);
	if (ret)
		return ret;

	if (val & BQ2589X_PUMPX_DOWN_MASK)
		return 1;   /* not finished*/
	else
		return 0;   /* pumpx down finished*/

}
EXPORT_SYMBOL_GPL(bq2589x_pumpx_decrease_volt_done);

static int bq2589x_force_ico(struct bq2589x *bq)
{
	u8 val;
	int ret;

	val = BQ2589X_FORCE_ICO << BQ2589X_FORCE_ICO_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_09,
						BQ2589X_FORCE_ICO_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_force_ico);

static int bq2589x_check_ico_done(struct bq2589x *bq)
{
	u8 val;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_14);
	if (ret)
		return ret;

	if (val & BQ2589X_ICO_OPTIMIZED_MASK)
		return 1;  /*finished*/
	else
		return 0;   /* in progress*/
}
EXPORT_SYMBOL_GPL(bq2589x_check_ico_done);

static int bq2589x_enable_term(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_TERM_ENABLE << BQ2589X_EN_TERM_SHIFT;
	else
		val = BQ2589X_TERM_DISABLE << BQ2589X_EN_TERM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_07,
						BQ2589X_EN_TERM_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_term);

static int bq2589x_enable_12v_handshake(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_ENABLE_12V << BQ2589X_EN12V_SHIFT;
	else
		val = BQ2589X_DISABLE_12V << BQ2589X_EN12V_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_01,
						BQ2589X_EN12V_MASK, val);

	return ret;
}
EXPORT_SYMBOL_GPL(bq2589x_enable_12v_handshake);

static int bq2589x_enable_auto_dpdm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_AUTO_DPDM_ENABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;
	else
		val = BQ2589X_AUTO_DPDM_DISABLE << BQ2589X_AUTO_DPDM_EN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02,
						BQ2589X_AUTO_DPDM_EN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_auto_dpdm);

static int bq2589x_use_absolute_vindpm(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_FORCE_VINDPM_ENABLE << BQ2589X_FORCE_VINDPM_SHIFT;
	else
		val = BQ2589X_FORCE_VINDPM_DISABLE << BQ2589X_FORCE_VINDPM_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_0D, BQ2589X_FORCE_VINDPM_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_use_absolute_vindpm);

static int bq2589x_enable_ico(struct bq2589x* bq, bool enable)
{
	u8 val;
	int ret;

	if (enable)
		val = BQ2589X_ICO_ENABLE << BQ2589X_ICOEN_SHIFT;
	else
		val = BQ2589X_ICO_DISABLE << BQ2589X_ICOEN_SHIFT;

	ret = bq2589x_update_bits(bq, BQ2589X_REG_02, BQ2589X_ICOEN_MASK, val);

	return ret;

}
EXPORT_SYMBOL_GPL(bq2589x_enable_ico);

static int bq2589x_read_idpm_limit(struct bq2589x *bq)
{
	uint8_t val;
	int curr;
	int ret;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_13);
	if (ret < 0) {
		dev_err(bq->dev, "read vbus voltage failed :%d\n", ret);
		return ret;
	} else{
		curr = BQ2589X_IDPM_LIM_BASE + ((val & BQ2589X_IDPM_LIM_MASK) >> BQ2589X_IDPM_LIM_SHIFT) * BQ2589X_IDPM_LIM_LSB ;
		return curr;
	}
}
EXPORT_SYMBOL_GPL(bq2589x_read_idpm_limit);

#if 0
static bool bq2589x_is_charge_done(struct bq2589x *bq)
{
	int ret;
	u8 val;

	ret = bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	if (ret < 0) {
		dev_err(bq->dev, "%s:read REG0B failed :%d\n", __func__, ret);
		return false;
	}
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;

	return (val == BQ2589X_CHRG_STAT_CHGDONE);
}
EXPORT_SYMBOL_GPL(bq2589x_is_charge_done);
#endif

static int bq2589x_init_device(struct bq2589x *bq)
{
	int ret;

	bq2589x_disable_watchdog_timer(bq);

	if (bq->cfg.enable_hvdcp)
		bq2589x_enable_hvdcp(bq);
	else
		bq2589x_disable_hvdcp(bq);

	if (bq->cfg.enable_maxc)
		bq2589x_enable_maxc(bq);
	else
		bq2589x_disable_maxc(bq);

	bq2589x_enable_12v_handshake(bq, bq->cfg.enable_12v);
	bq2589x_enable_auto_dpdm(bq, bq->cfg.enable_auto_dpdm);
	bq2589x_enable_term(bq, bq->cfg.enable_term);
	bq2589x_enable_ico(bq, bq->cfg.enable_ico);

	/*force use absolute vindpm if auto_dpdm not enabled*/
	if (!bq->cfg.enable_auto_dpdm)
		bq->cfg.use_absolute_vindpm = true;
	bq2589x_use_absolute_vindpm(bq, bq->cfg.use_absolute_vindpm);

	bq->chg_ma = bq->cfg.charge_current;
	bq->chg_mv = bq->cfg.charge_voltage;

/*
 * ret = bq2589x_set_vindpm_offset(bq, 600);
	if (ret < 0) {
		pr_err("Failed to set vindpm offset:%d\n",  ret);
	}
*/
	ret = bq2589x_set_term_current(bq, bq->cfg.term_current);
	if (ret < 0) {
		pr_err("Failed to set termination current:%d\n",  ret);
	}

	ret = bq2589x_set_chargevoltage(bq, bq->cfg.charge_voltage);
	if (ret < 0) {
		pr_err("Failed to set charge voltage:%d\n",  ret);
	}

	ret = bq2589x_set_chargecurrent(bq, bq->cfg.charge_current);
	if (ret < 0) {
		pr_err("Failed to set charge current:%d\n",  ret);
	}

	ret = bq2589x_set_otg_volt(bq, bq->cfg.boost_voltage);
	if (ret < 0) {
		pr_err("Failed to set boost voltage:%d\n", ret);
	}

	ret = bq2589x_set_otg_current(bq, bq->cfg.boost_current);
	if (ret < 0) {
		pr_err("Failed to set boost current:%d\n", ret);
	}

	ret = bq2589x_enable_charger(bq);
	if (ret < 0) {
		pr_err("Failed to enable charger:%d\n",  ret);
		return ret;
	} else {
		bq->charge_enabled = true;
	}

	if (pe.enable) {
		ret = bq2589x_pumpx_enable(bq, true);
		if (ret < 0)
			pr_err("Failed to enable pumpx\n");
	}

	bq2589x_adc_start(bq, false);


	return 0;
}

#if 0
static int bq2589x_charge_status(struct bq2589x *bq)
{
	u8 val = 0;

	bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	switch (val) {
	case BQ2589X_CHRG_STAT_FASTCHG:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case BQ2589X_CHRG_STAT_PRECHG:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case BQ2589X_CHRG_STAT_CHGDONE:
	case BQ2589X_CHRG_STAT_IDLE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}
#endif
static int bq2589x_charging_disable(struct bq2589x *bq, int reason,
						int disable)
{

	int ret = 0;
	int disabled;

	mutex_lock(&bq->charging_disable_lock);

	disabled = bq->charging_disabled_status;

	pr_info("reason=%d requested_disable=%d disabled_status=%d\n",
					reason, disable, disabled);

	if (disable == true)
		disabled |= reason;
	else
		disabled &= ~reason;

	if (disabled && bq->charge_enabled)
		ret = bq2589x_disable_charger(bq);
	else if (!disabled && !bq->charge_enabled)
		ret = bq2589x_enable_charger(bq);

	if (ret) {
		pr_err("Couldn't disable/enable charging for reason=%d ret=%d\n",
							ret, reason);
	} else {
		bq->charging_disabled_status = disabled;
		mutex_lock(&bq->data_lock);
		bq->charge_enabled = !disabled;
		mutex_unlock(&bq->data_lock);
	}
	mutex_unlock(&bq->charging_disable_lock);

	return ret;
}


static void bq2589x_dump_status(struct bq2589x* bq);
static inline bool is_device_suspended(struct bq2589x *bq);

static int bq2589x_get_prop_charge_type(struct bq2589x *bq)
{
	u8 val = 0;
	if (is_device_suspended(bq))
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	bq2589x_read_byte(bq, &val, BQ2589X_REG_0B);
	val &= BQ2589X_CHRG_STAT_MASK;
	val >>= BQ2589X_CHRG_STAT_SHIFT;
	switch (val) {
	case CHARGE_STATE_FASTCHG:
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	case CHARGE_STATE_PRECHG:
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	case CHARGE_STATE_CHGDONE:
	case CHARGE_STATE_IDLE:
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	default:
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}
}

static int bq2589x_get_prop_charge_status(struct bq2589x *bq)
{
	int ret;
	u8 status;

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret) {
		return 	POWER_SUPPLY_STATUS_UNKNOWN;
	}

	mutex_lock(&bq->data_lock);
	bq->charge_state =
		(status & BQ2589X_CHRG_STAT_MASK) >> BQ2589X_CHRG_STAT_SHIFT;
	mutex_unlock(&bq->data_lock);

	switch(bq->charge_state) {
		case CHARGE_STATE_FASTCHG:
		case CHARGE_STATE_PRECHG:
			return POWER_SUPPLY_STATUS_CHARGING;
		case CHARGE_STATE_CHGDONE:
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		case CHARGE_STATE_IDLE:
			return POWER_SUPPLY_STATUS_DISCHARGING;
		default:
			return 	POWER_SUPPLY_STATUS_UNKNOWN;
	}

}
#if 0
static int bq2589x_get_prop_health(struct bq2589x *bq)
{
	int ret;

	if (bq->software_jeita_supported) {
		if (bq->jeita_active) {
			if (bq->batt_hot)
				ret = POWER_SUPPLY_HEALTH_OVERHEAT;
			else if (bq->batt_warm)
				ret = POWER_SUPPLY_HEALTH_WARM;
			else if (bq->batt_cool)
				ret = POWER_SUPPLY_HEALTH_COOL;
			else if (bq->batt_cold)
				ret = POWER_SUPPLY_HEALTH_COLD;
		} else {
			ret = POWER_SUPPLY_HEALTH_GOOD;
		}
	}
	ret = POWER_SUPPLY_HEALTH_GOOD;
	return ret;
}
#endif

static enum power_supply_property bq2589x_charger_props[] = {
		POWER_SUPPLY_PROP_CHARGE_TYPE,
		POWER_SUPPLY_PROP_PRESENT,
		POWER_SUPPLY_PROP_CHARGING_ENABLED,
		POWER_SUPPLY_PROP_STATUS,
		POWER_SUPPLY_PROP_INPUT_SUSPEND,
		POWER_SUPPLY_PROP_TEMP,
		POWER_SUPPLY_PROP_REAL_TYPE,
#ifdef CONFIG_PRODUCT_MOBA
		POWER_SUPPLY_PROP_ONLINE,
		POWER_SUPPLY_PROP_TI_CHARGER_BATTERY_VOLTAGE,
		POWER_SUPPLY_PROP_TI_CHARGER_SYSTEM_VOLTAGE,
		POWER_SUPPLY_PROP_TI_CHARGER_BUS_VOLTAGE,
		POWER_SUPPLY_PROP_TI_CHARGER_BATTERY_CURRENT,
		POWER_SUPPLY_PROP_TI_CHARGER_CONFIG_ICL,
		POWER_SUPPLY_PROP_TI_CHARGER_CONFIG_FCC,
		POWER_SUPPLY_PROP_TI_CHARGER_ICL,
		POWER_SUPPLY_PROP_FAC_INPUT_SUSPEND,
		POWER_SUPPLY_PROP_FAC_CHARGING_ENABLED,
#endif
};

int bq25890_chg_status = POWER_SUPPLY_STATUS_UNKNOWN;

#ifdef CONFIG_PRODUCT_MOBA
extern bool fusb_orient_en;
extern bool qcom_orient_en;
extern int usb2_otg_en;
#endif

static int bq2589x_charger_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{

	//struct bq2589x *bq = container_of(psy, struct bq2589x, batt_psy);
	struct bq2589x *bq = power_supply_get_drvdata(psy);

	int ret;
	u8 hiz;
		if (bq == NULL){
			pr_err("2589x is null\n");
		return 0;
		}
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = bq2589x_get_prop_charge_type(bq);
		pr_debug("POWER_SUPPLY_PROP_BQ_CHARGE_TYPE:%d\n", val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		bq2589x_get_charge_enable(bq);
		val->intval = bq->charge_enabled;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq2589x_get_prop_charge_status(bq);
		bq25890_chg_status = val->intval;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
#ifdef SUPPORT_ONSEMI_PDCONTROL
		val->intval = sec_port_online;
#endif
		break;
#ifdef CONFIG_PRODUCT_MOBA
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = fusb_orient_en && (!usb2_otg_en);
		break;
	case POWER_SUPPLY_PROP_FAC_INPUT_SUSPEND:
		val->intval = fac_input_suspend;
		ret = bq2589x_get_hiz_mode(bq, &hiz);
		if (!ret)
			val->intval = hiz;
		break;
	case POWER_SUPPLY_PROP_FAC_CHARGING_ENABLED:
		val->intval = fac_charging_enabled;
#endif
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
		ret = bq2589x_get_hiz_mode(bq, &hiz);
		if (!ret)
			val->intval = hiz;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = bq2589x_adc_read_temperature(bq);
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = bq->usb_type;
		break;
#ifdef CONFIG_PRODUCT_MOBA
	case POWER_SUPPLY_PROP_TI_CHARGER_BATTERY_VOLTAGE:
		val->intval = bq2589x_adc_read_battery_volt(bq);
		break;
	case POWER_SUPPLY_PROP_TI_CHARGER_BUS_VOLTAGE:
		val->intval = bq2589x_adc_read_vbus_volt(bq);
		break;
	case POWER_SUPPLY_PROP_TI_CHARGER_BATTERY_CURRENT:
		val->intval = bq2589x_adc_read_charge_current(bq);
		break;
	case POWER_SUPPLY_PROP_TI_CHARGER_SYSTEM_VOLTAGE:
		val->intval = bq2589x_adc_read_sys_volt(bq);
		break;
	case POWER_SUPPLY_PROP_TI_CHARGER_CONFIG_ICL:
		val->intval = bq2589x_read_config_icl(bq);
		break;
	case POWER_SUPPLY_PROP_TI_CHARGER_CONFIG_FCC:
		val->intval = bq2589x_read_config_fcc(bq);
		break;
	case POWER_SUPPLY_PROP_TI_CHARGER_ICL:
		val->intval = bq2589x_read_idpm_limit(bq);
		break;
#endif
	default:
		return -EINVAL;

	}
	return 0;
}

static int bq2589x_charger_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	//struct bq2589x *bq = container_of(psy, struct bq2589x, batt_psy);
#ifdef CONFIG_PRODUCT_MOBA
	int ret;
#endif
	struct bq2589x *bq = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
#ifdef CONFIG_PRODUCT_MOBA
		if (force_usb_suspend)
			break;

		pr_info("batt_sys: sec main charge enable %d\n", val->intval);
		if (val->intval)
			g_chg_en_dev |= 1 << CHG_EN_SEC_MAIN;
		else
			g_chg_en_dev &= ~(1 << CHG_EN_SEC_MAIN);
#endif
		bq2589x_charging_disable(bq, USER, !val->intval);
		power_supply_changed(bq->fc_main_psy);
		break;
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
#ifdef CONFIG_PRODUCT_MOBA
		if (force_usb_suspend)
			break;
#endif
		if (val->intval)
			bq2589x_enter_hiz_mode(bq);
		else
			bq2589x_exit_hiz_mode(bq);
		break;
#ifdef CONFIG_PRODUCT_MOBA
	case POWER_SUPPLY_PROP_FAC_INPUT_SUSPEND:
		pr_info("batt_sys: [FACTORY] sec main input suspend %d\n", val->intval);
		fac_input_suspend = val->intval;

		if (val->intval)
			bq2589x_enter_hiz_mode(bq);
		else
			bq2589x_exit_hiz_mode(bq);
		break;
	case POWER_SUPPLY_PROP_FAC_CHARGING_ENABLED:
		pr_info("batt_sys: [FACTORY] sec main charge enable %d\n", val->intval);
		fac_charging_enabled = val->intval;

		if (val->intval)
			g_chg_en_dev |= 1 << CHG_EN_SEC_MAIN;
		else
			g_chg_en_dev &= ~(1 << CHG_EN_SEC_MAIN);
		bq2589x_charging_disable(bq, USER, !val->intval);
		power_supply_changed(bq->fc_main_psy);
		break;
	case POWER_SUPPLY_PROP_FC_LIMIT_CURRENT:
		ret = bq2589x_set_chargecurrent(bq, val->intval);
		if (ret < 0) {
			pr_err("Failed to set bq2589h charge current %d\n", ret);
		}
		pr_info("set bq2589h fcc limit current is %d\n", val->intval);
	break;
#endif
	default:
		return -EINVAL;
	}
	return 0;
}

static int bq2589x_charger_is_writeable(struct power_supply *psy,
					enum power_supply_property prop)
{
	int ret;

	switch(prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_INPUT_SUSPEND:
#ifdef CONFIG_PRODUCT_MOBA
	case POWER_SUPPLY_PROP_FAC_INPUT_SUSPEND:
	case POWER_SUPPLY_PROP_FAC_CHARGING_ENABLED:
#endif
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}

static int bq2589x_psy_register(struct bq2589x *bq)
{
	int ret;
	bq->batt_cfg.drv_data = bq;
	bq->batt_cfg.of_node = bq->dev->of_node;

	bq->batt_psy.name = "bq2589h-charger";
	bq->batt_psy.type = POWER_SUPPLY_TYPE_MAINS;
	bq->batt_psy.properties = bq2589x_charger_props;
	bq->batt_psy.num_properties = ARRAY_SIZE(bq2589x_charger_props);
	bq->batt_psy.get_property = bq2589x_charger_get_property;
	bq->batt_psy.set_property = bq2589x_charger_set_property;
	//bq->batt_psy.external_power_changed = NULL;//bq2589x_external_power_changed;
	bq->batt_psy.property_is_writeable = bq2589x_charger_is_writeable;

	bq->fc_main_psy = devm_power_supply_register(bq->dev,
			&bq->batt_psy, &bq->batt_cfg);

	if (IS_ERR(bq->fc_main_psy)) {
		pr_err("failed to register fc_main_psy:%d\n", ret);
		return PTR_ERR(bq->fc_main_psy);
	}
#if 0
	ret = power_supply_register(bq->dev, &bq->batt_psy);
	if (ret < 0) {
		pr_err("failed to register batt_psy:%d\n", ret);
		return ret;
	}
#endif
	return 0;
}

static void bq2589x_psy_unregister(struct bq2589x *bq)
{
	power_supply_unregister(bq->fc_main_psy);
}

static int bq2589x_otg_regulator_enable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2589x *bq = rdev_get_drvdata(rdev);

	ret = bq2589x_enable_otg(bq);
	if (ret) {
		pr_err("Couldn't enable OTG mode ret=%d\n", ret);
	} else {
		bq->otg_enabled = true;
		pr_info("bq2589x OTG mode Enabled!\n");
	}

	return ret;
}


static int bq2589x_otg_regulator_disable(struct regulator_dev *rdev)
{
	int ret;
	struct bq2589x *bq = rdev_get_drvdata(rdev);

	ret = bq2589x_disable_otg(bq);
	if (ret) {
		pr_err("Couldn't disable OTG mode, ret=%d\n", ret);
	} else {
		bq->otg_enabled = false;
		pr_info("bq2589x OTG mode Disabled\n");
	}

	return ret;
}


static int bq2589x_otg_regulator_is_enable(struct regulator_dev *rdev)
{
	int ret;
	u8 status;
	u8 enabled;
	struct bq2589x *bq = rdev_get_drvdata(rdev);

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_03);
	if (ret)
		return ret;
	enabled = ((status & BQ2589X_OTG_CONFIG_MASK) >> BQ2589X_OTG_CONFIG_SHIFT);

	return (enabled == BQ2589X_OTG_ENABLE) ? 1 : 0;

}


struct regulator_ops bq2589x_otg_reg_ops = {
	.enable		= bq2589x_otg_regulator_enable,
	.disable	= bq2589x_otg_regulator_disable,
	.is_enabled = bq2589x_otg_regulator_is_enable,
};

static int bq2589x_regulator_init(struct bq2589x *bq)
{
	int ret = 0;
	struct regulator_init_data *init_data;
	struct regulator_config cfg = {};

	init_data = of_get_regulator_init_data(bq->dev, bq->dev->of_node,bq->batt_init_desc);
	if (!init_data) {
		dev_err(bq->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	if (init_data->constraints.name) {
		bq->otg_vreg.rdesc.owner = THIS_MODULE;
		bq->otg_vreg.rdesc.type = REGULATOR_VOLTAGE;
		bq->otg_vreg.rdesc.ops = &bq2589x_otg_reg_ops;
		bq->otg_vreg.rdesc.name = init_data->constraints.name;
		pr_info("regualtor name = %s\n", bq->otg_vreg.rdesc.name);

		cfg.dev = bq->dev;
		cfg.init_data = init_data;
		cfg.driver_data = bq;
		cfg.of_node = bq->dev->of_node;

		init_data->constraints.valid_ops_mask
			|= REGULATOR_CHANGE_STATUS;

		bq->otg_vreg.rdev = regulator_register(
					&bq->otg_vreg.rdesc, &cfg);
		if (IS_ERR(bq->otg_vreg.rdev)) {
			ret = PTR_ERR(bq->otg_vreg.rdev);
			bq->otg_vreg.rdev = NULL;
			if (ret != -EPROBE_DEFER)
				dev_err(bq->dev,
					"OTG reg failed, rc=%d\n", ret);
		}
	}

	return ret;
}

static ssize_t bq2589x_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2589x *bq = dev_get_drvdata(dev);
	u8 addr;
	u8 val;
	u8 tmpbuf[300];
	int len;
	int idx = 0;
	int ret ;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "bq25890");
	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(bq, &val, addr);
		if (ret == 0) {
			len = snprintf(tmpbuf, PAGE_SIZE - idx,
						"Reg[%.2X] = 0x%.2x\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bq2589x_store_register(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct bq2589x *bq = dev_get_drvdata(dev);
	int ret;
	unsigned int reg;
	unsigned int val;

	ret = sscanf(buf,"%x %x",&reg, &val);
	if (ret == 2 && reg <= 0x14) {
		bq2589x_write_byte(bq,(unsigned char)reg,(unsigned char)val);
	}

	return count;
}

static ssize_t bq2589x_show_part_no(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct bq2589x *bq = dev_get_drvdata(dev);
	int idx = 0;
	int ret;
	u8 data;
	u8 chip_part_no;
	u8 chip_revision;

	ret = bq2589x_read_byte(bq, &data, BQ2589X_REG_14);
	if (ret == 0) {
		chip_part_no = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
		chip_revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
	} else
		chip_part_no = 0x0;

	idx = snprintf(buf, PAGE_SIZE, "%04X\n", chip_part_no);

	return idx;
}

static DEVICE_ATTR(registers, 0660, bq2589x_show_registers, bq2589x_store_register);
static DEVICE_ATTR(part_no, 0660, bq2589x_show_part_no, NULL);

static struct attribute *bq2589x_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_part_no.attr,
	NULL,
};

static const struct attribute_group bq2589x_attr_group = {
	.attrs = bq2589x_attributes,
};


static int bq2589x_parse_dt(struct device *dev, struct bq2589x *bq)
{
	int ret;
	struct device_node *np = dev->of_node;

	ret = of_property_read_u32(np, "ti,bq2589x,dpdm-switch-gpio",
								&bq->cfg.dpdm_sw_gpio);
    if(ret)
		pr_err("Failed to read node of ti,bq2589x,dpdm-switch-gpio\n");

	bq->cfg.usb_id_gpio = of_get_named_gpio(np, "ti,usbid-gpio",0);
	if (bq->cfg.usb_id_gpio < 0) {
		pr_err("usb_id_gpio is not avaiable\n");
	}

	if (pe.enable) {
		ret = of_property_read_u32(np, "ti,bq2589x,vbus-volt-high-level",
									&pe.high_volt_level);
		if (ret)
			pr_err("Failed to read node of ti,bq2589x,vbus-volt-high-level\n");


		ret = of_property_read_u32(np, "ti,bq2589x,vbus-volt-low-level",
									&pe.low_volt_level);
		if (ret)
			pr_err("Failed to read node of ti,bq2589x,vbus-volt-low-level\n");

		ret = of_property_read_u32(np, "ti,bq2589x,vbat-min-volt-to-tuneup",
									&pe.vbat_min_volt);
		if (ret)
			pr_err("Failed to read node of ti,bq2589x,vbat-min-volt-to-tuneup\n");
	}

	bq->cfg.enable_auto_dpdm = of_property_read_bool(np,
								"ti,bq2589x,enable-auto-dpdm");
	bq->cfg.enable_term = of_property_read_bool(np,
								"ti,bq2589x,enable-termination");
	bq->cfg.enable_ico = of_property_read_bool(np,
								"ti,bq2589x,enable-ico");
	bq->cfg.use_absolute_vindpm = of_property_read_bool(np,
								"ti,bq2589x,use-absolute-vindpm");
	bq->cfg.enable_12v = of_property_read_bool(np,
								"ti,bq2589x,enable-12v");
	bq->cfg.enable_hvdcp = of_property_read_bool(np,
								"ti,bq2589x,enable-hvdcp");
	bq->cfg.enable_maxc = of_property_read_bool(np,
								"ti,bq2589x,enable-maxc");

	ret = of_property_read_u32(np, "ti,bq2589x,charge-voltage",
								&bq->cfg.charge_voltage);
	if (ret)
		pr_err("Failed to read node of ti,bq2589x,charge-voltage\n");

	ret = of_property_read_u32(np, "ti,bq2589x,charge-current",
								&bq->cfg.charge_current);
	if (ret)
		pr_err("Failed to read node of ti,bq2589x,charge-current\n");

	ret = of_property_read_u32(np, "ti,bq2589x,termination-current",
								&bq->cfg.term_current);
	if (ret)
		pr_err("Failed to read node of ti,bq2589x,termination-current\n");

	return 0;
}


static int bq2589x_parse_jeita_dt(struct device *dev, struct bq2589x* bq)
{
    struct device_node *np = dev->of_node;
	int ret;

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-hot-degc",
						&bq->batt_hot_degc);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-hot-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-warm-degc",
						&bq->batt_warm_degc);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-warm-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-cool-degc",
						&bq->batt_cool_degc);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-cool-degc\n");
		return ret;
	}
	ret = of_property_read_u32(np,"ti,bq2589x,jeita-cold-degc",
						&bq->batt_cold_degc);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-cold-degc\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-hot-hysteresis",
						&bq->hot_temp_hysteresis);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-hot-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-cold-hysteresis",
						&bq->cold_temp_hysteresis);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-cold-hysteresis\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-cool-ma",
						&bq->batt_cool_ma);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-cool-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-cool-mv",
						&bq->batt_cool_mv);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-cool-mv\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-warm-ma",
						&bq->batt_warm_ma);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-warm-ma\n");
		return ret;
	}

	ret = of_property_read_u32(np,"ti,bq2589x,jeita-warm-mv",
						&bq->batt_warm_mv);
    if(ret) {
		pr_err("Failed to read ti,bq2589x,jeita-warm-mv\n");
		return ret;
	}

	bq->software_jeita_supported =
		of_property_read_bool(np,"ti,bq2589x,software-jeita-supported");

	return 0;
}

static void bq2589x_init_jeita(struct bq2589x *bq)
{

	bq->batt_temp = -EINVAL;

	/* set default value in case of dts read fail */
	bq->batt_hot_degc = 600;
	bq->batt_warm_degc = 450;
	bq->batt_cool_degc = 100;
	bq->batt_cold_degc = 0;

	bq->hot_temp_hysteresis = 50;
	bq->cold_temp_hysteresis = 50;

	bq->batt_cool_ma = 400;
	bq->batt_cool_mv = 4100;
	bq->batt_warm_ma = 400;
	bq->batt_warm_mv = 4100;

	bq->software_jeita_supported = true;

	/* DTS setting will overwrite above default value */

	bq2589x_parse_jeita_dt(&bq->client->dev, bq);
}

static int bq2589x_detect_device(struct bq2589x *bq)
{
	int ret;
	u8 data;

	ret = bq2589x_read_byte(bq, &data, BQ2589X_REG_14);
	if (ret == 0) {
		bq->part_no = (data & BQ2589X_PN_MASK) >> BQ2589X_PN_SHIFT;
		bq->revision = (data & BQ2589X_DEV_REV_MASK) >> BQ2589X_DEV_REV_SHIFT;
	}

	return ret;
}

static void bq2589x_adjust_absolute_vindpm(struct bq2589x *bq)
{
	u16 vbus_volt;
	int ret;

	/* wait for new adc data */
	msleep(1000);
	vbus_volt = bq2589x_adc_read_vbus_volt(bq);

	if (vbus_volt < 6000)
		bq->ivl_mv = vbus_volt - 600;
	else
		bq->ivl_mv = vbus_volt - 1200;
	ret = bq2589x_set_input_volt_limit(bq, bq->ivl_mv);

	pr_err("Set absolute vindpm threshold %s\n",
			!ret ? "Successfully" : "Failed");

}


static int bq2589x_update_charging_profile(struct bq2589x *bq)
{
	int ret;
	int chg_ma;
	int chg_mv;
	int icl;

	if (!bq->usb_present)
		return 0;

return 0;
	pr_info("chg_mv:%d,chg_ma:%d,icl:%d,ivl:%d\n",
					bq->chg_mv, bq->chg_ma, bq->icl_ma, bq->ivl_mv);


	mutex_lock(&bq->profile_change_lock);

	if (bq->jeita_active) {
		chg_ma = bq->jeita_ma;
		chg_mv = bq->jeita_mv;
	} else {
		chg_ma = bq->chg_ma;
		chg_mv = bq->chg_mv;
	}

	icl = bq->icl_ma;

	/*TODO: add therm_lvl_sel*/

	pr_info("chg_mv:%d, chg_ma:%d, icl:%d, ivl:%d\n",
				chg_mv, chg_ma, icl, bq->ivl_mv);

	if (bq->chg_mv > 0) {
		ret = bq2589x_set_chargevoltage(bq, bq->chg_mv);
		if (ret < 0)
			pr_err("Failed to set charge current:%d\n", ret);
	}

	ret = bq2589x_set_chargecurrent(bq, bq->chg_ma);
	if (ret < 0)
		pr_err("Failed to set charge current:%d\n", ret);

	ret = bq2589x_set_input_volt_limit(bq, bq->ivl_mv);
	if (ret < 0)
		pr_err("failed to set input volt limit:%d\n", ret);

	ret = bq2589x_set_input_current_limit(bq, bq->icl_ma);
	if (ret < 0)
		pr_err("failed to set input current limit:%d\n", ret);

	mutex_unlock(&bq->profile_change_lock);

	return 0;
}

#ifdef CONFIG_PRODUCT_MOBA
int bq2589x_dynamic_update_charging_profile(int chg_uv, int chg_ua, int icl_ua, int ivl_uv)
{
	int ret;

	//if (!bq2->usb_present)
	//	return 0;

	pr_debug("[CHARGE] profile chg_uv:%d,chg_ua:%d,icl:%d,ivl:%d\n",
					chg_uv, chg_ua, icl_ua, ivl_uv);

	//mutex_lock(&bq2->profile_change_lock);

	if (chg_uv > 0) {
		ret = bq2589x_set_chargevoltage(bq2, chg_uv/1000);
		if (ret < 0)
			pr_err("[CHARGE] Failed to set charge current:%d\n", ret);
	}

	ret = bq2589x_set_chargecurrent(bq2, chg_ua/1000);
	if (ret < 0)
		pr_err("[CHARGE] Failed to set charge current:%d\n", ret);

	ret = bq2589x_set_input_volt_limit(bq2, ivl_uv/1000);
	if (ret < 0)
		pr_err("[CHARGE] failed to set input volt limit:%d\n", ret);

	ret = bq2589x_set_input_current_limit(bq2, icl_ua/1000);
	if (ret < 0)
		pr_err("[CHARGE] failed to set input current limit:%d\n", ret);

	//mutex_unlock(&bq2->profile_change_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(bq2589x_dynamic_update_charging_profile);

static void bq2589x_adapter_in_handler(struct bq2589x *bq);
static void charger_recheck_work(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, charger_recheck_work.work);

	pr_info("charger recheck work\n");
	charger_recheck = 1;
	bq2589x_adapter_in_handler(bq);
}

int bq25890h_charger_insert_remove_event(struct bq2589x *bq, int online)
{
	int rc;
	union power_supply_propval val = {0,};

	if (!bq->qcom_usb_psy) {
		bq->qcom_usb_psy = power_supply_get_by_name("usb");
		if (!bq->qcom_usb_psy)
			return -ENODEV;
	}

	val.intval = !!online;
	rc = power_supply_set_property(bq->qcom_usb_psy,
				POWER_SUPPLY_PROP_SEC_ONLINE,
				&val);
	if (rc < 0)
		pr_err("Couldn't set qcom usb POWER_SUPPLY_PROP_SEC_ONLINE, rc=%d\n",
			rc);

	return rc;
}
#endif

static void bq2589x_start_pe_handshake(struct bq2589x *bq, bool up)
{

	if (!pe.enable)
		return;

	if (up) {
		pe.target_volt = pe.high_volt_level;
		pe.tune_up = true;
		pe.tune_down = false;
	} else {
		pe.target_volt = pe.low_volt_level;
		pe.tune_up = false;
		pe.tune_down = true;
	}
	pe.tune_done = false;
	pe.tune_count = 0;
	pe.tune_fail = false;

	//disable charger, enable it again after voltage tune done
//	bq2589x_charging_disable(bq, BATT_TUNE, true);
	schedule_delayed_work(&bq->pe_work, 0);
}



static void bq2589x_convert_vbus_type(struct bq2589x *bq)
{
	switch (bq->vbus_type) {
	case BQ2589X_VBUS_USB_SDP:
		bq->usb_type = POWER_SUPPLY_TYPE_USB;
		type_detect_done = true;
		break;
	case BQ2589X_VBUS_USB_CDP:
		bq->usb_type = POWER_SUPPLY_TYPE_USB_CDP;
		type_detect_done = true;
		break;
	case BQ2589X_VBUS_USB_DCP:
		bq->usb_type = POWER_SUPPLY_TYPE_USB_DCP;
		type_detect_done = false;
		break;
	case BQ2589X_VBUS_MAXC:
		bq->usb_type = POWER_SUPPLY_TYPE_USB_HVDCP;
		break;
	case BQ2589X_VBUS_NONSTAND:
		bq->usb_type = POWER_SUPPLY_TYPE_USB_ACA;
		break;
	case BQ2589X_VBUS_UNKNOWN:
		bq->usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	default:
		bq->usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
		break;
	}
}

static void bq2589x_get_hvdcp_profile(struct bq2589x *bq)
{
	u16 vbus_volt;

	vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	if (vbus_volt <= 5300) {
		bq->icl_ma = 500;
		bq->ivl_mv = 4600;
		bq->chg_ma = 500;
	} else if (vbus_volt <= 9500) {
		bq->icl_ma = 2000;
		bq->ivl_mv = vbus_volt - 1200;
		bq->chg_ma = 3000;
	} else {
		bq->icl_ma = 1500;
		bq->ivl_mv = vbus_volt - 1200;
		bq->chg_ma = 3000;
	}
}

static void bq2589x_adapter_in_handler(struct bq2589x *bq)
{
	int ret;
	bool update_profile = true;

#ifdef CONFIG_PRODUCT_MOBA
	if ((!charger_recheck) && (POWER_SUPPLY_TYPE_USB_DCP == bq->usb_type)) {
		bq->usb_type = POWER_SUPPLY_TYPE_USB;
		pr_info("vbus_type: DCP to SDP\n");
	}
#endif
	bq2589x_convert_vbus_type(bq);
	pr_err("vbus_type:%d, usb_type:%d\n", bq->vbus_type, bq->usb_type);

	switch (bq->usb_type) {
	case POWER_SUPPLY_TYPE_USB:
#ifdef CONFIG_PRODUCT_MOBA
		bq->icl_ma = USB_BQ2589X_USBIN_CURRENT;
		bq->ivl_mv = 4600;
		bq->chg_ma = FCC_BQ2589X_TYPE_USB_CURRENT;
#else
		bq->icl_ma = 2000;
		bq->ivl_mv = 4600;
		bq->chg_ma = 2000;
#endif
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
#ifdef CONFIG_PRODUCT_MOBA
		bq->icl_ma = CDP_BQ2589X_USBIN_CURRENT;
		bq->ivl_mv = 4600;
		bq->chg_ma = FCC_BQ2589X_TYPE_CDP_CURRENT;
#else
		bq->icl_ma = 1500;
		bq->ivl_mv = 4600;
		bq->chg_ma = 1500;
#endif
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
#ifdef CONFIG_PRODUCT_MOBA
		bq->icl_ma = DCP_BQ2589X_USBIN_CURRENT;
		bq->ivl_mv = 4600;
		bq->chg_ma = FCC_BQ2589X_TYPE_DCP_CURRENT;
#else
		bq->icl_ma = 2000;
		bq->ivl_mv = 4600;
		bq->chg_ma = 2000;
#endif
		bq2589x_start_pe_handshake(bq, true);
		break;
	case POWER_SUPPLY_TYPE_USB_HVDCP:
		bq2589x_get_hvdcp_profile(bq);
		break;
	case POWER_SUPPLY_TYPE_USB_ACA:
		bq->usb_type = POWER_SUPPLY_TYPE_USB_ACA;
#ifdef CONFIG_PRODUCT_MOBA
		bq->icl_ma = USB_BQ2589X_USBIN_CURRENT;
		bq->ivl_mv = 4600;
		bq->chg_ma = FCC_BQ2589X_TYPE_USB_CURRENT;
#else
		bq->icl_ma = 1000;
		bq->ivl_mv = 4600;
		bq->chg_ma = 1000;
#endif
		break;
	case POWER_SUPPLY_TYPE_UNKNOWN:
#ifdef CONFIG_PRODUCT_MOBA
		bq->icl_ma = USB_BQ2589X_USBIN_CURRENT;
		bq->ivl_mv = 4600;
		bq->chg_ma = FCC_BQ2589X_TYPE_USB_CURRENT;
#else
		bq->icl_ma = 2000;
		bq->ivl_mv = 4600;
		bq->chg_ma = 2000;
#endif
		break;
	default:
		break;
	}

	if (update_profile)
#ifdef CONFIG_PRODUCT_MOBA
		if ((!insert_delay_done) && (!bootup_delay_done))
#endif
		bq2589x_update_charging_profile(bq);

	if (bq->usb_type == POWER_SUPPLY_TYPE_USB_DCP ||
		bq->usb_type == POWER_SUPPLY_TYPE_USB_HVDCP) {
		ret = bq2589x_force_ico(bq);
		if (!ret) {
			schedule_delayed_work(&bq->ico_work, 2 * HZ);
			pr_info("Force ICO successfully\n");
		} else {
			pr_err("Force ICO failed\n");
		}
	}

	cancel_delayed_work(&bq->discharge_jeita_work);

//	bq2589x_set_watchdog_timer(bq, 80);
}

static void bq2589x_adapter_out_handler(struct bq2589x *bq)
{
	bq2589x_disable_watchdog_timer(bq);
	type_detect_done = false;

	pr_err("usb removed, set usb present = %d\n", bq->usb_present);
}

static const unsigned char* charge_stat_str[] = {
	"Not Charging",
	"Precharging",
	"Fast Charging",
	"Charge Done",
};

static void bq2589x_dump_reg(struct bq2589x *bq)
{
	int ret;
	int addr;
	u8 val;

	for (addr = 0x0; addr <= 0x14; addr++) {
		msleep(5);
		ret = bq2589x_read_byte(bq, &val, addr);
		if (ret == 0)
			pr_err("Reg[%02X] = 0x%02X\n", addr, val);
	}

}

static void bq2589x_dump_reg2(struct bq2589x *bq)
{
	int ret;
	int addr;
	char data[128];
	int len = 0;
	u8 val;

	memset(data, 0x00, sizeof(data));
	for (addr = 0x0; addr <= 0x14; addr++) {
		msleep(5);
		ret = bq2589x_read_byte(bq, &val, addr);
		if (ret == 0)
			sprintf(data + len, "%02X ", val);
			len += 3;
	}
	pr_err("%s\n", data);
}
#if 1
static void bq2589x_dump_status(struct bq2589x *bq)
{
	int ret;
	u8 status,fault;
	int chg_current;

	bq->vbus_volt = bq2589x_adc_read_vbus_volt(bq);
	bq->vbat_volt = bq2589x_adc_read_battery_volt(bq);
	chg_current = bq2589x_adc_read_charge_current(bq);

	pr_info("vbus:%d,vbat:%d,ibat:%d\n", bq->vbus_volt,
					bq->vbat_volt, chg_current);

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_13);
	if (!ret){
		if (status & BQ2589X_VDPM_STAT_MASK)
			dev_info(bq->dev, "%s:VINDPM occurred\n", __func__);
		if (status & BQ2589X_IDPM_STAT_MASK)
			dev_info(bq->dev, "%s:IINDPM occurred\n", __func__);
	}


	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (!ret) {
		mutex_lock(&bq->data_lock);
		bq->fault_status = fault;
		mutex_unlock(&bq->data_lock);
	}

//	if (bq->fault_status & BQ2589X_FAULT_WDT_MASK)
//		pr_err("Watchdog timer expired!\n");
	if (bq->fault_status & BQ2589X_FAULT_BOOST_MASK)
		pr_err("Boost fault occurred!\n");

	status = (bq->fault_status & BQ2589X_FAULT_CHRG_MASK) >> BQ2589X_FAULT_CHRG_SHIFT;
	if (status == BQ2589X_FAULT_CHRG_INPUT)
		pr_err("input fault!\n");
	else if (status == BQ2589X_FAULT_CHRG_THERMAL)
		pr_err("charge thermal shutdown fault!\n");
	else if (status == BQ2589X_FAULT_CHRG_TIMER)
		pr_err("charge timer expired fault!\n");

	if (bq->fault_status & BQ2589X_FAULT_BAT_MASK)
		pr_err("battery ovp fault!\n");

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (!ret) {
		bq->charge_state = status & BQ2589X_CHRG_STAT_MASK;
		bq->charge_state >>= BQ2589X_CHRG_STAT_SHIFT;
		pr_debug("%s\n", charge_stat_str[bq->charge_state]);
	}

//	bq2589x_dump_reg(bq);
}
#endif

static void bq2589x_ico_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, ico_work.work);
	int ret;

	ret = bq2589x_check_ico_done(bq);

	if (ret == 1) {
		ret = bq2589x_read_idpm_limit(bq);
		if (ret < 0)
			pr_info("ICO done, but failed to read idmp limit:%d\n", ret);
		else
			pr_info("ICO done, idpm limit = %dmA\n", ret);
	} else {
		schedule_delayed_work(&bq->ico_work, 2 * HZ);
	}

}

static void bq2589x_pe_workfunc(struct work_struct *work)
{
	struct bq2589x *bq = container_of(work, struct bq2589x, pe_work.work);
	int ret;
	static bool pumpx_cmd_issued;

	bq->vbus_volt = bq2589x_adc_read_vbus_volt(bq);

	pr_info("VBus:%d, Target:%d\n", bq->vbus_volt, pe.target_volt);

	if ((pe.tune_up && bq->vbus_volt > pe.target_volt) ||
	    (pe.tune_down && bq->vbus_volt < pe.target_volt)) {
		pr_err("voltage tune successfully\n");
		pe.tune_done = true;
		bq2589x_adjust_absolute_vindpm(bq);
		if (pe.tune_up) {
			ret = bq2589x_force_ico(bq);
			if (!ret) {
				schedule_delayed_work(&bq->ico_work, 2 * HZ);
				pr_info("Force ICO successfully\n");
			} else {
				pr_err("Force ICO failed\n");
			}
		}
		return;
	}

	if (pe.tune_count > 5) {
		pr_info("voltage tune failed,reach max retry count\n");
		pe.tune_fail = true;
		bq2589x_adjust_absolute_vindpm(bq);

		if (pe.tune_up) {
#ifdef CONFIG_PRODUCT_MOBA
		if ((!insert_delay_done) && (!bootup_delay_done))
#endif
			bq2589x_update_charging_profile(bq);
			ret = bq2589x_force_ico(bq);
			if (!ret) {
				schedule_delayed_work(&bq->ico_work, 2 * HZ);
				pr_info("Force ICO successfully\n");
			} else {
				pr_err("Force ICO failed\n");
			}
		}
		return;
	}

	if (!pumpx_cmd_issued) {
		if (pe.tune_up)
			ret = bq2589x_pumpx_increase_volt(bq);
		else if (pe.tune_down)
			ret =  bq2589x_pumpx_decrease_volt(bq);
		if (ret) {
			schedule_delayed_work(&bq->pe_work, HZ);
		} else {
			dev_info(bq->dev, "%s:pumpx command issued.\n", __func__);
			pumpx_cmd_issued = true;
			pe.tune_count++;
			schedule_delayed_work(&bq->pe_work, 3*HZ);
		}
	} else {
		if (pe.tune_up)
			ret = bq2589x_pumpx_increase_volt_done(bq);
		else if (pe.tune_down)
			ret = bq2589x_pumpx_decrease_volt_done(bq);
		if (ret == 0) {
			dev_info(bq->dev, "%s:pumpx command finishedd!\n", __func__);
			pumpx_cmd_issued = 0;
		}
		schedule_delayed_work(&bq->pe_work, HZ);
	}


}

static irqreturn_t bq2589x_charger_interrupt(int irq, void *data)
{
	int ret;
	struct bq2589x *bq = data;
	u8 status = 0;
	u8 fault = 0;

	msleep(5);

	//bq2589x_dump_reg(bq);
	bq2589x_dump_status(bq);

	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		dev_dbg(bq->dev, "IRQ triggered before device-resume\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(irq);
			bq->irq_disabled = true;
		}
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->irq_waiting = false;


	/* Read STATUS and FAULT registers */
	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (ret) {
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}

	ret = bq2589x_read_byte(bq, &fault, BQ2589X_REG_0C);
	if (ret) {
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	} else {
		mutex_lock(&bq->data_lock);
		bq->fault_status = fault;
		mutex_unlock(&bq->data_lock);
	}

	bq->vbus_type = (status & BQ2589X_VBUS_STAT_MASK) >> BQ2589X_VBUS_STAT_SHIFT;

	if (((bq->vbus_type == BQ2589X_VBUS_NONE) || (bq->vbus_type == BQ2589X_VBUS_OTG))
						&& bq->usb_present) {
		bq->usb_present = false;
#ifdef SUPPORT_ONSEMI_PDCONTROL
		sec_port_online = bq->usb_present;
		pr_info("adapter removed,sec_port_online = %d\n", sec_port_online);
		bq->usb_type = POWER_SUPPLY_TYPE_UNKNOWN;
#endif
#ifdef CONFIG_PRODUCT_MOBA
		cancel_delayed_work(&bq->charger_recheck_work);
		charger_recheck = 0;
		bq25890h_charger_insert_remove_event(bq, 0);
#endif
		bq2589x_adapter_out_handler(bq);
#ifdef SUPPORT_ONSEMI_PDCONTROL
		cancel_work(&bq_usbpd_pm->usb_psy_change_work);
#endif
	} else if (bq->vbus_type != BQ2589X_VBUS_NONE && (bq->vbus_type != BQ2589X_VBUS_OTG)
						&& !bq->usb_present) {
		bq->usb_present = true;

#ifdef SUPPORT_ONSEMI_PDCONTROL
		sec_port_online = bq->usb_present;
		pr_info("adapter plugged in,sec_port_online = %d\n", sec_port_online);
#endif
#ifdef CONFIG_PRODUCT_MOBA
		cancel_delayed_work(&bq->charger_recheck_work);
		charger_recheck = 0;
		bq25890h_charger_insert_remove_event(bq, 1);
		schedule_delayed_work(&bq->charger_recheck_work, 4 * HZ);
#endif
		bq2589x_adapter_in_handler(bq);
#ifdef SUPPORT_ONSEMI_PDCONTROL
		schedule_work(&bq_usbpd_pm->usb_psy_change_work);
#endif
	}

	mutex_unlock(&bq->irq_complete);
	power_supply_changed(bq->fc_main_psy);

	return IRQ_HANDLED;
}

static void determine_initial_status(struct bq2589x *bq)
{
	int ret;
	u8 status;

	ret = bq2589x_read_byte(bq, &status, BQ2589X_REG_0B);
	if (!ret && (status & BQ2589X_PG_STAT_MASK)){
		msleep(2);
		bq2589x_force_dpdm(bq);
	}
}

#ifdef DBG_FS
static int show_registers(struct seq_file *m, void *data)
{
	struct bq2589x *bq = m->private;
	u8 addr;
	int ret;
	u8 val;

	for (addr = 0x0; addr <= 0x14; addr++) {
		ret = bq2589x_read_byte(bq, &val, addr);
		if (!ret)
			seq_printf(m, "Reg[0x%02X] = 0x%02X\n", addr, val);
	}
	return 0;
}

static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bq2589x *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}


static const struct file_operations reg_debugfs_ops = {
	.owner		= THIS_MODULE,
	.open		= reg_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void create_debugfs_entry(struct bq2589x *bq)
{
	bq->debug_root = debugfs_create_dir("bq25890h-main-charger", NULL);
	if (!bq->debug_root)
		pr_err("Failed to create debug dir\n");

	if (bq->debug_root) {

		debugfs_create_file("registers", S_IFREG | S_IRUGO,
						bq->debug_root, bq, &reg_debugfs_ops);

		debugfs_create_x32("charging_disable_status", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->charging_disabled_status));

		debugfs_create_x32("vbus_type", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->vbus_type));

		debugfs_create_x32("charge_state", S_IFREG | S_IRUGO,
						bq->debug_root, &(bq->charge_state));

		debugfs_create_x32("skip_reads",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->skip_reads));
		debugfs_create_x32("skip_writes",
					  S_IFREG | S_IWUSR | S_IRUGO,
					  bq->debug_root,
					  &(bq->skip_writes));
	}
}
#endif

static int bq2589x_charger_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct bq2589x *bq;
	int ret;
	bq = devm_kzalloc(&client->dev, sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}

#ifdef CONFIG_PRODUCT_MOBA
	bq2 = devm_kzalloc(&client->dev, sizeof(struct bq2589x), GFP_KERNEL);
	if (!bq) {
		pr_err("out of memory\n");
		return -ENOMEM;
	}

	g_chg_en_dev |= 1 << CHG_EN_SEC_MAIN;
#endif
	bq->dev = &client->dev;
	bq->client = client;
	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->profile_change_lock);
	mutex_init(&bq->charging_disable_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	ret = bq2589x_detect_device(bq);
	if (!ret && bq->part_no == BQ25890) {
		pr_info("charger device bq25890 detected, revision:%d\n",
									bq->revision);
	} else {
		pr_info("no bq25890H charger device found:%d\n",ret);
		return -ENODEV;
	}


	pe.enable = false;

	if (client->dev.of_node)
		bq2589x_parse_dt(&client->dev, bq);

	bq2589x_dump_reg(bq);
	ret = bq2589x_init_device(bq);
	if (ret) {
		dev_err(bq->dev, "device init failure: %d\n", ret);
		goto err_0;
	}

	ret = bq2589x_psy_register(bq);
	if (ret)
		goto err_0;


	ret = bq2589x_regulator_init(bq);
	if (ret) {
		pr_err("Couldn't initialize bq2589x regulator ret=%d\n", ret);
		return ret;
	}

	INIT_DELAYED_WORK(&bq->ico_work, bq2589x_ico_workfunc);
	INIT_DELAYED_WORK(&bq->pe_work, bq2589x_pe_workfunc);
#ifdef CONFIG_PRODUCT_MOBA
	INIT_DELAYED_WORK(&bq->charger_recheck_work, charger_recheck_work);
#endif

	bq2589x_init_jeita(bq);


	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
				NULL, bq2589x_charger_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"bq2589x_charger1_irq", bq);
		if (ret) {
			pr_err("Request IRQ %d failed: %d\n", client->irq, ret);
			goto err_irq;
		} else {
			pr_info("irq = %d\n", client->irq);
		}
		enable_irq_wake(client->irq);
	}

	bq2589x_wakeup_src_init(bq);

	device_init_wakeup(bq->dev, true);



#ifdef DBG_FS
	create_debugfs_entry(bq);
#endif
	ret = sysfs_create_group(&bq->dev->kobj, &bq2589x_attr_group);
	if (ret) {
		pr_err("failed to register sysfs. err: %d\n", ret);
		goto err_irq;
	}


	determine_initial_status(bq);
	pr_err(" probe successfully\n");

	bq2589x_dump_reg(bq);

#ifdef CONFIG_PRODUCT_MOBA
	bq2 = bq;
#endif
	return 0;

err_irq:

err_0:
	bq2589x_psy_unregister(bq);
	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->irq_complete);

	return ret;
}

static inline bool is_device_suspended(struct bq2589x *bq)
{
	return !bq->resume_completed;
}

static int bq2589x_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);
	pr_err("Suspend successfully!");

	return 0;
}

static int bq2589x_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending\n");
		return -EBUSY;
	}
	return 0;
}

static int bq2589x_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq2589x *bq = i2c_get_clientdata(client);


	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		bq2589x_charger_interrupt(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(bq->fc_main_psy);
	pr_err("Resume successfully!");

	return 0;
}

static int bq2589x_charger_remove(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	pr_info("remove..");
	bq2589x_adc_stop(bq);

	regulator_unregister(bq->otg_vreg.rdev);

	bq2589x_psy_unregister(bq);

#ifdef CONFIG_PRODUCT_MOBA
	cancel_delayed_work(&bq->charger_recheck_work);
#endif
	cancel_delayed_work(&bq->ico_work);
	cancel_delayed_work(&bq->pe_work);

	mutex_destroy(&bq->charging_disable_lock);
	mutex_destroy(&bq->profile_change_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->irq_complete);

#ifdef DBG_FS
	debugfs_remove_recursive(bq->debug_root);
#endif

	sysfs_remove_group(&bq->dev->kobj, &bq2589x_attr_group);

	return 0;
}

static void bq2589x_charger_shutdown(struct i2c_client *client)
{
	struct bq2589x *bq = i2c_get_clientdata(client);

	pr_info("bq2589x_charger_shutdown ...");
	is_shutdown = 1;
	bq2589x_dynamic_update_charging_profile(4400, 100000, 100000, 4600);
	bq2589x_charging_disable(bq, SYS_OFF, 1);
	bq2589x_adc_stop(bq);
}

static struct of_device_id bq2589x_charger_match_table[] = {
	{.compatible = "ti,bq2589x-charger",},
	{},
};


static const struct i2c_device_id bq2589x_charger_id[] = {
	{ "bq2589x-charger", BQ25890 },
	{},
};

MODULE_DEVICE_TABLE(i2c, bq2589x_charger_id);

const struct dev_pm_ops bq2589x_pm = {
	.suspend = bq2589x_suspend,
	.resume = bq2589x_resume,
};

static const struct dev_pm_ops bq2589x_pm_ops = {
	.resume		= bq2589x_resume,
	.suspend_noirq = bq2589x_suspend_noirq,
	.suspend	= bq2589x_suspend,
};

static struct i2c_driver bq2589x_charger_driver = {
	.driver		= {
		.name	= "bq25890h",
		.of_match_table = bq2589x_charger_match_table,
		.pm	= &bq2589x_pm_ops,
	},
	.id_table	= bq2589x_charger_id,

	.probe		= bq2589x_charger_probe,
	.shutdown  	= bq2589x_charger_shutdown,
	.remove		= bq2589x_charger_remove,
};

module_i2c_driver(bq2589x_charger_driver);

MODULE_DESCRIPTION("TI BQ2589x Charger Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");
