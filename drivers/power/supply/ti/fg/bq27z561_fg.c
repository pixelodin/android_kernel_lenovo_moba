/*
 * bq27z561 fuel gauge driver
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt)	"[bq27z561] %s: " fmt, __func__
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/gpio/consumer.h>

#define USER_FAKE_BATTERY_SOC				20
#define USER_SLAVE_BATTERY_ERROR_SOC			30
#define USER_MASTER_BATTERY_ERROR_SOC			40
#define USER_BOTH_BATTERY_ERROR_SOC			50
#define USER_BAD_BATTERY_SOC				0
#define USER_BAD_BATTERY_VOLTAGE			0
#define USER_BAD_BATTERY_CURRENT			0
#define USER_BAD_BATTERY_TEMP				-1000

int last_batt1_soc = -1;
int last_batt2_soc = -1;
int last_batt1_real_soc = -1;
int last_batt2_real_soc = -1;
int last_user_soc = -1;
int last_cc1 = -1;
int last_cc2 = -1;
int last_charge_full1 = -1;
int last_charge_full2 = -1;

extern int bq25890_chg_status;

enum bq_fg_device {
	BQ27Z561_MASTER,
	BQ27Z561_SLAVE,
};

static int bq27z561_mode_data[] = {
	[BQ27Z561_MASTER] = BQ27Z561_MASTER,
	[BQ27Z561_SLAVE] = BQ27Z561_SLAVE,
};

static const unsigned char *device2str[] = {
	"bq27z561-master",
	"bq27z561-slave",
};

#ifdef CONFIG_PRODUCT_MOBA
u8 is_fake_battery = 0;
int debug_temp_enable = 0;
int debug_temp1 = 250;
int debug_temp2 = 250;
static int bq_wake_lock = -1;
#endif

#if 0
#define bq_info	pr_info
#define bq_dbg	pr_debug
#define bq_err	pr_err
#define bq_log	pr_err
#else
#define bq_info(fmt, ...)								\
do {											\
	if (bq->mode == BQ27Z561_MASTER)						\
		printk(KERN_ERR "[bq27z561-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else if (bq->mode == BQ27Z561_SLAVE)						\
		printk(KERN_ERR "[bq27z561--SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else										\
		printk(KERN_ERR "[Unknown]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);
#define bq_dbg(fmt, ...)								\
do {											\
	if (bq->mode == BQ27Z561_MASTER)						\
		printk(KERN_ERR "[bq27z561-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else if (bq->mode == BQ27Z561_SLAVE)						\
		printk(KERN_ERR "[bq27z561--SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else										\
		printk(KERN_ERR "[Unknown]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);
#define bq_err(fmt, ...)								\
do {											\
	if (bq->mode == BQ27Z561_MASTER)						\
		printk(KERN_ERR "[bq27z561-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else if (bq->mode == BQ27Z561_SLAVE)						\
		printk(KERN_ERR "[bq27z561--SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else										\
		printk(KERN_ERR "[Unknown]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);
#define bq_log(fmt, ...)								\
do {											\
	if (bq->mode == BQ27Z561_MASTER)						\
		printk(KERN_ERR "[bq27z561-MASTER]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else if (bq->mode == BQ27Z561_SLAVE)						\
		printk(KERN_ERR "[bq27z561--SLAVE]:%s:" fmt, __func__, ##__VA_ARGS__);	\
	else										\
		printk(KERN_ERR "[Unknown]:%s:" fmt, __func__, ##__VA_ARGS__);\
} while(0);
#endif


#define	INVALID_REG_ADDR	0xFF

#define FG_FLAGS_FD				BIT(4)
#define	FG_FLAGS_FC				BIT(5)
#define	FG_FLAGS_DSG				BIT(6)
#define FG_FLAGS_RCA				BIT(9)

#ifdef CONFIG_PRODUCT_MOBA
int get_batt2_vol;
#endif

enum bq_fg_reg_idx {
	BQ_FG_REG_CTRL = 0,
	BQ_FG_REG_TEMP,		/* Battery Temperature */
	BQ_FG_REG_VOLT,		/* Battery Voltage */
	BQ_FG_REG_AI,		/* Average Current */
	BQ_FG_REG_BATT_STATUS,	/* BatteryStatus */
	BQ_FG_REG_TTE,		/* Time to Empty */
	BQ_FG_REG_TTF,		/* Time to Full */
	BQ_FG_REG_FCC,		/* Full Charge Capacity */
	BQ_FG_REG_RM,		/* Remaining Capacity */
	BQ_FG_REG_CC,		/* Cycle Count */
	BQ_FG_REG_SOC,		/* Relative State of Charge */
	BQ_FG_REG_SOH,		/* State of Health */
	BQ_FG_REG_DC,		/* Design Capacity */
	BQ_FG_REG_ALT_MAC,	/* AltManufactureAccess*/
	BQ_FG_REG_MAC_CHKSUM,	/* MACChecksum */
	BQ_FG_REG_MI,		/* Measured Current */
	NUM_REGS,
};

enum bq_fg_mac_cmd {
	FG_MAC_CMD_CTRL_STATUS		= 0x0000,
	FG_MAC_CMD_DEV_TYPE		= 0x0001,
	FG_MAC_CMD_FW_VER		= 0x0002,
	FG_MAC_CMD_HW_VER		= 0x0003,
	FG_MAC_CMD_IF_SIG		= 0x0004,
	FG_MAC_CMD_CHEM_ID		= 0x0006,
	FG_MAC_CMD_GAUGING		= 0x0021,
	FG_MAC_CMD_SEAL			= 0x0030,
	FG_MAC_CMD_DEV_RESET		= 0x0041,
	FG_MAC_CMD_DEV_NAME		= 0x004A,
	FG_MAC_CMD_DEV_CHEM		= 0x004B,
	FG_MAC_CMD_MANU_NAME		= 0x004C,
	FG_MAC_CMD_MANU_DATE		= 0x004D,
	FG_MAC_CMD_SERIAL_NUMBER	= 0x004E,
	FG_MAC_CMD_ITSTATUS1		= 0x0073,
};


enum {
	SEAL_STATE_RSVED,
	SEAL_STATE_UNSEALED,
	SEAL_STATE_SEALED,
	SEAL_STATE_FA,
};

static u8 bq27z561_regs[NUM_REGS] = {
	0x00,	/* CONTROL */
	0x06,	/* TEMP */
	0x08,	/* VOLT */
	0x14,	/* AVG CURRENT */
	0x0A,	/* FLAGS */
	0x16,	/* Time to empty */
	0x18,	/* Time to full */
	0x12,	/* Full charge capacity */
	0x10,	/* Remaining Capacity */
	0x2A,	/* CycleCount */
	0x2C,	/* State of Charge */
	0x2E,	/* State of Health */
	0x3C,	/* Design Capacity */
	0x3E,	/* AltManufacturerAccess*/
	0x60,	/* MACChecksum */
	0x0C,	/* Measured Current*/
};

struct bq_fg_chip {
	struct device *dev;
	struct i2c_client *client;


	struct mutex i2c_rw_lock;
	struct mutex data_lock;
	struct mutex irq_complete;

	bool irq_waiting;
	bool irq_disabled;
	bool resume_completed;

	int fw_ver;
	int df_ver;

	u8 chip;
	u8 regs[NUM_REGS];

	/* status tracking */

	bool batt_fc;
	bool batt_fd;	/* full depleted */

	bool batt_dsg;
	bool batt_rca;	/* remaining capacity alarm */

	int seal_state; /* 0 - Full Access, 1 - Unsealed, 2 - Sealed */
	int batt_tte;
	int batt_ttf;
	int batt_soc;
	int batt_soh;
	int batt_fcc;	/* Full charge capacity */
	int batt_rm;	/* Remaining capacity */
	int batt_dc;	/* Design Capacity */
	int batt_volt;
	int batt_temp;
	int batt_curr;

	int batt_cyclecnt;	/* cycle count */

	/* debug */
	int skip_reads;
	int skip_writes;

	int fake_soc;
	int fake_temp;

	struct power_supply *fg_psy;
	struct power_supply *bq_batt_psy;
	struct power_supply *fg_master_psy;
	struct power_supply *fg_slave_psy;
	struct power_supply *bq2589x_psy;
	struct power_supply *bq2597x_1_psy;
	struct power_supply *bq2597x_2_psy;
	struct power_supply *bq2597x_3_psy;
	struct power_supply *bq2597x_4_psy;
	struct power_supply *qcom_usb_psy;
	struct power_supply *qcom_batt_psy;
	struct power_supply_desc fg_psy_d;
	struct power_supply_desc bq_batt_psy_d;

	int last_batt_soc;
	int last_batt_soh;
	int last_batt_status;
	int last_batt_volt;
	int last_batt_temp;
	int last_batt_present;
	int last_batt_current;
	int last_batt_charge_full;
	int last_batt_design_full;
	int last_batt_cc;

	int mode;
};



static int __fg_read_byte(struct bq_fg_chip *bq, u8 reg, u8 *val)
{
	s32 ret;

#ifdef CONFIG_PRODUCT_MOBA
	if (is_fake_battery)
		return 0;
#endif
	ret = i2c_smbus_read_byte_data(bq->client, reg);
	if (ret < 0) {
		bq_err("i2c read byte fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u8)ret;

	return 0;
}

#if 0
static int __fg_write_byte(struct bq_fg_chip *bq, u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(bq->client, reg, val);
	if (ret < 0) {
		bq_err("i2c write byte fail: can't write 0x%02X to reg 0x%02X\n",
				val, reg);
		return ret;
	}

	return 0;
}
#endif

static int __fg_read_word(struct bq_fg_chip *bq, u8 reg, u16 *val)
{
	s32 ret;

#ifdef CONFIG_PRODUCT_MOBA
	if (is_fake_battery)
		return 0;
#endif
	ret = i2c_smbus_read_word_data(bq->client, reg);
	if (ret < 0) {
		bq_err("i2c read word fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u16)ret;

	return 0;
}


#if 0 //Lenovo
static int __fg_write_word(struct bq_fg_chip *bq, u8 reg, u16 val)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(bq->client, reg, val);
	if (ret < 0) {
		bq_err("i2c write word fail: can't write 0x%02X to reg 0x%02X\n",
				val, reg);
		return ret;
	}

	return 0;
}
#endif

static int __fg_read_block(struct bq_fg_chip *bq, u8 reg, u8 *buf, u8 len)
{

	int ret;

#ifdef CONFIG_PRODUCT_MOBA
	if (is_fake_battery)
		return 0;
#endif

	ret = i2c_smbus_read_i2c_block_data(bq->client, reg, len, buf);

	return ret;
}

static int __fg_write_block(struct bq_fg_chip *bq, u8 reg, u8 *buf, u8 len)
{
	int ret;

#ifdef CONFIG_PRODUCT_MOBA
	if (is_fake_battery)
		return 0;
#endif
	ret = i2c_smbus_write_i2c_block_data(bq->client, reg, len, buf);

	return ret;
}


static int fg_read_byte(struct bq_fg_chip *bq, u8 reg, u8 *val)
{
	int ret;

	if (bq->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_byte(bq, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

#if 0
static int fg_write_byte(struct bq_fg_chip *bq, u8 reg, u8 val)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_byte(bq, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}
#endif

static int fg_read_word(struct bq_fg_chip *bq, u8 reg, u16 *val)
{
	int ret;

	if (bq->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_word(bq, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

#if 0 //Lenovo
static int fg_write_word(struct bq_fg_chip *bq, u8 reg, u16 val)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_word(bq, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}
#endif

static int fg_read_block(struct bq_fg_chip *bq, u8 reg, u8 *buf, u8 len)
{
	int ret;

	if (bq->skip_reads)
		return 0;
	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_read_block(bq, reg, buf, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;

}

static int fg_write_block(struct bq_fg_chip *bq, u8 reg, u8 *data, u8 len)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __fg_write_block(bq, reg, data, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static u8 checksum(u8 *data, u8 len)
{
	u8 i;
	u16 sum = 0;

	for (i = 0; i < len; i++)
		sum += data[i];

	sum &= 0xFF;

	return 0xFF - sum;
}

#if 0
static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{
	int i;
	int idx = 0;
	int num;
	u8 strbuf[128];

	bq_err("%s buf: ", msg);
	for (i = 0; i < len; i++) {
		num = sprintf(&strbuf[idx], "%02X ", buf[i]);
		idx += num;
	}
	bq_err("%s\n", strbuf);
}
#else
static void fg_print_buf(const char *msg, u8 *buf, u8 len)
{}
#endif

#if 0
#define TIMEOUT_INIT_COMPLETED	100
static int fg_check_init_completed(struct bq_fg_chip *bq)
{
	int ret;
	int i = 0;
	u16 status;

	while (i++ < TIMEOUT_INIT_COMPLETED) {
		ret = fg_read_word(bq, bq->regs[BQ_FG_REG_BATT_STATUS], &status);
		if (ret >= 0 && (status & 0x0080))
			return 0;
		msleep(100);
	}
	bq_err("wait for FG INITCOMP timeout\n");
	return ret;
}
#endif

#if 0
static int fg_get_seal_state(struct bq_fg_chip *bq)
{
	int ret;
	u16 status;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CTRL], &status);
	if (ret < 0) {
		bq_err("Failed to read control status, ret = %d\n", ret);
		return ret;
	}
	status &= 0x6000;
	status >>= 13;

	if (status == 1)
		bq->seal_state = SEAL_STATE_FA;
	else if (status == 2)
		bq->seal_state = SEAL_STATE_UNSEALED;
	else if (status == 3)
		bq->seal_state = SEAL_STATE_SEALED;

	return 0;
}

static int fg_unseal_send_key(struct bq_fg_chip *bq, int key)
{
	int ret;

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_ALT_MAC], key & 0xFFFF);

	if (ret < 0) {
		bq_err("unable to write unseal key step 1, ret = %d\n", ret);
		return ret;
	}

	msleep(5);

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_ALT_MAC], (key >> 16) & 0xFFFF);
	if (ret < 0) {
		bq_err("unable to write unseal key step 2, ret = %d\n", ret);
		return ret;
	}

	msleep(100);

	return 0;
}

#define FG_DEFAULT_UNSEAL_KEY	0x80008000
static int fg_unseal(struct bq_fg_chip *bq)
{
	int ret;
	int retry = 0;

	ret = fg_unseal_send_key(bq, FG_DEFAULT_UNSEAL_KEY);
	if (!ret) {
		while (retry++ < 100) {
			ret = fg_get_seal_state(bq);
			if (bq->seal_state == SEAL_STATE_UNSEALED ||
			    bq->seal_state == SEAL_STATE_FA) {
				bq_log("FG is unsealed");
				return 0;
			}
		}
	}

	return -1;
}

#define FG_DEFAULT_UNSEAL_FA_KEY	0x36724614
static int fg_unseal_full_access(struct bq_fg_chip *bq)
{
	int ret;
	int retry = 0;

	ret = fg_unseal_send_key(bq, FG_DEFAULT_UNSEAL_FA_KEY);
	if (!ret) {
		while (retry++ < 100) {
			fg_get_seal_state(bq);
			if (bq->seal_state == SEAL_STATE_FA) {
				bq_log("FG is in full access.");
				return 0;
			}
			msleep(200);
		}
	}

	return -1;
}


static int fg_seal(struct bq_fg_chip *bq)
{
	int ret;
	int retry = 0;

	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_ALT_MAC], FG_MAC_CMD_SEAL);

	if (ret < 0) {
		bq_err("Failed to send seal command\n");
		return ret;
	}

	while (retry++ < 100) {
		fg_get_seal_state(bq);
		if (bq->seal_state == SEAL_STATE_SEALED) {
			bq_log("FG is sealed successfully");
			return 0;
		}
		msleep(200);
	}

	return -1;
}
#endif

static int fg_mac_read_block(struct bq_fg_chip *bq, u16 cmd, u8 *buf, u8 len)
{
	int ret;
	u8 cksum_calc, cksum;
	u8 t_buf[36];
	u8 t_len;
	int i;

	memset(t_buf, 0x00, sizeof(t_buf));
	t_buf[0] = (u8)cmd;
	t_buf[1] = (u8)(cmd >> 8);
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_ALT_MAC], t_buf, 2);
	if (ret < 0)
		return ret;

	msleep(100);

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_read_block(bq, bq->regs[BQ_FG_REG_ALT_MAC], t_buf, 32);
	if (ret < 0)
		return ret;

	ret = fg_read_block(bq, bq->regs[BQ_FG_REG_ALT_MAC] + 32, t_buf + 32, 4);
	if (ret < 0)
		return ret;

	fg_print_buf("mac_read_block", t_buf, 36);

	cksum = t_buf[34];
	t_len = t_buf[35];

	cksum_calc = checksum(t_buf, t_len - 2);
	if (cksum_calc != cksum)
		return 1;

	for (i = 0; i < len; i++)
		buf[i] = t_buf[i+2];

	return 0;
}
#if 0
static int fg_mac_write_block(struct bq_fg_chip *bq, u16 cmd, u8 *data, u8 len)
{
	int ret;
	u8 cksum;
	u8 t_buf[40];
	int i;

	if (len > 32)
		return -1;

	t_buf[0] = (u8)(cmd >> 8);
	t_buf[1] = (u8)cmd;
	for (i = 0; i < len; i++)
		t_buf[i+2] = data[i];

	cksum = checksum(data, len + 2);
	/*write command/addr, data*/
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_ALT_MAC], t_buf, len + 2);
	if (ret < 0)
		return ret;
	t_buf[0] = cksum;
	t_buf[1] = len + 4; /*buf length, cmd, CRC and len byte itself*/
	/*write checksum and length*/
	ret = fg_write_block(bq, bq->regs[BQ_FG_REG_MAC_CHKSUM], t_buf, 2);

	return ret;
}

#endif

static void fg_fw_init(struct bq_fg_chip *bq)
{
	int ret;
	u8 buf[36];

#if 0 //Lenovo
	ret = fg_write_word(bq, bq->regs[BQ_FG_REG_ALT_MAC], FG_MAC_CMD_FW_VER);

	if (ret < 0) {
		bq_err("Failed to send firmware version subcommand:%d\n", ret);
		return;
	}

	mdelay(2);
#endif

	//ret = fg_mac_read_block(bq, bq->regs[BQ_FG_REG_ALT_MAC], buf, 11);
	ret = fg_mac_read_block(bq, FG_MAC_CMD_FW_VER, buf, 11);
	if (ret != 0) {
		bq_err("Failed to read firmware version:%d\n", ret);
#ifdef CONFIG_PRODUCT_MOBA
		is_fake_battery = 1;
#endif
		return;
	}

	bq_log("FW Ver:%04X, Build:%04X\n",
		buf[3] << 8 | buf[2], buf[5] << 8 | buf[4]);
	bq_log("Ztrack Ver:%04X\n", buf[8] << 8 | buf[7]);

	ret = fg_mac_read_block(bq, 0x44C2, buf, 2);
	if (ret != 0) {
		bq_err("Failed to read it gauging configuration:%d\n", ret);
		return;
	}
	buf[2] = '\0';
	bq_log("it gauging configuration:0x%02X%02X\n", buf[1], buf[0]);
}

static int fg_read_status(struct bq_fg_chip *bq)
{
	int ret;
	u16 flags;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_BATT_STATUS], &flags);
	if (ret < 0)
		return ret;

	mutex_lock(&bq->data_lock);
	bq->batt_fc		= !!(flags & FG_FLAGS_FC);
	bq->batt_fd		= !!(flags & FG_FLAGS_FD);
	bq->batt_rca		= !!(flags & FG_FLAGS_RCA);
	bq->batt_dsg		= !!(flags & FG_FLAGS_DSG);
	mutex_unlock(&bq->data_lock);

	return 0;
}


static int fg_read_rsoc(struct bq_fg_chip *bq)
{
#if 0
	int ret;
	u16 soc = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_SOC], &soc);
	if (ret < 0) {
		bq_err("could not read RSOC, ret = %d\n", ret);
		return ret;
	}

	return soc;
#endif
	int cc;
	int soc;
	int real_soc;
	int charge_full;

#ifdef CONFIG_PRODUCT_MOBA
	if (is_fake_battery) {
		bq_err("fake battery soc is %d\n", USER_FAKE_BATTERY_SOC);
		return USER_FAKE_BATTERY_SOC;
	}
#endif
	if (bq->mode == BQ27Z561_MASTER) {
		cc = last_cc1;
		charge_full = last_charge_full1;
	} else if (bq->mode == BQ27Z561_SLAVE) {
		cc = last_cc2;
		charge_full = last_charge_full2;
	}

	soc = (cc * 100 * 1000) / (charge_full * 1000 - charge_full * 50);
	soc = (soc > 100) ? 100 : soc;
	real_soc = cc * 100 / charge_full;
	real_soc = (real_soc > 100) ? 100 : real_soc;

	if (bq->mode == BQ27Z561_MASTER) {
		last_batt1_soc = soc;
		last_batt1_real_soc = real_soc;
	} else if (bq->mode == BQ27Z561_SLAVE) {
		last_batt2_soc = soc;
		last_batt2_real_soc = real_soc;
	}

	return soc;
}

static int fg_read_soh(struct bq_fg_chip *bq)
{
	int ret;
	u16 soh = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_SOH], &soh);
	if (ret < 0) {
		bq_err("could not read SOH, ret = %d\n", ret);
		return ret;
	}

	return soh;

}

static int fg_read_temperature(struct bq_fg_chip *bq)
{
	int ret;
	u16 temp = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TEMP], &temp);
	if (ret < 0) {
		bq_err("could not read temperature, ret = %d\n", ret);
		return ret;
	}

	return temp - 2730;

}

static int fg_read_volt(struct bq_fg_chip *bq)
{
	int ret;
	u16 volt = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_VOLT], &volt);
	if (ret < 0) {
		bq_err("could not read voltage, ret = %d\n", ret);
		return ret;
	}

	return volt;

}

static int fg_read_current(struct bq_fg_chip *bq, int *curr)
{
	int ret;
	u16 avg_curr = 0;

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_MI], &avg_curr);
	if (ret < 0) {
		bq_err("could not read current, ret = %d\n", ret);
		return ret;
	}
	*curr = (int)((s16)avg_curr);

	return ret;
}

static int fg_read_fcc(struct bq_fg_chip *bq)
{
	int ret;
	u16 fcc;

	if (bq->regs[BQ_FG_REG_FCC] == INVALID_REG_ADDR) {
		bq_err("FCC command not supported!\n");
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_FCC], &fcc);

	if (ret < 0)
		bq_err("could not read FCC, ret=%d\n", ret);

	return fcc;
}

static int fg_read_dc(struct bq_fg_chip *bq)
{

	int ret;
	u16 dc;

	if (bq->regs[BQ_FG_REG_DC] == INVALID_REG_ADDR) {
		bq_err("DesignCapacity command not supported!\n");
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_DC], &dc);

	if (ret < 0) {
		bq_err("could not read DC, ret=%d\n", ret);
		return ret;
	}

	return dc;
}


static int fg_read_rm(struct bq_fg_chip *bq)
{
	int ret;
	u16 rm;

	if (bq->regs[BQ_FG_REG_RM] == INVALID_REG_ADDR) {
		bq_err("RemainingCapacity command not supported!\n");
		return 0;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_RM], &rm);

	if (ret < 0) {
		bq_err("could not read DC, ret=%d\n", ret);
		return ret;
	}

	return rm;

}

static int fg_read_cyclecount(struct bq_fg_chip *bq)
{
	int ret;
	u16 cc;

	if (bq->regs[BQ_FG_REG_CC] == INVALID_REG_ADDR) {
		bq_err("Cycle Count not supported!\n");
		return -1;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_CC], &cc);

	if (ret < 0) {
		bq_err("could not read Cycle Count, ret=%d\n", ret);
		return ret;
	}

	return cc;
}

static int fg_read_tte(struct bq_fg_chip *bq)
{
	int ret;
	u16 tte;

	if (bq->regs[BQ_FG_REG_TTE] == INVALID_REG_ADDR) {
		bq_err("Time To Empty not supported!\n");
		return -1;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TTE], &tte);

	if (ret < 0) {
		bq_err("could not read Time To Empty, ret=%d\n", ret);
		return ret;
	}

	if (ret == 0xFFFF)
		return -ENODATA;

	return tte;
}

static int fg_read_ttf(struct bq_fg_chip *bq)
{
	int ret;
	u16 ttf;

	if (bq->regs[BQ_FG_REG_TTF] == INVALID_REG_ADDR) {
		bq_err("Time To Full not supported!\n");
		return -1;
	}

	ret = fg_read_word(bq, bq->regs[BQ_FG_REG_TTF], &ttf);

	if (ret < 0) {
		bq_err("could not read Time To Full, ret=%d\n", ret);
		return ret;
	}

	if (ret == 0xFFFF)
		return -ENODATA;

	return ttf;
}

static int get_first_charger_online(struct bq_fg_chip *bq)
{
	int rc = -EINVAL;
	union power_supply_propval val = {0, };

	if (!bq->qcom_usb_psy) {
		bq->qcom_usb_psy = power_supply_get_by_name("usb");
		if (!bq->qcom_usb_psy)
			return 0;
	}

	rc = power_supply_get_property(bq->qcom_usb_psy,
				POWER_SUPPLY_PROP_ONLINE,
				&val);
	if (rc < 0) {
		bq_err("Couldn't get qcom usb POWER_SUPPLY_PROP_ONLINE, rc=%d\n",
				rc);
		return 0;
	}

	return val.intval;
}

static int get_sec_charger_online(struct bq_fg_chip *bq)
{
	int rc = -EINVAL;
	union power_supply_propval val = {0, };

	if (!bq->bq2589x_psy) {
		bq->bq2589x_psy = power_supply_get_by_name("bq2589h-charger");
		if (!bq->bq2589x_psy)
			return 0;
	}

	rc = power_supply_get_property(bq->bq2589x_psy,
				POWER_SUPPLY_PROP_ONLINE,
				&val);
	if (rc < 0) {
		bq_err("Couldn't get bq25890h battery POWER_SUPPLY_PROP_ONLINE, rc=%d\n",
				rc);
		return 0;
	}

	return val.intval;
}

bool bq_master_psy_initialized(struct bq_fg_chip *bq)
{
	if (bq->fg_master_psy)
		return true;

	bq->fg_master_psy = power_supply_get_by_name("bq27z561-master");
	if (!bq->fg_master_psy)
		return false;

	return true;
}

bool bq_slave_psy_initialized(struct bq_fg_chip *bq)
{
	if (bq->fg_slave_psy)
		return true;

	bq->fg_slave_psy = power_supply_get_by_name("bq27z561-slave");
	if (!bq->fg_slave_psy)
		return false;

	return true;
}

bool bq_battery_psy_initialized(struct bq_fg_chip *bq)
{
	if (bq->fg_psy)
		return true;

	bq->fg_psy = power_supply_get_by_name("bq_fg");
	if (!bq->fg_psy)
		return false;

	return true;
}

static int fg_get_batt_status(struct bq_fg_chip *bq)
{

	union power_supply_propval pval = {0, };
	int rc;
	int batt_status;

	if (is_fake_battery)
		return POWER_SUPPLY_STATUS_DISCHARGING;

#if 0
	fg_read_status(bq);

	if (bq->batt_fc) {
		batt_status = POWER_SUPPLY_STATUS_FULL;
	} else if (bq->batt_dsg)
		batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		batt_status = POWER_SUPPLY_STATUS_CHARGING;
#endif

	if (bq->mode == BQ27Z561_MASTER) {
		if (!bq_master_psy_initialized(bq))
			return POWER_SUPPLY_STATUS_DISCHARGING;

		rc = power_supply_get_property(bq->fg_master_psy,
				POWER_SUPPLY_PROP_CAPACITY, &pval);
		if (rc < 0) {
			bq_err("Failed to read master battery capacity. (2)");
			return POWER_SUPPLY_STATUS_DISCHARGING;
		}
	} else if (bq->mode == BQ27Z561_SLAVE) {
		if (!bq_slave_psy_initialized(bq))
			return POWER_SUPPLY_STATUS_DISCHARGING;

		rc = power_supply_get_property(bq->fg_slave_psy,
				POWER_SUPPLY_PROP_CAPACITY, &pval);
		if (rc < 0) {
			bq_err("Failed to read slave battery capacity. (2)");
			return POWER_SUPPLY_STATUS_DISCHARGING;
		}
	}

	if ((!get_first_charger_online(bq)) && (!get_sec_charger_online(bq))) {
		batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else if (100 == pval.intval) {
		batt_status = POWER_SUPPLY_STATUS_FULL;
	} else {
		if (!(g_chg_en_dev & (1 << CHG_EN_FIR_MAIN)) &&
				!(g_chg_en_dev & (1 << CHG_EN_SEC_MAIN)))
			batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			batt_status = POWER_SUPPLY_STATUS_CHARGING;
	}

	if (get_first_charger_online(bq) || get_sec_charger_online(bq)) {
		if (bq_wake_lock != 1) {
			pm_stay_awake(bq->dev);
			bq_wake_lock = 1;
			pr_info("batt_sys: bq_fg pm_stay_awake\n");
		}
	} else {
		if (bq_wake_lock != 0) {
			pm_relax(bq->dev);
			bq_wake_lock = 0;
			pr_info("batt_sys: bq_fg pm_relax\n");
		}
	}
	return batt_status;
}


static int fg_get_batt_capacity_level(struct bq_fg_chip *bq)
{

	if (bq->batt_fc)
		return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
	else if (bq->batt_rca)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else if (bq->batt_fd)
		return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

}


#if 1
#if 1
bool bq2589x_psy_initialized(struct bq_fg_chip *bq)
{
	if (bq->bq2589x_psy)
		return true;

	bq->bq2589x_psy = power_supply_get_by_name("bq2589h-charger");
	if (!bq->bq2589x_psy)
		return false;

	return true;
}

static int get_slave_battery_temp(struct bq_fg_chip *bq)
{
	int rc;
	int temp;
	union power_supply_propval pval = {0, };

	if (!bq2589x_psy_initialized(bq))
		return USER_BAD_BATTERY_TEMP;

	rc = power_supply_get_property(bq->bq2589x_psy,
			POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		bq_err("Failed to read bq2589x battery temp.");
		return USER_BAD_BATTERY_TEMP;
	}
	temp = pval.intval;

	return temp;
}
#endif

#if 0
bool qcom_battery_psy_initialized(struct bq_fg_chip *bq)
{
	if (bq->qcom_batt_psy)
		return true;

	bq->qcom_batt_psy = power_supply_get_by_name("battery");
	if (!bq->qcom_batt_psy)
		return false;

	return true;
}

static int get_master_battery_temp(struct bq_fg_chip *bq)
{
	int rc;
	int temp;
	union power_supply_propval pval = {0, };

	if (!bq2589x_psy_initialized(bq))
		return USER_BAD_BATTERY_TEMP;

	rc = power_supply_get_property(bq->qcom_batt_psy,
			POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		bq_err("Failed to read qcom battery temp.");
		return USER_BAD_BATTERY_TEMP;
	}
	temp = pval.intval;

	return temp;
}
#endif

static int get_master_battery_temp(struct bq_fg_chip *bq)
{
	int rc = -EINVAL;
	union power_supply_propval val = {0, };

	if (!bq->qcom_batt_psy) {
		bq->qcom_batt_psy = power_supply_get_by_name("battery");
		if (!bq->qcom_batt_psy)
			return USER_BAD_BATTERY_TEMP;
	}

	rc = power_supply_get_property(bq->qcom_batt_psy,
				POWER_SUPPLY_PROP_TEMP,
				&val);
	if (rc < 0) {
		bq_err("Couldn't get qcom battery POWER_SUPPLY_PROP_TEMP, rc=%d\n",
				rc);
		return USER_BAD_BATTERY_TEMP;
	}

	return val.intval;
}
#endif

static enum power_supply_property fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	/*POWER_SUPPLY_PROP_HEALTH,*//*implement it in battery power_supply*/
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_UPDATE_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE,
	POWER_SUPPLY_PROP_SOH,
};

static int fg_get_property(struct power_supply *psy, enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq_fg_chip *bq = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = fg_get_batt_status(bq);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = fg_read_volt(bq);
		mutex_lock(&bq->data_lock);
		//if (ret >= 0)
			bq->batt_volt = ret;
		val->intval = bq->batt_volt * 1000;
		mutex_unlock(&bq->data_lock);

		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		mutex_lock(&bq->data_lock);
		fg_read_current(bq, &bq->batt_curr);
		val->intval = -bq->batt_curr * 1000;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (bq->fake_soc >= 0) {
			val->intval = bq->fake_soc;
			break;
		}
		ret = fg_read_rsoc(bq);
		mutex_lock(&bq->data_lock);
		//if (ret >= 0)
			bq->batt_soc = ret;
		val->intval = bq->batt_soc;
		mutex_unlock(&bq->data_lock);

		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = fg_get_batt_capacity_level(bq);
		break;

	case POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE:
		if (bq->fake_temp != -EINVAL) {
			val->intval = bq->fake_temp;
			break;
		}

		ret = fg_read_temperature(bq);
		mutex_lock(&bq->data_lock);
		//if (ret > 0)
			bq->batt_temp = ret;
		val->intval = bq->batt_temp;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		if (bq->fake_temp != -EINVAL) {
			val->intval = bq->fake_temp;
			break;
		}

		if (bq->mode == BQ27Z561_MASTER) {
			ret = get_master_battery_temp(bq);
		} else if (bq->mode == BQ27Z561_SLAVE) {
			ret = get_slave_battery_temp(bq);
		} else
			ret = USER_BAD_BATTERY_TEMP;

		mutex_lock(&bq->data_lock);
		//if (ret > 0)
			bq->batt_temp = ret;
		val->intval = bq->batt_temp;
		mutex_unlock(&bq->data_lock);

		if (debug_temp_enable) {
			if (bq->mode == BQ27Z561_MASTER)
				val->intval = debug_temp1;
			else if (bq->mode == BQ27Z561_SLAVE)
				val->intval = debug_temp2;
		}
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = fg_read_tte(bq);
		mutex_lock(&bq->data_lock);
		//if (ret >= 0)
			bq->batt_tte = ret;

		val->intval = bq->batt_tte;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = fg_read_ttf(bq);
		mutex_lock(&bq->data_lock);
		//if (ret >= 0)
			bq->batt_ttf = ret;

		val->intval = bq->batt_ttf;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = fg_read_fcc(bq);
		mutex_lock(&bq->data_lock);
		//if (ret > 0)
			bq->batt_fcc = ret;
		val->intval = bq->batt_fcc * 1000;
		mutex_unlock(&bq->data_lock);

		if (bq->mode == BQ27Z561_MASTER)
			last_charge_full1 = bq->batt_fcc;
		else if (bq->mode == BQ27Z561_SLAVE)
			last_charge_full2 = bq->batt_fcc;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = fg_read_dc(bq);
		mutex_lock(&bq->data_lock);
		//if (ret > 0)
			bq->batt_dc = ret;
		val->intval = bq->batt_dc * 1000;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = fg_read_cyclecount(bq);
		mutex_lock(&bq->data_lock);
		//if (ret >= 0)
			bq->batt_cyclecnt = ret;
		val->intval = bq->batt_cyclecnt;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		ret = fg_read_rm(bq);
		mutex_lock(&bq->data_lock);
		//if (ret >= 0)
			bq->batt_rm = ret;
		val->intval = bq->batt_rm;
		mutex_unlock(&bq->data_lock);

		if (bq->mode == BQ27Z561_MASTER)
			last_cc1 = val->intval;
		else if (bq->mode == BQ27Z561_SLAVE)
			last_cc2 = val->intval;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_SOH:
		if (bq->fake_soc >= 0) {
			val->intval = bq->fake_soc;
			break;
		}
		ret = fg_read_soh(bq);
		mutex_lock(&bq->data_lock);
		//if (ret >= 0)
			bq->batt_soh = ret;
		val->intval = bq->batt_soh;
		mutex_unlock(&bq->data_lock);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
static void fg_dump_registers(struct bq_fg_chip *bq);

static int fg_set_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       const union power_supply_propval *val)
{
	struct bq_fg_chip *bq = power_supply_get_drvdata(psy);

	switch (prop) {
#if 0
	case POWER_SUPPLY_PROP_TEMP:
		bq->fake_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		bq->fake_soc = val->intval;
		power_supply_changed(bq->fg_psy);
		break;
#endif
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		fg_dump_registers(bq);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (debug_temp_enable) {
			if (bq->mode == BQ27Z561_MASTER)
				debug_temp1 = val->intval;
			else if (bq->mode == BQ27Z561_SLAVE)
				debug_temp2 = val->intval;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int fg_prop_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
#if 0
	case POWER_SUPPLY_PROP_CAPACITY:
#endif
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}


static enum power_supply_property bq_batt_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_RESISTANCE_ID,
	POWER_SUPPLY_PROP_UPDATE_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE,
	POWER_SUPPLY_PROP_SOH,
};

static int bq_user_get_batt_capacity(struct bq_fg_chip *bq)
{
	union power_supply_propval pval = {0, };
	int rc;
	int soc, soc2, soc_tmp;

#ifdef CONFIG_PRODUCT_MOBA
	if (is_fake_battery)
		return USER_FAKE_BATTERY_SOC;
#endif

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return USER_BAD_BATTERY_SOC;

	rc = power_supply_get_property(bq->fg_master_psy,
			POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery capacity.");
		//return bq->last_batt_soc;
		return USER_FAKE_BATTERY_SOC;
	}
	soc = pval.intval;

	rc = power_supply_get_property(bq->fg_slave_psy,
			POWER_SUPPLY_PROP_CAPACITY, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery capacity.");
		//return bq->last_batt_soc;
		return USER_FAKE_BATTERY_SOC;
	}
	soc2 = pval.intval;

	if ((soc < 0) && (soc2 < 0))
		return bq->last_batt_soc;
	else if (soc < 0)
		return bq->last_batt_soc;
	else if (soc2 < 0)
		return bq->last_batt_soc;
	else {
		if (!bq_battery_psy_initialized(bq))
			return bq->last_batt_soc;

		rc = power_supply_get_property(bq->fg_psy,
				POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
		if (rc < 0) {
			bq_err("Failed to read bq battery current_now. (3)");
			return bq->last_batt_soc;
		}

		soc_tmp= (soc + soc2) / 2 + (soc + soc2) % 2;
		if ((pval.intval < 0) && (soc_tmp < bq->last_batt_soc)) {
			bq_err("batt_sys: WA for soc %d, %d, %d", bq->last_batt_soc, soc_tmp, pval.intval);
			return bq->last_batt_soc;
		}

		bq->last_batt_soc = soc_tmp;
	}
	return bq->last_batt_soc;
}

static int bq_user_get_batt_soh(struct bq_fg_chip *bq)
{
	union power_supply_propval pval = {0, };
	int rc;
	int soh;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return 0;

	rc = power_supply_get_property(bq->fg_master_psy,
			POWER_SUPPLY_PROP_SOH, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery soh.");
		//return bq->last_batt_soh;
		return 0;
	}
	soh = pval.intval;

	rc = power_supply_get_property(bq->fg_slave_psy,
			POWER_SUPPLY_PROP_SOH, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery soh.");
		//return bq->last_batt_soh;
		return 0;
	}
	bq->last_batt_soh = (soh + pval.intval) / 2;
	return bq->last_batt_soh;
}

static int bq_user_get_batt_status(struct bq_fg_chip *bq)
{
	union power_supply_propval pval = {0, };
	int rc;
	int status;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	rc = power_supply_get_property(bq->fg_master_psy,
			POWER_SUPPLY_PROP_STATUS, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery status.");
		return bq->last_batt_status;
	}
	status = pval.intval;

	rc = power_supply_get_property(bq->fg_slave_psy,
			POWER_SUPPLY_PROP_STATUS, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery status.");
		return bq->last_batt_status;
	}


	//if ((status == POWER_SUPPLY_STATUS_FULL) &&
	//		(bq25890_chg_status == POWER_SUPPLY_STATUS_NOT_CHARGING))
	//	return POWER_SUPPLY_STATUS_FULL;
	if ((status == POWER_SUPPLY_STATUS_FULL) &&
			(pval.intval == POWER_SUPPLY_STATUS_FULL))
		return POWER_SUPPLY_STATUS_FULL;
	else if ((status == POWER_SUPPLY_STATUS_CHARGING) ||
			(pval.intval == POWER_SUPPLY_STATUS_CHARGING))
		return POWER_SUPPLY_STATUS_CHARGING;
	else if ((status == POWER_SUPPLY_STATUS_NOT_CHARGING) &&
			(pval.intval == POWER_SUPPLY_STATUS_NOT_CHARGING))
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	else
		return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int bq_user_get_batt_volt(struct bq_fg_chip *bq)
{
	union power_supply_propval pval = {0, };
	int rc;
	int volt = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return USER_BAD_BATTERY_VOLTAGE;

	rc = power_supply_get_property(bq->fg_master_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery voltage.");
		return bq->last_batt_volt;
	}
	volt = pval.intval;

	rc = power_supply_get_property(bq->fg_slave_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &pval);
#ifdef CONFIG_PRODUCT_MOBA
	get_batt2_vol = pval.intval;
#endif

	if (rc < 0) {
		bq_err("Failed to read slave battery voltage.");
		return bq->last_batt_volt;
	}

	bq->last_batt_volt = (volt + pval.intval) / 2;
	return bq->last_batt_volt;
}

static int bq_user_get_batt_persent(struct bq_fg_chip *bq)
{
	union power_supply_propval pval = {0, };
	int rc;
	int present = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return 0;

	rc = power_supply_get_property(bq->fg_master_psy,
			POWER_SUPPLY_PROP_PRESENT, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery present.");
		return bq->last_batt_present;
	}
	present = pval.intval;

	rc = power_supply_get_property(bq->fg_slave_psy,
			POWER_SUPPLY_PROP_PRESENT, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery present.");
		return bq->last_batt_present;
	}

	return (present & pval.intval);
}

static int bq_user_get_batt_current(struct bq_fg_chip *bq)
{
	union power_supply_propval pval = {0, };
	int rc;
	int curr = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return USER_BAD_BATTERY_CURRENT;

	rc = power_supply_get_property(bq->fg_master_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery current.");
		return bq->last_batt_current;
	}
	curr = pval.intval;

	rc = power_supply_get_property(bq->fg_slave_psy,
			POWER_SUPPLY_PROP_CURRENT_NOW, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery voltage.");
		return bq->last_batt_current;
	}

	bq->last_batt_current = curr + pval.intval;
	return bq->last_batt_current;
}

static int bq_user_get_batt_temp(struct bq_fg_chip *bq)
{
	union power_supply_propval pval = {0, };
	int rc;
	int temp = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return USER_BAD_BATTERY_TEMP;

	rc = power_supply_get_property(bq->fg_master_psy,
			POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery temp.");
		return bq->last_batt_temp;
	}
	temp = pval.intval;

	rc = power_supply_get_property(bq->fg_slave_psy,
			POWER_SUPPLY_PROP_TEMP, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery temp.");
		return bq->last_batt_temp;
	}

	bq->last_batt_temp = (temp > pval.intval) ? temp : pval.intval;
	return bq->last_batt_temp;
}

static int bq_user_get_die_temp(struct bq_fg_chip *bq)
{
	union power_supply_propval pval = {0, };
	int rc;
	int temp = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return USER_BAD_BATTERY_TEMP;

	rc = power_supply_get_property(bq->fg_master_psy,
			POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery temp.");
		return USER_BAD_BATTERY_TEMP;
		//return bq->last_batt_temp;
	}
	temp = pval.intval;

	rc = power_supply_get_property(bq->fg_slave_psy,
			POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery temp.");
		return USER_BAD_BATTERY_TEMP;
		//return bq->last_batt_temp;
	}

	bq->last_batt_temp = (temp > pval.intval) ? temp : pval.intval;
	return bq->last_batt_temp;
}

static int bq_user_get_batt_charge_full(struct bq_fg_chip *bq)
{
	union power_supply_propval pval = {0, };
	int rc;
	int charge_full = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return 0;

	rc = power_supply_get_property(bq->fg_master_psy,
			POWER_SUPPLY_PROP_CHARGE_FULL, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery charge full.");
		return bq->last_batt_charge_full;
	}
	charge_full = pval.intval;

	rc = power_supply_get_property(bq->fg_slave_psy,
			POWER_SUPPLY_PROP_CHARGE_FULL, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery charge full.");
		return bq->last_batt_charge_full;
	}

	bq->last_batt_charge_full = charge_full + pval.intval;
	return bq->last_batt_charge_full;
}

static int bq_user_get_batt_desgin_full(struct bq_fg_chip *bq)
{
	union power_supply_propval pval = {0, };
	int rc;
	int design_full = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return 0;

	rc = power_supply_get_property(bq->fg_master_psy,
			POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery charge full desgin.");
		return bq->last_batt_design_full;
	}
	design_full = pval.intval;

	rc = power_supply_get_property(bq->fg_slave_psy,
			POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery charge full desgin.");
		return bq->last_batt_design_full;
	}

	bq->last_batt_design_full = design_full + pval.intval;
	return bq->last_batt_design_full;
}

static int bq_user_get_batt_charge_counter(struct bq_fg_chip *bq)
{
	union power_supply_propval pval = {0, };
	int rc;
	int cc = 0;

	if ((!bq_master_psy_initialized(bq)) || (!bq_slave_psy_initialized(bq)))
		return 0;

	rc = power_supply_get_property(bq->fg_master_psy,
			POWER_SUPPLY_PROP_CHARGE_COUNTER, &pval);
	if (rc < 0) {
		bq_err("Failed to read master battery cc.");
		return bq->last_batt_cc;
	}
	cc = pval.intval;

	rc = power_supply_get_property(bq->fg_slave_psy,
			POWER_SUPPLY_PROP_CHARGE_COUNTER, &pval);
	if (rc < 0) {
		bq_err("Failed to read slave battery cc.");
		return bq->last_batt_cc;
	}

	bq->last_batt_cc = cc + pval.intval;
	return bq->last_batt_cc;
}

static int bq_batt_get_property(struct power_supply *psy, enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq_fg_chip *bq = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bq_user_get_batt_status(bq);
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq_user_get_batt_volt(bq);
		break;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq_user_get_batt_persent(bq);
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = bq_user_get_batt_current(bq);
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq_user_get_batt_capacity(bq);
		last_user_soc = val->intval;
		break;

	case POWER_SUPPLY_PROP_SOH:
		val->intval = bq_user_get_batt_soh(bq);
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = fg_get_batt_capacity_level(bq);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq_user_get_batt_temp(bq);
		break;

	case POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE:
		val->intval = bq_user_get_die_temp(bq);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = fg_read_tte(bq);
		mutex_lock(&bq->data_lock);
		if (ret >= 0)
			bq->batt_tte = ret;

		val->intval = bq->batt_tte;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = fg_read_ttf(bq);
		mutex_lock(&bq->data_lock);
		if (ret >= 0)
			bq->batt_ttf = ret;

		val->intval = bq->batt_ttf;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bq_user_get_batt_charge_full(bq);
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bq_user_get_batt_desgin_full(bq);
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = fg_read_cyclecount(bq);
		mutex_lock(&bq->data_lock);
		if (ret >= 0)
			bq->batt_cyclecnt = ret;
		val->intval = bq->batt_cyclecnt;
		mutex_unlock(&bq->data_lock);
		break;

	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		val->intval = bq_user_get_batt_charge_counter(bq);
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
		break;

	case POWER_SUPPLY_PROP_RESISTANCE_ID:
		val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		val->intval = 0;
		break;

	default:
		return -EINVAL;
	}
	return 0;
}

static int bq_batt_set_property(struct power_supply *psy,
			       enum power_supply_property prop,
			       const union power_supply_propval *val)
{
	struct bq_fg_chip *bq = power_supply_get_drvdata(psy);

	switch (prop) {
#if 0
	case POWER_SUPPLY_PROP_TEMP:
		bq->fake_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		bq->fake_soc = val->intval;
		power_supply_changed(bq->fg_psy);
		break;
#endif
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		fg_dump_registers(bq);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static int bq_batt_prop_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int ret;

	switch (prop) {
#if 0
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
#endif
	case POWER_SUPPLY_PROP_UPDATE_NOW:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static int fg_psy_register(struct bq_fg_chip *bq)
{
	struct power_supply_config fg_psy_cfg = {};

	if (bq->mode == BQ27Z561_MASTER)
		bq->fg_psy_d.name = "bq27z561-master";
	else
		bq->fg_psy_d.name = "bq27z561-slave";
	bq->fg_psy_d.type = POWER_SUPPLY_TYPE_BMS;
	bq->fg_psy_d.properties = fg_props;
	bq->fg_psy_d.num_properties = ARRAY_SIZE(fg_props);
	bq->fg_psy_d.get_property = fg_get_property;
	bq->fg_psy_d.set_property = fg_set_property;
	bq->fg_psy_d.property_is_writeable = fg_prop_is_writeable;

	fg_psy_cfg.drv_data = bq;
	fg_psy_cfg.num_supplicants = 0;
	bq->fg_psy = devm_power_supply_register(bq->dev,
						&bq->fg_psy_d,
						&fg_psy_cfg);
	if (IS_ERR(bq->fg_psy)) {
		bq_err("Failed to register fg_psy");
		return PTR_ERR(bq->fg_psy);
	}

	return 0;
}

static int bq_batt_psy_register(struct bq_fg_chip *bq)
{
	struct power_supply_config bq_batt_psy_cfg = {};

	bq->bq_batt_psy_d.name = "bq-battery";
		
	bq->bq_batt_psy_d.type = POWER_SUPPLY_TYPE_BATTERY;
	bq->bq_batt_psy_d.properties = bq_batt_props;
	bq->bq_batt_psy_d.num_properties = ARRAY_SIZE(bq_batt_props);
	bq->bq_batt_psy_d.get_property = bq_batt_get_property;
	bq->bq_batt_psy_d.set_property = bq_batt_set_property;
	bq->bq_batt_psy_d.property_is_writeable = bq_batt_prop_is_writeable;

	bq_batt_psy_cfg.drv_data = bq;
	bq_batt_psy_cfg.num_supplicants = 0;
	bq->bq_batt_psy = devm_power_supply_register(bq->dev,
						&bq->bq_batt_psy_d,
						&bq_batt_psy_cfg);
	if (IS_ERR(bq->bq_batt_psy)) {
		bq_err("Failed to register bq_batt_psy");
		return PTR_ERR(bq->bq_batt_psy);
	}

	return 0;
}

static void fg_psy_unregister(struct bq_fg_chip *bq)
{

	power_supply_unregister(bq->fg_psy);
	if (bq->mode == BQ27Z561_MASTER)
		power_supply_unregister(bq->bq_batt_psy);
}

static const u8 fg_dump_regs[] = {
	0x00, 0x02, 0x04, 0x06,
	0x08, 0x0A, 0x0C, 0x0E,
	0x10, 0x16, 0x18, 0x1A,
	0x1C, 0x1E, 0x20, 0x28,
	0x2A, 0x2C, 0x2E, 0x30,
	0x66, 0x68, 0x6C, 0x6E,
	//0x70,
};

static ssize_t fg_attr_show_Ra_table(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	u8 t_buf[40];
	u8 temp_buf[40];
	int ret;
	int i, idx, len;

	ret = fg_mac_read_block(bq, 0x40C0, t_buf, 32);
	if (ret != 0) {
		bq_err("Failed to read Ra table:%d\n", ret);
		return 0;
	}

	idx = 0;
	len = sprintf(temp_buf, "Ra Flag:0x%02X\n", t_buf[1] << 8 | t_buf[0]);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;
	len = sprintf(temp_buf, "RaTable:\n");
	memcpy(&buf[idx], temp_buf, len);
	idx += len;
	for (i = 1; i < 16; i++) {
		len = sprintf(temp_buf, "%d ", t_buf[i*2 + 1] << 8 | t_buf[i*2]);
		memcpy(&buf[idx], temp_buf, len);
		idx += len;
	}

	return idx;
}

static ssize_t fg_attr_show_Qmax(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	int ret;
	u8 t_buf[64];
	int len;

	memset(t_buf, 0, 64);

	ret = fg_mac_read_block(bq, 0x4146, t_buf, 2);
	if (ret < 0) {
		bq_err("Failed to read Qmax:%d\n", ret);
		return 0;
	}

	//len = sprintf(buf, "Qmax Cell 0 = %d\n", (t_buf[1] << 8) | t_buf[0]);
	len = sprintf(buf, "%d\n", (t_buf[1] << 8) | t_buf[0]);

	return len;
}

static ssize_t fg_attr_show_Deivce_info(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 t_buf[40];
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	u8 temp_buf[40];
	int ret;
	int idx, len;

	idx = 0;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_DEV_TYPE, t_buf, 2);
	if (ret != 0) {
		bq_err("Failed to read device type:%d\n", ret);
		return 0;
	} 
	len = sprintf(temp_buf, "Device Type:%04X\n", t_buf[0] | t_buf[1] << 8);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_HW_VER, t_buf, 2);
	if (ret != 0) {
		bq_err("Failed to read hardware version:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "HW Ver:%04X\n", t_buf[0] | t_buf[1] << 8);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_CHEM_ID, t_buf, 2);
	if (ret != 0) {
		bq_err("Failed to read chemical ID:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "Chemical ID:%04X\n", t_buf[0] | t_buf[1] << 8);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_DEV_NAME, t_buf, 36);
	if (ret != 0) {
		bq_err("Failed to read devie name:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "Dev Name:%s\n", t_buf);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_DEV_CHEM, t_buf, 36);
	if (ret != 0) {
		bq_err("Failed to read devie chem:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "Dev Chem:%s\n", t_buf);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_MANU_NAME, t_buf, 36);
	if (ret != 0) {
		bq_err("Failed to read manufacturer name:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "Manufacturer:%s\n", t_buf);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_MANU_DATE, t_buf, 36);
	if (ret != 0) {
		bq_err("Failed to read manufacture date:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "Manu Date:%s\n", t_buf);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	memset(t_buf, 0x00, sizeof(t_buf));
	ret = fg_mac_read_block(bq, FG_MAC_CMD_SERIAL_NUMBER, t_buf, 36);
	if (ret != 0) {
		bq_err("Failed to read serial number:%d\n", ret);
		return 0;
	}
	len = sprintf(temp_buf, "Serial:%04X\n", t_buf[0] | t_buf[1] << 8);
	memcpy(&buf[idx], temp_buf, len);
	idx += len;

	return idx;
}


static ssize_t fg_attr_show_fw(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);
	int ret;
	u8 t_buf[64];
	int len;

	memset(t_buf, 0, 64);

	ret = fg_mac_read_block(bq, FG_MAC_CMD_FW_VER, t_buf, 11);
	if (ret != 0) {
		bq_err("Failed to read firmware version:%d\n", ret);
		len = sprintf(buf, "%04X\n", 0x0);
	} else
		len = sprintf(buf, "%04X\n", t_buf[3] << 8 | t_buf[2]);

	return len;
}

static int fg_read_itstatus1(struct bq_fg_chip *bq, int of)
{
	int ret;
	u8 buf[36];

	ret = fg_mac_read_block(bq, FG_MAC_CMD_ITSTATUS1, buf, 20);
	if (ret != 0) {
		bq_err("Failed to read itstatus1, offset=%d, err=%d\n", of, ret);
		return 0;
	}

	return (buf[of + 1] << 8 | buf[of]);
}

static ssize_t fg_attr_show_TrueRemQ(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", fg_read_itstatus1(bq, 0));
}

static ssize_t fg_attr_show_TrueFullChgQ(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", fg_read_itstatus1(bq, 8));
}

static ssize_t fg_attr_show_T_sim(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", fg_read_itstatus1(bq, 12));
}

static ssize_t fg_attr_show_T_ambient(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", fg_read_itstatus1(bq, 14));
}

static DEVICE_ATTR(RaTable, S_IRUGO, fg_attr_show_Ra_table, NULL);
static DEVICE_ATTR(Qmax, S_IRUGO, fg_attr_show_Qmax, NULL);
static DEVICE_ATTR(DeviceInfo, S_IRUGO, fg_attr_show_Deivce_info, NULL);
static DEVICE_ATTR(fw_version, S_IRUGO, fg_attr_show_fw, NULL);
static DEVICE_ATTR(TrueRemQ, S_IRUGO, fg_attr_show_TrueRemQ, NULL);
static DEVICE_ATTR(TrueFullChgQ, S_IRUGO, fg_attr_show_TrueFullChgQ, NULL);
static DEVICE_ATTR(Tsim, S_IRUGO, fg_attr_show_T_sim, NULL);
static DEVICE_ATTR(Tambient, S_IRUGO, fg_attr_show_T_ambient, NULL);

static struct attribute *fg_attributes[] = {
	&dev_attr_RaTable.attr,
	&dev_attr_Qmax.attr,
	&dev_attr_DeviceInfo.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_TrueRemQ.attr,
	&dev_attr_TrueFullChgQ.attr,
	&dev_attr_Tsim.attr,
	&dev_attr_Tambient.attr,
	NULL,
};

static const struct attribute_group fg_attr_group = {
	.attrs = fg_attributes,
};


static void fg_dump_registers(struct bq_fg_chip *bq)
{
	int i;
	int ret;
	u16 val;

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		msleep(5);
		ret = fg_read_word(bq, fg_dump_regs[i], &val);
		if (!ret)
			bq_err("Reg[%02X] = 0x%04X\n", fg_dump_regs[i], val);
	}
}

static irqreturn_t fg_irq_thread(int irq, void *dev_id)
{
	struct bq_fg_chip *bq = dev_id;
	u8 status;
	int ret;

	mutex_lock(&bq->irq_complete);
	bq->irq_waiting = true;
	if (!bq->resume_completed) {
		bq_err("IRQ triggered before device resume\n");
		if (!bq->irq_disabled) {
			disable_irq_nosync(irq);
			bq->irq_disabled = true;
		}
		mutex_unlock(&bq->irq_complete);
		return IRQ_HANDLED;
	}
	bq->irq_waiting = false;

	fg_read_status(bq);

	fg_dump_registers(bq);

	mutex_lock(&bq->data_lock);

	bq->batt_soc = fg_read_rsoc(bq);
	bq->batt_soh = fg_read_soh(bq);
	bq->batt_volt = fg_read_volt(bq);
	fg_read_current(bq, &bq->batt_curr);
	bq->batt_temp = fg_read_temperature(bq);
	bq->batt_rm = fg_read_rm(bq);

	mutex_unlock(&bq->data_lock);

	bq_log("RSOC:%d, Volt:%d, Current:%d, Temperature:%d\n",
		bq->batt_soc, bq->batt_volt, bq->batt_curr, bq->batt_temp);

	ret = fg_read_byte(bq, 0x6E, &status); /*InterruptStatus Reg*/
	if (!ret) {
		bq_log("VOLT_HI %s\n", status & 0x01 ? "set" : "clear");
		bq_log("TEMP_HI %s\n", status & 0x04 ? "set" : "clear");
		bq_log("VOLT_LOW %s\n", status & 0x02 ? "set" : "clear");
		bq_log("TEMP_LOW %s\n", status & 0x08 ? "set" : "clear");
	}
	return IRQ_HANDLED;
}


static void determine_initial_status(struct bq_fg_chip *bq)
{
	fg_irq_thread(bq->client->irq, bq);
}

static struct of_device_id bq_fg_match_table[] = {
	{
		.compatible = "ti,bq27z561-master",
		.data = &bq27z561_mode_data[BQ27Z561_MASTER],
	},
	{
		.compatible = "ti,bq27z561-slave",
		.data = &bq27z561_mode_data[BQ27Z561_SLAVE],
	},
	{},
};
//MODULE_DEVICE_TABLE(of, bq_fg_match_table);

static int bq_fg_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{

	int ret;
	struct bq_fg_chip *bq;
	const struct of_device_id *match;
	struct device_node *node = client->dev.of_node;
	u8 *regs;

	pr_err("enter\n");
	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);

	if (!bq)
		return -ENOMEM;

	bq->dev = &client->dev;
	bq->client = client;
	bq->chip = id->driver_data;

	bq->batt_soc	= -ENODATA;
	bq->batt_soh	= -ENODATA;
	bq->batt_fcc	= -ENODATA;
	bq->batt_rm	= -ENODATA;
	bq->batt_dc	= -ENODATA;
	bq->batt_volt	= -ENODATA;
	bq->batt_temp	= -ENODATA;
	bq->batt_curr	= -ENODATA;
	bq->batt_cyclecnt = -ENODATA;

	bq->last_batt_soc = 0;
	bq->last_batt_soh = 0;
	bq->last_batt_volt = 0;
	bq->last_batt_temp = 0;
	bq->last_batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
	bq->last_batt_present = 1;
	bq->last_batt_current = 0;
	bq->last_batt_charge_full = 0;
	bq->last_batt_design_full = 0;
	bq->last_batt_cc = 0;

	bq->fake_soc	= -EINVAL;
	bq->fake_temp	= -EINVAL;

	match = of_match_node(bq_fg_match_table, node);
	if (match == NULL) {
		pr_err("device tree match not found!\n");
		return -ENODEV;
	}
	bq->mode	= *(int *)match->data;
	pr_err("check %s\n", device2str[bq->mode]);

	if (bq->chip == BQ27Z561_MASTER) {
		regs = bq27z561_regs;
	} else if (bq->chip == BQ27Z561_SLAVE) {
		regs = bq27z561_regs;
	} else {
		bq_err("unexpected fuel gauge: %d\n", bq->chip);
		regs = bq27z561_regs;
	}

	memcpy(bq->regs, regs, NUM_REGS);

	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);
	mutex_init(&bq->irq_complete);

	bq->resume_completed = true;
	bq->irq_waiting = false;

	if (client->irq) {
		ret = devm_request_threaded_irq(&client->dev, client->irq, NULL,
			fg_irq_thread,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"bq fuel gauge irq", bq);
		if (ret < 0) {
			bq_err("request irq for irq=%d failed, ret = %d\n", client->irq, ret);
			goto err_1;
		}
		enable_irq_wake(client->irq);
	}

	device_init_wakeup(bq->dev, 1);

	fg_fw_init(bq);

	fg_psy_register(bq);

	if (bq->mode == BQ27Z561_MASTER)
		bq_batt_psy_register(bq);

	ret = sysfs_create_group(&bq->dev->kobj, &fg_attr_group);
	if (ret)
		bq_err("Failed to register sysfs, err:%d\n", ret);

	determine_initial_status(bq);

	bq_log("bq fuel gauge probe successfully, %s\n",
			device2str[bq->chip]);

	return 0;

err_1:
	fg_psy_unregister(bq);
	return ret;
}


static inline bool is_device_suspended(struct bq_fg_chip *bq)
{
	return !bq->resume_completed;
}


static int bq_fg_suspend(struct device *dev)
{
#if 0
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = false;
	mutex_unlock(&bq->irq_complete);

	return 0;
#else
	return 0;
#endif
}

static int bq_fg_suspend_noirq(struct device *dev)
{
#if 0
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	if (bq->irq_waiting) {
		pr_err_ratelimited("Aborting suspend, an interrupt was detected while suspending, %d\n", bq->mode);
		return -EBUSY;
	}
	return 0;
#else
	return 0;
#endif
}


static int bq_fg_resume(struct device *dev)
{
#if 0
	struct i2c_client *client = to_i2c_client(dev);
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	mutex_lock(&bq->irq_complete);
	bq->resume_completed = true;
	if (bq->irq_waiting) {
		bq->irq_disabled = false;
		enable_irq(client->irq);
		mutex_unlock(&bq->irq_complete);
		fg_irq_thread(client->irq, bq);
	} else {
		mutex_unlock(&bq->irq_complete);
	}

	power_supply_changed(bq->fg_psy);

	return 0;
#else
	return 0;
#endif
}

static int bq_fg_remove(struct i2c_client *client)
{
	struct bq_fg_chip *bq = i2c_get_clientdata(client);

	fg_psy_unregister(bq);

	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);
	mutex_destroy(&bq->irq_complete);

	sysfs_remove_group(&bq->dev->kobj, &fg_attr_group);

	return 0;

}

static void bq_fg_shutdown(struct i2c_client *client)
{
	pr_err("bq fuel gauge driver shutdown!\n");
}


static const struct i2c_device_id bq_fg_id[] = {
	{ "bq27z561-master", BQ27Z561_MASTER },
	{ "bq27z561-slave", BQ27Z561_SLAVE },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq_fg_id);

static const struct dev_pm_ops bq_fg_pm_ops = {
	.resume		= bq_fg_resume,
	.suspend_noirq = bq_fg_suspend_noirq,
	.suspend	= bq_fg_suspend,
};

static struct i2c_driver bq_fg_driver = {
	.driver	= {
		.name   = "bq_fg",
		.owner  = THIS_MODULE,
		.of_match_table = bq_fg_match_table,
		.pm     = &bq_fg_pm_ops,
	},
	.id_table       = bq_fg_id,

	.probe          = bq_fg_probe,
	.remove		= bq_fg_remove,
	.shutdown	= bq_fg_shutdown,

};

module_i2c_driver(bq_fg_driver);

MODULE_DESCRIPTION("TI BQ27Z561 Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Texas Instruments");

