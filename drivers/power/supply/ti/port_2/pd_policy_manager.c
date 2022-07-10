#define pr_fmt(fmt)	"[USBPD-SEC-PM]: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/power_supply.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/usb/usbpd.h>
#include <linux/slab.h>
#include "pd_policy_manager.h"
#include "../../qcom/smb5-lib.h"

#ifdef CONFIG_PRODUCT_MOBA
#define SUPPORT_ONSEMI_PDCONTROL
#endif

#define PD_SRC_PDO_TYPE_FIXED		0
#define PD_SRC_PDO_TYPE_BATTERY		1
#define PD_SRC_PDO_TYPE_VARIABLE	2
#define PD_SRC_PDO_TYPE_AUGMENTED	3

#define BATT_MAX_CHG_VOLT		4400

#ifdef SUPPORT_ONSEMI_PDCONTROL
#define BATT_FAST_CHG_CURR		7500		//bq2597x max flash current
#else
#define BATT_FAST_CHG_CURR		8000
#endif

#define	BUS_OVP_THRESHOLD		12000
#define	BUS_OVP_ALARM_THRESHOLD		9500

#ifdef SUPPORT_ONSEMI_PDCONTROL
#define BUS_VOLT_INIT_UP		600
#else
#define BUS_VOLT_INIT_UP		300
#endif


#define BAT_VOLT_LOOP_LMT		BATT_MAX_CHG_VOLT
#define BAT_CURR_LOOP_LMT		BATT_FAST_CHG_CURR
#define BUS_VOLT_LOOP_LMT		BUS_OVP_THRESHOLD

#ifdef SUPPORT_ONSEMI_PDCONTROL
#define PM_WORK_RUN_INTERVAL		1000			//every 1s start bq2597x work sent request pdo info
#else
#define PM_WORK_RUN_INTERVAL		200			//default 200ms bq2597x work sent request pdo info
#endif

#ifdef SUPPORT_ONSEMI_PDCONTROL
struct usbpd_pm *bq_usbpd_pm = NULL;
extern PPSStatus_t *fusb_pps_st;
extern struct charger_object* fusb_fetch_pdo_chip;
extern struct fusb30x_chip* fusb_pps_chip;
extern int pd2_max_current;
extern bool is_batt2_stop;
extern bool is_batt2_thermal_stop;
extern bool is_batt2_full;
int is_pd2_done = 0;
int realpd2_request_curr;
extern bool fusb_pd_enable;
#endif

enum {
	PM_ALGO_RET_OK,
	PM_ALGO_RET_THERM_FAULT,
	PM_ALGO_RET_OTHER_FAULT,
	PM_ALGO_RET_CHG_DISABLED,
	PM_ALGO_RET_TAPER_DONE,
};

static const struct pdpm_config pm_config = {
	.bat_volt_lp_lmt		= BAT_VOLT_LOOP_LMT,
	.bat_curr_lp_lmt		= BAT_CURR_LOOP_LMT + 1000,
	.bus_volt_lp_lmt		= BUS_VOLT_LOOP_LMT,
	.bus_curr_lp_lmt		= BAT_CURR_LOOP_LMT >> 1 ,

	.fc2_taper_current		= 2000,
	.fc2_steps			= 1,

	.min_adapter_volt_allowed	= 11000,
	.min_adapter_curr_allowed	= 2000,

	.min_vbat_for_cp		= 3500,

	.cp_sec_enable			= true,
	.fc2_disable_sw			= true,
};

static struct usbpd_pm *__pdpm;

static int fc2_taper_timer;
static int ibus_lmt_change_timer;


static void usbpd_check_usb_psy(struct usbpd_pm *pdpm)
{
	if (!pdpm->usb_psy) {
		pdpm->usb_psy = power_supply_get_by_name("usb");
		if (!pdpm->usb_psy)
			pr_err("sec port usb psy not found!\n");
		else
			pr_info("sec port usb psy ok\n");
	}
}
static void usbpd_check_cp_psy(struct usbpd_pm *pdpm)
{

	if (!pdpm->cp_psy) {
		if (pm_config.cp_sec_enable)
			pdpm->cp_psy = power_supply_get_by_name("bq2597x-sec-master");
		else
			pdpm->cp_psy = power_supply_get_by_name("bq2597x-s-standalone");
		if (!pdpm->cp_psy)
			pr_err("sec port cp_psy not found\n");
		else
			pr_info("sec port cp_psy ok\n");
	}
}

static void usbpd_check_cp_sec_psy(struct usbpd_pm *pdpm)
{
	if (!pdpm->cp_sec_psy) {
		pdpm->cp_sec_psy = power_supply_get_by_name("bq2597x-sec-slave");
		if (!pdpm->cp_sec_psy)
			pr_err("sec port cp_sec_psy not found\n");
		else
			pr_info("sec port cp_sec_psy ok\n");
	}
}

static void usbpd_pm_update_cp_status(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_cp_psy(pdpm);

	if (!pdpm->cp_psy)
		return;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_BATTERY_VOLTAGE, &val);
	if (!ret)
		pdpm->cp.vbat_volt = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_BATTERY_CURRENT, &val);
	if (!ret)
		pdpm->cp.ibat_curr = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_BUS_VOLTAGE, &val);
	if (!ret)
		pdpm->cp.vbus_volt = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_BUS_CURRENT, &val);
	if (!ret)
		pdpm->cp.ibus_curr = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_BUS_TEMPERATURE, &val);
	if (!ret)
		pdpm->cp.bus_temp = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_BATTERY_TEMPERATURE, &val);
	if (!ret)
		pdpm->cp.bat_temp = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_DIE_TEMPERATURE, &val);
	if (!ret)
		pdpm->cp.die_temp = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_BATTERY_PRESENT, &val);
	if (!ret)
		pdpm->cp.batt_pres = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_VBUS_PRESENT, &val);
	if (!ret)
		pdpm->cp.vbus_pres = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_CHARGE_ENABLED, &val);
	if (!ret)
		pdpm->cp.charge_enabled = val.intval;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_ALARM_STATUS, &val);
	if (!ret) {
		pdpm->cp.bat_ovp_alarm = !!(val.intval & BAT_OVP_ALARM_MASK);
		pdpm->cp.bat_ocp_alarm = !!(val.intval & BAT_OCP_ALARM_MASK);
		pdpm->cp.bus_ovp_alarm = !!(val.intval & BUS_OVP_ALARM_MASK);
		pdpm->cp.bus_ocp_alarm = !!(val.intval & BUS_OCP_ALARM_MASK);
		pdpm->cp.bat_ucp_alarm = !!(val.intval & BAT_UCP_ALARM_MASK);
		pdpm->cp.bat_therm_alarm = !!(val.intval & BAT_THERM_ALARM_MASK);
		pdpm->cp.bus_therm_alarm = !!(val.intval & BUS_THERM_ALARM_MASK);
		pdpm->cp.die_therm_alarm = !!(val.intval & DIE_THERM_ALARM_MASK);
	}

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_FAULT_STATUS, &val);
	if (!ret) {
		pdpm->cp.bat_ovp_fault = !!(val.intval & BAT_OVP_FAULT_MASK);
		pdpm->cp.bat_ocp_fault = !!(val.intval & BAT_OCP_FAULT_MASK);
		pdpm->cp.bus_ovp_fault = !!(val.intval & BUS_OVP_FAULT_MASK);
		pdpm->cp.bus_ocp_fault = !!(val.intval & BUS_OCP_FAULT_MASK);
		pdpm->cp.bat_therm_fault = !!(val.intval & BAT_THERM_FAULT_MASK);
		pdpm->cp.bus_therm_fault = !!(val.intval & BUS_THERM_FAULT_MASK);
		pdpm->cp.die_therm_fault = !!(val.intval & DIE_THERM_FAULT_MASK);
	}

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_TI_REG_STATUS, &val);
	if (!ret) {
		pdpm->cp.vbat_reg = !!(val.intval & VBAT_REG_STATUS_MASK);
		pdpm->cp.ibat_reg = !!(val.intval & IBAT_REG_STATUS_MASK);
	}
}

static void usbpd_pm_update_cp_sec_status(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pm_config.cp_sec_enable)
		return;

	usbpd_check_cp_sec_psy(pdpm);

	if (!pdpm->cp_sec_psy)
		return;

	ret = power_supply_get_property(pdpm->cp_sec_psy,
			POWER_SUPPLY_PROP_TI_BUS_CURRENT, &val);
	if (!ret)
		pdpm->cp_sec.ibus_curr = val.intval;

	ret = power_supply_get_property(pdpm->cp_sec_psy,
			POWER_SUPPLY_PROP_CHARGE_ENABLED, &val);
	if (!ret)
		pdpm->cp_sec.charge_enabled = val.intval;
}

static int usbpd_pm_enable_cp(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_cp_psy(pdpm);

	if (!pdpm->cp_psy)
		return -ENODEV;

	val.intval = enable;
	ret = power_supply_set_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);

	return ret;
}

static int usbpd_pm_enable_cp_sec(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_cp_sec_psy(pdpm);

	if (!pdpm->cp_sec_psy)
		return -ENODEV;

	val.intval = enable;
	ret = power_supply_set_property(pdpm->cp_sec_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);

	return ret;
}

static int usbpd_pm_check_cp_enabled(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_cp_psy(pdpm);

	if (!pdpm->cp_psy)
		return -ENODEV;

	ret = power_supply_get_property(pdpm->cp_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pdpm->cp.charge_enabled = !!val.intval;

	return ret;
}

static int usbpd_pm_check_cp_sec_enabled(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	usbpd_check_cp_sec_psy(pdpm);

	if (!pdpm->cp_sec_psy)
		return -ENODEV;

	ret = power_supply_get_property(pdpm->cp_sec_psy,
			POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
	if (!ret)
		pdpm->cp_sec.charge_enabled = !!val.intval;

	return ret;
}
#ifdef SUPPORT_ONSEMI_PDCONTROL
static int lenovo_sec_usbpd_enable_sw(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pdpm->bq_2589x_chg_psy) {
		pdpm->bq_2589x_chg_psy = power_supply_get_by_name("bq2589h-charger");
		if (!pdpm->bq_2589x_chg_psy) {
			return -ENODEV;
		}
	}

	val.intval = enable;
	pdpm->sw.system_bq_chg_disable= !enable;
	if (val.intval){
		ret = power_supply_set_property(pdpm->bq_2589x_chg_psy,
				POWER_SUPPLY_PROP_CHARGING_ENABLED, &val);
		if (!ret){
			pr_info("resume BQ2589H_charger successfully\n");
		}
		val.intval =2000;
		ret = power_supply_set_property(pdpm->bq_2589x_chg_psy,
				POWER_SUPPLY_PROP_FC_LIMIT_CURRENT, &val);
		if (!ret){
			pr_info("set BQ2589H_charger max 2000mA\n");
		}
	}
	else{
		val.intval =500;
		ret = power_supply_set_property(pdpm->bq_2589x_chg_psy,
				POWER_SUPPLY_PROP_FC_LIMIT_CURRENT, &val);
		if (!ret){
			pr_info("set BQ2589H_charger max 500mA\n");
		}
	}

	return ret;
}
#else
static int usbpd_pm_enable_sw(struct usbpd_pm *pdpm, bool enable)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pdpm->sw_psy) {
		pdpm->sw_psy = power_supply_get_by_name("battery");
		if (!pdpm->sw_psy) {
			return -ENODEV;
		}
	}

	val.intval = enable;
	ret = power_supply_set_property(pdpm->sw_psy,
			POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, &val);

	return ret;
}
#endif
static int usbpd_pm_check_sw_enabled(struct usbpd_pm *pdpm)
{
	int ret;
	union power_supply_propval val = {0,};

	if (!pdpm->sw_psy) {
		pdpm->sw_psy = power_supply_get_by_name("battery");
		if (!pdpm->sw_psy) {
			return -ENODEV;
		}
	}

	ret = power_supply_get_property(pdpm->sw_psy,
			POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED, &val);
	if (!ret)
		pdpm->sw.charge_enabled = !!val.intval;

	return ret;
}

static void usbpd_pm_update_sw_status(struct usbpd_pm *pdpm)
{
	usbpd_pm_check_sw_enabled(pdpm);
}

#ifdef CONFIG_PRODUCT_MOBA
static int charger2a_5v_obj_current = 0;
static int charger2a_9v_obj_current = 0;
static int charger2a_pd_max_power = 0;
extern void reset_charger2_pd_power_data(void);

int get_charger2a_max_power(void) {
	return charger2a_pd_max_power;
}

int get_charger2a_5v_obj_current(void) {
	return charger2a_5v_obj_current;
}

int get_charger2a_9v_obj_current(void) {
	return charger2a_9v_obj_current;
}

void reset_charger2a_pd_power_data(void) {
	charger2a_pd_max_power = 0;
	charger2a_5v_obj_current = 0;
	charger2a_9v_obj_current = 0;
}
#endif

static void usbpd_pm_evaluate_src_caps(struct usbpd_pm *pdpm)
{
#ifndef SUPPORT_ONSEMI_PDCONTROL
	int ret;
	int i;
#ifdef CONFIG_PRODUCT_MOBA
	int power, max_power;
#endif
	if (!pdpm->pd) {
		pdpm->pd = smb_get_usbpd();
		if (!pdpm->pd) {
			pr_err("couldn't get usbpd device\n");
			return;
		}
	}

	for(i = 0; i < 7; i++)
		pdpm->pdo[i].pps = false;

	ret = usbpd_fetch_pdo(pdpm->pd, pdpm->pdo);
	if (ret) {
		pr_err("Failed to fetch pdo info\n");
		return;
	}
#endif

#ifndef SUPPORT_ONSEMI_PDCONTROL
	pdpm->apdo_max_volt = pm_config.min_adapter_volt_allowed;
	pdpm->apdo_max_curr = pm_config.min_adapter_curr_allowed;

	for (i = 0; i < 7; i++) {
		if (pdpm->pdo[i].type == PD_SRC_PDO_TYPE_AUGMENTED
			&& pdpm->pdo[i].pps && pdpm->pdo[i].pos) {
			if (pdpm->pdo[i].max_volt_mv >= pdpm->apdo_max_volt
					&& pdpm->pdo[i].curr_ma > pdpm->apdo_max_curr) {
				pdpm->apdo_max_volt = pdpm->pdo[i].max_volt_mv;
				pdpm->apdo_max_curr = pdpm->pdo[i].curr_ma;
				pdpm->apdo_selected_pdo = pdpm->pdo[i].pos;
				pdpm->pps_supported = true;
			}
		}
#ifdef CONFIG_PRODUCT_MOBA
		power = pdpm->pdo[i].max_volt_mv * pdpm->pdo[i].curr_ma / 1000000;
		pr_info("sec-usbpd-list apdo fixed %d : volt %d, curr %d, power %d\n",
				i, pdpm->pdo[i].max_volt_mv, pdpm->pdo[i].curr_ma, power);
		if (power > max_power)
			max_power = power;
		if ((pdpm->pdo[i].max_volt_mv >= 5000) && (pdpm->pdo[i].max_volt_mv <= 5300))
			charger1_5v_obj_current = pdpm->pdo[i].curr_ma;
		else if ((pdpm->pdo[i].max_volt_mv >= 9000) && (pdpm->pdo[i].max_volt_mv <= 9300))
			charger1_9v_obj_current = pdpm->pdo[i].curr_ma;
#endif
	}
#endif

#ifdef SUPPORT_ONSEMI_PDCONTROL
	if (fusb_pd_enable){
		pdpm->apdo_selected_pdo =fusb_fetch_pdo_chip->requested_pdo;
		pdpm->apdo_max_volt = fusb_fetch_pdo_chip->req_voltage*20;
		pdpm->apdo_max_curr = fusb_fetch_pdo_chip->req_current*50;
		pdpm->pps_supported = true;
		pr_info("ti src cap apdo_obj = %d,apdo_max_vol = %d,apdo_max_curr = %d\n",
			pdpm->apdo_selected_pdo, pdpm->apdo_max_volt, pdpm->apdo_max_curr);
#ifdef CONFIG_PRODUCT_MOBA
		charger2a_pd_max_power = pdpm->apdo_max_volt * pdpm->apdo_max_curr / 1000000;
		pr_info("sec-usbpd-list apdo max fixed power %d, 5v obj %dmA, 9v obj %dmA\n",
				charger2a_pd_max_power, charger2a_5v_obj_current, charger2a_9v_obj_current);
#endif
	}
#endif


	if (pdpm->pps_supported){
		realpd2_request_curr = pdpm->apdo_max_curr;
		pr_notice("PPS supported, preferred APDO pos:%d, max volt:%d, current:%d\n",
				pdpm->apdo_selected_pdo,
				pdpm->apdo_max_volt,
				pdpm->apdo_max_curr);
	}
	else
		pr_notice("Not qualified PPS adapter\n");
}

#ifndef SUPPORT_ONSEMI_PDCONTROL
static void usbpd_update_pps_status(struct usbpd_pm *pdpm)
{
	int ret;
	u32 status;

	ret = usbpd_get_pps_status(pdpm->pd, &status);

	if (!ret) {
		/*TODO: check byte order to insure data integrity*/
		pdpm->adapter_voltage = ((status >> 16) & 0xFFFF )* 20;
		pdpm->adapter_current = ((status >> 8) & 0xFF ) * 50;
		pdpm->adapter_ptf = (status & 0x06) >> 1;
		pdpm->adapter_omf = !!(status & 0x08);
		pr_debug("adapter_volt:%d, adapter_current:%d\n",
				pdpm->adapter_voltage, pdpm->adapter_current);
	}
}
#endif

#define TAPER_TIMEOUT	(5000 / PM_WORK_RUN_INTERVAL)
#define IBUS_CHANGE_TIMEOUT  (500 / PM_WORK_RUN_INTERVAL)
static int usbpd_pm_fc2_charge_algo(struct usbpd_pm *pdpm)
{
	int steps;
	int sw_ctrl_steps = 0;
	int hw_ctrl_steps = 0;
	int step_vbat = 0;
	int step_ibus = 0;
	int step_ibat = 0;
	int step_bat_reg = 0;
	int ibus_total = 0;
#ifdef CONFIG_PRODUCT_MOBA
	int BUS_CP_MAX_CURRENT;
	int BATT_CP_MAX_CURRENT;
#endif
	static int ibus_limit;

#ifdef CONFIG_PRODUCT_MOBA
	BUS_CP_MAX_CURRENT = pd2_max_current/2;
	BATT_CP_MAX_CURRENT = pd2_max_current+1000;
#endif

	if (ibus_limit == 0)
		ibus_limit = pm_config.bus_curr_lp_lmt;// + 400;
#ifdef CONFIG_PRODUCT_MOBA
		if (BUS_CP_MAX_CURRENT < ibus_limit)
		ibus_limit = BUS_CP_MAX_CURRENT;
#endif
	/* reduce bus current in cv loop */
	if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 50) {
		if (ibus_lmt_change_timer++ > IBUS_CHANGE_TIMEOUT) {
			ibus_lmt_change_timer = 0;
			ibus_limit = pm_config.bus_curr_lp_lmt - 400;
#ifdef CONFIG_PRODUCT_MOBA
			if (ibus_limit > (BUS_CP_MAX_CURRENT -400))
				ibus_limit = (BUS_CP_MAX_CURRENT -400);
#endif
		}
	} else if (pdpm->cp.vbat_volt < pm_config.bat_volt_lp_lmt - 250) {
		ibus_limit = pm_config.bus_curr_lp_lmt;// + 400;
#ifdef CONFIG_PRODUCT_MOBA
		if (BUS_CP_MAX_CURRENT < ibus_limit)
			ibus_limit = BUS_CP_MAX_CURRENT;
#endif
		ibus_lmt_change_timer = 0;
	} else {
		ibus_lmt_change_timer = 0;
	}

	/* battery voltage loop*/
	if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt)
		step_vbat = -pm_config.fc2_steps;
	else if (pdpm->cp.vbat_volt < pm_config.bat_volt_lp_lmt - 7)
		step_vbat = pm_config.fc2_steps;;


	/* battery charge current loop*/
#ifdef CONFIG_PRODUCT_MOBA
	if (BATT_CP_MAX_CURRENT < pm_config.bat_curr_lp_lmt){
		if (pdpm->cp.ibat_curr < BATT_CP_MAX_CURRENT)
			step_ibat = pm_config.fc2_steps;
		else if (pdpm->cp.ibat_curr > BATT_CP_MAX_CURRENT + 100)
			step_ibat = -pm_config.fc2_steps;
	}else{
		if (pdpm->cp.ibat_curr < pm_config.bat_curr_lp_lmt )
			step_ibat = pm_config.fc2_steps;
		else if (pdpm->cp.ibat_curr > pm_config.bat_curr_lp_lmt + 100)
			step_ibat = -pm_config.fc2_steps;
	}
#else
		if (pdpm->cp.ibat_curr < pm_config.bat_curr_lp_lmt )
			step_ibat = pm_config.fc2_steps;
		else if (pdpm->cp.ibat_curr > pm_config.bat_curr_lp_lmt + 100)
			step_ibat = -pm_config.fc2_steps;
#endif

	/* bus current loop*/
	ibus_total = pdpm->cp.ibus_curr;

	if (pm_config.cp_sec_enable)
		ibus_total += pdpm->cp_sec.ibus_curr;

	if (ibus_total < ibus_limit - 50)
		step_ibus = pm_config.fc2_steps;
	else if (ibus_total > ibus_limit)
		step_ibus = -pm_config.fc2_steps * 3;

	pr_info("ibus_total = %d,main:salve=%d:%d\n", ibus_total, pdpm->cp.ibus_curr, pdpm->cp_sec.ibus_curr);

	/* hardware regulation loop*/
	if (pdpm->cp.vbat_reg || pdpm->cp.ibat_reg)
		step_bat_reg = 5 * (-pm_config.fc2_steps);
	else
		step_bat_reg = pm_config.fc2_steps;

	sw_ctrl_steps = min(min(step_vbat, step_ibus), step_ibat);
	sw_ctrl_steps = min(sw_ctrl_steps, step_bat_reg);

	/* hardware alarm loop */
	if (pdpm->cp.bat_ocp_alarm /*|| pdpm->cp.bat_ovp_alarm */
		|| pdpm->cp.bus_ocp_alarm || pdpm->cp.bus_ovp_alarm
		/*|| pdpm->cp.tbat_temp > 60
		  || pdpm->cp.tbus_temp > 50*/)
		hw_ctrl_steps = -pm_config.fc2_steps;
	else
		hw_ctrl_steps = pm_config.fc2_steps;

	/* check if cp disabled due to other reason*/
	usbpd_pm_check_cp_enabled(pdpm);

	if (pm_config.cp_sec_enable)
		usbpd_pm_check_cp_sec_enabled(pdpm);

	if (pdpm->cp.bat_therm_fault ) { /* battery overheat, stop charge*/
		pr_notice("bat_therm_fault:%d\n", pdpm->cp.bat_therm_fault);
		return PM_ALGO_RET_THERM_FAULT;
	} else if (pdpm->cp.bat_ocp_fault || pdpm->cp.bus_ocp_fault
			|| pdpm->cp.bat_ovp_fault || pdpm->cp.bus_ovp_fault) {
			pr_notice("bat_ocp_fault:%d, bus_ocp_fault:%d, bat_ovp_fault:%d, \
					bus_ovp_fault:%d\n", pdpm->cp.bat_ocp_fault,
				pdpm->cp.bus_ocp_fault, pdpm->cp.bat_ovp_fault,
				pdpm->cp.bus_ovp_fault);
	        return PM_ALGO_RET_OTHER_FAULT; /* go to switch, and try to ramp up*/
	} else if (!pdpm->cp.charge_enabled
			|| (pm_config.cp_sec_enable && !pdpm->cp_sec.charge_enabled && !pdpm->cp_sec_stopped)) {
		pr_notice("cp.charge_enabled:%d, cp_sec.charge_enabled:%d\n",
				pdpm->cp.charge_enabled, pdpm->cp_sec.charge_enabled);
		return PM_ALGO_RET_CHG_DISABLED;
	}

	/* charge pump taper charge */
	if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 50
			&& ibus_total < pm_config.fc2_taper_current/2) {
	//		&& pdpm->cp.ibat_curr < pm_config.fc2_taper_current) {
		if (fc2_taper_timer++ > TAPER_TIMEOUT) {
			pr_info("charge pump taper charging done\n");
			fc2_taper_timer = 0;
			return PM_ALGO_RET_TAPER_DONE;
		}
	} else {
		fc2_taper_timer = 0;
	}
	/*TODO: customer can add hook here to check system level
	 * thermal mitigation*/

	steps = min(sw_ctrl_steps, hw_ctrl_steps);

	pdpm->request_voltage += steps * 20;

	if (pdpm->request_voltage > pdpm->apdo_max_volt - 1000)
		pdpm->request_voltage = pdpm->apdo_max_volt - 1000;

	return PM_ALGO_RET_OK;
}

static const unsigned char *pm_str[] = {
	"PD_PM_STATE_ENTRY",
	"PD_PM_STATE_FC2_ENTRY",
	"PD_PM_STATE_FC2_ENTRY_1",
	"PD_PM_STATE_FC2_ENTRY_2",
	"PD_PM_STATE_FC2_ENTRY_3",
	"PD_PM_STATE_FC2_TUNE",
	"PD_PM_STATE_FC2_EXIT",
};

static void usbpd_pm_move_state(struct usbpd_pm *pdpm, enum pm_state state)
{
#if 1
	pr_debug("state change:%s -> %s\n",
		pm_str[pdpm->state], pm_str[state]);
#endif
	pdpm->state = state;
}

static int sec_port_usbpd_pm_sm(struct usbpd_pm *pdpm)
{
	int ret;
	int rc = 0;
	static int tune_vbus_retry;
	static bool stop_sw;
	static bool recover;
#ifdef SUPPORT_ONSEMI_PDCONTROL
	int flash_chg_max_curr;
	FSC_U16 report_vol = 0;
	FSC_U8 report_curr=0;
	FSC_U8 obj = 0;
	pdpm->port = fusb_fetch_pdo_chip->port;
	flash_chg_max_curr = pd2_max_current/2;
#endif
	if (is_batt2_stop || is_batt2_thermal_stop){
		if (is_batt2_full){
			recover = false;
			pr_info("stop port2 charge pump for batt full\n");
		}
		else{
			recover = true;
			pr_info("stop port2 charge pump for thermal\n");
		}
		usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
	}

	switch (pdpm->state) {
	case PD_PM_STATE_ENTRY:
		stop_sw = false;
		recover = false;

		if (pdpm->cp.vbat_volt < pm_config.min_vbat_for_cp) {
			pr_info("batt_volt-%d, waiting...\n", pdpm->cp.vbat_volt);
		} else if (pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 100) {
			pr_info("batt_volt-%d is too high for cp,\
					charging with switch charger\n",
					pdpm->cp.vbat_volt);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
		} else {
			pr_info("batt_volt-%d is ok, start flash charging\n",
					pdpm->cp.vbat_volt);
#ifdef CONFIG_PRODUCT_MOBA
			is_pd2_done = 1;
#endif
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY);
		}
		break;
	case PD_PM_STATE_FC2_ENTRY:
#ifndef CONFIG_PRODUCT_MOBA
		if (pm_config.fc2_disable_sw) {
			if (pdpm->sw.charge_enabled) {
				usbpd_pm_enable_sw(pdpm,false);
				usbpd_pm_check_sw_enabled(pdpm);
			}
			if (!pdpm->sw.charge_enabled)
				usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_1);
		} else {
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_1);
		}
#endif
#ifdef CONFIG_PRODUCT_MOBA
		if (!pdpm->sw.system_bq_chg_disable){
			lenovo_sec_usbpd_enable_sw(pdpm,false);
		}
		usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_1);
#endif
		break;

	case PD_PM_STATE_FC2_ENTRY_1:
#if 0
		if (pm_config.cp_sec_enable)
				pdpm->request_voltage = pdpm->cp.vbat_volt * 2 + BUS_VOLT_INIT_UP * 2;
		else
				pdpm->request_voltage = pdpm->cp.vbat_volt * 2 + BUS_VOLT_INIT_UP;
#endif
		pdpm->request_voltage = pdpm->cp.vbat_volt * 2 + BUS_VOLT_INIT_UP;

		pdpm->request_current = min(pdpm->apdo_max_curr, pm_config.bus_curr_lp_lmt);

#ifndef SUPPORT_ONSEMI_PDCONTROL
		usbpd_select_pdo(pdpm->pd, pdpm->apdo_selected_pdo,
			pdpm->request_voltage * 1000,
			pdpm->request_current * 1000);
		pr_debug("request_voltage:%d, request_current:%d\n",
				pdpm->request_voltage, pdpm->request_current);
#endif
#ifdef SUPPORT_ONSEMI_PDCONTROL
		pdpm->request_current = flash_chg_max_curr;
		report_vol = (FSC_U16) (pdpm->request_voltage/20);
		report_curr = (FSC_U8)(pdpm->request_current/50);
		obj = (FSC_U8)pdpm->apdo_selected_pdo;

		DPM_SendPPSRequest(pdpm->port, report_vol, report_curr, obj);
		core_state_machine(pdpm->port);

		pr_info("FC2_ENTRY_1:request_voltage =%d,request_current =%d,apdo_selected_pdo=%d\n",
			pdpm->request_voltage,pdpm->request_current,pdpm->apdo_selected_pdo);
#endif

		usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_2);

		tune_vbus_retry = 0;
		break;

	case PD_PM_STATE_FC2_ENTRY_2:
		pr_info("vbus_volt:%d  vbat_volt:%d\n", pdpm->cp.vbus_volt, pdpm->cp.vbat_volt);
		if (pdpm->cp.vbus_volt < (pdpm->cp.vbat_volt * 2 + BUS_VOLT_INIT_UP - 50)) {
			tune_vbus_retry++;
			pdpm->request_voltage += 20;
#ifndef SUPPORT_ONSEMI_PDCONTROL
			usbpd_select_pdo(pdpm->pd, pdpm->apdo_selected_pdo,
						pdpm->request_voltage * 1000,
						pdpm->request_current * 1000);
			pr_info("request_voltage:%d, request_current:%d\n",
					pdpm->request_voltage, pdpm->request_current);
#endif
#ifdef SUPPORT_ONSEMI_PDCONTROL
		pdpm->request_current = flash_chg_max_curr;
		report_vol = (FSC_U16)(pdpm->request_voltage/20);
		report_curr = (FSC_U8)(pdpm->request_current/50);
		obj = (FSC_U8)pdpm->apdo_selected_pdo;
		DPM_SendPPSRequest(pdpm->port, report_vol, report_curr, obj);
		core_state_machine(pdpm->port);

		pr_info("FC2_ENTRY_2_1:request_voltage =%d,request_current =%d,apdo_selected_pdo=%d\n",
			pdpm->request_voltage,pdpm->request_current,pdpm->apdo_selected_pdo);
#endif

		} else if (pdpm->cp.vbus_volt > (pdpm->cp.vbat_volt * 2 + BUS_VOLT_INIT_UP + 50)) {
			tune_vbus_retry++;
			pdpm->request_voltage -= 20;
#ifndef SUPPORT_ONSEMI_PDCONTROL
			usbpd_select_pdo(pdpm->pd, pdpm->apdo_selected_pdo,
						pdpm->request_voltage * 1000,
						pdpm->request_current * 1000);
			pr_info("request_voltage:%d, request_current:%d\n",
					pdpm->request_voltage, pdpm->request_current);
#endif
#ifdef SUPPORT_ONSEMI_PDCONTROL
		pdpm->request_current = flash_chg_max_curr;
		report_vol = (FSC_U16)(pdpm->request_voltage/20);
		report_curr = (FSC_U8)(pdpm->request_current/50);
		obj = (FSC_U8)pdpm->apdo_selected_pdo;

		DPM_SendPPSRequest(pdpm->port, report_vol, report_curr, obj);
		core_state_machine(pdpm->port);

		pr_info("FC2_ENTRY_2_2:request_voltage =%d,request_current =%d,apdo_selected_pdo=%d\n",
			pdpm->request_voltage,pdpm->request_current,pdpm->apdo_selected_pdo);
#endif

		} else {
			pr_info("adapter volt tune ok, retry %d times\n", tune_vbus_retry);
		        usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_ENTRY_3);
			break;
		}

		if (tune_vbus_retry > 30) {
			pr_err("Failed to tune adapter volt into valid range, charge with switching charger\n");
#ifdef CONFIG_PRODUCT_MOBA
			recover = true;
#endif
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
		}
		break;
	case PD_PM_STATE_FC2_ENTRY_3:
		if (!pdpm->cp.charge_enabled) {
			usbpd_pm_enable_cp(pdpm,true);
			msleep(30);
			usbpd_pm_check_cp_enabled(pdpm);
		}

		if (pm_config.cp_sec_enable && !pdpm->cp_sec.charge_enabled) {
			usbpd_pm_enable_cp_sec(pdpm,true);
			msleep(30);
			usbpd_pm_check_cp_sec_enabled(pdpm);
		}

		if (pdpm->cp.charge_enabled) {
			if ((pm_config.cp_sec_enable && pdpm->cp_sec.charge_enabled)
					|| !pm_config.cp_sec_enable) {
				usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_TUNE);
				ibus_lmt_change_timer = 0;
				fc2_taper_timer = 0;
			}
		}
		break;

	case PD_PM_STATE_FC2_TUNE:
#if 0
		if (pdpm->cp.vbat_volt < pm_config.min_vbat_for_cp - 400){
			usbpd_pm_move_state(PD_PM_STATE_SW_ENTRY);
			break;
		}
#endif
#ifndef SUPPORT_ONSEMI_PDCONTROL
		usbpd_update_pps_status(pdpm);
#endif
		ret = usbpd_pm_fc2_charge_algo(pdpm);
		if (ret == PM_ALGO_RET_THERM_FAULT) {
			pr_info("Move to stop charging:%d\n", ret);
			stop_sw = true;
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		} else if (ret == PM_ALGO_RET_OTHER_FAULT || ret == PM_ALGO_RET_TAPER_DONE) {
			pr_info("Move to switch charging:%d\n", ret);
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		} else if (ret == PM_ALGO_RET_CHG_DISABLED) {
			pr_info("Move to switch charging, will try to recover \
					flash charging:%d\n", ret);
			recover = true;
			usbpd_pm_move_state(pdpm, PD_PM_STATE_FC2_EXIT);
			break;
		} else {
#ifndef SUPPORT_ONSEMI_PDCONTROL
			usbpd_select_pdo(pdpm->pd, pdpm->apdo_selected_pdo,
						pdpm->request_voltage * 1000,
						pdpm->request_current * 1000);
			pr_debug("request_voltage:%d, request_current:%d\n",
					pdpm->request_voltage, pdpm->request_current);
#endif
#ifdef SUPPORT_ONSEMI_PDCONTROL
		pdpm->request_current = flash_chg_max_curr;
		report_vol = (FSC_U16)(pdpm->request_voltage/20);
		report_curr = (FSC_U8)(pdpm->request_current/50);
		obj = (FSC_U8)pdpm->apdo_selected_pdo;

		DPM_SendPPSRequest(pdpm->port, report_vol, report_curr, obj);
		core_state_machine(pdpm->port);

		pr_info("FC2_TUNE:request_voltage =%d,request_current =%d,apdo_selected_pdo=%d\n",
			pdpm->request_voltage,pdpm->request_current,pdpm->apdo_selected_pdo);
#endif
		}
		/*stop second charge pump if either of ibus is lower than 750ma during CV*/
		if (pm_config.cp_sec_enable && pdpm->cp_sec.charge_enabled
				&& pdpm->cp.vbat_volt > pm_config.bat_volt_lp_lmt - 50
				&& (pdpm->cp.ibus_curr < 750 || pdpm->cp_sec.ibus_curr < 750)) {
			pr_info("second cp is disabled due to ibus < 750mA\n");
			usbpd_pm_enable_cp_sec(pdpm,false);
			usbpd_pm_check_cp_sec_enabled(pdpm);
			pdpm->cp_sec_stopped = true;
		}
		break;

	case PD_PM_STATE_FC2_EXIT:
		/* select default 5V*/
#ifdef SUPPORT_ONSEMI_PDCONTROL
		//usbpd_select_pdo(pdpm->pd, 1, 0, 0);
#endif
		if (pdpm->cp.charge_enabled) {
			usbpd_pm_enable_cp(pdpm,false);
			usbpd_pm_check_cp_enabled(pdpm);
		}

		if (pm_config.cp_sec_enable && pdpm->cp_sec.charge_enabled) {
			usbpd_pm_enable_cp_sec(pdpm,false);
			usbpd_pm_check_cp_sec_enabled(pdpm);
		}
#ifndef SUPPORT_ONSEMI_PDCONTROL
		if (stop_sw && pdpm->sw.charge_enabled)
			usbpd_pm_enable_sw(pdpm,false);
		else if (!stop_sw && !pdpm->sw.charge_enabled)
			usbpd_pm_enable_sw(pdpm,true);

		usbpd_pm_check_sw_enabled(pdpm);
#endif
#ifdef CONFIG_PRODUCT_MOBA
		if (pdpm->sw.system_bq_chg_disable){
			lenovo_sec_usbpd_enable_sw(pdpm,true);
		}
#endif
		if (recover)
			usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
		else{
			rc = 1;
#ifdef CONFIG_PRODUCT_MOBA
			is_pd2_done = 2;
#endif
			pr_info("exit sec_port_usbpd_pm_sm\n");
			}

		break;
	}

	return rc;
}

static void sec_port_usbpd_pm_workfunc(struct work_struct *work)
{
	struct usbpd_pm *pdpm = container_of(work, struct usbpd_pm,
				sec_pm_work.work);

#ifdef SUPPORT_ONSEMI_PDCONTROL
	if (fusb_fetch_pdo_chip == NULL)
		return;
#endif

	usbpd_pm_update_sw_status(pdpm);

	usbpd_pm_update_cp_status(pdpm);
	usbpd_pm_update_cp_sec_status(pdpm);

#ifdef SUPPORT_ONSEMI_PDCONTROL
	if ((!sec_port_usbpd_pm_sm(pdpm)) &&fusb_fetch_pdo_chip->fusb302_pd_active)
#else
	if (!sec_port_usbpd_pm_sm(pdpm) && pdpm->pd_active)
#endif
		schedule_delayed_work(&pdpm->sec_pm_work,
				msecs_to_jiffies(PM_WORK_RUN_INTERVAL));

}

static void usbpd_pm_disconnect(struct usbpd_pm *pdpm)
{

	//cancel_work(&pdpm->pm_work);
	cancel_delayed_work(&pdpm->sec_pm_work);
	pdpm->pps_supported = false;
	pdpm->apdo_selected_pdo = 0;
	pdpm->cp_sec_stopped = false;

	usbpd_pm_move_state(pdpm, PD_PM_STATE_ENTRY);
}

static void usbpd_pd_contact(struct usbpd_pm *pdpm, bool connected)
{
	pdpm->pd_active = connected;

	if (connected) {
		usbpd_pm_evaluate_src_caps(pdpm);
		if (pdpm->pps_supported)
			schedule_delayed_work(&pdpm->sec_pm_work, 0);
	} else {
		usbpd_pm_disconnect(pdpm);
	}
}

static void cp_psy_change_work(struct work_struct *work)
{
	struct usbpd_pm *pdpm = container_of(work, struct usbpd_pm,
					cp_psy_change_work);
#if 0
	union power_supply_propval val = {0,};
	bool ac_pres = pdpm->cp.vbus_pres;
	int ret;


	if (!pdpm->cp_psy)
		return;

	ret = power_supply_get_property(pdpm->cp_psy, POWER_SUPPLY_PROP_TI_VBUS_PRESENT, &val);
	if (!ret)
		pdpm->cp.vbus_pres = val.intval;

	if (!ac_pres && pdpm->cp.vbus_pres)
		schedule_delayed_work(&pdpm->sec_pm_work, 0);
#endif
	pdpm->psy_change_running = false;
}

#ifdef SUPPORT_ONSEMI_PDCONTROL
void usb_psy_change_work(struct work_struct *work)
{
	struct usbpd_pm *pdpm = container_of(work, struct usbpd_pm,
		usb_psy_change_work);

	if ((fusb_fetch_pdo_chip != NULL)  && (fusb_fetch_pdo_chip->fusb302_pd_active)){
		usbpd_pd_contact(pdpm,true);
	}
	else{ //if (pdpm->pd_active && !val.intval)
		usbpd_pd_contact(pdpm,false);
	}
}
#else
static void usb_psy_change_work(struct work_struct *work)
{
	struct usbpd_pm *pdpm = container_of(work, struct usbpd_pm,
					usb_psy_change_work);
	union power_supply_propval val = {0,};
	int ret;

	ret = power_supply_get_property(pdpm->usb_psy,
			POWER_SUPPLY_PROP_TYPEC_POWER_ROLE,&val);
	if (ret) {
		pr_err("Failed to read typec power role\n");
		goto out;
	}

	if (val.intval != POWER_SUPPLY_TYPEC_PR_SINK &&
			val.intval != POWER_SUPPLY_TYPEC_PR_DUAL)
		goto out;

	ret = power_supply_get_property(pdpm->usb_psy,
			POWER_SUPPLY_PROP_PD_ACTIVE, &val);
	if (ret) {
		pr_err("Failed to get usb pd active state\n");
		goto out;
	}

	if (!pdpm->pd_active && val.intval)
		usbpd_pd_contact(pdpm,true);
	else if (pdpm->pd_active && !val.intval)
		usbpd_pd_contact(pdpm,false);
out:
	pdpm->psy_change_running = false;
}
#endif

#ifdef SUPPORT_ONSEMI_PDCONTROL
static int usbpd_psy_notifier_cb(struct notifier_block *nb,
			unsigned long event, void *data)
{
	return NOTIFY_OK;
}
#else
static int usbpd_psy_notifier_cb(struct notifier_block *nb,
			unsigned long event, void *data)
{
	struct usbpd_pm *pdpm = container_of(nb, struct usbpd_pm, nb);
	struct power_supply *psy = data;
	unsigned long flags;

	if (event != PSY_EVENT_PROP_CHANGED)
		return NOTIFY_OK;

	usbpd_check_cp_psy(pdpm);
	usbpd_check_usb_psy(pdpm);

	if (!pdpm->cp_psy || !pdpm->usb_psy)
		return NOTIFY_OK;

	if (psy == pdpm->cp_psy || psy == pdpm->usb_psy) {
		spin_lock_irqsave(&pdpm->psy_change_lock, flags);
		if (!pdpm->psy_change_running) {
			pdpm->psy_change_running = true;
			if (psy == pdpm->cp_psy)
				schedule_work(&pdpm->cp_psy_change_work);
			else
				schedule_work(&pdpm->usb_psy_change_work);
		}
		spin_unlock_irqrestore(&pdpm->psy_change_lock, flags);
	}

	return NOTIFY_OK;
}
#endif

static int __init usbpd_pm_init(void)
{
	struct usbpd_pm *pdpm;

	pdpm = kzalloc(sizeof(struct usbpd_pm), GFP_KERNEL);
	if (!pdpm)
		return -ENOMEM;

	__pdpm = pdpm;
#ifdef SUPPORT_ONSEMI_PDCONTROL
	bq_usbpd_pm = pdpm;
#endif


	INIT_WORK(&pdpm->cp_psy_change_work, cp_psy_change_work);
	INIT_WORK(&pdpm->usb_psy_change_work, usb_psy_change_work);

	spin_lock_init(&pdpm->psy_change_lock);

	usbpd_check_cp_psy(pdpm);
	usbpd_check_cp_sec_psy(pdpm);
	usbpd_check_usb_psy(pdpm);

	INIT_DELAYED_WORK(&pdpm->sec_pm_work, sec_port_usbpd_pm_workfunc);

	pdpm->nb.notifier_call = usbpd_psy_notifier_cb;
	power_supply_reg_notifier(&pdpm->nb);
	return 0;
}

static void __exit usbpd_pm_exit(void)
{
	power_supply_unreg_notifier(&__pdpm->nb);
	cancel_delayed_work(&__pdpm->sec_pm_work);
	cancel_work(&__pdpm->cp_psy_change_work);
	cancel_work(&__pdpm->usb_psy_change_work);

}

module_init(usbpd_pm_init);
module_exit(usbpd_pm_exit);
