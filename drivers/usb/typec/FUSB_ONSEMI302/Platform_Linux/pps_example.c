#include "pps_example.h"
#include "../core/core.h"
#include "../core/platform.h"

#ifdef CONFIG_PRODUCT_MOBA
#define SUPPORT_ONSEMI_PDCONTROL
extern void reset_charger2_pd_power_data(void);
extern void reset_charger2a_pd_power_data(void);
#endif

#ifdef SUPPORT_ONSEMI_PDCONTROL
#include "../core/PDPolicy.h"
#include "fusb30x_global.h"
#include "../../../../power/supply/ti/port_2/pd_policy_manager.h"
#include "../../../pd/usbpd.h"
#endif

#ifdef SUPPORT_ONSEMI_PDCONTROL
#define MAX_PPS_V 550   /* 20mV Units */
#define MAX_PPS_I 80    /* 50mA Units */
#else
#define MAX_PPS_V 250   /* 20mV Units */
#define MAX_PPS_I 60    /* 50mA Units */
#endif

#ifdef SUPPORT_ONSEMI_PDCONTROL
extern struct fusb30x_chip* fusb_pps_chip;
extern struct charger_object* fusb_fetch_pdo_chip;
PPSStatus_t *fusb_pps_st = NULL;
bool fusb_call_time = 1;
bool fusb_pd_enable;

struct charger_object* qq_chip = NULL;
struct charger_object* fusb30x_charger_GetChip(void)
{
	return qq_chip;	// return a pointer to our structs
}
void fusb30x_charger_SetChip(struct charger_object* ccn_Chip)
{
	qq_chip = ccn_Chip;	// assign the pointer to our struct
}
#endif

void init_sink_pps_example(struct Port *port, struct charger_object *charger)
{
	/* Init charger object */
	charger->port = port;
	charger->requested_pdo = 0;
	charger->direct_charge_active = 0;

	/* Register observers */
	register_observer(EVENT_SRC_CAPS_UPDATED, src_caps_updated_handler, charger);
	register_observer(SRC_CAPS_EXT_RECEIVED, src_caps_ext_received_handler, charger);
	register_observer(PD_NEW_CONTRACT, new_contract_handler, charger);
	register_observer(ALERT_EVENT, alert_received_handler, charger);
	register_observer(STATUS_RECEIVED, status_received_handler, charger);
	register_observer(PPS_STATUS_RECEIVED, pps_status_received_handler, charger);
	register_observer(EVENT_PD_CONTRACT_FAILED, pd_failed_handler, charger);
	register_observer(EVENT_HARD_RESET, hard_reset_handler, charger);
	register_observer(CC_NO_ORIENT, typec_detach_handler, charger);
}

void src_caps_updated_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;
	struct Port *port = charger->port;
	unsigned int num_caps;
	unsigned int obj_pos;
	unsigned int i;
	unsigned int obj_power = 0;
	unsigned int max_power = 0;

    if(port->PolicyIsSource == TRUE) return;

    obj_pos = 0;
	num_caps = port->SrcCapsHeaderReceived.NumDataObjects;

	/* Select APDO - choose highest APDO */
	for(i = 0; i < num_caps; i++) {
		if(port->SrcCapsReceived[i].PDO.SupplyType == pdoTypeAugmented
				&& port->SrcCapsReceived[i].APDO.APDOType == apdoTypePPS) {
				charger->req_voltage =
				port->SrcCapsReceived[i].PPSAPDO.MaxVoltage * 5;
				charger->req_current =
				port->SrcCapsReceived[i].PPSAPDO.MaxCurrent;

			if(charger->req_voltage > MAX_PPS_V) {
				charger->req_voltage = MAX_PPS_V;
			}

			if(charger->req_current > MAX_PPS_I) {
				charger->req_current = MAX_PPS_I;
			}

			obj_power =
				(unsigned int) (charger->req_voltage * charger->req_current);
			if (obj_power > max_power){
				max_power = obj_power;
				obj_pos = i + 1;
			}
#ifdef SUPPORT_ONSEMI_PDCONTROL
			if (fusb_fetch_pdo_chip != NULL){
				fusb_fetch_pdo_chip->fusb302_pd_active = true;
				fusb_pd_enable = fusb_fetch_pdo_chip->fusb302_pd_active;
			}
#endif
		}
	}

    /* Save Request object */
	charger->requested_pdo = obj_pos;

	/* Evaluate voltage and current for request */
    if(obj_pos > 0) {
        charger->req_voltage = port->SrcCapsReceived[obj_pos - 1].PPSAPDO.MaxVoltage * 5;
        charger->req_current = port->SrcCapsReceived[obj_pos - 1].PPSAPDO.MaxCurrent;

        if(charger->req_voltage > MAX_PPS_V) {
            charger->req_voltage = MAX_PPS_V;
        }

        if(charger->req_current > MAX_PPS_I) {
            charger->req_current = MAX_PPS_I;
        }
    }

	/* Queue Get Source Caps Extended to be sent */
    if(port->PdRevSop == USBPDSPECREV3p0) {
        DPM_SendControlMessage(port, CMTGetSourceCapExt);
    }
}

void src_caps_ext_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;
	struct Port *port = charger->port;
	//ExtSrcCapBlock_t *ext_src_caps = (ExtSrcCapBlock_t *)app_ctx;

    /* If app_ctx is 0, message was Not Supported */
    if(app_ctx) {
        /* Evaluate Extended Source Cap info, e.g. check VID for whitelist */
    }

	if(charger->requested_pdo != 0 && port->PolicyIsSource == FALSE) {
		//DPM_SendPPSRequest(port, charger->req_voltage, charger->req_current, charger->requested_pdo);
	}
}

void new_contract_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;
	//struct Port *port = charger->port;
	doDataObject_t *request = (doDataObject_t *)app_ctx;

	/*
	 * Check the app_ctx - if it's the requested PPS, configure the charging path
	 * Note: Valid object positions are 1-7
	 */
	if(request->PPSRDO.ObjectPosition == charger->requested_pdo) {
		charger->direct_charge_active = 1;
	} else {
		charger->direct_charge_active = 0;
	}
}

void alert_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;
	struct Port *port = charger->port;
	doDataObject_t *alert = (doDataObject_t *)app_ctx;

	/* Open battery path if OVP, OTP, OCP */
    if(alert->ADO.OVP || alert->ADO.OTP || alert->ADO.OCP) {
        /* Disable direct charging */
		charger->requested_pdo = 0;
		charger->direct_charge_active = 0;
    }

	/* Possibly reduce load on next request to avoid issue */

	/* Request Status */
	if(!alert->ADO.Battery) {
		DPM_SendControlMessage(port, CMTGetStatus);
	}
}

void status_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	//struct charger_object *charger = (struct charger_object *)usr_ctx;
	//struct Port *port = charger->port;
	//Status_t *status = (Status_t *)app_ctx;

	/* Check status info */
}

void pps_status_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	//struct charger_object *charger = (struct charger_object *)usr_ctx;
	//struct Port *port = charger->port;
	//PPSStatus_t *pps_status = (PPSStatus_t *)app_ctx;

	/* Check pps status info */

}

void pd_failed_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;
	struct Port *port = charger->port;
	doDataObject_t *request = (doDataObject_t *)app_ctx;

    if(port->PolicyIsSource == TRUE) return;
    if(!app_ctx) return;

	/*
	 * Check the app_ctx - if its the requested PPS, send get_source_caps
	 * to check for new source capabilities.
	 * Note: Valid object positions are 1-7
	 */
	if(request->PPSRDO.ObjectPosition == charger->requested_pdo) {
		DPM_SendControlMessage(port, CMTGetSourceCap);
	}
#ifdef SUPPORT_ONSEMI_PDCONTROL
	if (fusb_fetch_pdo_chip != NULL) {
		fusb_fetch_pdo_chip->fusb302_pd_active = false;
		fusb_pd_enable = fusb_fetch_pdo_chip->fusb302_pd_active;
	}
#endif
}

void hard_reset_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;

	/* Disable direct charging */
	charger->requested_pdo = 0;
	charger->direct_charge_active = 0;
}

void typec_detach_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx)
{
	struct charger_object *charger = (struct charger_object *)usr_ctx;

	/* Disable direct charging */
	charger->requested_pdo = 0;
	charger->direct_charge_active = 0;

#ifdef SUPPORT_ONSEMI_PDCONTROL
	pr_info("typec_detach_handler\n");
	if (fusb_fetch_pdo_chip != NULL)
		fusb_fetch_pdo_chip->fusb302_pd_active = false;
		fusb_pd_enable = fusb_fetch_pdo_chip->fusb302_pd_active;

	fusb_call_time = 1;
	//cancel_work(&__pdpm->usb_psy_change_work);
	//fusb_fetch_pdo_chip->get_pdo_info_flag = 0
#endif
#ifdef CONFIG_PRODUCT_MOBA
	reset_charger2_pd_power_data();
	reset_charger2a_pd_power_data();
#endif

}
