#ifndef _PPS_EXAMPLE_H_
#define _PPS_EXAMPLE_H_

#include "../core/modules/observer.h"
#include "../core/Port.h"
#include "../core/PD_Types.h"

#ifdef CONFIG_PRODUCT_MOBA
#define SUPPORT_ONSEMI_PDCONTROL
#endif

#ifdef SUPPORT_ONSEMI_PDCONTROL
#include "../../../../power/supply/ti/port_2/pd_policy_manager.h"
#endif

struct charger_object {
	struct Port *port;
	unsigned int requested_pdo;
	unsigned int req_voltage; /* 20mV units */
	unsigned int req_current; /* 50mA units */
	unsigned int direct_charge_active;
#ifdef SUPPORT_ONSEMI_PDCONTROL
	bool fusb302_pd_active;
#endif
};

#ifdef SUPPORT_ONSEMI_PDCONTROL
extern struct charger_object* qq_chip;
struct charger_object* fusb30x_charger_GetChip(void);
void fusb30x_charger_SetChip(struct charger_object* cc_Chip);
#endif

void init_sink_pps_example(struct Port *port, struct charger_object *charger);
void src_caps_updated_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void src_caps_ext_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void new_contract_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void alert_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void status_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void pps_status_received_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void pd_failed_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void hard_reset_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);
void typec_detach_handler(FSC_U32 event, FSC_U8 port_id, void *usr_ctx, void *app_ctx);






#endif /* _PPS_EXAMPLE_H_ */
