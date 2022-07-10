/* SPDX-License-Identifier: GPL-2.0 */
/*
<<<<<<< HEAD
 * Copyright (c) 2017, 2019 The Linux Foundation. All rights reserved.
=======
 * Copyright (c) 2017, 2019-2020 The Linux Foundation. All rights reserved.
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 */

#ifndef __BATTERY_H
#define __BATTERY_H

struct charger_param {
	u32 fcc_step_delay_ms;
	u32 fcc_step_size_ua;
	u32 smb_version;
<<<<<<< HEAD
	u32 hvdcp3_max_icl_ua;
	u32 forced_main_fcc;
=======
	u32 hvdcp2_max_icl_ua;
	u32 hvdcp3_max_icl_ua;
	u32 forced_main_fcc;
	u32 qc4_max_icl_ua;
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
};

int qcom_batt_init(struct charger_param *param);
void qcom_batt_deinit(void);
#endif /* __BATTERY_H */
