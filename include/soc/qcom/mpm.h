/* SPDX-License-Identifier: GPL-2.0-only */
/*
<<<<<<< HEAD
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
=======
 * Copyright (c) 2018-2020, The Linux Foundation. All rights reserved.
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 */

#ifndef __QCOM_MPM_H__
#define __QCOM_MPM_H__

#include <linux/irq.h>
#include <linux/device.h>

struct mpm_pin {
	int pin;
	irq_hw_number_t hwirq;
};

extern const struct mpm_pin mpm_bengal_gic_chip_data[];
<<<<<<< HEAD
=======
extern const struct mpm_pin mpm_scuba_gic_chip_data[];
extern const struct mpm_pin mpm_sdm660_gic_chip_data[];
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
#endif /* __QCOM_MPM_H__ */
