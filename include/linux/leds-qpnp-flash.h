/* SPDX-License-Identifier: GPL-2.0 */
/*
<<<<<<< HEAD
 * Copyright (c) 2016-2019, The Linux Foundation. All rights reserved.
=======
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 */

#ifndef __LEDS_QPNP_FLASH_H
#define __LEDS_QPNP_FLASH_H

#include <linux/leds.h>

#define ENABLE_REGULATOR		BIT(0)
#define DISABLE_REGULATOR		BIT(1)
#define QUERY_MAX_AVAIL_CURRENT		BIT(2)
#define QUERY_MAX_CURRENT		BIT(3)

#define FLASH_LED_PREPARE_OPTIONS_MASK	GENMASK(3, 0)

<<<<<<< HEAD
#ifdef CONFIG_LEDS_QPNP_FLASH_V2
=======
int qpnp_flash_register_led_prepare(struct device *dev, void *data);

#if (defined CONFIG_LEDS_QTI_FLASH || defined CONFIG_LEDS_QPNP_FLASH_V2)
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
int qpnp_flash_led_prepare(struct led_trigger *trig, int options,
					int *max_current);
#else
static inline int qpnp_flash_led_prepare(struct led_trigger *trig, int options,
					int *max_current)
{
<<<<<<< HEAD
	return -EINVAL;
=======
	return -ENODEV;
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}
#endif

#ifdef CONFIG_BACKLIGHT_QCOM_SPMI_WLED
int wled_flash_led_prepare(struct led_trigger *trig, int options,
					int *max_current);
#else
static inline int wled_flash_led_prepare(struct led_trigger *trig, int options,
					int *max_current)
{
	return -EINVAL;
}
#endif

#endif
