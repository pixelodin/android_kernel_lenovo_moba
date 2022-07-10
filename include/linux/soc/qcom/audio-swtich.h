/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/notifier.h>
#include <linux/device.h>

extern struct atomic_notifier_head audio_switch_notifier;
extern int audio_switch_reg_notifier(struct notifier_block *nb);
extern void audio_switch_unreg_notifier(struct notifier_block *nb);
enum audio_switch_notifier_events {
	PSY_EVENT_PROP_CHANGED_AUDIO,
};

struct fsa4480_logic_switch_data{
    int irq_gpio;
	struct device	*dev;
};

extern struct fsa4480_logic_switch_data *fsa4480_logic_data;


extern int audio_switch_state;

