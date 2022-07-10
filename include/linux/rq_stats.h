/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2011,2013-2014,2019, The Linux Foundation. All rights reserved.
 */

#define MPCTL_MAX_CMD 128

struct rq_data {
	unsigned long def_timer_jiffies;
	unsigned long def_timer_last_jiffy;
	int64_t def_start_time;
	unsigned char mpctl[MPCTL_MAX_CMD];
	struct attribute_group *attr_group;
	struct kobject *kobj;
	struct work_struct def_timer_work;
	int init;
};

extern spinlock_t rq_lock;
extern struct rq_data rq_info;
extern struct workqueue_struct *rq_wq;
