/* SPDX-License-Identifier: GPL-2.0-only */
/*
<<<<<<< HEAD
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
=======
 * Copyright (c) 2018, 2020 The Linux Foundation. All rights reserved.
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 */

#ifndef __SOC_COM_CX_IPEAK_H
#define __SOC_COM_CX_IPEAK_H

<<<<<<< HEAD
=======
typedef int (*cx_ipeak_victim_fn)(void *data, u32 freq_limit);

>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
struct device_node;
struct cx_ipeak_client;

#ifndef CONFIG_QCOM_CX_IPEAK

static inline struct cx_ipeak_client *cx_ipeak_register(
		struct device_node *dev_node,
		const char *client_name)
{
	return NULL;
}

static inline void cx_ipeak_unregister(struct cx_ipeak_client *client)
{
}

static inline int cx_ipeak_update(struct cx_ipeak_client *ipeak_client,
			bool vote)
{
	return 0;
}
<<<<<<< HEAD
=======

static inline int cx_ipeak_victim_register(struct cx_ipeak_client *client,
		cx_ipeak_victim_fn victim_cb, void *data)
{
	return 0;
}

static inline void cx_ipeak_victim_unregister(struct cx_ipeak_client *client)
{
}
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
#else

struct cx_ipeak_client *cx_ipeak_register(struct device_node *dev_node,
		const char *client_name);
void cx_ipeak_unregister(struct cx_ipeak_client *client);
int cx_ipeak_update(struct cx_ipeak_client *ipeak_client, bool vote);
<<<<<<< HEAD
=======
int cx_ipeak_victim_register(struct cx_ipeak_client *client,
		cx_ipeak_victim_fn victim_cb, void *data);
void cx_ipeak_victim_unregister(struct cx_ipeak_client *client);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

#endif

#endif /*__SOC_COM_CX_IPEAK_H*/
