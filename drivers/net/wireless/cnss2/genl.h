/* SPDX-License-Identifier: GPL-2.0-only */
<<<<<<< HEAD
/* Copyright (c) 2019, The Linux Foundation. All rights reserved. */
=======
/* Copyright (c) 2019-2020, The Linux Foundation. All rights reserved. */
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

#ifndef __CNSS_GENL_H__
#define __CNSS_GENL_H__

enum cnss_genl_msg_type {
	CNSS_GENL_MSG_TYPE_UNSPEC,
	CNSS_GENL_MSG_TYPE_QDSS,
};

<<<<<<< HEAD
#ifdef CONFIG_CNSS2_DEBUG
=======
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
int cnss_genl_init(void);
void cnss_genl_exit(void);
int cnss_genl_send_msg(void *buff, u8 type,
		       char *file_name, u32 total_size);
<<<<<<< HEAD
#else
static inline int cnss_genl_init(void)
{
	return 0;
}

static inline void cnss_genl_exit(void)
{
}

static inline int cnss_genl_send_msg(void *buff, u8 type,
				     char *file_name, u32 total_size)
{
	return 0;
}
#endif
=======
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

#endif
