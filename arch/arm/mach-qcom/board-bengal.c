// SPDX-License-Identifier: GPL-2.0-only
/*
<<<<<<< HEAD
 * Copyright (c) 2019, The Linux Foundation. All rights reserved.
=======
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 */

#include <linux/kernel.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include "board-dt.h"

static const char *trinket_dt_match[] __initconst = {
	"qcom,bengal",
<<<<<<< HEAD
=======
	"qcom,bengal-iot",
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	NULL
};

static void __init trinket_init(void)
{
	board_dt_populate(NULL);
}

DT_MACHINE_START(BENGAL,
	"Qualcomm Technologies, Inc. BENGAL (Flattened Device Tree)")
	.init_machine		= trinket_init,
	.dt_compat		= trinket_dt_match,
MACHINE_END
