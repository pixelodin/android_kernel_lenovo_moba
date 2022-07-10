// SPDX-License-Identifier: GPL-2.0-only
/*
<<<<<<< HEAD
 * Copyright (c) 2011,2013,2015,2019 The Linux Foundation. All rights reserved.
=======
 * Copyright (c) 2011,2013,2015,2019-2020 The Linux Foundation. All rights reserved.
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 */

#include <linux/module.h>

#include "kgsl_device.h"

/* Instantiate tracepoints */
#define CREATE_TRACE_POINTS
#include "kgsl_trace.h"
<<<<<<< HEAD
=======
#include "kgsl_trace_power.h"
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

EXPORT_TRACEPOINT_SYMBOL(kgsl_regwrite);
EXPORT_TRACEPOINT_SYMBOL(kgsl_issueibcmds);
EXPORT_TRACEPOINT_SYMBOL(kgsl_user_pwrlevel_constraint);
EXPORT_TRACEPOINT_SYMBOL(kgsl_constraint);
<<<<<<< HEAD
=======

EXPORT_TRACEPOINT_SYMBOL(gpu_frequency);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
