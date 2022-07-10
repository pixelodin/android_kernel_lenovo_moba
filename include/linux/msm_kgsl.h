/* SPDX-License-Identifier: GPL-2.0 */
/*
<<<<<<< HEAD
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
=======
 * Copyright (c) 2018, 2020 The Linux Foundation. All rights reserved.
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 */
#ifndef _MSM_KGSL_H
#define _MSM_KGSL_H

#define KGSL_DEVICE_3D0 0

/* Limits mitigations APIs */
void *kgsl_pwr_limits_add(u32 id);
void kgsl_pwr_limits_del(void *limit);
int kgsl_pwr_limits_set_freq(void *limit, unsigned int freq);
<<<<<<< HEAD
=======
int kgsl_pwr_limits_set_gpu_fmax(void *limit, unsigned int freq);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
void kgsl_pwr_limits_set_default(void *limit);
unsigned int kgsl_pwr_limits_get_freq(u32 id);

#endif /* _MSM_KGSL_H */
