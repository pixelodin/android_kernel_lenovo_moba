/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ARM_LIBFDT_ENV_H
#define _ARM_LIBFDT_ENV_H

<<<<<<< HEAD
=======
#include <linux/limits.h>
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
#include <linux/types.h>
#include <linux/string.h>
#include <asm/byteorder.h>

<<<<<<< HEAD
=======
#define INT32_MAX	S32_MAX
#define UINT32_MAX	U32_MAX

>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
typedef __be16 fdt16_t;
typedef __be32 fdt32_t;
typedef __be64 fdt64_t;

#define fdt16_to_cpu(x)		be16_to_cpu(x)
#define cpu_to_fdt16(x)		cpu_to_be16(x)
#define fdt32_to_cpu(x)		be32_to_cpu(x)
#define cpu_to_fdt32(x)		cpu_to_be32(x)
#define fdt64_to_cpu(x)		be64_to_cpu(x)
#define cpu_to_fdt64(x)		cpu_to_be64(x)

#endif
