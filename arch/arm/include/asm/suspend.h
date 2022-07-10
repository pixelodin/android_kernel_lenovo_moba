/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_ARM_SUSPEND_H
#define __ASM_ARM_SUSPEND_H

#include <linux/types.h>

struct sleep_save_sp {
	u32 *save_ptr_stash;
	u32 save_ptr_stash_phys;
};

extern void cpu_resume(void);
<<<<<<< HEAD
=======
extern void cpu_resume_no_hyp(void);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
extern void cpu_resume_arm(void);
extern int cpu_suspend(unsigned long, int (*)(unsigned long));

#endif
