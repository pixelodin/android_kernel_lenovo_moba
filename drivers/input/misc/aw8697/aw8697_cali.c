/*
 * aw_cali_fs.c cali_module
 *
 * Version: v0.1.4
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/module.h>
#include <linux/debugfs.h>
#include <asm/ioctls.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/device.h>
#include "aw8697_cali.h"

#ifdef AW_CALI_STORE_EXAMPLE
/*write cali to persist file example*/
#define AWINIC_CALI_LRA_FILE  "/mnt/vendor/persist/aw_cali_lra.bin"
#define AW_CHAR_DEC_DIGIT 4
static int aw8697_write_cali_lra_to_file(char cali_lra, int channel){
	struct file *fp;
	char buf[50] = {0};
	loff_t pos = 0;
	mm_segment_t fs;

	fp = filp_open(AWINIC_CALI_LRA_FILE, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_err("%s: open %s failed!", __func__, AWINIC_CALI_LRA_FILE);
		return -EINVAL;
	}

	if (channel == AW8697X_CHANNLE)
		pos = AW_CHAR_DEC_DIGIT;

	snprintf(buf, PAGE_SIZE, "%d", cali_lra);

	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp, buf, 4, &pos);

	set_fs(fs);

	pr_info("%s: %s", __func__, buf);

	filp_close(fp, NULL);
	return 0;
}

static int aw8697_get_cali_lra_from_file(char *cali_lra, int channel){
	struct file *fp;
	struct inode *node;
	int f_size;
	char *buf;
	char char_cali_lra = 0;

	loff_t pos = 0;
	mm_segment_t fs;

	fp = filp_open(AWINIC_CALI_LRA_FILE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("%s: open %s failed!", __func__, AWINIC_CALI_LRA_FILE);
		return -EINVAL;
	}
	
	if (channel == AW8697X_CHANNLE)
		pos = AW_CHAR_DEC_DIGIT;

	//#define f_dentry f_path.dentry
	node = fp->f_path.dentry->d_inode;
	f_size = node->i_size;

	buf = (char *)kmalloc(f_size + 1, GFP_ATOMIC);
	if (buf == NULL) {
		pr_err("%s: malloc mem %d failed!", __func__, f_size);
		filp_close(fp, NULL);
		return -EINVAL;
	}

	fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_read(fp, buf, f_size, &pos);

	set_fs(fs);

	if (sscanf(buf, "%d", &char_cali_lra) == 1)
		*cali_lra = char_cali_lra;
	else
		*cali_lra = 0xff;

	pr_info("%s: %d", __func__, *cali_lra);

	kfree(buf);
	filp_close(fp, NULL);

	return  0;

}
#endif

 /*custom need add to set/get cali_lra form/to nv*/
int aw8697_set_cali_lra_to_nvram(char cali_lra, int channel){
	/*custom add, if success return value is 0, else -1*/
#ifdef AW_CALI_STORE_EXAMPLE
	return aw8697_write_cali_lra_to_file(cali_lra, channel);
#else
	return -EBUSY;
#endif
}

int aw8697_get_cali_lra_from_nvram(char *cali_lra, int channel){
	/*custom add, if success return value is 0 , else -1*/
#ifdef AW_CALI_STORE_EXAMPLE
	return aw8697_get_cali_lra_from_file(cali_lra, channel);
#else
	return -EBUSY;
#endif
}


