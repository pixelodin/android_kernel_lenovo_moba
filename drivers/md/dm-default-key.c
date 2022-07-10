<<<<<<< HEAD
/*
 * Copyright (C) 2017 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device-mapper.h>
#include <linux/module.h>
#include <linux/pfk.h>

#define DM_MSG_PREFIX "default-key"

struct default_key_c {
	struct dm_dev *dev;
	sector_t start;
	struct blk_encryption_key key;
};

static void default_key_dtr(struct dm_target *ti)
{
	struct default_key_c *dkc = ti->private;

	if (dkc->dev)
		dm_put_device(ti, dkc->dev);
	kzfree(dkc);
}

/*
 * Construct a default-key mapping: <mode> <key> <dev_path> <start>
=======
// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2017 Google, Inc.
 */

#include <linux/blk-crypto.h>
#include <linux/device-mapper.h>
#include <linux/module.h>

#define DM_MSG_PREFIX		"default-key"

#define DM_DEFAULT_KEY_MAX_WRAPPED_KEY_SIZE 128

static const struct dm_default_key_cipher {
	const char *name;
	enum blk_crypto_mode_num mode_num;
	int key_size;
} dm_default_key_ciphers[] = {
	{
		.name = "aes-xts-plain64",
		.mode_num = BLK_ENCRYPTION_MODE_AES_256_XTS,
		.key_size = 64,
	}, {
		.name = "xchacha12,aes-adiantum-plain64",
		.mode_num = BLK_ENCRYPTION_MODE_ADIANTUM,
		.key_size = 32,
	},
};

/**
 * struct dm_default_c - private data of a default-key target
 * @dev: the underlying device
 * @start: starting sector of the range of @dev which this target actually maps.
 *	   For this purpose a "sector" is 512 bytes.
 * @cipher_string: the name of the encryption algorithm being used
 * @iv_offset: starting offset for IVs.  IVs are generated as if the target were
 *	       preceded by @iv_offset 512-byte sectors.
 * @sector_size: crypto sector size in bytes (usually 4096)
 * @sector_bits: log2(sector_size)
 * @key: the encryption key to use
 * @max_dun: the maximum DUN that may be used (computed from other params)
 */
struct default_key_c {
	struct dm_dev *dev;
	sector_t start;
	const char *cipher_string;
	u64 iv_offset;
	unsigned int sector_size;
	unsigned int sector_bits;
	struct blk_crypto_key key;
	bool is_hw_wrapped;
	u64 max_dun;
};

static const struct dm_default_key_cipher *
lookup_cipher(const char *cipher_string)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dm_default_key_ciphers); i++) {
		if (strcmp(cipher_string, dm_default_key_ciphers[i].name) == 0)
			return &dm_default_key_ciphers[i];
	}
	return NULL;
}

static void default_key_dtr(struct dm_target *ti)
{
	struct default_key_c *dkc = ti->private;
	int err;

	if (dkc->dev) {
		err = blk_crypto_evict_key(dkc->dev->bdev->bd_queue, &dkc->key);
		if (err && err != -ENOKEY)
			DMWARN("Failed to evict crypto key: %d", err);
		dm_put_device(ti, dkc->dev);
	}
	kzfree(dkc->cipher_string);
	kzfree(dkc);
}

static int default_key_ctr_optional(struct dm_target *ti,
				    unsigned int argc, char **argv)
{
	struct default_key_c *dkc = ti->private;
	struct dm_arg_set as;
	static const struct dm_arg _args[] = {
		{0, 4, "Invalid number of feature args"},
	};
	unsigned int opt_params;
	const char *opt_string;
	bool iv_large_sectors = false;
	char dummy;
	int err;

	as.argc = argc;
	as.argv = argv;

	err = dm_read_arg_group(_args, &as, &opt_params, &ti->error);
	if (err)
		return err;

	while (opt_params--) {
		opt_string = dm_shift_arg(&as);
		if (!opt_string) {
			ti->error = "Not enough feature arguments";
			return -EINVAL;
		}
		if (!strcmp(opt_string, "allow_discards")) {
			ti->num_discard_bios = 1;
		} else if (sscanf(opt_string, "sector_size:%u%c",
				  &dkc->sector_size, &dummy) == 1) {
			if (dkc->sector_size < SECTOR_SIZE ||
			    dkc->sector_size > 4096 ||
			    !is_power_of_2(dkc->sector_size)) {
				ti->error = "Invalid sector_size";
				return -EINVAL;
			}
		} else if (!strcmp(opt_string, "iv_large_sectors")) {
			iv_large_sectors = true;
		} else if (!strcmp(opt_string, "wrappedkey_v0")) {
			dkc->is_hw_wrapped = true;
		} else {
			ti->error = "Invalid feature arguments";
			return -EINVAL;
		}
	}

	/* dm-default-key doesn't implement iv_large_sectors=false. */
	if (dkc->sector_size != SECTOR_SIZE && !iv_large_sectors) {
		ti->error = "iv_large_sectors must be specified";
		return -EINVAL;
	}

	return 0;
}

static void default_key_adjust_sector_size_and_iv(char **argv,
						  struct dm_target *ti,
						  struct default_key_c **dkc,
						  u8 *raw, u32 size,
						  bool is_legacy)
{
	struct dm_dev *dev;
	int i;
	union {
		u8 bytes[BLK_CRYPTO_MAX_WRAPPED_KEY_SIZE];
		u32 words[BLK_CRYPTO_MAX_WRAPPED_KEY_SIZE / sizeof(u32)];
	} key_new;

	dev = (*dkc)->dev;

	if (is_legacy) {
		memcpy(key_new.bytes, raw, size);

		for (i = 0; i < ARRAY_SIZE(key_new.words); i++)
			__cpu_to_be32s(&key_new.words[i]);

		memcpy(raw, key_new.bytes, size);

		if (ti->len & (((*dkc)->sector_size >> SECTOR_SHIFT) - 1))
			(*dkc)->sector_size = SECTOR_SIZE;

		if (dev->bdev->bd_part)
			(*dkc)->iv_offset += dev->bdev->bd_part->start_sect;
	}
}

/*
 * Construct a default-key mapping:
 * <cipher> <key> <iv_offset> <dev_path> <start>
 *
 * This syntax matches dm-crypt's, but lots of unneeded functionality has been
 * removed.  Also, dm-default-key requires that the "iv_large_sectors" option be
 * given whenever a non-default sector size is used.
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 */
static int default_key_ctr(struct dm_target *ti, unsigned int argc, char **argv)
{
	struct default_key_c *dkc;
<<<<<<< HEAD
	size_t key_size;
	unsigned long long tmp;
	char dummy;
	int err;

	if (argc != 4) {
		ti->error = "Invalid argument count";
=======
	const struct dm_default_key_cipher *cipher;
	u8 raw_key[DM_DEFAULT_KEY_MAX_WRAPPED_KEY_SIZE];
	unsigned int raw_key_size;
	unsigned int dun_bytes;
	unsigned long long tmpll;
	char dummy;
	int err;
	char *_argv[10];
	bool is_legacy = false;

	if (argc >= 4 && !strcmp(argv[0], "AES-256-XTS")) {
		argc = 0;
		_argv[argc++] = "aes-xts-plain64";
		_argv[argc++] = argv[1];
		_argv[argc++] = "0";
		_argv[argc++] = argv[2];
		_argv[argc++] = argv[3];
		_argv[argc++] = "3";
		_argv[argc++] = "allow_discards";
		_argv[argc++] = "sector_size:4096";
		_argv[argc++] = "iv_large_sectors";
		_argv[argc] = NULL;
		argv = _argv;
		is_legacy = true;
	}

	if (argc < 5) {
		ti->error = "Not enough arguments";
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
		return -EINVAL;
	}

	dkc = kzalloc(sizeof(*dkc), GFP_KERNEL);
	if (!dkc) {
		ti->error = "Out of memory";
		return -ENOMEM;
	}
	ti->private = dkc;

<<<<<<< HEAD
	if (strcmp(argv[0], "AES-256-XTS") != 0) {
		ti->error = "Unsupported encryption mode";
=======
	/* <cipher> */
	dkc->cipher_string = kstrdup(argv[0], GFP_KERNEL);
	if (!dkc->cipher_string) {
		ti->error = "Out of memory";
		err = -ENOMEM;
		goto bad;
	}
	cipher = lookup_cipher(dkc->cipher_string);
	if (!cipher) {
		ti->error = "Unsupported cipher";
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
		err = -EINVAL;
		goto bad;
	}

<<<<<<< HEAD
	key_size = strlen(argv[1]);
	if (key_size != 2 * BLK_ENCRYPTION_KEY_SIZE_AES_256_XTS) {
		ti->error = "Unsupported key size";
		err = -EINVAL;
		goto bad;
	}
	key_size /= 2;

	if (hex2bin(dkc->key.raw, argv[1], key_size) != 0) {
=======
	/* <key> */
	raw_key_size = strlen(argv[1]);
	if (raw_key_size > 2 * DM_DEFAULT_KEY_MAX_WRAPPED_KEY_SIZE ||
	    raw_key_size % 2) {
		ti->error = "Invalid keysize";
		err = -EINVAL;
		goto bad;
	}
	raw_key_size /= 2;
	if (hex2bin(raw_key, argv[1], raw_key_size) != 0) {
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
		ti->error = "Malformed key string";
		err = -EINVAL;
		goto bad;
	}

<<<<<<< HEAD
	err = dm_get_device(ti, argv[2], dm_table_get_mode(ti->table),
=======
	/* <iv_offset> */
	if (sscanf(argv[2], "%llu%c", &dkc->iv_offset, &dummy) != 1) {
		ti->error = "Invalid iv_offset sector";
		err = -EINVAL;
		goto bad;
	}

	/* <dev_path> */
	err = dm_get_device(ti, argv[3], dm_table_get_mode(ti->table),
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
			    &dkc->dev);
	if (err) {
		ti->error = "Device lookup failed";
		goto bad;
	}

<<<<<<< HEAD
	if (sscanf(argv[3], "%llu%c", &tmp, &dummy) != 1) {
=======
	/* <start> */
	if (sscanf(argv[4], "%llu%c", &tmpll, &dummy) != 1 ||
	    tmpll != (sector_t)tmpll) {
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
		ti->error = "Invalid start sector";
		err = -EINVAL;
		goto bad;
	}
<<<<<<< HEAD
	dkc->start = tmp;

	if (!blk_queue_inlinecrypt(bdev_get_queue(dkc->dev->bdev))) {
		ti->error = "Device does not support inline encryption";
=======
	dkc->start = tmpll;

	/* optional arguments */
	dkc->sector_size = SECTOR_SIZE;
	if (argc > 5) {
		err = default_key_ctr_optional(ti, argc - 5, &argv[5]);
		if (err)
			goto bad;
	}

	default_key_adjust_sector_size_and_iv(argv, ti, &dkc, raw_key,
					      raw_key_size, is_legacy);

	dkc->sector_bits = ilog2(dkc->sector_size);
	if (ti->len & ((dkc->sector_size >> SECTOR_SHIFT) - 1)) {
		ti->error = "Device size is not a multiple of sector_size";
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
		err = -EINVAL;
		goto bad;
	}

<<<<<<< HEAD
	/* Pass flush requests through to the underlying device. */
	ti->num_flush_bios = 1;

	/*
	 * We pass discard requests through to the underlying device, although
	 * the discarded blocks will be zeroed, which leaks information about
	 * unused blocks.  It's also impossible for dm-default-key to know not
	 * to decrypt discarded blocks, so they will not be read back as zeroes
	 * and we must set discard_zeroes_data_unsupported.
	 */
	ti->num_discard_bios = 1;

	/*
	 * It's unclear whether WRITE_SAME would work with inline encryption; it
	 * would depend on whether the hardware duplicates the data before or
	 * after encryption.  But since the internal storage in some  devices
	 * (MSM8998-based) doesn't claim to support WRITE_SAME anyway, we don't
	 * currently have a way to test it.  Leave it disabled it for now.
	 */
	/*ti->num_write_same_bios = 1;*/

	return 0;

bad:
	default_key_dtr(ti);
=======
	dkc->max_dun = (dkc->iv_offset + ti->len - 1) >>
		       (dkc->sector_bits - SECTOR_SHIFT);
	dun_bytes = DIV_ROUND_UP(fls64(dkc->max_dun), 8);

	err = blk_crypto_init_key(&dkc->key, raw_key, raw_key_size,
				  dkc->is_hw_wrapped, cipher->mode_num,
				  dun_bytes, dkc->sector_size);
	if (err) {
		ti->error = "Error initializing blk-crypto key";
		goto bad;
	}

	err = blk_crypto_start_using_mode(cipher->mode_num, dun_bytes,
					  dkc->sector_size, dkc->is_hw_wrapped,
					  dkc->dev->bdev->bd_queue);
	if (err) {
		ti->error = "Error starting to use blk-crypto";
		goto bad;
	}

	ti->num_flush_bios = 1;

	ti->may_passthrough_inline_crypto = true;

	err = 0;
	goto out;

bad:
	default_key_dtr(ti);
out:
	memzero_explicit(raw_key, sizeof(raw_key));
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	return err;
}

static int default_key_map(struct dm_target *ti, struct bio *bio)
{
	const struct default_key_c *dkc = ti->private;
<<<<<<< HEAD

	bio_set_dev(bio, dkc->dev->bdev);
	if (bio_sectors(bio)) {
		bio->bi_iter.bi_sector = dkc->start +
			dm_target_offset(ti, bio->bi_iter.bi_sector);
	}

	if (!bio->bi_crypt_key && !bio->bi_crypt_skip)
		bio->bi_crypt_key = &dkc->key;
=======
	sector_t sector_in_target;
	u64 dun[BLK_CRYPTO_DUN_ARRAY_SIZE] = { 0 };

	bio_set_dev(bio, dkc->dev->bdev);

	/*
	 * If the bio is a device-level request which doesn't target a specific
	 * sector, there's nothing more to do.
	 */
	if (bio_sectors(bio) == 0)
		return DM_MAPIO_REMAPPED;

	/* Map the bio's sector to the underlying device. (512-byte sectors) */
	sector_in_target = dm_target_offset(ti, bio->bi_iter.bi_sector);
	bio->bi_iter.bi_sector = dkc->start + sector_in_target;

	/*
	 * If the bio should skip dm-default-key (i.e. if it's for an encrypted
	 * file's contents), or if it doesn't have any data (e.g. if it's a
	 * DISCARD request), there's nothing more to do.
	 */
	if (bio_should_skip_dm_default_key(bio) || !bio_has_data(bio))
		return DM_MAPIO_REMAPPED;

	/*
	 * Else, dm-default-key needs to set this bio's encryption context.
	 * It must not already have one.
	 */
	if (WARN_ON_ONCE(bio_has_crypt_ctx(bio)))
		return DM_MAPIO_KILL;

	/* Calculate the DUN and enforce data-unit (crypto sector) alignment. */
	dun[0] = dkc->iv_offset + sector_in_target; /* 512-byte sectors */
	if (dun[0] & ((dkc->sector_size >> SECTOR_SHIFT) - 1))
		return DM_MAPIO_KILL;
	dun[0] >>= dkc->sector_bits - SECTOR_SHIFT; /* crypto sectors */

	/*
	 * This check isn't necessary as we should have calculated max_dun
	 * correctly, but be safe.
	 */
	if (WARN_ON_ONCE(dun[0] > dkc->max_dun))
		return DM_MAPIO_KILL;

	bio_crypt_set_ctx(bio, &dkc->key, dun, GFP_NOIO);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

	return DM_MAPIO_REMAPPED;
}

static void default_key_status(struct dm_target *ti, status_type_t type,
			       unsigned int status_flags, char *result,
			       unsigned int maxlen)
{
	const struct default_key_c *dkc = ti->private;
	unsigned int sz = 0;
<<<<<<< HEAD
=======
	int num_feature_args = 0;
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

	switch (type) {
	case STATUSTYPE_INFO:
		result[0] = '\0';
		break;

	case STATUSTYPE_TABLE:
<<<<<<< HEAD

		/* encryption mode */
		DMEMIT("AES-256-XTS");

		/* reserved for key; dm-crypt shows it, but we don't for now */
		DMEMIT(" -");

		/* name of underlying device, and the start sector in it */
		DMEMIT(" %s %llu", dkc->dev->name,
		       (unsigned long long)dkc->start);
=======
		/* Omit the key for now. */
		DMEMIT("%s - %llu %s %llu", dkc->cipher_string, dkc->iv_offset,
		       dkc->dev->name, (unsigned long long)dkc->start);

		num_feature_args += !!ti->num_discard_bios;
		if (dkc->sector_size != SECTOR_SIZE)
			num_feature_args += 2;
		if (dkc->is_hw_wrapped)
			num_feature_args += 1;
		if (num_feature_args != 0) {
			DMEMIT(" %d", num_feature_args);
			if (ti->num_discard_bios)
				DMEMIT(" allow_discards");
			if (dkc->sector_size != SECTOR_SIZE) {
				DMEMIT(" sector_size:%u", dkc->sector_size);
				DMEMIT(" iv_large_sectors");
			}
			if (dkc->is_hw_wrapped)
				DMEMIT(" wrappedkey_v0");
		}
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
		break;
	}
}

static int default_key_prepare_ioctl(struct dm_target *ti,
				     struct block_device **bdev)
{
<<<<<<< HEAD
	struct default_key_c *dkc = ti->private;
	struct dm_dev *dev = dkc->dev;

	*bdev = dev->bdev;

	/*
	 * Only pass ioctls through if the device sizes match exactly.
	 */
	if (dkc->start ||
=======
	const struct default_key_c *dkc = ti->private;
	const struct dm_dev *dev = dkc->dev;

	*bdev = dev->bdev;

	/* Only pass ioctls through if the device sizes match exactly. */
	if (dkc->start != 0 ||
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	    ti->len != i_size_read(dev->bdev->bd_inode) >> SECTOR_SHIFT)
		return 1;
	return 0;
}

static int default_key_iterate_devices(struct dm_target *ti,
				       iterate_devices_callout_fn fn,
				       void *data)
{
<<<<<<< HEAD
	struct default_key_c *dkc = ti->private;
=======
	const struct default_key_c *dkc = ti->private;
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

	return fn(ti, dkc->dev, dkc->start, ti->len, data);
}

<<<<<<< HEAD
static struct target_type default_key_target = {
	.name   = "default-key",
	.version = {1, 0, 0},
	.module = THIS_MODULE,
	.ctr    = default_key_ctr,
	.dtr    = default_key_dtr,
	.map    = default_key_map,
	.status = default_key_status,
	.prepare_ioctl = default_key_prepare_ioctl,
	.iterate_devices = default_key_iterate_devices,
=======
static void default_key_io_hints(struct dm_target *ti,
				 struct queue_limits *limits)
{
	const struct default_key_c *dkc = ti->private;
	const unsigned int sector_size = dkc->sector_size;

	limits->logical_block_size =
		max_t(unsigned short, limits->logical_block_size, sector_size);
	limits->physical_block_size =
		max_t(unsigned int, limits->physical_block_size, sector_size);
	limits->io_min = max_t(unsigned int, limits->io_min, sector_size);
}

static struct target_type default_key_target = {
	.name			= "default-key",
	.version		= {2, 1, 0},
	.module			= THIS_MODULE,
	.ctr			= default_key_ctr,
	.dtr			= default_key_dtr,
	.map			= default_key_map,
	.status			= default_key_status,
	.prepare_ioctl		= default_key_prepare_ioctl,
	.iterate_devices	= default_key_iterate_devices,
	.io_hints		= default_key_io_hints,
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
};

static int __init dm_default_key_init(void)
{
	return dm_register_target(&default_key_target);
}

static void __exit dm_default_key_exit(void)
{
	dm_unregister_target(&default_key_target);
}

module_init(dm_default_key_init);
module_exit(dm_default_key_exit);

MODULE_AUTHOR("Paul Lawrence <paullawrence@google.com>");
MODULE_AUTHOR("Paul Crowley <paulcrowley@google.com>");
MODULE_AUTHOR("Eric Biggers <ebiggers@google.com>");
MODULE_DESCRIPTION(DM_NAME " target for encrypting filesystem metadata");
<<<<<<< HEAD
MODULE_LICENSE("GPL v2");
=======
MODULE_LICENSE("GPL");
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
