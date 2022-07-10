 /* Copyright (C) 2004-2006, Advanced Micro Devices, Inc.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/crypto.h>
#include <linux/spinlock.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
<<<<<<< HEAD
=======
#include <crypto/internal/skcipher.h>
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

#include <linux/io.h>
#include <linux/delay.h>

#include "geode-aes.h"

/* Static structures */

static void __iomem *_iobase;
static spinlock_t lock;

/* Write a 128 bit field (either a writable key or IV) */
static inline void
<<<<<<< HEAD
_writefield(u32 offset, void *value)
=======
_writefield(u32 offset, const void *value)
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
{
	int i;

	for (i = 0; i < 4; i++)
<<<<<<< HEAD
		iowrite32(((u32 *) value)[i], _iobase + offset + (i * 4));
=======
		iowrite32(((const u32 *) value)[i], _iobase + offset + (i * 4));
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}

/* Read a 128 bit field (either a writable key or IV) */
static inline void
_readfield(u32 offset, void *value)
{
	int i;

	for (i = 0; i < 4; i++)
		((u32 *) value)[i] = ioread32(_iobase + offset + (i * 4));
}

static int
<<<<<<< HEAD
do_crypt(void *src, void *dst, int len, u32 flags)
=======
do_crypt(const void *src, void *dst, u32 len, u32 flags)
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
{
	u32 status;
	u32 counter = AES_OP_TIMEOUT;

<<<<<<< HEAD
	iowrite32(virt_to_phys(src), _iobase + AES_SOURCEA_REG);
=======
	iowrite32(virt_to_phys((void *)src), _iobase + AES_SOURCEA_REG);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	iowrite32(virt_to_phys(dst), _iobase + AES_DSTA_REG);
	iowrite32(len,  _iobase + AES_LENA_REG);

	/* Start the operation */
	iowrite32(AES_CTRL_START | flags, _iobase + AES_CTRLA_REG);

	do {
		status = ioread32(_iobase + AES_INTR_REG);
		cpu_relax();
	} while (!(status & AES_INTRA_PENDING) && --counter);

	/* Clear the event */
	iowrite32((status & 0xFF) | AES_INTRA_PENDING, _iobase + AES_INTR_REG);
	return counter ? 0 : 1;
}

<<<<<<< HEAD
static unsigned int
geode_aes_crypt(struct geode_aes_op *op)
=======
static void
geode_aes_crypt(const struct geode_aes_tfm_ctx *tctx, const void *src,
		void *dst, u32 len, u8 *iv, int mode, int dir)
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
{
	u32 flags = 0;
	unsigned long iflags;
	int ret;

<<<<<<< HEAD
	if (op->len == 0)
		return 0;

=======
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	/* If the source and destination is the same, then
	 * we need to turn on the coherent flags, otherwise
	 * we don't need to worry
	 */

	flags |= (AES_CTRL_DCA | AES_CTRL_SCA);

<<<<<<< HEAD
	if (op->dir == AES_DIR_ENCRYPT)
=======
	if (dir == AES_DIR_ENCRYPT)
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
		flags |= AES_CTRL_ENCRYPT;

	/* Start the critical section */

	spin_lock_irqsave(&lock, iflags);

<<<<<<< HEAD
	if (op->mode == AES_MODE_CBC) {
		flags |= AES_CTRL_CBC;
		_writefield(AES_WRITEIV0_REG, op->iv);
	}

	if (!(op->flags & AES_FLAGS_HIDDENKEY)) {
		flags |= AES_CTRL_WRKEY;
		_writefield(AES_WRITEKEY0_REG, op->key);
	}

	ret = do_crypt(op->src, op->dst, op->len, flags);
	BUG_ON(ret);

	if (op->mode == AES_MODE_CBC)
		_readfield(AES_WRITEIV0_REG, op->iv);

	spin_unlock_irqrestore(&lock, iflags);

	return op->len;
=======
	if (mode == AES_MODE_CBC) {
		flags |= AES_CTRL_CBC;
		_writefield(AES_WRITEIV0_REG, iv);
	}

	flags |= AES_CTRL_WRKEY;
	_writefield(AES_WRITEKEY0_REG, tctx->key);

	ret = do_crypt(src, dst, len, flags);
	BUG_ON(ret);

	if (mode == AES_MODE_CBC)
		_readfield(AES_WRITEIV0_REG, iv);

	spin_unlock_irqrestore(&lock, iflags);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}

/* CRYPTO-API Functions */

static int geode_setkey_cip(struct crypto_tfm *tfm, const u8 *key,
		unsigned int len)
{
<<<<<<< HEAD
	struct geode_aes_op *op = crypto_tfm_ctx(tfm);
	unsigned int ret;

	op->keylen = len;

	if (len == AES_KEYSIZE_128) {
		memcpy(op->key, key, len);
=======
	struct geode_aes_tfm_ctx *tctx = crypto_tfm_ctx(tfm);
	unsigned int ret;

	tctx->keylen = len;

	if (len == AES_KEYSIZE_128) {
		memcpy(tctx->key, key, len);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
		return 0;
	}

	if (len != AES_KEYSIZE_192 && len != AES_KEYSIZE_256) {
		/* not supported at all */
		tfm->crt_flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
		return -EINVAL;
	}

	/*
	 * The requested key size is not supported by HW, do a fallback
	 */
<<<<<<< HEAD
	op->fallback.cip->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
	op->fallback.cip->base.crt_flags |= (tfm->crt_flags & CRYPTO_TFM_REQ_MASK);

	ret = crypto_cipher_setkey(op->fallback.cip, key, len);
	if (ret) {
		tfm->crt_flags &= ~CRYPTO_TFM_RES_MASK;
		tfm->crt_flags |= (op->fallback.cip->base.crt_flags & CRYPTO_TFM_RES_MASK);
=======
	tctx->fallback.cip->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
	tctx->fallback.cip->base.crt_flags |=
		(tfm->crt_flags & CRYPTO_TFM_REQ_MASK);

	ret = crypto_cipher_setkey(tctx->fallback.cip, key, len);
	if (ret) {
		tfm->crt_flags &= ~CRYPTO_TFM_RES_MASK;
		tfm->crt_flags |= (tctx->fallback.cip->base.crt_flags &
				   CRYPTO_TFM_RES_MASK);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	}
	return ret;
}

<<<<<<< HEAD
static int geode_setkey_blk(struct crypto_tfm *tfm, const u8 *key,
		unsigned int len)
{
	struct geode_aes_op *op = crypto_tfm_ctx(tfm);
	unsigned int ret;

	op->keylen = len;

	if (len == AES_KEYSIZE_128) {
		memcpy(op->key, key, len);
=======
static int geode_setkey_skcipher(struct crypto_skcipher *tfm, const u8 *key,
				 unsigned int len)
{
	struct geode_aes_tfm_ctx *tctx = crypto_skcipher_ctx(tfm);
	unsigned int ret;

	tctx->keylen = len;

	if (len == AES_KEYSIZE_128) {
		memcpy(tctx->key, key, len);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
		return 0;
	}

	if (len != AES_KEYSIZE_192 && len != AES_KEYSIZE_256) {
		/* not supported at all */
<<<<<<< HEAD
		tfm->crt_flags |= CRYPTO_TFM_RES_BAD_KEY_LEN;
=======
		crypto_skcipher_set_flags(tfm, CRYPTO_TFM_RES_BAD_KEY_LEN);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
		return -EINVAL;
	}

	/*
	 * The requested key size is not supported by HW, do a fallback
	 */
<<<<<<< HEAD
	op->fallback.blk->base.crt_flags &= ~CRYPTO_TFM_REQ_MASK;
	op->fallback.blk->base.crt_flags |= (tfm->crt_flags & CRYPTO_TFM_REQ_MASK);

	ret = crypto_blkcipher_setkey(op->fallback.blk, key, len);
	if (ret) {
		tfm->crt_flags &= ~CRYPTO_TFM_RES_MASK;
		tfm->crt_flags |= (op->fallback.blk->base.crt_flags & CRYPTO_TFM_RES_MASK);
	}
	return ret;
}

static int fallback_blk_dec(struct blkcipher_desc *desc,
		struct scatterlist *dst, struct scatterlist *src,
		unsigned int nbytes)
{
	unsigned int ret;
	struct crypto_blkcipher *tfm;
	struct geode_aes_op *op = crypto_blkcipher_ctx(desc->tfm);

	tfm = desc->tfm;
	desc->tfm = op->fallback.blk;

	ret = crypto_blkcipher_decrypt_iv(desc, dst, src, nbytes);

	desc->tfm = tfm;
	return ret;
}
static int fallback_blk_enc(struct blkcipher_desc *desc,
		struct scatterlist *dst, struct scatterlist *src,
		unsigned int nbytes)
{
	unsigned int ret;
	struct crypto_blkcipher *tfm;
	struct geode_aes_op *op = crypto_blkcipher_ctx(desc->tfm);

	tfm = desc->tfm;
	desc->tfm = op->fallback.blk;

	ret = crypto_blkcipher_encrypt_iv(desc, dst, src, nbytes);

	desc->tfm = tfm;
=======
	crypto_skcipher_clear_flags(tctx->fallback.skcipher,
				    CRYPTO_TFM_REQ_MASK);
	crypto_skcipher_set_flags(tctx->fallback.skcipher,
				  crypto_skcipher_get_flags(tfm) &
				  CRYPTO_TFM_REQ_MASK);
	ret = crypto_skcipher_setkey(tctx->fallback.skcipher, key, len);
	crypto_skcipher_set_flags(tfm,
				  crypto_skcipher_get_flags(tctx->fallback.skcipher) &
				  CRYPTO_TFM_RES_MASK);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	return ret;
}

static void
geode_encrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
<<<<<<< HEAD
	struct geode_aes_op *op = crypto_tfm_ctx(tfm);

	if (unlikely(op->keylen != AES_KEYSIZE_128)) {
		crypto_cipher_encrypt_one(op->fallback.cip, out, in);
		return;
	}

	op->src = (void *) in;
	op->dst = (void *) out;
	op->mode = AES_MODE_ECB;
	op->flags = 0;
	op->len = AES_BLOCK_SIZE;
	op->dir = AES_DIR_ENCRYPT;

	geode_aes_crypt(op);
=======
	const struct geode_aes_tfm_ctx *tctx = crypto_tfm_ctx(tfm);

	if (unlikely(tctx->keylen != AES_KEYSIZE_128)) {
		crypto_cipher_encrypt_one(tctx->fallback.cip, out, in);
		return;
	}

	geode_aes_crypt(tctx, in, out, AES_BLOCK_SIZE, NULL,
			AES_MODE_ECB, AES_DIR_ENCRYPT);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}


static void
geode_decrypt(struct crypto_tfm *tfm, u8 *out, const u8 *in)
{
<<<<<<< HEAD
	struct geode_aes_op *op = crypto_tfm_ctx(tfm);

	if (unlikely(op->keylen != AES_KEYSIZE_128)) {
		crypto_cipher_decrypt_one(op->fallback.cip, out, in);
		return;
	}

	op->src = (void *) in;
	op->dst = (void *) out;
	op->mode = AES_MODE_ECB;
	op->flags = 0;
	op->len = AES_BLOCK_SIZE;
	op->dir = AES_DIR_DECRYPT;

	geode_aes_crypt(op);
=======
	const struct geode_aes_tfm_ctx *tctx = crypto_tfm_ctx(tfm);

	if (unlikely(tctx->keylen != AES_KEYSIZE_128)) {
		crypto_cipher_decrypt_one(tctx->fallback.cip, out, in);
		return;
	}

	geode_aes_crypt(tctx, in, out, AES_BLOCK_SIZE, NULL,
			AES_MODE_ECB, AES_DIR_DECRYPT);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}

static int fallback_init_cip(struct crypto_tfm *tfm)
{
	const char *name = crypto_tfm_alg_name(tfm);
<<<<<<< HEAD
	struct geode_aes_op *op = crypto_tfm_ctx(tfm);

	op->fallback.cip = crypto_alloc_cipher(name, 0,
				CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK);

	if (IS_ERR(op->fallback.cip)) {
		printk(KERN_ERR "Error allocating fallback algo %s\n", name);
		return PTR_ERR(op->fallback.cip);
=======
	struct geode_aes_tfm_ctx *tctx = crypto_tfm_ctx(tfm);

	tctx->fallback.cip = crypto_alloc_cipher(name, 0,
						 CRYPTO_ALG_NEED_FALLBACK);

	if (IS_ERR(tctx->fallback.cip)) {
		printk(KERN_ERR "Error allocating fallback algo %s\n", name);
		return PTR_ERR(tctx->fallback.cip);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	}

	return 0;
}

static void fallback_exit_cip(struct crypto_tfm *tfm)
{
<<<<<<< HEAD
	struct geode_aes_op *op = crypto_tfm_ctx(tfm);

	crypto_free_cipher(op->fallback.cip);
	op->fallback.cip = NULL;
=======
	struct geode_aes_tfm_ctx *tctx = crypto_tfm_ctx(tfm);

	crypto_free_cipher(tctx->fallback.cip);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}

static struct crypto_alg geode_alg = {
	.cra_name			=	"aes",
	.cra_driver_name	=	"geode-aes",
	.cra_priority		=	300,
	.cra_alignmask		=	15,
	.cra_flags			=	CRYPTO_ALG_TYPE_CIPHER |
							CRYPTO_ALG_NEED_FALLBACK,
	.cra_init			=	fallback_init_cip,
	.cra_exit			=	fallback_exit_cip,
	.cra_blocksize		=	AES_BLOCK_SIZE,
<<<<<<< HEAD
	.cra_ctxsize		=	sizeof(struct geode_aes_op),
=======
	.cra_ctxsize		=	sizeof(struct geode_aes_tfm_ctx),
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	.cra_module			=	THIS_MODULE,
	.cra_u				=	{
		.cipher	=	{
			.cia_min_keysize	=	AES_MIN_KEY_SIZE,
			.cia_max_keysize	=	AES_MAX_KEY_SIZE,
			.cia_setkey			=	geode_setkey_cip,
			.cia_encrypt		=	geode_encrypt,
			.cia_decrypt		=	geode_decrypt
		}
	}
};

<<<<<<< HEAD
static int
geode_cbc_decrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct geode_aes_op *op = crypto_blkcipher_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err, ret;

	if (unlikely(op->keylen != AES_KEYSIZE_128))
		return fallback_blk_dec(desc, dst, src, nbytes);

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);
	op->iv = walk.iv;

	while ((nbytes = walk.nbytes)) {
		op->src = walk.src.virt.addr,
		op->dst = walk.dst.virt.addr;
		op->mode = AES_MODE_CBC;
		op->len = nbytes - (nbytes % AES_BLOCK_SIZE);
		op->dir = AES_DIR_DECRYPT;

		ret = geode_aes_crypt(op);

		nbytes -= ret;
		err = blkcipher_walk_done(desc, &walk, nbytes);
	}

	return err;
}

static int
geode_cbc_encrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct geode_aes_op *op = crypto_blkcipher_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err, ret;

	if (unlikely(op->keylen != AES_KEYSIZE_128))
		return fallback_blk_enc(desc, dst, src, nbytes);

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);
	op->iv = walk.iv;

	while ((nbytes = walk.nbytes)) {
		op->src = walk.src.virt.addr,
		op->dst = walk.dst.virt.addr;
		op->mode = AES_MODE_CBC;
		op->len = nbytes - (nbytes % AES_BLOCK_SIZE);
		op->dir = AES_DIR_ENCRYPT;

		ret = geode_aes_crypt(op);
		nbytes -= ret;
		err = blkcipher_walk_done(desc, &walk, nbytes);
	}

	return err;
}

static int fallback_init_blk(struct crypto_tfm *tfm)
{
	const char *name = crypto_tfm_alg_name(tfm);
	struct geode_aes_op *op = crypto_tfm_ctx(tfm);

	op->fallback.blk = crypto_alloc_blkcipher(name, 0,
			CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK);

	if (IS_ERR(op->fallback.blk)) {
		printk(KERN_ERR "Error allocating fallback algo %s\n", name);
		return PTR_ERR(op->fallback.blk);
	}

	return 0;
}

static void fallback_exit_blk(struct crypto_tfm *tfm)
{
	struct geode_aes_op *op = crypto_tfm_ctx(tfm);

	crypto_free_blkcipher(op->fallback.blk);
	op->fallback.blk = NULL;
}

static struct crypto_alg geode_cbc_alg = {
	.cra_name		=	"cbc(aes)",
	.cra_driver_name	=	"cbc-aes-geode",
	.cra_priority		=	400,
	.cra_flags			=	CRYPTO_ALG_TYPE_BLKCIPHER |
						CRYPTO_ALG_KERN_DRIVER_ONLY |
						CRYPTO_ALG_NEED_FALLBACK,
	.cra_init			=	fallback_init_blk,
	.cra_exit			=	fallback_exit_blk,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct geode_aes_op),
	.cra_alignmask		=	15,
	.cra_type			=	&crypto_blkcipher_type,
	.cra_module			=	THIS_MODULE,
	.cra_u				=	{
		.blkcipher	=	{
			.min_keysize	=	AES_MIN_KEY_SIZE,
			.max_keysize	=	AES_MAX_KEY_SIZE,
			.setkey			=	geode_setkey_blk,
			.encrypt		=	geode_cbc_encrypt,
			.decrypt		=	geode_cbc_decrypt,
			.ivsize			=	AES_BLOCK_SIZE,
		}
	}
};

static int
geode_ecb_decrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct geode_aes_op *op = crypto_blkcipher_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err, ret;

	if (unlikely(op->keylen != AES_KEYSIZE_128))
		return fallback_blk_dec(desc, dst, src, nbytes);

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);

	while ((nbytes = walk.nbytes)) {
		op->src = walk.src.virt.addr,
		op->dst = walk.dst.virt.addr;
		op->mode = AES_MODE_ECB;
		op->len = nbytes - (nbytes % AES_BLOCK_SIZE);
		op->dir = AES_DIR_DECRYPT;

		ret = geode_aes_crypt(op);
		nbytes -= ret;
		err = blkcipher_walk_done(desc, &walk, nbytes);
=======
static int geode_init_skcipher(struct crypto_skcipher *tfm)
{
	const char *name = crypto_tfm_alg_name(&tfm->base);
	struct geode_aes_tfm_ctx *tctx = crypto_skcipher_ctx(tfm);

	tctx->fallback.skcipher =
		crypto_alloc_skcipher(name, 0, CRYPTO_ALG_NEED_FALLBACK |
				      CRYPTO_ALG_ASYNC);
	if (IS_ERR(tctx->fallback.skcipher)) {
		printk(KERN_ERR "Error allocating fallback algo %s\n", name);
		return PTR_ERR(tctx->fallback.skcipher);
	}

	crypto_skcipher_set_reqsize(tfm, sizeof(struct skcipher_request) +
				    crypto_skcipher_reqsize(tctx->fallback.skcipher));
	return 0;
}

static void geode_exit_skcipher(struct crypto_skcipher *tfm)
{
	struct geode_aes_tfm_ctx *tctx = crypto_skcipher_ctx(tfm);

	crypto_free_skcipher(tctx->fallback.skcipher);
}

static int geode_skcipher_crypt(struct skcipher_request *req, int mode, int dir)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	const struct geode_aes_tfm_ctx *tctx = crypto_skcipher_ctx(tfm);
	struct skcipher_walk walk;
	unsigned int nbytes;
	int err;

	if (unlikely(tctx->keylen != AES_KEYSIZE_128)) {
		struct skcipher_request *subreq = skcipher_request_ctx(req);

		*subreq = *req;
		skcipher_request_set_tfm(subreq, tctx->fallback.skcipher);
		if (dir == AES_DIR_DECRYPT)
			return crypto_skcipher_decrypt(subreq);
		else
			return crypto_skcipher_encrypt(subreq);
	}

	err = skcipher_walk_virt(&walk, req, false);

	while ((nbytes = walk.nbytes) != 0) {
		geode_aes_crypt(tctx, walk.src.virt.addr, walk.dst.virt.addr,
				round_down(nbytes, AES_BLOCK_SIZE),
				walk.iv, mode, dir);
		err = skcipher_walk_done(&walk, nbytes % AES_BLOCK_SIZE);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	}

	return err;
}

<<<<<<< HEAD
static int
geode_ecb_encrypt(struct blkcipher_desc *desc,
		  struct scatterlist *dst, struct scatterlist *src,
		  unsigned int nbytes)
{
	struct geode_aes_op *op = crypto_blkcipher_ctx(desc->tfm);
	struct blkcipher_walk walk;
	int err, ret;

	if (unlikely(op->keylen != AES_KEYSIZE_128))
		return fallback_blk_enc(desc, dst, src, nbytes);

	blkcipher_walk_init(&walk, dst, src, nbytes);
	err = blkcipher_walk_virt(desc, &walk);

	while ((nbytes = walk.nbytes)) {
		op->src = walk.src.virt.addr,
		op->dst = walk.dst.virt.addr;
		op->mode = AES_MODE_ECB;
		op->len = nbytes - (nbytes % AES_BLOCK_SIZE);
		op->dir = AES_DIR_ENCRYPT;

		ret = geode_aes_crypt(op);
		nbytes -= ret;
		ret =  blkcipher_walk_done(desc, &walk, nbytes);
	}

	return err;
}

static struct crypto_alg geode_ecb_alg = {
	.cra_name			=	"ecb(aes)",
	.cra_driver_name	=	"ecb-aes-geode",
	.cra_priority		=	400,
	.cra_flags			=	CRYPTO_ALG_TYPE_BLKCIPHER |
						CRYPTO_ALG_KERN_DRIVER_ONLY |
						CRYPTO_ALG_NEED_FALLBACK,
	.cra_init			=	fallback_init_blk,
	.cra_exit			=	fallback_exit_blk,
	.cra_blocksize		=	AES_BLOCK_SIZE,
	.cra_ctxsize		=	sizeof(struct geode_aes_op),
	.cra_alignmask		=	15,
	.cra_type			=	&crypto_blkcipher_type,
	.cra_module			=	THIS_MODULE,
	.cra_u				=	{
		.blkcipher	=	{
			.min_keysize	=	AES_MIN_KEY_SIZE,
			.max_keysize	=	AES_MAX_KEY_SIZE,
			.setkey			=	geode_setkey_blk,
			.encrypt		=	geode_ecb_encrypt,
			.decrypt		=	geode_ecb_decrypt,
		}
	}
=======
static int geode_cbc_encrypt(struct skcipher_request *req)
{
	return geode_skcipher_crypt(req, AES_MODE_CBC, AES_DIR_ENCRYPT);
}

static int geode_cbc_decrypt(struct skcipher_request *req)
{
	return geode_skcipher_crypt(req, AES_MODE_CBC, AES_DIR_DECRYPT);
}

static int geode_ecb_encrypt(struct skcipher_request *req)
{
	return geode_skcipher_crypt(req, AES_MODE_ECB, AES_DIR_ENCRYPT);
}

static int geode_ecb_decrypt(struct skcipher_request *req)
{
	return geode_skcipher_crypt(req, AES_MODE_ECB, AES_DIR_DECRYPT);
}

static struct skcipher_alg geode_skcipher_algs[] = {
	{
		.base.cra_name		= "cbc(aes)",
		.base.cra_driver_name	= "cbc-aes-geode",
		.base.cra_priority	= 400,
		.base.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY |
					  CRYPTO_ALG_NEED_FALLBACK,
		.base.cra_blocksize	= AES_BLOCK_SIZE,
		.base.cra_ctxsize	= sizeof(struct geode_aes_tfm_ctx),
		.base.cra_alignmask	= 15,
		.base.cra_module	= THIS_MODULE,
		.init			= geode_init_skcipher,
		.exit			= geode_exit_skcipher,
		.setkey			= geode_setkey_skcipher,
		.encrypt		= geode_cbc_encrypt,
		.decrypt		= geode_cbc_decrypt,
		.min_keysize		= AES_MIN_KEY_SIZE,
		.max_keysize		= AES_MAX_KEY_SIZE,
		.ivsize			= AES_BLOCK_SIZE,
	}, {
		.base.cra_name		= "ecb(aes)",
		.base.cra_driver_name	= "ecb-aes-geode",
		.base.cra_priority	= 400,
		.base.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY |
					  CRYPTO_ALG_NEED_FALLBACK,
		.base.cra_blocksize	= AES_BLOCK_SIZE,
		.base.cra_ctxsize	= sizeof(struct geode_aes_tfm_ctx),
		.base.cra_alignmask	= 15,
		.base.cra_module	= THIS_MODULE,
		.init			= geode_init_skcipher,
		.exit			= geode_exit_skcipher,
		.setkey			= geode_setkey_skcipher,
		.encrypt		= geode_ecb_encrypt,
		.decrypt		= geode_ecb_decrypt,
		.min_keysize		= AES_MIN_KEY_SIZE,
		.max_keysize		= AES_MAX_KEY_SIZE,
	},
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
};

static void geode_aes_remove(struct pci_dev *dev)
{
	crypto_unregister_alg(&geode_alg);
<<<<<<< HEAD
	crypto_unregister_alg(&geode_ecb_alg);
	crypto_unregister_alg(&geode_cbc_alg);
=======
	crypto_unregister_skciphers(geode_skcipher_algs,
				    ARRAY_SIZE(geode_skcipher_algs));
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

	pci_iounmap(dev, _iobase);
	_iobase = NULL;

	pci_release_regions(dev);
	pci_disable_device(dev);
}


static int geode_aes_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret;

	ret = pci_enable_device(dev);
	if (ret)
		return ret;

	ret = pci_request_regions(dev, "geode-aes");
	if (ret)
		goto eenable;

	_iobase = pci_iomap(dev, 0, 0);

	if (_iobase == NULL) {
		ret = -ENOMEM;
		goto erequest;
	}

	spin_lock_init(&lock);

	/* Clear any pending activity */
	iowrite32(AES_INTR_PENDING | AES_INTR_MASK, _iobase + AES_INTR_REG);

	ret = crypto_register_alg(&geode_alg);
	if (ret)
		goto eiomap;

<<<<<<< HEAD
	ret = crypto_register_alg(&geode_ecb_alg);
	if (ret)
		goto ealg;

	ret = crypto_register_alg(&geode_cbc_alg);
	if (ret)
		goto eecb;

	dev_notice(&dev->dev, "GEODE AES engine enabled.\n");
	return 0;

 eecb:
	crypto_unregister_alg(&geode_ecb_alg);

=======
	ret = crypto_register_skciphers(geode_skcipher_algs,
					ARRAY_SIZE(geode_skcipher_algs));
	if (ret)
		goto ealg;

	dev_notice(&dev->dev, "GEODE AES engine enabled.\n");
	return 0;

>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 ealg:
	crypto_unregister_alg(&geode_alg);

 eiomap:
	pci_iounmap(dev, _iobase);

 erequest:
	pci_release_regions(dev);

 eenable:
	pci_disable_device(dev);

	dev_err(&dev->dev, "GEODE AES initialization failed.\n");
	return ret;
}

static struct pci_device_id geode_aes_tbl[] = {
	{ PCI_VDEVICE(AMD, PCI_DEVICE_ID_AMD_LX_AES), },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, geode_aes_tbl);

static struct pci_driver geode_aes_driver = {
	.name = "Geode LX AES",
	.id_table = geode_aes_tbl,
	.probe = geode_aes_probe,
	.remove = geode_aes_remove,
};

module_pci_driver(geode_aes_driver);

MODULE_AUTHOR("Advanced Micro Devices, Inc.");
MODULE_DESCRIPTION("Geode LX Hardware AES driver");
MODULE_LICENSE("GPL");
