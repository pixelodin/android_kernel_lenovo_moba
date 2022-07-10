/* SPDX-License-Identifier: GPL-2.0 */
/*
<<<<<<< HEAD
 * A security identifier table (sidtab) is a hash table
 * of security context structures indexed by SID value.
 *
 * Author : Stephen Smalley, <sds@tycho.nsa.gov>
=======
 * A security identifier table (sidtab) is a lookup table
 * of security context structures indexed by SID value.
 *
 * Original author: Stephen Smalley, <sds@tycho.nsa.gov>
 * Author: Ondrej Mosnacek, <omosnacek@gmail.com>
 *
 * Copyright (C) 2018 Red Hat, Inc.
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
 */
#ifndef _SS_SIDTAB_H_
#define _SS_SIDTAB_H_

<<<<<<< HEAD
#include "context.h"

struct sidtab_node {
	u32 sid;		/* security identifier */
	struct context context;	/* security context structure */
	struct sidtab_node *next;
};

#define SIDTAB_HASH_BITS 7
#define SIDTAB_HASH_BUCKETS (1 << SIDTAB_HASH_BITS)
#define SIDTAB_HASH_MASK (SIDTAB_HASH_BUCKETS-1)

#define SIDTAB_SIZE SIDTAB_HASH_BUCKETS

struct sidtab {
	struct sidtab_node **htable;
	unsigned int nel;	/* number of elements */
	unsigned int next_sid;	/* next SID to allocate */
	unsigned char shutdown;
#define SIDTAB_CACHE_LEN	3
	struct sidtab_node *cache[SIDTAB_CACHE_LEN];
	spinlock_t lock;
};

int sidtab_init(struct sidtab *s);
int sidtab_insert(struct sidtab *s, u32 sid, struct context *context);
struct context *sidtab_search(struct sidtab *s, u32 sid);
struct context *sidtab_search_force(struct sidtab *s, u32 sid);

int sidtab_map(struct sidtab *s,
	       int (*apply) (u32 sid,
			     struct context *context,
			     void *args),
	       void *args);

int sidtab_context_to_sid(struct sidtab *s,
			  struct context *context,
			  u32 *sid);

void sidtab_hash_eval(struct sidtab *h, char *tag);
void sidtab_destroy(struct sidtab *s);
void sidtab_set(struct sidtab *dst, struct sidtab *src);
void sidtab_shutdown(struct sidtab *s);
=======
#include <linux/spinlock_types.h>
#include <linux/log2.h>
#include <linux/hashtable.h>

#include "context.h"
#include "flask.h"

struct sidtab_entry_leaf {
	u32 sid;
	struct context context;
	struct hlist_node list;
};

struct sidtab_node_inner;
struct sidtab_node_leaf;

union sidtab_entry_inner {
	struct sidtab_node_inner *ptr_inner;
	struct sidtab_node_leaf  *ptr_leaf;
};

/* align node size to page boundary */
#define SIDTAB_NODE_ALLOC_SHIFT PAGE_SHIFT
#define SIDTAB_NODE_ALLOC_SIZE  PAGE_SIZE

#define size_to_shift(size) ((size) == 1 ? 1 : (const_ilog2((size) - 1) + 1))

#define SIDTAB_INNER_SHIFT \
	(SIDTAB_NODE_ALLOC_SHIFT - size_to_shift(sizeof(union sidtab_entry_inner)))
#define SIDTAB_INNER_ENTRIES ((size_t)1 << SIDTAB_INNER_SHIFT)
#define SIDTAB_LEAF_ENTRIES \
	(SIDTAB_NODE_ALLOC_SIZE / sizeof(struct sidtab_entry_leaf))

#define SIDTAB_MAX_BITS 32
#define SIDTAB_MAX U32_MAX
/* ensure enough tree levels for SIDTAB_MAX entries */
#define SIDTAB_MAX_LEVEL \
	DIV_ROUND_UP(SIDTAB_MAX_BITS - size_to_shift(SIDTAB_LEAF_ENTRIES), \
		     SIDTAB_INNER_SHIFT)

struct sidtab_node_leaf {
	struct sidtab_entry_leaf entries[SIDTAB_LEAF_ENTRIES];
};

struct sidtab_node_inner {
	union sidtab_entry_inner entries[SIDTAB_INNER_ENTRIES];
};

struct sidtab_isid_entry {
	int set;
	struct sidtab_entry_leaf leaf;
};

struct sidtab_convert_params {
	int (*func)(struct context *oldc, struct context *newc, void *args);
	void *args;
	struct sidtab *target;
};

#define SIDTAB_HASH_BITS CONFIG_SECURITY_SELINUX_SIDTAB_HASH_BITS
#define SIDTAB_HASH_BUCKETS (1 << SIDTAB_HASH_BITS)

struct sidtab {
	/*
	 * lock-free read access only for as many items as a prior read of
	 * 'count'
	 */
	union sidtab_entry_inner roots[SIDTAB_MAX_LEVEL + 1];
	/*
	 * access atomically via {READ|WRITE}_ONCE(); only increment under
	 * spinlock
	 */
	u32 count;
	/* access only under spinlock */
	struct sidtab_convert_params *convert;
	spinlock_t lock;

	/* index == SID - 1 (no entry for SECSID_NULL) */
	struct sidtab_isid_entry isids[SECINITSID_NUM];

	/* Hash table for fast reverse context-to-sid lookups. */
	DECLARE_HASHTABLE(context_to_sid, SIDTAB_HASH_BITS);
};

int sidtab_init(struct sidtab *s);
int sidtab_set_initial(struct sidtab *s, u32 sid, struct context *context);
struct context *sidtab_search(struct sidtab *s, u32 sid);
struct context *sidtab_search_force(struct sidtab *s, u32 sid);

int sidtab_convert(struct sidtab *s, struct sidtab_convert_params *params);

int sidtab_context_to_sid(struct sidtab *s, struct context *context, u32 *sid);

void sidtab_destroy(struct sidtab *s);

int sidtab_hash_stats(struct sidtab *sidtab, char *page);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82

#endif	/* _SS_SIDTAB_H_ */


