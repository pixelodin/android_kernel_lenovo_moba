// SPDX-License-Identifier: GPL-2.0
/*
 * Implementation of the SID table type.
 *
<<<<<<< HEAD
 * Author : Stephen Smalley, <sds@tycho.nsa.gov>
 */
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/errno.h>
=======
 * Original author: Stephen Smalley, <sds@tycho.nsa.gov>
 * Author: Ondrej Mosnacek, <omosnacek@gmail.com>
 *
 * Copyright (C) 2018 Red Hat, Inc.
 */
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <asm/barrier.h>
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
#include "flask.h"
#include "security.h"
#include "sidtab.h"

<<<<<<< HEAD
#define SIDTAB_HASH(sid) \
(sid & SIDTAB_HASH_MASK)

int sidtab_init(struct sidtab *s)
{
	int i;

	s->htable = kmalloc_array(SIDTAB_SIZE, sizeof(*s->htable), GFP_ATOMIC);
	if (!s->htable)
		return -ENOMEM;
	for (i = 0; i < SIDTAB_SIZE; i++)
		s->htable[i] = NULL;
	s->nel = 0;
	s->next_sid = 1;
	s->shutdown = 0;
=======
#define index_to_sid(index) (index + SECINITSID_NUM + 1)
#define sid_to_index(sid) (sid - (SECINITSID_NUM + 1))

int sidtab_init(struct sidtab *s)
{
	u32 i;

	memset(s->roots, 0, sizeof(s->roots));

	for (i = 0; i < SECINITSID_NUM; i++)
		s->isids[i].set = 0;

	s->count = 0;
	s->convert = NULL;
	hash_init(s->context_to_sid);

>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
	spin_lock_init(&s->lock);
	return 0;
}

<<<<<<< HEAD
int sidtab_insert(struct sidtab *s, u32 sid, struct context *context)
{
	int hvalue;
	struct sidtab_node *prev, *cur, *newnode;

	if (!s)
		return -ENOMEM;

	hvalue = SIDTAB_HASH(sid);
	prev = NULL;
	cur = s->htable[hvalue];
	while (cur && sid > cur->sid) {
		prev = cur;
		cur = cur->next;
	}

	if (cur && sid == cur->sid)
		return -EEXIST;

	newnode = kmalloc(sizeof(*newnode), GFP_ATOMIC);
	if (!newnode)
		return -ENOMEM;

	newnode->sid = sid;
	if (context_cpy(&newnode->context, context)) {
		kfree(newnode);
		return -ENOMEM;
	}

	if (prev) {
		newnode->next = prev->next;
		wmb();
		prev->next = newnode;
	} else {
		newnode->next = s->htable[hvalue];
		wmb();
		s->htable[hvalue] = newnode;
	}

	s->nel++;
	if (sid >= s->next_sid)
		s->next_sid = sid + 1;
	return 0;
}

static struct context *sidtab_search_core(struct sidtab *s, u32 sid, int force)
{
	int hvalue;
	struct sidtab_node *cur;

	if (!s)
		return NULL;

	hvalue = SIDTAB_HASH(sid);
	cur = s->htable[hvalue];
	while (cur && sid > cur->sid)
		cur = cur->next;

	if (force && cur && sid == cur->sid && cur->context.len)
		return &cur->context;

	if (!cur || sid != cur->sid || cur->context.len) {
		/* Remap invalid SIDs to the unlabeled SID. */
		sid = SECINITSID_UNLABELED;
		hvalue = SIDTAB_HASH(sid);
		cur = s->htable[hvalue];
		while (cur && sid > cur->sid)
			cur = cur->next;
		if (!cur || sid != cur->sid)
			return NULL;
	}

	return &cur->context;
=======
static u32 context_to_sid(struct sidtab *s, struct context *context)
{
	struct sidtab_entry_leaf *entry;
	u32 sid = 0;

	rcu_read_lock();
	hash_for_each_possible_rcu(s->context_to_sid, entry, list,
				   context->hash) {
		if (context_cmp(&entry->context, context)) {
			sid = entry->sid;
			break;
		}
	}
	rcu_read_unlock();
	return sid;
}

int sidtab_set_initial(struct sidtab *s, u32 sid, struct context *context)
{
	struct sidtab_isid_entry *entry;
	int rc;

	if (sid == 0 || sid > SECINITSID_NUM)
		return -EINVAL;

	entry = &s->isids[sid - 1];

	rc = context_cpy(&entry->leaf.context, context);
	if (rc)
		return rc;

	entry->set = 1;

	/*
	 * Multiple initial sids may map to the same context. Check that this
	 * context is not already represented in the context_to_sid hashtable
	 * to avoid duplicate entries and long linked lists upon hash
	 * collision.
	 */
	if (!context_to_sid(s, context)) {
		entry->leaf.sid = sid;
		hash_add(s->context_to_sid, &entry->leaf.list, context->hash);
	}

	return 0;
}

int sidtab_hash_stats(struct sidtab *sidtab, char *page)
{
	int i;
	int chain_len = 0;
	int slots_used = 0;
	int entries = 0;
	int max_chain_len = 0;
	int cur_bucket = 0;
	struct sidtab_entry_leaf *entry;

	rcu_read_lock();
	hash_for_each_rcu(sidtab->context_to_sid, i, entry, list) {
		entries++;
		if (i == cur_bucket) {
			chain_len++;
			if (chain_len == 1)
				slots_used++;
		} else {
			cur_bucket = i;
			if (chain_len > max_chain_len)
				max_chain_len = chain_len;
			chain_len = 0;
		}
	}
	rcu_read_unlock();

	if (chain_len > max_chain_len)
		max_chain_len = chain_len;

	return scnprintf(page, PAGE_SIZE, "entries: %d\nbuckets used: %d/%d\n"
			 "longest chain: %d\n", entries,
			 slots_used, SIDTAB_HASH_BUCKETS, max_chain_len);
}

static u32 sidtab_level_from_count(u32 count)
{
	u32 capacity = SIDTAB_LEAF_ENTRIES;
	u32 level = 0;

	while (count > capacity) {
		capacity <<= SIDTAB_INNER_SHIFT;
		++level;
	}
	return level;
}

static int sidtab_alloc_roots(struct sidtab *s, u32 level)
{
	u32 l;

	if (!s->roots[0].ptr_leaf) {
		s->roots[0].ptr_leaf = kzalloc(SIDTAB_NODE_ALLOC_SIZE,
					       GFP_ATOMIC);
		if (!s->roots[0].ptr_leaf)
			return -ENOMEM;
	}
	for (l = 1; l <= level; ++l)
		if (!s->roots[l].ptr_inner) {
			s->roots[l].ptr_inner = kzalloc(SIDTAB_NODE_ALLOC_SIZE,
							GFP_ATOMIC);
			if (!s->roots[l].ptr_inner)
				return -ENOMEM;
			s->roots[l].ptr_inner->entries[0] = s->roots[l - 1];
		}
	return 0;
}

static struct sidtab_entry_leaf *sidtab_do_lookup(struct sidtab *s, u32 index,
						  int alloc)
{
	union sidtab_entry_inner *entry;
	u32 level, capacity_shift, leaf_index = index / SIDTAB_LEAF_ENTRIES;

	/* find the level of the subtree we need */
	level = sidtab_level_from_count(index + 1);
	capacity_shift = level * SIDTAB_INNER_SHIFT;

	/* allocate roots if needed */
	if (alloc && sidtab_alloc_roots(s, level) != 0)
		return NULL;

	/* lookup inside the subtree */
	entry = &s->roots[level];
	while (level != 0) {
		capacity_shift -= SIDTAB_INNER_SHIFT;
		--level;

		entry = &entry->ptr_inner->entries[leaf_index >> capacity_shift];
		leaf_index &= ((u32)1 << capacity_shift) - 1;

		if (!entry->ptr_inner) {
			if (alloc)
				entry->ptr_inner = kzalloc(SIDTAB_NODE_ALLOC_SIZE,
							   GFP_ATOMIC);
			if (!entry->ptr_inner)
				return NULL;
		}
	}
	if (!entry->ptr_leaf) {
		if (alloc)
			entry->ptr_leaf = kzalloc(SIDTAB_NODE_ALLOC_SIZE,
						  GFP_ATOMIC);
		if (!entry->ptr_leaf)
			return NULL;
	}
	return &entry->ptr_leaf->entries[index % SIDTAB_LEAF_ENTRIES];
}

static struct context *sidtab_lookup(struct sidtab *s, u32 index)
{
	/* read entries only after reading count */
	u32 count = smp_load_acquire(&s->count);

	if (index >= count)
		return NULL;

	return &sidtab_do_lookup(s, index, 0)->context;
}

static struct context *sidtab_lookup_initial(struct sidtab *s, u32 sid)
{
	return s->isids[sid - 1].set ? &s->isids[sid - 1].leaf.context : NULL;
}

static struct context *sidtab_search_core(struct sidtab *s, u32 sid, int force)
{
	struct context *context;

	if (sid != 0) {
		if (sid > SECINITSID_NUM)
			context = sidtab_lookup(s, sid_to_index(sid));
		else
			context = sidtab_lookup_initial(s, sid);
		if (context && (!context->len || force))
			return context;
	}

	return sidtab_lookup_initial(s, SECINITSID_UNLABELED);
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}

struct context *sidtab_search(struct sidtab *s, u32 sid)
{
	return sidtab_search_core(s, sid, 0);
}

struct context *sidtab_search_force(struct sidtab *s, u32 sid)
{
	return sidtab_search_core(s, sid, 1);
}

<<<<<<< HEAD
int sidtab_map(struct sidtab *s,
	       int (*apply) (u32 sid,
			     struct context *context,
			     void *args),
	       void *args)
{
	int i, rc = 0;
	struct sidtab_node *cur;

	if (!s)
		goto out;

	for (i = 0; i < SIDTAB_SIZE; i++) {
		cur = s->htable[i];
		while (cur) {
			rc = apply(cur->sid, &cur->context, args);
			if (rc)
				goto out;
			cur = cur->next;
		}
	}
out:
	return rc;
}

static void sidtab_update_cache(struct sidtab *s, struct sidtab_node *n, int loc)
{
	BUG_ON(loc >= SIDTAB_CACHE_LEN);

	while (loc > 0) {
		s->cache[loc] = s->cache[loc - 1];
		loc--;
	}
	s->cache[0] = n;
}

static inline u32 sidtab_search_context(struct sidtab *s,
						  struct context *context)
{
	int i;
	struct sidtab_node *cur;

	for (i = 0; i < SIDTAB_SIZE; i++) {
		cur = s->htable[i];
		while (cur) {
			if (context_cmp(&cur->context, context)) {
				sidtab_update_cache(s, cur, SIDTAB_CACHE_LEN - 1);
				return cur->sid;
			}
			cur = cur->next;
		}
	}
	return 0;
}

static inline u32 sidtab_search_cache(struct sidtab *s, struct context *context)
{
	int i;
	struct sidtab_node *node;

	for (i = 0; i < SIDTAB_CACHE_LEN; i++) {
		node = s->cache[i];
		if (unlikely(!node))
			return 0;
		if (context_cmp(&node->context, context)) {
			sidtab_update_cache(s, node, i);
			return node->sid;
		}
	}
	return 0;
}

int sidtab_context_to_sid(struct sidtab *s,
			  struct context *context,
			  u32 *out_sid)
{
	u32 sid;
	int ret = 0;
	unsigned long flags;

	*out_sid = SECSID_NULL;

	sid  = sidtab_search_cache(s, context);
	if (!sid)
		sid = sidtab_search_context(s, context);
	if (!sid) {
		spin_lock_irqsave(&s->lock, flags);
		/* Rescan now that we hold the lock. */
		sid = sidtab_search_context(s, context);
		if (sid)
			goto unlock_out;
		/* No SID exists for the context.  Allocate a new one. */
		if (s->next_sid == UINT_MAX || s->shutdown) {
			ret = -ENOMEM;
			goto unlock_out;
		}
		sid = s->next_sid++;
		if (context->len)
			pr_info("SELinux:  Context %s is not valid (left unmapped).\n",
			       context->str);
		ret = sidtab_insert(s, sid, context);
		if (ret)
			s->next_sid--;
unlock_out:
		spin_unlock_irqrestore(&s->lock, flags);
	}

	if (ret)
		return ret;

	*out_sid = sid;
	return 0;
}

void sidtab_hash_eval(struct sidtab *h, char *tag)
{
	int i, chain_len, slots_used, max_chain_len;
	struct sidtab_node *cur;

	slots_used = 0;
	max_chain_len = 0;
	for (i = 0; i < SIDTAB_SIZE; i++) {
		cur = h->htable[i];
		if (cur) {
			slots_used++;
			chain_len = 0;
			while (cur) {
				chain_len++;
				cur = cur->next;
			}

			if (chain_len > max_chain_len)
				max_chain_len = chain_len;
		}
	}

	pr_debug("%s:  %d entries and %d/%d buckets used, longest "
	       "chain length %d\n", tag, h->nel, slots_used, SIDTAB_SIZE,
	       max_chain_len);
=======
int sidtab_context_to_sid(struct sidtab *s, struct context *context,
			  u32 *sid)
{
	unsigned long flags;
	u32 count;
	struct sidtab_convert_params *convert;
	struct sidtab_entry_leaf *dst, *dst_convert;
	int rc;

	*sid = context_to_sid(s, context);
	if (*sid)
		return 0;

	/* lock-free search failed: lock, re-search, and insert if not found */
	spin_lock_irqsave(&s->lock, flags);

	rc = 0;
	*sid = context_to_sid(s, context);
	if (*sid)
		goto out_unlock;

	/* read entries only after reading count */
	count = smp_load_acquire(&s->count);
	convert = s->convert;

	/* bail out if we already reached max entries */
	rc = -EOVERFLOW;
	if (count >= SIDTAB_MAX)
		goto out_unlock;

	/* insert context into new entry */
	rc = -ENOMEM;
	dst = sidtab_do_lookup(s, count, 1);
	if (!dst)
		goto out_unlock;

	dst->sid = index_to_sid(count);

	rc = context_cpy(&dst->context, context);
	if (rc)
		goto out_unlock;

	/*
	 * if we are building a new sidtab, we need to convert the context
	 * and insert it there as well
	 */
	if (convert) {
		rc = -ENOMEM;
		dst_convert = sidtab_do_lookup(convert->target, count, 1);
		if (!dst_convert) {
			context_destroy(&dst->context);
			goto out_unlock;
		}

		rc = convert->func(context, &dst_convert->context,
				convert->args);
		if (rc) {
			context_destroy(&dst->context);
			goto out_unlock;
		}
		dst_convert->sid = index_to_sid(count);
		convert->target->count = count + 1;

		hash_add_rcu(convert->target->context_to_sid,
				&dst_convert->list, dst_convert->context.hash);
	}

	if (context->len)
		pr_info("SELinux:  Context %s is not valid (left unmapped).\n",
			context->str);

	*sid = index_to_sid(count);

	/* write entries before updating count */
	smp_store_release(&s->count, count + 1);
	hash_add_rcu(s->context_to_sid, &dst->list, dst->context.hash);

	rc = 0;
out_unlock:
	spin_unlock_irqrestore(&s->lock, flags);
	return rc;
}

static void sidtab_convert_hashtable(struct sidtab *s, u32 count)
{
	struct sidtab_entry_leaf *entry;
	u32 i;

	for (i = 0; i < count; i++) {
		entry = sidtab_do_lookup(s, i, 0);
		entry->sid = index_to_sid(i);

		hash_add_rcu(s->context_to_sid, &entry->list,
				entry->context.hash);

	}
}

static int sidtab_convert_tree(union sidtab_entry_inner *edst,
			       union sidtab_entry_inner *esrc,
			       u32 *pos, u32 count, u32 level,
			       struct sidtab_convert_params *convert)
{
	int rc;
	u32 i;

	if (level != 0) {
		if (!edst->ptr_inner) {
			edst->ptr_inner = kzalloc(SIDTAB_NODE_ALLOC_SIZE,
						  GFP_KERNEL);
			if (!edst->ptr_inner)
				return -ENOMEM;
		}
		i = 0;
		while (i < SIDTAB_INNER_ENTRIES && *pos < count) {
			rc = sidtab_convert_tree(&edst->ptr_inner->entries[i],
						 &esrc->ptr_inner->entries[i],
						 pos, count, level - 1,
						 convert);
			if (rc)
				return rc;
			i++;
		}
	} else {
		if (!edst->ptr_leaf) {
			edst->ptr_leaf = kzalloc(SIDTAB_NODE_ALLOC_SIZE,
						 GFP_KERNEL);
			if (!edst->ptr_leaf)
				return -ENOMEM;
		}
		i = 0;
		while (i < SIDTAB_LEAF_ENTRIES && *pos < count) {
			rc = convert->func(&esrc->ptr_leaf->entries[i].context,
					   &edst->ptr_leaf->entries[i].context,
					   convert->args);
			if (rc)
				return rc;
			(*pos)++;
			i++;
		}
		cond_resched();
	}

	return 0;
}

int sidtab_convert(struct sidtab *s, struct sidtab_convert_params *params)
{
	unsigned long flags;
	u32 count, level, pos;
	int rc;

	spin_lock_irqsave(&s->lock, flags);

	/* concurrent policy loads are not allowed */
	if (s->convert) {
		spin_unlock_irqrestore(&s->lock, flags);
		return -EBUSY;
	}

	count = s->count;
	level = sidtab_level_from_count(count);

	/* allocate last leaf in the new sidtab (to avoid race with
	 * live convert)
	 */
	rc = sidtab_do_lookup(params->target, count - 1, 1) ? 0 : -ENOMEM;
	if (rc) {
		spin_unlock_irqrestore(&s->lock, flags);
		return rc;
	}

	/* set count in case no new entries are added during conversion */
	params->target->count = count;

	/* enable live convert of new entries */
	s->convert = params;

	/* we can safely convert the tree outside the lock */
	spin_unlock_irqrestore(&s->lock, flags);

	pr_info("SELinux:  Converting %u SID table entries...\n", count);

	/* convert all entries not covered by live convert */
	pos = 0;
	rc = sidtab_convert_tree(&params->target->roots[level],
				 &s->roots[level], &pos, count, level, params);
	if (rc) {
		/* we need to keep the old table - disable live convert */
		spin_lock_irqsave(&s->lock, flags);
		s->convert = NULL;
		spin_unlock_irqrestore(&s->lock, flags);
		return rc;
	}
	/*
	 * The hashtable can also be modified in sidtab_context_to_sid()
	 * so we must re-acquire the lock here.
	 */
	spin_lock_irqsave(&s->lock, flags);
	sidtab_convert_hashtable(params->target, count);
	spin_unlock_irqrestore(&s->lock, flags);

	return 0;
}

static void sidtab_destroy_tree(union sidtab_entry_inner entry, u32 level)
{
	u32 i;

	if (level != 0) {
		struct sidtab_node_inner *node = entry.ptr_inner;

		if (!node)
			return;

		for (i = 0; i < SIDTAB_INNER_ENTRIES; i++)
			sidtab_destroy_tree(node->entries[i], level - 1);
		kfree(node);
	} else {
		struct sidtab_node_leaf *node = entry.ptr_leaf;

		if (!node)
			return;

		for (i = 0; i < SIDTAB_LEAF_ENTRIES; i++)
			context_destroy(&node->entries[i].context);
		kfree(node);
	}
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}

void sidtab_destroy(struct sidtab *s)
{
<<<<<<< HEAD
	int i;
	struct sidtab_node *cur, *temp;

	if (!s)
		return;

	for (i = 0; i < SIDTAB_SIZE; i++) {
		cur = s->htable[i];
		while (cur) {
			temp = cur;
			cur = cur->next;
			context_destroy(&temp->context);
			kfree(temp);
		}
		s->htable[i] = NULL;
	}
	kfree(s->htable);
	s->htable = NULL;
	s->nel = 0;
	s->next_sid = 1;
}

void sidtab_set(struct sidtab *dst, struct sidtab *src)
{
	unsigned long flags;
	int i;

	spin_lock_irqsave(&src->lock, flags);
	dst->htable = src->htable;
	dst->nel = src->nel;
	dst->next_sid = src->next_sid;
	dst->shutdown = 0;
	for (i = 0; i < SIDTAB_CACHE_LEN; i++)
		dst->cache[i] = NULL;
	spin_unlock_irqrestore(&src->lock, flags);
}

void sidtab_shutdown(struct sidtab *s)
{
	unsigned long flags;

	spin_lock_irqsave(&s->lock, flags);
	s->shutdown = 1;
	spin_unlock_irqrestore(&s->lock, flags);
=======
	u32 i, level;

	for (i = 0; i < SECINITSID_NUM; i++)
		if (s->isids[i].set)
			context_destroy(&s->isids[i].leaf.context);

	level = SIDTAB_MAX_LEVEL;
	while (level && !s->roots[level].ptr_inner)
		--level;

	sidtab_destroy_tree(s->roots[level], level);
	/*
	 * The context_to_sid hashtable's objects are all shared
	 * with the isids array and context tree, and so don't need
	 * to be cleaned up here.
	 */
>>>>>>> abf4fbc657532dbe8f302d9ce2d78dbd2a009b82
}
