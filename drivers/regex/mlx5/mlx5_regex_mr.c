/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2016 6WIND S.A.
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#ifdef PEDANTIC
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <infiniband/verbs.h>
#ifdef PEDANTIC
#pragma GCC diagnostic error "-Wpedantic"
#endif

#include <rte_eal_memconfig.h>
#include <rte_mempool.h>
#include <rte_malloc.h>
#include <rte_rwlock.h>
#include <rte_bus_pci.h>
#include <rte_regexdev.h>
#include <rte_regexdev_core.h>

#include <mlx5_glue.h>

#include "mlx5.h"
#include "mlx5_regex_mr.h"
#include "mlx5_regex_utils.h"

struct mr_find_contig_memsegs_data {
	uintptr_t addr;
	uintptr_t start;
	uintptr_t end;
	const struct rte_memseg_list *msl;
};

/**
 * Expand B-tree table to a given size. Can't be called with holding
 * memory_hotplug_lock or priv->mr.rwlock due to rte_realloc().
 *
 * @param bt
 *   Pointer to B-tree structure.
 * @param n
 *   Number of entries for expansion.
 *
 * @return
 *   0 on success, -1 on failure.
 */
static int
mr_btree_expand(struct mlx5_mr_btree *bt, int n)
{
	void *mem;
	int ret = 0;

	if (n <= bt->size)
		return ret;
	/*
	 * Downside of directly using rte_realloc() is that SOCKET_ID_ANY is
	 * used inside if there's no room to expand. Because this is a quite
	 * rare case and a part of very slow path, it is very acceptable.
	 * Initially cache_bh[] will be given practically enough space and once
	 * it is expanded, expansion wouldn't be needed again ever.
	 */
	mem = rte_realloc(bt->table, n * sizeof(struct mlx5_mr_cache), 0);
	if (mem == NULL) {
		/* Not an error, B-tree search will be skipped. */
		DRV_LOG(WARNING, "failed to expand MR B-tree (%p) table",
			(void *)bt);
		ret = -1;
	} else {
		DRV_LOG(DEBUG, "expanded MR B-tree table (size=%u)", n);
		bt->table = mem;
		bt->size = n;
	}
	return ret;
}

/**
 * Look up LKey from given B-tree lookup table, store the last index and return
 * searched LKey.
 *
 * @param bt
 *   Pointer to B-tree structure.
 * @param[out] idx
 *   Pointer to index. Even on search failure, returns index where it stops
 *   searching so that index can be used when inserting a new entry.
 * @param addr
 *   Search key.
 *
 * @return
 *   Searched LKey on success, UINT32_MAX on no match.
 */
static uint32_t
mr_btree_lookup(struct mlx5_mr_btree *bt, uint16_t *idx, uintptr_t addr)
{
	struct mlx5_mr_cache *lkp_tbl;
	uint16_t n;
	uint16_t base = 0;

	assert(bt != NULL);
	lkp_tbl = *bt->table;
	n = bt->len;
	/* First entry must be NULL for comparison. */
	assert(bt->len > 0 || (lkp_tbl[0].start == 0 &&
			       lkp_tbl[0].lkey == UINT32_MAX));
	/* Binary search. */
	do {
		register uint16_t delta = n >> 1;

		if (addr < lkp_tbl[base + delta].start) {
			n = delta;
		} else {
			base += delta;
			n -= delta;
		}
	} while (n > 1);
	assert(addr >= lkp_tbl[base].start);
	*idx = base;
	if (addr < lkp_tbl[base].end)
		return lkp_tbl[base].lkey;
	/* Not found. */
	return UINT32_MAX;
}

/**
 * Insert an entry to B-tree lookup table.
 *
 * @param bt
 *   Pointer to B-tree structure.
 * @param entry
 *   Pointer to new entry to insert.
 *
 * @return
 *   0 on success, -1 on failure.
 */
static int
mr_btree_insert(struct mlx5_mr_btree *bt, struct mlx5_mr_cache *entry)
{
	struct mlx5_mr_cache *lkp_tbl;
	uint16_t idx = 0;
	size_t shift;

	assert(bt != NULL);
	assert(bt->len <= bt->size);
	assert(bt->len > 0);
	lkp_tbl = *bt->table;
	/* Find out the slot for insertion. */
	if (mr_btree_lookup(bt, &idx, entry->start) != UINT32_MAX) {
		DRV_LOG(DEBUG,
			"abort insertion to B-tree(%p): already exist at"
			" idx=%u [0x%" PRIxPTR ", 0x%" PRIxPTR ") lkey=0x%x",
			(void *)bt, idx, entry->start, entry->end, entry->lkey);
		/* Already exist, return. */
		return 0;
	}
	/* If table is full, return error. */
	if (unlikely(bt->len == bt->size)) {
		bt->overflow = 1;
		return -1;
	}
	/* Insert entry. */
	++idx;
	shift = (bt->len - idx) * sizeof(struct mlx5_mr_cache);
	if (shift)
		memmove(&lkp_tbl[idx + 1], &lkp_tbl[idx], shift);
	lkp_tbl[idx] = *entry;
	bt->len++;
	DRV_LOG(DEBUG,
		"inserted B-tree(%p)[%u],"
		" [0x%" PRIxPTR ", 0x%" PRIxPTR ") lkey=0x%x",
		(void *)bt, idx, entry->start, entry->end, entry->lkey);
	return 0;
}

/**
 * Initialize B-tree and allocate memory for lookup table.
 *
 * @param bt
 *   Pointer to B-tree structure.
 * @param n
 *   Number of entries to allocate.
 * @param socket
 *   NUMA socket on which memory must be allocated.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
int
mlx5_mr_btree_init(struct mlx5_mr_btree *bt, int n, int socket)
{
	if (bt == NULL) {
		rte_errno = EINVAL;
		return -rte_errno;
	}
	assert(!bt->table && !bt->size);
	memset(bt, 0, sizeof(*bt));
	bt->table = rte_calloc_socket("B-tree table",
				      n, sizeof(struct mlx5_mr_cache),
				      0, socket);
	if (bt->table == NULL) {
		rte_errno = ENOMEM;
		DEBUG("failed to allocate memory for btree cache on socket %d",
		      socket);
		return -rte_errno;
	}
	bt->size = n;
	/* First entry must be NULL for binary search. */
	(*bt->table)[bt->len++] = (struct mlx5_mr_cache) {
		.lkey = UINT32_MAX,
	};
	DEBUG("initialized B-tree %p with table %p",
	      (void *)bt, (void *)bt->table);
	return 0;
}

/**
 * Free B-tree resources.
 *
 * @param bt
 *   Pointer to B-tree structure.
 */
void
mlx5_mr_btree_free(struct mlx5_mr_btree *bt)
{
	if (bt == NULL)
		return;
	DEBUG("freeing B-tree %p with table %p",
	      (void *)bt, (void *)bt->table);
	rte_free(bt->table);
	memset(bt, 0, sizeof(*bt));
}

/**
 * Find virtually contiguous memory chunk in a given MR.
 *
 * @param dev
 *   Pointer to MR structure.
 * @param[out] entry
 *   Pointer to returning MR cache entry. If not found, this will not be
 *   updated.
 * @param start_idx
 *   Start index of the memseg bitmap.
 *
 * @return
 *   Next index to go on lookup.
 */
static int
mr_find_next_chunk(struct mlx5_mr *mr, struct mlx5_mr_cache *entry,
		   int base_idx)
{
	uintptr_t start = 0;
	uintptr_t end = 0;
	uint32_t idx = 0;

	/* MR for external memory doesn't have memseg list. */
	if (mr->msl == NULL) {
		struct ibv_mr *ibv_mr = mr->ibv_mr;

		assert(mr->ms_bmp_n == 1);
		assert(mr->ms_n == 1);
		assert(base_idx == 0);
		/*
		 * Can't search it from memseg list but get it directly from
		 * verbs MR as there's only one chunk.
		 */
		entry->start = (uintptr_t)ibv_mr->addr;
		entry->end = (uintptr_t)ibv_mr->addr + mr->ibv_mr->length;
		entry->lkey = rte_cpu_to_be_32(mr->ibv_mr->lkey);
		/* Returning 1 ends iteration. */
		return 1;
	}
	for (idx = base_idx; idx < mr->ms_bmp_n; ++idx) {
		if (rte_bitmap_get(mr->ms_bmp, idx)) {
			const struct rte_memseg_list *msl;
			const struct rte_memseg *ms;

			msl = mr->msl;
			ms = rte_fbarray_get(&msl->memseg_arr,
					     mr->ms_base_idx + idx);
			assert(msl->page_sz == ms->hugepage_sz);
			if (!start)
				start = ms->addr_64;
			end = ms->addr_64 + ms->hugepage_sz;
		} else if (start) {
			/* Passed the end of a fragment. */
			break;
		}
	}
	if (start) {
		/* Found one chunk. */
		entry->start = start;
		entry->end = end;
		entry->lkey = rte_cpu_to_be_32(mr->ibv_mr->lkey);
	}
	return idx;
}

/**
 * Insert a MR to the global B-tree cache. It may fail due to low-on-memory.
 * Then, this entry will have to be searched by mr_lookup_dev_list() in
 * mlx5_mr_create() on miss.
 *
 * @param priv
 *   Pointer to REGEX device private context.
 * @param mr
 *   Pointer to MR to insert.
 *
 * @return
 *   0 on success, -1 on failure.
 */
static int
mr_insert_dev_cache(struct mlx5_regex_priv *priv, struct mlx5_mr *mr)
{
	unsigned int n;

	DRV_LOG(DEBUG, "REGEX dev %u inserting MR(%p) to global cache",
		priv->regex_dev.dev_id, (void *)mr);
	for (n = 0; n < mr->ms_bmp_n; ) {
		struct mlx5_mr_cache entry;

		memset(&entry, 0, sizeof(entry));
		/* Find a contiguous chunk and advance the index. */
		n = mr_find_next_chunk(mr, &entry, n);
		if (!entry.end)
			break;
		if (mr_btree_insert(&priv->mr.cache, &entry) < 0) {
			/*
			 * Overflowed, but the global table cannot be expanded
			 * because of deadlock.
			 */
			return -1;
		}
	}
	return 0;
}

/**
 * Look up address in the original global MR list.
 *
 * @param priv
 *   Pointer to REGEX device private context.
 * @param[out] entry
 *   Pointer to returning MR cache entry. If no match, this will not be updated.
 * @param addr
 *   Search key.
 *
 * @return
 *   Found MR on match, NULL otherwise.
 */
static struct mlx5_mr *
mr_lookup_dev_list(struct mlx5_regex_priv *priv, struct mlx5_mr_cache *entry,
		   uintptr_t addr)
{
	struct mlx5_mr *mr;

	/* Iterate all the existing MRs. */
	LIST_FOREACH(mr, &priv->mr.mr_list, mr) {
		unsigned int n;

		if (mr->ms_n == 0)
			continue;
		for (n = 0; n < mr->ms_bmp_n; ) {
			struct mlx5_mr_cache ret;

			memset(&ret, 0, sizeof(ret));
			n = mr_find_next_chunk(mr, &ret, n);
			if (addr >= ret.start && addr < ret.end) {
				/* Found. */
				*entry = ret;
				return mr;
			}
		}
	}
	return NULL;
}

/**
 * Look up address on device.
 *
 * @param priv
 *   Pointer to REGEX device private context.
 * @param[out] entry
 *   Pointer to returning MR cache entry. If no match, this will not be updated.
 * @param addr
 *   Search key.
 *
 * @return
 *   Searched LKey on success, UINT32_MAX on failure and rte_errno is set.
 */
static uint32_t
mr_lookup_dev(struct mlx5_regex_priv *priv, struct mlx5_mr_cache *entry,
	      uintptr_t addr)
{
	uint16_t idx;
	uint32_t lkey = UINT32_MAX;
	struct mlx5_mr *mr;

	/*
	 * If the global cache has overflowed since it failed to expand the
	 * B-tree table, it can't have all the existing MRs. Then, the address
	 * has to be searched by traversing the original MR list instead, which
	 * is very slow path. Otherwise, the global cache is all inclusive.
	 */
	if (!unlikely(priv->mr.cache.overflow)) {
		lkey = mr_btree_lookup(&priv->mr.cache, &idx, addr);
		if (lkey != UINT32_MAX)
			*entry = (*priv->mr.cache.table)[idx];
	} else {
		/* Falling back to the slowest path. */
		mr = mr_lookup_dev_list(priv, entry, addr);
		if (mr != NULL)
			lkey = entry->lkey;
	}
	assert(lkey == UINT32_MAX || (addr >= entry->start &&
				      addr < entry->end));
	return lkey;
}

/**
 * Free MR resources. MR lock must not be held to avoid a deadlock. rte_free()
 * can raise memory free event and the callback function will spin on the lock.
 *
 * @param mr
 *   Pointer to MR to free.
 */
static void
mr_free(struct mlx5_mr *mr)
{
	if (mr == NULL)
		return;
	DRV_LOG(DEBUG, "freeing MR(%p):", (void *)mr);
	if (mr->ibv_mr != NULL)
		claim_zero(mlx5_glue->dereg_mr(mr->ibv_mr));
	if (mr->ms_bmp != NULL)
		rte_bitmap_free(mr->ms_bmp);
	rte_free(mr);
}

/**
 * Release resources of detached MR having no online entry.
 *
 * @param priv
 *   Pointer to REGEX device private context.
 */
static void
mlx5_mr_garbage_collect(struct mlx5_regex_priv *priv)
{
	struct mlx5_mr *mr_next;
	struct mlx5_mr_list free_list = LIST_HEAD_INITIALIZER(free_list);

	/* Must be called from the primary process. */
	assert(rte_eal_process_type() == RTE_PROC_PRIMARY);
	/*
	 * MR can't be freed with holding the lock because rte_free() could call
	 * memory free callback function. This will be a deadlock situation.
	 */
	rte_rwlock_write_lock(&priv->mr.rwlock);
	/* Detach the whole free list and release it after unlocking. */
	free_list = priv->mr.mr_free_list;
	LIST_INIT(&priv->mr.mr_free_list);
	rte_rwlock_write_unlock(&priv->mr.rwlock);
	/* Release resources. */
	mr_next = LIST_FIRST(&free_list);
	while (mr_next != NULL) {
		struct mlx5_mr *mr = mr_next;

		mr_next = LIST_NEXT(mr, mr);
		mr_free(mr);
	}
}

/* Called during rte_memseg_contig_walk() by mlx5_mr_create(). */
static int
mr_find_contig_memsegs_cb(const struct rte_memseg_list *msl,
			  const struct rte_memseg *ms, size_t len, void *arg)
{
	struct mr_find_contig_memsegs_data *data = arg;

	if (data->addr < ms->addr_64 || data->addr >= ms->addr_64 + len)
		return 0;
	/* Found, save it and stop walking. */
	data->start = ms->addr_64;
	data->end = ms->addr_64 + len;
	data->msl = msl;
	return 1;
}

/**
 * Create a new global Memory Region (MR) for a missing virtual address.
 * Register entire virtually contiguous memory chunk around the address.
 *
 * @param dev
 *   Pointer to Regex device.
 * @param[out] entry
 *   Pointer to returning MR cache entry, found in the global cache or newly
 *   created. If failed to create one, this will not be updated.
 * @param addr
 *   Target virtual address to register.
 *
 * @return
 *   Searched LKey on success, UINT32_MAX on failure and rte_errno is set.
 */
static uint32_t
mlx5_regex_mr_create(struct rte_regex_dev *dev,
		     struct mlx5_mr_cache *entry, uintptr_t addr)
{
	struct mlx5_regex_priv *priv =
		container_of(dev, struct mlx5_regex_priv, regex_dev);
	struct mr_find_contig_memsegs_data data = {.addr = addr, };
	struct mr_find_contig_memsegs_data data_re;
	const struct rte_memseg_list *msl;
	const struct rte_memseg *ms;
	struct mlx5_mr *mr = NULL;
	int ms_idx_shift = -1;
	uint32_t bmp_size;
	void *bmp_mem;
	uint32_t ms_n;
	uint32_t n;
	size_t len;

	DRV_LOG(DEBUG, "REGEX_dev %u creating a MR using address (%p)",
		dev->dev_id, (void *)addr);
	/*
	 * Release detached MRs if any. This can't be called with holding either
	 * memory_hotplug_lock or priv->mr.rwlock. MRs on the free list have
	 * been detached by the memory free event but it couldn't be released
	 * inside the callback due to deadlock. As a result, releasing resources
	 * is quite opportunistic.
	 */
	mlx5_mr_garbage_collect(priv);
	/*
	 * Just register one memseg (page). Then, memory
	 * consumption will be minimized but it may drop performance if there
	 * are many MRs to lookup on the datapath.
	 */
	data.msl = rte_mem_virt2memseg_list((void *)addr);
	data.start = RTE_ALIGN_FLOOR(addr, data.msl->page_sz);
	data.end = data.start + data.msl->page_sz;
alloc_resources:
	/* Addresses must be page-aligned. */
	assert(rte_is_aligned((void *)data.start, data.msl->page_sz));
	assert(rte_is_aligned((void *)data.end, data.msl->page_sz));
	msl = data.msl;
	ms = rte_mem_virt2memseg((void *)data.start, msl);
	len = data.end - data.start;
	assert(msl->page_sz == ms->hugepage_sz);
	/* Number of memsegs in the range. */
	ms_n = len / msl->page_sz;
	DEBUG("REGEX_dev %u extending %p to [0x%" PRIxPTR ", 0x%" PRIxPTR "),"
	      " page_sz=0x%" PRIx64 ", ms_n=%u",
	      dev->dev_id, (void *)addr,
	      data.start, data.end, msl->page_sz, ms_n);
	/* Size of memory for bitmap. */
	bmp_size = rte_bitmap_get_memory_footprint(ms_n);
	mr = rte_zmalloc_socket(NULL,
				RTE_ALIGN_CEIL(sizeof(*mr),
					       RTE_CACHE_LINE_SIZE) +
				bmp_size,
				RTE_CACHE_LINE_SIZE, msl->socket_id);
	if (mr == NULL) {
		DEBUG("REGEX_dev %u unable to allocate memory for a new MR of"
		      " address (%p).", dev->dev_id, (void *)addr);
		rte_errno = ENOMEM;
		goto err_nolock;
	}
	mr->msl = msl;
	/*
	 * Save the index of the first memseg and initialize memseg bitmap. To
	 * see if a memseg of ms_idx in the memseg-list is still valid, check:
	 *	rte_bitmap_get(mr->bmp, ms_idx - mr->ms_base_idx)
	 */
	mr->ms_base_idx = rte_fbarray_find_idx(&msl->memseg_arr, ms);
	bmp_mem = RTE_PTR_ALIGN_CEIL(mr + 1, RTE_CACHE_LINE_SIZE);
	mr->ms_bmp = rte_bitmap_init(ms_n, bmp_mem, bmp_size);
	if (mr->ms_bmp == NULL) {
		DEBUG("REGEX dev %u unable to initialize bitmap for a new MR of"
		      " address (%p).",
		      dev->dev_id, (void *)addr);
		rte_errno = EINVAL;
		goto err_nolock;
	}
	/*
	 * Should recheck whether the extended contiguous chunk is still valid.
	 * Because memory_hotplug_lock can't be held if there's any memory
	 * related calls in a critical path, resource allocation above can't be
	 * locked. If the memory has been changed at this point, try again with
	 * just single page. If not, go on with the big chunk atomically from
	 * here.
	 */
	rte_mcfg_mem_read_lock();
	data_re = data;
	if (len > msl->page_sz &&
	    !rte_memseg_contig_walk(mr_find_contig_memsegs_cb, &data_re)) {
		DEBUG("REGEX_dev %u unable to find virtually contiguous"
		      " chunk for address (%p)."
		      " rte_memseg_contig_walk() failed.",
		      dev->dev_id, (void *)addr);
		rte_errno = ENXIO;
		goto err_memlock;
	}
	if (data.start != data_re.start || data.end != data_re.end) {
		/*
		 * The extended contiguous chunk has been changed. Try again
		 * with single memseg instead.
		 */
		data.start = RTE_ALIGN_FLOOR(addr, msl->page_sz);
		data.end = data.start + msl->page_sz;
		rte_mcfg_mem_read_unlock();
		mr_free(mr);
		goto alloc_resources;
	}
	assert(data.msl == data_re.msl);
	rte_rwlock_write_lock(&priv->mr.rwlock);
	/*
	 * Check the address is really missing. If other thread already created
	 * one or it is not found due to overflow, abort and return.
	 */
	if (mr_lookup_dev(priv, entry, addr) != UINT32_MAX) {
		/*
		 * Insert to the global cache table. It may fail due to
		 * low-on-memory. Then, this entry will have to be searched
		 * here again.
		 */
		mr_btree_insert(&priv->mr.cache, entry);
		DEBUG("REGEX dev %u found MR for %p on final lookup, abort",
		      dev->dev_id, (void *)addr);
		rte_rwlock_write_unlock(&priv->mr.rwlock);
		rte_mcfg_mem_read_unlock();
		/*
		 * Must be unlocked before calling rte_free() because
		 * mlx5_mr_mem_event_free_cb() can be called inside.
		 */
		mr_free(mr);
		return entry->lkey;
	}
	/*
	 * Trim start and end addresses for verbs MR. Set bits for registering
	 * memsegs but exclude already registered ones. Bitmap can be
	 * fragmented.
	 */
	for (n = 0; n < ms_n; ++n) {
		uintptr_t start;
		struct mlx5_mr_cache ret;

		memset(&ret, 0, sizeof(ret));
		start = data_re.start + n * msl->page_sz;
		/* Exclude memsegs already registered by other MRs. */
		if (mr_lookup_dev(priv, &ret, start) == UINT32_MAX) {
			/*
			 * Start from the first unregistered memseg in the
			 * extended range.
			 */
			if (ms_idx_shift == -1) {
				mr->ms_base_idx += n;
				data.start = start;
				ms_idx_shift = n;
			}
			data.end = start + msl->page_sz;
			rte_bitmap_set(mr->ms_bmp, n - ms_idx_shift);
			++mr->ms_n;
		}
	}
	len = data.end - data.start;
	mr->ms_bmp_n = len / msl->page_sz;
	assert(ms_idx_shift + mr->ms_bmp_n <= ms_n);
	/*
	 * Finally create a verbs MR for the memory chunk. ibv_reg_mr() can be
	 * called with holding the memory lock because it doesn't use
	 * mlx5_alloc_buf_extern() which eventually calls rte_malloc_socket()
	 * through mlx5_alloc_verbs_buf().
	 */
	mr->ibv_mr = mlx5_glue->reg_mr(priv->pd, (void *)data.start, len,
				       IBV_ACCESS_LOCAL_WRITE);
	if (mr->ibv_mr == NULL) {
		DEBUG("REGEX_dev %u fail to create a verbs MR for address (%p)",
		      dev->dev_id, (void *)addr);
		rte_errno = EINVAL;
		goto err_mrlock;
	}
	assert((uintptr_t)mr->ibv_mr->addr == data.start);
	assert(mr->ibv_mr->length == len);
	LIST_INSERT_HEAD(&priv->mr.mr_list, mr, mr);
	DEBUG("REGEX_dev %u MR CREATED (%p) for %p:\n"
	      "  [0x%" PRIxPTR ", 0x%" PRIxPTR "),"
	      " lkey=0x%x base_idx=%u ms_n=%u, ms_bmp_n=%u",
	      dev->dev_id, (void *)mr, (void *)addr,
	      data.start, data.end, rte_cpu_to_be_32(mr->ibv_mr->lkey),
	      mr->ms_base_idx, mr->ms_n, mr->ms_bmp_n);
	/* Insert to the global cache table. */
	mr_insert_dev_cache(priv, mr);
	/* Fill in output data. */
	mr_lookup_dev(priv, entry, addr);
	/* Lookup can't fail. */
	assert(entry->lkey != UINT32_MAX);
	rte_rwlock_write_unlock(&priv->mr.rwlock);
	rte_mcfg_mem_read_unlock();
	return entry->lkey;
err_mrlock:
	rte_rwlock_write_unlock(&priv->mr.rwlock);
err_memlock:
	rte_mcfg_mem_read_unlock();
err_nolock:
	/*
	 * In case of error, as this can be called in a datapath, a warning
	 * message per an error is preferable instead. Must be unlocked before
	 * calling rte_free() because mlx5_mr_mem_event_free_cb() can be called
	 * inside.
	 */
	mr_free(mr);
	return UINT32_MAX;
}

/**
 * Look up address in the global MR cache table. If not found, create a new MR.
 * Insert the found/created entry to local bottom-half cache table.
 *
 * @param dev
 *   Pointer to REGEX device.
 * @param mr_ctrl
 *   Pointer to per-queue MR control structure.
 * @param[out] entry
 *   Pointer to returning MR cache entry, found in the global cache or newly
 *   created. If failed to create one, this is not written.
 * @param addr
 *   Search key.
 *
 * @return
 *   Searched LKey on success, UINT32_MAX on no match.
 */
static uint32_t
mlx5_regex_mr_lookup_dev(struct rte_regex_dev *dev, struct mlx5_mr_ctrl *mr_ctrl,
			 struct mlx5_mr_cache *entry, uintptr_t addr)
{
	struct mlx5_regex_priv *priv =
		container_of(dev, struct mlx5_regex_priv, regex_dev);
	struct mlx5_mr_btree *bt = &mr_ctrl->cache_bh;
	uint32_t lkey;
	uint16_t idx;

	/* If local cache table is full, try to double it. */
	if (unlikely(bt->len == bt->size))
		mr_btree_expand(bt, bt->size << 1);
	/* Look up in the global cache. */
	rte_rwlock_read_lock(&priv->mr.rwlock);
	lkey = mr_btree_lookup(&priv->mr.cache, &idx, addr);
	if (lkey != UINT32_MAX) {
		/* Found. */
		*entry = (*priv->mr.cache.table)[idx];
		rte_rwlock_read_unlock(&priv->mr.rwlock);
		/*
		 * Update local cache. Even if it fails, return the found entry
		 * to update top-half cache. Next time, this entry will be found
		 * in the global cache.
		 */
		mr_btree_insert(bt, entry);
		return lkey;
	}
	rte_rwlock_read_unlock(&priv->mr.rwlock);
	/* First time to see the address? Create a new MR. */
	lkey = mlx5_regex_mr_create(dev, entry, addr);
	/*
	 * Update the local cache if successfully created a new global MR. Even
	 * if failed to create one, there's no action to take in this datapath
	 * code. As returning LKey is invalid, this will eventually make HW
	 * fail.
	 */
	if (lkey != UINT32_MAX)
		mr_btree_insert(bt, entry);
	return lkey;
}

/**
 * Bottom-half of LKey search on datapath. First search in cache_bh[] and if
 * misses, search in the global MR cache table and update the new entry to
 * per-queue local caches.
 *
 * @param dev
 *   Pointer to REGEX device.
 * @param mr_ctrl
 *   Pointer to per-queue MR control structure.
 * @param addr
 *   Search key.
 *
 * @return
 *   Searched LKey on success, UINT32_MAX on no match.
 */
uint32_t mlx5_regex_mr_addr2mr_bh(struct rte_regex_dev *dev,
				  struct mlx5_mr_ctrl *mr_ctrl, uintptr_t addr)
{
	uint32_t lkey;
	uint16_t bh_idx = 0;
	/* Victim in top-half cache to replace with new entry. */
	struct mlx5_mr_cache *repl = &mr_ctrl->cache[mr_ctrl->head];

	/* Binary-search MR translation table. */
	lkey = mr_btree_lookup(&mr_ctrl->cache_bh, &bh_idx, addr);
	/* Update top-half cache. */
	if (likely(lkey != UINT32_MAX)) {
		*repl = (*mr_ctrl->cache_bh.table)[bh_idx];
	} else {
		/*
		 * If missed in local lookup table, search in the global cache
		 * and local cache_bh[] will be updated inside if possible.
		 * Top-half cache entry will also be updated.
		 */
		lkey = mlx5_regex_mr_lookup_dev(dev, mr_ctrl, repl, addr);
		if (unlikely(lkey == UINT32_MAX))
			return UINT32_MAX;
	}
	/* Update the most recently used entry. */
	mr_ctrl->mru = mr_ctrl->head;
	/* Point to the next victim, the oldest. */
	mr_ctrl->head = (mr_ctrl->head + 1) % MLX5_MR_CACHE_N;
	return lkey;
}

/**
 * Release all the created MRs and resources for REGEX device.
 * list.
 *
 * @param dev
 *   Pointer to REGEX device;
 */
void
mlx5_regex_mr_release(struct rte_regex_dev *dev)
{
	struct mlx5_regex_priv *priv =
		container_of(dev, struct mlx5_regex_priv, regex_dev);
	struct mlx5_mr *mr_next;

	rte_rwlock_write_lock(&priv->mr.rwlock);
	/* Detach from MR list and move to free list. */
	mr_next = LIST_FIRST(&priv->mr.mr_list);
	while (mr_next != NULL) {
		struct mlx5_mr *mr = mr_next;

		mr_next = LIST_NEXT(mr, mr);
		LIST_REMOVE(mr, mr);
		LIST_INSERT_HEAD(&priv->mr.mr_free_list, mr, mr);
	}
	LIST_INIT(&priv->mr.mr_list);
	/* Free global cache. */
	mlx5_mr_btree_free(&priv->mr.cache);
	rte_rwlock_write_unlock(&priv->mr.rwlock);
	/* Free all remaining MRs. */
	mlx5_mr_garbage_collect(priv);
}
