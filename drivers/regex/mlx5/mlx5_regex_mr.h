/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2018 6WIND S.A.
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#ifndef MLX5_REGEX_MR_H_
#define MLX5_REGEX_MR_H_

#include <stddef.h>
#include <stdint.h>
#include <sys/queue.h>

/* ISO C doesn't support unnamed structs/unions, disabling -pedantic. */
#ifdef PEDANTIC
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <infiniband/verbs.h>
#include <infiniband/mlx5dv.h>
#ifdef PEDANTIC
#pragma GCC diagnostic error "-Wpedantic"
#endif

#include <rte_regexdev.h>
#include <rte_regexdev_core.h>
#include <rte_rwlock.h>
#include <rte_bitmap.h>

/* Size of per-queue MR cache array for linear search. */
#define MLX5_MR_CACHE_N 8

/* Memory Region object. */
struct mlx5_mr {
	LIST_ENTRY(mlx5_mr) mr; /**< Pointer to the prev/next entry. */
	struct ibv_mr *ibv_mr; /* Verbs Memory Region. */
	const struct rte_memseg_list *msl;
	int ms_base_idx; /* Start index of msl->memseg_arr[]. */
	int ms_n; /* Number of memsegs in use. */
	uint32_t ms_bmp_n; /* Number of bits in memsegs bit-mask. */
	struct rte_bitmap *ms_bmp; /* Bit-mask of memsegs belonged to MR. */
};

/* Cache entry for Memory Region. */
struct mlx5_mr_cache {
	uintptr_t start; /* Start address of MR. */
	uintptr_t end; /* End address of MR. */
	uint32_t lkey; /* rte_cpu_to_be_32(ibv_mr->lkey). */
} __rte_packed;

/* MR Cache table for Binary search. */
struct mlx5_mr_btree {
	uint16_t len; /* Number of entries. */
	uint16_t size; /* Total number of entries. */
	int overflow; /* Mark failure of table expansion. */
	struct mlx5_mr_cache (*table)[];
} __rte_packed;

/* Per-queue MR control descriptor. */
struct mlx5_mr_ctrl {
	uint32_t *dev_gen_ptr; /* Generation number of device to poll. */
	uint32_t cur_gen; /* Generation number saved to flush caches. */
	uint16_t mru; /* Index of last hit entry in top-half cache. */
	uint16_t head; /* Index of the oldest entry in top-half cache. */
	struct mlx5_mr_cache cache[MLX5_MR_CACHE_N]; /* Cache for top-half. */
	struct mlx5_mr_btree cache_bh; /* Cache for bottom-half. */
} __rte_packed;

LIST_HEAD(mlx5_mr_list, mlx5_mr);

int mlx5_mr_btree_init(struct mlx5_mr_btree *bt, int n, int socket);
void mlx5_mr_btree_free(struct mlx5_mr_btree *bt);

uint32_t mlx5_regex_mr_addr2mr_bh(struct rte_regex_dev *dev,
			    struct mlx5_mr_ctrl *mr_ctrl, uintptr_t addr);
void mlx5_regex_mr_release(struct rte_regex_dev *dev);

#endif /* MLX5_REGEX_MR_H_ */
