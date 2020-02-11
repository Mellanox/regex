/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#ifndef MLX5_REGEX_QUEUE_H_
#define MLX5_REGEX_QUEUE_H_

#include <mlx5_prm.h>
#include "mlx5_regex_mr.h"

struct mlx5_regex_wqe_ctrl_seg {
	__be32 le_subset_id_0_subset_id_1;
	__be32 ctrl_subset_id_2_subset_id_3;
};

struct mlx5_regex_buffers {
	struct rte_regex_iov m_iov; /* Buffer for metadata */
	struct rte_regex_iov resp_iov; /* Buffer to receive response from dev */
} __rte_cache_aligned;

/* Queue structure to send requests and receive responses */
struct mlx5_regex_queue {
	uint16_t q_id; /* Index of this queue */
	struct mlx5_mr_ctrl mr_ctrl; /* MR control descriptor */
	/* CQ related fields. */
	void *cqes; /* Completion queue. */
	volatile uint32_t *cq_db; /* Completion queue doorbell. */
	uint16_t cq_ci; /* Consumer index for completion queue. */
	/* WQ related fields. */
	uint32_t sqn; /* sq num */
	void *wqes; /* Work queue. */
	uint16_t q_pi; /* Producer index for work queue. */
	uint16_t q_ci; /* Producer index for work queue. */
	uint16_t q_size; /* Size of the work queue. */
	volatile uint32_t *q_db; /* Work queue doorbell. */
	bool busy; /* State of the queue is being used or not. */
	/* Buffers for reqs, metadata and receive responses from dev */
	struct mlx5_regex_buffers *regex_bufs[];
} __rte_cache_aligned;

LIST_HEAD(mlx5_queue_list, mlx5_queue);

static __rte_always_inline void
set_ctrl_seg(void *seg, uint8_t le, struct rte_regex_ops *ops, uint8_t ctrl)
{
	DEVX_SET(regexp_mmo_control, seg, le, le);
	DEVX_SET(regexp_mmo_control, seg, ctrl, ctrl);
	DEVX_SET(regexp_mmo_control, seg, subset_id_0, ops->group_id0);
	DEVX_SET(regexp_mmo_control, seg, subset_id_1, ops->group_id1);
	DEVX_SET(regexp_mmo_control, seg, subset_id_2, ops->group_id2);
	DEVX_SET(regexp_mmo_control, seg, subset_id_3, ops->group_id3);
}

static __rte_always_inline void
mlx5dv_set_metadata_seg(struct mlx5_wqe_metadata_seg *seg,
			uint32_t mmo_control_31_0, uint32_t lkey,
			uintptr_t address)
{
	seg->mmo_control_31_0 = htobe32(mmo_control_31_0);
	seg->lkey = htobe32(lkey);
	seg->addr = htobe64(address);
}

int mlx5_regex_enqueue(struct rte_regex_dev *dev, uint16_t q_id,
		       struct rte_regex_ops **ops, uint16_t nb_ops);

#endif /* MLX5_REGEX_QUEUE_H_ */
