/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */
#include <unistd.h>
#include <stdint.h>
#include <assert.h>
#include <fcntl.h>

#include <rte_malloc.h>
#include <rte_errno.h>
#include <rte_lcore.h>
#include <rte_atomic.h>
#include <rte_common.h>

#include "mlx5_glue.h"
#include "mlx5_common.h"
#include "mlx5_common_sq.h"

int
mlx5_common_create_sq(struct ibv_context *ctx, struct mlx5dv_devx_uar *uar,
		      uint16_t desc_n, uint32_t pdn, uint32_t qid,
		      const struct mlx5_common_cq *cq,
		      struct mlx5_common_sq *sq)
{
	struct mlx5_devx_modify_sq_attr mod_attr;
	struct mlx5_devx_create_sq_attr attr;
	uint32_t log_desc_n = rte_log2_u32(desc_n);
	uint32_t umem_size;

	memset(&attr, 0, sizeof(attr));
	sq->log_desc_n = log_desc_n;
	umem_size = MLX5_SEND_WQE_BB * (1 << log_desc_n) + sizeof(*sq->db_rec) * 2;
	sq->umem_buf = rte_zmalloc(__func__, umem_size, 4096);
	if (!sq->umem_buf) {
		rte_errno = ENOMEM;
		return -ENOMEM;
	}
	sq->umem_obj = mlx5_glue->devx_umem_reg(ctx,
						(void *)(uintptr_t)sq->umem_buf,
						umem_size,
						IBV_ACCESS_LOCAL_WRITE);
	if (!sq->umem_obj)
		goto error;
	attr.state = MLX5_SQC_STATE_RST;
	attr.user_index = qid;
	attr.cqn = cq->cq->id;
	attr.tis_lst_sz = 0;
	attr.wq_attr.pd = pdn;
	attr.wq_attr.wq_type = MLX5_WQ_TYPE_CYCLIC;
	attr.wq_attr.uar_page = uar->page_id;
	attr.wq_attr.wq_type = MLX5_WQ_TYPE_CYCLIC;
	attr.wq_attr.wq_umem_id = sq->umem_obj->umem_id;
	attr.wq_attr.dbr_umem_id = sq->umem_obj->umem_id;
	attr.wq_attr.dbr_addr = MLX5_SEND_WQE_BB * (1 << log_desc_n);
	attr.wq_attr.log_wq_stride = rte_log2_u32(MLX5_SEND_WQE_BB);
	attr.wq_attr.log_wq_sz = log_desc_n;
	sq->sq = mlx5_devx_cmd_create_sq(ctx, &attr);
	if (!sq->sq)
		goto error;
	sq->db_rec = RTE_PTR_ADD(sq->umem_buf, (uintptr_t)attr.wq_attr.dbr_addr);
	sq->db_addr = RTE_PTR_ADD(uar->base_addr, MLX5_QP_DOORBELL);
	sq->ci = 0;
	sq->pi = 0;
	sq->num_entries = desc_n;
	sq->sqn = sq->sq->id;
	/* Move sq to ready state */
	memset(&mod_attr, 0, sizeof(mod_attr));
	mod_attr.state = MLX5_SQC_STATE_RDY;
	if (mlx5_devx_cmd_modify_sq(sq->sq, &mod_attr))
		goto error;
	return 0;
error:
	mlx5_common_destroy_sq(sq);
	return -1;
}

void
mlx5_common_destroy_sq(struct mlx5_common_sq *sq)
{
	int ret __rte_unused;

	if (sq->sq) {
		ret = mlx5_devx_cmd_destroy(sq->sq);
		assert(!ret);
	}
	if (sq->umem_obj) {
		ret = mlx5_glue->devx_umem_dereg(sq->umem_obj);
		assert(!ret);
	}
	if (sq->umem_buf)
		rte_free((void *)(uintptr_t)sq->umem_buf);
	memset(sq, 0, sizeof(*sq));
}
