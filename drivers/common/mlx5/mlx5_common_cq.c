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
#include "mlx5_common_cq.h"

int
mlx5_common_create_cq(struct ibv_context *ctx, struct mlx5dv_devx_uar *uar,
		      uint16_t desc_n, uint32_t eqn,
		      struct mlx5_common_cq *cq)
{
	uint32_t log_desc_n = rte_log2_u32(desc_n);
	uint16_t round_desc_n = 1 << log_desc_n;
	size_t pgsize = sysconf(_SC_PAGESIZE);
	struct mlx5_devx_cq_attr attr;
	volatile struct mlx5_cqe *cqe;
	uint32_t umem_size;
	int i;

	cq->log_desc_n = log_desc_n;
	umem_size = sizeof(*cqe) * round_desc_n + sizeof(*cq->db_rec) * 2;
	cq->umem_buf = rte_zmalloc(__func__, umem_size, pgsize);
	if (!cq->umem_buf) {
		rte_errno = ENOMEM;
		return -ENOMEM;
	}
	cq->umem_obj = mlx5_glue->devx_umem_reg(ctx,
						(void *)(uintptr_t)cq->umem_buf,
						umem_size,
						IBV_ACCESS_LOCAL_WRITE);
	if (!cq->umem_obj)
		goto error;
	attr.q_umem_valid = 1;
	attr.db_umem_valid = 1;
	attr.use_first_only = 0;
	attr.overrun_ignore = 0;
	attr.uar_page_id = uar->page_id;
	attr.q_umem_id = cq->umem_obj->umem_id;
	attr.q_umem_offset = 0;
	attr.db_umem_id = cq->umem_obj->umem_id;
	attr.db_umem_offset = sizeof(*cqe) * round_desc_n;
	attr.eqn = eqn;
	attr.log_cq_size = log_desc_n;
	attr.log_page_size = rte_log2_u32(pgsize);
	cq->cq = mlx5_devx_cmd_create_cq(ctx, &attr);
	if (!cq->cq)
		goto error;
	cq->db_rec = RTE_PTR_ADD(cq->umem_buf, (uintptr_t)attr.db_umem_offset);
	cq->db_addr = RTE_PTR_ADD(uar->base_addr, MLX5_CQ_DOORBELL);
	cq->cq_ci = 0;
	for (i = 0; i < round_desc_n; i++) {
		cqe = cq->cqes + (i & (round_desc_n - 1));
		cqe->op_own = 0xff;
	}
	return 0;
error:
	mlx5_common_destroy_cq(cq);
	return -1;
}

void
mlx5_common_destroy_cq(struct mlx5_common_cq *cq)
{
	int ret __rte_unused;

	if (cq->cq) {
		ret = mlx5_devx_cmd_destroy(cq->cq);
		assert(!ret);
	}
	if (cq->umem_obj) {
		ret = mlx5_glue->devx_umem_dereg(cq->umem_obj);
		assert(!ret);
	}
	if (cq->umem_buf)
		rte_free((void *)(uintptr_t)cq->umem_buf);
	memset(cq, 0, sizeof(*cq));
}

void
mlx5_common_arm_cq(struct mlx5_common_cq *cq)
{
	const unsigned int cqe_mask = (1 << cq->log_desc_n) - 1;
	uint32_t arm_sn = cq->arm_sn << MLX5_CQ_SQN_OFFSET;
	uint32_t cq_ci = cq->cq_ci & MLX5_CI_MASK & cqe_mask;
	uint32_t doorbell_hi = arm_sn | MLX5_CQ_DBR_CMD_ALL | cq_ci;
	uint64_t doorbell = ((uint64_t)doorbell_hi << 32) | cq->cq->id;
	uint64_t db_be = rte_cpu_to_be_64(doorbell);

	rte_io_wmb();
	cq->db_rec[MLX5_CQ_ARM_DB] = rte_cpu_to_be_32(doorbell_hi);
	rte_wmb();
#ifdef RTE_ARCH_64
	*(uint64_t *)cq->db_addr = db_be;
#else
	*cq->db_addr = db_be;
	rte_io_wmb();
	*(cq->db_addr + 1) = db_be >> 32;
#endif
	cq->arm_sn++;
}
