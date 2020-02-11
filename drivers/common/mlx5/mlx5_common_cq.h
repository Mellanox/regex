/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */
#ifndef RTE_PMD_MLX5_COMMON_CQ_H_
#define RTE_PMD_MLX5_COMMON_CQ_H_

#include <infiniband/mlx5dv.h>
#include "mlx5_devx_cmds.h"

struct mlx5_common_cq {
	uint16_t log_desc_n;
	uint32_t cq_ci:24;
	uint32_t arm_sn:2;
	struct mlx5_devx_obj *cq;
	struct mlx5dv_devx_umem *umem_obj;
	union {
		volatile void *umem_buf;
		volatile struct mlx5_cqe *cqes;
	};
	volatile uint32_t *db_rec;
	uint32_t *db_addr;
	uint64_t errors;
};

int mlx5_common_create_cq(struct ibv_context *ctx, struct mlx5dv_devx_uar *uar,
			  uint16_t desc_n, uint32_t eqn,
			  struct mlx5_common_cq *cq);
void mlx5_common_destroy_cq(struct mlx5_common_cq *cq);
void mlx5_common_arm_cq(struct mlx5_common_cq *cq);

#endif /* RTE_PMD_MLX5_COMMON_CQ_H_ */
