/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */
#ifndef RTE_PMD_MLX5_COMMON_SQ_H_
#define RTE_PMD_MLX5_COMMON_SQ_H_

#include <infiniband/mlx5dv.h>
#include "mlx5_devx_cmds.h"
#include "mlx5_common_cq.h"

struct mlx5_common_sq {
	uint16_t log_desc_n;
	struct mlx5_devx_obj *sq;
	struct mlx5dv_devx_umem *umem_obj;
	volatile void *umem_buf;
	volatile uint32_t *db_rec;
	uint32_t *db_addr;
	uint16_t ci;	/* Consumer index of the SQ */
	uint16_t pi;	/* Producer index of the SQ */
	uint16_t num_entries;	/* Number of entries in SQ */
	uint32_t sqn;
};

int mlx5_common_create_sq(struct ibv_context *ctx, struct mlx5dv_devx_uar *uar,
		          uint16_t desc_n, uint32_t pdn, uint32_t qid,
			  const struct mlx5_common_cq *cq,
			  struct mlx5_common_sq *sq);
void mlx5_common_destroy_sq(struct mlx5_common_sq *sq);

#endif /* RTE_PMD_MLX5_COMMON_SQ_H_ */
