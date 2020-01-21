/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#ifndef MLX5_REGEX_H
#define MLX5_REGEX_H

#include <mlx5_devx_cmds.h>

struct mlx5_database_ctx {
	uint32_t umem_id;
	uint64_t offset;
};

int mlx5_regex_database_set(struct ibv_context *ctx, int engine_id,
			    const struct mlx5_database_ctx *db_ctx);
int mlx5_regex_database_query(struct ibv_context *ctx, int engine_id,
			      struct mlx5_database_ctx *db_ctx);
int mlx5_regex_engine_stop(struct ibv_context *ctx, int engine_id);
int mlx5_regex_engine_go(struct ibv_context *ctx, int engine_id);

int mlx5_regex_register_write(struct ibv_context *ctx, int engine_id,
			      uint32_t addr, uint32_t data);
int mlx5_regex_register_read(struct ibv_context *ctx, int engine_id,
			     uint32_t addr, uint32_t *data);

#endif /* MLX5_REGEX_H */
