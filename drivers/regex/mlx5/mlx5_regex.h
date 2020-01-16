/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */

#ifndef MLX5_REGEX_H
#define MLX5_REGEX_H

#include <mlx5_devx_cmds.h>

int mlx5_regex_is_supported(struct ibv_context *ibv_ctx);

struct mlx5_database_ctx {
	uint32_t umem_id;
	uint64_t offset;
};

/*
 * Sets the database address.
 * When the database umem will be freed,
 * the database will no longer be available to the HW.
 */
int mlx5_regex_database_set(struct ibv_context *ctx, int engine_id,
						    struct mlx5_database_ctx *db_ctx);
int mlx5_regex_database_query(struct ibv_context *ctx, int engine_id,
							  struct mlx5_database_ctx *db_ctx);

/*
 * Stop engine after all outstanding jobs to this engine are done.
 */
int mlx5_regex_engine_stop(struct ibv_context *ctx, int engine_id);

/*
 * Return engine to work.
 */
int mlx5_regex_engine_resume(struct ibv_context *ctx, int engine_id);

#endif /* MLX5_REGEX_H */
