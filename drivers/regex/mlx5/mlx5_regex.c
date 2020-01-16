/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */
#include <mlx5_prm.h>

#include "mlx5_regex.h"

int mlx5_regex_is_supported(struct ibv_context *ibv_ctx)
{
	struct mlx5_hca_attr caps;
	int err;

	err = mlx5_devx_cmd_query_hca_attr(ibv_ctx, &caps);
	if (err)
		return 0;

	return caps.regex;
}

static int _mlx5_regex_database_set(struct ibv_context *ctx, int engine_id,
				    struct mlx5_database_ctx *db_ctx, int stop,
				    int resume)
{
	uint32_t out[DEVX_ST_SZ_DW(set_regexp_params_out)] = {};
	uint32_t in[DEVX_ST_SZ_DW(set_regexp_params_in)] = {};
	int err;

	DEVX_SET(set_regexp_params_in, in, opcode, MLX5_CMD_SET_REGEX_PARAMS);
	DEVX_SET(set_regexp_params_in, in, engine_id, engine_id);
	if (stop || resume) {
		DEVX_SET(set_regexp_params_in, in, regexp_params.stop_engine, stop);
		DEVX_SET(set_regexp_params_in, in, field_select.stop_engine, 1);
	}

	if (db_ctx) {
		DEVX_SET(set_regexp_params_in, in, regexp_params.db_umem_id, db_ctx->umem_id);
		DEVX_SET64(set_regexp_params_in, in, regexp_params.db_umem_offset, db_ctx->offset);
		DEVX_SET(set_regexp_params_in, in, field_select.db_umem_id, 1);
	}
	err = mlx5dv_devx_general_cmd(ctx, in, sizeof(in), out, sizeof(out));
	if (err) {
		fprintf(stderr, "Set regexp params failed %d\n", err);
		return err;
	}
	return 0;
}

int mlx5_regex_database_set(struct ibv_context *ctx, int engine_id,
			    struct mlx5_database_ctx *db_ctx)
{
	return _mlx5_regex_database_set(ctx, engine_id, db_ctx, 0, 0);
}

int mlx5_regex_engine_stop(struct ibv_context *ctx, int engine_id)
{
	return _mlx5_regex_database_set(ctx, engine_id, NULL, 1, 0);
}

int mlx5_regex_engine_resume(struct ibv_context *ctx, int engine_id)
{
	return _mlx5_regex_database_set(ctx, engine_id, NULL, 0, 1);
}

int mlx5_regex_database_query(struct ibv_context *ctx, int engine_id,
			      struct mlx5_database_ctx *db_ctx)
{
	uint32_t out[DEVX_ST_SZ_DW(query_regexp_params_out)] = {};
	uint32_t in[DEVX_ST_SZ_DW(query_regexp_params_in)] = {};
	int err;

	DEVX_SET(query_regexp_params_in, in, opcode, MLX5_CMD_QUERY_REGEX_PARAMS);
	DEVX_SET(query_regexp_params_in, in, engine_id, engine_id);

	err = mlx5dv_devx_general_cmd(ctx, in, sizeof(in), out, sizeof(out));
	if (err) {
		fprintf(stderr, "Query regexp params failed %d\n", err);
		return err;
	}
	db_ctx->umem_id = DEVX_GET(query_regexp_params_out, out, regexp_params.db_umem_id);
	db_ctx->offset = DEVX_GET(query_regexp_params_out, out, regexp_params.db_umem_offset);
	return 0;
}
