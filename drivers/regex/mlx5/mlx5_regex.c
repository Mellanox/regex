/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */
#include <mlx5_prm.h>

#include "mlx5_regex.h"
#include "mlx5_regex_utils.h"

int mlx5_regex_logtype;

static int
regex_database_set(struct ibv_context *ctx, int engine_id,
		   const struct mlx5_database_ctx *db_ctx, int stop, int go)
{
	uint32_t out[DEVX_ST_SZ_DW(set_regexp_params_out)] = {};
	uint32_t in[DEVX_ST_SZ_DW(set_regexp_params_in)] = {};
	int err;

	DEVX_SET(set_regexp_params_in, in, opcode, MLX5_CMD_SET_REGEX_PARAMS);
	DEVX_SET(set_regexp_params_in, in, engine_id, engine_id);
	if (stop || go) {
		/* stop == 0 AND field select == 1 means GO */
		DEVX_SET(set_regexp_params_in, in, regexp_params.stop_engine,
			 stop);
		DEVX_SET(set_regexp_params_in, in, field_select.stop_engine, 1);
	}

	if (db_ctx) {
		DEVX_SET(set_regexp_params_in, in,
			 regexp_params.db_umem_id, db_ctx->umem_id);
		DEVX_SET64(set_regexp_params_in, in,
			   regexp_params.db_umem_offset, db_ctx->offset);
		DEVX_SET(set_regexp_params_in, in, field_select.db_umem_id, 1);
	}
	err = mlx5dv_devx_general_cmd(ctx, in, sizeof(in), out, sizeof(out));
	if (err) {
		DRV_LOG(ERR, "Set regexp params failed %d", err);
		return err;
	}
	return 0;
}

/**
 * Set the rules database umem
 *
 * Will set the database umem from which HW will read the rules.
 * Should be called only after engine stop.
 *
 * @param ctx
 *   ibv device handle.
 * @param engine_id
 *   The engine id to which the umem will be set.
 * @param db_ctx
 *   The struct which contains the umem, and offset inside the umem.
 *
 * @return
 *   0 on success, error otherwise
 */
int
mlx5_regex_database_set(struct ibv_context *ctx, int engine_id,
			const struct mlx5_database_ctx *db_ctx)
{
	return regex_database_set(ctx, engine_id, db_ctx, 0, 0);
}

/**
 * Stops the engine.
 *
 * Function will return when engine is idle.
 *
 * @param ctx
 *   ibv device handle.
 * @param engine_id
 *   The engine id to stop.
 *
 * @return
 *   0 on success, error otherwise
 */
int
mlx5_regex_engine_stop(struct ibv_context *ctx, int engine_id)
{
	return regex_database_set(ctx, engine_id, NULL, 1, 0);
}

/**
 * Starts the engine.
 *
 * Engine will start processing jobs.
 *
 * @param ctx
 *   ibv device handle.
 * @param engine_id
 *   The engine id to start.
 *
 * @return
 *   0 on success, error otherwise
 */
int
mlx5_regex_engine_go(struct ibv_context *ctx, int engine_id)
{
	return regex_database_set(ctx, engine_id, NULL, 0, 1);
}

/**
 * Query the engine parameters.
 *
 * Engine will start processing jobs.
 *
 * @param ctx
 *   ibv device handle.
 * @param engine_id
 *   The engine id to start.
 * @param db_ctx
 *   Output containing the umem id, and offset inside the umem.
 *
 * @return
 *   0 on success, error otherwise
 */
int
mlx5_regex_database_query(struct ibv_context *ctx, int engine_id,
			  struct mlx5_database_ctx *db_ctx)
{
	uint32_t out[DEVX_ST_SZ_DW(query_regexp_params_out)] = {};
	uint32_t in[DEVX_ST_SZ_DW(query_regexp_params_in)] = {};
	int err;

	DEVX_SET(query_regexp_params_in, in, opcode,
		 MLX5_CMD_QUERY_REGEX_PARAMS);
	DEVX_SET(query_regexp_params_in, in, engine_id, engine_id);

	err = mlx5dv_devx_general_cmd(ctx, in, sizeof(in), out, sizeof(out));
	if (err) {
		DRV_LOG(ERR, "Query regexp params failed %d", err);
		return err;
	}
	db_ctx->umem_id = DEVX_GET(query_regexp_params_out, out,
				   regexp_params.db_umem_id);
	db_ctx->offset = DEVX_GET(query_regexp_params_out, out,
				  regexp_params.db_umem_offset);
	return 0;
}


/**
 * Write to RXP registers.
 *
 * @param ctx
 *   ibv device handle
 * @param engine_id
 *   Chooses on which engine the register will be written..
 * @param addr
 *   Register address.
 * @param data
 *   Data to be written to the register.
 *
 * @return
 *   0 on success, error otherwise
 */
int
mlx5_regex_register_write(struct ibv_context *ctx, int engine_id,
			  uint32_t addr, uint32_t data)
{
	uint32_t out[DEVX_ST_SZ_DW(set_regexp_register_out)] = {};
	uint32_t in[DEVX_ST_SZ_DW(set_regexp_register_in)] = {};
	int err;

	DEVX_SET(set_regexp_register_in, in, opcode,
		 MLX5_CMD_SET_REGEX_REGISTERS);
	DEVX_SET(set_regexp_register_in, in, engine_id, engine_id);
	DEVX_SET(set_regexp_register_in, in, register_address, addr);
	DEVX_SET(set_regexp_register_in, in, register_data, data);

	err = mlx5dv_devx_general_cmd(ctx, in, sizeof(in), out, sizeof(out));
	if (err) {
		DRV_LOG(ERR, "Set regexp register failed %d", err);
		return err;
	}
	return 0;
}


/**
 * Read from RXP registers
 *
 * @param ctx
 *   ibv device handle
 * @param engine_id
 *   Chooses from which engine to read.
 * @param addr
 *   Register address.
 * @param data
 *   Output containing the pointer to the data..
 *
 * @return
 *   0 on success, error otherwise
 */
int
mlx5_regex_register_read(struct ibv_context *ctx, int engine_id,
			 uint32_t addr, uint32_t *data)
{
	uint32_t out[DEVX_ST_SZ_DW(query_regexp_register_out)] = {};
	uint32_t in[DEVX_ST_SZ_DW(query_regexp_register_in)] = {};
	int err;

	DEVX_SET(query_regexp_register_in, in, opcode,
		 MLX5_CMD_QUERY_REGEX_REGISTERS);
	DEVX_SET(query_regexp_register_in, in, engine_id, engine_id);
	DEVX_SET(query_regexp_register_in, in, register_address, addr);

	err = mlx5dv_devx_general_cmd(ctx, in, sizeof(in), out, sizeof(out));
	if (err) {
		DRV_LOG(ERR, "Query regexp register failed %d", err);
		return err;
	}
	*data = DEVX_GET(query_regexp_register_out, out, register_data);
	return 0;
}
