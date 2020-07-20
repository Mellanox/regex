/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#include <rte_errno.h>
#include <rte_log.h>

#include <mlx5_glue.h>
#include <mlx5_devx_cmds.h>
#include <mlx5_prm.h>

#include "mlx5_regex.h"
#include "mlx5_regex_utils.h"

int
mlx5_devx_regex_register_write(struct ibv_context *ctx, int engine_id,
			       uint32_t addr, uint32_t data)
{
	uint32_t out[MLX5_ST_SZ_DW(set_regexp_register_out)] = {0};
	uint32_t in[MLX5_ST_SZ_DW(set_regexp_register_in)] = {0};
	int ret;

	MLX5_SET(set_regexp_register_in, in, opcode,
		 MLX5_CMD_SET_REGEX_REGISTERS);
	MLX5_SET(set_regexp_register_in, in, engine_id, engine_id);
	MLX5_SET(set_regexp_register_in, in, register_address, addr);
	MLX5_SET(set_regexp_register_in, in, register_data, data);

	ret = mlx5_glue->devx_general_cmd(ctx, in, sizeof(in), out,
					  sizeof(out));
	if (ret) {
		DRV_LOG(ERR, "Set regexp register failed %d", ret);
		rte_errno = errno;
		return -errno;
	}
	return 0;
}

int
mlx5_devx_regex_register_read(struct ibv_context *ctx, int engine_id,
			      uint32_t addr, uint32_t *data)
{
	uint32_t out[MLX5_ST_SZ_DW(query_regexp_register_out)] = {0};
	uint32_t in[MLX5_ST_SZ_DW(query_regexp_register_in)] = {0};
	int ret;

	MLX5_SET(query_regexp_register_in, in, opcode,
		 MLX5_CMD_QUERY_REGEX_REGISTERS);
	MLX5_SET(query_regexp_register_in, in, engine_id, engine_id);
	MLX5_SET(query_regexp_register_in, in, register_address, addr);

	ret = mlx5_glue->devx_general_cmd(ctx, in, sizeof(in), out,
					  sizeof(out));
	if (ret) {
		DRV_LOG(ERR, "Query regexp register failed %d", ret);
		rte_errno = errno;
		return -errno;
	}
	*data = MLX5_GET(query_regexp_register_out, out, register_data);
	return 0;
}
