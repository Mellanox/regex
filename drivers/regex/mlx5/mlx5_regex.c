/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */

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
