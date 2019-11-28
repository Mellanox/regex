/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */

#include "mlx5_common.h"


int mlx5_common_logtype;


RTE_INIT(rte_mlx5_common_pmd_init)
{
	/* Initialize driver log type. */
	mlx5_common_logtype = rte_log_register("pmd.common.mlx5");
	if (mlx5_common_logtype >= 0)
		rte_log_set_level(mlx5_common_logtype, RTE_LOG_NOTICE);
}
