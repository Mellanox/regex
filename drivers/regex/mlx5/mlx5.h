/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */
#ifndef __MLX5_H__
#define __MLX5_H__

#include <rte_regexdev_driver.h>

struct mlx5_regex_priv {
	struct rte_regex_dev regex_dev;
	struct ibv_context *ctx; /* Device context. */

	TAILQ_ENTRY(mlx5_regex_priv) next;
	struct rte_pci_device *pci_dev;
};

#endif
