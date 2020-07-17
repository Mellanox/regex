/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#include "rte_bus_mlx5_pci.h"

static TAILQ_HEAD(mlx5_pci_bus_drv_head, rte_mlx5_pci_driver) drv_list =
				TAILQ_HEAD_INITIALIZER(drv_list);

void
rte_mlx5_pci_driver_register(struct rte_mlx5_pci_driver *driver)
{
	TAILQ_INSERT_TAIL(&drv_list, driver, next);
}
