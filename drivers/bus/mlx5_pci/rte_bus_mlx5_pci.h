/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#ifndef _RTE_BUS_MLX5_PCI_H_
#define _RTE_BUS_MLX5_PCI_H_

/**
 * @file
 *
 * RTE Mellanox PCI Bus Interface
 * Mellanox ConnectX PCI device supports multiple class (net/vdpa/regex)
 * devices. This bus enables creating such multiple class of devices on a
 * single PCI device by allowing to bind multiple class specific device
 * driver to attach to mlx5_pci bus driver.
 *
 * -----------    ------------    -----------------
 * |   mlx5  |    |   mlx5   |    |   mlx5        |
 * | net pmd |    | vdpa pmd |    | new class pmd |
 * -----------    ------------    -----------------
 *      \              |                /
 *       \             |               /
 *        \       -------------       /
 *         \______|   mlx5    |_____ /
 *                |   pci bus |
 *                -------------
 *                     |
 *                 -----------
 *                 |   mlx5  |
 *                 | pci dev |
 *                 -----------
 *
 * - mlx5 pci bus driver binds to mlx5 PCI devices defined by PCI
 *   ID table of all related mlx5 PCI devices.
 * - mlx5 class driver such as net, vdpa, regex PMD defines its
 *   specific PCI ID table and mlx5 bus driver probes matching
 *   class drivers.
 * - mlx5 pci bus driver is cental place that validates supported
 *   class combinations.
 */

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <rte_pci.h>
#include <rte_bus_pci.h>

#include <mlx5_common.h>

/**
 * A structure describing a mlx5 pci driver.
 */
struct rte_mlx5_pci_driver {
	struct rte_pci_driver pci_driver;	/**< Inherit core pci driver. */
	uint32_t dev_class;	/**< Class of this driver, enum mlx5_class */
	TAILQ_ENTRY(rte_mlx5_pci_driver) next;
};

/**
 * Register a mlx5_pci device driver.
 *
 * @param driver
 *   A pointer to a rte_mlx5_pci_driver structure describing the driver
 *   to be registered.
 */
__rte_internal
void
rte_mlx5_pci_driver_register(struct rte_mlx5_pci_driver *driver);

#define RTE_PMD_REGISTER_MLX5_PCI(nm, drv) \
	static const char *mlx5_pci_drvinit_fn_ ## nm; \
	RTE_INIT(mlx5_pci_drvinit_fn_ ##drv) \
	{ \
		(drv).driver.name = RTE_STR(nm); \
		rte_mlx5_pci_driver_register(&drv); \
	} \
	RTE_PMD_EXPORT_NAME(nm, __COUNTER__)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _RTE_BUS_MLX5_PCI_H_ */
