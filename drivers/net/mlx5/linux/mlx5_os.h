/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2015 6WIND S.A.
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#ifndef RTE_PMD_MLX5_OS_H_
#define RTE_PMD_MLX5_OS_H_

/* verb enumerations translations to local enums. */
enum {
	DEV_SYSFS_NAME_MAX = IBV_SYSFS_NAME_MAX,
	DEV_SYSFS_PATH_MAX = IBV_SYSFS_PATH_MAX
};

#define PCI_DRV_FLAGS  (RTE_PCI_DRV_INTR_LSC | \
			RTE_PCI_DRV_INTR_RMV | \
			RTE_PCI_DRV_PROBE_AGAIN)
#endif /* RTE_PMD_MLX5_OS_H_ */
