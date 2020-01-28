/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2020 Mellanox Corporation
 */

#ifndef _RTE_REGEXDEV_DRIVER_H_
#define _RTE_REGEXDEV_DRIVER_H_

/**
 * @file
 *
 * RTE RegEx Device PMD API
 *
 * These APIs for the use from regex drivers, user applications shouldn't
 * use them.
 */

#include <rte_regexdev.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @internal
 * Register a new regexdev slot for a regex device and returns the id
 * to that slot for the driver to use.
 *
 * @param dev
 *   regex device structure..
 *
 * @return
 *   Slot in the rte_regex_devices array for a new device in case of success,
 *   negative errno otherwise.
 */
int rte_regex_dev_register(struct rte_regex_dev *dev);

/**
 * @internal
 * Unregister the specified regexdev port.
 *
 * @param dev
 *   Device to be released.
 *
 * @return
 *   0 on success, Negative errno value on error.
 */
int rte_regex_dev_unregister(struct rte_regex_dev *dev);

#ifdef __cplusplus
}
#endif

#endif /* _RTE_REGEXDEV_DRIVER_H_ */
