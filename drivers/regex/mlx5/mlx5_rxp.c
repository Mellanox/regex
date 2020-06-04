/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#include <rte_log.h>
#include <rte_errno.h>
#include <rte_regexdev.h>
#include <rte_regexdev_core.h>
#include <rte_regexdev_driver.h>

#include "mlx5_regex.h"

#define MLX5_REGEX_MAX_MATCHES 255
#define MLX5_REGEX_MAX_PAYLOAD_SIZE UINT16_MAX
#define MLX5_REGEX_MAX_RULES_PER_GROUP UINT16_MAX
#define MLX5_REGEX_MAX_GROUPS UINT16_MAX

/**
 * DPDK callback for reading device info.
 *
 * @param dev
 *   Pointer to RegEx device structure.
 * @param[out] info
 *   Pointer to the regexdev info structure to be filled with the contextual
 *   information of the device.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
int
mlx5_regex_info_get(struct rte_regexdev *dev __rte_unused,
		  struct rte_regexdev_info *info)
{
	info->max_matches = MLX5_REGEX_MAX_MATCHES;
	info->max_payload_size = MLX5_REGEX_MAX_PAYLOAD_SIZE;
	info->max_rules_per_group = MLX5_REGEX_MAX_RULES_PER_GROUP;
	info->max_groups = MLX5_REGEX_MAX_GROUPS;
	info->regexdev_capa = RTE_REGEXDEV_SUPP_PCRE_GREEDY_F;
	info->rule_flags = 0;
	return 0;
}
