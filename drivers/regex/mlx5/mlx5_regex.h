/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */

#ifndef MLX5_REGEX_H
#define MLX5_REGEX_H

#include <mlx5_devx_cmds.h>

int mlx5_regex_is_supported(struct ibv_context *ibv_ctx);

#endif /* MLX5_REGEX_H */
