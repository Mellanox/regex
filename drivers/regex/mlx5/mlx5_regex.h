/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#ifndef MLX5_REGEX_H
#define MLX5_REGEX_H

struct mlx5_regex_qp_obj {
	uint32_t nb_desc; /* Number of desc for this object. */
};

struct mlx5_regex_qp {
	uint32_t flags; /* QP user flags. */
	uint32_t nb_desc; /* Total number of desc for thsi qp. */
	struct mlx5_regex_qp_obj *qp_obj; /* Pointer to qp_obj array. */
};

struct mlx5_regex_priv {
	TAILQ_ENTRY(mlx5_regex_priv) next;
	struct ibv_context *ctx; /* Device context. */
	struct rte_pci_device *pci_dev;
	struct rte_regexdev *regexdev; /* Pointer to the RegEx dev. */
	uint16_t nb_queues; /* Number of queues. */
	struct mlx5_regex_qp *qps; /* Pointer to the qp array. */
	uint16_t nb_max_matches; /* Max number of matches. */
};

/* mlx5_rxp.c */
int mlx5_regex_info_get(struct rte_regexdev *dev,
			struct rte_regexdev_info *info);
int mlx5_regex_configure(struct rte_regexdev *dev,
			 const struct rte_regexdev_config *cfg);

#endif /* MLX5_REGEX_H */
