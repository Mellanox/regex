/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#ifndef MLX5_REGEX_H
#define MLX5_REGEX_H

#include "mlx5_rxp.h"

struct mlx5_regex_qp_obj {
	uint32_t nb_desc; /* Number of desc for this object. */
};

struct mlx5_regex_qp {
	uint32_t flags; /* QP user flags. */
	uint32_t nb_desc; /* Total number of desc for thsi qp. */
	struct mlx5_regex_qp_obj *qp_obj; /* Pointer to qp_obj array. */
};

struct mlx5_regex_db {
	void *ptr; /* Pointer to the db memory. */
	uint32_t len; /* The memory len. */
	bool active; /* Active flag. */
	uint8_t db_assigned_to_eng_num;
	/**< To which engine the db is connected. */
	struct mlx5_regex_umem umem;
	/**< The umem struct. */
};

struct mlx5_regex_priv {
	TAILQ_ENTRY(mlx5_regex_priv) next;
	struct ibv_context *ctx; /* Device context. */
	struct rte_pci_device *pci_dev;
	struct rte_regexdev *regexdev; /* Pointer to the RegEx dev. */
	uint16_t nb_queues; /* Number of queues. */
	struct mlx5_regex_qp *qps; /* Pointer to the qp array. */
	uint16_t nb_max_matches; /* Max number of matches. */
	//ORI check, where best to add this mode? :
	enum mlx5_rxp_program_mode prog_mode;
	struct mlx5_regex_db db[MLX5_RXP_MAX_ENGINES +
				MLX5_RXP_SHADOW_EM_COUNT];
};

/* mlx5_rxp.c */
int mlx5_regex_info_get(struct rte_regexdev *dev,
			struct rte_regexdev_info *info);
int mlx5_regex_configure(struct rte_regexdev *dev,
			 const struct rte_regexdev_config *cfg);
int mlx5_regex_rules_db_import(struct rte_regexdev *dev,
		     const struct rte_regexdev_config *cfg);
#endif /* MLX5_REGEX_H */
