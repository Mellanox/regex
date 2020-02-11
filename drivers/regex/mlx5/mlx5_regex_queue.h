/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#ifndef MLX5_REGEX_QUEUE_H_
#define MLX5_REGEX_QUEUE_H_

int mlx5_regex_enqueue(struct rte_regex_dev *dev, uint16_t q_id,
		       struct rte_regex_ops **ops, uint16_t nb_ops);
int mlx5_regex_dequeue(struct rte_regex_dev *dev, uint16_t q_id,
		       struct rte_regex_ops **ops, uint16_t nb_ops);

#endif /* MLX5_REGEX_QUEUE_H_ */
