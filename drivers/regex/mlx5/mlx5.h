#ifndef __MLX5_H__
#define __MLX5_H__

#include <rte_regexdev_driver.h>
#include <rte_rwlock.h>

#include "mlx5_regex_mr.h"
#include "rxp-api.h"

#define MLX5_REGEX_MAX_QUEUES 32

#define MLX5_REGEX_MAX_JOBS 64

struct mlx5_database_ctx {
	uint32_t umem_id;
	uint64_t offset;
};

struct mlx5_regex_job {
	uint64_t user_id;
	int job_id;
	int used;
} __rte_cached_aligned;

struct mlx5_regex_queues {
	int handle;
	uint32_t pi;
	uint32_t ci;
	struct rxp_response_batch resp_ctx;
	struct mlx5_regex_job jobs[MLX5_REGEX_MAX_JOBS];
};

struct mlx5_regex_priv {
	struct rte_regex_dev regex_dev;
	struct ibv_context *ctx; /* Device context. */
	struct ibv_pd *pd;
	uint32_t pdn;
	uint32_t eqn;
	uint16_t nb_queues;
	struct mlx5_regex_queues queues[MLX5_REGEX_MAX_QUEUES];
	struct mlx5dv_devx_uar *uar;
	struct mlx5_regex_db *db_desc;
	int num_db_desc;
	TAILQ_ENTRY(mlx5_regex_priv) next;
	struct rte_pci_device *pci_dev;
	struct {
		uint32_t dev_gen; /* Generation number to flush local caches. */
		rte_rwlock_t rwlock; /* MR Lock. */
		struct mlx5_mr_btree cache; /* Global MR cache table. */
		struct mlx5_mr_list mr_list; /* Registered MR list. */
		struct mlx5_mr_list mr_free_list; /* Freed MR list. */
	} mr;
};

int mlx5_regex_dev_enqueue(struct rte_regex_dev *dev, uint16_t qp_id,
			   struct rte_regex_ops **ops, uint16_t nb_ops);
int mlx5_regex_dev_dequeue(struct rte_regex_dev *dev, uint16_t qp_id,
			   struct rte_regex_ops **ops, uint16_t nb_ops);
#endif
