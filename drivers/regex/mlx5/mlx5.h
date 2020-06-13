#ifndef __MLX5_H__
#define __MLX5_H__

#include <rte_regexdev_driver.h>
#include <rte_rwlock.h>

#include "mlx5_regex_mr.h"
#include "mlx5_regex.h"

#include "rxp-api.h"

#define MLX5_REGEX_MAX_QUEUES 32

#define MLX5_REGEX_NUM_SQS 16
#define MLX5_REGEX_SQ_SIZE SQ_SIZE
#define MLX5_REGEX_MAX_JOBS (MLX5_REGEX_NUM_SQS*MLX5_REGEX_SQ_SIZE)
#define MLX5_REGEX_MAX_INPUT_OUTPUT (1 << 14)
#define MLX5_REGEX_METADATA_SIZE 64


struct mlx5_regex_buff_ctx {
	void *ptr;
	struct mlx5_regex_buff *buff;
};

struct mlx5_regex_job {
	uint64_t user_id;
	uint32_t job_id;
	int used;
	uint8_t* input;
	volatile uint8_t* output;
	volatile uint8_t* metadata;
	struct mlx5_wqe_data_seg input_seg;
	struct mlx5_wqe_data_seg output_seg;
	struct mlx5_wqe_data_seg metadata_seg;
	struct mlx5_regex_wqe_ctrl_seg regex_ctrl;
} __rte_cached_aligned;

static inline uint32_t
mlx5_regex_job_id_get(uint32_t qid, uint32_t entry) {
	return qid*MLX5_REGEX_SQ_SIZE + entry;
}


static inline uint32_t
mlx5_regex_job2queue(uint32_t job_id) {
	return (job_id - (job_id%MLX5_REGEX_SQ_SIZE))/MLX5_REGEX_SQ_SIZE;
}

static inline uint32_t
mlx5_regex_job2entry(uint32_t job_id) {
	return job_id%MLX5_REGEX_SQ_SIZE;
}

static inline uint32_t
mlx5_regex_buffer_offset_get(uint32_t job_id)
{
	return (job_id%MLX5_REGEX_MAX_JOBS)*MLX5_REGEX_MAX_INPUT_OUTPUT;
}

static inline uint32_t
mlx5_regex_metadata_offset_get(uint32_t job_id)
{
	return (job_id%MLX5_REGEX_MAX_JOBS)*MLX5_REGEX_METADATA_SIZE;
}

struct mlx5_regex_queues {
	int handle;
	uint32_t job_cnt;
	uint32_t pi;
	uint32_t ci;
	uint32_t free_sqs;
	struct mlx5_regex_ctx *ctx;
	struct rxp_response_batch resp_ctx;
	struct mlx5_regex_job jobs[MLX5_REGEX_MAX_JOBS];
	struct mlx5_regex_buff_ctx metadata;
	struct mlx5_regex_buff_ctx inputs;
	struct mlx5_regex_buff_ctx outputs;
};

struct mlx5_regex_priv {
	struct rte_regex_dev regex_dev;
	struct ibv_context *ctx; /* Device context. */
	struct ibv_pd *pd;
	uint32_t pdn;
	uint32_t eqn;
	uint16_t nb_queues;
	struct mlx5_regex_queues queues[MLX5_REGEX_MAX_QUEUES];
	//struct mlx5dv_devx_uar *uar; /* used inside RXP code. */
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
			   int
mlx5_regex_dev_fastpath_prep(struct mlx5_regex_priv *priv, uint16_t qp_id);

#endif
