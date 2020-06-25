/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */
#include <unistd.h>
#include <sys/mman.h>

#include <rte_malloc.h>
#include <rte_log.h>
#include <rte_errno.h>
#include <rte_bus_pci.h>
#include <rte_pci.h>
#include <rte_regexdev_driver.h>
#include <rte_mbuf.h>


#include <infiniband/mlx5dv.h>
#include <mlx5_glue.h>
#include <mlx5_common.h>
#include <mlx5_prm.h>
#include <strings.h>

#include "mlx5_regex_utils.h"
#include "mlx5_rxp.h"
#include "mlx5_regex.h"

/* Verbs header. */
/* ISO C doesn't support unnamed structs/unions, disabling -pedantic. */
#ifdef PEDANTIC
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#include <infiniband/mlx5dv.h>
#ifdef PEDANTIC
#pragma GCC diagnostic error "-Wpedantic"
#endif

#define MAX_WQE_INDEX 0xffff
#define MLX5_REGEX_METADATA_SIZE 64
#define MLX5_REGEX_MAX_INPUT (1<<14)
#define MLX5_REGEX_MAX_OUTPUT (1<<11)


static inline uint32_t
sq_size_get(struct mlx5_regex_sq *sq)
{
	return (1U << sq->log_nb_desc);
}
static inline uint32_t
cq_size_get(struct mlx5_regex_cq *cq)
{
	return (1U << cq->log_nb_desc);
}

struct mlx5_regex_job {
	uint64_t user_id;
	uint8_t *input;
	volatile uint8_t *output;
	volatile uint8_t *metadata;
} __rte_cached_aligned;


static MLX5DV_ALWAYS_INLINE
void mlx5dv_set_metadata_seg(struct mlx5_wqe_metadata_seg *seg,
			     uint32_t mmo_control_31_0, uint32_t lkey,
			     uintptr_t address)
{
	seg->mmo_control_31_0 = htobe32(mmo_control_31_0);
	seg->lkey       = htobe32(lkey);
	seg->addr       = htobe64(address);
}

static void
setup_sqs(struct mlx5_regex_qp *queue)
{
	size_t sqid, entry;
	uint32_t job_id;
	for (sqid = 0; sqid < queue->nb_obj; sqid++) {
		struct mlx5_regex_sq *sq = &queue->sqs[sqid];
		uint8_t *wqe = (uint8_t *)sq->wqe;
		for (entry = 0 ; entry < sq_size_get(sq); entry++) {
			job_id = sqid * sq_size_get(sq) + entry;
			struct mlx5_regex_job *job = &queue->jobs[job_id];

			mlx5dv_set_metadata_seg((struct mlx5_wqe_metadata_seg *)
						(wqe + 16),
						0, queue->metadata->lkey,
						(uintptr_t)job->metadata);
			mlx5dv_set_data_seg((struct mlx5_wqe_data_seg *)
					    (wqe + 32),
					    0, queue->inputs->lkey,
					    (uintptr_t)job->input);
			mlx5dv_set_data_seg((struct mlx5_wqe_data_seg *)
					    (wqe + 48),
					    1 << 11, queue->outputs->lkey,
					    (uintptr_t)job->output);
			wqe += 64;
		}
		queue->free_sqs |= 1 << sqid;
	}
}

static int
setup_buffers(struct mlx5_regex_qp *qp, struct ibv_pd *pd)
{
	int i, err;

	void *ptr = rte_calloc(__func__, qp->nb_desc,
			       MLX5_REGEX_METADATA_SIZE,
			       MLX5_REGEX_METADATA_SIZE);
	if (!ptr)
		return -ENOMEM;

	qp->metadata = mlx5_glue->reg_mr(pd, ptr,
					 MLX5_REGEX_METADATA_SIZE*qp->nb_desc,
					 IBV_ACCESS_LOCAL_WRITE);
	if (!qp->metadata) {
		rte_free(ptr);
		return -EINVAL;
	}
	ptr = rte_calloc(__func__, qp->nb_desc,
			 MLX5_REGEX_MAX_INPUT,
			 MLX5_REGEX_MAX_INPUT);

	if (!ptr) {
		err = -ENOMEM;
		goto err_input;
	}
	qp->inputs = mlx5_glue->reg_mr(pd, ptr,
				       MLX5_REGEX_MAX_INPUT*qp->nb_desc,
				       IBV_ACCESS_LOCAL_WRITE);
	if (!qp->inputs) {
		rte_free(ptr);
		err = -EINVAL;
		goto err_input;
	}

	ptr = rte_calloc(__func__, qp->nb_desc,
			 MLX5_REGEX_MAX_OUTPUT,
			 MLX5_REGEX_MAX_OUTPUT);
	if (!ptr) {
		err = -ENOMEM;
		goto err_output;
	}
	qp->outputs = mlx5_glue->reg_mr(pd, ptr,
					MLX5_REGEX_MAX_OUTPUT*qp->nb_desc,
					IBV_ACCESS_LOCAL_WRITE);
	if (!qp->outputs) {
		rte_free(ptr);
		err = -EINVAL;
		goto err_output;
	}

	/* distribute buffers to jobs */
	for (i = 0; i < qp->nb_desc; i++) {
		qp->jobs[i].input =
			(uint8_t *)qp->inputs->addr +
			(i%qp->nb_desc)*MLX5_REGEX_MAX_INPUT;
		qp->jobs[i].output =
			(uint8_t *)qp->outputs->addr +
			(i%qp->nb_desc)*MLX5_REGEX_MAX_OUTPUT;
		qp->jobs[i].metadata =
			(uint8_t *)qp->metadata->addr +
			(i%qp->nb_desc)*MLX5_REGEX_METADATA_SIZE;
	}
	return 0;

err_output:
	ptr = qp->inputs->addr;
	rte_free(ptr);
	mlx5_glue->dereg_mr(qp->inputs);
err_input:
	ptr = qp->metadata->addr;
	rte_free(ptr);
	mlx5_glue->dereg_mr(qp->metadata);
	return err;
}

int
mlx5_regexdev_setup_fastpath(struct mlx5_regex_priv *priv, uint32_t qp_id)
{
	struct mlx5_regex_qp *qp = &priv->qps[qp_id];
	int err;

	qp->jobs = rte_calloc(__func__, qp->nb_desc, sizeof(*qp->jobs),
			      sizeof(*qp->jobs));
	if (!qp->jobs)
		return -ENOMEM;
	err = setup_buffers(qp, priv->pd);
	if (err)
		return err;
	setup_sqs(qp);
	return 0;
}
