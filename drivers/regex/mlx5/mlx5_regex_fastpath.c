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
void mlx5_regex_set_ctrl_seg(void *seg,
			     uint8_t le, uint16_t subset_id0,
			     uint16_t subset_id1, uint16_t subset_id2,
			     uint16_t subset_id3, uint8_t ctrl)
{
	DEVX_SET(regexp_mmo_control, seg, le, le);
	DEVX_SET(regexp_mmo_control, seg, ctrl, ctrl);
	DEVX_SET(regexp_mmo_control, seg, subset_id_0, subset_id0);
	DEVX_SET(regexp_mmo_control, seg, subset_id_1, subset_id1);
	DEVX_SET(regexp_mmo_control, seg, subset_id_2, subset_id2);
	DEVX_SET(regexp_mmo_control, seg, subset_id_3, subset_id3);
}

static inline void
prep_one(struct mlx5_regex_sq *sq, struct rte_regex_ops *op,
	 struct mlx5_regex_job *job)
{
	memcpy(job->input,
		rte_pktmbuf_mtod(op->mbuf, void *),
		rte_pktmbuf_data_len(op->mbuf));

	size_t wqe_offset = (sq->pi % sq_size_get(sq)) * MLX5_SEND_WQE_BB;
	uint8_t *wqe = (uint8_t *)sq->wqe + wqe_offset;
	int ds = 4; /*  ctrl + meta + input + output */

	mlx5dv_set_ctrl_seg((struct mlx5_wqe_ctrl_seg *)wqe, sq->pi,
			    MLX5_OPCODE_MMO,
			    MLX5_OPC_MOD_MMO_REGEX, sq->obj->id,
			    0, ds, 0, 0);

	mlx5_regex_set_ctrl_seg(wqe+12, 0, op->group_id0, op->group_id1,
				op->group_id2,
				op->group_id3, 0);

	struct mlx5_wqe_data_seg *input_seg =
		(struct mlx5_wqe_data_seg *)(wqe+32);
	input_seg->byte_count = htobe32(rte_pktmbuf_data_len(op->mbuf));

	job->user_id = op->user_id;
	sq->db_pi = sq->pi;
	sq->pi = (sq->pi+1)%MAX_WQE_INDEX;
}

static inline void
send_doorbell(struct mlx5dv_devx_uar *uar, struct mlx5_regex_sq *sq)
{
	size_t wqe_offset = (sq->db_pi % sq_size_get(sq)) * MLX5_SEND_WQE_BB;
	uint8_t *wqe = (uint8_t *)sq->wqe + wqe_offset;
	((struct mlx5_wqe_ctrl_seg *)wqe)->fm_ce_se = MLX5_WQE_CTRL_CQ_UPDATE;
	uint64_t *doorbell_addr =
		(uint64_t *)((uint8_t *)uar->base_addr + 0x800);
	rte_cio_wmb();
	sq->dbr[MLX5_SND_DBR] = htobe32(sq->db_pi);
	rte_wmb();
	*doorbell_addr = *(volatile uint64_t *)wqe;
	rte_wmb();
}

static inline int
can_send(struct mlx5_regex_sq *sq) {
	return unlikely(sq->ci > sq->pi) ?
			MAX_WQE_INDEX + sq->pi - sq->ci < sq_size_get(sq) :
			sq->pi - sq->ci < sq_size_get(sq);
}

static inline uint32_t
job_id_get(uint32_t qid, size_t sq_size, size_t index) {
	return qid*sq_size + index%sq_size;
}

/**
 * DPDK callback for enqueue.
 *
 * @param dev
 *   Pointer to the regex dev structure.
 * @param qp_id
 *   The queue to enqueue the traffic to.
 * @param ops
 *   List of regex ops to enqueue.
 * @param nb_ops
 *   Number of ops in ops parameter.
 *
 * @return
 *   Number of packets successfully enqueued (<= pkts_n).
 */
uint16_t
mlx5_regexdev_enqueue(struct rte_regexdev *dev, uint16_t qp_id,
		      struct rte_regex_ops **ops, uint16_t nb_ops)
{
	struct mlx5_regex_priv *priv = dev->data->dev_private;
	struct mlx5_regex_qp *queue = &priv->qps[qp_id];
	struct mlx5_regex_sq *sq;
	size_t sqid, job_id, i = 0;

	while ((sqid = ffs(queue->free_sqs))) {
		sqid--; /* ffs returns 1 for bit 0 */
		sq = &queue->sqs[sqid];
		while (can_send(sq)) {
			job_id = job_id_get(sqid, sq_size_get(sq), sq->pi);
			prep_one(sq, ops[i], &queue->jobs[job_id]);
			i++;
			if (unlikely(i == nb_ops)) {
				send_doorbell(priv->uar, sq);
				goto out;
			}
		}
		queue->free_sqs &= ~(1 << sqid);
		send_doorbell(priv->uar, sq);
	}

out:
	queue->pi += i;
	return i;
}


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

			mlx5dv_set_metadata_seg((struct mlx5_wqe_metadata_seg *)(wqe + 16),
						0, queue->metadata->lkey,
						(uintptr_t)job->metadata);

			mlx5dv_set_data_seg((struct mlx5_wqe_data_seg *)(wqe + 32),
					    0, queue->inputs->lkey,
					    (uintptr_t)job->input);
			mlx5dv_set_data_seg((struct mlx5_wqe_data_seg *)(wqe + 48),
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

	qp->jobs = rte_calloc(__func__, qp->nb_desc, sizeof(*qp->jobs), sizeof(*qp->jobs));
	if (!qp->jobs)
		return -ENOMEM;
	err = setup_buffers(qp, priv->pd);
	if (err)
		return err;
	setup_sqs(qp);
	return 0;
}
