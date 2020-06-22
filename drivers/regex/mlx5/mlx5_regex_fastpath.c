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

#include <infiniband/mlx5dv.h>
#include <mlx5_glue.h>
#include <mlx5_common.h>
#include <mlx5_prm.h>
#include <strings.h>

#include "mlx5.h"
#include "mlx5_regex.h"
#include "mlx5_regex_utils.h"
#include "rxp-csrs.h"
#include "rxp-api.h"

static inline void
prep_one(struct mlx5_regex_sq *sq, struct rte_regex_ops *op,
		 struct mlx5_regex_job *job)
{
	memcpy(job->input,
			(*op->bufs)[0]->buf_addr,
			(*op->bufs)[0]->buf_size);

	size_t wqe_offset = (sq->pi % SQ_SIZE) * MLX5_SEND_WQE_BB;
	uint8_t *wqe = (uint8_t *)sq->wq_buff + wqe_offset;
	
	mlx5dv_set_ctrl_seg((struct mlx5_wqe_ctrl_seg *)wqe, sq->pi, MLX5_OPCODE_MMO,
			MLX5_OPC_MOD_MMO_REGEX, sq->qpn,
			0, 4, 0, 0);
	
	mlx5_regex_set_ctrl_seg(wqe+12, 0, op->group_id0, op->group_id1,
						    op->group_id2, op->group_id3, 0);

	struct mlx5_wqe_data_seg *input_seg = (struct mlx5_wqe_data_seg *)(wqe+32);
	input_seg->byte_count = htobe32((*op->bufs)[0]->buf_size);

	job->user_id = op->user_id;
	sq->db_pi = sq->pi;
	sq->pi = (sq->pi+1)%MAX_WQE_INDEX;
}

static inline void
send_doorbell(struct mlx5_regex_sq *sq, struct mlx5_regex_queues *queue) {
	size_t wqe_offset = (sq->db_pi % SQ_SIZE) * MLX5_SEND_WQE_BB;
	uint8_t *wqe = (uint8_t *)sq->wq_buff + wqe_offset;
	((struct mlx5_wqe_ctrl_seg *)wqe)->fm_ce_se = MLX5_WQE_CTRL_CQ_UPDATE;
	uint64_t *doorbell_addr = (uint64_t *)((uint8_t *)queue->ctx->uar->base_addr + 0x800);
	rte_cio_wmb();
	sq->qp_dbr[MLX5_SND_DBR] = htobe32(sq->db_pi);
	rte_wmb();
	*doorbell_addr = *(volatile uint64_t *)wqe;
	rte_wmb();
}

static inline int
can_send(struct mlx5_regex_sq *sq) {
	return unlikely(sq->ci > sq->pi) ?
			MAX_WQE_INDEX + sq->pi - sq->ci < SQ_SIZE: 
			sq->pi - sq->ci < SQ_SIZE;
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

int
mlx5_regex_dev_enqueue(struct rte_regex_dev *dev, uint16_t qp_id,
		       struct rte_regex_ops **ops, uint16_t nb_ops)
{
	struct mlx5_regex_priv *priv = container_of(dev,
						    struct mlx5_regex_priv,
						    regex_dev);
	struct mlx5_regex_queues *queue = &priv->queues[qp_id];
	struct mlx5_regex_sq *sq;
	size_t qid, job_id, i = 0;

	while ((qid = ffs(queue->free_sqs))) {
		qid--; //ffs returns 1 for bit 0
		sq = &queue->ctx->qps[qid];
		while (can_send(sq)) {
			job_id = mlx5_regex_job_id_get(qid, sq->pi%SQ_SIZE);
			prep_one(sq, ops[i], &queue->jobs[job_id]);
			i++;
			if (unlikely(i == nb_ops)) {
				send_doorbell(sq, queue);
				goto out;
			}
		}
		queue->free_sqs &= ~(1 << qid);
		send_doorbell(sq, queue);
	}

out:
	queue->pi += i;
	return i;
}

#define MLX5_REGEX_RESP_SZ 8
static inline void
extract_result(struct rte_regex_ops *op, struct mlx5_regex_job *job)
{
	size_t j, offset;
	op->user_id = job->user_id;
	op->nb_matches = DEVX_GET(regexp_metadata, job->metadata + 32, match_count);
	op->nb_actual_matches = DEVX_GET(regexp_metadata, job->metadata +32,
									 detected_match_count);
	for (j = 0; j < op->nb_matches; j++) {
		offset = MLX5_REGEX_RESP_SZ * j;
		op->matches[j].group_id =
			DEVX_GET(regexp_match_tuple, (job->output + offset), rule_id);
		op->matches[j].offset =
			DEVX_GET(regexp_match_tuple, (job->output +  offset), start_ptr);
		op->matches[j].len =
			DEVX_GET(regexp_match_tuple, (job->output +  offset), length);
	}
}

static inline volatile struct mlx5_cqe*
poll_one(struct mlx5_regex_cq* cq)
{
	volatile struct mlx5_cqe *cqe;
	size_t next_cqe_offset;

	next_cqe_offset =  (cq->ci % cq->cq_size) * sizeof(*cqe);
	cqe = (volatile struct mlx5_cqe *)(cq->cq_buff + next_cqe_offset);
    rte_cio_wmb();

	int ret = check_cqe(cqe, cq->cq_size, cq->ci);

	if (unlikely (ret == MLX5_CQE_STATUS_ERR)) {
		DRV_LOG(ERR, "Completion with error on qp 0x%x",  0);
		exit(-1);
	}

	if (unlikely (ret != MLX5_CQE_STATUS_SW_OWN))
		return NULL;
		
	return cqe;

}


/**
 * DPDK callback for dequeue.
 *
 * @param dev
 *   Pointer to the regex dev structure.
 * @param qp_id
 *   The queue to enqueue the traffic to.
 * @param ops
 *   List of regex ops to dequeue.
 * @param nb_ops
 *   Number of ops in ops parameter.
 *
 * @return
 *   Number of packets successfully dequeued (<= pkts_n).
 */
int
mlx5_regex_dev_dequeue(struct rte_regex_dev *dev, uint16_t qp_id,
		       struct rte_regex_ops **ops, uint16_t nb_ops)
{
	struct mlx5_regex_priv *priv = container_of(dev,
						    struct mlx5_regex_priv,
						    regex_dev);
	struct mlx5_regex_queues *queue = &priv->queues[qp_id];
	struct mlx5_regex_cq *cq = &queue->ctx->cq;
	volatile struct mlx5_cqe *cqe;
 	size_t i = 0;

	while ((cqe = poll_one(cq))) {
		uint16_t wq_counter = (be16toh(cqe->wqe_counter) + 1)%MAX_WQE_INDEX;
		size_t qid = cqe->rsvd3[2];
		struct mlx5_regex_sq *sq = &queue->ctx->qps[qid];
		while (sq->ci != wq_counter) {
			if (unlikely(i == nb_ops)) {
				/* Return without updating cq->ci */
				goto out;
			}
			uint32_t job_id = mlx5_regex_job_id_get(qid, sq->ci%SQ_SIZE);
			extract_result(ops[i], &queue->jobs[job_id]);
			sq->ci = (sq->ci+1)%MAX_WQE_INDEX;
			i++;
		}
		cq->ci = (cq->ci + 1) & 0xffffff;
		asm volatile("" ::: "memory");
		cq->cq_dbr[0] = htobe32(cq->ci);
		queue->free_sqs |= (1 << qid);
	}

out:
	queue->ci += i;
	return i;
}

struct regex_wqe {
	struct mlx5_wqe_ctrl_seg ctrl;

	struct mlx5_wqe_data_seg input;

};

static MLX5DV_ALWAYS_INLINE
void mlx5dv_set_metadata_seg(struct mlx5_wqe_metadata_seg *seg,
			     uint32_t mmo_control_31_0, uint32_t lkey,
			     uintptr_t address)
{
	seg->mmo_control_31_0 = htobe32(mmo_control_31_0);
	seg->lkey       = htobe32(lkey);
	seg->addr       = htobe64(address);
}

int
mlx5_regex_dev_fastpath_prep(struct mlx5_regex_priv *priv, uint16_t qp_id) {
	struct mlx5_regex_queues *queue = &priv->queues[qp_id];
 	size_t qid, entry;
	uint32_t job_id;
	for (qid = 0; qid < MLX5_REGEX_NUM_SQS; qid++) {
		uint8_t* wqe = (uint8_t*)queue->ctx->qps[qid].wq_buff;
		uint32_t qpn = queue->ctx->qps[qid].qpn;
		for (entry = 0 ; entry < SQ_SIZE; entry++) {
			job_id = qid*SQ_SIZE + entry;
			struct mlx5_regex_job *job = &queue->jobs[job_id];
			int ds = 4; //ctrl + meta + input + output

			mlx5dv_set_ctrl_seg((struct mlx5_wqe_ctrl_seg *)wqe, 0, MLX5_OPCODE_MMO,
			    MLX5_OPC_MOD_MMO_REGEX, qpn, 0, ds, 0,
			    0);

			mlx5dv_set_metadata_seg((struct mlx5_wqe_metadata_seg *)(wqe + 16), 0,
									mlx5_regex_get_lkey(queue->metadata.buff),
									(uintptr_t)job->metadata);

			mlx5dv_set_data_seg((struct mlx5_wqe_data_seg *)(wqe+32), 0,
			    		    mlx5_regex_get_lkey(queue->inputs.buff),
							(uintptr_t)job->input);
			mlx5dv_set_data_seg((struct mlx5_wqe_data_seg *)(wqe+48), 1 << 11,
			    		    mlx5_regex_get_lkey(queue->outputs.buff),
							(uintptr_t)job->output);
			wqe += 64;

		}
		queue->free_sqs |= 1 << qid;
	}
	return 0;
}
