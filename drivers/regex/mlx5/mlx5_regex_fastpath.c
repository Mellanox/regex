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
 	int i;
	struct rte_regex_ops *op; 
	int sent = 0;
	struct mlx5_regex_job *job;
	uint32_t job_id;
	job_id = (queue->pi)%MLX5_REGEX_MAX_JOBS;
	struct mlx5_regex_sq *sq = &queue->ctx->qps[mlx5_regex_job2queue(job_id)];
	//printf(" pi = %d, ci = %d\n",  queue->pi, queue->ci);
	for (i = 0; i < nb_ops; i++) {
		if (unlikely((queue->pi - queue->ci) >= MLX5_REGEX_MAX_JOBS))
			break;
		op = ops[i];
		job = &queue->jobs[job_id];

		prep_one(sq, op, job);
		
		sq->db_pi = sq->pi;
		sq->pi = (sq->pi+1)%MAX_WQE_INDEX;

		queue->jobs[queue->pi % MLX5_REGEX_MAX_JOBS].user_id =
			op->user_id; 
		queue->jobs[queue->pi % MLX5_REGEX_MAX_JOBS].used = 1;
		sent++;
		queue->pi++;

		job_id = (queue->pi)%MLX5_REGEX_MAX_JOBS;
		
		struct mlx5_regex_sq *last_sq = sq;
		sq = &queue->ctx->qps[mlx5_regex_job2queue(job_id)];
		if (unlikely(sq!=last_sq) || (i == (nb_ops -1)) || (last_sq->db_pi%32)) {
			size_t wqe_offset = (last_sq->db_pi % SQ_SIZE) * MLX5_SEND_WQE_BB;
			uint8_t *wqe = (uint8_t *)last_sq->wq_buff + wqe_offset;
			((struct mlx5_wqe_ctrl_seg *)wqe)->fm_ce_se = MLX5_WQE_CTRL_CQ_UPDATE;
			uint64_t *doorbell_addr = (uint64_t *)((uint8_t *)queue->ctx->uar->base_addr + 0x800);
			rte_cio_wmb();
			last_sq->qp_dbr[MLX5_SND_DBR] = htobe32(last_sq->db_pi);
			rte_wmb();
			*doorbell_addr = *(volatile uint64_t *)wqe;
			rte_wmb();
			queue->ctx->uuar = !queue->ctx->uuar;
		}
	}

	return sent;
}

#define MLX5_REGEX_RESP_SZ 8

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
 	int i;
	struct rte_regex_ops *op; 
	int rec = 0;
	int j;
	int offset;

	for (i = 0; i < nb_ops;) {
		uint32_t job_id = (queue->ci)%MLX5_REGEX_MAX_JOBS;
		int ret = 0;
		
		ret = mlx5_regex_poll(queue->ctx, mlx5_regex_job2queue(job_id));

		if (ret < 0) {
			//printf("CQE error\n");
			break;
		}

		if (ret == 0)
			break;
		int c = 0;
		for (c = 0; c < ret; c++) {
			uint32_t job_id = (queue->ci)%MLX5_REGEX_MAX_JOBS;
			struct mlx5_regex_job *job = &queue->jobs[job_id];
			op = ops[i];
			op->user_id = job->user_id;
			//printf("jobid = %d, job->user_id)=%ld size=%ld\n", job_id, job->user_id, sizeof(struct rxp_response_desc));
			op->nb_matches = DEVX_GET(regexp_metadata, job->metadata + 32, match_count);
			op->nb_actual_matches = DEVX_GET(regexp_metadata, job->metadata +32,
							detected_match_count);
			//print_raw(job->output, 1);
			for (j = 0; j < op->nb_matches; j++) {
				offset = MLX5_REGEX_RESP_SZ * j;
				op->matches[j].group_id = DEVX_GET(regexp_match_tuple, 
								(job->output + offset), rule_id);
				op->matches[j].offset = DEVX_GET(regexp_match_tuple, 
								(job->output +  offset), start_ptr);
				op->matches[j].len = DEVX_GET(regexp_match_tuple, 
								(job->output +  offset), length);
			}
			job->used = 0;
			rec++;
			queue->ci++;
			
			i++;
			if (mlx5_regex_job2queue(job_id) != mlx5_regex_job2queue((queue->ci)%MLX5_REGEX_MAX_JOBS))
				break;
		}
	}
//exit:
//	for (; queue->ci < queue->pi; queue->ci++) {
//		/*if (queue->jobs[res->header.job_id].used == 1)
//			break;*/
//	}
	//printf("rec = %d\n", rec);
	return rec;
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
 	int qid, entry;
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
	}
	return 0;
}
