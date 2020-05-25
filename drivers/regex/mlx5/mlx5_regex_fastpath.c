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

#define MLX5_REGEX_RESP_SZ 8
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
	uint16_t subset[4];
	uint32_t job_id;
	int work_id;
	//printf(" pi = %d, ci = %d\n",  queue->pi, queue->ci);
	for (i = 0; i < nb_ops; i++) {
		if (unlikely((queue->pi - queue->ci) >= MLX5_REGEX_MAX_JOBS))
			return sent;
		op = ops[i];
		job_id = (queue->pi)%MLX5_REGEX_MAX_JOBS;
		job = &queue->jobs[job_id];

		memcpy(job->input,
			    (*op->bufs)[0]->buf_addr,
			    (*op->bufs)[0]->buf_size);

		subset[0] = op->group_id0;
		subset[1] = op->group_id1;
		subset[2] = op->group_id2;
		subset[3] = op->group_id3;

		mlx5_regex_set_ctrl_seg(&job->regex_ctrl, 0,
                                subset, 0);

		mlx5dv_set_data_seg(&job->input_seg, (*op->bufs)[0]->buf_size,
			    		    mlx5_regex_get_lkey(queue->inputs.buff),
							(uintptr_t)job->input);

		work_id =
		mlx5_regex_prep_work(queue->ctx, &job->regex_ctrl,
				 (volatile uint8_t*)job->metadata,
				 mlx5_regex_get_lkey(queue->metadata.buff),
				 &job->input_seg,
                	         &job->output_seg,
				 mlx5_regex_job2queue(job_id), 1);
		mlx5_regex_send_work(queue->ctx, mlx5_regex_job2queue(job_id));

		if (work_id < 0) {
			//printf("submit job failed err = %d\n", ret);
			return sent;
		}

		if ((uint32_t)work_id%MLX5_REGEX_SQ_SIZE != mlx5_regex_job2entry(job_id)) {
			printf("Job id mismatch job_id=%dwork_id %d, entry %d\n", queue->pi, work_id%MLX5_REGEX_SQ_SIZE, mlx5_regex_job2entry(job_id));
			exit(-1);
		}
		queue->jobs[queue->pi % MLX5_REGEX_MAX_JOBS].user_id =
			op->user_id; 
		queue->jobs[queue->pi % MLX5_REGEX_MAX_JOBS].used = 1;
		sent++;
		queue->pi++;
	}

	return sent;
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
 	int i;
	struct rte_regex_ops *op; 
	int rec = 0;
	int j;
	int offset;

	for (i = 0; i < nb_ops; i++) {
		uint32_t job_id = (queue->ci)%MLX5_REGEX_MAX_JOBS;
		struct mlx5_regex_job *job = &queue->jobs[job_id];
		int ret = 0;
		
		ret = mlx5_regex_poll(queue->ctx, mlx5_regex_job2queue(job_id));

		if (ret < 0) {
			printf("CQE error\n");
			break;
		}

		if (ret == 0)
			break;

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
	}
//exit:
//	for (; queue->ci < queue->pi; queue->ci++) {
//		/*if (queue->jobs[res->header.job_id].used == 1)
//			break;*/
//	}
	//printf("rec = %d\n", rec);
	return rec;
}
