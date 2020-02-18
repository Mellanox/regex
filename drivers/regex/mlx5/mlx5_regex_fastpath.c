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

	for (i = 0; i < nb_ops; i++) {
		if ((queue->pi - queue->ci) >= MLX5_REGEX_MAX_JOBS)
			return sent;
		op = ops[i];
		rxp_submit_job(queue->handle,
			       queue->pi % MLX5_REGEX_MAX_JOBS,
			       (*op->bufs)[0]->buf_addr,
			       (*op->bufs)[0]->buf_size,
			       op->group_id0, op->group_id1, op->group_id2,
			       op->group_id3, false, false);
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
	bool rx_ready;
	bool tx_ready;
	struct rxp_response *res;
	int j;
	int cnt = 0;

	for (i = 0; i < nb_ops; i++) {
		if ((queue->pi - queue->ci) == 0)
			return rec;
		op = ops[i];
		if (cnt <= 0) {
			rxp_queue_status(queue->handle, &rx_ready, &tx_ready);
			if (!rx_ready)
				goto exit;
			cnt = rxp_read_response_batch(queue->handle,
						      &queue->resp_ctx);
		}
		res = rxp_next_response(&queue->resp_ctx);
		cnt--;
		if (res == NULL)
			continue;
		op->user_id = queue->jobs[res->header.job_id].user_id;
		op->nb_matches = res->header.match_count;
		op->nb_actual_matches = res->header.detected_match_count;
		for (j = 0; j < op->nb_matches; j++) {
			op->matches[j].rule_id = res->matches[j].rule_id;
			op->matches[j].offset = res->matches[j].start_ptr;
			op->matches[j].len = res->matches[j].length;
		}
		queue->jobs[res->header.job_id].used = 0;
		rec++;
		queue->ci++;
		i++;
	}
exit:
	for (; queue->ci < queue->pi; queue->ci++) {
		if (queue->jobs[res->header.job_id].used == 1)
			break;
	}
	return rec;
}
