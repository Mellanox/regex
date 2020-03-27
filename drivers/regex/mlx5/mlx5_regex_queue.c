/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#include <rte_mbuf.h>

#include "mlx5.h"
#include "mlx5_regex_queue.h"
#include "mlx5_regex_utils.h"

static __rte_always_inline void
set_ctrl_seg(void *seg, uint8_t le, struct rte_regex_ops *ops, uint8_t ctrl)
{
	DEVX_SET(regexp_mmo_control, seg, le, le);
	DEVX_SET(regexp_mmo_control, seg, ctrl, ctrl);
	DEVX_SET(regexp_mmo_control, seg, subset_id_0, ops->group_id0);
	DEVX_SET(regexp_mmo_control, seg, subset_id_1, ops->group_id1);
	DEVX_SET(regexp_mmo_control, seg, subset_id_2, ops->group_id2);
	DEVX_SET(regexp_mmo_control, seg, subset_id_3, ops->group_id3);
}

static __rte_always_inline void
mlx5dv_set_metadata_seg(struct mlx5_wqe_metadata_seg *seg,
			uint32_t mmo_control_31_0, uint32_t lkey,
			uintptr_t address)
{
	seg->mmo_control_31_0 = rte_cpu_to_be_32(mmo_control_31_0);
	seg->lkey = rte_cpu_to_be_32(lkey);
	seg->addr = rte_cpu_to_be_64(address);
}

/**
 * Send a single request
 *
 * @param priv
 *   Pointer to REGEX device's private datastructure.
 * @param q
 *   The queue to send the request.
 * @param ops
 *   Pointer to the request.
 *
 * @return
 *   0: success else error code.
 */
static int
tx_burst_msegs_op(struct mlx5_regex_priv *priv, struct mlx5_regex_qp *q,
		  struct rte_regex_ops *ops)
{
	struct mlx5_wqe_metadata_seg *restrict mseg;
	int wqe_idx = q->sq.pi % q->sq.num_entries;
	struct mlx5_regex_wqe_ctrl_seg regex_cseg;
	struct mlx5_wqe_data_seg *restrict dseg;
	struct mlx5_wqe_ctrl_seg *restrict cseg;
	struct mlx5_wqe_data_seg tmp_dseg;
	struct mlx5_regex_job *job;
	size_t wqe_offset;
	uint32_t lkey;
	int nsegs;

	if (q->sq.pi - q->sq.ci >= q->sq.num_entries)
		return EBUSY;
	/*
	 * REGEX dev currently support single segment req buf.
	 * TODO: UMR to support multiple segments.
	 */
	assert(ops->num_of_bufs == 1);
	nsegs = NB_SEGS(ops->bufs);
	assert(nsegs == 1);
	/*
	 * WQE:
	 * - 1 Control Segment
	 * - 1 Metadata Segment
	 * - 1 Data Segment for request
	 * - 1 Data Segment for response
	 */
	nsegs += 3;
	wqe_offset = wqe_idx * MLX5_SEND_WQE_BB ;
	job = q->regex_jobs[wqe_idx];
	job->in_progress = true;
	job->user_id = ops->user_id;
	cseg = (struct mlx5_wqe_ctrl_seg *)
		((uint8_t *)(uintptr_t)q->sq.umem_buf + wqe_offset);
	/* TODO: ask Yuval about the last param ctrl_field */
	set_ctrl_seg((void *)&regex_cseg, 0, ops, /* ctrl_field */ 0);
#ifdef MLX5_REGEX_REAL_HW
	mlx5dv_set_ctrl_seg(cseg, q->sq.pi & MLX5_QP_CI_MASK, MLX5_OPCODE_MMO,
			    MLX5_OPC_MOD_MMO_REGEX, q->sq.sqn,
			    MLX5_WQE_CTRL_CQ_UPDATE, nsegs, 0,
			    regex_cseg.le_subset_id_0_subset_id_1);
#else
	mlx5dv_set_ctrl_seg(cseg, q->sq.pi & MLX5_QP_CI_MASK, MLX5_OPCODE_NOP,
			    0, q->sq.sqn, MLX5_WQE_CTRL_CQ_UPDATE, 1, 0, 0);
#endif
	/* Set wqe metadata segment */
	mseg = (struct mlx5_wqe_metadata_seg *)((uint8_t*)cseg + sizeof(*cseg));
	lkey = mlx5_mr_addr2mr_bh(priv->pd, priv->regex_dev.dev_id,
				  &priv->mr_scache, &q->mr_ctrl,
				  (uintptr_t) job->meta, 0);
	mlx5dv_set_data_seg(&tmp_dseg, MLX5_REGEX_METADATA_SIZE, lkey,
			    (uintptr_t) job->meta);
	mlx5dv_set_metadata_seg(mseg, regex_cseg.ctrl_subset_id_2_subset_id_3,
				lkey, tmp_dseg.addr);
	/* Data segment for nsegs buffer */
	dseg = (struct mlx5_wqe_data_seg *)((uint8_t*)mseg + sizeof(*mseg));
	lkey = mlx5_mr_addr2mr_bh(priv->pd, priv->regex_dev.dev_id,
				  &priv->mr_scache, &q->mr_ctrl,
				  rte_pktmbuf_mtod(ops->bufs, uintptr_t), 0);
	mlx5dv_set_data_seg(dseg, rte_pktmbuf_data_len(ops->bufs),
			    lkey, rte_pktmbuf_mtod(ops->bufs, uintptr_t));
	/* Data segment for response buffer */
	dseg += sizeof(*dseg);
	lkey = mlx5_mr_addr2mr_bh(priv->pd, priv->regex_dev.dev_id,
				  &priv->mr_scache, &q->mr_ctrl,
				  (uintptr_t) job->resp, 0);
	mlx5dv_set_data_seg(dseg, MLX5_REGEX_RESPONSE_SIZE, lkey,
			    (uintptr_t) job->resp);
	/* Ring doorbell */
	++q->sq.pi;
	rte_io_wmb();
	q->sq.db_rec[MLX5_SND_DBR] = rte_cpu_to_be_32(q->sq.pi & MLX5_QP_CI_MASK);
	rte_wmb();
#ifdef RTE_ARCH_64
	*(uint64_t *)q->sq.db_addr = *(volatile uint64_t *)cseg;
#else
	*(uint32_t *)q->sq.db_addr = *(volatile uint64_t *)cseg;
	rte_io_wmb();
	*((uint32_t *)q->sq.db_addr +1) = *(volatile uint64_t *)cseg >> 32;
#endif
	return 0;
}

/**
 * Process all the responses on the completion entry.
 *
 * @param dev
 *   Pointer to REGEX device.
 * @param q
 *   The queue to send the request.
 * @param ops
 *   Pointer to the request.
 *
 * @return
 *   0: success else error code.
 */
static int
process_queue_resp(struct mlx5_regex_qp *q, struct rte_regex_ops *ops,
		   volatile struct mlx5_cqe *cqe)
{
	struct mlx5_regex_match_tuple *matches;
	struct mlx5_regex_resp_desc *resp_desc;
	struct mlx5_regex_job *job;
	int match_count = 0;
	int i;

        job = q->regex_jobs[cqe->wqe_id];
	match_count = DEVX_GET(regexp_metadata, (uint8_t *)job->meta + 32,
			       match_count);
	if (match_count < 0)
		return EAGAIN;
	resp_desc = (struct mlx5_regex_resp_desc *)job->meta  + 32;
	matches = (struct mlx5_regex_match_tuple *)job->resp;
	ops->nb_matches = match_count;
	ops->nb_actual_matches = resp_desc->detected_match_count;
	ops->user_id = job->user_id;
	for (i = 0; i < ops->nb_matches; i++) {
		ops->matches[i].rule_id = rte_le_to_cpu_32(matches[i].rule_id);
		ops->matches[i].offset = rte_le_to_cpu_16(matches[i].start_ptr);
		ops->matches[i].len = rte_le_to_cpu_16(matches[i].length);
	}
	job->in_progress = false;
	q->sq.ci++;
	return 0;
}

/**
 * Enqueue a burst of scan requests to a queue of REGEX device
 *
 * @param dev
 *   Pointer to REGEX device.
 * @param q_id
 *   Index of the queue.
 * @param ops
 *   Pointer to an array of regex_ops.
 * @param nb_ops
 *   Number of requests in the ops array.
 *
 * @return
 *   Number of requests processed.
 */
int mlx5_regex_enqueue(struct rte_regex_dev *dev, uint16_t q_id,
		       struct rte_regex_ops **ops, uint16_t nb_ops)
{
	struct mlx5_regex_priv *priv =
		container_of(dev, struct mlx5_regex_priv, regex_dev);
	struct mlx5_regex_qp *q = &priv->qps[q_id];
	int processed = 0;
	int err;
	int i;

	if (unlikely(!nb_ops) || q->busy)
		return 0;
	for (i = 0; i < nb_ops; i++) {
		err = tx_burst_msegs_op(priv, q, ops[i]);
		if (err)
			break;
		++processed;
	}
	return processed;
}

/**
 * Dequeue the statuses and responses of previous submitted requests
 *
 * @param dev
 *   Pointer to REGEX device.
 * @param q_id
 *   Index of the queue.
 * @param ops
 *   Pointer to an array of regex_ops
 * @param nb_ops
 *   Number of regex_ops in the ops array.
 *
 * @return
 *   Number of regex_ops processed/responsed.
 */
int mlx5_regex_dequeue(struct rte_regex_dev *dev, uint16_t q_id,
		       struct rte_regex_ops **ops, uint16_t nb_ops)
{
	struct mlx5_regex_priv *priv =
		container_of(dev, struct mlx5_regex_priv, regex_dev);
	struct mlx5_regex_qp *q = &priv->qps[q_id];
	volatile struct mlx5_cqe *cqe = NULL;
	struct mlx5_common_cq *cq = &q->cq;
	unsigned int cqe_mask;
	int processed = 0;
	int i = 0;
	int ret;

	if (unlikely(!nb_ops) || q->busy)
		return 0;
	cqe_mask = (1 << cq->log_desc_n) - 1;
	do {
		cqe = cq->cqes + (cq->cq_ci & cqe_mask);
		ret = check_cqe(cqe, cqe_mask + 1, cq->cq_ci);
		switch (ret) {
		case MLX5_CQE_STATUS_ERR:
			DRV_LOG(ERR, "got CQE with ERROR\n");
			cq->errors++;
			cq->cq_ci++;
			goto out;
		case MLX5_CQE_STATUS_SW_OWN:
			cq->cq_ci++;
			ret = process_queue_resp(q, ops[i], cqe);
			if (ret)
				goto out;
			++processed;
			++i;
			break;
		case MLX5_CQE_STATUS_HW_OWN:
		default:
			break;
		}
	} while (ret != MLX5_CQE_STATUS_HW_OWN && i < nb_ops);
out:
	rte_io_wmb();
	cq->db_rec[MLX5_CQ_RING_DB] = rte_cpu_to_be_32(cq->cq_ci & MLX5_CI_MASK);
	return processed;
}
