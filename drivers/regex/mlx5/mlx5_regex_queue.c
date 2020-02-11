/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#include "mlx5.h"
#include "mlx5_regex_queue.h"

/**
 * Send a single request
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
tx_burst_msegs_op(struct rte_regex_dev *dev, struct mlx5_regex_queue *q,
		  struct rte_regex_ops *ops)
{
	struct mlx5_regex_priv *priv =
		container_of(dev, struct mlx5_regex_priv, regex_dev);
	struct mlx5_wqe_metadata_seg *restrict mseg;
	struct mlx5_regex_wqe_ctrl_seg regex_cseg;
	struct mlx5_wqe_data_seg *restrict dseg;
	struct mlx5_wqe_ctrl_seg *restrict cseg;
	struct mlx5_wqe_data_seg tmp_dseg;
	int wqe_idx = q->q_pi % q->q_size;
	struct mlx5_regex_buffers *regex_buf;
	size_t wqe_offset;
	uint64_t *db_addr;
	uint32_t lkey;
	int ds, i;

	if (q->q_pi - q->q_ci >= q->q_size)
		return EBUSY;

	ds = ops->num_of_bufs;
	/* REGEX dev currently support single segment req buf. */
	assert(ds > 1 && ds < 2);
	/*
	 * WQE:
	 * - 1 Control Segment
	 * - 1 Metadata Segment
	 * - nseg = 1 Data Segments for request
	 * - 1 Data Segment for response
	 */
	ds += 3;
	wqe_offset = wqe_idx * MLX5_SEND_WQE_BB ;
	regex_buf = q->regex_bufs[wqe_idx];
	cseg = (struct mlx5_wqe_ctrl_seg *)((uint8_t*)q->wqes + wqe_offset);
	/* TODO: ask Yuval about the last param ctrl_field */
	set_ctrl_seg((void *)&regex_cseg, 0, ops, /* ctrl_field */ 0);
	mlx5dv_set_ctrl_seg(cseg, q->q_pi, MLX5_OPCODE_MMO,
			    MLX5_OPC_MOD_MMO_REGEX, q->sqn,
			    MLX5_WQE_CTRL_CQ_UPDATE, ds, 0,
			    regex_cseg.le_subset_id_0_subset_id_1);
	/* Set wqe metadata segment */
	mseg = (struct mlx5_wqe_metadata_seg *)((uint8_t*)cseg + sizeof(*cseg));
	lkey = mlx5_regex_mr_addr2mr_bh(dev, &q->mr_ctrl,
					regex_buf->m_iov.buf_iova);
	mlx5dv_set_data_seg(&tmp_dseg, regex_buf->m_iov.buf_size,
			    lkey, regex_buf->m_iov.buf_iova);
	mlx5dv_set_metadata_seg(mseg, regex_cseg.ctrl_subset_id_2_subset_id_3,
				lkey, tmp_dseg.addr);
	/* Data segment for nsegs buffer */
	dseg = (struct mlx5_wqe_data_seg *)((uint8_t*)mseg + sizeof(*mseg));
	for (i = 0; i < ds; ++i) {
		lkey = mlx5_regex_mr_addr2mr_bh(dev, &q->mr_ctrl,
						(*ops->bufs)[i]->buf_iova);
		mlx5dv_set_data_seg(dseg, (*ops->bufs)[i]->buf_size,
				    lkey, (*ops->bufs)[i]->buf_iova);
		++dseg;
	}
	/* Data segment for response buffer */
	lkey = mlx5_regex_mr_addr2mr_bh(dev, &q->mr_ctrl,
					regex_buf->resp_iov.buf_iova);
	mlx5dv_set_data_seg(dseg, regex_buf->resp_iov.buf_size,
			    lkey, regex_buf->resp_iov.buf_iova);

	/* Ring doorbell */
	db_addr = (uint64_t *)((uint8_t *)priv->uar->base_addr + 0x800);
	asm volatile("" ::: "memory");
	q->q_db[MLX5_SND_DBR] = htobe32(q->q_pi & 0xffff);
	asm volatile("" ::: "memory");
	*db_addr = *(uint64_t *)cseg;
	asm volatile("" ::: "memory");

	++q->q_pi;
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
 *   Pointer to an array of requests.
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
	struct mlx5_regex_queue *q = (*priv->queues)[q_id];
	int processed = 0;
	int err;
	int i;

	if (unlikely(!nb_ops) || q->busy)
		return 0;

	for (i = 0; i < nb_ops; i++) {
		err = tx_burst_msegs_op(dev, q, ops[i]);
		if (err)
			break;
		++processed;
	}
	return processed;
}
