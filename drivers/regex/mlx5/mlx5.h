/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */
#ifndef __MLX5_H__
#define __MLX5_H__

#include <infiniband/mlx5dv.h>
#include <rte_regexdev_driver.h>
#include <rte_rwlock.h>

#include <mlx5_common_mr.h>
#include <mlx5_common_cq.h>
#include <mlx5_common_sq.h>

struct mlx5_database_ctx {
	uint32_t umem_id;
	uint64_t offset;
};

struct mlx5_regex_db {
	/* TODO: refer to struct rxp_database of Yuval code */
	void *raw_mem;
	struct mlx5dv_devx_umem *umem;
	/* TODO: refer to Yuval's struct mlx5_database_ctx */
	struct mlx5_database_ctx umem_ctx;
};

struct mlx5_regex_wqe_ctrl_seg {
	__be32 le_subset_id_0_subset_id_1;
	__be32 ctrl_subset_id_2_subset_id_3;
};

struct mlx5_regex_resp_desc {
	uint32_t job_id;
	uint16_t status;
	uint8_t detected_match_count;
	uint8_t match_count;
	uint16_t primary_thread_count;
	uint16_t instruction_count;
	uint16_t latency_count;
	uint16_t pmi_min_byte_ptr;
};

struct mlx5_regex_match_tuple {
	uint32_t rule_id;
	uint16_t start_ptr;
	uint16_t length;
};

#define MLX5_REGEX_METADATA_SIZE	64
#define MLX5_REGEX_RESPONSE_SIZE	(254 * sizeof(struct mlx5_regex_match_tuple))

struct mlx5_regex_job {
	struct rte_regex_iov m_iov; /* Buffer for metadata */
	struct rte_regex_iov resp_iov; /* Buffer to receive response from dev */
	bool in_progress; /* Buffer is in used and not dequeued yet */
	uint64_t user_id; /* Store req's user_id used this buffer */
};

struct mlx5_regex_qp {
	struct mlx5dv_devx_uar *uar;
	struct mlx5_mr_ctrl mr_ctrl;
	bool busy;
	struct mlx5_common_cq cq;
	struct mlx5_common_sq sq;
	struct mlx5_regex_job **regex_jobs;
};

struct mlx5_regex_priv {
	struct rte_regex_dev regex_dev;
	struct ibv_context *ctx; /* Device context. */
	struct ibv_pd *pd;
	uint32_t pdn;
	uint32_t eqn;
	struct mlx5_regex_db *db_desc;
	int num_db_desc;
	struct mlx5_mr_share_cache mr_scache; /* Global shared MR cache. */
	struct mlx5_regex_qp *qps; /* Table of QPs */
	int num_qps;			/* QP table length */
	TAILQ_ENTRY(mlx5_regex_priv) next;
	struct rte_pci_device *pci_dev;
};

#endif
