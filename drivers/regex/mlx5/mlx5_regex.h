/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#ifndef MLX5_REGEX_H
#define MLX5_REGEX_H

#include <mlx5_devx_cmds.h>
#include <mlx5_prm.h>

#include "mlx5_regex_utils.h"

void print_raw(volatile uint8_t*, size_t cnt);

//#define REGEX_MLX5_NO_REAL_HW 1
#define LOG_SQ_SIZE 4
#define SQ_SIZE (1<<LOG_SQ_SIZE)
struct mlx5_regex_wqe_ctrl_seg {
       __be32 le_subset_id_0_subset_id_1;
       __be32 ctrl_subset_id_2_subset_id_3;
};


#define MAX_WQE_INDEX (1<<16)

int mlx5_regex_logtype;

struct regex_caps {
	uint8_t supported;
	u8 num_of_engines;
	u8 log_crspace_size;
};

struct mlx5_regex_cq {
	int cqn;
	volatile uint32_t *cq_dbr;
	volatile uint8_t *cq_buff;
	unsigned int ci;
	struct mlx5dv_devx_obj *devx_obj;
	size_t cq_size;
};

struct mlx5_regex_sq {
	int qpn;
	struct mlx5dv_devx_obj *devx_obj;
	uint32_t *qp_dbr;
	void* wq_buff;
	unsigned int pi;
	unsigned int ci;
	unsigned int db_pi;
};

struct mlx5_regex_ctx {
	struct ibv_context *ibv_ctx;
	struct regex_caps caps;
	struct mlx5_regex_sq *qps;
	unsigned int num_qps;
	int pd;
	uint32_t eq;
	void *eq_buff;
	struct mlx5dv_devx_uar *uar;
	struct mlx5_regex_cq cq;
	int uuar;
};
struct mlx5_database_ctx {
	uint32_t umem_id;
	uint64_t offset;
};

// This function should be used in order to pack the segment correctly
static MLX5DV_ALWAYS_INLINE
void mlx5_regex_set_ctrl_seg(void *seg,
			     uint8_t le, uint16_t subset_id0, uint16_t subset_id1,
				 uint16_t subset_id2, uint16_t subset_id3, uint8_t ctrl)
{
	DEVX_SET(regexp_mmo_control, seg, le, le);
	DEVX_SET(regexp_mmo_control, seg, ctrl, ctrl);
	DEVX_SET(regexp_mmo_control, seg, subset_id_0, subset_id0);
	DEVX_SET(regexp_mmo_control, seg, subset_id_1, subset_id1);
	DEVX_SET(regexp_mmo_control, seg, subset_id_2, subset_id2);
	DEVX_SET(regexp_mmo_control, seg, subset_id_3, subset_id3);
}

// This function should be used in order to pack the segment correctly
static MLX5DV_ALWAYS_INLINE
void mlx5_regex_set_metadata(void *seg,
			     uint16_t rof_version,
			     uint16_t latency_count,
			     uint16_t instruction_count,
			     uint16_t primary_thread_count,
			     uint8_t match_count,
			     uint8_t detected_match_count,
			     uint16_t status,
			     uint32_t job_id)
{
	DEVX_SET(regexp_metadata, seg, rof_version, rof_version);
	DEVX_SET(regexp_metadata, seg, latency_count, latency_count);
	DEVX_SET(regexp_metadata, seg, instruction_count, instruction_count);
	DEVX_SET(regexp_metadata, seg, primary_thread_count, primary_thread_count);
	DEVX_SET(regexp_metadata, seg, match_count, match_count);
	DEVX_SET(regexp_metadata, seg, detected_match_count, detected_match_count);
	DEVX_SET(regexp_metadata, seg, status, status);
	DEVX_SET(regexp_metadata, seg, job_id, job_id);

}

int mlx5_regex_is_supported(struct ibv_context *ibv_ctx);

struct mlx5_regex_ctx;

struct mlx5_regex_ctx *mlx5_regex_device_open(struct ibv_context *device,
					      unsigned int num_sqs);
void mlx5_regex_device_close(struct mlx5_regex_ctx *ctx);

int mlx5_regex_register_write(struct ibv_context *ctx, int engine_id,
			      uint32_t addr, uint32_t data);
int mlx5_regex_register_read(struct ibv_context *ctx, int engine_id,
			     uint32_t addr, uint32_t *data);

struct ibv_contxt *mlx5_regex_ibv_ctx_get(struct mlx5_regex_ctx * ctx);

struct mlx5_database_ctx;
/*
 * Sets the database address.
 * When the database umem will be freed,
 * the database will no longer be available to the HW.
 */
int mlx5_regex_database_set(struct ibv_context *ctx, int engine_id,
			    struct mlx5_database_ctx *db_ctx);
int mlx5_regex_database_query(struct ibv_context *ctx, int engine_id,
			      struct mlx5_database_ctx *db_ctx);

/*
 * Stop engine after all outstanding jobs to this engine are done.
 */
int mlx5_regex_engine_stop(struct ibv_context *ctx, int engine_id);

/*
 * Return engine to work.
 */
int mlx5_regex_engine_resume(struct ibv_context *ctx, int engine_id);


struct mlx5_regex_buff;

/*
 * This funtion should be used to register input/output/metdata buffers.
 * Returns lkey.
 */
struct mlx5_regex_buff *mlx5_regex_reg_buffer(struct mlx5_regex_ctx *ctx, void* buff, size_t size);

/*
 * Returns the lkey associated with buff, for input/output/metdata wqe segements.
 */
unsigned int mlx5_regex_get_lkey(struct mlx5_regex_buff *buff);


#endif /* MLX5_REGEX_H */
