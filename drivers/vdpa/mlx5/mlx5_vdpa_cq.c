/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */
#include <unistd.h>
#include <stdint.h>
#include <assert.h>

#include <rte_malloc.h>
#include <rte_errno.h>
#include <rte_lcore.h>

#include "mlx5_vdpa_utils.h"
#include "mlx5_vdpa.h"

void
mlx5_vdpa_cq_global_release(struct mlx5_vdpa_priv *priv)
{
	if (priv->uar) {
		mlx5_glue->devx_free_uar(priv->uar);
		priv->uar = NULL;
	}
	if (priv->eventc) {
		mlx5_glue->devx_destroy_event_channel(priv->eventc);
		priv->eventc = NULL;
	}
	priv->eqn = 0;
}

/* Prepare all the global resources for all the CQs.*/
static int
mlx5_vdpa_cq_global_prepare(struct mlx5_vdpa_priv *priv)
{
	uint32_t lcore;

	if (priv->eventc)
		return 0;
	lcore = (uint32_t)rte_lcore_to_cpu_id(-1);
	if (mlx5_glue->devx_query_eqn(priv->ctx, lcore, &priv->eqn)) {
		rte_errno = errno;
		DRV_LOG(ERR, "Failed to query EQ number %d.", rte_errno);
		return -1;
	}
	priv->eventc = mlx5_glue->devx_create_event_channel(priv->ctx,
			   MLX5DV_DEVX_CREATE_EVENT_CHANNEL_FLAGS_OMIT_EV_DATA);
	if (!priv->eventc) {
		rte_errno = errno;
		DRV_LOG(ERR, "Failed to create event channel %d.",
			rte_errno);
		goto error;
	}
	priv->uar = mlx5_glue->devx_alloc_uar(priv->ctx, 0);
	if (!priv->uar) {
		rte_errno = errno;
		DRV_LOG(ERR, "Failed to allocate UAR.");
		goto error;
	}
	return 0;
error:
	mlx5_vdpa_cq_global_release(priv);
	return -1;
}

void
mlx5_vdpa_cq_destroy(struct mlx5_vdpa_cq *cq)
{
	int ret __rte_unused;

	if (cq->cq) {
		ret = mlx5_devx_cmd_destroy(cq->cq);
		assert(!ret);
	}
	if (cq->umem_obj) {
		ret = mlx5_glue->devx_umem_dereg(cq->umem_obj);
		assert(!ret);
	}
	if (cq->umem_buf)
		rte_free((void *)(uintptr_t)cq->umem_buf);
	memset(cq, 0, sizeof(*cq));
}

int
mlx5_vdpa_cq_create(struct mlx5_vdpa_priv *priv, uint16_t desc_n, int callfd,
		    struct mlx5_vdpa_cq *cq)
{
	struct mlx5_devx_cq_attr attr;
	size_t pgsize = sysconf(_SC_PAGESIZE);
	uint32_t log_desc_n = rte_log2_u32(desc_n);
	uint32_t umem_size;
	int ret;
	uint16_t event_nums[1] = {0};

	if (mlx5_vdpa_cq_global_prepare(priv))
		return -1;
	cq->log_desc_n = log_desc_n;
	umem_size = sizeof(struct mlx5_cqe) * (1 << log_desc_n) +
							sizeof(*cq->db_rec) * 2;
	cq->umem_buf = rte_zmalloc(__func__, umem_size, 4096);
	if (!cq->umem_buf) {
		DRV_LOG(ERR, "Failed to allocate memory for CQ.");
		rte_errno = ENOMEM;
		return -ENOMEM;
	}
	cq->umem_obj = mlx5_glue->devx_umem_reg(priv->ctx,
						(void *)(uintptr_t)cq->umem_buf,
						umem_size,
						IBV_ACCESS_LOCAL_WRITE);
	if (!cq->umem_obj) {
		DRV_LOG(ERR, "Failed to register umem for CQ.");
		goto error;
	}
	attr.q_umem_valid = 1;
	attr.db_umem_valid = 1;
	attr.use_first_only = 0;
	attr.overrun_ignore = 0;
	attr.uar_page_id = priv->uar->page_id;
	attr.q_umem_id = cq->umem_obj->umem_id;
	attr.q_umem_offset = 0;
	attr.db_umem_id = cq->umem_obj->umem_id;
	attr.db_umem_offset = sizeof(struct mlx5_cqe) * (1 << log_desc_n);
	attr.eqn = priv->eqn;
	attr.log_cq_size = log_desc_n;
	attr.log_page_size = rte_log2_u32(pgsize);
	cq->cq = mlx5_devx_cmd_create_cq(priv->ctx, &attr);
	if (!cq->cq)
		goto error;
	cq->db_rec = RTE_PTR_ADD(cq->umem_buf, (uintptr_t)attr.db_umem_offset);
	cq->cq_ci = 0;
	rte_spinlock_init(&cq->sl);
	/* Subscribe CQ event to the event channel controlled by the driver. */
	ret = mlx5_glue->devx_subscribe_devx_event(priv->eventc, cq->cq->obj,
						   sizeof(event_nums),
						   event_nums,
						   (uint64_t)(uintptr_t)cq);
	if (ret) {
		DRV_LOG(ERR, "Failed to subscribe CQE event.");
		rte_errno = errno;
		goto error;
	}
	/* Subscribe CQ event to the guest FD only if it is not in poll mode. */
	if (callfd != -1) {
		ret = mlx5_glue->devx_subscribe_devx_event_fd(priv->eventc,
							      callfd,
							      cq->cq->obj, 0);
		if (ret) {
			DRV_LOG(ERR, "Failed to subscribe CQE event fd.");
			rte_errno = errno;
			goto error;
		}
	}
	return 0;
error:
	mlx5_vdpa_cq_destroy(cq);
	return -1;
}
