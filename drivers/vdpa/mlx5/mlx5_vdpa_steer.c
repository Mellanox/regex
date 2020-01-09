/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */
#include <assert.h>
#include <netinet/in.h>

#include <rte_malloc.h>
#include <rte_errno.h>

#include "mlx5_vdpa_utils.h"
#include "mlx5_vdpa.h"

int
mlx5_vdpa_steer_unset(struct mlx5_vdpa_priv *priv)
{
	int ret __rte_unused;
	unsigned i;

	for (i = 0; i < RTE_DIM(priv->steer.rss); ++i) {
		if (priv->steer.rss[i].flow) {
			ret = mlx5_glue->dv_destroy_flow
						      (priv->steer.rss[i].flow);
			assert(!ret);
			priv->steer.rss[i].flow = NULL;
		}
		if (priv->steer.rss[i].tir_action) {
			ret = mlx5_glue->destroy_flow_action
						(priv->steer.rss[i].tir_action);
			assert(!ret);
			priv->steer.rss[i].tir_action = NULL;
		}
		if (priv->steer.rss[i].tir) {
			ret = mlx5_devx_cmd_destroy(priv->steer.rss[i].tir);
			assert(!ret);
			priv->steer.rss[i].tir = NULL;
		}
		if (priv->steer.rss[i].matcher) {
			ret = mlx5_glue->dv_destroy_flow_matcher
						   (priv->steer.rss[i].matcher);
			assert(!ret);
			priv->steer.rss[i].matcher = NULL;
		}
	}
	if (priv->steer.tbl) {
		ret = mlx5_glue->dr_destroy_flow_tbl(priv->steer.tbl);
		assert(!ret);
		priv->steer.tbl = NULL;
	}
	if (priv->steer.domain) {
		ret = mlx5_glue->dr_destroy_domain(priv->steer.domain);
		assert(!ret);
		priv->steer.domain = NULL;
	}
	if (priv->steer.rqt) {
		ret = mlx5_devx_cmd_destroy(priv->steer.rqt);
		assert(!ret);
		priv->steer.rqt = NULL;
	}
	return 0;
}

/*
 * According to VIRTIO_NET Spec the virtqueues index identity its type by:
 * 0 receiveq1
 * 1 transmitq1
 * ...
 * 2(N-1) receiveqN
 * 2(N-1)+1 transmitqN
 * 2N controlq
 */
static uint8_t
is_virtq_recvq(int virtq_index, int nr_vring)
{
	if (virtq_index % 2 == 0 && virtq_index != nr_vring - 1)
		return 1;
	return 0;
}

static int __rte_unused
mlx5_vdpa_rqt_create(struct mlx5_vdpa_priv *priv)
{
	struct mlx5_vdpa_virtq *virtq;
	struct mlx5_devx_rqt_attr *attr = rte_zmalloc(__func__, sizeof(*attr)
						      + priv->nr_virtqs *
						      sizeof(uint32_t), 0);
	uint16_t rq_num = 0;

	if (!attr) {
		DRV_LOG(ERR, "Failed to allocate RQT attributes memory.");
		rte_errno = ENOMEM;
		return -ENOMEM;
	}
	attr->rq_type = MLX5_INLINE_Q_TYPE_VIRTQ;
	attr->rqt_max_size = priv->nr_virtqs;
	rq_num = 0;
	SLIST_FOREACH(virtq, &priv->virtq_list, next) {
		if (is_virtq_recvq(virtq->index, priv->nr_virtqs)) {
			attr->rq_list[rq_num] = virtq->virtq->id;
			rq_num++;
		}
	}
	attr->rqt_actual_size = rq_num;
	priv->steer.rqt = mlx5_devx_cmd_create_rqt(priv->ctx, attr);
	rte_free(attr);
	if (!priv->steer.rqt) {
		DRV_LOG(ERR, "Failed to create RQT.");
		return -1;
	}
	return 0;
}

static int __rte_unused
mlx5_vdpa_rss_flows_create(struct mlx5_vdpa_priv *priv)
{
#ifdef HAVE_MLX5DV_DR
	struct mlx5_devx_tir_attr tir_att = {
		.disp_type = MLX5_TIRC_DISP_TYPE_INDIRECT,
		.rx_hash_fn = MLX5_RX_HASH_FN_TOEPLITZ,
		.transport_domain = priv->td->id,
		.indirect_table = priv->steer.rqt->id,
		.rx_hash_symmetric = 1,
		.rx_hash_toeplitz_key = { 0x2cc681d1, 0x5bdbf4f7, 0xfca28319,
					  0xdb1a3e94, 0x6b9e38d9, 0x2c9c03d1,
					  0xad9944a7, 0xd9563d59, 0x063c25f3,
					  0xfc1fdc2a },
	};
	struct {
		size_t size;
		/**< Size of match value. Do NOT split size and key! */
		uint32_t buf[MLX5_ST_SZ_DW(fte_match_param)];
		/**< Matcher value. This value is used as the mask or a key. */
	} matcher_mask = {
				.size = sizeof(matcher_mask.buf),
			},
	  matcher_value = {
				.size = sizeof(matcher_value.buf),
			};
	struct mlx5dv_flow_matcher_attr dv_attr = {
		.type = IBV_FLOW_ATTR_NORMAL,
		.match_mask = (void *)&matcher_mask,
	};
	void *match_m = matcher_mask.buf;
	void *match_v = matcher_value.buf;
	void *headers_m = MLX5_ADDR_OF(fte_match_param, match_m, outer_headers);
	void *headers_v = MLX5_ADDR_OF(fte_match_param, match_v, outer_headers);
	void *actions[1];
	const uint8_t l3_hash =
		(1 << MLX5_RX_HASH_FIELD_SELECT_SELECTED_FIELDS_SRC_IP) |
		(1 << MLX5_RX_HASH_FIELD_SELECT_SELECTED_FIELDS_DST_IP);
	const uint8_t l4_hash =
		(1 << MLX5_RX_HASH_FIELD_SELECT_SELECTED_FIELDS_L4_SPORT) |
		(1 << MLX5_RX_HASH_FIELD_SELECT_SELECTED_FIELDS_L4_DPORT);
	enum { PRIO, CRITERIA, IP_VER_M, IP_VER_V, IP_PROT_M, IP_PROT_V, L3_BIT,
	       L4_BIT, HASH, END};
	const uint8_t vars[RTE_DIM(priv->steer.rss)][END] = {
		{ 7, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 6, 1 << MLX5_MATCH_CRITERIA_ENABLE_OUTER_BIT, 0xf, 4, 0, 0,
		 MLX5_L3_PROT_TYPE_IPV4, 0, l3_hash },
		{ 6, 1 << MLX5_MATCH_CRITERIA_ENABLE_OUTER_BIT, 0xf, 6, 0, 0,
		 MLX5_L3_PROT_TYPE_IPV6, 0, l3_hash },
		{ 5, 1 << MLX5_MATCH_CRITERIA_ENABLE_OUTER_BIT, 0xf, 4, 0xff,
		 IPPROTO_UDP, MLX5_L3_PROT_TYPE_IPV4, MLX5_L4_PROT_TYPE_UDP,
		 l3_hash | l4_hash },
		{ 5, 1 << MLX5_MATCH_CRITERIA_ENABLE_OUTER_BIT, 0xf, 4, 0xff,
		 IPPROTO_TCP, MLX5_L3_PROT_TYPE_IPV4, MLX5_L4_PROT_TYPE_TCP,
		 l3_hash | l4_hash },
		{ 5, 1 << MLX5_MATCH_CRITERIA_ENABLE_OUTER_BIT, 0xf, 6, 0xff,
		 IPPROTO_UDP, MLX5_L3_PROT_TYPE_IPV6, MLX5_L4_PROT_TYPE_UDP,
		 l3_hash | l4_hash },
		{ 5, 1 << MLX5_MATCH_CRITERIA_ENABLE_OUTER_BIT, 0xf, 6, 0xff,
		 IPPROTO_TCP, MLX5_L3_PROT_TYPE_IPV6, MLX5_L4_PROT_TYPE_TCP,
		 l3_hash | l4_hash },
	};
	unsigned i;

	for (i = 0; i < RTE_DIM(priv->steer.rss); ++i) {
		dv_attr.priority = vars[i][PRIO];
		dv_attr.match_criteria_enable = vars[i][CRITERIA];
		MLX5_SET(fte_match_set_lyr_2_4, headers_m, ip_version,
			 vars[i][IP_VER_M]);
		MLX5_SET(fte_match_set_lyr_2_4, headers_v, ip_version,
			 vars[i][IP_VER_V]);
		MLX5_SET(fte_match_set_lyr_2_4, headers_m, ip_protocol,
			 vars[i][IP_PROT_M]);
		MLX5_SET(fte_match_set_lyr_2_4, headers_v, ip_protocol,
			 vars[i][IP_PROT_V]);
		tir_att.rx_hash_field_selector_outer.l3_prot_type =
								vars[i][L3_BIT];
		tir_att.rx_hash_field_selector_outer.l4_prot_type =
								vars[i][L4_BIT];
		tir_att.rx_hash_field_selector_outer.selected_fields =
								  vars[i][HASH];
		priv->steer.rss[i].matcher = mlx5_glue->dv_create_flow_matcher
					 (priv->ctx, &dv_attr, priv->steer.tbl);
		if (!priv->steer.rss[i].matcher) {
			DRV_LOG(ERR, "Failed to create matcher %d.", i);
			goto error;
		}
		priv->steer.rss[i].tir = mlx5_devx_cmd_create_tir(priv->ctx,
								  &tir_att);
		if (!priv->steer.rss[i].tir) {
			DRV_LOG(ERR, "Failed to create TIR %d.", i);
			goto error;
		}
		priv->steer.rss[i].tir_action =
				mlx5_glue->dv_create_flow_action_dest_devx_tir
						  (priv->steer.rss[i].tir->obj);
		if (!priv->steer.rss[i].tir_action) {
			DRV_LOG(ERR, "Failed to create TIR action %d.", i);
			goto error;
		}
		actions[0] = priv->steer.rss[i].tir_action;
		priv->steer.rss[i].flow = mlx5_glue->dv_create_flow
					(priv->steer.rss[i].matcher,
					 (void *)&matcher_value, 1, actions);
		if (!priv->steer.rss[i].flow) {
			DRV_LOG(ERR, "Failed to create flow %d.", i);
			goto error;
		}
	}
	return 0;
error:
	/* Resources will be freed by the caller. */
	return -1;
#else
	(void)priv;
	return -ENOTSUP;
#endif /* HAVE_MLX5DV_DR */
}

int
mlx5_vdpa_steer_setup(struct mlx5_vdpa_priv *priv)
{
#ifdef HAVE_MLX5DV_DR
	if (mlx5_vdpa_rqt_create(priv))
		return -1;
	priv->steer.domain = mlx5_glue->dr_create_domain(priv->ctx,
						  MLX5DV_DR_DOMAIN_TYPE_NIC_RX);
	if (!priv->steer.domain) {
		DRV_LOG(ERR, "Failed to create Rx domain.");
		goto error;
	}
	priv->steer.tbl = mlx5_glue->dr_create_flow_tbl(priv->steer.domain, 0);
	if (!priv->steer.tbl) {
		DRV_LOG(ERR, "Failed to create table 0 with Rx domain.");
		goto error;
	}
	if (mlx5_vdpa_rss_flows_create(priv))
		goto error;
	return 0;
error:
	mlx5_vdpa_steer_unset(priv);
	return -1;
#else
	(void)priv;
	return -ENOTSUP;
#endif /* HAVE_MLX5DV_DR */
}
