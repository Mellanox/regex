/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 6WIND S.A.
 * Copyright 2019 Mellanox Technologies, Ltd
 */

#include <assert.h>
#include <stdio.h>
#include <time.h>

#include <rte_eal.h>
#include <rte_errno.h>

#include "mlx5_common_mp.h"
#include "mlx5_common_utils.h"

/**
 * Request Memory Region creation to the primary process.
 *
 * @param[in] port_id
 *   Port ID of the device.
 * @param addr
 *   Target virtual address to register.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
int
mlx5_mp_req_mr_create(uint16_t port_id, uintptr_t addr)
{
	struct rte_mp_msg mp_req;
	struct rte_mp_msg *mp_res;
	struct rte_mp_reply mp_rep;
	struct mlx5_mp_param *req = (struct mlx5_mp_param *)mp_req.param;
	struct mlx5_mp_param *res;
	struct timespec ts = {.tv_sec = MLX5_MP_REQ_TIMEOUT_SEC, .tv_nsec = 0};
	int ret;

	assert(rte_eal_process_type() == RTE_PROC_SECONDARY);
	mp_init_msg(port_id, &mp_req, MLX5_MP_REQ_CREATE_MR);
	req->args.addr = addr;
	ret = rte_mp_request_sync(&mp_req, &mp_rep, &ts);
	if (ret) {
		DRV_LOG(ERR, "port %u request to primary process failed",
			port_id);
		return -rte_errno;
	}
	assert(mp_rep.nb_received == 1);
	mp_res = &mp_rep.msgs[0];
	res = (struct mlx5_mp_param *)mp_res->param;
	ret = res->result;
	if (ret)
		rte_errno = -ret;
	free(mp_rep.msgs);
	return ret;
}

/**
 * Request Verbs queue state modification to the primary process.
 *
 * @param[in] port_id
 *   Port ID of the device.
 * @param sm
 *   State modify parameters.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
int
mlx5_mp_req_queue_state_modify(uint16_t port_id,
			       struct mlx5_mp_arg_queue_state_modify *sm)
{
	struct rte_mp_msg mp_req;
	struct rte_mp_msg *mp_res;
	struct rte_mp_reply mp_rep;
	struct mlx5_mp_param *req = (struct mlx5_mp_param *)mp_req.param;
	struct mlx5_mp_param *res;
	struct timespec ts = {.tv_sec = MLX5_MP_REQ_TIMEOUT_SEC, .tv_nsec = 0};
	int ret;

	assert(rte_eal_process_type() == RTE_PROC_SECONDARY);
	mp_init_msg(port_id, &mp_req, MLX5_MP_REQ_QUEUE_STATE_MODIFY);
	req->args.state_modify = *sm;
	ret = rte_mp_request_sync(&mp_req, &mp_rep, &ts);
	if (ret) {
		DRV_LOG(ERR, "port %u request to primary process failed",
			port_id);
		return -rte_errno;
	}
	assert(mp_rep.nb_received == 1);
	mp_res = &mp_rep.msgs[0];
	res = (struct mlx5_mp_param *)mp_res->param;
	ret = res->result;
	free(mp_rep.msgs);
	return ret;
}

/**
 * Request Verbs command file descriptor for mmap to the primary process.
 *
 * @param[in] port_id
 *   Port ID of the device.
 *
 * @return
 *   fd on success, a negative errno value otherwise and rte_errno is set.
 */
int
mlx5_mp_req_verbs_cmd_fd(uint16_t port_id)
{
	struct rte_mp_msg mp_req;
	struct rte_mp_msg *mp_res;
	struct rte_mp_reply mp_rep;
	struct mlx5_mp_param *res;
	struct timespec ts = {.tv_sec = MLX5_MP_REQ_TIMEOUT_SEC, .tv_nsec = 0};
	int ret;

	assert(rte_eal_process_type() == RTE_PROC_SECONDARY);
	mp_init_msg(port_id, &mp_req, MLX5_MP_REQ_VERBS_CMD_FD);
	ret = rte_mp_request_sync(&mp_req, &mp_rep, &ts);
	if (ret) {
		DRV_LOG(ERR, "port %u request to primary process failed",
			port_id);
		return -rte_errno;
	}
	assert(mp_rep.nb_received == 1);
	mp_res = &mp_rep.msgs[0];
	res = (struct mlx5_mp_param *)mp_res->param;
	if (res->result) {
		rte_errno = -res->result;
		DRV_LOG(ERR,
			"port %u failed to get command FD from primary process",
			port_id);
		ret = -rte_errno;
		goto exit;
	}
	assert(mp_res->num_fds == 1);
	ret = mp_res->fds[0];
	DRV_LOG(DEBUG, "port %u command FD from primary is %d",
		port_id, ret);
exit:
	free(mp_rep.msgs);
	return ret;
}

/**
 * Initialize by primary process.
 */
int
mlx5_mp_init_primary(rte_mp_t primary_action)
{
	int ret;

	assert(rte_eal_process_type() == RTE_PROC_PRIMARY);

	/* primary is allowed to not support IPC */
	ret = rte_mp_action_register(MLX5_MP_NAME, primary_action);
	if (ret && rte_errno != ENOTSUP)
		return -1;
	return 0;
}

/**
 * Un-initialize by primary process.
 */
void
mlx5_mp_uninit_primary(void)
{
	assert(rte_eal_process_type() == RTE_PROC_PRIMARY);
	rte_mp_action_unregister(MLX5_MP_NAME);
}

/**
 * Initialize by secondary process.
 */
int
mlx5_mp_init_secondary(rte_mp_t secondary_action)
{
	assert(rte_eal_process_type() == RTE_PROC_SECONDARY);
	return rte_mp_action_register(MLX5_MP_NAME, secondary_action);
}

/**
 * Un-initialize by secondary process.
 */
void
mlx5_mp_uninit_secondary(void)
{
	assert(rte_eal_process_type() == RTE_PROC_SECONDARY);
	rte_mp_action_unregister(MLX5_MP_NAME);
}
