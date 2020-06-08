/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2020 Mellanox Technologies, Ltd
 */

#include <errno.h>

#include <rte_log.h>
#include <rte_errno.h>
#include <rte_malloc.h>
#include <rte_regexdev.h>
#include <rte_regexdev_core.h>
#include <rte_regexdev_driver.h>

#include <mlx5_glue.h>
#include <mlx5_devx_cmds.h>
#include <mlx5_prm.h>

#include "mlx5_regex.h"
#include "mlx5_regex_utils.h"
#include "mlx5_rxp_csrs.h"
#include "mlx5_rxp.h"

#define MLX5_REGEX_MAX_MATCHES 255
#define MLX5_REGEX_MAX_PAYLOAD_SIZE UINT16_MAX
#define MLX5_REGEX_MAX_RULES_PER_GROUP UINT16_MAX
#define MLX5_REGEX_MAX_GROUPS UINT16_MAX

/**
 * DPDK callback for reading device info.
 *
 * @param dev
 *   Pointer to RegEx device structure.
 * @param[out] info
 *   Pointer to the regexdev info structure to be filled with the contextual
 *   information of the device.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
int
mlx5_regex_info_get(struct rte_regexdev *dev __rte_unused,
		    struct rte_regexdev_info *info)
{
	info->max_matches = MLX5_REGEX_MAX_MATCHES;
	info->max_payload_size = MLX5_REGEX_MAX_PAYLOAD_SIZE;
	info->max_rules_per_group = MLX5_REGEX_MAX_RULES_PER_GROUP;
	info->max_groups = MLX5_REGEX_MAX_GROUPS;
	info->regexdev_capa = RTE_REGEXDEV_SUPP_PCRE_GREEDY_F;
	info->rule_flags = 0;
	return 0;
}

static int
rxp_poll_csr_for_value(struct ibv_context *ctx, uint32_t *value,
		       uint32_t address, uint32_t expected_value,
		       uint32_t expected_mask, uint32_t timeout_ms, uint8_t id)
{
	unsigned int i;
	int ret;

	ret = -EBUSY;
	for (i = 0; i < timeout_ms; i++) {
		if (mlx5_devx_regex_register_read(ctx, id, address, value))
			return -1;

		if ((*value & expected_mask) == expected_value) {
			ret = 0;
			break;
		}
		rte_delay_us(1000);
	}
	return ret;
}

/**
 * Start the selected engine.
 *
 * @param ctx
 *   The IBV context.
 * @param id
 *   The selected engine.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
static int
rxp_start_engine(struct ibv_context *ctx, uint8_t id)
{
	uint32_t ctrl;
	int ret;

	ret = mlx5_devx_regex_register_read(ctx, id, MLX5_RXP_CSR_CTRL, &ctrl);
	if (ret)
		return ret;
	ctrl |= MLX5_RXP_CSR_CTRL_GO;
	ret = mlx5_devx_regex_register_write(ctx, id, MLX5_RXP_CSR_CTRL, ctrl);
	return ret;
}

/**
 * Stop the selected engine.
 *
 * @param ctx
 *   The IBV context.
 * @param id
 *   The selected engine.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
static int
rxp_stop_engine(struct ibv_context *ctx, uint8_t id)
{
	uint32_t ctrl;
	int ret;

	ret = mlx5_devx_regex_register_read(ctx, id, MLX5_RXP_CSR_CTRL, &ctrl);
	if (ret)
		return ret;
	ctrl &= ~MLX5_RXP_CSR_CTRL_GO;
	ret = mlx5_devx_regex_register_write(ctx, id, MLX5_RXP_CSR_CTRL, ctrl);
	return ret;
}

static int
rxp_init_rtru(struct ibv_context *ctx, uint8_t id, uint32_t init_bits)
{
	uint32_t ctrl_value;
	uint32_t poll_value;
	uint32_t expected_value;
	uint32_t expected_mask;
	int ret = 0;

	/* Read the rtru ctrl CSR */
	ret = mlx5_devx_regex_register_read(ctx, id, MLX5_RXP_RTRU_CSR_CTRL,
					    &ctrl_value);
	if (ret)
		return -1;
	/* Clear any previous init modes */
	ctrl_value &= ~(MLX5_RXP_RTRU_CSR_CTRL_INIT_MODE_MASK);
	if (ctrl_value & MLX5_RXP_RTRU_CSR_CTRL_INIT) {
		ctrl_value &= ~(MLX5_RXP_RTRU_CSR_CTRL_INIT);
		mlx5_devx_regex_register_write(ctx, id, MLX5_RXP_RTRU_CSR_CTRL,
					       ctrl_value);
	}
	/* Set the init_mode bits in the rtru ctrl CSR */
	ctrl_value |= init_bits;
	mlx5_devx_regex_register_write(ctx, id, MLX5_RXP_RTRU_CSR_CTRL,
				       ctrl_value);
	/* Need to sleep for a short period after pulsing the rtru init bit.  */
	rte_delay_us(20000);
	/* Poll the rtru status CSR until all the init done bits are set. */
	DRV_LOG(DEBUG, "waiting for RXP rule memory to complete init");
	/* Set the init bit in the rtru ctrl CSR. */
	ctrl_value |= MLX5_RXP_RTRU_CSR_CTRL_INIT;
	mlx5_devx_regex_register_write(ctx, id, MLX5_RXP_RTRU_CSR_CTRL,
				       ctrl_value);
	/* Clear the init bit in the rtru ctrl CSR */
	ctrl_value &= ~MLX5_RXP_RTRU_CSR_CTRL_INIT;
	mlx5_devx_regex_register_write(ctx, id, MLX5_RXP_RTRU_CSR_CTRL,
				       ctrl_value);
	/* Check that the following bits are set in the RTRU_CSR. */
	if (init_bits == MLX5_RXP_RTRU_CSR_CTRL_INIT_MODE_L1_L2) {
		/* Must be incremental mode */
		expected_value = MLX5_RXP_RTRU_CSR_STATUS_L1C_INIT_DONE |
				 MLX5_RXP_RTRU_CSR_STATUS_L2C_INIT_DONE;
	} else {
		expected_value = MLX5_RXP_RTRU_CSR_STATUS_IM_INIT_DONE |
				 MLX5_RXP_RTRU_CSR_STATUS_L1C_INIT_DONE |
				 MLX5_RXP_RTRU_CSR_STATUS_L2C_INIT_DONE;
	}
	expected_mask = expected_value;
	ret = rxp_poll_csr_for_value(ctx, &poll_value,
				     MLX5_RXP_RTRU_CSR_STATUS,
				     expected_value, expected_mask,
				     MLX5_RXP_CSR_STATUS_TRIAL_TIMEOUT, id);
	if (ret)
		return ret;
	DRV_LOG(DEBUG, "rule Memory initialise: 0x%08X", poll_value);
	/* Clear the init bit in the rtru ctrl CSR */
	ctrl_value &= ~(MLX5_RXP_RTRU_CSR_CTRL_INIT);
	mlx5_devx_regex_register_write(ctx, id, MLX5_RXP_RTRU_CSR_CTRL,
				       ctrl_value);
	return 0;
}

#if 0
static int
rxp_parse_rof(const char *buf, uint32_t len, struct mlx5_rxp_ctl_rules_pgm **rules)
{
	const char del[] = "\n\r";
	char *line;
	char *tmp;
	char *cur_pos;
	uint32_t lines;
	uint32_t entries;
	struct mlx5_rxp_rof_entry *curentry;

	tmp = malloc(len);
	if (!tmp)
		return -ENOMEM;
	memcpy(tmp, buf, len);
	line = strtok(tmp, del);
	while (line) {
		if (line[0] != '#' && line[0] != '\0')
			lines++;
		line = strtok(NULL, del);
	}
	*rules = rte_malloc("", lines * sizeof(*curentry) + sizeof(**rules), 0);
	if (!(*rules)) {
		free(tmp);
		return -ENOMEM;
	}
	memset(*rules, 0, lines * sizeof(curentry) + sizeof(**rules));
	curentry = (*rules)->rules;
	(*rules)->hdr.cmd = MLX5_RXP_CTL_RULES_PGM;
	entries = 0;
	memcpy(tmp, buf, len);
	line = strtok(tmp, del);
	while (line) {
		if (line[0] == '#' || line[0] == '\0') {
			line = strtok(NULL, del);
			continue;
		}
		line = strtok(NULL, del);
		curentry->type = strtoul(line, &cur_pos, 10);
		if (cur_pos == line || cur_pos[0] != ',')
			goto parse_error;
		cur_pos++;
		curentry->addr = strtoul(line, &cur_pos, 16);
		if (cur_pos[0] != ',')
			goto parse_error;
		cur_pos++;
		curentry->value = strtoul(line, &cur_pos, 16);
		if (cur_pos[0] != '\0' && cur_pos[0] != '\n')
			goto parse_error;
		curentry++;
		entries++;
		if (entries > lines)
			goto parse_error;
		line = strtok(NULL, del);
	}
	(*rules)->count = entries;
	(*rules)->hdr.len = entries * sizeof(*curentry) + sizeof(**rules);
	free(tmp);
	return 0;
parse_error:
	free(tmp);
	if (*rules)
		rte_free(*rules);
	return -EINVAL;
}
static enum mlx5_rxp_program_mode
rxp_prog_mode_get(void)
{
    return MLX5_RXP_PRIVATE_PROG_MODE;
}
#endif

static int __rte_unused
mlnx_update_database(struct mlx5_regex_db *db __rte_unused,
		     uint16_t cmd __rte_unused, uint8_t id __rte_unused)
{
#if 0
	unsigned int i;
	uint8_t db_free = MLX5_RXP_MAX_NOT_USED;
	uint8_t rxp_eng_currently_assigned = MLX5_RXP_MAX_NOT_USED;

	/* Check which database rxp_eng is currently located if any? */
	for (i = 0; i < (MLX5_RXP_MAX_ENGINES + MLX5_RXP_SHADOW_EM_COUNT); i++)
	{
		if (db[i].db_assigned_to_eng_num == id)
		{
			rxp_eng_currently_assigned = i;
			break;
		}
	}
	/*
     * If private mode then, we can keep the same db ptr as RXP will be
     * programming EM itself if necessary, however need to see if programmed yet
     */
	if ((RXP_PRIVATE_PROG_MODE == mlnx_prog_mode_get()) &&
	    (rxp_eng_currently_assigned != RXP_MAX_NOT_USED))
	{
		return rxp_eng_currently_assigned;
	}

	/* TODO ensure set all DB memory to 0xff before setting db up! */

	/* Check for inactive db memory to use */
	for (i = 0; i < (MAX_RXP_ENGINES + RXP_SHADOW_EM_COUNT); i++)
	{
		if (rxp.rxp_db_desc[i].db_active == false)
		{
			/* Set this db to active now as free to use */
			rxp.rxp_db_desc[i].db_active = true;

			/* Now unassign last db index in use by RXP Eng if any? */
			if (rxp_eng_currently_assigned != RXP_MAX_NOT_USED)
			{
				rxp.rxp_db_desc[rxp_eng_currently_assigned].db_active = false;
				rxp.rxp_db_desc[rxp_eng_currently_assigned].db_assigned_to_eng_num = RXP_MAX_NOT_USED;

				/* Set all DB memory to 0x00 before setting up Database */
				memset(rxp.rxp_db_desc[i].database_ptr, 0x00, MAX_DB_SIZE);
			}

			/* Now reassign new db index with RXP Eng */
			rxp.rxp_db_desc[i].db_assigned_to_eng_num = rxp_eng;

			db_free = i;
			break;
		}
	}

	if (db_free == RXP_MAX_NOT_USED)
	{
		return -1;
	}

#endif
	return 0;
}

static int
rxp_program_rules(struct mlx5_regex_priv *priv __rte_unused,
		  const char *rule_buf __rte_unused, uint32_t len __rte_unused)
{
#if 0
	struct ibv_context *ctx = priv->ctx;
	struct mlx5_rxp_ctl_rules_pgm *rules = NULL;
	int ret;
	uint32_t rule_cnt;

	if (len ==0 || !rule_buf)
		return -EINVAL;
	ret = rxp_parse_rof(rule_buf, len, &rules);
	if (ret) {
		DRV_LOG(ERR, "can't parse ROF file.");
		returnn ret;
	}
	rule_cnt = rules->count;
	/* Program both RXP's with the following rules...*/
	for (i = 0; i < MLX5_RXP_MAX_ENGINES; i++)
	{
		if (MLX5_RXP_PRIVATE_PROG_MODE == rxp_prog_mode_get())
		{
			/*
        		 * Need to set the mlnx_set_database immediately as when
			 * we start pushing instructions to RXP, we need to be
			 * sure the RXP has the capability to write to
			 * Shared/External memory!
        		 */
			db_free = mlnx_update_database(priv->db,rules->hdr.cmd,
						       i);
			if (db_free < 0)
			{
				/* Failed to find free database to use */
				mlnx_log("Failed to setup new database memory - Error [%d]!\n",
					 db_free);
				return db_free;
			}

			/*
             * TODO: Note: As unsure what mlx5_regex_database_set() function
             *       actually does, I'm setting up db pointer early here before
             *       actually doing any private rule writes. Notably when start
             *       pushing rules to RXP this database must be setup else
             *       probably run into RXP write errors etc!??
             */
			ret = mlnx_set_database(i, db_free); //Ensure RXP Eng idle before run

			if (ret < 0)
			{
				/* Failed to set/register database with Mellanox */
				mlnx_log("Failed to reg database with Mellanox - Error [%d]!\n",
					 ret);
				return ret;
			}

			ret = mlnx_write_rules(rules, rules->hdr.len, i);

			if (ret < 0)
			{
				/* Failed to program rules */
				mlnx_log("Failed to write rules to RXP - Error [%d]!\n", ret);
				return ret;
			}

			mlnx_log("Info: Programmed RXP Eng %d - mlnx_init!\n", i);
		}
		else if (RXP_SHARED_PROG_MODE == mlnx_prog_mode_get())
		{
			/* Write all External Rules from rules file into shared/EM memory */

			/* Writing External/Shared rules first as can keep RXP running*/
			db_free = mlnx_update_database(rules->hdr.cmd, i);

			if (db_free < 0)
			{
				/* Failed to find free database to use */
				mlnx_log("Failed to setup new database memory - Error [%d]!\n",
					 db_free);
				return db_free;
			}

			/*
             * NOTE: TODO: Might speed up the 2nd programming by copying RXP
             *             ENG0 rules into RXP Eng1 instead of looping rules?
             */

			/* Now write rules first before taking RXP eng offline */
			ret = mlnx_write_shared_rules(rules, rules->hdr.len, i, db_free);

			if (ret < 0)
			{
				/* Failed to write new rules to EM */
				mlnx_log("Failed to write rules to RXP - Error [%d]!\n", db_free);
				return db_free;
			}

			/*
             * Now inform Mlnx that db has changed address and RXP idle.
             * The following call will block until RXPx is idle, it effectively
             * disables this engine until resumed later following programming.
             */
			ret = mlnx_set_database(i, db_free);

			if (ret < 0)
			{
				/* Failed to set/register database with Mellanox */
				mlnx_log("Failed to reg database with Mellanox - Error [%d]!\n",
					 ret);
				return ret;
			}

			/* Finally write any internal private rules to RXP */
			ret = mlnx_write_rules(rules, rules->hdr.len, i);

			if (ret < 0)
			{
				/* Failed to program rules */
				mlnx_log("Failed to write rules to RXP - Error [%d]!\n", ret);
				return ret;
			}
		}
		else
		{
			/* Failed to read valid programming mode */
			mlnx_log("Failed to read valid RXP programming mode: Mode [%d]\n",
				 mlnx_prog_mode_get());
			return -1;
		}

		/*
         * Need to start RXP engine again after programming has
         * finished successfully.
         */
		mlnx_resume_rxp(i);

		/* A bit of a fudge! But ensure we keep rule cnt valid for both RXPs */
		rules->count = keep_rule_count;
	}

	/* Both RXP's now programmed */
	rxp_prog_status = true;

	free(rules);

	return ret;
#endif
	return 0;
}

/**
 * Init the engine.
 *
 * @param ctx
 *   The IBV context.
 * @param id
 *   The selected engine.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
static int
rxp_init(struct mlx5_regex_priv *priv, uint8_t id)
{
	uint32_t ctrl;
	uint32_t reg;
	struct ibv_context *ctx = priv->ctx;
	int ret;

	ret = mlx5_devx_regex_register_read(ctx, id, MLX5_RXP_CSR_CTRL, &ctrl);
	if (ret)
		return ret;
	if (ctrl & MLX5_RXP_CSR_CTRL_INIT) {
		ctrl &= ~MLX5_RXP_CSR_CTRL_INIT;
		ret = mlx5_devx_regex_register_write(ctx, id, MLX5_RXP_CSR_CTRL,
						     ctrl);
		if (ret)
			return ret;
	}
	ctrl |= MLX5_RXP_CSR_CTRL_INIT;
	ret = mlx5_devx_regex_register_write(ctx, id, MLX5_RXP_CSR_CTRL, ctrl);
	if (ret)
		return ret;
	ctrl &= ~MLX5_RXP_CSR_CTRL_INIT;
	ret = mlx5_devx_regex_register_write(ctx, id, MLX5_RXP_CSR_CTRL, ctrl);
	rte_delay_us(20000);

	ret = rxp_poll_csr_for_value(ctx, &ctrl, MLX5_RXP_CSR_STATUS,
				     MLX5_RXP_CSR_STATUS_INIT_DONE,
				     MLX5_RXP_CSR_STATUS_INIT_DONE,
				     MLX5_RXP_CSR_STATUS_TRIAL_TIMEOUT, id);
	if (ret)
		return ret;
	ret = mlx5_devx_regex_register_read(ctx, id, MLX5_RXP_CSR_CTRL, &ctrl);
	if (ret)
		return ret;
	ctrl &= ~MLX5_RXP_CSR_CTRL_INIT;
	ret = mlx5_devx_regex_register_write(ctx, id, MLX5_RXP_CSR_CTRL,
					     ctrl);
	if (ret)
		return ret;
	rxp_init_rtru(ctx, id, MLX5_RXP_RTRU_CSR_CTRL_INIT_MODE_IM_L1_L2);
	ret = rxp_init_rtru(ctx, id, MLX5_RXP_RTRU_CSR_CTRL_INIT_MODE_IM_L1_L2);
	if (ret)
		return ret;
	ret = mlx5_devx_regex_register_read(ctx, id, MLX5_RXP_CSR_CAPABILITY_5,
					    &reg);
	if (ret)
		return ret;
	DRV_LOG(DEBUG, "max matches: %d, DDOS threshold: %d", reg >> 16,
		reg & 0xffff);
	ret = mlx5_devx_regex_register_write(ctx, id, MLX5_RXP_CSR_MAX_MATCH,
					     priv->nb_max_matches);
	ret |= mlx5_devx_regex_register_write(ctx, id,
					      MLX5_RXP_CSR_MAX_LATENCY, 0);
	ret |= mlx5_devx_regex_register_write(ctx, id,
					      MLX5_RXP_CSR_MAX_PRI_THREAD, 0);
	return ret;
}

/**
 * DPDK callback for reading device info.
 *
 * @param dev
 *   Pointer to RegEx device structure.
 * @param[in] cfg
 *   Pointer to the regexdev device configuration structure.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
int
mlx5_regex_configure(struct rte_regexdev *dev,
		     const struct rte_regexdev_config *cfg)
{
	struct mlx5_regex_priv *priv = dev->data->dev_private;
	int ret;
	uint8_t id;

	priv->nb_queues = cfg->nb_queue_pairs;
	priv->qps = rte_zmalloc(NULL, sizeof(struct mlx5_regex_qp) *
				priv->nb_queues, 0);
	if (!priv->nb_queues) {
		DRV_LOG(ERR, "can't allocate qps memory");
		rte_errno = ENOMEM;
		return -rte_errno;
	}
	priv->nb_max_matches = cfg->nb_max_matches;
	for (id = 0; id < 2; id++) {
		ret = rxp_stop_engine(priv->ctx, id);
		if (ret) {
			DRV_LOG(ERR, "can't stop engine.");
			rte_errno = ENODEV;
			return -rte_errno;
		}
		ret = rxp_init(priv, id);
		if (ret) {
			DRV_LOG(ERR, "can't init engine.");
			rte_errno = ENODEV;
			return -rte_errno;
		}
		ret = mlx5_devx_regex_register_write(priv->ctx, id,
						     MLX5_RXP_CSR_MAX_MATCH,
						     priv->nb_max_matches);
		if (ret) {
			DRV_LOG(ERR, "can't update number of matches.");
			rte_errno = ENODEV;
			goto configure_error;
		}
		ret = rxp_start_engine(priv->ctx, id);
		if (ret) {
			DRV_LOG(ERR, "can't start engine.");
			rte_errno = ENODEV;
			goto configure_error;
		}

	}
	rxp_program_rules(priv, cfg->rule_db, cfg->rule_db_len);
	return 0;
configure_error:
	if (priv->qps)
		rte_free(priv->qps);
	return -rte_errno;
}
