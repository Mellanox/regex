/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2016-2017 Intel Corporation
 */

#include <stdio.h>
#include <unistd.h>

#include <rte_malloc.h>
#include <rte_random.h>
#include <rte_eal.h>
#include <rte_regexdev.h>

static int setup_dev_one(int dev_id)
{
	struct rte_regex_dev_config dev_cfg;
	struct rte_regex_dev_info dev_info;
	int ret;

	ret = rte_regex_dev_info_get(dev_id, &dev_info);
	if (ret)
		return ret;

	printf("max_qps = %d\n", dev_info.max_queue_pairs);
	printf("max_sges = %d\n", dev_info.max_scatter_gather);

	dev_cfg.nb_queue_pairs = dev_info.max_queue_pairs;

	ret = rte_regex_dev_configure(dev_id, &dev_cfg);
	return ret;
}

int main(int argc, char **argv)
{
	uint8_t dev_count;
	int ret;
	int i;

	/* Initialise DPDK EAL */
	ret = rte_eal_init(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Invalid EAL arguments!\n");

	dev_count = rte_regex_dev_count();
	printf("regex devices = %d\n", dev_count);

	for (i = 0; i < dev_count; i++) {
		ret = setup_dev_one(i);
		if (ret)
			break;
	}

	return ret;
}
