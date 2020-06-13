/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2016-2017 Intel Corporation
 */

#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

#include <rte_malloc.h>
#include <rte_random.h>
#include <rte_eal.h>
#include <rte_regexdev.h>

#include "tests.h"

static volatile int force_quit;

#define TOTAL_JOBS 4
#define STRING_LEN 16000

static int
setup_dev_one(int dev_id)
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

static void
signal_handler(int signum)
{
	if (signum == SIGINT || signum == SIGTERM) {
		printf("\n\nSignal %d received, preparing to exit...\n",
				signum);
		force_quit = 1;
	}
}

int
main(int argc, char **argv)
{
	uint8_t dev_count;
	int ret;
	int i;

	// do this until we understand how to add options.
	const char *rules_db_file = argv[1];
	const char *match_str = argv[2];
	const char *txt_file = argv[3];

	/* Initialise DPDK EAL */
	ret = rte_eal_init(argc - 3, argv + 3);
	if (ret < 0) {
		rte_exit(EXIT_FAILURE, "Invalid EAL arguments!, usage: dpdk-test-regexdev [dpdk options]\n");
	}

	force_quit = 0;
	if (0) {signal(SIGINT, signal_handler);
		signal(SIGTERM, signal_handler);
	}

	dev_count = rte_regex_dev_count();
	printf("regex devices = %d\n", dev_count);

	for (i = 0; i < dev_count; i++) {
		ret = setup_dev_one(i);
		if (ret)
			break;
	}
	
	rte_regex_rule_db_import(0, rules_db_file);

	ret |= mlx5_regex_simple_test(1, 0, 1, match_str);
	ret |= mlx5_regex_simple_test(1, 0, 10, match_str);
	ret |= mlx5_regex_simple_test(1, 0, 100, match_str);
	ret |= mlx5_regex_simple_test(1, 1, 1, match_str);
	ret |= mlx5_regex_simple_test(10, 1, 1, match_str);
	ret |= mlx5_regex_simple_test(100, 1, 1, match_str);
	ret |= mlx5_regex_simple_test(10, 1, 10, match_str);
	ret |= mlx5_regex_simple_test(100, 1, 100, match_str);
	ret |= mlx5_regex_simple_test(100, 1, 10000, match_str);

	ret |= mlx5_regex_perf_test(16*16, match_str);

	ret |= mlx5_regex_perf_test_file(16*16*4, txt_file);
	return ret;
}
