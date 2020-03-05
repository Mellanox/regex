/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2016-2017 Intel Corporation
 */

#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include <rte_malloc.h>
#include <rte_random.h>
#include <rte_eal.h>
#include <rte_regexdev.h>

static volatile int force_quit;

static void
main_loop(void)
{
	struct rte_regex_ops *ops[2];
	struct rte_regex_iov *iov[1];
	struct rte_regex_iov *iov2[1];
	char buf[100];
	int i;

	sprintf(buf, " hello world                          ");
	sprintf(buf + 15, "hello world");
	printf("oooOri size = %ld\n",sizeof(*ops[0]));
	iov[0] = rte_malloc(NULL, sizeof(*iov[0]), 0);
	iov[0]->buf_addr = buf;
	iov[0]->buf_size = 20;
	ops[0] = rte_malloc(NULL, sizeof(*ops[0]) +
			    sizeof(struct rte_regex_match) * 255, 0);
	ops[0]->num_of_bufs = 1;
	ops[0]->bufs = &iov;
	ops[0]->group_id0 = 1;
	iov2[0] = rte_malloc(NULL, sizeof(*iov[0]), 0);
	iov2[0]->buf_addr = buf;
	iov2[0]->buf_size = 100;
	ops[1] = rte_malloc(NULL, sizeof(*ops[0]) +
			    sizeof(struct rte_regex_match) * 255, 0);
	ops[1]->num_of_bufs = 1;
	ops[1]->bufs = &iov2;
	ops[1]->group_id0 = 1;
	//while (!force_quit) {
		rte_regex_enqueue_burst(0, 0, ops, 2);
		rte_regex_dequeue_burst(0, 0, ops, 2);
		printf("--1 number of matches = %d\n",ops[0]->nb_matches);
		printf("--2 number of matches = %d\n",ops[1]->nb_matches);
		for (i =0; i < ops[0]->nb_matches; i++)
			printf("--1 found start %d, len %d, id %d\n",
			      ops[0]->matches[i].offset, ops[0]->matches[i].len, ops[0]->matches[i].rule_id);
		for (i =0; i < ops[1]->nb_matches; i++)
			printf("--2 found start %d, len %d, id %d\n",
			      ops[1]->matches[i].offset, ops[1]->matches[i].len, ops[1]->matches[i].rule_id);
	//}
	printf("%s\n", buf);
	/* closing and releasing resources */
}

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

	/* Initialise DPDK EAL */
	ret = rte_eal_init(argc, argv);
	if (ret < 0)
		rte_exit(EXIT_FAILURE, "Invalid EAL arguments!\n");

	force_quit = 0;
	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	dev_count = rte_regex_dev_count();
	printf("regex devices = %d\n", dev_count);

	for (i = 0; i < dev_count; i++) {
		ret = setup_dev_one(i);
		if (ret)
			break;
	}
	main_loop();
	return ret;
}
