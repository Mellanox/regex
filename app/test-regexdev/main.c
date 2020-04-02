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

static volatile int force_quit;

#define TOTAL_JOBS 400
#define STRING_LEN 16000

struct sw_job {
	struct rte_regex_iov (*iov[1]);
};
static void
main_loop(void)
{
	struct rte_regex_ops *ops[TOTAL_JOBS];
	char buf[STRING_LEN];
	int i;

	for (i = 0; i < STRING_LEN; i++)
		buf[i] = '.';
	buf[STRING_LEN-1] = 0;	
	const char *match_str = "hello world";
	for (i = 0; i < 10; i++) {
		int offset = rand()%(STRING_LEN - 50);
		strncpy(buf + offset, match_str, strlen(match_str));
	}

	sprintf(buf + STRING_LEN -strlen(match_str)-1, "%s", match_str);
	
	struct sw_job sw_jobs[TOTAL_JOBS];

	//printf("Posting %d random jobs on buffer %s \n", TOTAL_JOBS, buf);
	for (i = 0; i < TOTAL_JOBS; i++) {
		ops[i] = rte_malloc(NULL, sizeof(*ops[0]), 0);
   	        sw_jobs[i].iov[0] = rte_malloc(NULL, sizeof(* sw_jobs[i].iov[0]), 0);
		ops[i]->bufs = &sw_jobs[i].iov;
		ops[i]->num_of_bufs = 1;
		ops[i]->group_id0 = 1;
		ops[i]->user_id = i;
		int offset = rand()%(STRING_LEN - 50);
		(*ops[i]->bufs)[0]->buf_addr = buf + offset;
		(*ops[i]->bufs)[0]->buf_size = (STRING_LEN - offset);
	}
	
	int sent = 0, total_sent=0;
	while (total_sent < TOTAL_JOBS) {
		sent = rte_regex_enqueue_burst(0, 0, ops + total_sent, TOTAL_JOBS - total_sent);
		printf("sent = %d\n", sent);
		int done = 0;
		while(done < sent) {
			int d = rte_regex_dequeue_burst(0, 0, ops, sent);
			for (i = 0; i < d; i++) {
				printf("[%d] [%ld] number of matches = %d\n",i,ops[i]->user_id, ops[i]->nb_matches);
			}
			done += d;
		}
		total_sent += sent;
	}
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

	// do this until we understand how to add options.
	const char *rules_db_file = argv[1];

	/* Initialise DPDK EAL */
	ret = rte_eal_init(argc-1, argv+1);
	if (ret < 0) {
		rte_exit(EXIT_FAILURE, "Invalid EAL arguments!, usage: dpdk-test-regexdev <rof2 file> [dpdk options]\n");
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

	rte_regex_rule_db_import(i, rules_db_file);
	main_loop();
	return ret;
}
