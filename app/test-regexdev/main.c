/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2016-2017 Intel Corporation
 */

#include <stdio.h>
#include <unistd.h>

#include <rte_malloc.h>
#include <rte_mbuf.h>
#include <rte_mempool.h>
#include <rte_random.h>
#include <rte_eal.h>
#include <rte_regexdev.h>

static int setup_and_test_dev_one(int dev_id)
{
	struct rte_regex_dev_config dev_cfg;
	struct rte_regex_dev_info dev_info;
	struct rte_mempool *rte_mp = NULL;
	struct rte_regex_qp_conf qp_conf;
	struct rte_regex_ops *ops;
	int ret;
	int i;
	int j;

	ret = rte_regex_dev_info_get(dev_id, &dev_info);
	if (ret)
		return ret;

	printf("max qps=%d\n", dev_info.max_queue_pairs);
	printf("max sges=%d\n", dev_info.max_scatter_gather);

	dev_cfg.nb_queue_pairs = dev_info.max_queue_pairs;

	ret = rte_regex_dev_configure(dev_id, &dev_cfg);
	if (ret)
		return ret;

	qp_conf.nb_desc = 128;
	for (i = 0; i < dev_info.max_queue_pairs; i++) {
		ret = rte_regex_queue_pair_setup(dev_id, i, &qp_conf);
		if (ret)
			printf("dev_id=%d qp_id=%d ret=%d\n", dev_id, i, ret);
	}
	printf("created qps=%d nb_desc=%d\n", ret ? i - 1 : i, qp_conf.nb_desc);

	ops = rte_calloc("ops", 1,
			 sizeof(*ops) + 255 * sizeof(struct rte_regex_match), 0);
	if (!ops)
		return ENOMEM;

	rte_mp = rte_pktmbuf_pool_create("test-regexdev-pool", 2, 0, 0, 16384,
					 SOCKET_ID_ANY);
	if (!rte_mp)
		return ENOMEM;
	rte_mempool_obj_iter(rte_mp, rte_pktmbuf_init, NULL);
	ops[0].bufs = rte_pktmbuf_alloc(rte_mp);
	if (!ops[0].bufs)
		return ENOMEM;
	for (i = 0; i < dev_cfg.nb_queue_pairs; i++) {
		printf("Test enq/deq qp_id=%d nb_desc=%d n_iter=%d\n",
		       i, qp_conf.nb_desc, qp_conf.nb_desc * 3);
		for (j = 0; j < qp_conf.nb_desc * 3; j++) {
			ops[0].user_id = j;
			ops[0].num_of_bufs = 1;
			ret = rte_regex_enqueue_burst(dev_id, i,
						      (struct rte_regex_ops **)&ops, 1);
			printf("enq[%d] qp_id=%d num_enq=%d\n", j, i, ret);

			ret = rte_regex_dequeue_burst(dev_id, i,
						      (struct rte_regex_ops **)&ops, 1);
			printf("dequeue[%d] qp_id=%d num_deq=%d\n", j, i, ret);
		}
	}
	rte_pktmbuf_free(ops[0].bufs);
	rte_free(ops);
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
		ret = setup_and_test_dev_one(i);
		if (ret)
			break;
	}

	return ret;
}
