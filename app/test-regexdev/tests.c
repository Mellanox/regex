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

void print_raw(void*,size_t);

#define MAX_BUF_SZ (1<<14)

struct sw_job {
	struct rte_regex_iov (*iov[1]);
};

static uint32_t
get_expected(struct rte_regex_match* expected_matches, const char *match_str,
		  char* buf, size_t size, char** positions, size_t num_pos) 
{
	size_t i;
	uint32_t matches = 0;
	for (i=0 ; i < num_pos; i++) {
		if ((positions[i] >= buf) &&
		    ((positions[i] + strlen(match_str)) <= (buf + size))) {
			expected_matches[matches].group_id = 1;
			expected_matches[matches].len = strlen(match_str);
			expected_matches[matches].offset = positions[i] - buf;
			matches++;
		}
	}
	return matches;
}

#define PRINT_DIFF(TYPE, ACTUAL, EXPECTED, ENTRY) if (ACTUAL != EXPECTED) {diff = 1; printf("[%ld] %s actual %d, expected %d\n" ,ENTRY, #TYPE, ACTUAL, EXPECTED);}
int mlx5_regex_simple_test(size_t num_str, int rand_pos, size_t num_jobs,
			   const char *match_str)
{
	struct rte_regex_ops **ops = rte_zmalloc(NULL, sizeof(*ops)*num_jobs, 0);
	struct sw_job *sw_jobs = rte_zmalloc(NULL, sizeof(*sw_jobs)*num_jobs, 0);
	char **positions = rte_zmalloc(NULL, sizeof(char*)*num_str, 0);
	struct rte_regex_match **expected_matches;
	size_t total_matches_expected = 0;
	char buf[MAX_BUF_SZ];
	size_t i, num_pos = 0;
	size_t max_matches;
	int ret = 0;

	expected_matches = rte_zmalloc(NULL, sizeof(*ops)*num_jobs, 0);
	max_matches = 254; //TODO: take from attr
	memset(buf, 0, MAX_BUF_SZ);
	uint32_t str_offset = 0;
	for (i=0; i < num_str; i++) { 
		str_offset += strlen(match_str) + rand()%(MAX_BUF_SZ/10 - 2*strlen(match_str) - str_offset);
		if (str_offset > (MAX_BUF_SZ - strlen(match_str)))
			break;
		strncpy(buf + str_offset, match_str, strlen(match_str));
		positions[i] = buf + str_offset;
		//printf("Position = %d \n", str_offset);
		num_pos++;
	}
	printf("Running test with %ld strings of \"%s\", num jobs=%ld, rand=%d\n", num_pos, match_str, num_jobs, rand_pos);
	for (i = 0; i < num_jobs; i++) {
		ops[i] = rte_zmalloc(NULL, sizeof(*ops[0]) + sizeof(struct rte_regex_match)*max_matches, 0);
   	    	sw_jobs[i].iov[0] = rte_zmalloc(NULL, sizeof(* sw_jobs[i].iov[0]), 0);
		ops[i]->bufs = &sw_jobs[i].iov;
		ops[i]->num_of_bufs = 1;
		ops[i]->group_id0 = 1;
		ops[i]->user_id = i;
		int offset = rand_pos ? rand()%(MAX_BUF_SZ/10) : 0;
		(*ops[i]->bufs)[0]->buf_addr = buf + offset;
		(*ops[i]->bufs)[0]->buf_size = rand()%(MAX_BUF_SZ - offset);
		if ((*ops[i]->bufs)[0]->buf_size == 0) {
			(*ops[i]->bufs)[0]->buf_size = 1;
		}

		expected_matches[i] = rte_zmalloc(NULL, sizeof(struct rte_regex_match)*max_matches, 0);
		total_matches_expected += get_expected(expected_matches[i], match_str,
			     (*ops[i]->bufs)[0]->buf_addr,
			     (*ops[i]->bufs)[0]->buf_size,
			     positions, num_pos);
		//printf("[%ld] offset = %d, buf_size=%d\n",i, offset, (*ops[i]->bufs)[0]->buf_size);
	}

	printf("\nStarting test..\n");
	
	size_t sent = 0, total_sent=0, total_matches=0, m=0;
	while (total_sent < num_jobs) {
		struct rte_regex_ops **cur_ops = ops + total_sent;
		sent = rte_regex_enqueue_burst(0, 0, cur_ops, num_jobs - total_sent);
		size_t done = 0;
		while(done < sent) {
			size_t d = rte_regex_dequeue_burst(0, 0, cur_ops, sent);
			for (i = 0; i < d; i++) {
				int diff = 0;
				total_matches += cur_ops[i]->nb_matches;
				for ( m = 0; m < cur_ops[i]->nb_matches; m++) {
					PRINT_DIFF(group_id, 	cur_ops[i]->matches[m].group_id,   	expected_matches[cur_ops[i]->user_id][m].group_id, m);
					PRINT_DIFF(len, 		cur_ops[i]->matches[m].len,		expected_matches[cur_ops[i]->user_id][m].len, m);
					PRINT_DIFF(offset , 	cur_ops[i]->matches[m].offset, 	expected_matches[cur_ops[i]->user_id][m].offset, m);
					
				}
				if (diff) {
					printf("Job %ld has diffs\n", i+total_sent);

					ret = -1;
					getchar();
				}
			}
			done += d;
		}
		total_sent += sent;

	}

	if (total_matches != total_matches_expected) {
		printf("Error: matches expected %ld, actual %ld\n", total_matches_expected, total_matches);
		getchar();
		ret = -1;
	}
	if (ret < 0)
		printf("test FAILED\n");
	return ret;
}

int mlx5_regex_perf_test(size_t burst, const char* match_str)
{
	struct rte_regex_ops **ops = rte_zmalloc(NULL, sizeof(*ops)*burst, 0);
	struct sw_job *sw_jobs = rte_zmalloc(NULL, sizeof(*sw_jobs)*burst, 0);
	char buf[MAX_BUF_SZ];
	size_t i, b;
	size_t max_matches;
	int ret = 0;

	max_matches = 254; //TODO: take from attr

	memset(buf, 0, MAX_BUF_SZ);
	strncpy(buf, match_str, strlen(match_str));
	
	for (b = 9; b <= 14; b++) {
		
		size_t buf_size = 1 << b;
		printf("Runing performance test for %ld byte buffers\n", buf_size);
		for (i = 0; i < burst; i++) {
			ops[i] = rte_zmalloc(NULL, sizeof(*ops[0]) + sizeof(struct rte_regex_match)*max_matches, 0);
	   	    	sw_jobs[i].iov[0] = rte_zmalloc(NULL, sizeof(* sw_jobs[i].iov[0]), 0);
			ops[i]->bufs = &sw_jobs[i].iov;
			ops[i]->num_of_bufs = 1;
			ops[i]->group_id0 = 1;
			ops[i]->user_id = i;
			(*ops[i]->bufs)[0]->buf_addr = buf;
			(*ops[i]->bufs)[0]->buf_size = buf_size;
		}
	
		time_t start = clock();
		size_t total_jobs = (1<<25);
		size_t j, total_done=0, empty_polls=0, done, total_polls=0, total_sent=0;
		for (j = 0; j < total_jobs; j+=burst) {
			total_sent += rte_regex_enqueue_burst(0, 0, ops, burst);
			done = rte_regex_dequeue_burst(0, 0, ops, burst);
			empty_polls += (done ? 0 : 1);
			total_polls++;
			total_done +=done;
		}
		while (total_sent > total_done) {
			done = rte_regex_dequeue_burst(0, 0, ops, burst);
			empty_polls += (done ? 0 : 1);
			total_polls++;
			total_done +=done;
		}
		time_t end = clock();
		double time = ((double)end-start)/CLOCKS_PER_SEC;
        printf("Empty polls=%ld, Total non-empty polls =%ld, total_sent=%ld\n", empty_polls, total_polls, total_sent);
        printf("Time = %lf sec\n",  time);
        printf("Perf = %lf Gbps\n", (((double)buf_size*total_done*8)/time)/1000000000.0);
	}
	
	if (ret < 0)
		printf("test FAILED\n");
	return ret;
}
