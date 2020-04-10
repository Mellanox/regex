/**
 * @file    rxp-mlnx-api.c
 * @author  Titan IC Systems <support@titanicsystems.com>
 *
 * @section DESCRIPTION
 *
 *   Minimal reference application. The RXP is initialized and the rules memories
 *   are programmed using a pre-compiled ROF file. Random length jobs are created
 *   containing the string "hello world" at a random offset. These jobs are
 *   dispatched to the RXP and responses are received. The number of jobs,
 *   responses and matches are periodically displayed along with the job bit rate.
 *   The application supports multiple queues and multiple Hyperion cards. Each
 *   queue is used to communicate using a single rx/tx queue pair.
 *
 * @section LICENSE
 *
 *   BSD LICENSE
 *
 *   Copyright (C) 2020 Titan IC Systems Ltd. All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Titan IC Systems Ltd. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <rte_malloc.h>
#include <rte_random.h>
#include <rte_eal.h>
#include <rte_regexdev.h>

#include "rxp_api_app.h"

#define UNUSED(x)           (void)(x)

struct sw_job {
    struct rte_regex_iov (*iov[1]);
};

struct rxp_response *resp_copy;
struct rte_regex_dev_config dev_cfg;
struct rte_regex_ops *ops[HRA_NUM_JOBS];
struct sw_job sw_jobs[HRA_NUM_JOBS];

static bool received_ops[HRA_NUM_JOBS];
static uint64_t hra_job_count = 0;
static uint64_t total_jobs_sent = 0;
static uint64_t total_dequeued = 0;
static uint64_t next_response_ops = 0;
static uint64_t resp_to_process_cnt = 0;
static int dev_id = 0;

/* RXP and DPDK specific initialistion. */
int rxp_platform_init_app(unsigned rxp, int argc, char *argv[])
{
    int ret, j;
    struct rte_regex_dev_info dev_info;
    uint8_t dev_count;
    uint64_t i;

    UNUSED(rxp); //Currently not used in this API
    ret = rte_eal_init(argc, argv);
    if (ret < 0)
    {
        return -1;
    }

    /* Parse application arguments (after the EAL options) */
    argc -= ret;
    argv += ret;

    //TODO add command line parameter checks below for hra job count
    if (hra_job_count <= 0)
    {
        hra_job_count = HRA_NUM_JOBS;
    }
    else if (hra_job_count > HRA_NUM_JOBS)
    {
        /* Force job count to ensure dont overflow buffers etc! */
        printf("Warning: Job number greater than Max Job allowed, so setting"
               "to default [%llu]!\n", HRA_NUM_JOBS);

        hra_job_count = HRA_NUM_JOBS;
    }

    dev_count = rte_regex_dev_count();
    printf("Info: REGEX Devices = %d\n", dev_count);

    if (dev_count == 0)
    {
        return -1;
    }

    /* Get card ID and program RXP/s */
    for (j = 0; j < dev_count; j++)
    {
        ret = rte_regex_dev_info_get(j, &dev_info);

        if (ret)
        {
            return ret;
        }

        printf("Info: max_qps = %d\n", dev_info.max_queue_pairs);
        printf("Info: max_sges = %d\n", dev_info.max_scatter_gather);

        dev_cfg.nb_queue_pairs = dev_info.max_queue_pairs;

        /* Program RXP/s */
        ret = rte_regex_dev_configure(j, &dev_cfg);

        if (ret)
        {
            dev_id = j;
            break;
        }
    }

    /* Setup/Configure DPDK operations */
    for(i = 0; i < hra_job_count; i++)
    {
        ops[i] = rte_malloc(NULL, sizeof(*ops[0]), 0);
        sw_jobs[i].iov[0] = rte_malloc(NULL, sizeof(* sw_jobs[i].iov[0]), 0);
        memset(received_ops, 0x00, hra_job_count);
    }

    /* Create rxp_response used to copy returned matches back to app*/
    resp_copy = rte_malloc(NULL, sizeof(RXP_RESP_BUF_SIZE_MIN), 0);

    return ret;
}


/* Template for platform specific un-initialistion. */
int rxp_platform_uninit_app(unsigned rxp)
{
    uint64_t i;
    int ret = 1;

    UNUSED(rxp); //Currently not used in this API

    for(i = 0; i < hra_job_count; i++)
    {
        if (ops[i] != NULL)
        {
            rte_free(ops[i]);
        }

        if (sw_jobs[i].iov[0] != NULL)
        {
	    rte_free(sw_jobs[i].iov[0]);
        }
    }

    rte_free(resp_copy);

    rte_regex_dev_stop(0);//TODO Add back in later // dev_id);
    //TODO: Validate what happens if dev_id not in use here?!
    //ret = rte_regex_dev_close(0); //TODO add back in //dev_id);

    return ret;
}

void *rxp_platform_malloc_app(const char *type, size_t size, unsigned align)
{
    return rte_malloc(type, size, align);
}

void rxp_platform_unmalloc_app(void *ptr)
{
    rte_free (ptr);
}

void *rxp_platform_calloc_app(const char *type, size_t num, size_t size, unsigned align)
{
    return rte_calloc(type, num, size, align);
}


int rxp_queue_status_app(int rxp_handle, bool *rx_ready, bool *tx_ready)
{
    int ret = 1;

    UNUSED(rxp_handle); //Currently not used in this API

    *rx_ready = true;
    *tx_ready = true;

    return ret;
}

int rxp_submit_job_app(int rxp_handle, uint32_t jobid, uint8_t *buf,
                   uint16_t len, uint16_t subset1, uint16_t subset2,
                   uint16_t subset3, uint16_t subset4, bool enable_hpm,
                   bool enable_anymatch)
{
    uint16_t num_enqueued = 0;
    uint32_t job_index;

    if ((rxp_handle < 0) || (rxp_handle > (int)RXP_NUM_QUEUES))
    {
        return -EPERM;
    }
    if (len > RXP_MAX_JOB_LENGTH)
    {
        return -EINVAL;
    }
    if ((subset1 == 0) || (jobid == 0))
    {
        return -EINVAL;
    }

    //NOTE: Note enable_hpm && enable_anymatch are not used in BF2 as only 1bit
    if (enable_hpm || enable_anymatch)
    {
        printf("Error: HPM and Anymatch not supported in BF2\n");
        return -EINVAL;
    }

    /* Manage jobid as hra-lite starts with jobid of 1. */
    job_index = jobid - 1;

    /* Populate job into DPDK ops */
    ops[job_index]->num_of_bufs = 1;
    ops[job_index]->bufs = &sw_jobs[job_index].iov;
    ops[job_index]->group_id0 = subset1;
    ops[job_index]->group_id1 = subset2;
    ops[job_index]->group_id2 = subset3;
    ops[job_index]->group_id3 = subset4;
    ops[job_index]->user_id = jobid;
    (*ops[job_index]->bufs)[0]->buf_addr = buf;
    (*ops[job_index]->bufs)[0]->buf_size = len;

    /* DPDK Job submission -HRA lite currently only submits 1 job per enqueue */
//    num_enqueued = rte_regex_enqueue_burst(dev_id, dev_cfg.nb_queue_pairs, ops + job_index, 1);
//    Note: Currently setting DEVID and QP to zero, this will need to change later
    num_enqueued = rte_regex_enqueue_burst(0, 0, ops + job_index, 1);

    if (num_enqueued < 1)
    {
        printf("Error: rxp_submit_job() failed\n");
        return -1; //TODO possibly return -BUSY here instead?
    }

    total_jobs_sent++;

    return len;
}

int rxp_read_response_batch_app(int rxp_handle, struct rxp_response_batch *ctx)
{
    int ret = 1;
    uint64_t num_dequeued = 0;

    if ((rxp_handle < 0) || (rxp_handle > (int)RXP_NUM_QUEUES))
    {
        return -EPERM;
    }

    if (total_dequeued >= total_jobs_sent)
    {
        return 1; //No responses to try and fetch
    }

    /* Clear the context. */
    ctx->buf_used = 0;
    ctx->next_offset = 0;

    //num_dequeued = rte_regex_dequeue_burst(dev_id, dev_cfg.nb_queue_pairs, ops, (total_jobs_sent - total_dequeued));
    num_dequeued = rte_regex_dequeue_burst(0, 0, ops, (total_jobs_sent - total_dequeued));

    total_dequeued += num_dequeued;
    resp_to_process_cnt = num_dequeued;
    next_response_ops = 0;

    return ret;
}

/*
 * This function is simular to response batch above, but this function
 * simply rotates through each response where above it pull data from host
 * and endian swaps all responses.
 */
struct rxp_response* rxp_next_response_app(struct rxp_response_batch *ctx)
{
    uint64_t i = 0;

    UNUSED(ctx); //Currently not used in this API

    if (resp_to_process_cnt <= 0)
    {
        return NULL;
    }

    if (next_response_ops < resp_to_process_cnt)
    {
        i = next_response_ops;

        /* Fill in RESP Desc, note: Not all data available in ops currently */
        resp_copy->header.job_id = ops[i]->user_id;//(response_index_to_read + 1);
        resp_copy->header.status = 0; //No sure its returned in ops!?
        resp_copy->header.detected_match_count = ops[i]->nb_actual_matches;
        resp_copy->header.match_count = ops[i]->nb_matches;
        resp_copy->header.primary_thread_count = 0; //Not in ops!
        resp_copy->header.instruction_count = 0; //Not in ops!
        resp_copy->header.latency_count = 0; //Not in ops!
        resp_copy->header.pmi_min_byte_ptr = 0; //Not in ops!

        /* Copy over any matches available from ops... */
        if (resp_copy->header.match_count > 0)
        {
           memcpy(resp_copy->matches, ops[i]->matches,
            (sizeof(struct rxp_match_tuple) * resp_copy->header.match_count));
        }

        next_response_ops++;
    }
    else
    {
       return NULL;
    }

    return resp_copy;
}


/*
 * Note: Can ignore rxp parameter, as dont care about RXP port as we need to
 * manage RXP via Load Balancer so both RXP's will be programmed by the first
 * initialisation call, and both will be closed by last thread/app.
 */
int rxp_open_app(unsigned rxp)
{
    UNUSED(rxp); //Currently not used in this API
    return 0;
}

int rxp_close_app(int rxp_handle)
{
    UNUSED(rxp_handle); //Currently not used in this API
    return 0;
}

int rxp_program_rules_app(unsigned rxp, const char *rulesfile, bool incremental)
{
    int ret;

    /* NOTE: This is now done in platform_init!*/
    UNUSED(rxp); //Currently not used in this API
    UNUSED(incremental); //Currently not used in this API
    
    //TODO change the import dev_id later to valid number!
    ret = rte_regex_rule_db_import(0, rulesfile);
 
    if (ret)
    {
        printf("Failed to set rules db for dev %d\n", 0);
	return -1;
    }

    //Regex start devoce mpw ? There is an API for this, but looks like not being used? Remove later
//    ret = rte_regex_dev_start(0);
  //  if (ret < 0)
    //{
//        printf("Failed to start dev %d\n", 0);
  //      return -1;
    //}

    return 1;
}

