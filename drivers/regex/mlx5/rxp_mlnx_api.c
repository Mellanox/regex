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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "rxp-api.h"
#include "rxp-csrs.h"
#include "host.h"

static bool rxp_init_status = false;
static bool rxp_prog_status = false;
struct rxp_queue queue[RXP_NUM_QUEUES];

__attribute__ ((visibility ("default")))
int rxp_queue_status(int rxp_handle, bool *rx_ready, bool *tx_ready)
{
    int ret = 1;

    if ((rxp_handle >= 0) && ((unsigned int)rxp_handle <= RXP_NUM_QUEUES))
    {
        ret = mlnx_poll(&queue[rxp_handle], rx_ready, tx_ready);
    }
    else
    {
        return -EBUSY; //TODO should this be an error?
    }

    return ret;
}

__attribute__ ((visibility ("default")))
void rxp_job_batch_free(struct rxp_job_batch *ctx)
{
    if (ctx)
    {
        free(ctx);
    }
}

__attribute__ ((visibility ("default")))
struct rxp_job_batch *rxp_job_batch_alloc(size_t max_jobs,
                                          size_t bytes_threshold)
{
    struct rxp_job_batch *ctx = NULL;

    if (max_jobs > RXP_TX_BURST_MAX || max_jobs < RXP_TX_BURST_MIN)
        return NULL;

    ctx = calloc(1, sizeof(*ctx) + (sizeof(ctx->job[0]) * max_jobs));

    if (ctx)
    {
        ctx->max_jobs = max_jobs;
        ctx->bytes_threshold = bytes_threshold;
    }

    return ctx;
}

__attribute__ ((visibility ("default")))
int rxp_dispatch_jobs(int rxp_handle, struct rxp_job_batch *ctx)
{
    unsigned int i, last;
    int written, towrite;
    struct rxp_mlnx_job_desc data[RXP_TX_BURST_MAX * 2];

    if ((rxp_handle < 0) || ((unsigned int)rxp_handle > RXP_NUM_QUEUES))
    {
        return -EPERM;
    }
    if (ctx->count == 0)
    {
	printf("CNT = 0\n");
        return 0;
    }

    towrite = 0;

    for (i = 0; i < ctx->count; i++)
    {
        data[i * 2].data_ptr = &ctx->job[i].desc;
        data[i * 2].len = sizeof(ctx->job[i].desc);
        towrite += data[i * 2].len;
        data[i * 2 + 1].data_ptr = (void *) ctx->job[i].data;
        data[i * 2 + 1].len = ctx->job[i].len;
        towrite += data[i * 2 + 1].len;
    }

    written = mlnx_submit_job(&queue[rxp_handle], data, i * 2);

    if (written < 0)
    {
        return -1; //TODO add appropriate error here!
    }

    /* If we didn't manage to write all outstanding jobs, update the context */
    if (written != towrite)
    {
        last = i;
        towrite = 0;
        for (i = 0; i < last; i++)
        {
            if (towrite == written)
            {
                break;
            }
            towrite += data[i * 2].len;
            /* Should not end up writing a header without the associated data */
            assert(towrite != written);
            towrite += data[i * 2 + 1].len;
        }

        last = i;
        ctx->count -= last;
        /* Shuffle the unsent jobs to the start of the batch context buffer */
        ctx->bytes_total = 0;
        for (i = 0; i < ctx->count; i++)
        {
            ctx->job[i].desc = ctx->job[i+last].desc;
            ctx->job[i].data = ctx->job[i+last].data;
            ctx->job[i].len = ctx->job[i+last].len;
            ctx->bytes_total += ctx->job[i + last].len;
        }
        i = last;
    }
    else
    {
        /* We wrote all outstanding jobs, clear outstanding count */
        ctx->count = 0;
        ctx->bytes_total = 0;
    }

    return i;
}

__attribute__ ((visibility ("default")))
int rxp_scan_job(int rxp_handle, struct rxp_job_batch *ctx, uint32_t jobid,
    const uint8_t *buf, uint16_t len, uint16_t subset1, uint16_t subset2,
    uint16_t subset3, uint16_t subset4, bool enable_hpm, bool enable_anymatch)
{
    int i,  ret = 0;
    if ((rxp_handle < 0) || ((unsigned int)rxp_handle > RXP_NUM_QUEUES))
    {
        printf("-EPERM\n");
        return -EPERM;
    }
    if (len > RXP_MAX_JOB_LENGTH)
    {
        return -EINVAL;
    }
    if (subset1 == 0)
    {
        return -EINVAL;
    }

    //TODO: Note sure if we have these modes currently as control bits
    //      may not be all used!!
    /* Can't set both modes. */
    if (enable_hpm && enable_anymatch)
    {
        return -EINVAL;
    }

    /* Attempt to dispatch some jobs if:
     *  - There are not slots available in the context structure
     *  - or the total number of job bytes pending has reached a threshold.
     */
    if (ctx->count >= ctx->max_jobs ||
            (ctx->bytes_threshold && ctx->bytes_total >= ctx->bytes_threshold))
    {
	printf("ctx->count %ld\n", ctx->count);
        ret = rxp_dispatch_jobs(rxp_handle, ctx);
        if (ret < 0)
            return ret;
        else if (ret == 0) {

        printf("-EBUSY\n");
            return -EBUSY;
	}
        /* +ve means we submitted ok, so have room to add */
    }
    i = ctx->count++;
    ctx->job[i].desc.job_id = htole32(jobid);
    ctx->job[i].desc.len = htole16(len);
    ctx->job[i].desc.ctrl = htole16(RXP_CTRL_TYPE_JOB_DESCRIPTOR |
                    RXP_CTRL_VALID |
                    (enable_hpm ? RXP_CTRL_JOB_DESC_HPM_ENABLE : 0) |
                    (enable_anymatch ? RXP_CTRL_JOB_DESC_ANYMATCH_ENABLE : 0));
    ctx->job[i].desc.subset[0] = htole16(subset1);
    ctx->job[i].desc.subset[1] = subset2 ? htole16(subset2) : htole16(subset1);
    ctx->job[i].desc.subset[2] = subset3 ? htole16(subset3) : htole16(subset1);
    ctx->job[i].desc.subset[3] = subset4 ? htole16(subset4) : htole16(subset1);

    /* Check subset range not > 4095 limit? */
    ctx->job[i].data = (void *)(size_t)buf;
    ctx->job[i].len = len;
    ctx->bytes_total += len;

    return ret;
}

__attribute__ ((visibility ("default")))
int rxp_submit_job(int rxp_handle, uint32_t jobid, const uint8_t *buf,
                   uint16_t len, uint16_t subset1, uint16_t subset2,
                   uint16_t subset3, uint16_t subset4, bool enable_hpm,
                   bool enable_anymatch)
{
    int written;
    struct rxp_job_desc job;
    struct rxp_mlnx_job_desc data[2];

    if ((rxp_handle < 0) || ((unsigned int)rxp_handle > RXP_NUM_QUEUES))
    {
        return -EPERM;
    }
    if (len > RXP_MAX_JOB_LENGTH)
    {
        return -EINVAL;
    }
    if (subset1 == 0)
    {
        return -EINVAL;
    }

    //TODO: Note sure if we have these modes currently as control bits
    //      may not be all used!!
    /* Can't set both modes. */
    if (enable_hpm && enable_anymatch)
    {
        return -EINVAL;
    }

    job.job_id = htole32(jobid);
    job.len = htole16(len);
    job.ctrl = htole16(RXP_CTRL_TYPE_JOB_DESCRIPTOR |
                    RXP_CTRL_VALID |
                    (enable_hpm ? RXP_CTRL_JOB_DESC_HPM_ENABLE : 0) |
                    (enable_anymatch ? RXP_CTRL_JOB_DESC_ANYMATCH_ENABLE : 0));
    job.subset[0] = htole16(subset1);
    job.subset[1] = subset2 ? htole16(subset2) : htole16(subset1);
    job.subset[2] = subset3 ? htole16(subset3) : htole16(subset1);
    job.subset[3] = subset4 ? htole16(subset4) : htole16(subset1);

    /* Check subset range not > 4095 limit? */

    data[0].data_ptr = &job;
    data[0].len = sizeof(job);
    data[1].data_ptr = (void *)(size_t)buf;
    data[1].len = len;

    written = mlnx_submit_job(&queue[rxp_handle], data, 1);

    if (written < 0)
    {
        return written;
    }
    else if ((unsigned int)written < (sizeof(struct rxp_job_desc) + len))
    {
        return -EBUSY;
    }

    return 0;
}

__attribute__ ((visibility ("default")))
int rxp_read_response_batch(int rxp_handle, struct rxp_response_batch *ctx)
{
    unsigned int num_resps = 0;
    uint32_t num_returned_resp = 0;
    if ((rxp_handle < 0) || ((unsigned)rxp_handle > RXP_NUM_QUEUES))
    {
        return -EPERM;
    }

    /* Clear the context. */
    ctx->buf_used = 0;
    ctx->next_offset = 0;

    /* Read responses from Mlnx AP */
    //printf("ctx->buf_size = %ld\n", ctx->buf_size);
    int read_ret = mlnx_read_resp(&queue[rxp_handle], ctx->buf, ctx->buf_size,
                                  &num_returned_resp);

    if (read_ret < 0)
    {
        /* pass error code from API back to caller/App */
        return read_ret;
    }
    else if (read_ret == 0)
    {
        /* no bytes read, pass zero (no responses) back to caller. */
        return 0;
    }
    else
    {
        unsigned next_offset;
        ctx->buf_used = read_ret;

        /*
         * Notably for first release block/fixed read sizes will be used,
         * so there may be some padding in the buffer, so need to filter this
         * out as appropriate!
         */
        next_offset = 0;

        /* Iterate through responses in the buffer and fix endianess. */
        while (next_offset < ctx->buf_used)
        {
            struct rxp_response *resp =
                                (struct rxp_response*)&ctx->buf[next_offset];
            int match_count = DEVX_GET(regexp_metadata,
                                &ctx->buf[next_offset], match_count);
            unsigned resp_size = sizeof(resp->header) +
                                (sizeof(resp->matches[0]) * match_count);
#if 0
            struct rxp_response *resp =
                                (struct rxp_response*)&ctx->buf[next_offset];
            unsigned resp_size = sizeof(resp->header) +
                                (sizeof(resp->matches[0]) *
                                        resp->header.match_count);
            resp->header.job_id = le32toh(resp->header.job_id);
            resp->header.status = le16toh(resp->header.status);
            resp->header.primary_thread_count =
                                    le16toh(resp->header.primary_thread_count);
            resp->header.instruction_count =
                                    le16toh(resp->header.instruction_count);
            resp->header.latency_count = le16toh(resp->header.latency_count);
            resp->header.pmi_min_byte_ptr = le16toh(resp->header.pmi_min_byte_ptr);

            if (resp->header.job_id == 0)
            {
                mlnx_log("rxp_read_response_batch: jobid equal 0!");
                return -1;
            }

            if (resp->header.match_count > 0)
            {
                int i;
                //TODO check endianess with Mlnx Project!
                /* Fix up the endianness on the responses */
                for (i = 0; i < resp->header.match_count; i++)
                {
                    //TODO:JOBID Not in Mlnx-Remove comment after initial tests!
                    //resp->matches[i].job_id = le32toh(resp->matches[i].job_id);
                    resp->matches[i].rule_id =
                                            le32toh(resp->matches[i].rule_id);
                    /* TODO: Note: Changed le32 to le16 for Mlnx */
                    resp->matches[i].start_ptr =
                                            le16toh(resp->matches[i].start_ptr);
                    resp->matches[i].length = le16toh(resp->matches[i].length);
                }
            }
#endif
            next_offset += resp_size;
            num_resps++;

            /* Ensure dont try and read more than actually returned. */
            if (num_resps >= num_returned_resp)
            {
                /* No need to read any further data as all responses handled,
                 * the remainder must be padding/redundant. */
                ctx->buf_used = next_offset;
                break;
            }
        }
    }

    return num_resps;
}

/*
 * This function is simular to response batch above, but this function
 * simply rotates through each response where above it pull data from host
 * and endian swaps all responses.
 */
__attribute__ ((visibility ("default")))
struct rxp_response* rxp_next_response(struct rxp_response_batch *ctx)
{
    struct rxp_response *resp;
    if (ctx->next_offset < ctx->buf_used)
    {
        int resp_size;
        resp = (struct rxp_response*)&ctx->buf[ctx->next_offset];
        int match_count = DEVX_GET(regexp_metadata,
                        	   &ctx->buf[ctx->next_offset], match_count);
        resp_size = sizeof(resp->header) +
                   (sizeof(resp->matches[0]) * match_count);
        ctx->next_offset += resp_size;
    }
    else
    {
        resp = NULL;
    }

    return resp;
}


/*
 * Note: Can ignore rxp parameter, as dont care about RXP port as we need to
 * manage RXP via Load Balancer so both RXP's will be programmed by the first
 * initialisation call, and both will be closed by last thread/app.
 */
__attribute__ ((visibility ("default")))
int rxp_open(unsigned rxp __rte_unused, struct ibv_context *ctx)
{
    int ret;
    int rxp_handle = -1;

    /* Have we initilised the rxp/s yet? */
    if (rxp_init_status == false)
    {
        /* Only do mlnx_init once! */
        rxp_init_status = true;
        ret = mlnx_init(ctx);
        if (ret < 0)
        {
            mlnx_log("rxp_open: Failed to setup RXP/s - mlnx_init!");
            return ret;
        }
    }

    rxp_handle = mlnx_open(queue);

    if (rxp_handle < 0)
    {
        mlnx_log("rxp_open: Failed to rxp open!");
        return -EPERM;
    }

    return rxp_handle;
}

__attribute__ ((visibility ("default")))
int rxp_close(int rxp_handle)
{
    int ret;
    if ((rxp_handle < 0) || ((unsigned int)rxp_handle > RXP_NUM_QUEUES))
    {
        return -EPERM;
    }
    ret = mlnx_release(&queue[rxp_handle]);

    /* TODO Basic tidyup here, when add multiple queues, need to properly remove
     * rxp_handle from queue via lookup func! */
    if (queue[rxp_handle].rxp->open_queues == 0)
    {
        /*
         * Clear init flag as all opens have been closed so this will allow
         * a further rxp_open/mlnx_init */
        rxp_init_status = false;

        /*
         * TODO: Should we disable both RXP engines at this point as this is
         *       the last application to use RXPs? */
        rxp_prog_status = false; //TODO put in this simple flag for first release of libRXP!
    }

    return ret;
}

/*
 * Application to call this function to set the Programming mode of the
 * system.  Note we currently can have "Private" or "Shared" programming modes.
 */
void rxp_programming_mode_set(enum rxp_program_mode mode)
{
    mlnx_prog_mode_set(mode);
}


static int parse_rof(const char *filename, struct rxp_ctl_rules_pgm **rules)
{
    FILE *roffile;
    char *curline, *curpos;
    unsigned int lines, entries;
    size_t len, maxlen;
    struct rxp_rof_entry *curentry;
    int ret = 0;

    if (!rules)
    {
        return -EINVAL;
    }

    roffile = fopen(filename, "r");

    if (!roffile)
    {
        return -errno;
    }
    maxlen = 80;            /* Expected line length */
    curline = malloc(maxlen);

    if (!curline)
    {
        fclose(roffile);
        return -ENOMEM;
    }
    lines = 0;
    len = maxlen;

    while (getline(&curline, &len, roffile) != -1)
    {
        if (len > maxlen)
        {
            maxlen = len;
        }
        if (curline[0] == '#' || curline[0] == '\0')
        {
            continue;
        }
        lines++;
        len = maxlen;
    }

    rewind(roffile);

    *rules = malloc(lines * sizeof(*curentry) + sizeof(**rules));
    memset(*rules, 0, lines * sizeof(*curentry) + sizeof(**rules));
    curentry = (*rules)->rules;

    (*rules)->hdr.cmd = RXP_CTL_RULES_PGM;
    entries = 0;
    len = maxlen;

    while (getline(&curline, &len, roffile) != -1)
    {
        if (curline[0] == '#' || curline[0] == '\0')
        {
            continue;
        }
        curentry->type = strtoul(curline, &curpos, 10);
        if (curpos == curline || curpos[0] != ',')
        {
            ret = -EINVAL;
            break;
        }
        curpos++;
        curentry->addr = strtoul(curpos, &curpos, 16);
        if (curpos[0] != ',')
        {
            ret = -EINVAL;
            break;
        }
        curpos++;
        curentry->value = strtoull(curpos, &curpos, 16);
        if (curpos[0] != '\0' && curpos[0] != '\n')
        {
            ret = -EINVAL;
            break;
        }
        curentry++;
        entries++;
        len = maxlen;
        if (entries > lines)
        {
            ret = -EINVAL;
            break;
        }
    }
    (*rules)->count = entries;
    (*rules)->hdr.len = sizeof(**rules) + entries * sizeof(*curentry);
    if (ret < 0)
    {
        free(*rules);
        *rules = NULL;
    }
    else
    {
        ret = entries;
    }
    free(curline);
    fclose(roffile);

    return ret;
}



//TODO: Need to consider making this thread safe, as multiple application
//      for Mlnx will be attempting to call this!

/*
 * Note: For Mellanox we need to consider programming both RXP engines on
 * start up. This version of code will be a simple design not concerned with
 * multiple applications attempting to program RXP's!
 */
__attribute__ ((visibility ("default")))
int rxp_program_rules(unsigned rxp __rte_unused, const char *rulesfile,
		      bool incremental, struct ibv_context *ctx)
{
    unsigned i;
	int ret,  db_free;
    uint32_t keep_rule_count;
    struct rxp_ctl_rules_pgm *rules = NULL;

    /* Have we initilised the rxp yet? */
    if (rxp_init_status == false)
    {
        /* Only do mlnx_init once! */
        rxp_init_status = true;
        ret = mlnx_init(ctx);

        if (ret < 0)
        {
            mlnx_log("Failed to setup RXP - mlnx_init!\n");
            return ret;
        }
    }

    //TODO: Temp block of multiple apps attempting to program RXPs if already programmed!
    //As more complex checks and returns required e.g. return possibly ROF versions to
    //allow app to proceed if identicle; merge databases/rulesets
    //TODO could add another parameter to this function to force new programming sequence!?
    //Note that if incremental programming allow this for now...
    if ((rxp_prog_status == true) && (!incremental))
    {
        /*
         * Return Error as currently programmed, can only do incremental update
         * for this first draft release...
         */
        mlnx_log("Error: RXP currently programmed - (rxp_program_rules)!\n");
        return -1;
    }

    ret = parse_rof(rulesfile, &rules);
    if (ret < 0)
    {
        return ret;
    }

    if (incremental)
    {
        rules->hdr.cmd = RXP_CTL_RULES_PGM_INCR;
    }

//TODO - Need to check with Mlnx if need to mlnx_set_database first and whats the conseqeunces of doing that call early??
//TODO - Check that incremental rules doesn't occur before full programming and init of RXP!?

    /*
     * Keep account of the total rules count as each programming of rules for
     * each RXP engine will decrement this count (Private only), so reset it
     * following use!
     */
    keep_rule_count = rules->count;

    /* Program both RXP's with the following rules...*/
    for (i = 0; i < MAX_RXP_ENGINES; i++)
    {
        if (RXP_PRIVATE_PROG_MODE == mlnx_prog_mode_get())
        {
            /*
            * Need to set the mlnx_set_database immediately as when we start
            * pushing instructions to RXP, we need to be sure the RXP has the
            * capability to write to Shared/External memory!
            */
            db_free = mlnx_update_database(rules->hdr.cmd, i);

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
            ret = mlnx_set_database(i, db_free);//Ensure RXP Eng idle before run

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
}

__attribute__ ((visibility ("default")))
int rxp_read_stats(unsigned rxp __rte_unused, struct rxp_stats *stats)
{
    int ret, i = 0;
    uint32_t tmp;
    uint64_t byte_cnt;

    /* Return RXP details not multiqueue details. */
    //TODO: Make HRA Apps manage multiple RXP engine stats...
    //      For now hardcode to RXP engine 0
    //for (i = 0; i < MAX_RXP_ENGINES; i++)
    //{
        ret = mlnx_csr_read(RXP_CSR_JOB_COUNT, &stats->num_jobs, i);
        if (ret < 0)
        {
            return ret;
        }

        ret = mlnx_csr_read(RXP_CSR_RESPONSE_COUNT, &stats->num_responses, i);
        if (ret < 0)
        {
            return ret;
        }

        ret = mlnx_csr_read(RXP_CSR_MATCH_COUNT, &stats->num_matches, i);
        if (ret < 0)
        {
            return ret;
        }

        ret = mlnx_csr_read(RXP_CSR_JOB_ERROR_COUNT, &stats->num_job_errors, i);
        if (ret < 0)
        {
            return ret;
        }

        ret = mlnx_csr_read(RXP_CSR_JOB_BYTE_COUNT0, &tmp, i);
        if (ret < 0)
        {
            return ret;
        }

        stats->num_bytes = tmp;
        ret = mlnx_csr_read(RXP_CSR_JOB_BYTE_COUNT1, &tmp, i);
        if (ret < 0)
        {
            return ret;
        }
    //}

    byte_cnt = tmp;
    stats->num_bytes |= (uint64_t)(byte_cnt << 32);

    return ret;
}

static unsigned rxp_count_zeroes(uint8_t val)
{
    unsigned n = 0;
    val = ~val;
    while (val)
    {
        val &= (val - 1);
        n++;
    }

    return n;
}

__attribute__ ((visibility ("default")))
int rxp_read_perf_stats(unsigned rxp __rte_unused, struct rxp_perf_stats *stats)
{
    uint32_t csr_stat = 0;
    int i, ret, j = 0;

    //TODO: Make HRA Apps manage multiple RXP engine stats...
    //      For now hardcode to RXP engine 0
    /* Cluster statistics */
    for (i = 0; i < 16; i++) 
    {
        /* Read Control Plane RXP_STAT_CSR registers */
        ret = mlnx_csr_read((RXP_STATS_CSR_CLUSTER + i), &csr_stat, j);
        if (ret < 0)
        {
            return ret;
        }

        stats->cluster[i].jce_idle_id = rxp_count_zeroes(csr_stat & 0xff);
        stats->cluster[i].tce_idle_id =
                                    rxp_count_zeroes((csr_stat >> 8) & 0xff);
        stats->cluster[i].hit_duty_cycle =
                                        (((csr_stat >> 16) & 0xff) * 100) / 256;
        stats->cluster[i].instruction_duty_cycle =
                                        (((csr_stat >> 24) & 0xff) * 100) / 256;
    }

    /* L2 cache statistics */
    ret = mlnx_csr_read(RXP_STATS_CSR_L2_CACHE, &csr_stat, j);
    if (ret < 0)
    {
        return ret;
    }

    stats->l2_cache.cache_hit_duty_cycle = ((csr_stat & 0xff) * 100) / 256;
    stats->l2_cache.cache_miss_duty_cycle =
                                        (((csr_stat >> 8) & 0xff) * 100) / 256;
    stats->l2_cache.request_fifo_num_entries = (csr_stat >> 16) & 0xff;
    stats->l2_cache.read_pending_completion_fifo_num_entries =
                                                        (csr_stat >> 24) & 0xff;

    /* MPFE fifo depths */
    ret = mlnx_csr_read(RXP_STATS_CSR_MPFE_FIFO, &csr_stat, j);
    if (ret < 0)
    {
        return ret;
    }

    stats->mpfe_fifo_entries[0] = csr_stat & 0xff;
    stats->mpfe_fifo_entries[1] = (csr_stat >> 8) & 0xff;
    stats->mpfe_fifo_entries[2] = (csr_stat >> 16) & 0xff;
    stats->mpfe_fifo_entries[3] = (csr_stat >> 24) & 0xff;
    /* Prefix Engine statistics */
    ret = mlnx_csr_read(RXP_STATS_CSR_PE, &csr_stat, j);
    if (ret < 0)
    {
        return ret;
    }

    stats->pe.nd_duty_cycle = ((csr_stat & 0xff) * 100) / 256;
    stats->pe.primary_thread_valid_duty_cycle =
                                        (((csr_stat >> 8) & 0xff) * 100) / 256;

    return 0;
}

//TODO: Make HRA Apps manage multiple RXP engine stats...
//      For now hardcode to RXP engine 0 for all functions below...
__attribute__ ((visibility ("default")))
int rxp_read_max_matches(unsigned rxp __rte_unused, uint32_t *max_matches)
{
    return mlnx_csr_read(RXP_CSR_MAX_MATCH, max_matches, 0);
}

__attribute__ ((visibility ("default")))
int rxp_set_max_matches(unsigned rxp __rte_unused, uint32_t max_matches)
{
    return mlnx_csr_write(&max_matches, RXP_CSR_MAX_MATCH, 0);
}

__attribute__ ((visibility ("default")))
int rxp_read_max_prefixes(unsigned rxp __rte_unused, uint32_t *max_prefixes)
{
    return mlnx_csr_read(RXP_CSR_MAX_PREFIX, max_prefixes, 0);
}

__attribute__ ((visibility ("default")))
int rxp_set_max_prefixes(unsigned rxp __rte_unused, uint32_t max_prefixes)
{
    return mlnx_csr_write(&max_prefixes, RXP_CSR_MAX_PREFIX, 0);
}

__attribute__ ((visibility ("default")))
int rxp_read_max_latency(unsigned rxp __rte_unused, uint32_t *max_latency)
{
    return mlnx_csr_read(RXP_CSR_MAX_LATENCY, max_latency, 0);
}

__attribute__ ((visibility ("default")))
int rxp_set_max_latency(unsigned rxp __rte_unused, uint32_t max_latency)
{
    return mlnx_csr_write(&max_latency, RXP_CSR_MAX_LATENCY, 0);
}

__attribute__ ((visibility ("default")))
int rxp_read_max_pri_threads(unsigned rxp __rte_unused, uint32_t *max_pri_threads)
{
    return mlnx_csr_read(RXP_CSR_MAX_PRI_THREAD, max_pri_threads, 0);
}

__attribute__ ((visibility ("default")))
int rxp_set_max_pri_threads(unsigned rxp __rte_unused, uint32_t max_pri_threads)
{
    return mlnx_csr_write(&max_pri_threads, RXP_CSR_MAX_PRI_THREAD, 0);
}
