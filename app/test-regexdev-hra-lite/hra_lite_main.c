/**
 * @file    hra_lite_main.c
 * @author  Titan IC Systems <support@titanicsystems.com>
 *
 * @section DESCRIPTION
 *
 *   Minimal reference application. The RXP is initialized and the rules memories
 *   are programmed using a pre-compiled ROF file. Random length jobs are created
 *   containing the string "hello world" at a random offset. These jobs are
 *   dispatched to the RXP and responses are received. The number of jobs,
 *   responses and matches are periodically displayed along with the job bit rate.
 *   The application supports multiple queues and multiple RXP Accelerator cards. Each
 *   queue is used to communicate using a single rx/tx queue pair.
 *
 * @section LICENSE
 *
 *   BSD LICENSE
 *
 *   Copyright (C) 2014-2019 Titan IC Systems Ltd. All rights reserved.
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

#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>
#include <sys/types.h>
#include <time.h>

#include "rxp_api_app.h"
#include "hra_platform.h"

#define EXTRA_TESTS 1
#define SNORT_SUBSETS 1
/*
 * Number of threads to use. There must be sufficient RXP hardware queues to
 * support this (max 8 on the RXP Accelerator PCIe card)
 */
#define HRA_THREADS          1

/*
 * Print matches if non-0.
 */
#define HRA_DISPLAY_MATCHES  0

/*
 * ROF file used to program rules memories.
 */
const char *rof_file_name = "rof/synthetic.rof2";

/*
 * Packets are processed whilst this variable is non-0. Set to 0 by Ctrl-C
 * signal handler.
 */
int process_packets = 1;

/*
 * Global data structure which is initialized with random data.
 */
char data[RXP_MAX_JOB_LENGTH];
char testdata1[] = {"This is a test string, hello world! ABCDEFGHIJKLM GIF89a YCLC_E NOPQRSTUVWXYZABCDThis is a test string, hello world! ABCDEFGHIJKLMNOPQRSTUVWXYZABCD skillz abcdefghijklmnop ficken connected client"};
//char testdata1[] = {[0 ... 126] = 1}; //?? Query all zeros
//char testdata1[] = {[0 ... 510] = 1};
//char testdata1[] = { [0 ... 1023] = 1};
//char testdata1[] = { [0 ... 2047] = 1};
//char testdata1[] = { [0 ... 4095] = 1};
//char testdata1[] = { [0 ... 8191] = 1};
//char testdata1[] = { [0 ... 16382] = 1};

uint64_t hra_job_count = 0;

/******************************************************************************/
/*              LOCAL FUNCTIONS                                               */
/******************************************************************************/

/*
 * Display information on each match.
 */
#if HRA_DISPLAY_MATCHES
static void
hra_display_matches(int id, struct rxp_response *resp_data)
{
    unsigned int i;

    for (i = 0; i < resp_data->header.match_count; i++)
    {
        printf("[%d] Job ID [0x%08x] Match [%d] rule_id [%u] start_ptr [%u] length [%u]\n",
            id, resp_data->header.job_id, i, resp_data->matches[i].rule_id,
            resp_data->matches[i].start_ptr,
            resp_data->matches[i].length);
    }
}
#endif //if HRA_DISPLAY_MATCHES

/*
 * Create a random length job, with a length between the length of
 * "hello world" and RXP_MAX_JOB_LENGTH bytes. Copy the data from the randomly
 * initialized buffer to a local buffer and write "hello world" to
 * a random offset. Every job will have a match to the
 * "hello world" rule.
 */
static int
hra_new_job(char *buffer)
{
    int length;
    int offset;
    char hello[12] = "hello world";
    int hello_len = 11;

    length = (rand() % (RXP_MAX_JOB_LENGTH - hello_len + 1)) + hello_len;
    offset = rand() % (length - hello_len + 1);
    memcpy(buffer, data, length);
    memcpy(&buffer[offset], hello, hello_len);

    return (length);
}

static long
timeval_diff_ns(struct timespec *start, struct timespec *end)
{
    long diff;

    diff = (end->tv_sec - start->tv_sec) * 1000000000L;
    diff += (end->tv_nsec - start->tv_nsec);

    return diff;
}

/*
 * Function loops, creating jobs and dispatching them to the RXP. Each job has
 * a random length and has "hello world" embedded at a random location.
 * Periodically performance statistics are printed to screen.
 */
static void *
hra_main_loop(void *thread_data)
{
    intptr_t ret = -1;
    uint32_t job_id = 1;
    struct timespec start, end, prev, cur;
    uint64_t jobs = 0;
    uint64_t responses = 0;
    uint64_t prev_job_bytes = 0;
    uint64_t matches = 0;
    double rate;
    char local_data[RXP_MAX_JOB_LENGTH];
    int length;
    uint64_t job_bytes = 0;
    int rxp_handle;
    bool rx_ready, tx_ready;
    int id = *(int *) thread_data;
    struct rxp_response_batch resp_ctx = {0};
    struct rxp_response *resp;

    resp_ctx.buf_size = RXP_RESP_BUF_SIZE_MIN;
    resp_ctx.buf = malloc(resp_ctx.buf_size);

    rxp_handle = rxp_open_app(0);

    printf("thread: thread_id = %d\n", id);

    clock_gettime(CLOCK_MONOTONIC, &start);
    clock_gettime(CLOCK_MONOTONIC, &prev);

    /*
     * Loop until the maximum number of responses have been received and
     * process_packets is non-zero. process_packets is set to 0 in the Ctrl-C
     * handler.
     */
    while ((responses < hra_job_count) && process_packets)
    {
        clock_gettime(CLOCK_MONOTONIC, &cur);

        /*
         * If there is space on the dispatch list is not full, then generate a
         * new job and add it to the dispatch list. Else dispatch the jobs.
         * Stop adding jobs if the maximum number of jobs have been created.
         */
        if (rxp_queue_status_app(rxp_handle, &rx_ready, &tx_ready) < 0)
        {
            printf("Error: rxp_queue_status() failed\n");
            goto early_exit;
        }

        if (tx_ready && (jobs < hra_job_count))
        {
            /*
             * Get a new random sized job with "hello world" embedded at a
             * random offset.
             */
            length = hra_new_job(local_data);
            jobs++;
#ifndef EXTRA_TESTS
            job_bytes += length;

            if (rxp_submit_job_app(rxp_handle, job_id++, (uint8_t *) local_data, length,
                                1, 1, 1, 1, false, false) < 0)
#else
            length = strlen(testdata1);
            job_bytes += strlen(testdata1);
#ifdef SNORT_SUBSETS
            if (rxp_submit_job_app(rxp_handle, job_id++, (uint8_t *) testdata1, length,
                                1, 2, 6, 7, false, false) < 0)


#else
            if (rxp_submit_job_app(rxp_handle, job_id++, (uint8_t *) testdata1, length,
                                1, 1, 1, 1, false, false) < 0)
#endif //SNORT_SUBSETS
#endif //EXTRA_TESTS
            {
                printf("Error: rxp_submit_job() failed\n");
                goto early_exit;
            }

            /*
             * 0 is an invalid job_id.
             */
            if (job_id == 0)
            {
                job_id++;
            }
        }

        /*
         * Receive responses from the queue. A maximum of RXP_MAX_PKT_BURST
         * responses will be received. Zero matches may be received.
         */
        if (rx_ready)
        {
            if (rxp_read_response_batch_app(rxp_handle, &resp_ctx) < 0)
            {
                printf("Error: rxp_read_response_batch() failed\n");
                goto early_exit;
            }
            while ((resp = rxp_next_response_app(&resp_ctx)) != NULL)
            {
                responses += 1;

                /*
                 * For each response, get the response data and increment the matches
                 * count by the number of matches in the response. Free the received
                 * buffer when complete. Optionally print out the match info.
                 */
               matches += resp->header.match_count;
#if HRA_DISPLAY_MATCHES
               hra_display_matches(id, resp);
#endif
           }
        }

        /*
         * Periodically print statistics (once a second).
         */
        if (timeval_diff_ns(&prev, &cur) > (1 * 1000000000L))
        {
            /*
             * Calculate the rate which jobs were dispatched in the previous period.
             * The rate is in Giga-bits per second.
             */
            rate = (double)((job_bytes - prev_job_bytes) * 8) /
                ((cur.tv_sec - prev.tv_sec) * 1000000000L + (cur.tv_nsec - prev.tv_nsec));
            printf("[%u]: jobs = %" PRIu64 " responses = %" PRIu64 " matches = %"
                PRIu64 " rate = %.3f Gbps\n", id, jobs, responses, matches, rate);
            prev = cur;
            prev_job_bytes = job_bytes;
        }
    }

    /*
     * Before loop exits print the total number of jobs, responses, matches and
     * the average rate.
     */
    clock_gettime(CLOCK_MONOTONIC, &end);
    rate = (double)((job_bytes) * 8) /
        ((end.tv_sec - start.tv_sec) * 1000000000L + (end.tv_nsec - start.tv_nsec));
    printf("Total [%u]: jobs = %" PRIu64 " responses = %" PRIu64 " matches = %"
         PRIu64 " rate = %.3f Gbps\n", id, jobs, responses, matches, rate);

    ret = 0;

early_exit:
    rxp_close_app(rxp_handle);
    free(resp_ctx.buf);

    return (void *)ret;
}

/*
 * CTRL-C handler function
 *
 * @param signal_number   unused
 */
static void
hra_signal_handler(int signal_number)
{
    (void) signal_number;
    process_packets = 0;
}

/*
 * Function registers handler for CTRL-C
 */
static int
hra_init_signal_handler(void)
{
    struct sigaction sa;
    int rc;

    memset (&sa, 0, sizeof (sa));
    sa.sa_handler = &hra_signal_handler;
    rc = sigaction (SIGINT, &sa, NULL);

    return (rc);
}

/******************************************************************************/
/*                          MAIN FUNCTION                                     */
/******************************************************************************/

int
main(int argc, char **argv)
{
    int ret = 0;
    int i;
    pthread_t threads[HRA_THREADS];
    int thread_ids[HRA_THREADS];

    /* Platform specific initialisation. */
    if ((ret = rxp_platform_init_app(0, argc, argv)) < 0)
    {
        perror("Error: rxp_platform_init() failed");
        exit(ret);
    }

    printf("Info: Beginning to process application command line arguments\n");

    if (hra_job_count <= 0)
    {
        hra_job_count = HRA_NUM_JOBS;
    }

    /*
     * Start by programming our rules into the RXP.
     */
    if ((ret = rxp_program_rules_app(0, rof_file_name, false)) < 0)
    {
        perror("Error: rxp_program_rules() failed");
        goto cleanup;
    }
    else
    {
        ret = 0;
    }

    /*
     * Initialize a data buffer with random values.
     */
    srand(time(NULL));
    for (i = 0; i < RXP_MAX_JOB_LENGTH; i++)
    {
        data[i] = rand();
    }

    /*
     * Initialize Ctrl-C signal handler
     */
    hra_init_signal_handler();

    /* Get queue/core mapping, i.e. what core will each queue thread execute on. */
    unsigned queue_core[HRA_THREADS];
    hra_get_queue_core_map(0, queue_core, HRA_THREADS);

    /*
     * Launch the hra_main_loop() function for each secondary thread.
     */
    for (i = 1; i < HRA_THREADS; i++)
    {
        thread_ids[i] = i;
        hra_thread_create(&threads[i], hra_main_loop, &thread_ids[i], queue_core[i]);
    }

    /* Main thread. */
    hra_main_loop(&thread_ids[i]);

    /*
     * Wait for each secondary thread to finish and return.
     */
    for (i = 1; i < HRA_THREADS; i++)
    {
        hra_thread_join(&threads[i], i, NULL);
    }

cleanup:
    /* Platform specific un-initialisation. */
    ret = rxp_platform_uninit_app(0);

    if (ret < 0)
    {
        perror("Error: rxp_platform_uninit() failed");
    }

    return (ret);
}

