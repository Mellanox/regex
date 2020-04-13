/**
 * @file    hra_platform.c
 * @author  Titan IC Systems <support@titanicsystems.com>
 *
 * @section DESCRIPTION
 *
 *   This module contains some platform specific routines used by the
 *   Host Reference Apps.
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
#ifndef _GNU_SOURCE
#define _GNU_SOURCE  /* for CPU_SET/CPU_ZERO macros */
#endif
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "hra_platform.h"

#ifdef DPDK
#include <rte_cycles.h>
#include <rte_launch.h>
#include <rte_lcore.h>
#else
#include <pthread.h>
#include <sys/sysinfo.h>
#define USE_TSC
#endif

/* How many tsc ticks per second. */
static uint64_t g_ticks_per_sec = 0;

uint64_t
hra_ticks_per_sec(void)
{
    /* Ensure hra_ticks_init() has been called otherwise. */
    if (!g_ticks_per_sec)
    {
        printf("*********************************************\n" \
               "*  g_ticks_per_sec has not been set.        *\n" \
               "*  Ensure hra_ticks_init() has been called. *\n" \
               "*********************************************\n");
        exit(-1);
    }
    return g_ticks_per_sec;
}

#ifdef USE_TSC

static uint64_t
hra_tsc_read(void)
{
#if defined(__aarch64__)
    uint64_t ticks;
    asm volatile("mrs %0, CNTVCT_EL0" : "=r" (ticks));
    return ticks;
#else
    union {
        uint64_t tsc_64;
        struct {
            uint32_t lo_32;
            uint32_t hi_32;
        };
    } tsc;

    asm volatile("rdtsc" :
             "=a" (tsc.lo_32),
             "=d" (tsc.hi_32));

    return tsc.tsc_64;
#endif
}

#endif

uint64_t
hra_ticks_read(void)
{
#ifdef DPDK
    return rte_rdtsc();
#elif defined(USE_TSC)
    return hra_tsc_read();
#else
    /* Time in nano-seconds. */
    struct timespec cur_time;
    clock_gettime(CLOCK_MONOTONIC, &cur_time);
    return ((cur_time.tv_sec * NS_PER_SEC) + cur_time.tv_nsec);
#endif
}


void
hra_ticks_init(void)
{
#ifdef DPDK
    g_ticks_per_sec = rte_get_tsc_hz();
#else
#ifdef USE_TSC
#ifdef CLOCK_MONOTONIC_RAW

    /* Work out how many tsc 'ticks' in a second. */
    struct timespec sleeptime = {.tv_nsec = NS_PER_SEC / 10 }; /* 1/10 second */
    struct timespec t_start, t_end;

    if (clock_gettime(CLOCK_MONOTONIC_RAW, &t_start) == 0) {
        uint64_t ns, end, start = hra_ticks_read();
        nanosleep(&sleeptime, NULL);
        clock_gettime(CLOCK_MONOTONIC_RAW, &t_end);
        end = hra_ticks_read();
        ns = ((t_end.tv_sec - t_start.tv_sec) * NS_PER_SEC);
        ns += (t_end.tv_nsec - t_start.tv_nsec);

        double secs = (double)ns/NS_PER_SEC;
        g_ticks_per_sec = (uint64_t)((end - start)/secs);
    }
#else
#error "CLOCK_MONOTONIC_RAW not defined.  Cannot determine ticks per second"
#endif
#else
    /* We're using clock_gettime().  We're defining a tick as 1 ns. */
    g_ticks_per_sec = 1E9;
    printf("************************************************\n" \
           "*  WARNING: using clock_gettime() for timing   *\n" \
           "*  measurements.  Performance may be impacted. *\n" \
           "************************************************\n");
#endif
#endif
}


#define handle_error_en(en, msg) \
        do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

int hra_thread_create(void *tid, void *fn, void *arg, unsigned core_id)
{
#ifdef DPDK
    (void)tid;

    return rte_eal_remote_launch(fn, arg, core_id);
#else
    int ret;
    pthread_attr_t attr;

    if ((ret = pthread_attr_init(&attr)) == 0)
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core_id, &cpuset);
        if ((ret = pthread_attr_setaffinity_np(&attr, sizeof(cpu_set_t), &cpuset)) != 0)
        {
            /* error */
        }
        else if ((ret = pthread_create(tid, &attr, fn, arg)) != 0)
        {
            /* error */
        }
        pthread_attr_destroy(&attr);
    }

    return ret;
#endif
}

int hra_thread_join(void *tid, unsigned core_id, void **retval)
{
#ifdef DPDK
    (void)tid;
    (void)retval;
    return rte_eal_wait_lcore(core_id);
#else
    (void)core_id;
    return pthread_join(*(pthread_t*)tid, retval);
#endif
}

/*
 * TODO: add a return to this function.
 *       Let the caller decide what to do rather than call handle_error_en.
 */
void
hra_set_thread_affinity(int core_id)
{
#ifdef DPDK
    (void)core_id;
    /* Nothing. */
#else
    int s;
    cpu_set_t cpuset;
    pthread_t thread;

    thread = pthread_self();

    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    s = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (s != 0)
        handle_error_en(s, "pthread_setaffinity_np");

    /* Check the actual affinity mask assigned to the thread */
    s = pthread_getaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (s != 0)
        handle_error_en(s, "pthread_getaffinity_np");
#endif

    return;
}

int hra_get_queue_core_map(unsigned rxp_id, unsigned *queue_core, unsigned num_queues)
{
    unsigned qid = 0;

#ifdef DPDK
    (void)rxp_id;
    unsigned lcore_id;

    RTE_LCORE_FOREACH(lcore_id)
    {
        if (qid < num_queues)
        {
            queue_core[qid++] = lcore_id;
        }
        else
        {
            printf("%s %d, coding error. lcore %d, qid %d\n", __FUNCTION__, __LINE__, lcore_id, qid++);
        }
    }
#else
    (void)rxp_id;
    int max_cores = get_nprocs();
    for (qid = 0; qid < num_queues; qid++)
    {
        queue_core[qid] = qid % max_cores;
    }
#endif

    return qid;
}
