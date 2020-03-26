/**
 * @file    hra_platform.h
 * @author  Titan IC Systems <support@titanicsystems.com>
 *
 * @section DESCRIPTION
 *
 * API's for using platform specific stuff in HRA apps,
 * e.g. TSC, thread affinity.
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

#ifndef _HRA_PLATFORM_H_
#define _HRA_PLATFORM_H_

#define NS_PER_SEC 1E9
#define US_PER_SEC 1E6

/**
 * This function needs to be called once before
 * using the hra_ticks_per_sec() API.
 * It determines how many ticks elapse in a one second period.
 */
void     hra_ticks_init(void);

/**
 * Return how many ticks elapse in a one second period.
 * Use this API to help convert between ticks and time in seconds.
 *
 * @return                  Number of ticks in one second.
 */
uint64_t hra_ticks_per_sec(void);

/**
 * Return the current tick counts.
 * This API is intended to be used in collaboration with hra_ticks_per_second()
 * to perform relative timing measurements, i.e. how long did some processing
 * take.
 * It is not a mechanism for date/time stamping.
 *
 * @return                  Number of ticks elapsed.
 */
uint64_t hra_ticks_read(void);


/**
 *
 * Launch a thread on the given cpu
 *
 * @param   tid       Thread id.
 * @param   fn        Thread function to execute.
 * @param   arg       Arg to pass to thread function
 * @param   core_id   Core to run the thread on.
 * @return            0 for success, otherwise and error.
 */
int hra_thread_create(void *tid, void *fn, void *arg, unsigned core_id);

/**
 *
 * 'Join' a thread.  Wait for thread to complete.
 *
 * @param   tid       Pointer to thread id.
 * @param   core_id   Core that the thread was running on.
 * @param   retval    For passing thread function return value back to caller.
 * @return            0 for success, otherwise and error.
 */

int hra_thread_join(void *tid, unsigned core_id, void **retval);

/**
 *
 * This API is for setting the CPU affinity of the calling thread.
 *
 * @param                  CPU to run the thread on.
 */
void hra_set_thread_affinity(int core_id);

/**
 *
 * This API is getting a default mapping of RXP queue to cpu core.
 *
 * @param  rxp_id        RXP port id.
 * @param  queue_core    Array for returning core id per queue.
 * @param  num_queues    Size of queue_core array.
 */

int hra_get_queue_core_map(unsigned rxp_id, unsigned *queue_core, unsigned num_queues);

#endif /* _HRA_PLATFORM_H_ */
