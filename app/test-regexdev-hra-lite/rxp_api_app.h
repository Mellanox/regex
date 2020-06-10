/**
 * @file    rxp_api_app.h
 * @author  Titan IC Systems <support@titanicsystems.com>
 *
 * @section DESCRIPTION
 *
 * Header used for API's between HRA and DPDK 
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

#ifndef _RXP_API_APP_H_
#define _RXP_API_APP_H_

/*
 * Number of jobs to scan before exiting.
 */
#define HRA_NUM_JOBS         1ull
#define RXP_NUM_QUEUES       1u      //#App queue/cores, not RXP Engs

#define INVALID_STRTOL_CONVERSION(value, string, endptr) \
                (errno == ERANGE) || \
                ((errno != 0) && (value == 0)) || \
                (endptr == string) || \
                (*endptr != '\0') || \
                ((int)value < 0)

#define RXP_MAX_JOB_LENGTH  16384
#define RXP_MAX_MATCHES     254


struct rxp_response_batch {
    /*! Size of response buffer. */
    size_t buf_size;

    /*! Response buffer. */
    uint8_t *buf;

    /*! Number of bytes in buffer actually used. */
    size_t buf_used;

    /*! Buffer offset of next response. Used by rxp_next_response(). */
    size_t next_offset;
};


struct rxp_response_desc {
    uint32_t    job_id;
    uint16_t    status;
    uint8_t     detected_match_count;
    uint8_t     match_count;
    uint16_t    primary_thread_count;
    uint16_t    instruction_count;
    uint16_t    latency_count;
    uint16_t    pmi_min_byte_ptr;
}  __attribute__ ((__packed__));


//New 64bit match tuple for Mellanox
struct rxp_match_tuple {
    uint32_t    rule_id;
    uint16_t    start_ptr;
    uint16_t    length;
}  __attribute__ ((__packed__));

struct rxp_response {
    struct rxp_response_desc header;
    struct rxp_match_tuple matches[0];
};


/*! Minimum size of a response batch buffer. */
#define RXP_RESP_BUF_SIZE_MIN (sizeof(struct rxp_response_desc) \
                            + (sizeof(struct rxp_match_tuple) * RXP_MAX_MATCHES))


int rxp_platform_init_app(unsigned rxp, int argc, char *argv[]);
int rxp_platform_uninit_app(unsigned rxp);
void *rxp_platform_malloc_app(const char *type, size_t size, unsigned align);
void rxp_platform_unmalloc_app(void *ptr);
void *rxp_platform_calloc_app(const char *type, size_t num, size_t size, unsigned align);
int rxp_queue_status_app(int rxp_handle, bool *rx_ready, bool *tx_ready);
int rxp_submit_job_app(int rxp_handle, uint32_t jobid, uint8_t *buf,
                   uint16_t len, uint16_t subset1, uint16_t subset2,
                   uint16_t subset3, uint16_t subset4, bool enable_hpm,
                   bool enable_anymatch);
int rxp_read_response_batch_app(int rxp_handle, struct rxp_response_batch *ctx);
struct rxp_response* rxp_next_response_app(struct rxp_response_batch *ctx);
int rxp_open_app(unsigned rxp);
int rxp_close_app(int rxp_handle);
int rxp_program_rules_app(unsigned rxp, const char *rulesfile, bool incremental);

#endif /* _RXP_API_APP_H_ */
