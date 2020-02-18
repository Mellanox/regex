/**
 * @file   rxp-api.h
 * @author Titan IC Systems <support@titan-ic.com>
 *
 */
#ifndef _RXP_API_H_
#define _RXP_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* The structure definitions live in the kernel headers */
#include "rxp.h"

#include <infiniband/mlx5dv.h>

/**
 * \mainpage
 *
 * \section intro_sec Introduction
 *
 * The Titan IC Regular eXpression Processor (RXP) is a custom-purpose processor
 * developed to efficiently process data to detect matches for a set of
 * user-defined string and regular expression (RegEx) based rules.
 *
 * \subsection Scope
 *
 * This document is intended as a reference for those developing their own
 * application utilising the Titan IC RXP. It documents the C API for
 * interacting with the RXP to submit jobs for data to be scanned and receive
 * the associated responses. This API is designed to be lightweight, allowing
 * for easy integration into existing applications without requiring major
 * restructuring.
 *
 * \section flow_sec Typical Programming Flow
 *
 * Typical application utilisation of the RXP will follow the following
 * programming flow.
 *
 * - rxp_program_rules() or rxp_program_rules_struct()
 * - rxp_open()
 * - rxp_submit_job()
 * - rxp_read_response_batch() and rxp_next_response()
 * - rxp_close()
 */

/*! Default number of jobs to put in a single batch for RXP submission. */
#define RXP_TX_BURST_DEFAULT 64

/*! Minimum number of jobs to put in a single batch for RXP submission. */
#define RXP_TX_BURST_MIN 1

/*! Maximum number of jobs to put in a single batch for RXP submission. */
#define RXP_TX_BURST_MAX 256

/*! Minimum size of a response batch buffer. */
#define RXP_RESP_BUF_SIZE_MIN (sizeof(struct rxp_response_desc) \
                            + (sizeof(struct rxp_match_tuple) * RXP_MAX_MATCHES))

/**
 * Context holding a batch of jobs ready for RXP submission
 *
 * This structure is used to accumulate up to 'max_jobs' jobs ready for
 * submission to the RXP.
 * Use functions rxp_job_batch_alloc() and rxp_job_batch_free() to
 * allocate or free the resources used by the structure.
 */
struct rxp_job_batch {
    /*! Maximum number of jobs in the batch. */
    size_t max_jobs;

    /*! Number of jobs currently ready to submit */
    size_t count;

    /*! Total number of job bytes ready to submit. */
    size_t bytes_total;
    /*! Threshold total job bytes before submitting. Zero if not using threshold. */
    size_t bytes_threshold;

    /*! Array of job meta-data, one entry for each batched job. */
    struct {
        /*! Job descriptor of the batched job */
        struct rxp_job_desc desc;
        /*! Pointer to the data to be scanned for the batched job */
        uint8_t             *data;
        /*! Length of the data to be scanned for the batched job */
        size_t              len;
    } job[0];
};

/**
 * Context holding a batch of responses read from the RXP.
 *
 * This structure is passed to the rxp_read_response_batch() API.
 * A batch of variable length responses (of type struct rxp_response)
 * will be read into the flat buffer.
 * Subsequently, the struct is passed to the rxp_next_response() API
 * to iterate through the individual responses in the flat buffer.
 *
 * The batch buffer must be at least of size RXP_RESP_BUF_SIZE_MIN.
 * This is sufficient to contain a response with the maximum number of matches.
 * However, in many cases a response will be smaller.  In that case, the
 * buffer will be able to contain multiple responses.
 */
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

/**
 * Structure containing RXP provided statistics
 *
 * This structure contains various pieces of information the RXP provides about
 * jobs + bytes processed, errors seen and responses returned.
 */
struct rxp_stats {
    /*! Number of jobs the RXP has processed */
    uint32_t num_jobs;
    /*! Number of responses the RXP has returned */
    uint32_t num_responses;
    /*! Number of matches the RXP has seen across all jobs processed */
    uint32_t num_matches;
    /*! Number of submitted jobs that resulted in an error */
    uint32_t num_job_errors;
    /*! Total count of data bytes scanned */
    uint64_t num_bytes;
};

/**
 * Structure containing cluster statistics
 *
 * Stores details about the statistics related to a single RXP cluster.
 */
struct rxp_cluster_stats {
    /*! Number of JCEs idle over the previous 256 clock cycles */
    unsigned jce_idle_id;
    /*! Number of TCEs idle over the previous 256 clock cycles */
    unsigned tce_idle_id;
    /*! Percentage hit rate over the previous 256 clock cycles */
    unsigned hit_duty_cycle;
    /*! Percentage instruction execution over the previous 256 clock cycles */
    unsigned instruction_duty_cycle;
};

/**
 * Structure containing level 2 cache statistics
 *
 * Stores details about the RXP L2 cache utilisation.
 */
struct rxp_l2_cache_stats {
    /*! Percentage cache hit rate over previous 256 clock cycles */
    unsigned cache_hit_duty_cycle;
    /*! Percentage cache miss rate over previous 256 clock cycles */
    unsigned cache_miss_duty_cycle;
    /*! Number of entries in the cache request fifo */
    unsigned request_fifo_num_entries;
    /*! Number of entries in the read pending completion fifo */
    unsigned read_pending_completion_fifo_num_entries;
};

/**
 * Structure containing statistics related to the prefix engine
 *
 * Stores details related to the prefix engine (PE) utilisation
 */
struct rxp_pe_stats {
    /*! Percentage rate for new data over the previous 256 clock cycles */
    unsigned nd_duty_cycle;
    /*! Percentage rate for new primary thread creation over previous 256 clock cycles */
    unsigned primary_thread_valid_duty_cycle;
};

/**
 * RXP performance statistics
 *
 * Ties together the performance statistics structures for each component of
 * the RXP.
 */
struct rxp_perf_stats {
    /*! Cluster statistics for each RXP cluster */
    struct rxp_cluster_stats cluster[16];
    /*! Statistics for the RXP L2 cache */
    struct rxp_l2_cache_stats l2_cache;
    /*! Statistics for the prefix engine */
    struct rxp_pe_stats pe;
    /*! FIFO entry details for the MPFE */
    unsigned mpfe_fifo_entries[4];
};

/**
 * Submit a job to the RXP
 *
 * Takes the buffer provided, constructs the appropriate job descriptor and
 * submits it to the RXP for scanning. subset1-4 indicate which rule subsets
 * (as defined in the compiled rules file) should be used to scan the data.
 * At least subset1 must be non-zero.
 *
 * jobid is used when providing responses (via rxp_read_response_batch()), to
 * enable the caller to match responses up to submitted jobs. It must be
 * non-zero but is otherwise not interpreted by the RXP.
 *
 * @param rxp_handle  Handle to the RXP as returned by rxp_open()
 * @param jobid       A non-zero id to enable association of the RXP response
 * @param buf         Pointer to the data to be scanned by the RXP
 * @param len         Length of the data to be scanned by the RXP
 * @param subset1     Subset ID #1 for the rule set to use to scan this job
 * @param subset2     Subset ID #2 for the rule set to use to scan this job
 * @param subset3     Subset ID #3 for the rule set to use to scan this job
 * @param subset4     Subset ID #4 for the rule set to use to scan this job
 * @param enable_hpm  Enable Highest Priority Matching (HPM) mode
 * @param enable_anymatch  Enable Any Match Termination mode
 */
int rxp_submit_job(int rxp_handle, uint32_t jobid, const uint8_t *buf,
    uint16_t len, uint16_t subset1, uint16_t subset2, uint16_t subset3,
    uint16_t subset4, bool enable_hpm, bool enable_anymatch);

/**
 * Add a new job to a batch of jobs for submission to the RXP
 *
 * Takes the buffer provided, constructs the appropriate job descriptor and
 * builds a job ready to send to the RXP for scanning. subset1-4 indicate which
 * rule subsets (as defined in the compiled rules file) should be used to scan
 * the data. At least subset1 must be non-zero.
 *
 * This function differs from rxp_submit_job() in that the contructed job is
 * not immediately submitted to the RXP. Instead up to 'max_jobs' jobs
 * are prepared and submitted at once, to improve throughput.
 * The jobs will be submitted if the maximum number of jobs have been batched
 * or optionally a threshold number of bytes has been reached.
 * If rxp_scan_job() submits the jobs, it will return a positive value indicating the
 * number of jobs that were submitted. Alternatively an application can force
 * all outstanding jobs to be sent to the RXP by calling rxp_dispatch_jobs()
 *
 * jobid is used when providing responses (via rxp_read_response_batch()), to
 * enable the caller to match responses up to submitted jobs. It must be
 * non-zero but is otherwise not interpreted by the RXP.
 *
 * @param rxp_handle  Handle to the RXP as returned by rxp_open()
 * @param ctx         Context structure holding the jobs being batched up
 * @param jobid       A non-zero id to enable association of the RXP response
 * @param buf         Pointer to the data to be scanned by the RXP
 * @param len         Length of the data to be scanned by the RXP
 * @param subset1     Subset ID #1 for the rule set to use to scan this job
 * @param subset2     Subset ID #2 for the rule set to use to scan this job
 * @param subset3     Subset ID #3 for the rule set to use to scan this job
 * @param subset4     Subset ID #4 for the rule set to use to scan this job
 * @param enable_hpm  Enable Highest Priority Matching (HPM) mode
 * @param enable_anymatch  Enable Any Match Termination mode
 */
int rxp_scan_job(int rxp_handle, struct rxp_job_batch *ctx, uint32_t jobid,
    const uint8_t *buf, uint16_t len, uint16_t subset1, uint16_t subset2,
    uint16_t subset3, uint16_t subset4, bool enable_hpm, bool enable_anymatch);

/**
 * Allocate a job batch structure.
 *
 * Allocate a structure of type rxp_job_batch and all the memory therein.
 * Parameter max_jobs determines the number of jobs that can be stored
 * in the batch.
 * Parameter bytes_threshold defines whether jobs should be dispatched after
 * a given number of job bytes have been batched. If zero, this threshold
 * will not be used by rxp_scan_job().
 *
 * @param max_jobs        How many jobs slots to allocate in the batch structure.
 * @param bytes_threshold Number of bytes to batch before dispatching.
 */
struct rxp_job_batch *rxp_job_batch_alloc(size_t max_jobs, size_t bytes_threshold);


/**
 * Free a job batch structure.
 *
 * Free a structure of type rxp_job_batch and all the memory therein.
 *
 * @param ctx         Pointer to batch structure.
 */
void rxp_job_batch_free(struct rxp_job_batch *ctx);


/**
 * Send a batch of jobs to the RXP for processing
 *
 * Provides a method of explicitly requesting that a batch of jobs prepared by
 * rxp_scan_job() is immediately submitted to the RXP for processing.
 *
 * Returns a positive value indicating the number of jobs which were submitted
 * to the RXP.
 *
 * @param rxp_handle  Handle to the RXP as returned by rxp_open()
 * @param ctx         Context structure holding the jobs being batched up
 */
int rxp_dispatch_jobs(int rxp_handle, struct rxp_job_batch *ctx);

/**
 * Read a batch of available responses from the RXP.
 *
 * Read the next available responses from the RXP for the queue associated
 * with the provided rxp_handle.
 * If no responses are currently available this function will block within
 * the kernel driver waiting for at least one response.
 * Use rxp_queue_status() in order to check in advance if a response is
 * ready to be received.
 * The function will continue reading responses until no more are availble
 * or until the buffer has insufficient space to read the next response.
 * Only complete responses are returned.
 * Bearing in mind that responses are variable length, the number of responses
 * may vary. Use API rxp_next_response() to iterate through the responses in the
 * batch buffer.
 *
 * The batch buffer must be preallocated by the caller and be at least of
 * size RXP_RESP_BUF_SIZE_MIN.
 * This is sufficient to contain a response with the maximum number of matches.
 * However, in many cases a response will be smaller.  In that case, the
 * buffer will be able to contain multiple responses.
 * See also struct rxp_response_batch.
 *
 * Returns a positive value indicating number of responses read,
 * or a negative value indicating an error.
 * On success, the number of response bytes actually read will be
 * populated in the batch context structure.
 *
 * @param rxp_handle Handle to the RXP as returned by rxp_open()
 * @param ctx        Pointer to batch context structure.
 */
int rxp_read_response_batch(int rxp_handle, struct rxp_response_batch *ctx);

/**
 * Iterate through a batch of responses.
 *
 * This function is used in collaboration with rxp_read_response_batch().
 * rxp_read_response_batch() reads N responses into a flat buffer.
 * Then this function is used to iterate through the individual responses
 * in the aforementioned buffer.
 *
 * Returns a pointer to the next response descriptor or NULL if there
 * are no more.
 *
 * @param ctx        Pointer to batch context structure.
 */
struct rxp_response *rxp_next_response(struct rxp_response_batch *ctx);

/**
 * Open a connection to the RXP
 *
 * This function opens a connection to the RXP and returns a handle which can
 * subsequentally be used to access it. Each open returns a handle to a unique
 * RXP queue.
 *
 * In the event of an error -1 will be returned and errno will be set
 * appropriately.
 *
 * - EBUSY: No free RXP queue was available to be assigned
 * - EPERM: The process does not have the appropriate permissions to access the RXP device file
 *
 * @param  rxp    RXP instance to use. Usually 0, unless there are multiple RXPs present in the system.
 * @return Handle to the RXP; -1 if an error occurs. errno is set as above.
 */
int rxp_open(unsigned rxp, struct ibv_context *ctx);

/**
 * Close the connection to the RXP
 *
 * Closes the connection to the RXP and releases the assigned queue. Any
 * outstanding job responses which have not been received are discarded.
 *
 * @param rxp_handle Handle to the RXP as returned by rxp_open()
 */
int rxp_close(int rxp_handle);

/**
 * Query the readiness of the RXP to accept a new job and/or return a response
 *
 * Provides a non-blocking way to check the status of the TX + RX queues
 * associated with the provided RXP handle. If rx_ready is true then there is
 * at least one response waiting to be checked. If tx_ready is true then the
 * RXP has space for a job to be submitted.
 *
 * Either of rx_ready and tx_ready can be NULL if the status of the respective
 * queue is not of interest.
 *
 * @param rxp_handle Handle to the RXP as returned by rxp_open()
 * @param rx_ready   Pointer to where to store the RX status
 * @param tx_ready   Pointer to where to store the TX status
 */
int rxp_queue_status(int rxp_handle, bool *rx_ready, bool *tx_ready);

/**
 * Program the provided ROF file into the RXP
 *
 * This function configures the RXP to perform pattern matching against the
 * provided ROF file, as generated by the RXP Compiler.
 *
 * @param rxp         RXP instance to use. Usually 0, unless there are multiple RXPs present in the system.
 * @param rulesfile   ROF file to program into the RXP
 * @param incremental Indicate whether this is an incremental update, and thus that the RXP should not be reset.
 */
int rxp_program_rules(unsigned rxp, const char *rulesfile, bool incremental, struct ibv_context *ctx);

/**
 * Program the provided ROF structure into the RXP
 *
 * This function configures the RXP to perform pattern matching against the
 * provided ROF file, as generated by the RXP Compiler.
 *
 * @param rxp         RXP instance to use. Usually 0, unless there are multiple RXPs present in the system.
 * @param rof         ROF structure to program into the RXP
 * @param incremental Indicate whether this is an incremental update, and thus that the RXP should not be reset.
 */
int rxp_program_rules_struct(unsigned rxp, const struct rxp_rof *rof, bool incremental);

/**
 * Query the RXP for its statistics information
 *
 * Reads the statistics information from the RXP driver; jobs, responses,
 * matches and job errors are all 32 bit values which will wrap. Received
 * bytes is a 64 bit value. Statistics are not cleared on a read.
 *
 * @param rxp   RXP instance to use. Usually 0, unless there are multiple RXPs present in the system.
 * @param stats Pointer to the structure which will hold the retrieved statistics.
 */
int rxp_read_stats(unsigned rxp, struct rxp_stats *stats);

/**
 * Query the RXP for performance related statistics information
 *
 * Reads information from the RXP driver about how busy each cluster is, the
 * level 2 cache status, details about the prefix engine statistics and fifo
 * depths for the MPFE. These can be useful for determining whether the RXP is
 * being kept fully utilised, or if there is any particular bottleneck
 * preventing full efficiency.
 *
 * @param rxp   RXP instance to use. Usually 0, unless there are multiple RXPs present in the system.
 * @param stats Pointer to the structure which will hold the retrieved statistics.
 */
int rxp_read_perf_stats(unsigned rxp, struct rxp_perf_stats *stats);

/**
 * Query the RXP for its 'max_matches' value.
 *
 * Reads the 'max_matches' value from the RXP driver, i.e.
 * the maximum number of matches that the RXP will return.
 *
 * @param rxp         RXP instance to use.
 *                    Usually 0, unless multiple RXPs present in the system.
 * @param max_matches Pointer to uint32_t for returning the value.
 */
int rxp_read_max_matches(unsigned rxp, uint32_t *max_matches);

/**
 * Set the RXP 'max_matches' value.
 *
 * Update the 'max_matches' value in the RXP driver, i.e.
 * the maximum number of matches that the RXP will return.
 *
 * @param rxp         RXP instance to use.
 *                    Usually 0, unless multiple RXPs present in the system.
 * @param max_matches Value to set.
 */
int rxp_set_max_matches(unsigned rxp, uint32_t max_matches);

/**
 * Query the RXP for its 'max_prefixes' value.
 *
 * Reads the 'max_prefixes' value from the RXP driver, i.e.
 * the maximum number of prefixes that the RXP will detect before
 * terminating a job and incrementing the DDOS counter.
 *
 * @param rxp          RXP instance to use.
 *                     Usually 0, unless multiple RXPs present in the system.
 * @param max_prefixes Pointer to uint32_t for returning the value.
 */
int rxp_read_max_prefixes(unsigned rxp, uint32_t *max_prefixes);

/**
 * Set the RXP 'max_prefixes' value.
 *
 * Update the 'max_prefixes' value in the RXP driver, i.e.
 * the maximum number of prefixes that the RXP will detect before
 * terminating a job and incrementing the DDOS counter.
 *
 * @param rxp           RXP instance to use.
 *                      Usually 0, unless multiple RXPs present in the system.
 * @param max_prefixes  Value to set.
 */
int rxp_set_max_prefixes(unsigned rxp, uint32_t max_prefixes);

/**
 * Query the RXP for its 'max_latency' value.
 *
 * Reads the 'max_latency' value from the RXP driver, i.e.
 * the maximum time that primary threads will be executed for before
 * a job is terminated. Each increment of max_latency corresponds to 256
 * clocks cycles within the RXP core.
 *
 * @param rxp         RXP instance to use.
 *                    Usually 0, unless multiple RXPs present in the system.
 * @param max_latency Pointer to uint32_t for returning the value.
 */
int rxp_read_max_latency(unsigned rxp, uint32_t *max_latency);

/**
 * Set the RXP 'max_latency' value.
 *
 * Update the 'max_latency' value in the RXP driver, i.e.
 * the maximum time that primary threads will be executed for before
 * a job is terminated. Each increment of max_latency corresponds to 256
 * clocks cycles within the RXP core.
 *
 * @param rxp         RXP instance to use.
 *                    Usually 0, unless multiple RXPs present in the system.
 * @param max_latency Value to set.
 */
int rxp_set_max_latency(unsigned rxp, uint32_t max_latency);

/**
 * Query the RXP for its 'max_pri_threads' value.
 *
 * Reads the 'max_pri_threads' value from the RXP driver, i.e.
 * the maximum number of primary threads that will be assigned to a job.
 *
 * @param rxp             RXP instance to use.
 *                        Usually 0, unless multiple RXPs present in the system.
 * @param max_pri_threads Pointer to uint32_t for returning the value.
 */
int rxp_read_max_pri_threads(unsigned rxp, uint32_t *max_pri_threads);

/**
 * Set the RXP 'max_pri_threads' value.
 *
 * Update the 'max_pri_threads' value in the RXP driver, i.e.
 * the maximum number of primary threads that will be assigned to a job.
 *
 * @param rxp             RXP instance to use.
 *                        Usually 0, unless multiple RXPs present in the system.
 * @param max_pri_threads Value to set.
 */
int rxp_set_max_pri_threads(unsigned rxp, uint32_t max_pri_threads);

#ifdef __cplusplus
}
#endif

#endif /* _RXP_API_H_ */
