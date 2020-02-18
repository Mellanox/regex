/**
 * @file   host.h
 * @author Titan IC Systems <support@titan-ic.com>
 *
 */
#ifndef _HOST_H_
#define _HOST_H_

#include "mlx5_regex.h"
#include "rxp.h"

#define MLNX_LOGGING_ENABLED            1       //Switch off printf's.

#define RXP_POLL_CSR_FOR_VALUE_TIMEOUT  3000    //Poll timeout in ms
#define RXP_INITIALIZATION_TIMEOUT      60000   //Initialization timeout in ms
#define RXP_NUM_QUEUES                  1u      //#App queue/cores, not RXP Engs
#define MAX_RXP_ENGINES                 2u

/*
 * The number of jobs that can be batched in any transmission. 1 job per SQ as
 * want to get out of order responses
 * Note: Check if inline with RXP_TX_BURST_DEFAULT
 */
#define NUM_SQS             64  //RXP_TX_BURST_DEFAULT?

/* Used for Submit jobs to API instead of doing extra copy! */
struct rxp_mlnx_job_desc {
    uint16_t len;
    void *data_ptr;
};

struct rxp_database {
    void                    *database_ptr;
    struct mlx5dv_devx_umem *db_umem;
    struct mlx5_database_ctx db_ctx;
};

struct rxp_mlnx_dev {
    pthread_mutex_t         lock;
    struct                  ibv_device **dev_list;
    int                     num_devices;
    struct ibv_context      *device_ctx;
    struct rxp_database     rxp_db_desc[MAX_RXP_ENGINES];

    /* Keep account of the number of queues opened */
    uint8_t                 open_queues;  // Number of queues opened
    uint32_t                queues_active;
};

struct rxp_sq_buffers {
    void *input_p;      //Jobs data
    void *output_p;     //response matchs
    void *metadata_p;   ///Response header data

    struct mlx5_regex_buff *input_buff;
    struct mlx5_regex_buff *output_buff;
    struct mlx5_regex_buff *metadata_buff;
    struct mlx5_regex_wqe_ctrl_seg ctrl_seg;
    struct mlx5_wqe_data_seg input_seg;
    struct mlx5_wqe_data_seg output_seg;

    bool sq_busy;       //This is a flag to state if SQ is being used or not
    bool sq_resp_ready; //Set within mlnx_poll() highlighting data ready
    int work_id;        //TODO Is this really required as SQ id might = work_id!
    //TODO: Trying this method of store jobId into SQ. As Mlnx not handling it.
    uint32_t job_id;
};

struct rxp_queue {
    struct rxp_mlnx_dev     *rxp;
    struct mlx5_regex_ctx   *rxp_job_ctx;   //Create a regex ctx per thread
    struct rxp_sq_buffers   sq_buf[NUM_SQS];
    //uint32_t                num_jobs_processed;
    uint32_t                num_resp_to_read;
    int                     q_id;           // App Queue ID number
};


/* Prototypes: */
int mlnx_resume_rxp(uint8_t rxp_eng);
int mlnx_update_database(uint8_t rxp_eng);
int mlnx_set_database(uint8_t rxp_eng);
int mlnx_csr_read(uint32_t csr_addr_offset, uint32_t *returnVal,
                    uint8_t rxp_eng);
uint32_t mlnx_csr_write(uint32_t *value, uint32_t csr_addr_offset,
                    uint8_t rxp_eng);
int mlnx_write_rules(struct rxp_ctl_rules_pgm *rules, uint32_t rules_len,
                    uint8_t rxp_eng);
int mlnx_read_resp(struct rxp_queue *rxp_queue, uint8_t *buf, size_t buf_size,
                    uint32_t *num_returned_resp);
size_t mlnx_submit_job(struct rxp_queue *rxp_queue,
                        struct rxp_mlnx_job_desc *data,
                        uint16_t job_count);
int mlnx_poll(struct rxp_queue *rxp_queue, bool *rx_ready, bool *tx_ready);
int mlnx_open(struct rxp_queue *queue);
int mlnx_init(struct ibv_context *ctx);
int mlnx_close(struct rxp_mlnx_dev *rxp);
int mlnx_release(struct rxp_queue *queue);

#if MLNX_LOGGING_ENABLED
void mlnx_log(const char *fmt, ...);
#else
#define mlnx_log(x,...) //Do nothing if Mellanox logging disabled
#endif

#endif /* _HOST_H_ */
