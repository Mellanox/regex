/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2019 Mellanox Corporation
 */

#ifndef _RTE_REGEX_CORE_H_
#define _RTE_REGEX_CORE_H_

/**
 * @file
 *
 * RTE RegEx Device internal header.
 *
 * This header contains internal data types. But they are still part of the
 * public API because they are used by inline functions in the published API.
 *
 * Applications should not use these directly.
 *
 */

struct rte_regex_dev;

typedef int (*regex_dev_info_get_t)(struct rte_regex_dev *dev,
				    struct rte_regex_dev_info *info);
/**< @internal Get the regex device info. */

typedef int (*regex_dev_configure_t)(struct rte_regex_dev *dev,
				     const struct rte_regex_dev_config *cfg);
/**< @internal Configure the regex device. */

typedef int (*regex_dev_qp_setup_t)(struct rte_regex_dev *dev, uint8_t id,
				    const struct rte_regex_qp_conf *qp_conf);
/**< @internal Setup a queue pair.*/

typedef int (*regex_dev_start_t)(struct rte_regex_dev *dev);
/**< @internal Start the regex device. */

typedef int (*regex_dev_stop_t)(struct rte_regex_dev *dev);
/**< @internal Stop the regex device. */

typedef int (*regex_dev_close_t)(struct rte_regex_dev *dev);
/**< @internal Close the regex device. */

typedef int (*regex_dev_attr_get_t)(struct rte_regex_dev *dev,
				    enum rte_regex_dev_attr_id id,
				    void *value);
/**< @internal Get selected attribute from regex device. */

typedef int (*regex_dev_attr_set_t)(struct rte_regex_dev *dev,
				    enum rte_regex_dev_attr_id id,
				    void *value);
/**< @internal Set selected attribute to regex device. */

typedef int (*regex_dev_rule_db_update_t)(struct rte_regex_dev *dev,
				    	  const struct rte_regex_rule *rules,
					  uint16_t nb_rules);
/**< @internal Update the rule database for the regex device. */

typedef int (*regex_dev_rule_db_import_t)(struct rte_regex_dev *dev,
				    	  const char *rule_db,
					  uint32_t rule_db_len);
/**< @internal Upload a pre created rule database to the regex device. */

typedef int (*regex_dev_rule_db_export_t)(struct rte_regex_dev *dev,
				    	  const char *rule_db);
/**< @internal Export the current rule database from the regex device. */

typedef int (*regex_dev_xstats_names_get_t)(struct rte_regex_dev *dev,
				    	    struct rte_regex_dev_xstats_map
					    *xstats_map);
/**< @internal Get xstats name map for the regex device. */

typedef int (*regex_dev_xstats_get_t)(struct rte_regex_dev *dev,
				      const uint16_t ids[], uint64_t values[],
				      uint16_t n);
/**< @internal Get xstats values for the regex device. */

typedef int (*regex_dev_xstats_by_name_get_t)(struct rte_regex_dev *dev,
					      const char *name, uint64_t value);
/**< @internal Get xstat value for the regex device based on the xstats name. */

typedef int (*regex_dev_xstats_reset_t)(struct rte_regex_dev *dev,
					const uint16_t ids[], uint64_t values[],
					uint16_t n);
/**< @internal Reset xstats values for the regex device. */

typedef int (*regex_dev_xstats_reset_t)(struct rte_regex_dev *dev,
					const uint16_t ids[], uint64_t values[],
					uint16_t n);
/**< @internal Reset xstats values for the regex device. */

typedef int (*regex_dev_selftest_t)(struct rte_regex_dev *dev);
/**< @internal Trigger regex self test. */

typedef int (*regex_dev_dump_t)(struct rte_regex_dev *dev, FILE *f);
/**< @internal Dump internal information about the regex device. */

typedef int (*regex_dev_enqueue_t)(struct rte_regex_dev *dev, uint16_t qp_id,
				   struct rte_regex_ops **ops, uint16_t nb_ops);
/**< @internal Enqueue a burst of scan requests to a queue on regex device. */

typedef int (*regex_dev_dequeue_t)(struct rte_regex_dev *dev, uint16_t qp_id,
				   struct rte_regex_ops **ops,
				   uint16_t nb_ops);
/**< @internal Dequeue a burst of scan response from a queue on regex device. */

/**
 * regex device operations
 */
struct rte_regex_dev_ops {
	regex_dev_info_get_t dev_info_get;
	regex_dev_configure_t dev_configure;
	regex_dev_qp_setup_t dev_qp_setup;
	regex_dev_start_t dev_start;
	regex_dev_stop_t dev_stop;
	regex_dev_close_t dev_close;
	regex_dev_rule_db_update_t dev_rule_db_update;
	regex_dev_rule_db_import_t dev_db_import;
	regex_dev_rule_db_export_t dev_db_export;
	regex_dev_xstats_names_get_t dev_xstats_names_get;
	regex_dev_xstats_get_t dev_xstats_get;
	regex_dev_xstats_by_name_get_t dev_xstats_by_name_get;
	regex_dev_xstats_reset_t dev_xstats_reset;
	regex_dev_selftest_t dev_selftest;
	regex_dev_dump_t dev_dump;

	/** Reserved for future extension */
	void *reserved[5];
};

/**
 * @internal
 * The data part, with no function pointers, associated with each regex device.
 *
 * This structure is safe to place in shared memory to be common among different
 * processes in a multi-process configuration.
 */
struct rte_regex_dev_data {
	char name[RTE_REGEX_NAME_MAX_LEN]; /**< Unique identifier name */
	uint16_t nb_qp; /**< Number of queue pairs. */
	void *dev_private; /**< PMD-specific private data. */
	struct rte_regex_dev_config dev_conf;
	/**< Configuration applied to device. */
	uint16_t port_id;           /**< Device [external] port identifier. */
	uint64_t reserved_64s[4]; /**< Reserved for future fields */
	void *reserved_ptrs[4];   /**< Reserved for future fields */
} __rte_cache_aligned;

/**
 * @internal
 * The generic data structure associated with each RegEx device.
 *
 * Pointers to burst-oriented packet receive and transmit functions are
 * located at the beginning of the structure, along with the pointer to
 * where all the data elements for the particular device are stored in shared
 * memory. This split allows the function pointer and driver data to be per-
 * process, while the actual configuration data for the device is shared.
 */
struct rte_regex_dev {
	regex_dev_enqueue_t enqueue;
	regex_dev_dequeue_t dequeue;
	struct rte_regex_dev_data *data;  /**< Pointer to device data. */
	const struct regex_dev_ops *dev_ops; /**< Functions exported by PMD */
	struct rte_device *device; /**< Backing device */
	uint64_t reserved_64s[4]; /**< Reserved for future fields */
	void *reserved_ptrs[4];   /**< Reserved for future fields */
} __rte_cache_aligned;

#endif /* _RTE_REGEX_CORE_H_ */
