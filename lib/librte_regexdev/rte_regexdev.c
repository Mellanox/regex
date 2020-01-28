/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(C) 2019 Marvell International Ltd.
 * Copyright(C) 2020 Mellanox International Ltd.
 */

#include <string.h>

#include <rte_spinlock.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_string_fns.h>

#include "rte_regexdev.h"
#include "rte_regexdev_driver.h"

enum regex_dev_state {
	REGEX_DEV_FREE = 0,
	REGEX_DEV_USED,
};

static struct {
	struct rte_regex_dev_data data[RTE_MAX_REGEXDEV_DEVS];
} regex_dev_shared_data;

static struct rte_regex_dev regex_devices[RTE_MAX_REGEXDEV_DEVS];

static  enum regex_dev_state regex_device_state[RTE_MAX_REGEXDEV_DEVS];

/* spinlock for shared data allocation */
static rte_spinlock_t regex_shared_data_lock = RTE_SPINLOCK_INITIALIZER;


static uint16_t
regex_dev_find_free_dev(void)
{
	unsigned i;

	for (i = 0; i < RTE_MAX_REGEXDEV_DEVS; i++) {
		if (regex_device_state[i] == REGEX_DEV_FREE)
			return i;
	}
	return RTE_MAX_REGEXDEV_DEVS;
}

static const struct rte_regex_dev*
regex_dev_allocated(const char *name)
{
	unsigned i;

	for (i = 0; i < RTE_MAX_REGEXDEV_DEVS; i++) {
		if (regex_device_state[i] == REGEX_DEV_USED)
			if (strcmp(name, regex_devices[i].name))
				return &regex_devices[i];
	}
	return NULL;
}

static struct rte_regex_dev*
regex_dev_get(uint16_t dev_id)
{
	return (&regex_devices[dev_id]);
}

struct rte_regex_dev *
rte_regex_dev_register(const char *name)
{
	uint16_t dev_id;
	struct rte_regex_dev *regex_dev = NULL;
	size_t name_len;

	name_len = strnlen(name, RTE_REGEX_NAME_MAX_LEN);
	if (name_len == 0) {
		RTE_REGEXDEV_LOG(ERR, "Zero length regex device name\n");
		return NULL;
	}
	if (name_len >= RTE_REGEX_NAME_MAX_LEN) {
		RTE_REGEXDEV_LOG(ERR, "Regex device name is too long\n");
		return NULL;
	}
	/* Synchronize dev creation between primary and secondary threads. */
	rte_spinlock_lock(&regex_shared_data_lock);
	if (regex_dev_allocated(name) != NULL) {
		RTE_REGEXDEV_LOG
			(ERR, "RegEx device with name %s already allocated\n",
			 name);
		goto unlock;
	}
	dev_id = regex_dev_find_free_dev();
	if (dev_id == RTE_MAX_REGEXDEV_DEVS) {
		RTE_REGEXDEV_LOG
			(ERR, "Reached maximum number of regex devs\n");
		goto unlock;
	}
	regex_dev = regex_dev_get(dev_id);
	regex_dev->data = &regex_dev_shared_data.data[dev_id];
	strlcpy(regex_dev->name, name, sizeof(*regex_dev->name));
	regex_dev->data->dev_id = dev_id;
unlock:
	rte_spinlock_unlock(&regex_shared_data_lock);
	return regex_dev;
}

int
rte_regex_dev_unregister(struct rte_regex_dev *regex_dev)
{
	if (regex_dev == NULL)
		return -EINVAL;
	rte_spinlock_lock(&regex_shared_data_lock);
	memset(regex_dev->data, 0, sizeof(*regex_dev->data));
	rte_spinlock_unlock(&regex_shared_data_lock);
	return 0;
}

