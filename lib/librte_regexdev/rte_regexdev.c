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

static struct {
	struct rte_regex_dev_data data[RTE_MAX_REGEXDEV_DEVS];
} rte_regex_dev_shared_data;

static struct rte_regex_dev rte_regex_devices[RTE_MAX_REGEXDEV_DEVS];

/* spinlock for shared data allocation */
static rte_spinlock_t rte_regex_shared_data_lock = RTE_SPINLOCK_INITIALIZER;


static uint16_t
regex_dev_find_free_dev(void)
{
	unsigned i;

	for (i = 0; i < RTE_MAX_REGEXDEV_DEVS; i++) {
		if (rte_regex_devices[i].data != NULL &&
		    rte_regex_devices[i].data->name[0] == '\0')
			return i;
	}
	return RTE_MAX_REGEXDEV_DEVS;
}

static const struct rte_regex_dev*
regex_dev_allocated(const char *name)
{
	unsigned i;

	for (i = 0; i < RTE_MAX_REGEXDEV_DEVS; i++) {
		if (rte_regex_devices[i].data != NULL &&
		    strcmp(rte_regex_devices[i].data->name, name) == 0)
			return &rte_regex_devices[i];
	}
	return NULL;
}

static struct rte_regex_dev*
regex_dev_get(uint16_t dev_id)
{
	return (&rte_regex_devices[dev_id]);
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
	rte_spinlock_lock(&rte_regex_shared_data_lock);
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
	regex_dev->data = &rte_regex_dev_shared_data.data[dev_id];
	strlcpy(regex_dev->data->name, name, sizeof(regex_dev->data->name));
	regex_dev->data->dev_id = dev_id;
unlock:
	rte_spinlock_unlock(&rte_regex_shared_data_lock);
	return regex_dev;
}

int
rte_regex_dev_release(struct rte_regex_dev *regex_dev)
{
	if (regex_dev == NULL)
		return -EINVAL;
	rte_spinlock_lock(&rte_regex_shared_data_lock);
	memset(regex_dev->data, 0, sizeof(struct rte_regex_dev_data));
	rte_spinlock_unlock(&rte_regex_shared_data_lock);
	return 0;
}

