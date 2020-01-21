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

static struct rte_regex_dev *regex_devices[RTE_MAX_REGEXDEV_DEVS];

int rte_regex_dev_logtype;

/* spinlock for shared data allocation */
static rte_spinlock_t regex_shared_data_lock = RTE_SPINLOCK_INITIALIZER;


static uint16_t
regex_dev_find_free_dev(void)
{
	unsigned i;

	for (i = 0; i < RTE_MAX_REGEXDEV_DEVS; i++) {
		if (regex_devices[i] == NULL)
			return i;
	}
	return RTE_MAX_REGEXDEV_DEVS;
}

static const struct rte_regex_dev*
regex_dev_allocated(const char *name)
{
	unsigned i;

	for (i = 0; i < RTE_MAX_REGEXDEV_DEVS; i++) {
		if (regex_devices[i] != NULL)
			if (!strcmp(name, regex_devices[i]->dev_name))
				return regex_devices[i];
	}
	return NULL;
}

int
rte_regex_dev_register(struct rte_regex_dev *dev)
{
	uint16_t dev_id;
	int res;

	if (dev->dev_ops == NULL) {
		RTE_REGEXDEV_LOG(ERR, "RegEx device invalid device ops\n");
		return -EINVAL;
	}

	/* Synchronize dev creation between primary and secondary threads. */
	rte_spinlock_lock(&regex_shared_data_lock);
	if (regex_dev_allocated(dev->dev_name) != NULL) {
		RTE_REGEXDEV_LOG
			(ERR, "RegEx device with name %s already allocated\n",
			 dev->dev_name);
		res = -ENOMEM;
		goto unlock_register;
	}
	dev_id = regex_dev_find_free_dev();
	if (dev_id == RTE_MAX_REGEXDEV_DEVS) {
		RTE_REGEXDEV_LOG
			(ERR, "Reached maximum number of regex devs\n");
		res = -ENOMEM;
		goto unlock_register;
	}
	dev->dev_id = dev_id;
	regex_devices[dev_id] = dev;
	res = dev_id;
unlock_register:
	rte_spinlock_unlock(&regex_shared_data_lock);
	return res;
}

void
rte_regex_dev_unregister(struct rte_regex_dev *dev)
{
	rte_spinlock_lock(&regex_shared_data_lock);
	regex_devices[dev->dev_id] = NULL;
	rte_spinlock_unlock(&regex_shared_data_lock);
}
