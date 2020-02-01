# SPDX-License-Identifier: BSD-3-Clause
# Copyright(c) 2016-2017 Intel Corporation
# Copyright(c) 2020 Mellanox Technologies

include $(RTE_SDK)/mk/rte.vars.mk


APP = dpdk-test-pmd-regexdev

CFLAGS += $(WERROR_FLAGS)
CFLAGS += -DALLOW_EXPERIMENTAL_API
CFLAGS += -O3

# all source are stored in SRCS-y
SRCS-y := main.c

LDLIBS += -lrte_regexdev

include $(RTE_SDK)/mk/rte.app.mk
