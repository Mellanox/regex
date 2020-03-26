# SPDX-License-Identifier: BSD-3-Clause
# Copyright(c) 2016-2017 Intel Corporation
# Copyright(c) 2020 Mellanox Technologies

include $(RTE_SDK)/mk/rte.vars.mk


APP = dpdk-test-pmd-regexdev

CFLAGS += $(WERROR_FLAGS)
CFLAGS += -DALLOW_EXPERIMENTAL_API
CFLAGS += -O3

# all source are stored in SRCS-y
SRCS-y += hra_lite_main.c
SRCS-y += hra_platform.c
SRCS-y += rxp_mlnx_api_dpdk.c

LDLIBS += -lrte_regexdev

include $(RTE_SDK)/mk/rte.app.mk

#CFLAGS += -O3 -g -Wall -I ./include -D DPDK

#hra_lite: hra_lite_main.o hra_platform.o
#	$(CC) $(LDFLAGS) -o $@ $^ -lpthread -lrxpapi -lrt

#clean:
#	rm -f hra_lite_main.o hra_platform.o hra_lite

