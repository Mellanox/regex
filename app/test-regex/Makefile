# SPDX-License-Identifier: BSD-3-Clause
# Copyright(c) 2020 Mellanox Corporation

include $(RTE_SDK)/mk/rte.vars.mk

ifeq ($(CONFIG_RTE_LIBRTE_REGEXDEV),y)

#
# library name
#
APP = testregex

CFLAGS += -O3
CFLAGS += $(WERROR_FLAGS)
CFLAGS += -Wno-deprecated-declarations

#
# all source are stored in SRCS-y
#
SRCS-y := main.c

include $(RTE_SDK)/mk/rte.app.mk

endif
