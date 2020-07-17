# SPDX-License-Identifier: BSD-3-Clause
# Copyright 2020 Mellanox Technologies, Ltd

include $(RTE_SDK)/mk/rte.vars.mk

#
# library name
#
LIB = librte_bus_mlx5_pci.a

CFLAGS += -O3 -Wall -Wextra
CFLAGS += $(WERROR_FLAGS)
CFLAGS += -Wno-strict-prototypes
CFLAGS += -I$(RTE_SDK)/drivers/common/mlx5
CFLAGS += -I$(BUILDDIR)/drivers/common/mlx5
CFLAGS += -I$(RTE_SDK)/drivers/common/mlx5/linux
CFLAGS += -I$(BUILDDIR)/drivers/bus/pci
CFLAGS += -I$(BUILDDIR)/drivers/bus
CFLAGS += -D_DEFAULT_SOURCE
LDLIBS += -lrte_eal
LDLIBS += -lrte_kvargs
LDLIBS += -lrte_common_mlx5
LDLIBS += -lrte_pci -lrte_bus_pci

# versioning export map
EXPORT_MAP := rte_bus_mlx5_pci_version.map

SRCS-y += mlx5_pci_bus.c

# DEBUG which is usually provided on the command-line may enable
# CONFIG_RTE_LIBRTE_MLX5_DEBUG.
ifeq ($(DEBUG),1)
CONFIG_RTE_LIBRTE_MLX5_DEBUG := y
endif

#
# Export include files
#
SYMLINK-y-include += rte_bus_mlx5_pci.h

include $(RTE_SDK)/mk/rte.lib.mk
