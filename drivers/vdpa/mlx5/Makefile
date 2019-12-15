#   SPDX-License-Identifier: BSD-3-Clause
#   Copyright 2019 Mellanox Technologies, Ltd

include $(RTE_SDK)/mk/rte.vars.mk

# Library name.
LIB = librte_pmd_mlx5_vdpa.a

# Sources.
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_VDPA_PMD) += mlx5_vdpa.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_VDPA_PMD) += mlx5_vdpa_mem.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_VDPA_PMD) += mlx5_vdpa_cq.c
SRCS-$(CONFIG_RTE_LIBRTE_MLX5_VDPA_PMD) += mlx5_vdpa_virtq.c

# Basic CFLAGS.
CFLAGS += -O3
CFLAGS += -std=c11 -Wall -Wextra
CFLAGS += -g
CFLAGS += -I$(RTE_SDK)/drivers/common/mlx5
CFLAGS += -I$(RTE_SDK)/drivers/net/mlx5_vdpa
CFLAGS += -I$(RTE_SDK)/lib/librte_sched
CFLAGS += -I$(BUILDDIR)/drivers/common/mlx5
CFLAGS += -D_BSD_SOURCE
CFLAGS += -D_DEFAULT_SOURCE
CFLAGS += -D_XOPEN_SOURCE=600
CFLAGS += $(WERROR_FLAGS)
CFLAGS += -Wno-strict-prototypes
LDLIBS += -lrte_common_mlx5
LDLIBS += -lrte_eal -lrte_vhost -lrte_kvargs -lrte_bus_pci -lrte_sched

# A few warnings cannot be avoided in external headers.
CFLAGS += -Wno-error=cast-qual

EXPORT_MAP := rte_pmd_mlx5_vdpa_version.map
# memseg walk is not part of stable API
CFLAGS += -DALLOW_EXPERIMENTAL_API

CFLAGS += -DNDEBUG -UPEDANTIC

include $(RTE_SDK)/mk/rte.lib.mk
