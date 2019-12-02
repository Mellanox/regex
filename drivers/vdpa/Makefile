#   SPDX-License-Identifier: BSD-3-Clause
#   Copyright 2019 Mellanox Technologies, Ltd

include $(RTE_SDK)/mk/rte.vars.mk

# DIRS-$(<configuration>) += <directory>

ifeq ($(CONFIG_RTE_LIBRTE_VHOST),y)
DIRS-$(CONFIG_RTE_LIBRTE_MLX5_VDPA_PMD) += mlx5
ifeq ($(CONFIG_RTE_EAL_VFIO),y)
DIRS-$(CONFIG_RTE_LIBRTE_IFC_PMD) += ifc
endif
endif # $(CONFIG_RTE_LIBRTE_VHOST)

include $(RTE_SDK)/mk/rte.subdir.mk
