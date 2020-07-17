# SPDX-License-Identifier: BSD-3-Clause
# Copyright(c) 2020 Mellanox Technologies Ltd

deps += ['pci', 'bus_pci', 'common_mlx5', 'kvargs']
install_headers('rte_bus_mlx5_pci.h')
sources = files('mlx5_pci_bus.c')

cflags_options = [
	'-std=c11',
	'-Wno-strict-prototypes',
	'-D_BSD_SOURCE',
	'-D_DEFAULT_SOURCE',
	'-D_XOPEN_SOURCE=600'
]
foreach option:cflags_options
	if cc.has_argument(option)
		cflags += option
	endif
endforeach
