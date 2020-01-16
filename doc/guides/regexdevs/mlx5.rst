..  SPDX-License-Identifier: BSD-3-Clause
    Copyright 2020 Mellanox Technologies, Ltd

.. include:: <isonum.txt>

MLX5 RegEx driver
=================

The MLX5 RegEx (Regular Expression) driver library
(**librte_pmd_mlx5_regex**) provides support for **Mellanox BlueField 2**
families of 25/50/100/200 Gb/s adapters.

.. note::

   Due to external dependencies, this driver is disabled in default
   configuration of the "make" build. It can be enabled with
   ``CONFIG_RTE_LIBRTE_MLX5_REGEX_PMD=y`` or by using "meson" build system which
   will detect dependencies.


Design
------

This PMD is configuring the RegEx HW engine.
For the PMD to work, the application must supply
a precompiled rule file in rof2 format.

The PMD can use libibverbs and libmlx5 to access the device firmware
or directly the hardware components.
There are different levels of objects and bypassing abilities
to get the best performances:

- Verbs is a complete high-level generic API
- Direct Verbs is a device-specific API
- DevX allows to access firmware objects
- Direct Rules manages flow steering at low-level hardware layer

Enabling librte_pmd_mlx5_regex causes DPDK applications to be linked against
libibverbs.

A Mellanox mlx5 PCI device can be probed by either net/mlx5 driver or regex/mlx5
driver but not in parallel. Hence, the user should decide the driver by dissabling
the net device using ``CONFIG_RTE_LIBRTE_MLX5_PMD``.

Supported NICs
--------------

* Mellanox\ |reg| BlueField 2 SmartNIC

Prerequisites
-------------

- Mellanox OFED version: **5.0**
  see :doc:`../../nics/mlx5` guide for more Mellanox OFED details.
- Enable the RegEx caps using system call from the BlueField 2.
  Contact Mellanox support for detail explanation.

Compilation options
~~~~~~~~~~~~~~~~~~~

These options can be modified in the ``.config`` file.

- ``CONFIG_RTE_LIBRTE_MLX5_REGEX_PMD`` (default **n**)

  Toggle compilation of librte_pmd_mlx5 itself.

- ``CONFIG_RTE_IBVERBS_LINK_DLOPEN`` (default **n**)

  Build PMD with additional code to make it loadable without hard
  dependencies on **libibverbs** nor **libmlx5**, which may not be installed
  on the target system.

  In this mode, their presence is still required for it to run properly,
  however their absence won't prevent a DPDK application from starting (with
  ``CONFIG_RTE_BUILD_SHARED_LIB`` disabled) and they won't show up as
  missing with ``ldd(1)``.

  It works by moving these dependencies to a purpose-built rdma-core "glue"
  plug-in which must either be installed in a directory whose name is based
  on ``CONFIG_RTE_EAL_PMD_PATH`` suffixed with ``-glue`` if set, or in a
  standard location for the dynamic linker (e.g. ``/lib``) if left to the
  default empty string (``""``).

  This option has no performance impact.

- ``CONFIG_RTE_IBVERBS_LINK_STATIC`` (default **n**)

  Embed static flavor of the dependencies **libibverbs** and **libmlx5**
  in the PMD shared library or the executable static binary.


Run-time configuration
~~~~~~~~~~~~~~~~~~~~~~

- **ethtool** operations on related kernel interfaces also affect the PMD.
