/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019 Mellanox Technologies, Ltd
 */
#include <unistd.h>
#include <sys/mman.h>

#include <rte_malloc.h>
#include <rte_log.h>
#include <rte_errno.h>
#include <rte_bus_pci.h>
#include <rte_pci.h>
#include <rte_regexdev_driver.h>

#include <infiniband/mlx5dv.h>
#include <mlx5_glue.h>
#include <mlx5_common.h>
#include <mlx5_prm.h>

#include "mlx5.h"
#include "mlx5_regex.h"
#include "mlx5_regex_utils.h"
#include "rxp-csrs.h"

static TAILQ_HEAD(mlx5_regex_privs, mlx5_regex_priv) priv_list =
					      TAILQ_HEAD_INITIALIZER(priv_list);
static pthread_mutex_t priv_list_lock = PTHREAD_MUTEX_INITIALIZER;
int mlx5_regex_logtype;

struct mlx5_regex_db {
	/* TODO: refer to struct rxp_database of Yuval code */
	void *raw_mem;
	struct mlx5dv_devx_umem *umem;
	/* TODO: refer to Yuval's struct mlx5_database_ctx */
	struct mlx5_database_ctx umem_ctx;
};

#define MLX5_REGEX_MAX_RXP_ENGINES 2
#define MLX5_REGEX_DB_MEM_SIZE (1u << 27u)

/* TODO: move t mlx5/common and reuse with vpda */
static struct ibv_device *
mlx5_regex_get_ib_device_match(const struct rte_pci_addr *addr)
{
	int n;
	struct ibv_device **ibv_list = mlx5_glue->get_device_list(&n);
	struct ibv_device *ibv_match = NULL;

	if (!ibv_list) {
		rte_errno = ENOSYS;
		return NULL;
	}
	while (n-- > 0) {
		struct rte_pci_addr pci_addr;

		DRV_LOG(DEBUG, "Checking device \"%s\"..", ibv_list[n]->name);
		if (mlx5_dev_to_pci_addr(ibv_list[n]->ibdev_path, &pci_addr))
			continue;
		if (memcmp(addr, &pci_addr, sizeof(pci_addr)))
			continue;
		ibv_match = ibv_list[n];
		break;
	}
	if (!ibv_match)
		rte_errno = ENOENT;
	return ibv_match;
}

static int mlx5_regex_engines_status(struct mlx5_regex_priv *priv, int num_engines)
{
	uint32_t fpga_ident = 0;
	int err;
	int i;

	for (i = 0; i < num_engines; i++) {
		err = mlx5_regex_register_read(priv->ctx, i,
					       RXP_CSR_IDENTIFIER, &fpga_ident);
		fpga_ident = (fpga_ident & (0x0000FFFF));
		if (err || fpga_ident != 0x5254) {
			DRV_LOG(ERR, "Failed setup RXP %d err %d database memory 0x%x",
				i, err, fpga_ident);
			if (!err)
				err = EINVAL;
			return err;
		}
	}
	return 0;
}

static int mlx5_regex_setup_dev(struct mlx5_regex_priv *priv, int num_engines)
{
	struct mlx5dv_pd mlx5_pd = { 0 };
	struct mlx5dv_obj dv_obj;
	int ret;

	ret = mlx5_regex_engines_status(priv, num_engines);
	if (ret)
		return ret;

	ret = mlx5dv_devx_query_eqn(priv->ctx, 0, &priv->eqn);
	if (ret || !priv->eqn) {
		if (!ret)
			ret = EINVAL;
		return ret;
	}

	priv->pd = ibv_alloc_pd(priv->ctx);
	if (!priv->pd)
		return ENOMEM;
	dv_obj.pd.in = priv->pd;
	dv_obj.pd.out = &mlx5_pd;
	mlx5dv_init_obj(&dv_obj, MLX5DV_OBJ_PD);
	priv->pdn = mlx5_pd.pdn;

	priv->uar = mlx5dv_devx_alloc_uar(priv->ctx, 0);
	if (!priv->uar) {
		ibv_dealloc_pd(priv->pd);
		return ENOMEM;
	}
	return 0;
}

static void mlx5_regex_cleanup_dev(struct mlx5_regex_priv *priv)
{
	mlx5dv_devx_free_uar(priv->uar);
	ibv_dealloc_pd(priv->pd);
}

static int
mlx5_regex_setup_db_one(struct mlx5_regex_priv *priv,
			struct mlx5_regex_db *db)
{
	void *raw_mem;
	int err;

	raw_mem = mmap(NULL, MLX5_REGEX_DB_MEM_SIZE, PROT_READ | PROT_WRITE,
		       MAP_SHARED | MAP_ANONYMOUS | MAP_POPULATE | MAP_HUGETLB,
		       -1, 0);
	if (!raw_mem)
		return ENOMEM;

	db->raw_mem = raw_mem;
	/* Register the memory with Mellanox */
	db->umem = mlx5dv_devx_umem_reg(priv->ctx, raw_mem,
                                        MLX5_REGEX_DB_MEM_SIZE, 7);
	if (!db->umem) {
		err = ENOMEM;
		goto umem_err;
	}

	return 0;

umem_err:
	munmap(db->raw_mem, MLX5_REGEX_DB_MEM_SIZE);
	return err;
}

static void mlx5_regex_cleanup_db_one(struct mlx5_regex_db *db)
{
	if (db->umem)
		mlx5dv_devx_umem_dereg(db->umem);
	if (db->raw_mem)
		munmap(db->raw_mem, MLX5_REGEX_DB_MEM_SIZE);
}

static int mlx5_regex_setup_db(struct mlx5_regex_priv *priv, int num_engines)
{
	int ret;
	int i;

	priv->db_desc = calloc(num_engines, sizeof(*priv->db_desc));
	if (!priv->db_desc)
		return ENOMEM;

	for (i = 0; i < num_engines; i++) {
		ret = mlx5_regex_setup_db_one(priv, &priv->db_desc[i]);
		if (ret)
			goto cleanup;
	}
	priv->num_db_desc = num_engines;
	return 0;

cleanup:
	for (; i >= 0; i--)
		mlx5_regex_cleanup_db_one(&priv->db_desc[i]);
	free(priv->db_desc);
	return ret;
}

static void mlx5_regex_cleanup_db(struct mlx5_regex_priv *priv)
{
	int i;

	for (i = priv->num_db_desc - 1; i >= 0; i--)
		mlx5_regex_cleanup_db_one(&priv->db_desc[i]);

	free(priv->db_desc);
}

static int mlx5_regex_dev_info_get(struct rte_regex_dev *dev __rte_unused,
				   struct rte_regex_dev_info *info)
{
	info->max_queue_pairs = 32;
	info->max_scatter_gather = 1;
	return 0;
}

static const struct rte_regex_dev_ops dev_ops = {
	.dev_info_get = mlx5_regex_dev_info_get,
};

/**
 * DPDK callback to register a PCI device.
 *
 * This function spawns regex device out of a given PCI device.
 *
 * @param[in] pci_drv
 *   PCI driver structure (mlx5_vpda_driver).
 * @param[in] pci_dev
 *   PCI device information.
 *
 * @return
 *   0 on success, a negative errno value otherwise and rte_errno is set.
 */
static int
mlx5_regex_pci_probe(struct rte_pci_driver *pci_drv __rte_unused,
		     struct rte_pci_device *pci_dev)
{
	struct ibv_device *ibv = mlx5_regex_get_ib_device_match(&pci_dev->addr);
	struct mlx5_regex_priv *priv = NULL;
	struct mlx5_hca_attr attr;
	struct ibv_context *ctx;
	int ret;

	if (!ibv) {
		DRV_LOG(ERR, "No matching IB device for PCI slot "
			PCI_PRI_FMT ".", pci_dev->addr.domain,
			pci_dev->addr.bus, pci_dev->addr.devid,
			pci_dev->addr.function);
		rte_errno = ENOENT;
		return -rte_errno;
	} else {
		DRV_LOG(INFO, "PCI information matches for device \"%s\".",
			ibv->name);
	}
	ctx = mlx5_glue->dv_open_device(ibv);
	if (!ctx) {
		DRV_LOG(ERR, "Failed to open IB device \"%s\".", ibv->name);
		rte_errno = ENODEV;
		return -rte_errno;
	}

	ret = mlx5_devx_cmd_query_hca_attr(ctx, &attr);
	if (ret) {
		DRV_LOG(ERR, "Unable to read HCA capabilities.");
		rte_errno = ENOTSUP;
		return -rte_errno;
	} else {
		if (!attr.regex || attr.regexp_num_of_engines == 0) {
			DRV_LOG(ERR, "Not enough capabilities to support regex,"
				" maybe old FW/OFED version?");
			rte_errno = ENOTSUP;
			return -rte_errno;
		}
	}
	priv = rte_zmalloc("mlx5 regex device private", sizeof(*priv),
			   RTE_CACHE_LINE_SIZE);
	if (!priv) {
		DRV_LOG(ERR, "Failed to allocate private memory.");
		rte_errno = ENOMEM;
		return -rte_errno;
	}

	priv->ctx = ctx;
	priv->pci_dev = pci_dev;
	sprintf(&priv->regex_dev.dev_name[0], "poc");
	priv->regex_dev.dev_ops = &dev_ops;

	ret = mlx5_regex_setup_dev(priv, attr.regexp_num_of_engines);
	if (ret) {
		rte_errno = ret;
		goto dev_error;
	}

	ret = mlx5_regex_setup_db(priv, attr.regexp_num_of_engines);
	if (ret) {
		DRV_LOG(ERR, "Failed to setup database memory.");
		rte_errno = ret;
		goto db_error;
	}
	sprintf(&priv->regex_dev.dev_name[0], "poc");

	pthread_mutex_lock(&priv_list_lock);
	TAILQ_INSERT_TAIL(&priv_list, priv, next);
	pthread_mutex_unlock(&priv_list_lock);

	ret = rte_regex_dev_register(&priv->regex_dev);
	if (ret < 0) {
		DRV_LOG(ERR, "Failed to register regex device.");
		rte_errno = ret;
		goto error;
	}
	return 0;

error:
	pthread_mutex_lock(&priv_list_lock);
	TAILQ_REMOVE(&priv_list, priv, next);
	pthread_mutex_unlock(&priv_list_lock);
	mlx5_regex_cleanup_db(priv);
db_error:
	mlx5_regex_cleanup_dev(priv);
dev_error:
	rte_free(priv);
	return rte_errno;
}

static int mlx5_regex_pci_remove(struct rte_pci_device *pci_dev)
{
	struct mlx5_regex_priv *priv = NULL;
	struct rte_regex_dev *regex_dev;
	bool found = false;

	TAILQ_FOREACH(priv, &priv_list, next) {
		if (priv->pci_dev == pci_dev) {
			found = true;
			break;
		}
	}
	if (!found)
		return 0;

	regex_dev = &priv->regex_dev;
	rte_regex_dev_unregister(regex_dev);
	mlx5_regex_cleanup_db(priv);
	mlx5_regex_cleanup_dev(priv);

	pthread_mutex_lock(&priv_list_lock);
	TAILQ_REMOVE(&priv_list, priv, next);
	pthread_mutex_unlock(&priv_list_lock);
	rte_free(priv);
	return 0;
}

static const struct rte_pci_id mlx5_regex_pci_id_map[] = {
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX4)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX4VF)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX4LX)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX4LXVF)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX5)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX5VF)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX5BF)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
			       PCI_DEVICE_ID_MELLANOX_CONNECTX5BFVF)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
				PCI_DEVICE_ID_MELLANOX_CONNECTX6)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
				PCI_DEVICE_ID_MELLANOX_CONNECTX6VF)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
				PCI_DEVICE_ID_MELLANOX_CONNECTX6DX)
	},
	{
		RTE_PCI_DEVICE(PCI_VENDOR_ID_MELLANOX,
				PCI_DEVICE_ID_MELLANOX_CONNECTX6DXVF)
	},
	{
		.vendor_id = 0
	}
};

static struct rte_pci_driver mlx5_regex_driver = {
	.driver = {
		.name = "mlx5_regex",
	},
	.id_table = mlx5_regex_pci_id_map,
	.probe = mlx5_regex_pci_probe,
	.remove = mlx5_regex_pci_remove,
};

/**
 * Driver initialization routine.
 */
RTE_INIT(rte_mlx5_regex_init)
{
	/* Initialize common log type. */
	mlx5_regex_logtype = rte_log_register("pmd.regex.mlx5");
	if (mlx5_regex_logtype >= 0)
		rte_log_set_level(mlx5_regex_logtype, RTE_LOG_NOTICE);
	if (mlx5_glue)
		rte_pci_register(&mlx5_regex_driver);
}

RTE_PMD_EXPORT_NAME(regex_mlx5, __COUNTER__);
RTE_PMD_REGISTER_PCI_TABLE(regex_mlx5, mlx5_regex_pci_id_map);
RTE_PMD_REGISTER_KMOD_DEP(regex_mlx5, "* ib_uverbs & mlx5_core & mlx5_ib");
