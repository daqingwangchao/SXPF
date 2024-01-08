/*
 * Xilinx XVC PCIe Driver
 * Copyright (C) 2017 Xilinx Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/pci.h>
#include <linux/mod_devicetable.h>

#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include "xvc_pcie_driver.h"
#include "xvc_pcie_user_config.h"

#ifdef INCLUDE_CUSTOM_MMCONFIG
#include "mmconfig.h"
#endif

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("David Gronlund <dgronlu@xilinx.com>, Elessar Taggart <elessar@xilinx.com>");
MODULE_DESCRIPTION("XVC Debug Register Access over PCIe");

struct xil_xvc_char {
	struct pci_dev *pci_dev;
	struct xvc_algo_t xvc_algo;
	struct pcie_user_config *user_config;
};

static struct class *xvc_dev_class;

static const size_t CFG_DEVICES_MAX = 10;
static struct xil_xvc_char *xil_xvc_devices = NULL;

#define CHAR_DEVICES_MAX  CFG_DEVICES_MAX * USER_CONFIG_COUNT

static int xvc_ioc_dev_region_status = -1;
static dev_t xvc_ioc_dev_region;
static struct cdev xvc_char_ioc_dev;

static dev_t get_device_number(dev_t region, int offset) {
	return MKDEV(MAJOR(region), MINOR(region) + offset);
}

static void xil_xvc_cleanup(void) {
	printk(KERN_INFO LOG_PREFIX "Cleaning up resources...\n");

	if (!IS_ERR(xvc_dev_class)) {
		class_unregister(xvc_dev_class);
		class_destroy(xvc_dev_class);
	}

	if (xvc_char_ioc_dev.owner != NULL) {
		cdev_del(&xvc_char_ioc_dev);
	}

	if (xvc_ioc_dev_region_status == 0) {
		unregister_chrdev_region(xvc_ioc_dev_region, CFG_DEVICES_MAX);
	}

	if (xil_xvc_devices) kfree(xil_xvc_devices);
}

static long setup_xvc_algo(struct xil_xvc_char *xvc_char, const struct pcie_user_config *user_config) {
	int status = -EINVAL;
	int bar_index = user_config->bar_info.bar_index;

	// initialize algo selection if not already selected
	if (!xvc_char->xvc_algo.type) {
		
		struct xvc_algo_t *algo = &xvc_char->xvc_algo;

		// try to find the cfg capability if CONFIG or AUTO was specified by user
		if (user_config->config_space == CONFIG || user_config->config_space == AUTO) {
			status = xil_xvc_get_offset(
				xvc_char->pci_dev, 
				user_config->config_info.config_vsec_id, 
				user_config->config_info.config_vsec_rev, 
				&algo->offset.cfg);
			if (status == 0) {
				// found the capability so use CFG space;
				algo->type = XVC_ALGO_CFG;
				return status;
			}
		}
		// try to use BAR space if selected, or fall through for AUTO
		if (user_config->config_space == BAR || user_config->config_space == AUTO) {
			resource_size_t bar_start;
			resource_size_t bar_len;
			resource_size_t map_len;

			if ((!pci_resource_flags(xvc_char->pci_dev, bar_index)) & IORESOURCE_MEM) {
				printk(KERN_ERR LOG_PREFIX "Incorrect BAR configuration\n");
				return -ENODEV;
			}

			bar_start = pci_resource_start(xvc_char->pci_dev, bar_index);
			bar_len = pci_resource_len(xvc_char->pci_dev, bar_index);
			map_len = bar_len;
				
			if (!bar_len) {
				printk(KERN_WARNING LOG_PREFIX "BAR #%d is not present.\n", bar_index);
			} else {
				// add user specified BAR offset to base address of mapping
				algo->offset.bar = pci_iomap(xvc_char->pci_dev, bar_index, map_len) + user_config->bar_info.bar_offset;
				if (!(algo->offset.bar)) {
					printk(KERN_ERR LOG_PREFIX "Could not map BAR %d\n", bar_index);
					return -ENODEV;
				} else {
					algo->type = XVC_ALGO_BAR;
					printk(KERN_INFO LOG_PREFIX "BAR%d at 0x%llx mapped at 0x%p, length=%llu(/%llu)\n", bar_index,
							(u64)bar_start, algo->offset.bar, (u64)map_len, (u64)bar_len);
					status = 0;
				}
			}
		}
	}

	return status;
}

long char_ctrl_ioctl(struct file *file_p, unsigned int cmd, unsigned long arg) {
	int char_index = iminor(file_p->f_path.dentry->d_inode) - MINOR(xvc_ioc_dev_region);
	int status = 0;
	unsigned long pci_config_lock_flags = 0;

	if (xil_xvc_devices[char_index].pci_dev == NULL) {
		printk(KERN_ERR LOG_PREFIX "Could not find char_index %d\n", char_index);
		return -EFAULT;
	}
	
	spin_lock_irqsave(&file_p->f_path.dentry->d_inode->i_lock, pci_config_lock_flags);

	switch (cmd)
	{
	case XDMA_IOCXVC:
		status = xil_xvc_ioctl(xil_xvc_devices[char_index].pci_dev, &xil_xvc_devices[char_index].xvc_algo, (void __user *)arg);
		break;
	case XDMA_RDXVC_PROPS:
		status = xil_xvc_readprops(xil_xvc_devices[char_index].pci_dev, &xil_xvc_devices[char_index].xvc_algo, xil_xvc_devices[char_index].user_config, (void __user *)arg);
		break;
	default:
		status = -ENOIOCTLCMD;
		break;
	}
	
	mb();
	spin_unlock_irqrestore(&file_p->f_path.dentry->d_inode->i_lock, pci_config_lock_flags);

	return status;
}

static struct file_operations xil_xvc_ioc_ops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = char_ctrl_ioctl
};


// -----------------------
// -----------------------
// PCI initialization code
// -----------------------
// -----------------------
int probe(struct pci_dev *pdev, const struct pci_device_id *id) {
	dev_t ioc_device_number;
	char ioc_device_name[25];
	struct device *xvc_ioc_device;

	int status;

	status = pci_enable_device(pdev);
	if (status == 0) {
		int d = 0;
		int u = 0;
		for (d = 0; d < CHAR_DEVICES_MAX; ++d) {
			if (xil_xvc_devices[d * USER_CONFIG_COUNT].pci_dev == NULL) {
				for (u = 0; u < USER_CONFIG_COUNT; ++u) {
					int i = d * USER_CONFIG_COUNT + u;
					const char *name;
					int index_used = 0;

					// Add device to table
					xil_xvc_devices[i].pci_dev = pdev;
					memset(&xil_xvc_devices[i].xvc_algo, 0, sizeof(struct xvc_algo_t));

					status = setup_xvc_algo(xil_xvc_devices + i, user_configs + u);
					if (status != 0) {
						printk(KERN_ERR LOG_PREFIX "Failed to setup xvc capability\n");
						xil_xvc_devices[i].pci_dev = NULL;
						return status;
					}
					xil_xvc_devices[i].user_config = (struct pcie_user_config *) user_configs + u;

					// Character driver file
					ioc_device_number = get_device_number(xvc_ioc_dev_region, i);

					// Make device character file name with right number
					name = user_configs[u].name;
					if (name && name[0]) {
						sprintf(ioc_device_name, "xil_xvc/cfg_ioc%d_%s", d, name);
					} else if (index_used) {
						sprintf(ioc_device_name, "xil_xvc/cfg_ioc%d-%d", d, index_used);
						++index_used;
					} else {
						sprintf(ioc_device_name, "xil_xvc/cfg_ioc%d", d);
						++index_used;
					}

					xvc_ioc_device = device_create(xvc_dev_class, NULL, ioc_device_number, NULL, ioc_device_name);
					if (IS_ERR(xvc_ioc_device)) {
						printk(KERN_WARNING LOG_PREFIX "Failed to create the device %d %s\n", i, ioc_device_name);
					} else {
						printk(KERN_INFO LOG_PREFIX "Created device %d %s, reg offset (0x%lX)\n", i, ioc_device_name,
								xil_xvc_devices[i].xvc_algo.offset.cfg);
					}
				}
				break;
			}
		}
	}
	return status;
}

void remove(struct pci_dev *pdev) {
	int d, u;
	for (d = 0; d < CFG_DEVICES_MAX; ++d) {
		if (pdev != NULL && xil_xvc_devices[d * USER_CONFIG_COUNT].pci_dev == pdev) {
			for (u = 0; u < USER_CONFIG_COUNT; ++u) {
				int i = d * USER_CONFIG_COUNT + u;
				if (xil_xvc_devices[i].xvc_algo.type == XVC_ALGO_BAR) {
					printk(KERN_INFO LOG_PREFIX "Unmapping BAR%llu.\n", (u64) xil_xvc_devices[i].xvc_algo.offset.bar);
					pci_iounmap(pdev, xil_xvc_devices[i].xvc_algo.offset.bar);
				}

				xil_xvc_devices[i].pci_dev = NULL;

				device_destroy(xvc_dev_class, get_device_number(xvc_ioc_dev_region, i));

				printk(KERN_INFO LOG_PREFIX "Device removal found at %d\n", i);
			}
			break;
		}
	}
}

static struct pci_device_id xilinx_ids[] = {
	{PCI_DEVICE(PCIE_VENDOR_ID, PCIE_DEVICE_ID)},
	{0, },
};

static struct pci_driver xil_xvc_pci_driver = {
	.name = PCIE_DRIVER_NAME,
	.id_table = xilinx_ids,
	.probe = probe,
	.remove = remove,
};


// --------------------------
// --------------------------
// Driver initialization code
// --------------------------
// --------------------------

static int __init xil_xvc_init(void)
{
	printk(KERN_INFO LOG_PREFIX "Starting...\n");

	if (!xil_xvc_devices) {
		xil_xvc_devices = (struct xil_xvc_char *) kmalloc(sizeof(struct xil_xvc_char) * CHAR_DEVICES_MAX, GFP_KERNEL);
	}

	// Register the character packet device major and minor numbers
	xvc_ioc_dev_region_status = alloc_chrdev_region(&xvc_ioc_dev_region, 0, 
			CHAR_DEVICES_MAX, "xilinx_xvc_pci_ioc_driver_region");
	if (xvc_ioc_dev_region_status != 0) {
		xil_xvc_cleanup();
		return xvc_ioc_dev_region_status;
	}

	// Add the character device, no actual files yet
	cdev_init(&xvc_char_ioc_dev, &xil_xvc_ioc_ops);
	xvc_char_ioc_dev.owner = THIS_MODULE;
	cdev_add(&xvc_char_ioc_dev, xvc_ioc_dev_region, CHAR_DEVICES_MAX);

	// Register the character device class for the actual files
	xvc_dev_class = class_create(THIS_MODULE, "xil_xvc_class");
	if (IS_ERR(xvc_dev_class)) {
		xil_xvc_cleanup();
		return PTR_ERR(xvc_dev_class);
	}

	// init device list
	memset(xil_xvc_devices, 0, sizeof(*xil_xvc_devices));

#ifdef INCLUDE_CUSTOM_MMCONFIG
	// Comment this out on 32 bit systems
	pci_mmcfg_driver_init();
#endif

	return pci_register_driver(&xil_xvc_pci_driver);
}

static void __exit xil_xvc_exit(void)
{
	pci_unregister_driver(&xil_xvc_pci_driver);

#ifdef INCLUDE_CUSTOM_MMCONFIG
	// Comment this out on 32 bit systems
	pci_mmcfg_driver_exit();
#endif

	xil_xvc_cleanup();	
}

//module_init(xil_xvc_init);
//module_exit(xil_xvc_exit);
