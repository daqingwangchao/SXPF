/**
 * Adaption layer of the Xilinx XVC PCIe driver for the SX proFRAME, based
 * on the original file xvc_pcie_driver_base.c, which this file replaces.
 *
 * original code and this adaption are licensed unter the GPL v2
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
#include "sxpf_xvc.h"
#include "sxpf.h"
#include "sxpf_module.h"

#ifdef ENABLE_XVC

#ifdef INCLUDE_CUSTOM_MMCONFIG
#include "mmconfig.h"
#endif


void xil_probe_xvc(sxpf_card_t *card)
{
    int status = -EINVAL;
    int u;

    for (u = 0; u < USER_CONFIG_COUNT; ++u)
    {
        const struct pcie_user_config *ucfg = user_configs + u;

        if (ucfg->config_space != CONFIG)   // only support CONFIG access, atm
            continue;

        status = xil_xvc_get_offset(card->pci_device,
                                    ucfg->config_info.config_vsec_id,
                                    ucfg->config_info.config_vsec_rev,
                                    &card->xvc_algo.offset.cfg);

        if (status == 0)
        {
            // config space access to XVC found!
            card->xvc_algo.type = XVC_ALGO_CFG;
            card->xvc_config = *ucfg;
            break;
        }
    }

    if (status != 0)
    {
        CARD_PRINTF(SXPF_INFO, "Card has no XVC built in.");
    }
}


long ioctl_xvc(sxpf_card_t* card, unsigned int cmd, unsigned long arg)
{
    int status = -ENOIOCTLCMD;
    unsigned long pci_config_lock_flags = 0;

    if (card->xvc_algo.type == XVC_ALGO_NULL)
        return status;

    spin_lock_irqsave(&card->xvc_lock, pci_config_lock_flags);

    switch (cmd)
    {
    case XDMA_IOCXVC:
        status = xil_xvc_ioctl(card->pci_device, &card->xvc_algo,
                               (void __user *)arg);
        break;

    case XDMA_RDXVC_PROPS:
        status = xil_xvc_readprops(card->pci_device, &card->xvc_algo,
                                   &card->xvc_config, (void __user *)arg);
        break;

    default:
        // -ENOIOCTLCMD;
        break;
    }

    mb();
    spin_unlock_irqrestore(&card->xvc_lock, pci_config_lock_flags);

    return status;
}


int xil_xvc_init(void)
{
	printk(KERN_INFO LOG_PREFIX "Starting...\n");

#ifdef INCLUDE_CUSTOM_MMCONFIG
	// Comment this out on 32 bit systems
	return pci_mmcfg_driver_init();
#endif

	return 0;
}

void xil_xvc_exit(void)
{
#ifdef INCLUDE_CUSTOM_MMCONFIG
	// Comment this out on 32 bit systems
	pci_mmcfg_driver_exit();
#endif
}

#endif
