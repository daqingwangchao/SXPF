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

#ifndef _XVC_PCIE_USER_CONFIG_H
#define _XVC_PCIE_USER_CONFIG_H


enum pcie_config_space {
    AUTO,
    CONFIG,
    BAR
};

struct config_space_info {
    unsigned int config_vsec_id;
    unsigned int config_vsec_rev;
};

struct bar_space_info {
    unsigned int bar_index;
    unsigned int bar_offset;
};

struct pcie_user_config {
    const char *name;
    enum pcie_config_space config_space;
    struct config_space_info config_info;
    struct bar_space_info bar_info;
};

/*
 *  Modify the macros and structure below with PCIe customizations for the driver, if desired.
 *    PCIE_VENDOR_ID  - hex value for PCIe device vendor
 *    PCIE_DEVICE_ID  - hex value for PCIe device ID
 *    config_space    - AUTO (to let the driver search first for CONFIG followed by BAR space)
 *                    - CONFIG (to explicitly use PCIe configuration space)
 *                    - BAR (to explicitly use PCIe BAR space)
 *    config_vsec_id  - PCIe CONFIG space hex value for vendor-specific extended capability ID
 *    config_vsec_rev - PCIe CONFIG space hex value for vendor-specific extended capability revision number
 *    bar_index       - PCIe BAR index specifies which PCIe BAR should be used to access the
 *                      XVC pcie peripheral for the AXI-XVC peripheral.
 *    bar_offset      - PCIe BAR space offset specifies the offset within the PCIe BAR that
 *                      should be used to access the AXI-XVC peripheral.
 */

#define PCIE_DRIVER_NAME "xilinx_xvc_pci_driver"

#define PCIE_VENDOR_ID  0x1556
#define PCIE_DEVICE_ID  0x5555

static const struct pcie_user_config user_configs[] = {
    /////////////////////////////////////////////////////////
    //  The single debug tree entry below with an empty 
    //  name modifier will create a character file called:
    //
    //   /dev/xil_xvc/cfg_ioc0
    //
    /////////////////////////////////////////////////////////
    {
        .name = "",
        .config_space   = CONFIG,
        .config_info = {
            .config_vsec_id  = 0x0008,
            .config_vsec_rev = 0x0,
        },
        .bar_info = {
            .bar_index      = 0x0,
            .bar_offset     = 0xC0000,
        },
    },
    /////////////////////////////////////////////////////////
    //  For two debug trees in the same driver, you can 
    //  uncomment and modify the entries below.  In this 
    //  case, the name modifers “tree0” and “tree1” are 
    //  appended to the character files as follows:
    // 
    //    /dev/xil_xvc/cfg_ioc0_tree0
    //    /dev/xil_xvc/cfg_ioc0_tree1
    // 
     /////////////////////////////////////////////////////////
     //
     // {
     //   .name = "tree0",
     //   .config_space   = BAR,
     //   .bar_info = {
     //         .bar_index      = 0x0,
     //         .bar_offset     = 0x00000,
     //   },
     // },
     // {
     //   .name = "tree1",
     //   .config_space   = BAR,
     //   .bar_info = {
     //         .bar_index      = 0x0,
     //         .bar_offset     = 0x10000,
     //   },
     // },
};


#define USER_CONFIG_COUNT (sizeof(user_configs) / sizeof(*user_configs))

#endif /* _XVC_PCIE_USER_CONFIG_H */
