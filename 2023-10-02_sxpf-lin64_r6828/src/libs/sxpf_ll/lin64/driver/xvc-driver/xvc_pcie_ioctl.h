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

#ifndef _XVC_PCIE_IOCTL_H
#define _XVC_PCIE_IOCTL_H

#include <linux/ioctl.h>

//#define XIL_XVC_MAGIC 0x58564344  // "XVCD"
#define XIL_XVC_MAGIC 'X'   /* must only use 8 bits */

struct xil_xvc_ioc {
	unsigned opcode;
	unsigned length;
	unsigned char *tms_buf;
	unsigned char *tdi_buf;
	unsigned char *tdo_buf;
};

struct xil_xvc_properties {
	unsigned int xvc_algo_type;
    unsigned int config_vsec_id;
    unsigned int config_vsec_rev;
    unsigned int bar_index;
    unsigned int bar_offset;    	
};

#define XDMA_IOCXVC		 _IOWR(XIL_XVC_MAGIC, 1, struct xil_xvc_ioc)
#define XDMA_RDXVC_PROPS _IOR(XIL_XVC_MAGIC, 2, struct xil_xvc_properties)

#endif /* _XVC_PCIE_IOCTL_H */
