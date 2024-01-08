Linux Driver for Xilinx XVC Over PCIe
=====================================

August 25, 2017

Contents
========

- Considerations Before Building
- Building and Installation
- Initial Testing and Verification
- Unloading and Uninstalling the Driver

Considerations Before Building
==============================

If you would like to customize this driver for your specific PCIe application, 
please take a look at xvc_pcie_user_config.h.  This header file provides 
instructions for how to modify driver variables to target specific use cases.
These variables are then included in the driver at compile time.

Many of the commands involved in the build and installation process will 
require root privileges.  It is recommended to perform the build and 
installation with full root privileges as the root user as opposed to using the
"sudo" command.

Any drivers that might conflict with this driver should be uninstalled or
removed from the system before installing this driver.

Building and Installation
=========================

To build this driver:

1. Modify the variables within xvc_pcie_user_config.h to match your hardware 
   design and IP settings. Consider modifying the 
   following variables.
       PCIE_VENDOR_ID:  PCIe Vendor ID defined in the PCIe IP customization.
       PCIE_DEVICE_ID:  PCIe Device ID defined in the PCIe IP customization.
       config_vsec_id:  XVC-VSEC ID defined in the Debug Bridge IP for the
                        "PCIe-to-BSCAN" mode.
       config_vsec_rev: XVC-VSEC ID defined in the Debug Bridge IP for the
                        "PCIe-to-BSCAN" mode.
       bar_index:       PCIe-BAR that should be used to access the Debug Bridge
                        IP for "AXI-to-BSCAN" mode. This BAR index is specified
                        as a combination of PCIe IP customization and the 
                        addressable AXI peripherals in your system design.
       bar_offset:      PCIe-BAR Offset that should be used to access the Debug
                        Bridge IP for "AXI-to-BSCAN" mode. This BAR offset is
                        specified as a combination of PCIe IP customization and
                        the addressable AXI peripherals in your system design. 

2. Move the source files to the directory of your choice.  For example, use 
   /home/username/xil_xvc or /usr/local/src/xil_xvc.

3. Change to the directory containing the driver source files.

      # cd /driver_vX.X/

4. Compile the driver module:

      # make install

   The kernel module object file will be installed as:

      /lib/modules/[KERNEL_VERSION]/kernel/drivers/pci/pcie/xilinx/xil_xvc_driver.ko

5. Run depmod to pick up newly installed kernel module:

      # depmod -a

6. Make sure no older version of the driver are loaded:

      # modprobe -r xilinx_xvc_pci_driver

7. Load the module:

      # modprobe xilinx_xvc_pci_driver

   You should at least see the message if you run the "dmesg" command:

      "kernel: xil_xvc_driver: Starting..."

   NOTE: You can also use insmod on the kernel object file to load the module:

      # insmod xil_xvc_driver.ko

   but this is not recommended unless necessary for compatibility with older 
   kernels.

8. The resulting character file, /dev/xil_xvc/cfg_ioc0, will be owned by user 
   root and group root, and it will have 660 permissions.  Make sure the 
   character file has the appropriate permissions to allow access to the 
   application interacting with it.

Initial Testing and Verification
================================

1. Build the simple test program for the driver:

      # make test

2. Run the test program:

      ./driver_test/verify_xil_xvc_driver

   You should see various successful tests of differing lengths, followed by 
   the message:

      "XVC PCIE Driver Verified Successfully!"

Unloading and Uninstalling the Driver
=====================================

1. Unload the kernel module:

      # modprobe -r xil_xvc_driver

   NOTE: You can also use rmmod to unload the kernel module:

      # rmmod xil_xvc_driver

   but this is not recommended unless necessary for compatibility with older 
   kernels.

2. From the directory containing the source files, make clean and uninstall:

      # make clean uninstall

   This will remove the compiled driver from the sources directory as well as 
   uninstall it from its location in /lib/modules/[KERNEL_VERSION].
