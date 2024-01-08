MODULENAME = xilinx_xvc_pci_driver
NO_MMCONFIG = 0
DRIVER_LOCATION = /lib/modules/$(shell uname -r)/kernel/drivers/pci/pcie/xilinx/
TEST = driver_test/verify_xil_xvc_driver

obj-m += $(MODULENAME).o
EXTRA_CFLAGS := -DLOG_PREFIX=\"$(MODULENAME):\ \"

ifeq ($(NO_MMCONFIG),1)
  $(MODULENAME)-objs := xvc_pcie_driver_base.o xvc_pcie_driver.o
else
  $(MODULENAME)-objs := mmconfig-shared.o mmconfig_$(BITS).o xvc_pcie_driver_base.o xvc_pcie_driver.o
  EXTRA_CFLAGS += -DINCLUDE_CUSTOM_MMCONFIG
endif

install: module
	mkdir -p $(DRIVER_LOCATION)
	cp -f $(MODULENAME).ko $(DRIVER_LOCATION)

module:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules

test:
	$(CC) -I. -o $(TEST) $(TEST).c

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean

uninstall:
	rm -rf $(DRIVER_LOCATION)
	rm -f $(TEST)
