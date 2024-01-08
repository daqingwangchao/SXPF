#https://medium.com/@navaneethrvce/debugging-your-linux-kernel-module-21bf8a8728ba
BINARY     := SXPFv_module
KERNEL      := /lib/modules/$(shell uname -r)/build
ARCH        := x86

C_FLAGS     := -Wall 
KMOD_DIR    := $(shell pwd)
TARGET_PATH := /lib/modules/$(shell uname -r)/kernel/drivers/char

OBJECTS := sxpf-virtualization.o #

mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
current_dir := $(shell pwd)

C_FLAGS += -I/home/zhy3rng/kernel_devolop/hil-sxpf-virtualization/extern/2023-10-02_sxpf-lin64_r6828/src/libs/sxpf_ll \
		   -I/home/zhy3rng/kernel_devolop/hil-sxpf-virtualization/extern/2023-10-02_sxpf-lin64_r6828/src/libs/sxpf_ll/lin64 \
		   -I/home/zhy3rng/kernel_devolop/hil-sxpf-virtualization/extern/2023-10-02_sxpf-lin64_r6828/src/libs/sxpf_ll/lin64/driver \
		   -I/home/zhy3rng/kernel_devolop/hil-sxpf-virtualization/extern/2023-10-02_sxpf-lin64_r6828/src/libs/sxpf_ll/lin64/driver/xvc-driver \
		   -I/home/zhy3rng/kernel_devolop/hil-sxpf-virtualization/extern/2023-10-02_sxpf-lin64_r6828/src/libs/support

#C_FLAGS += -I$(current_dir)/../sxpf_ll \
		   -I$(current_dir)/../sxpf_ll/lin64 \
		   -I$(current_dir)/../sxpf_ll/lin64/driver \
		   -I$(current_dir)/../sxpf_ll/lin64/driver/xvc-driver \
		   -I$(current_dir)/../support


C_FLAGS += -g -std=gnu99

ccflags-y += $(C_FLAGS)

obj-m += $(BINARY).o

$(BINARY)-y := $(OBJECTS)

$(BINARY).ko:
	make -C $(KERNEL) M=$(KMOD_DIR) modules

install:
	cp $(BINARY).ko $(TARGET_PATH)
	depmod -a

uninstall:
	rm $(TARGET_PATH)/$(BINARY).ko
	depmod -a
debug:
	echo -I$(current_dir)/../sxpf_ll

clean:
	make -C $(KERNEL) M=$(KMOD_DIR) clean