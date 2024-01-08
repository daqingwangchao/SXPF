# note: when cross-compiling, the variable KDIR must be set on the command line
CC	= $(CROSS_COMPILE)gcc
CXX	= $(CROSS_COMPILE)g++
AR	= $(CROSS_COMPILE)ar

KVER	?= $(shell uname -r)
KDIR	?= /lib/modules/$(KVER)
KSRC	?= $(KDIR)/build

# get architecture targeted by used C compiler
ARCH	?= $(shell $(CC) -v 2>&1 | \
	     awk '/^Target:/ { split($$2,tgt,"-"); print tgt[1] }')

USE_GPL := 1
EXTRA_CFLAGS = -DUSE_GPL_SYMBOLS=$(USE_GPL)

# the following line must not change, because it is matched verbatim and
# replaced during dkms installation
SXPF_NV_P2P ?=

ifeq ($(ARCH),x86_64)
# detect discrete NVIDIA GPU (NV_MOD is not empty): look for driver file with
# P2P symbols
NV_MOD ?= $(if $(SXPF_NV_P2P),$(wildcard $(KDIR)/updates/dkms/nvidia.ko))
NV_INC	?= $(if $(NV_MOD),$(wildcard /usr/src/nvidia-*/nvidia))
else
# Xavier AGX with GPL license
NV_MOD	?=
NV_INC	?= $(if $(SXPF_NV_P2P),$(wildcard \
	   	/usr/src/linux-headers-$(KVER)-ubuntu*/nvgpu/include/linux))
endif

# absolute path to this very Makefile (without relying on $(shell pwd))
THISFILE:= $(CURDIR)/$(lastword $(MAKEFILE_LIST))

PWD	:= $(dir $(THISFILE))


.PHONY: tags all clean mrproper check_kernel_headers install uninstall

# by default build only the kernel modules
all: kernel_modules

ifneq ($(NV_INC),)
EXTRA_CFLAGS += -DENABLE_NV_P2P -I$(NV_INC)

ifneq ($(NV_MOD),)
# NVIDIA module detected - switch to non-GPL license mode if it is not the
# GPL-licensed driver
USE_GPL := $(if $(shell modinfo $(NV_MOD) | grep "license.*GPL"),1,0)
EXTRA_CFLAGS += -DNV_BUILD_DGPU

NV_SYMVERS = $(PWD)nvidia.symvers

$(NV_SYMVERS): SHELL:=/bin/bash
$(NV_SYMVERS): $(NV_MOD)
	@declare -A SYMCRC ;\
	declare -a KCRCTAB ;\
	declare -i ofs=0 ;\
	while read crc ; do \
		KCRCTAB[$$(($$ofs))]="0x$$crc" ;\
		ofs=$$((ofs+4)) ;\
	done < <(readelf -x __kcrctab $(NV_MOD) | \
		 awk 'function r(s){ \
			return	substr(s,7,2) substr(s,5,2) \
				substr(s,3,2) substr(s,1,2) \
			} \
			/^  0x/{ printf("%s\n%s\n%s\n%s\n", \
					r($$2),r($$3),r($$4),r($$5)) }') ;\
	while read crc a sym ; do \
		[[ $$a == 'A' ]] \
		&& SYMCRC[$${sym/__crc_/}]="0x$$crc" \
		|| SYMCRC[$${sym/__crc_/}]="$${KCRCTAB[$$((0x$$crc))]}" ;\
	done < <(nm $(NV_MOD) | grep ' [Ar] __crc_') ;\
	nm -o $(NV_MOD) | grep ' T nvidia_p2p_' | while read line ; do \
		line1=$$(echo $$line|cut -f1 -d:) ;\
		line2=$$(echo $$line|cut -f2 -d:) ;\
		file=$$(echo $$line1| sed -e 's@.ko@@') ;\
		sym=$$(echo $$line2|cut -f3 -d" ") ;\
		crc=0x00000000 ;\
		[ -n "$${SYMCRC[$$sym]}" ] && crc="$${SYMCRC[$$sym]}" ;\
		printf "0x%08x\t$$sym\t$$file\tEXPORT_SYMBOL\t\n" $$crc ;\
	done > $(NV_SYMVERS)
endif
endif

dkms_install:
	@cd .. && ./dkms-install.sh

# install the kernel module
install: kernel_modules_install
	install --mode 0644 80-sxpf-access-for-all.rules /etc/udev/rules.d/
	depmod -a
	udevadm control --reload-rules
	udevadm trigger --action=add --subsystem-match=sxpf

# uninstall the kernel module
uninstall:
	rm -f $(KDIR)/extra/sxpf.ko /etc/udev/rules.d/80-sxpf-access-for-all.rules
	depmod -a

#.PHONY: doc
doc: Doxyfile ../../sxpf.h ../../sxpf.cpp sxpf_module.c sxpf_cmd_fifo.c sxpf_module.h
	doxygen

# In the development environment this conditionally updates svnversion.h.
# In released software sources this does nothing, because this header is part of
# the source code archive.
-include ../provide-svnversion.mk

# For goals starting with kernel_ we use the kernel's Makefile after stripping
# off the kernel_ prefix (except for kernel_tags - see below)
kernel_%: check_kernel_headers $(NV_SYMVERS)
	@$(MAKE) -C $(KSRC) M=$(PWD:/=) $(@:kernel_%=%) \
		KCFLAGS="$(EXTRA_CFLAGS)" KBUILD_EXTRA_SYMBOLS=$(NV_SYMVERS)


check_kernel_headers:
	@[ -d $(KSRC) ] || (echo Missing Kernel headers at '$(KSRC)'! && false)

clean: kernel_clean
	rm -rf tags doc/html doc/rtf $(NV_SYMVERS)

mrproper: clean
	rm -f kernel_tags

# default to Exuberant Ctags
MKTAGS     ?= ctags
TAGS_EXTRA ?= --extra=+q

pv-%:
	@echo $*=$(subst $@,TARGET,$($*))

tags: kernel_tags
	-cp $< $@
	$(MKTAGS) -a --c++-kinds=+p --fields=+iaS $(TAGS_EXTRA) -R --exclude=win64 \
		*.[ch] xvc-driver/*.[ch]

# We don't make the kernel's tags target, but create them on our own from the
# kernel headers, only.
kernel_tags:
	$(MKTAGS) -f $@ -R --c++-kinds=+p --fields=+iaS $(TAGS_EXTRA) $(KSRC)/include/

# vim: noet ts=8 sw=8
