SXPFLL_MK_DIR:=$(dir $(lastword $(MAKEFILE_LIST)))

# set VERMAJOR and VERMINOR based on content of sxpf_version.h
SX_VER_DEFS:=$(shell sed -n "s/\#define SXPF_SW_\(M..OR\) *\(.*\)/VER\1:=\2/p" \
			$(BASE_DIR)/sxpf_version.h)

$(foreach def,$(SX_VER_DEFS),$(eval $(def)))

# In the development environment this conditionally updates svnversion.h.
# In released software sources this does nothing, because this header is part of
# the source code archive.
-include $(SXPFLL_MK_DIR)/lin64/provide-svnversion.mk

dkms_install:
	@cd $(SXPFLL_MK_DIR)/lin64 && ./dkms-install.sh

INC_DIRS+=lin64
SOURCE_DIRS+=lin64
LIBS+=support

$(OUTDIR)libs/sxpf_ll/lin64/os_dep.o: libs/sxpf_ll/lin64/svnversion.h

# vim: ft=make noet ts=8 sw=8
