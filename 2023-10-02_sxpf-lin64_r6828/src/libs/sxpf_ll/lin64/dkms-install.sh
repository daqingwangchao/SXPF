#!/bin/bash
#
# defaults (for installing on local system):
#   ROOTFS=
#   KERNEL_VER=

SCRIPT_DIR="$(realpath `dirname $0`)"

cd "${SCRIPT_DIR}"

DRIVER_VERSION=$(cut -d' ' -f3 svnversion.h)
[ -z "${ROOTFS}" ] && KERNEL_VER=$(uname -r)
TGTDIR=${ROOTFS}/usr/src/sxpf-${DRIVER_VERSION}

INSTALLED_REVISIONS=$(ls -1 /var/lib/dkms/sxpf 2>/dev/null | grep '^[0-9]\+$')

for rev in ${INSTALLED_REVISIONS} ; do
    dkms remove sxpf/${rev} --all

    # clean previous installation
    rm -rf /usr/src/sxpf-${rev}
done
rm -rf ${TGTDIR}

echo "Setting up DKMS module source in ${TGTDIR}"

mkdir -p ${TGTDIR}/driver
mkdir -p ${TGTDIR}/driver/xvc-driver

# copy dkms.conf and replace version placeholder with actual revision number
sed -e "s/rrrrr/${DRIVER_VERSION}/" dkms.conf > ${TGTDIR}/dkms.conf

# copy source files
install --mode 0644 -p -t ${TGTDIR}  *.h ../sxpf*.h ../../support/{end,}packed.h
install --mode 0644 -p -t ${TGTDIR}/driver  driver/{Makefile,Kbuild}
install --mode 0644 -p -t ${TGTDIR}/driver  driver/sxpf_*.[ch] driver/proc_dbg.c
install --mode 0644 -p -t ${TGTDIR}/driver/xvc-driver  driver/xvc-driver/*.[ch]

# insert SXPF_NV_P2P enable option into installed Makefile to make it permanent
sed -i "s/^SXPF_NV_P2P ?=$/SXPF_NV_P2P ?= ${SXPF_NV_P2P}/" ${TGTDIR}/driver/Makefile

# add udev rule to make /dev/sxpfX accessible to everyone
[ -n "${ROOTFS}" ] && mkdir -p "${ROOTFS}/etc/udev/rules.d"
install --mode 0644 -t ${ROOTFS}/etc/udev/rules.d  driver/80-sxpf-access-for-all.rules

if [ -z "${ROOTFS}" ] ; then
    # register kernel module with DKMS if targetting local machine
    dkms install -m sxpf -v ${DRIVER_VERSION} -k ${KERNEL_VER}
else
    # create registration script to register kernel module with DKMS within chroot
    SCRIPT="${ROOTFS}/inst/dkms-install-sxpf.sh"
    echo "dkms install -m sxpf -v ${DRIVER_VERSION} -k ${KERNEL_VER}" > "${SCRIPT}"
fi
