#!/bin/bash

module="sxpf"
device="sxpf"
mode="666"

this_dir=$(dirname ${0})

# Group: since distributions do it differently, look for users or use staff
if grep -q '^users:' /etc/group;
then
	group="users"
else
	if grep -q '^staff:' /etc/group;
	then
		group="staff"
	else
		group="wheel"
	fi
fi

# unload previous module instance
if grep -q $module /proc/modules; then
    rmmod $module
fi

# remove old device nodes (in case NVIDIA P2P interface is enabled)
rm -f /dev/sxpf*

# invoke insmod with all arguments we got and use a pathname, as insmod doesn't
# look in . by default
/sbin/insmod ${this_dir}/$module.ko $* || exit 1

# retrieve major number
major=$(awk "\$2==\"$module\" {print \$1}" /proc/devices)

# make device nodes (in case NVIDIA P2P interface is enabled)
num_cards=$(lspci -d:5555 | wc -l)

for i in $(seq 0 $((${num_cards}-1)) ) ; do
    [ -c /dev/sxpf$i ] || mknod -m 666 /dev/sxpf$i c ${major} $i
done

# give device nodes permissions and a group ID so they can be accessed by users
chgrp $group /dev/${device}[0-9]
chmod $mode  /dev/${device}[0-9]


