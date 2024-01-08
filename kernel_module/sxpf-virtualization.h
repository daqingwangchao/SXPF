#ifndef MYCHARDEV_DRIVER_H_
#define MYCHARDEV_DRIVER_H_

#define KERNEL
#define USE_GPL_SYMBOLS 1

#include <asm/errno.h>
#include <asm/uaccess.h>
#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>/* for sprintf() */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/poll.h>
#include <linux/printk.h>
#include <linux/slab.h> /*kmalloc*/
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/uaccess.h> /* copy_to/copy_from */
#include <linux/version.h>

#include "sxpf.h"
#include "sxpf_module.h"
#include "driver_acc.h"

// forward declarations


// #include "driver_acc.h"
// #include "sxpf.h"
// #include "sxpf_module.h"
// #include "sxpf_regs.h"

static int GetNumofChannel(int dev_index);
static int sxpfv_create(int dev_num_sum, int num_of_channels);
static int __init sxpfv_init(void);
static void __exit sxpfv_exit(void);
static int sxpfv_open(struct inode *inode, struct file *file);
static int sxpfv_release(struct inode *inode, struct file *file);
static long sxpfv_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t sxpfv_read(struct file *file, char __user *buf, size_t count,
                          loff_t *offset);
// static ssize_t sxpfv_write(struct file *file, const char __user *buf, size_t
// count, loff_t *offset);
static int sxpfv_mmap(struct file *file, struct vm_area_struct *vma);
static unsigned int sxpfv_poll(struct file *file, poll_table *wait);
static int sxpfv_uevent(struct device *dev, struct kobj_uevent_env *env);

static const struct file_operations virtual_dev_fops = {
    .owner = THIS_MODULE,
    .open = sxpfv_open,
    .release = sxpfv_release,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36))
    .ioctl = sxpf_ioctl,
#else
    .compat_ioctl = sxpfv_ioctl,    // same endpoint in sxpf driver
    .unlocked_ioctl = sxpfv_ioctl,  // same endpoint in sxpf driver
#endif
    .mmap = sxpfv_mmap,
    .read = sxpfv_read,
    .poll = sxpfv_poll,
};

#endif /* MYCHARDEV_DRIVER_H_ */