#include "sxpf-virtualization.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Johannes Ruenz, Zhao Yuhang");
MODULE_DESCRIPTION("SXPF virtualization");
MODULE_VERSION("0.1");

#define DEV_NUM 4
#define DEFAULT_MAJOR_NUM 0

// Store the information for the current opened virtual devices (sxpfx_x)
int current_dev_ID =
    0;  // Store the device ID(Major + Minor) in the current running procedure
dev_t dev;  // The 12 most significant bits are the primary devnum, and the 20
            // least significant bits are the secondary devnum

// Global storage for device Major number
int major_number = DEFAULT_MAJOR_NUM;
int minor_number = 0;

int nr_dev =
    DEV_NUM;  // For initialization; later, the value of "nr_dev"(number of real
              // ohysical devices) can be changed
int total_channels = 0,
    channels_per_device[10];  // Global channel number over all devices

/**  Creating and managing module parameters that can be specified in a myriad
 * of convenient ways is trivial, allowing drivers to declare parameters that
 * the user can specify on either boot or module load and then have these
 * parameters exposed in your driver as global variables.
 *
 * Def: module_param(name, type, perm)
 *
 * @param name  Variable name which can be used to costumize.
 * @param type  Variable type for the name.
 * @param perm  Paramater argument specifies the permissions of the
 * corresponding file in sysfs. The permissions can be specified in the usual
 * octal format, for example 0644 (owner can read and write, group can read,
 * everyone else can read), or by ORing together the usual S_Ifoo defines, for
 * example S_IRUGO | S_IWUSR (everyone can read, user can also write). A value
 * of zero disables the sysfs entry altogether.
 */
module_param(major_number, int, S_IRUGO);  // linux/moduleparam.h
module_param(minor_number, int, S_IRUGO);
module_param(nr_dev, int, S_IRUGO);

struct VirtualDeviceData {
  struct cdev cdev;
};

// Sysfs class structure
static struct class *virtual_device_class = NULL;
// Array of virtual_device_structures
static struct VirtualDeviceData *virtual_device_structures;



/**  Find the corresponding avaliable channel number for every device
 *
 *   Def: get_num_of_channels(int dev_index)
 *
 * @param dev_index: Index of the currenct device within all detected real
 * physical device
 *
 * @return channel_count: Number of channel for each device
 */

static int get_num_of_channels(int dev_index) {
  int channel_count;
  struct file *f;
  int ret;
  char s[12];

  pr_info("SXPFx: open device\n");

  sprintf(s, "/dev/sxpf%d", dev_index);
  // snprintf(s, sizeof(s), "/dev/sxpf%d", dev_index);
  pr_info("Debug: open device: %s\n", s);
  f = filp_open(s, O_RDWR, 0);
  if (IS_ERR(f)) {
    pr_alert("SXPFx: cannot open file sxpf0\n");
    return (PTR_ERR(f));
  }
  pr_info("SXPFx: %x\n", f->f_inode->i_rdev);

  struct cdev *cdev = f->f_inode->i_cdev;
  sxpf_card_t *card = container_of(cdev, sxpf_card_t, cdev);
  sxpf_file_hdl_t *file_handle = container_of(&card, sxpf_file_hdl_t, card);

  pr_info("SXPFx_card-prop: %d\n", card->props.num_channels);
  pr_info("SXPFx_Number of available image DMA buffers: %d\n", card->props.num_buffers);
  pr_info("SXPFx_Number of available image DMA buffers that can be provided by the user: %d\n", card->props.num_user_buffers);
  pr_info("SXPFx_Number of I2C capture DMA buffers: %d\n", card->props.num_i2c_buffers);
  pr_info("SXPFx_card-numbers: %d\n", card->nr);
  ret = filp_close(f, NULL);
  if (ret) {
    pr_alert("SXPFx: cannot close sxpf0\n");
    return ret;
  }
  pr_info("sxpfx: device prob closed");

  channel_count = card->props.num_channels;

  return channel_count;
}
/**  Register the device number(Major number is all the same, minor number is
 * from 0 to N-1), create new devices, allocate the new memory space
 *
 *   For the created devices. For instance: two real phisical devices are
 * detected, i.e. sxpf0 and sxpf1 with the avaliable channels 4 and 2
 * respectively, nr_dev will be set to 2, one loop will call this function and
 * pass the device number to the dev_num_sum, at the same time the number of
 * avaliable channel number for each device is stored in the array
 * channels_per_device[j] (hier is aquivalent to channels_per_device[0]
 *   + channels_per_device[1], i.e. 4 + 2 = 6), summarize this array and then
 * the variable "total_num_of_channels" can be set.
 *
 *   Finally, the result for the creacted device is sxpfm_n, in this example:
 * sxpf0_0,sxpf0_1,sxpf0_2,sxpf0_3,sxpf1_0,sxpf1_1, the major number are all
 * equal, the minor number is from 0 to 5.
 *
 *   Def: static int sxpfv_create(int dev_num_sum, int total_num_of_channels)
 *
 * @param dev_num_sum  Number of the detected real physical device
 * @param total_num_of_channels Number of channel for all the devices
 *
 * @return 0 Sucess, negative value failure
 */
static int sxpfv_create(int dev_num_sum, int total_num_of_channels) {
  int i, j, k = 0, ret;

  // Assign the Major number and allocate chardev region
  printk(KERN_INFO "beginn the module creation...\n");
  if (major_number) {
    dev = MKDEV(major_number, minor_number);
    ret = register_chrdev_region(dev, total_num_of_channels,
                                 "sxpfv");  // use the specified device number
  } else {
    ret = alloc_chrdev_region(
        &dev, minor_number, total_num_of_channels,
        "sxpfv");  // dynamic allocate the device number, minor_number from 0 on
    major_number = MAJOR(dev);
  }
  printk(KERN_INFO "total number of channel: %d\n", total_num_of_channels);
  if (ret < 0) {
    printk(KERN_WARNING "device can't get %d\n", major_number);
    goto fail;
  }

  virtual_device_structures = kzalloc(
      sizeof(struct VirtualDeviceData) * total_num_of_channels, GFP_KERNEL);

  if (!virtual_device_structures) {
    printk(KERN_WARNING "alloc memery failed");
    ret = -ENOMEM;
    goto failure_kzalloc;
  }

  // Create necessary number of devices
  for (i = 0; i < total_num_of_channels; i++) {
    // init new device
    cdev_init(&virtual_device_structures[i].cdev, &virtual_dev_fops);
    virtual_device_structures[i].cdev.owner = THIS_MODULE;
    ret = cdev_add(&virtual_device_structures[i].cdev,
                   MKDEV(major_number, minor_number + i), 1);

    if (ret) {
      printk(KERN_WARNING "failed add dev%d", i);
    }
  }

  virtual_device_class = class_create(THIS_MODULE, "sxpfv_cls");
  virtual_device_class->dev_uevent = sxpfv_uevent;

  if (!virtual_device_class) {
    printk(KERN_WARNING "fail cerate class");
    ret = PTR_ERR(virtual_device_class);
    goto failure_class;
  }

  for (j = 0; j < dev_num_sum; j++) {
    for (i = 0; i < channels_per_device[j]; i++) {
      if (!channels_per_device[j]) {
        pr_info("class creation finish\n");
        // break;
      } else {
        if (j > 0) {
          k = channels_per_device[j - 1] +
              i;  // Use k to maintain the later MKDEV() is same as the first
                  // loop MKDEV in line 148
        } else {
          k = i;
        }

        // Create multiple devices, under the device class
        // sys/class/virtual_device_class
        device_create(virtual_device_class, NULL,
                      MKDEV(major_number, minor_number + k), NULL, "sxpf%d_%d",
                      j, i);
      }
    }
  }
  printk(KERN_INFO "end device creation");

return 0;

failure_class:
  kfree(virtual_device_structures);
failure_kzalloc:
  unregister_chrdev_region(dev, nr_dev);
fail:
  return ret;
}

static int __init sxpfv_init(void) {
  int j, temp;

  for (j = 0; j < nr_dev; j++) {
    temp = get_num_of_channels(j);
    temp = 8;  // Always use the maximal number of channel
    total_channels += temp;
    channels_per_device[j] = temp;  // Count the channel number of every device
  }

  pr_info("Debug: nr_dev:%d", nr_dev);
  pr_info("Debug: total number of channels: %d", total_channels);

  sxpfv_create(nr_dev, total_channels);

  return 0;
}

static void __exit sxpfv_exit(void)  // Unregister the device when unload the kernel module
{
  int i, j;
  pr_info("SXPFx: Unload sxpf vitualization kernel module.\n");

  for (i = 0; i < total_channels; i++) {
    device_destroy(virtual_device_class, MKDEV(major_number, minor_number + i));
  }
  class_unregister(virtual_device_class);
  class_destroy(virtual_device_class);
  for (i = 0; i < total_channels; i++) {
    cdev_del(&virtual_device_structures[i].cdev);
  }
  kfree(virtual_device_structures);
  unregister_chrdev_region(dev, total_channels);  // Release the devcie number
  unregister_chrdev_region(MKDEV(major_number, minor_number), MINORMASK);

  pr_info("SXPFx: goodbye.\n");
  printk(KERN_INFO "goodbye");
}

static int sxpfv_uevent(struct device *dev, struct kobj_uevent_env *env) {
  add_uevent_var(env, "DEVMODE=%#o", 0666);
  return 0;
}

static int sxpfv_open(struct inode *inode, struct file *file) {
  struct file *f;
  int j = 0, current_opened_channel, channel_acount = 0;

  char s[12];
  pr_info("SXPFx: open device\n");

  // Document the device number of the currently entering device
  current_dev_ID = inode->i_rdev;  // for the current opened virtual device, not
                                   // the real device!

  // Determine which device should open, according to the minor number of the
  // virtual device
  current_opened_channel = MINOR(current_dev_ID);
  do {
    channel_acount += channels_per_device[j];
    j++;
  } while (current_opened_channel > channel_acount);
  j--;

  sprintf(s, "/dev/sxpf%d", j);

  // IS_ERR - used to check, Returns non zero value if the ptr is an error,
  // Otherwise 0 if it's not an error. PTR_ERR - used to print. Current value of
  // the pointer.
  f = filp_open(s, O_RDWR, 0);
  if (IS_ERR(f)) {
    pr_alert("SXPFx: cannot open file sxpf0\n");
    return (PTR_ERR(f));
  } else if (IS_ERR(f->f_op)) {
    pr_alert("SXPFx: cannot open f->f_op\n");
    return (PTR_ERR(f->f_op));
  } else if (IS_ERR(f->f_op->poll)) {
    pr_alert("SXPFx: cannot open f->f_op->poll\n");
    return (PTR_ERR(f->f_op->poll));  // Not working
  }
    else if (IS_ERR(f->f_op->mmap)) {
    pr_alert("SXPFx: cannot open f->f_op->mmap\n");
    return (PTR_ERR(f->f_op->mmap));  // not working
  }

    else if (IS_ERR(f->f_op->unlocked_ioctl)) {
    pr_alert("SXPFx: cannot open f->unlocked_ioctl\n");
    return (PTR_ERR(f->f_op->unlocked_ioctl));  // Not working
  }

  else if (IS_ERR(f->f_op->read)) {
    pr_alert("SXPFx: cannot open f->read\n");
    return (PTR_ERR(f->f_op->read));  // Not working
  }

  else {
    pr_info("SXPFx: successfully opend sxpf%d\n", j);
    file->private_data = f;
    return 0;
  }
}

static int sxpfv_release(struct inode *inode, struct file *file) {
  int ret;

  ret = filp_close(file->private_data, NULL);
  if (ret) {
    pr_alert("SXPFx: cannot close sxpf0\n");
    return ret;
  } else {
    pr_info("SXPFx: closed device sxpf0\n");
    return 0;
  }
}

static unsigned int sxpfv_poll(struct file *file, poll_table *wait) {
  struct file *f;
  f = file->private_data;
  pr_info("SXPFx: enter sxpfv_poll");

  return f->f_op->poll(f, wait);
}


static int sxpfv_mmap(struct file *file, struct vm_area_struct *vma) {
  struct file *f;
  f = file->private_data;
  pr_info("SXPFx: sxpfv_mmap\n");

  return f->f_op->mmap(f, vma);
}

static long sxpfv_ioctl(struct file *file, unsigned int cmd,
                        unsigned long arg) {

  
  pr_info("SXPFx: sxpfv_ioct\n");

  u32 ch;

                        
  struct file *f = file->private_data;    // file struct from sxpf driver
  //sxpf_file_hdl_t *fh = f->private_data;  // private data from sxpf driver
  //sxpf_card_t *card = fh->card;           // associated card from sxpf driver
  //struct sxpf_buffer *buf;

  pr_info("SXPFx: currently is the device(current_dev_ID)%d, %d, %x\n", MAJOR(current_dev_ID), MINOR(current_dev_ID), current_dev_ID);

  ch = MINOR(current_dev_ID);
  if (ch > 7) {
    ch = ch % 8;
  }

  switch (cmd) {
    case IOCTL_SXPF_RELEASE_FRAME: {

       pr_info("now I am in IOCTL_SXPF_RELEASE_FRAME\n");
       sxpf_frame_info_t request;

        if (copy_from_user(&request, (void __user *)arg, sizeof(request))) {
            return -EIO;
        }
        u32 coded_channel = request.data_size;
        u32 decoded_channel = coded_channel >> 30;
        pr_info("SXPFv: coded channel: %ld, decoded channel: %ld", coded_channel, decoded_channel);

        coded_channel &= 0x3FFFFFFF; // clear the most two bits(30..31) 

        request.data_size = (ch << 30) | coded_channel; // set the modified value to the most two bits
        request.target = ch; // this has no influence to the channal mappping

        pr_info("SXPFv: moified channel number back to user space: %ld, channel_new: %ld", request.data_size, request.data_size >> 30);

        if (copy_to_user((void __user *)arg, &request, sizeof(request))){
          return -EIO;
        }  
      
    } break;
        

    case IOCTL_SXPF_CMD_SYNC: {

        pr_info("now I am in IOCTL_SXPF_CMD_SYNC\n");
        sxpf_cmd_fifo_el_t  request;

        if (copy_from_user(&request, (void __user *)arg, sizeof(request))) {
            return -EIO;
        }

        request.args[3] = ch;

        if (copy_to_user((void __user *)arg, &request, sizeof(request))){
          return -EIO;
        }  

    } break;
        
      
      default:
        pr_alert("Unrecognized ioctl cmd.");
  }

return f->f_op->unlocked_ioctl(f, cmd, arg);
}

static ssize_t sxpfv_read(struct file *file, char __user *buf, size_t count,
                          loff_t *offset) {
  struct file *f;
  f = file->private_data;
  pr_info("SXPFx: sxpfv_read\n");

  pr_info("SXPFV: Read from device\n");
  return f->f_op->read(f, buf, count, offset);
}

module_init(sxpfv_init);
module_exit(sxpfv_exit);