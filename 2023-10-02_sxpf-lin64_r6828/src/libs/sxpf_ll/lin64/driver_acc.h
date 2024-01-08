#ifndef SXPF_DRIVER_H_
#define SXPF_DRIVER_H_

#include <linux/ioctl.h>
#include <linux/types.h>


typedef int     HSXPFDEV;   /* Posix file descriptor */
typedef int     HWAITSXPF;


#include "sxpf_instance.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline int DEV_VALID(HSXPFDEV dev)   { return dev >= 0; }

#define DEV_OPEN                    open
#define DEV_DUP                     dup
#define DEV_CLOSE_WAIT_DEV          close
#define DEV_CLOSE                   close
#define DEV_IOCTL                   ioctl
#define DEV_READ                    read
#define DEV_MMAP                    mmap
#define DEV_MUNMAP                  munmap

#define SYSTEM_PAGE_SIZE            sysconf(_SC_PAGE_SIZE)


#define IOCTL_SXPF_MAGIC             'p'

/* config interface is deprecated! */
#define IOCTL_SXPF_READ_CONFIG      _IOWR(IOCTL_SXPF_MAGIC, 0, sxpf_config_t)
#define IOCTL_SXPF_WRITE_CONFIG     _IOW (IOCTL_SXPF_MAGIC, 1, sxpf_config_t)

#define IOCTL_SXPF_GET_CARD_PROPS   _IOR (IOCTL_SXPF_MAGIC, 3, sxpf_card_props_t)
#define IOCTL_SXPF_RELEASE_FRAME    _IOW (IOCTL_SXPF_MAGIC, 4, sxpf_frame_info_t)

#define IOCTL_SXPF_START_RECORD     _IO  (IOCTL_SXPF_MAGIC, 5)
#define IOCTL_SXPF_STOP             _IO  (IOCTL_SXPF_MAGIC, 6)
#define IOCTL_SXPF_START_PLAYBACK   _IO  (IOCTL_SXPF_MAGIC, 7)
#define IOCTL_SXPF_ALLOC_PLAY_FRAME _IOWR(IOCTL_SXPF_MAGIC, 7, __u32)

#define IOCTL_SXPF_READ_REG         _IOWR(IOCTL_SXPF_MAGIC, 8, sxpf_rw_register_t)
#define IOCTL_SXPF_WRITE_REG        _IOW (IOCTL_SXPF_MAGIC, 8, sxpf_rw_register_t)

#define IOCTL_SXPF_CMD_SYNC         _IOWR(IOCTL_SXPF_MAGIC, 9, sxpf_cmd_fifo_el_t)
#define IOCTL_SXPF_TRIG_EXPOSURE    _IOW (IOCTL_SXPF_MAGIC,10, sxpf_trigger_exposure_t)

#define IOCTL_SXPF_GET_TIMESTAMP    _IOR (IOCTL_SXPF_MAGIC,11, __s64)

#define IOCTL_SXPF_SET_USR_SEQUENCE _IOW (IOCTL_SXPF_MAGIC,12, \
                                          sxpf_user_sequence_t)
#define IOCTL_SXPF_ENABLE_USER_SEQ  _IOW (IOCTL_SXPF_MAGIC,13, \
                                          sxpf_enable_user_sequence_t)

#define IOCTL_SXPF_TIMESTAMP_SYNC   _IOWR(IOCTL_SXPF_MAGIC,14, \
                                          sxpf_timestamp_sync_t)

#define IOCTL_SXPF_GET_TIMESTAMP_SEC _IOR(IOCTL_SXPF_MAGIC,15, __s64)

#define IOCTL_SXPF_GET_BUF_HDR      _IOWR(IOCTL_SXPF_MAGIC, 16, sxpf_get_buf_hdr_t)

#define IOCTL_SXPF_INIT_USER_BUFFER _IOWR(IOCTL_SXPF_MAGIC, 17, sxpf_user_buffer_declare_t)

#define IOCTL_SXPF_GET_TIMESTAMP_ALT _IOR(IOCTL_SXPF_MAGIC, 18, __s64)

#define IOCTL_SXPF_INIT_I2C_SLAVE    _IOWR(IOCTL_SXPF_MAGIC, 19, sxpf_i2c_slave_init_t)
#define IOCTL_SXPF_REMOVE_I2C_SLAVE  _IOW (IOCTL_SXPF_MAGIC, 19, __u32)
#define IOCTL_SXPF_I2C_SLAVE_TX_ACK  _IOW (IOCTL_SXPF_MAGIC, 20, sxpf_i2c_slave_tx_ack_t)

#ifdef __cplusplus
};
#endif

#endif /* SXPF_DRIVER_H_ */

