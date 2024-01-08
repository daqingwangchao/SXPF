#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include<sys/ioctl.h>
#include <stdint.h>

#include "sxpftypes.h"

#define IOCTL_SXPF_MAGIC             'p'
/*************this is modify by Zhao ************************/
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

/*************this is modify by Zhao ************************/
#define IOCTL_SXPF_CHAMA_W   _IOW (IOCTL_SXPF_MAGIC, 21, uint32_t * )
#define IOCTL_SXPF_CHAMA_R   _IOR (IOCTL_SXPF_MAGIC, 22, uint32_t * )
#define IOCTL_SXPF_CHAMA_WR   _IOWR (IOCTL_SXPF_MAGIC, 23, uint32_t * )
/*************this is modify by Zhao ************************/
/*************this is modify by Zhao ************************/

int main(int argc, char* argv[]){
int temp=0;
char                *endp;
//sxpf_config_t       config;
//sxpf_config_t       *ch;
//sxpf_trigger_exposure_t trigger_ch, *ch; //IOCTL_SXPF_TRIG_EXPOSURE
//sxpf_enable_user_sequence_t enable_user_sequence_ch, *ch; //IOCTL_SXPF_ENABLE_USER_SEQ
sxpf_i2c_slave_init_t i2c_slave_init_ch, *ch; // IOCTL_SXPF_INIT_I2C_SLAVE

//ch = &config;
//ch = &trigger_ch;
//ch = &enable_user_sequence_ch;
ch = &i2c_slave_init_ch;


    /*************this is modify by Zhao ************************/
    uint32_t channel, dev_nr,ch_nr;
    char s[15];
    dev_nr = strtoul(argv[1], &endp, 0);
    ch_nr = strtoul(argv[2], &endp, 0);
    sprintf(s,"/dev/sxpf%d_%d",dev_nr,ch_nr);

    printf("DEBUG: %s\n",s);
    int dev_ID = open(s, O_RDWR);
    //ch->channel = channel;
    temp = ioctl(dev_ID, IOCTL_SXPF_TRIG_EXPOSURE, ch);
    printf("%d\n",temp);
    if(!temp) perror("write to sxpfx");
    
    channel = ch->channel;
    printf("channel modified: %d of device sxpf%d\n", channel,dev_nr);

    close(dev_ID);
     /*************this is modify by Zhao ************************/

     /*in the real application, the channel can use my modified */
    return 0;
}