/**
 * @file    sxpf.h
 *
 * SX proFRAME frame grabber driver and library public defintions.
 *
 * IOCTL definitions for the sxpf driver.
 *
 * @note    Everything contained in this header file is preliminary, pending
 *          review!
 */
#ifndef SXPF_H_
#define SXPF_H_

#include "sxpftypes.h"

#ifdef __cplusplus
extern "C" {
#define CPP_DEFAULT_NULL    = nullptr
#else
#define CPP_DEFAULT_NULL
#endif

#define SXPF_DEVICE_NAME        "/dev/sxpf%d"


/** Min.\ number of DMA buffers per card the driver needs to function */
#define SXPF_MIN_FRAME_SLOTS    (1)

/** Max.\ number of available DMA buffers per card for video recording/playback.
 *
 * @note    Bit 31 of a 32bit mask value is used to signal a lock-out period by
 *          a proccess currently modifying this avalability mask. Thus, only
 *          31 bits of the mask are available for buffer state indication.
 */
#define SXPF_MAX_FRAME_SLOTS    (31)

#define SXPF_I2C_FRAME_SIZE     (1024)  /**< I2C message DMA buffer size */

/** Max.\ number of available DMA buffers per card for I2C message recording. */
#define SXPF_MAX_I2C_MSG_SLOTS  (31)

/** Max.\ total number of DMA buffers used by the card, incl.\ video and I2C */
#define SXPF_MAX_DMA_BUFFERS    (SXPF_MAX_FRAME_SLOTS + SXPF_MAX_I2C_MSG_SLOTS)


/*
 * Plasma command definitions
 */
#define SXPF_CMD_SHIFT          (24)
#define SXPF_CMD_STATUS_SHIFT   (16)

#define SXPF_CMD_ARG_SHIFT      (0)
#define SXPF_CMD_ARG_MASK       (0xFFFF << SXPF_CMD_ARG_SHIFT)


/** Plasma command codes */
enum sxpf_plasma_cmd_e
{
    SXPF_CMD_MASK                           = 0xFFul << SXPF_CMD_SHIFT,
    SXPF_CMD_NOP                            = 0x00ul << SXPF_CMD_SHIFT,
    SXPF_CMD_I2C_SCAN                       = 0x01ul << SXPF_CMD_SHIFT,
    SXPF_CMD_I2C_TRANSFER                   = 0x02ul << SXPF_CMD_SHIFT,
    SXPF_CMD_I2C_SET_BAUDRATE               = 0x03ul << SXPF_CMD_SHIFT,
    SXPF_CMD_SPI_GET_ID                     = 0x04ul << SXPF_CMD_SHIFT,
    SXPF_CMD_SPI_READ                       = 0x05ul << SXPF_CMD_SHIFT,
    SXPF_CMD_SPI_WRITE                      = 0x06ul << SXPF_CMD_SHIFT,
    SXPF_CMD_SPI_WRITE_VERIFY               = 0x07ul << SXPF_CMD_SHIFT,
    SXPF_CMD_SPI_ERASE_4KB                  = 0x08ul << SXPF_CMD_SHIFT,
    SXPF_CMD_SPI_ERASE_ALL                  = 0x09ul << SXPF_CMD_SHIFT,
    SXPF_CMD_SPI_VERIFY                     = 0x0Aul << SXPF_CMD_SHIFT,
    SXPF_CMD_EEPROM_GET_SIZE                = 0x0Bul << SXPF_CMD_SHIFT,
    SXPF_CMD_EEPROM_READ                    = 0x0Cul << SXPF_CMD_SHIFT,
    SXPF_CMD_EEPROM_WRITE                   = 0x0Dul << SXPF_CMD_SHIFT,
    SXPF_CMD_REGISTER_WRITE                 = 0x0Eul << SXPF_CMD_SHIFT,
    SXPF_CMD_REGISTER_READ                  = 0x0Ful << SXPF_CMD_SHIFT,
    SXPF_CMD_SPI_ERASE_64KB                 = 0x10ul << SXPF_CMD_SHIFT,
    SXPF_CMD_REARM_IRQ                      = 0x11ul << SXPF_CMD_SHIFT,
    SXPF_CMD_USER_EVENT_SET                 = 0x12ul << SXPF_CMD_SHIFT,
    SXPF_CMD_USER_EVENT_REMOVE              = 0x13ul << SXPF_CMD_SHIFT,
    SXPF_CMD_USER_EVENT_GET_LOCATION        = 0x14ul << SXPF_CMD_SHIFT,
};


/** Plasma command return status codes */
enum sxpf_plasma_cmd_status_e
{
    SXPF_CMD_STATUS_MASK                    = 0xFF << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_REQUEST                 = 0x00 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_RESPONSE_OK             = 0x01 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_RESPONSE_ERROR          = 0x02 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_RESPONSE_UNKNOWN_CMD    = 0x03 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_RESPONSE_INVALID_ARG    = 0x04 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_RESPONSE_BUSY           = 0x05 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_RESPONSE_NOT_SUPPORTED  = 0x06 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_I2C_TIMEOUT             = 0x11 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_I2C_BUSY                = 0x12 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_I2C_NO_PRESENCE_ACK     = 0x13 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_I2C_ACK_ERROR           = 0x14 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_I2C_INVALID_ARGUMENT    = 0x15 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_I2C_NOT_SUPPORTED       = 0x16 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_I2C_NO_SLAVE_RESPONSE   = 0x17 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_I2C_ARBITRATION_LOST    = 0x18 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_I2C_INVALID_RESPONSE    = 0x19 << SXPF_CMD_STATUS_SHIFT,
    SXPF_CMD_STATUS_I2C_INVALID_DATA        = 0x1A << SXPF_CMD_STATUS_SHIFT,
};


#ifndef KERNEL

#include <stdarg.h>

/*
 * User space library functions that use the above IOCTLs.
 */

typedef struct sxpf_instance_s      *sxpf_hdl;  /**< Grabber instance handle. */

typedef __u32                       sxpf_buf_t; /**< Grabber buffer handle */


/** I2C transfer result filter.
 *
 * The user application may install a global filter that modifies the the result
 * of every I2C transaction.
 *
 * @param result    The result of the low-level I2C transaction as it was
 *                  returned by the driver to sxpf_i2c_xfer().
 * @param pf        The grabber handle (context info)
 * @param port      The addressed I2C port (context info)
 * @param dev_id    The addressed iI2C device ID (context info)
 * @param wbuf      The I2C command bytes to write (context info)
 * @param wbytes    The number of command bytes in wbuf (context info)
 * @param rbuf      Pointer to the receive buffer (context info)
 * @param rbytes    The number of bytes to read from the device (context info)
 *
 * @result  The translated result code. The default I2C filter returns what was
 *          passed in as \c result.
 */
typedef int (*sxpf_i2c_result_filter_t)(int result, sxpf_hdl pf, int port,
                                        __u8 dev_id,
                                        const __u8 *wbuf, unsigned int wbytes,
                                        __u8 *rbuf, unsigned int rbytes);


/* allocation/deallocation */
int sxpf_get_num_cards(void);
int sxpf_get_revisions(__u32 *library_revision, __u32 *driver_revision);
int sxpf_get_sw_versions(__u32 *library_version, __u32 *driver_version);
sxpf_hdl sxpf_open(int grabber_id);
void sxpf_close(sxpf_hdl pf);

/* configuration */
int sxpf_write_config(sxpf_hdl pf, sxpf_config_t *config);
int sxpf_read_config(sxpf_hdl pf, sxpf_config_t *config);
int sxpf_write_channel_config(sxpf_hdl pf, __u32 trig_chan,
                              sxpf_config_t *config);
int sxpf_read_channel_config(sxpf_hdl pf, __u32 trig_chan,
                             sxpf_config_t *config);
int sxpf_get_card_properties(sxpf_hdl pf, sxpf_card_props_t *props);
int sxpf_cmd_sync(sxpf_hdl pf, sxpf_cmd_fifo_el_t *req);
int sxpf_adapter_get_osc_freq(sxpf_hdl pf, uint8_t channel, uint32_t *freq,
                              uint32_t factoryFreq);
int sxpf_osc_modify_adapter(sxpf_hdl pf, __u8 channel, double newFreq,
                            __u32 factoryFreq);

int sxpf_get_single_card_info(sxpf_hdl pf, sxpf_card_info_t *card_info);

int sxpf_get_card_temperature(sxpf_hdl pf,
                              double *fpga_temp_celsius);

/* stream control */
int sxpf_start_record(sxpf_hdl pf, __u32 channel_mask);
int sxpf_start_playback(sxpf_hdl pf, __u32 channel_mask);
int sxpf_stop(sxpf_hdl pf, __u32 channel_mask);
int sxpf_trigger_exposure(sxpf_hdl pf, __u32 exposure);
int sxpf_trigger_channel_exposure(sxpf_hdl pf, __u32 trig_chan, __u32 exposure);

/* streaming */
int sxpf_read_event(sxpf_hdl pf, sxpf_event_t *evt_buf, int max_events);
void *sxpf_get_frame_ptr(sxpf_hdl pf, sxpf_buf_t slot);
sxpf_image_header_t *sxpf_get_image_ptr(sxpf_hdl pf, sxpf_buf_t slot,
                                        void **paddr_payload);
int sxpf_release_frame(sxpf_hdl pf, sxpf_buf_t slot, __u32 data_size);
int sxpf_release_frame_ex(sxpf_hdl pf, sxpf_buf_t slot, __u32 data_size,
                          int32_t target);
int sxpf_get_device_fd(sxpf_hdl pf, HWAITSXPF *fd);
int sxpf_wait_events(int nfds, HWAITSXPF *fds, int timeout_ms);
int sxpf_get_timestamp(sxpf_hdl pf, __s64 *timestamp);
int sxpf_get_secondary_timestamp(sxpf_hdl pf, __s64 *timestamp);
__s64 sxpf_get_system_time(sxpf_clock_select_t clock_source);
__s64 sxpf_get_system_clock_rate(sxpf_clock_select_t clock_source);
int sxpf_timestamp_sync(sxpf_hdl pf, sxpf_timestamp_sync_t *timestamp_sync);
int sxpf_alloc_playback_frame(sxpf_hdl pf, sxpf_buf_t *slot, __u32 timeout_us);
int sxpf_declare_heap_buffer(sxpf_hdl pf, void *header, void *payload,
                             size_t size);
int sxpf_declare_user_buffer(sxpf_hdl pf, sxpf_buf_alloc_type_t type,
                             void *header, void *payload, uint32_t size);
int sxpf_free_user_buffer(sxpf_hdl pf, sxpf_buf_t slot);
int sxpf_free_playback_frame(sxpf_hdl pf, sxpf_buf_t slot);
int sxpf_get_buffer_header(sxpf_hdl pf, sxpf_buf_t slot, void *header);

/* I2C */
void sxpf_set_i2c_result_filter(sxpf_i2c_result_filter_t filter);
int sxpf_i2c_xfer(sxpf_hdl pf, int chan, unsigned char dev_id,
                  const unsigned char *wbuf, unsigned int wbytes,
                  unsigned char *rbuf, unsigned int rbytes);
int sxpf_store_user_sequence(sxpf_hdl pf, __u32 ram_offset, __u8 *sequence,
                             __u32 len);
int sxpf_trigger_i2c_update(sxpf_hdl pf, int chan, int scanline, int repeat,
                            __u32 ram_offset);
int sxpf_disable_i2c_update(sxpf_hdl pf, int chan);

int sxpf_i2c_baudrate(sxpf_hdl pf, int chan, __u32 baudrate);

int sxpf_i2c_slave_init(sxpf_hdl pf, __u32 i2c_chan, __u8 dev_id);
int sxpf_i2c_slave_remove(sxpf_hdl pf, __u8 dev_id);
int sxpf_i2c_slave_tx_ack(sxpf_hdl pf, __u8 handle,
                          __u8 *tx_data, __u32 tx_size);

/* EEPROM */
int sxpf_eeprom_getSize(sxpf_hdl pf, __u32 channel, __u32* size);
int sxpf_eeprom_readBytes(sxpf_hdl pf, __u32 channel, __u32 offset,
                          __u8* bytes, __u32 length);
int sxpf_eeprom_writeBytes(sxpf_hdl pf, __u32 channel, __u32 offset,
                           __u8* bytes, __u32 length);

/* SPI-Flash */
int sxpf_flash_init(sxpf_hdl pf);
int sxpf_flash_get_info(sxpf_hdl pf, sxpf_flash_info_t* info);
int sxpf_flash_write(sxpf_hdl pf, __u32 address, __u8* bytes, __u32 size);
int sxpf_flash_read(sxpf_hdl pf, __u32 address, __u8* bytes, __u32 size);
int sxpf_flash_eraseSector4KB(sxpf_hdl pf, __u32 address);
int sxpf_flash_eraseSector64KB(sxpf_hdl pf, __u32 address);
int sxpf_flash_eraseSector128KB(sxpf_hdl pf, __u32 address);

/* PLASMA register access - only bar2 registers accessible */
int sxpf_plasma_writeRegister(sxpf_hdl pf, __u32 reg, __u32 val);
int sxpf_plasma_readRegister(sxpf_hdl pf, __u32 reg, __u32* val);

/* init sequence setup */
__u8* sxpf_load_init_sequence(const char *iniFileName, __u32 *data_size);
void sxpf_free_init_sequence(uint8_t* ini_buffer);
int sxpf_send_init_sequence(sxpf_hdl fg, __u8 *ini_data, __u32 data_size,
                            __u32 port, __u32 execute_event_id);
int sxpf_send_init_sequence_verify(sxpf_hdl fg, __u8 *ini_data, __u32 data_size,
                                   __u32 port, __u32 execute_event_id,
                                   __u8 *regsize_table,
                                   int (*log_cb)(void *log_ctx, char const *fmt,
                                                 va_list args) CPP_DEFAULT_NULL,
                                   void *log_ctx CPP_DEFAULT_NULL);
int sxpf_decode_init_sequence(__u8 *ini_data, __u32 data_size,
                              char* out_buf, __u32 buf_size);
int sxpf_analyze_init_sequence(uint8_t* data, uint32_t data_size,
                               sxpf_ini_props_t* props);

/* low-level access */
int sxpf_read_register(sxpf_hdl pf, __u32 bar, __u32 reg_off, __u32 *value);
int sxpf_write_register(sxpf_hdl pf,  __u32 bar, __u32 reg_off, __u32 value);

#endif /* ! KERNEL */

#ifdef __cplusplus
}
#endif

#endif  /* SXPF_H_ */
