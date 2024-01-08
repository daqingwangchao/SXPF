#ifndef SXPFTYPES_H_
#define SXPFTYPES_H_

// define base types that are dependent on the host operating system
#if defined(_MSC_VER)

// Microsoft Visual C++ 64 bit definitions

#ifdef NT_KERNEL

//#include <ntddk.h>
//#include <wdf.h>

typedef INT32       int32_t;
typedef INT64       int64_t;
typedef UINT8       uint8_t;
typedef UINT16      uint16_t;
typedef UINT32      uint32_t;
typedef UINT64      uint64_t;

#else   /* defined(NT_KERNEL) */

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <assert.h>
#include <stdint.h>
#include <sys/types.h>
#include <time.h>
#include <windows.h>

#define BUG_ON(cond)    assert(!(cond))

#endif  /* defined(NT_KERNEL) */

typedef int64_t     s64;
typedef uint8_t     u8;
typedef uint32_t    u32;
typedef uint64_t    u64;

typedef int64_t     __s64;
typedef uint32_t    __u32;
typedef uint64_t    __u64;
typedef uint16_t    __u16;
typedef uint8_t     __u8;


#include <BaseTsd.h>
typedef SSIZE_T ssize_t;

typedef HANDLE      HWAITSXPF;

#else

// Linux 64 bit definitions

#ifndef KERNEL
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>
#endif

#include <linux/ioctl.h>
#include <linux/types.h>

typedef int         HWAITSXPF;

#endif

#define SXPF_NUM_ARGS_PER_CMD    (256)   /**< Determined by "largest" command */


/** Bitmasks of error states in record mode */
typedef enum
{
    SXPF_RXERR_VID_OVERFLOW_0           = (1 << 0), /**< Data overflow in video data path #0, i.e. data is lost due to insufficient bandwidth. */
    SXPF_RXERR_VID_OVERFLOW_1           = (1 << 1), /**< Data overflow in video data path #1, i.e. data is lost due to insufficient bandwidth. */
    SXPF_RXERR_VID_OVERFLOW_2           = (1 << 2), /**< Data overflow in video data path #2, i.e. data is lost due to insufficient bandwidth. */
    SXPF_RXERR_VID_OVERFLOW_3           = (1 << 3), /**< Data overflow in video data path #3, i.e. data is lost due to insufficient bandwidth. */

    SXPF_RXERR_VID_FRAME_DROP_0         = (1 << 4), /**< No valid PCI address is available, so the current video frame is discarded. */
    SXPF_RXERR_VID_FRAME_DROP_1         = (1 << 5), /**< No valid PCI address is available, so the current video frame is discarded. */
    SXPF_RXERR_VID_FRAME_DROP_2         = (1 << 6), /**< No valid PCI address is available, so the current video frame is discarded. */
    SXPF_RXERR_VID_FRAME_DROP_3         = (1 << 7), /**< No valid PCI address is available, so the current video frame is discarded. */

    SXPF_RXERR_I2C_OVERFLOW_0           = (1 << 8),  /**< Data overflow in I2C path #0, i.e. data is lost due to insufficient bandwidth. */
    SXPF_RXERR_I2C_OVERFLOW_1           = (1 << 9),  /**< Data overflow in I2C path #0, i.e. data is lost due to insufficient bandwidth. */
    SXPF_RXERR_I2C_OVERFLOW_2           = (1 << 10), /**< Data overflow in I2C path #0, i.e. data is lost due to insufficient bandwidth. */
    SXPF_RXERR_I2C_OVERFLOW_3           = (1 << 11), /**< Data overflow in I2C path #0, i.e. data is lost due to insufficient bandwidth. */

    SXPF_RXERR_I2C_FRAME_DROP_0         = (1 << 12), /**< No valid PCI address is available, so the current I2C message is discarded. */
    SXPF_RXERR_I2C_FRAME_DROP_1         = (1 << 13), /**< No valid PCI address is available, so the current I2C message is discarded. */
    SXPF_RXERR_I2C_FRAME_DROP_2         = (1 << 14), /**< No valid PCI address is available, so the current I2C message is discarded. */
    SXPF_RXERR_I2C_FRAME_DROP_3         = (1 << 15), /**< No valid PCI address is available, so the current I2C message is discarded. */

    SXPF_RXERR_VID_DATA_TOO_LARGE       = (1 << 16), /**< Error occurred since the current frame size exceeds the maximum supported video buffer size. */
    SXPF_RXERR_I2C_DATA_TOO_LARGE       = (1 << 17), /**< Error occurred since the current I2C message exceeds the maximum supported buffer size. */

    SXPF_RXERR_VID_CMD_FIFO_OVERFLOW    = (1 << 18), /**< PCI address FIFO drops an error since there is no space left. The last written address is lost. */
    SXPF_RXERR_I2C_CMD_FIFO_OVERFLOW    = (1 << 19), /**< PCI address FIFO drops an error since there is no space left. The last written address is lost. */

    SXPF_RXERR_VID_COMPL_FIFO_UNDERFLOW = (1 << 20), /**< A read access took place although the PCI address FIFO was empty. The address just read back is invalid. */
    SXPF_RXERR_I2C_COMPL_FIFO_UNDERFLOW = (1 << 21), /**< A read access took place although the PCI address FIFO was empty. The address just read back is invalid. */

    SXPF_RXERR_EXT_SYNC_LOSS            = (1 << 22), /**< Obsolete error flag. Subject to be removed. */
    SXPF_RXERR_VID_CLOCK_NOT_LOCKED     = (1 << 23), /**< Obsolete error flag. Subject to be removed. */
    SXPF_RXERR_MEM_CALIBRATION_FAILED   = (1 << 24), /**< Obsolete error flag. Subject to be removed. */

 } sxpf_capture_error_t;


/** Bitmasks of error states in playback mode */
typedef enum
{
    SXPF_TXERR_VID_UNDERFLOW_0          = (1 << 0), /**< Data underflow in video path #0, i.e. not enough data was provided during video replay due to insufficient bandwidth. */
    SXPF_TXERR_VID_UNDERFLOW_1          = (1 << 1), /**< Data underflow in video path #1, i.e. not enough data was provided during video replay due to insufficient bandwidth. */
    SXPF_TXERR_VID_UNDERFLOW_2          = (1 << 2), /**< Data underflow in video path #2, i.e. not enough data was provided during video replay due to insufficient bandwidth. */
    SXPF_TXERR_VID_UNDERFLOW_3          = (1 << 3), /**< Data underflow in video path #3, i.e. not enough data was provided during video replay due to insufficient bandwidth. */

    SXPF_TXERR_VID_TIMESTAMP_START_0    = (1 << 4), /**< Start timestamp error in video channel #0. Video replay started after the configured starting time. */
    SXPF_TXERR_VID_TIMESTAMP_START_1    = (1 << 5), /**< Start timestamp error in video channel #1. Video replay started after the configured starting time. */
    SXPF_TXERR_VID_TIMESTAMP_START_2    = (1 << 6), /**< Start timestamp error in video channel #2. Video replay started after the configured starting time. */
    SXPF_TXERR_VID_TIMESTAMP_START_3    = (1 << 7), /**< Start timestamp error in video channel #3. Video replay started after the configured starting time. */

    SXPF_TXERR_VID_TIMESTAMP_END_0      = (1 << 8), /**< End timestamp error in video channel #0. Video replay ended after the configured end time. */
    SXPF_TXERR_VID_TIMESTAMP_END_1      = (1 << 9), /**< End timestamp error in video channel #1. Video replay ended after the configured end time. */
    SXPF_TXERR_VID_TIMESTAMP_END_2      = (1 << 10), /**< End timestamp error in video channel #2. Video replay ended after the configured end time. */
    SXPF_TXERR_VID_TIMESTAMP_END_3      = (1 << 11), /**< End timestamp error in video channel #3. Video replay ended after the configured end time. */

    SXPF_TXERR_VID_DMA_SIZE             = (1 << 16), /**< Error occurred since the issued DMA size exceeds the maximum supported video buffer size. */
    SXPF_TXERR_I2C_DMA_SIZE             = (1 << 17), /**< Error occurred since the issued DMA size exceeds the maximum supported buffer size. */

    SXPF_TXERR_VID_CMD_FIFO_OVERFLOW    = (1 << 18), /**< PCI address FIFO drops an error since there is no space left. The last written address is lost. */
    SXPF_TXERR_I2C_CMD_FIFO_OVERFLOW    = (1 << 19), /**< PCI address FIFO drops an error since there is no space left. The last written address is lost. */

    SXPF_TXERR_VID_COMPL_FIFO_UNDERFLOW = (1 << 20), /**< A read access took place although the PCI address FIFO was empty. The address just read back is invalid. */
    SXPF_TXERR_I2C_COMPL_FIFO_UNDERFLOW = (1 << 21), /**< A read access took place although the PCI address FIFO was empty. The address just read back is invalid. */

    SXPF_TXERR_EXT_SYNC_LOSS            = (1 << 22), /**< Obsolete error flag. Subject to be removed. */

    SXPF_TXERR_MEM_CALIBRATION_FAILED   = (1 << 24), /**< Obsolete error flag. Subject to be removed. */

} sxpf_replay_error_t;


/** Possible selections for video trigger generation */
typedef enum
{
    SXPF_TRIG_ENABLE     = (1 << 0), /**< Enable trigger pulse generation */
    SXPF_TRIG_DISABLE    = (0 << 0), /**< Disable trigger pulse generation */

    SXPF_TRIG_POL_MASK   = (1 << 1), /**< Trigger pulse polarity mask */
    SXPF_TRIG_POL_POS    = (0 << 1), /**< Trigger pulse is active-high */
    SXPF_TRIG_POL_NEG    = (1 << 1), /**< Trigger pulse is active-low */

    SXPF_TRIG_CHAN_MASK = (15 << 4), /**< Channel enable mask */
    SXPF_TRIG_CHAN_EN_0  = (1 << 4), /**< Channel 0 enable bit */
    SXPF_TRIG_CHAN_EN_1  = (1 << 5), /**< Channel 1 enable bit */
    SXPF_TRIG_CHAN_EN_2  = (1 << 6), /**< Channel 2 enable bit */
    SXPF_TRIG_CHAN_EN_3  = (1 << 7), /**< Channel 3 enable bit */

    SXPF_TRIG_TYPE_MASK  = (7 << 8), /**< Trigger type mask */
    SXPF_TRIG_CONTINUOUS = (0 << 8), /**< Continuous trigger generation */
    SXPF_TRIG_MANUAL     = (1 << 8), /**< Manual generation of single trigger */
    SXPF_TRIG_EXTERNAL   = (2 << 8), /**< Use external trigger pulse */
    SXPF_TRIG_EXT_MOD    = (3 << 8), /**< Use ext.\ trig IN, modify OUT width */
    SXPF_TRIG_EXT_MULT   = (4 << 8), /**< Use ext.\ trig IN, mult.\ OUT freq */
    SXPF_TRIG_OUT_DIV    = (5 << 8), /**< Cont capture trig, divide OUT freq */
    SXPF_TRIG_EXT_MULT_USER=(6<< 8), /**< Use ext.\ trig IN, mult.\ OUT freq (user mode) */

    SXPF_TRIG_MODULE_ENABLE=(1<<31), /**< Enable individual trigger modules */

    SXPF_TS_CONTROL_ENABLE        = (1 << 0), /**< Timestamp unit enable */
    SXPF_TS_CONTROL_TX            = (1 << 1), /**< Timestamp sender */
    SXPF_TS_CONTROL_RX            = (1 << 2), /**< Timestamp receiver */
    SXPF_TS_CONTROL_BYPASS        = (1 << 3), /**< Timestamp bypass */
    SXPF_TS_CONTROL_POLARITY      = (1 << 4), /**< Timestamp polarity */
    SXPF_TS_CONTROL_UNIT_ERROR    = (1 << 8), /**< Timestamp unit error */
    SXPF_TS_CONTROL_ENCODING_DONE = (1 << 10),/**< Timestamp encoding done */
    SXPF_TS_CONTROL_ENCODER_IDLE  = (1 << 11),/**< Timestamp encoder idle */
    SXPF_TS_CONTROL_ENCODER_ERROR = (1 << 12),/**< Timestamp encoder error */
    SXPF_TS_CONTROL_DECODING_DONE = (1 << 13),/**< Timestamp decoding done */
    SXPF_TS_CONTROL_DECODER_IDLE  = (1 << 14),/**< Timestamp decoder idle */
    SXPF_TS_CONTROL_DECODER_ERROR = (1 << 15),/**< Timestamp decoder error */

} sxpf_trigger_mode_t;


/** SXPF card hardware capability flags */
typedef enum
{
    SXPF_CAP_VIDEO_RECORD   = (1 << 0), /**< Card can be used for recording */
    SXPF_CAP_VIDEO_PLAYBACK = (1 << 1), /**< Card can be used for playback */
} sxpf_caps_t;


/** SXPF software capability flags */
typedef enum
{
    SXPF_SW_CAP_USER_ALLOC  = (1 << 0), /**< User-space buffers (for GPU DMA) */
} sxpf_sw_caps_t;


/** SXPF card properties */
typedef struct
{
    __u32   buffer_size;     /**< Buffer size available for video frames */
    __u32   fw_date;         /**< Firmware date code */
    __u32   fw_version;      /**< Firmware version code */
    __u32   capabilities;    /**< Hardware capabilities. @see sxpf_caps_t */
    __u32   num_buffers;     /**< Number of available image DMA buffers */
    __u32   num_user_buffers;/**< Number of image DMA buffers that can be
                              *   provided by the user */
    __u32   speed_index;     /**< Available bandwidth factor of 180MB/s */
    __u32   pcie_width;      /**< PCIe lane width */
    __u32   pcie_gen;        /**< PCIe generation */
    __u32   num_channels;    /**< Number of available video channels */
    __u32   i2c_buffer_size; /**< Buffer size available for I2C messages */
    __u32   num_i2c_buffers; /**< Number of I2C capture DMA buffers */
    __u32   sw_caps;         /**< API feature bits. @see sxpf_sw_caps_t */

} sxpf_card_props_t;


/** Access Hardware trigger configuration. */
typedef struct
{
    /** Select trigger channel to operate on:
     *  - 0 = global or channel 0,
     *  - 1..3 = individual settings
     *
     * @note: sxpf_write*_config() and sxpf_read*_config() set this member
     *        automatically based on the passed or implied channel number.
     */
    __u32   channel;

    /** Video trigger mode. This value is composed of an OR-combination
     *  of elements from the sxpf_trigger_mode_t enumeration.
     *
     * Examples:
     *  - SXPF_TRIG_DISABLE
     *  - SXPF_TRIG_ENABLE | SXPF_TRIG_CONTINUOUS | SXPF_TRIG_POL_NEG
     *  - SXPF_TRIG_ENABLE | SXPF_TRIG_MANUAL | SXPF_TRIG_POL_POS
     *  - SXPF_TRIG_ENABLE | SXPF_TRIG_EXTERNAL
     *  - SXPF_TRIG_ENABLE | SXPF_TRIG_EXT_MOD | SXPF_TRIG_POL_NEG
     */
    __u32   trig_mode;
    __u32   trig_period;        /**< Trigger period in units of 1e-8s */
    __u32   trig_length;        /**< Exposure in units of 1e-8s */
    __u32   trig_delay;         /**< Capture Trigger delay in units of 1e-8s */
    __u32   trig_ext_length;    /**< Trig.\ out length in units of 1e-8s */
    __u32   trig_in_mult;       /**< Trigger input frequency multiplier */
    __u32   trig_user_div;      /**< Trigger user divisor
                                      = 1 / TRIGG_IN_MULT * 2^31
                                      UQ1.31 notation,
                                      valid range [0.0, 1.0] */
    __u32   trig_out_div;       /**< Trigger output frequency divider */
    __u32   pps_source_ext;     /**< External PPS source */
	__u32   trig_source_ext;    /**< External trigger source */
    __u32   timestamp_mode;     /**< Timestamp mode */
    __u32   timestamp_offset;   /**< Timestamp offset */
    __u32   trig_secondary_in;  /**< Secondary external trigger input */

} sxpf_config_t;


/** Send a single trigger Pulse in manual trigger mode. */
typedef struct
{
    /** Select trigger channel to operate on:
     *  - 0 = global or channel 0,
     *  - 1..3 = individual settings
     */
    __u32   channel;

    /** The desired exposure time in units of 1e-8 seconds. */
    __u32   exposure;

} sxpf_trigger_exposure_t;


/** Stream ID bitmasks for use with IOCTL_SXPF_START/IOCTL_SXPF_STOP, and hence
 *  also in sxpf_start().
 */
typedef enum
{
    SXPF_STREAM_VIDEO0      = (1 << 2), /**< Enables video channel 0 */
    SXPF_STREAM_VIDEO1      = (1 << 3), /**< Enables video channel 1 */
    SXPF_STREAM_VIDEO2      = (1 << 4), /**< Enables video channel 2 */
    SXPF_STREAM_VIDEO3      = (1 << 5), /**< Enables video channel 3 */
    SXPF_STREAM_VIDEO4      = (1 << 6), /**< Enables video channel 4 */
    SXPF_STREAM_VIDEO5      = (1 << 7), /**< Enables video channel 5 */
    SXPF_STREAM_VIDEO6      = (1 << 8), /**< Enables video channel 6 */
    SXPF_STREAM_VIDEO7      = (1 << 9), /**< Enables video channel 7 */

    SXPF_STREAM_I2C0        = (1 << 20), /**< Enables I2C channel 0 */
    SXPF_STREAM_I2C1        = (1 << 21), /**< Enables I2C channel 1 */
    SXPF_STREAM_I2C2        = (1 << 22), /**< Enables I2C channel 2 */
    SXPF_STREAM_I2C3        = (1 << 23), /**< Enables I2C channel 3 */
    SXPF_STREAM_I2C4        = (1 << 24), /**< Enables I2C channel 4 */
    SXPF_STREAM_I2C5        = (1 << 25), /**< Enables I2C channel 5 */
    SXPF_STREAM_I2C6        = (1 << 26), /**< Enables I2C channel 6 */
    SXPF_STREAM_I2C7        = (1 << 27), /**< Enables I2C channel 7 */

    /** Enables all received video streams */
    SXPF_STREAM_ALL_VIDEO   =
        SXPF_STREAM_VIDEO0 | SXPF_STREAM_VIDEO1 | SXPF_STREAM_VIDEO2 |
        SXPF_STREAM_VIDEO3 | SXPF_STREAM_VIDEO4 | SXPF_STREAM_VIDEO5 |
        SXPF_STREAM_VIDEO6 | SXPF_STREAM_VIDEO7,

    /** Enables all received I2C streams */
    SXPF_STREAM_ALL_I2C     =
        SXPF_STREAM_I2C0 | SXPF_STREAM_I2C1 | SXPF_STREAM_I2C2 |
        SXPF_STREAM_I2C3 | SXPF_STREAM_I2C4 | SXPF_STREAM_I2C5 |
        SXPF_STREAM_I2C6 | SXPF_STREAM_I2C7,

    /** Enables all received streams */
    SXPF_STREAM_ALL         = SXPF_STREAM_ALL_VIDEO | SXPF_STREAM_ALL_I2C,

} sxpf_channel_t;


typedef struct sxpf_image_header_s  sxpf_image_header_t;
typedef struct sxpf_meta_header_s   sxpf_meta_header_t;

#include "packed.h"  /* start structure packing */

/** Image header provided by FPGA with each received video frame. */
struct sxpf_image_header_s
{
    // information fields that are delivered in this layout for all frame types
    __u32       frame_size;         /**< total size including header */
    __u32       version: 8;         /**< header version, currently: 0 */
    __u32       type: 8;            /**< frame type: video=1 */
    __u32       cam_id: 4;          /**< channel number: 0..3 */
    __u32       card_id: 4;         /**< card ID, fixed at 0 */
    __u32       reserved: 8;        /**< reserved for future extensions */

    // the following fields are frame type specific
    __u16       payload_offset;     /**< actual header size */
    __u8        bpp;                /**< number of bits captured per clock cycle */
    __u8        sample_size;        /**< obsolete: actual number of active pixel bits per clock cycle */
    __u16       rows;               /**< number of rows/CSI-2 packages in image frame */
    __u16       columns;            /**< irrelevant if capturing CSI-2 headers/footers;
                                     *   otherwise number of clock cycles per line */
    __u32       ts_start_lo;        /**< bits[31:0] of start timestamp based on 40MHz counter */
    __u32       ts_start_hi;        /**< bits[63:32] of start timestamp based on 40MHz counter */
    __u32       ts_end_lo;          /**< bits[31:0] of end timestamp based on 40MHz counter */
    __u32       ts_end_hi;          /**< bits[63:32] of end timestamp based on 40MHz counter */
    __u32       frame_counter;      /**< frame counter provided by FPGA */
    __u32       ilg;                /**< inter-line gap in clock cycles */
    __u32       ifg;                /**< inter-frame gap in clock cycles */
    __u32       reserved2;          /**< reserved for future extensions */
    __u32       ts_second_start_lo; /**< bits[31:0] of start timestamp */
    __u32       ts_second_start_hi; /**< bits[63:32] of start timestamp */
    __u32       ts_second_end_lo;   /**< bits[31:0] of end timestamp */
    __u32       ts_second_end_hi;   /**< bits[63:32] of end timestamp */

    // the payload data follows in memory directly after the preceeding
    // header fields, as indicated by the payload_offset member

} PACKED;

/** Meta data header provided by FPGA with each received I2C message. */
struct sxpf_meta_header_s
{
    // information fields that are delivered in this layout for all frame types
    __u32       frame_size;         /**< total size including header */
    __u32       version: 8;         /**< header version, currently: 1 */
    __u32       type: 8;            /**< frame type: I2C=4 (Eth=5 unsupp.) */
    __u32       cam_id: 4;          /**< channel number: 0..3 */
    __u32       card_id: 4;         /**< card ID, fixed at 0 */
    __u32       frame_incmpl : 1;   /**< frame incomplete */
    __u32       frame_err : 1;      /**< frame error */
    __u32       reserved: 6;        /**< reserved for future extensions */

    // the following fields are frame type specific
    __u16       payload_offset;     /**< actual header size */
    __u16       payload_size;       /**< actual payload size */
    __u32       reserved2;          /**< reserved for future extensions */
    __u32       ts_start_lo;        /**< bits[31:0] of start timestamp */
    __u32       ts_start_hi;        /**< bits[63:32] of start timestamp */
    __u32       ts_end_lo;          /**< bits[31:0] of end timestamp */
    __u32       ts_end_hi;          /**< bits[63:32] of end timestamp */
    __u32       frame_counter;      /**< frame counter provided by FPGA */
    __u32       reserved3;          /**< reserved for future extensions */
    __u32       reserved4;          /**< reserved for future extensions */
    __u32       reserved5;          /**< reserved for future extensions */
    __u32       ts_second_start_lo; /**< bits[31:0] of start timestamp */
    __u32       ts_second_start_hi; /**< bits[63:32] of start timestamp */
    __u32       ts_second_end_lo;   /**< bits[31:0] of end timestamp */
    __u32       ts_second_end_hi;   /**< bits[63:32] of end timestamp */

    // the payload data follows in memory directly after the preceeding
    // header fields, as indicated by the payload_offset member

} PACKED;

#include "endpacked.h"  /* no more structure packing */


/** enumeration of known DMA stream identifiers */
typedef enum
{
    SXPF_DATA_TYPE_VIDEO    = 0,    /**< Video DMA stream */
    SXPF_DATA_TYPE_META     = 1,    /**< Meta data (I2C) stream */

} sxpf_stream_type_t;


/** for releasing/posting a buffer to the driver or hardware (@see
 *  sxpf_release_frame), this enum contains possible targets that are not a
 *  specific channel.
 */
typedef enum
{
    SXPF_FREE_BUFFER = -1,          /**< free exclusively owned buffer */
    SXPF_RELEASE_TO_ALL_CHAN = -2,  /**< post buffer to hardware for capture
                                         on any channel */
} sxpf_release_buf_target_t;


/** Low-level driver information about location and frame ID of a captured
 *  image.
 *
 *  Used in IOCTL \ref IOCTL_SXPF_RELEASE_FRAME
 */
typedef struct
{
    int32_t slot;       /**< [In/Out] Slot number used for storing the image. */

    /** [Out] used size in bytes for captured data
     *  [In] number of bytes prepared to send via DMA
     */
    __u32   data_size;

    /** specifies where the buffer should go. @see sxpf_release_buf_target_t
     * - negative value: generic target: @see sxpf_release_buf_target_t
     * - value >= 0: specific channel number to post the buffer to
     */
    int32_t target;

} sxpf_frame_info_t;


/** IOCTL structure for writing/reading single FPGA registers. */
typedef struct
{
    __u32   bar;    /**< BAR number the register is located in */
    __u32   offs;   /**< Register's offset from start of BAR */
    __u32   data;   /**< Register value */
    __u32   size;   /**< word size in bytes. allowed values: 1, 2, 4 */

} sxpf_rw_register_t;


/** IOCTL structure for sending commands via the message FIFO */
typedef struct
{
    #define SXPF_CMD_FILE_EL_MAX_ARGS_LENGTH    SXPF_NUM_ARGS_PER_CMD
    __u32   timeout_ms; /**< maximum time to wait for response */
    __u32   cmd_stat;   /**< in: command code, out: result status */
    __u32   num_args;   /**< number of 8bit arguments */
    __u32   num_ret;    /**< number of 8bit return values */
    __u8    args[SXPF_CMD_FILE_EL_MAX_ARGS_LENGTH];   /**< command arguments */

} sxpf_cmd_fifo_el_t;


/** IOCTL structure for storing a user-defined INI sequence in the card's RAM */
typedef struct
{
    __u32   ram_offset;     /**< Offset from the start of the card's user
                             *   sequence RAM */
    __u32   seq_size;       /**< Size in bytes of the new user sequence */
    __u8    sequence[128];  /**< Binary INI sequence data */

} sxpf_user_sequence_t;


/** IOCTL structure used to enable/disable the automatic execution of
 *  user-defined INI sequences triggered by the reception of a specific
 *  scanline by the video system.
 */
typedef struct
{
    __u32   channel;    /**< Channel for which to set the auto-sequence: 0..3 */
    __u32   repeat;     /**< Flag: 0=single-shot; 1=repeat for each frame */
    __u32   ram_offset; /**< Start offset of the sequence in the user RAM */
    int32_t scanline;   /**< Scanline number that triggers the INI sequence.
                         *   If this value is -1, the automatic execution is
                         *   disabled.
                         */
} sxpf_enable_user_sequence_t;


/** IOCTL structure used to get the contents of a buffer's header area copied
 *  directly after it was received from the FPGA.
 */
typedef struct
{
    __u32   slot;       /**< IN: Buffer slot number to query. */
    __u8    header[64]; /**< OUT: Copied contents of the buffer's header. */

} sxpf_get_buf_hdr_t;


/** enumeration of known DMA stream identifiers */
typedef enum
{
    SXPF_IO_NORMAL    = 1,  /**< Normal function */
    SXPF_IO_FROZEN    = 2,  /**< Frozen after non-correctable PCIe error */
    SXPF_IO_FAILURE   = 3,  /**< Permanently disabled after fatal PCIe error */

} sxpf_io_state_t;


/** Event types delivered by the sxpf device's read function. */
typedef enum
{
    /** New image frame captured
     * @param data      ((sxpf_buf_t)slot << 24) + rx_size
     * @param extra     unsued
     * @note If the image is bigger than 16MB, rx_size will be set to 1. In
     *       this case only the image header contains valid size data.
     */
    SXPF_EVENT_FRAME_RECEIVED   = (1),

    /** Vertical blanking start
     * @param data      OR of (1 << channel)
     * @param extra     unsued
     */
    SXPF_EVENT_VSYNC            = (2),

    /** RX error
     * @param data      @see sxpf_capture_error_t
     * @param extra     unsued
     */
    SXPF_EVENT_CAPTURE_ERROR    = (3),

    /** TX error
     * @param data      @see sxpf_replay_error_t
     * @param extra     unsued
     */
    SXPF_EVENT_REPLAY_ERROR     = (4),

    /** Trigger sent
     * @param data              OR combination of SXPF_EVENT_DATA_TRIG_CHAN0 ...
     *                          SXPF_EVENT_DATA_TRIG_CHAN3 (see below)
     * @param extra.timestamp   Latched timestamp of HW trigger event
     */
    SXPF_EVENT_TRIGGER          = (5),

    #define SXPF_EVENT_DATA_TRIG_CHAN0  (1 << 0) /**< Trigger sent on chan 0 */
    #define SXPF_EVENT_DATA_TRIG_CHAN1  (1 << 1) /**< Trigger sent on chan 1 */
    #define SXPF_EVENT_DATA_TRIG_CHAN2  (1 << 2) /**< Trigger sent on chan 2 */
    #define SXPF_EVENT_DATA_TRIG_CHAN3  (1 << 3) /**< Trigger sent on chan 3 */

    /** Test interrupt
     * @param data  unused
     * @param extra unused
     */
    SXPF_EVENT_SW_TEST          = (6),

    /** Information about channel/adapter events
     * @param data              event data
     * @param extra.timestamp   HW timestamp at interrupt time
     */
    SXPF_EVENT_CHANNEL_INFO     = (7),

    /** Information about occured PCIe errors
     *
     * After receiving an event signifying a non-normal I/O state (i.e.,
     * data==2, 3 or 4 the device should not be used anymore, until normal
     * operation is signaled again with data==1 - which might never happen,
     * depending on the severity of the hardware error.
     *
     * @param data  @see enumeration sxpf_io_state_t
     * @param extra unused
     */
    SXPF_EVENT_IO_STATE         = (8),

    /** New I2C message captured.
     *
     * The actual I2C message data is appended to this event verbatim
     * @param data      ((sxpf_buf_t)slot << 24) + rx_size
     * @param extra     unsued
     * @note I2C messages are limited in size to 1024 bytes, including the
     *       64byte header.
     */
    SXPF_EVENT_I2C_MSG_RECEIVED = (9),

    /** New I2C slave event occured.
     *
     * One of the 8 internal I2C slaves triggers this event and the user
     * application must implement the neccesary bahavior.
     */
    SXPF_EVENT_I2C_SLAVE        = (10),

  //SXPF_EVENT_SIGNAL_CHANGED   = (4),  /**< Input signal changed state */

} sxpf_event_type_t;


/** Event structure that is delivered by the \ref sxpf_read_event() function. */
typedef struct
{
    __u32       type;   /**< Event type. @see sxpf_event_type_t */
    __u32       data;   /**< Type-dependent event data */

    // depending on type, this member delivers additional information relevant
    // for the event
    union
    {
        __u64   unused;     /**< only used to initialize event 'no extra info' */
        __u64   timestamp;  /**< latched timestamp */

    }           extra;

} sxpf_event_t;


/** Flash information, reconstructed by the flash ID */
typedef struct
{
    __u32 id;
    __u32 flashSize;
    __u32 sectorSize;
    __u32 pageSize;
} sxpf_flash_info_t;


/** System clocks that can be synced to the hardware timestamp */
typedef enum
{
    SXPF_CLOCK_REALTIME      = 0,   /**< use POSIX CLOCK_REALTIME */
    SXPF_CLOCK_MONOTONIC     = 1,   /**< use POSIX CLOCK_MONOTONIC */
    SXPF_CLOCK_MONOTONIC_RAW = 4,   /**< use POSIX CLOCK_MONOTONIC_RAW */

    SXPF_CLOCK_QPC           = 16,  /**< use QueryPerformanceCounter() */
    SXPF_CLOCK_FILETIME      = 17,  /**< use GetSystemTimePreciseAsFileTime() */

    SXPF_SYSTEM_CLOCK_MASK   = 0xff,/**< bits used for system clock selection */

    /** optionally ORed to system clock selection to sync to secondary hardware
     *  timestamp (e.g., GPS time)
     */
    SXPF_CLOCK_HW_SECONDARY  = (1 << 16),

#if defined(_WIN32)
    SXPF_CLOCK_DEFAULT = SXPF_CLOCK_QPC
#else
    SXPF_CLOCK_DEFAULT = SXPF_CLOCK_MONOTONIC_RAW
#endif
} sxpf_clock_select_t;


/** timestamp and system time struct */
typedef struct
{
    __s64       timestamp;      /**< Hardware timestamp */
    __s64       systemTime;     /**< System time */
    __s64       slack;          /**< duration of taking the HW timestamp in the
                                 *   selected clock's domain */
    __u32       clockSelect;    /**< Mode to get system time,
                                     @see sxpf_clock_select_t */
} sxpf_timestamp_sync_t;


/** Supported methods of user-space DMA memory allocation */
typedef enum
{
    SXPF_BUF_UNUSED     = 0,    /**< internal flag - not for user code */
    SXPF_ALLOC_DRIVER   = 1,    /**< memory allocated by driver (default) */
    SXPF_ALLOC_HEAP     = 2,    /**< memory coming from user heap/free store */
    SXPF_ALLOC_NV_DVP   = 3,    /**< memory comes from GPUDirect for Video */
    SXPF_ALLOC_NV_P2P   = 4,    /**< memory comes from cudaMalloc() */

} sxpf_buf_alloc_type_t;


/** announce a user-allocated video DMA buffer to the driver */
typedef struct
{
    __u32   slot;    /**< Used buffer slot, returned by driver */
    __u32   type;    /**< Type of allocation, @see sxpf_user_alloc_type_t */
    __u64   header;  /**< User-space header address value */
    __u64   payload; /**< User-space payload address value, depending on type */
    __u32   size;    /**< buffer size, must be at least driver's frame_size */

} sxpf_user_buffer_declare_t;


/** set up a new I2C slave device */
typedef struct
{
    __u32   channel;    /**< I2C bus to which to connect the slave */
    __u8    dev_id;     /**< I2C device ID (8-bit notation!) */
    __u8    handle;     /**< return value: device handle */

} sxpf_i2c_slave_init_t;


/** provide I2C slave device with TX data and acknowledment info */
typedef struct
{
    __u32   handle;     /**< return value: device handle */
    __u32   ack_data;   /**< acknowledge info */
    __u32   tx_size;    /**< number of bytes to send */
    __u8    tx_data[16];/**< return value: device handle */

} sxpf_i2c_slave_tx_ack_t;


#if defined(_MSC_VER) || (!defined(_MSC_VER) && !defined(KERNEL))

/***********************************************************************************/
/*      SXPF tree data structures for storing information of connected cards       */
/***********************************************************************************/

typedef struct
{
    int year;                               /**< year */
    int month;                              /**< month */
    int day;                                /**< day */
} sxpf_date_t;


typedef struct
{
    const char          *cl_name;           /**< SX-pF-cam-adapt crosslink name */
    int                  cl_type;           /**< SX-pF-cam-adapt crosslink type */
    int                  cl_version;        /**< SX-pF-cam-adapt crosslink vers */
    sxpf_date_t          cl_creat_date;     /**< SX-pF-cam-adapt crosslink date */
    uint8_t              cl_build_num;      /**< SX-pF-cam-adapt cl build num   */
} sxpf_cl_info_t;


typedef struct
{
    const char          *name;              /**< SX-proFRAME-camera-adapter name */
    int                  type;              /**< SX-proFRAME-camera-adapter type */
    int                  version;           /**< SX-proFRAME-camera-adapter vers */
    uint8_t              machxo_build_num;  /**< MachXO Build number             */
    sxpf_date_t          machxo_creat_date; /**< MachXO creation date            */
    sxpf_cl_info_t       cl_info;           /**< SX-pF-cam-adapt crosslink info  */
} sxpf_adapt_info_t;


typedef struct
{
    float               fpga_temp_celsius;  /* SX-proFRAME fpga temperature in C */
} sxpf_card_status_t;


typedef struct
{
    const char*         style;              /**< SX-proFRAME-card usage mode    */
    const char*         model;              /**< SX-proFRAME-card name          */
    char                fw_version[16];     /**< SX-proFRAME-card fw version    */
    sxpf_date_t         fw_build_date;      /**< SX-proFRAME-fw build date      */
    uint32_t            plasma_version;     /**< SX-proFRAME-card plasma        */
    uint32_t            card_generation;    /**< SX-proFRAME-card generattion   */
    int                 is_scatter_gather;  /**< SX-proFRAME-check var s.gather */
    uint32_t            device_dna[3];      /**< SX-proFRAME-card device dna    */

    sxpf_adapt_info_t   adapters[4U];     /**< SX-proFRAME-single adapt struct */
    sxpf_card_props_t   properties;       /**< SX-proFRAME-card propers struct */
    sxpf_card_status_t  status;           /**< SX-proFRAME-card status struct  */

} sxpf_card_info_t ;


typedef struct
{
    uint32_t            sxpf_revision;    /**< SX-proFRAME-sxpf-lib revision    */
    uint32_t            driver_revision;  /**< SX-proFRAME-sxpf-driver revision */
    int                 number_of_cards;  /**< SX-proFRAME-card found in syst   */

    sxpf_card_info_t    cards[15];
} cards_info_container_t;


typedef struct
{
    int                 eeprom_compatible; /**< INI contains only commands the
                                            *   plasma can execute */
    int                 spi_compatible;    /**< INI contains only commands the
                                            *   plasma can execute */
} sxpf_ini_props_t;

#endif /* defined(_MSC_VER) || (!defined(_MSC_VER) && !defined(KERNEL)) */

#endif /* SXPFTYPES_H_ */
