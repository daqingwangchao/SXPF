#ifndef SXPF_REGS_H_
#define SXPF_REGS_H_

#include "sxpftypes.h"

#define SXPF_FIRMWARE_NAME      "sxpf.fw"
#define SXPF_FIRMWARE_NAME_G2   "sxpf_g2.fw"
#define SXPF_FIRMWARE_NAME_G3   "sxpf_g3.fw"

/* defines */
#define PCI_VENDOR_ID_PLDA      (0x1556) /* missing in Linux' PCI vendor list */
#define PCI_DEVICE_ID_PROFRAME  (0x5555)


/** Plasma control registers are re-mapped into this virtual BAR region */
#define PLASMA_REGION           (1)

#define SXPF_NUM_ARGS_TOTAL      (512)   /**< Parameter RAM has 2KByte */
#define SXPF_NUM_BYTES_PER_CMD   (SXPF_NUM_ARGS_PER_CMD * 1)
#define SXPF_NUM_CMD_ARG_SLOTS   (SXPF_NUM_ARGS_TOTAL / SXPF_NUM_ARGS_PER_CMD)


/** System register adresses in BAR0 */
enum BAR0_REGISTERS_e
{
    REG_NAME            = 0x0000,
#define CARD_TYPE_SXPCIEX4      (0x4148494c)    /* "AHIL": first-gen card */
#define CARD_TYPE_SXPCIEX4_G2   (0x48494c32)    /* "HIL2": second-gen card */
#define CARD_TYPE_SXPCIEX8_G3   (0x48494c33)    /* "HIL3": third-gen card */
    REG_DATE            = 0x0004,
    REG_VERSION         = 0x0008,
#define SXPF_FW_VERSION_CODE(major, minor, build) \
    (((major) << 24) | ((minor) << 16) | (build))
    REG_SCRATCH         = 0x000c,
    REG_ACTIVITY_FLAG   = 0x0010,       /**< scratch reg: persistent run flag */

    REG_TIMER           = 0x0040,       /**< free-running timer value */
    REG_CONTROL         = 0x0044,
#define CTRL_RUN                    (1 << 0)
#define CTRL_MODE_RECORD            (0)
#define CTRL_MODE_PLAYBACK          (1 << 1)
#define CTRL_VCHAN_ENABLE(ch)       (1 << ((ch) + 2))
#define CTRL_VCHAN_ENABLE_ALL4      (15 << 2)   /* gen1/2: 4ch; gen3: 8ch */
#define CTRL_VCHAN_ENABLE_ALL       (255 << 2)  /* gen1/2: 4ch; gen3: 8ch */
#define CTRL_SOFT_IRQ               (1 << 10)   /* gen3 and newer */
#define CTRL_I2C_2_ENABLE           (1 << 11)
#define CTRL_I2C_3_ENABLE           (1 << 12)
#define CTRL_VCHAN_NBYPASS(ch)      (1 << ((ch) + 13))
#define CTRL_VCHAN_NBYPASS_ALL      (15 << 13)
#define CTRL_SOFT_RESET             (1 << 17)
#define CTRL_SOFT_IRQ_LEGACY        (1 << 20)   /* gen1/2 */
#define CTRL_I2C_CHAN_ENABLE(ch)    (1 << ((ch) + 20))
#define CTRL_I2C_CHAN_ENABLE_ALL4   (15 << 20)  /* gen1/2: 4ch; gen3: 8ch */
#define CTRL_I2C_CHAN_ENABLE_ALL    (255 << 20) /* gen1/2: 4ch; gen3: 8ch */
#define CTRL_EXT_TRIG_SOURCE        (1 << 28)
#define CTRL_PLDA_SG_ENABLE         (1 << 29)   /* gen 1 only */
#define CTRL_UNIX_TS_SYNC_SEL       (1 << 29)   /* gen 2 and newer */

    REG_STATUS          = 0x0048,
#define STAT_MEM0_CALIB_DONE        (1 << 0)
#define STAT_MEM1_CALIB_DONE        (1 << 1)
#define STAT_CAM0_DES_CLOCK_LOCKED  (1 << 2)
#define STAT_CAM1_DES_CLOCK_LOCKED  (1 << 3)
#define STAT_CAM2_DES_CLOCK_LOCKED  (1 << 4)
#define STAT_CAM3_DES_CLOCK_LOCKED  (1 << 5)
#define STAT_EXT_SYNC_LOCKED        (1 << 6)
#define STAT_IRQ                    (1 << 7)
#define STAT_PCLK0_OK               (1 << 8)
//#define STAT_PCLK1_OK               (1 << 9)
//#define STAT_PCLK2_OK               (1 << 10)
//#define STAT_PCLK3_OK               (1 << 11)
#define STAT_CAM0_INIT_DONE         (1 << 12)
#define STAT_CAM1_INIT_DONE         (1 << 13)
#define STAT_CAM2_INIT_DONE         (1 << 14)
#define STAT_CAM3_INIT_DONE         (1 << 15)
#define STAT_CAM_ALL_INIT_DONE  \
    (STAT_CAM0_INIT_DONE | STAT_CAM1_INIT_DONE | STAT_CAM2_INIT_DONE \
     STAT_CAM3_INIT_DONE)
#define STAT_CAM_INIT_COMPLETE      (1 << 16)
#define STAT_CAM0_INIT_ERROR        (1 << 17)
#define STAT_CAM1_INIT_ERROR        (1 << 18)
#define STAT_CAM2_INIT_ERROR        (1 << 19)
#define STAT_CAM3_INIT_ERROR        (1 << 20)

    REG_SYSTEM_ID       = 0x004c,
    REG_ERROR           = 0x0050,
    REG_ERROR_STORE     = 0x0054,
    REG_TIMESTAMP_LO    = 0x0058,
    REG_TIMESTAMP_HI    = 0x005c,
    REG_ERROR_ENABLE    = 0x0060,
    REG_ERROR_CRITICAL  = 0x0064,
    REG_ERROR_SINGLE    = 0x0068,

    REG_TRACE_CONTROL   = 0x0070,
#define TRACE_CTRL_DISABLE          (0 << 0)
#define TRACE_CTRL_ENABLE           (1 << 0)
    REG_TRACE_ADDR_LO   = 0x0074,
    REG_TRACE_ADDR_HI   = 0x0078,
    REG_TIME_80MHZ      = 0x007c,

    REG_TIMESTAMP_SEC_LO= 0x0080,
    REG_TIMESTAMP_SEC_HI= 0x0084,

    REG_TIMESTAMP_CONTROL = 0x0A0,  /**< Timestamp control */
#define TIMESTAMP_CONTROL_ENABLE         (1 << 0)
#define TIMESTAMP_CONTROL_TX             (1 << 1)
#define TIMESTAMP_CONTROL_RX             (1 << 2)
#define TIMESTAMP_CONTROL_BYPASS         (1 << 3)
#define TIMESTAMP_CONTROL_POLARITY       (1 << 4)
#define TIMESTAMP_CONTROL_UNIT_ERROR     (1 << 8)
#define TIMESTAMP_CONTROL_ENCODING_DONE  (1 << 10)
#define TIMESTAMP_CONTROL_ENCODER_IDLE   (1 << 11)
#define TIMESTAMP_CONTROL_ENCODER_ERROR  (1 << 12)
#define TIMESTAMP_CONTROL_DECODING_DONE  (1 << 13)
#define TIMESTAMP_CONTROL_DECODER_IDLE   (1 << 14)
#define TIMESTAMP_CONTROL_DECODER_ERROR  (1 << 15)
    REG_TIMESTAMP_LO_SAVED = 0x00A4, /**< Timestamp low saved when slot 0 was triggered */
    REG_TIMESTAMP_HI_SAVED = 0x00A8, /**< Timestamp high saved when slot 0 was triggered */
    REG_TIMESTAMP_OFFSET   = 0x00AC, /**< Timestamp offset */

    REG_SCANLINE_TRIG0  = 0x00c0,   /**< Trigger scanline for channel 0 */
    REG_SCANLINE_TRIG1  = 0x00c4,   /**< Trigger scanline for channel 1 */
    REG_SCANLINE_TRIG2  = 0x00c8,   /**< Trigger scanline for channel 2 */
    REG_SCANLINE_TRIG3  = 0x00cc,   /**< Trigger scanline for channel 3 */
#define REG_SCANLINE_TRIG(chan) (REG_SCANLINE_TRIG0 + (chan) * 4)

    REG_FEATURE0        = 0x00d0,       /**< Feature flag register 0 */
#define REG_FEATURE(num)        (REG_FEATURE0 + 4*(num))
// The list of feature bit positions follows:
#define SXPF_FEATURE_SG_PLDA    (0) /**< PLDA scatter-gather supported */
#define SXPF_FEATURE_FB_ASSIGN  (1) /**< FW flag: Selectable camera source for each frame buffer */
#define SXPF_FEATURE_EYE_SCAN   (2) /**< FW flag: In-system IBERT Eye Scan */
#define SXPF_FEATURE_SG_XDMA    (3) /**< XDMA scatter-gather supported */
#define SXPF_FEATURE_I2C_RECORD (4) /**< Card can record I2C messages */
#define SXPF_FEATURE_IND_TRIG   (5) /**< Independent trigger modules */
#define SXPF_FEATURE_GENERIC_SG (6) /**< Gen3: Generic scatter/gather (separate header, arbitrary payload) */
#define SXPF_FEATURE_I2C_SLAVES (7) /**< Card has 8 I2C slave cores */

    REG_IRQ_STATUS      = 0x0100,
    REG_IRQ_MASK        = 0x0104,
#define IRQ_STAT_VIDEO_DMA  (1 << 0)    /**< Video DMA finished */
#define IRQ_STAT_META_DMA   (1 << 1)    /**< I2C/Ethernet DMA finished */
#define IRQ_STAT_ERROR      (1 << 2)    /**< Error occured */
#define IRQ_STAT_TEST       (1 << 3)    /**< SW-triggered IRQ for testing */
#define IRQ_STAT_TRACE_BUF  (1 << 4)    /**< Trace information buffer ready */
#define IRQ_STAT_BAR2       (1 << 5)    /**< Cascaded IRQ from BAR2 modules */
#define IRQ_STAT_EVENT      (1 << 6)    /**< Entry in event FIFO from Plasma */
#define IRQ_STAT_TRIGGER    (15 << 8)   /**< Trigger was sent (record only) */
#define IRQ_STAT_I2CSLAVE   (1 << 12)   /**< I2c Slave IRQ occured */
#define IRQ_MASK_DEFAULT    \
    (IRQ_STAT_VIDEO_DMA | IRQ_STAT_META_DMA | IRQ_STAT_ERROR | IRQ_STAT_TEST | \
     IRQ_STAT_BAR2 | IRQ_STAT_EVENT | IRQ_STAT_TRIGGER | IRQ_STAT_I2CSLAVE)

    REG_VCMD_ADDR_LO    = 0x0200,
    REG_VCMD_ADDR_HI    = 0x0204,
    REG_VCMD_SIZE       = 0x0208,
    REG_MCMD_ADDR_LO    = 0x0210,
    REG_MCMD_ADDR_HI    = 0x0214,
    REG_MCMD_SIZE       = 0x0218,
    REG_VFIN_ADDR_LO    = 0x0220,
    REG_VFIN_ADDR_HI    = 0x0224,
    REG_VFIN_SIZE       = 0x0228,
    REG_MFIN_ADDR_LO    = 0x0230,
    REG_MFIN_ADDR_HI    = 0x0234,
    REG_MFIN_SIZE       = 0x0238,
    REG_FIFO_STATUS     = 0x0250,
#define FIFO_STAT_VCMD_FULL     (1 << 0)
#define FIFO_STAT_VCMD_EMPTY    (1 << 1)
#define FIFO_STAT_MCMD_FULL     (1 << 2)
#define FIFO_STAT_MCMD_EMPTY    (1 << 3)
#define FIFO_STAT_VFIN_FULL     (1 << 4)
#define FIFO_STAT_VFIN_EMPTY    (1 << 5)
#define FIFO_STAT_MFIN_FULL     (1 << 6)
#define FIFO_STAT_MFIN_EMPTY    (1 << 7)
    REG_FIFO_CONTROL    = 0x0254,
#define FIFO_CTRL_NOTIFY_USER   (0 << 0)
#define FIFO_CTRL_AUTO_COMPLETE (1 << 0)

    /* global trigger config registers (re-used for channel 0 with
     * SXPF_FEATURE_IND_TRIG) */
    REG_TRIG_CONTROL    = 0x0600,   /**< Capture trigger control */
    REG_TRIG_PERIOD     = 0x0604,   /**< Capture trigger period */
    REG_TRIG_LENGTH     = 0x0608,   /**< Capture trigger pulse length */
    REG_TRIG_DELAY      = 0x060c,   /**< Capture trigger pulse delay */
    REG_TRIG_EXT_LENGTH = 0x0614,   /**< Trigger output pulse length */
    REG_TRIG_OUT_DIV    = 0x0620,   /**< Trigger output frequency divider */
    REG_TRIG_IN_MULT    = 0x0628,   /**< Trigger input frequency multiplier */
    REG_TRIG_USER_DIV   = 0x062c,   /**< Trigger user divisor
                                         = 1 / TRIGG_IN_MULT * 2^31
                                         UQ1.31 notation,
                                         valid range [0.0, 1.0] */
    REG_SECONDARY_INPUT = 0x0630,   /**< Secondary external trig.\ input mask */
#define EXT_TRIG_SEL_MASK   (0x0f)  /**< mask bits selecting EXT_TRIG_IN_1 pins */
#define EXT_TRIG_CAM_MASK   ((1 << 12) | (3 << 8)) /**< mask bits selecting EXT_TRIG_IN_0 or EXT_TRIG_CAM */
#define EXT_TRIG_CAM_EN     (1 << 12)   /**< enable camera as trigger source */
#define EXT_TRIG_CAM_SEL(n) ((n) << 8)  /**< camera trigger selection (0..3) */

/** trigger module base addresss with feature SXPF_FEATURE_IND_TRIG enabled */
#define REG_TRIG_BASE_CH(ch) (0x0700 + 0x40 * (ch))

    REG_TSTAMP_TRIG0_LO = 0x0680, /**< Latched timestamp[31:0] on trigger at slot 0 */
    REG_TSTAMP_TRIG0_HI = 0x0684, /**< Latched timestamp[63:32] on trigger at slot 0 */
    REG_TSTAMP_TRIG1_LO = 0x0688, /**< Latched timestamp[31:0] on trigger at slot 1 */
    REG_TSTAMP_TRIG1_HI = 0x068c, /**< Latched timestamp[63:32] on trigger at slot 1 */
    REG_TSTAMP_TRIG2_LO = 0x0690, /**< Latched timestamp[31:0] on trigger at slot 2 */
    REG_TSTAMP_TRIG2_HI = 0x0694, /**< Latched timestamp[63:32] on trigger at slot 2 */
    REG_TSTAMP_TRIG3_LO = 0x0698, /**< Latched timestamp[31:0] on trigger at slot 3 */
    REG_TSTAMP_TRIG3_HI = 0x069c, /**< Latched timestamp[63:32] on trigger at slot 3 */
#define REG_TSTAMP_TRIG_LO(chan)    (REG_TSTAMP_TRIG0_LO + 8 * (chan))
#define REG_TSTAMP_TRIG_HI(chan)    (REG_TSTAMP_TRIG0_HI + 8 * (chan))

    REG_TSTAMP_SEC_TRIG0_LO = 0x06c0, /**< Latched secondary timestamp[31:0] on trigger at slot 0 */
    REG_TSTAMP_SEC_TRIG0_HI = 0x06c4, /**< Latched secondary timestamp[63:32] on trigger at slot 0 */
    REG_TSTAMP_SEC_TRIG1_LO = 0x06c8, /**< Latched secondary timestamp[31:0] on trigger at slot 1 */
    REG_TSTAMP_SEC_TRIG1_HI = 0x06cc, /**< Latched secondary timestamp[63:32] on trigger at slot 1 */
    REG_TSTAMP_SEC_TRIG2_LO = 0x06d0, /**< Latched secondary timestamp[31:0] on trigger at slot 2 */
    REG_TSTAMP_SEC_TRIG2_HI = 0x06d4, /**< Latched secondary timestamp[63:32] on trigger at slot 2 */
    REG_TSTAMP_SEC_TRIG3_LO = 0x06d8, /**< Latched secondary timestamp[31:0] on trigger at slot 3 */
    REG_TSTAMP_SEC_TRIG3_HI = 0x06dc, /**< Latched secondary timestamp[63:32] on trigger at slot 3 */
#define REG_TSTAMP_SEC_TRIG_LO(chan)    (REG_TSTAMP_SEC_TRIG0_LO + 8 * (chan))
#define REG_TSTAMP_SEC_TRIG_HI(chan)    (REG_TSTAMP_SEC_TRIG0_HI + 8 * (chan))

    REG_VIDEO_LEVEL0    = 0x0800,   /**< No.\ of video frames in mem bank 0 */
    REG_META_LEVEL0     = 0x0804,   /**< No.\ of meta frames in mem bank 0 */
    REG_VIDEO_LEVEL1    = 0x0808,   /**< No.\ of video frames in mem bank 1 */
    REG_META_LEVEL1     = 0x080C,   /**< No.\ of meta frames in mem bank 1 */
    REG_GPIO_STATUS     = 0x0820,   /**< GPIO status r/w */
    REG_GPIO_SET        = 0x0824,   /**< Write 1 to set REG_GPIO_STATUS bit */
    REG_GPIO_CLEAR      = 0x0828,   /**< Write 1 to clear REG_GPIO_STATUS bit */

    REG_DEBUG_TRIGGER   = 0x81C,    /**< Current signal level of trigger pins */
#define DBG_TRIG_CAM_OUT0     (1 << 0)  /**< Camera Adapter Trigger Output at slot 0 */
#define DBG_TRIG_CAM_OUT1     (1 << 1)  /**< Camera Adapter Trigger Output at slot 1 */
#define DBG_TRIG_CAM_OUT2     (1 << 2)  /**< Camera Adapter Trigger Output at slot 2 */
#define DBG_TRIG_CAM_OUT3     (1 << 3)  /**< Camera Adapter Trigger Output at slot 3 */
#define DBG_TRIG_CAM_EXT_IN0  (1 << 16) /**< Camera Adapter External Trigger Input at slot 0 */
#define DBG_TRIG_CAM_EXT_IN1  (1 << 17) /**< Camera Adapter External Trigger Input at slot 1 */
#define DBG_TRIG_CAM_EXT_IN2  (1 << 18) /**< Camera Adapter External Trigger Input at slot 2 */
#define DBG_TRIG_CAM_EXT_IN3  (1 << 19) /**< Camera Adapter External Trigger Input at slot 3 */
#define DBG_TRIG_EXT_IN0      (1 << 24) /**< External Trigger Input 0 */
#define DBG_TRIG_EXT_IN1      (1 << 25) /**< External Trigger Input 1 */
};


/** System register adresses in BAR1 */
enum BAR1_REGISTERS_e
{
    REG_I2C_SLAVE_0_BASE = 0x8000,          /**< i2c slave core 0 */
    REG_I2C_SLAVE_1_BASE = 0x8200,          /**< i2c slave core 1 */
    REG_I2C_SLAVE_2_BASE = 0x8400,          /**< i2c slave core 2 */
    REG_I2C_SLAVE_3_BASE = 0x8600,          /**< i2c slave core 3 */
    REG_I2C_SLAVE_4_BASE = 0x8800,          /**< i2c slave core 4 */
    REG_I2C_SLAVE_5_BASE = 0x8A00,          /**< i2c slave core 5 */
    REG_I2C_SLAVE_6_BASE = 0x8C00,          /**< i2c slave core 6 */
    REG_I2C_SLAVE_7_BASE = 0x8E00,          /**< i2c slave core 7 */

    REG_I2C_SLAVE_DGIER_OFFSET = 0x1C,      /**< Global Interrupt Enable Register */
    REG_I2C_SLAVE_IISR_OFFSET = 0x20,       /**< Interrupt Status Register */
    REG_I2C_SLAVE_IIER_OFFSET = 0x28,       /**< Interrupt Enable Register */
    REG_I2C_SLAVE_CR_REG_OFFSET = 0x100,    /**< Control Register */
    REG_I2C_SLAVE_SR_REG_OFFSET = 0x104,    /**< Status Register */
    REG_I2C_SLAVE_DTR_REG_OFFSET = 0x108,   /**< Data Tx Register */
    REG_I2C_SLAVE_DRR_REG_OFFSET = 0x10C,   /**< Data Rx Register */
    REG_I2C_SLAVE_ADR_REG_OFFSET = 0x110,   /**< Address Register */
    REG_I2C_SLAVE_TFO_REG_OFFSET = 0x114,   /**< Tx FIFO Occupancy */
    REG_I2C_SLAVE_RFO_REG_OFFSET = 0x118,   /**< Rx FIFO Occupancy */
    REG_I2C_SLAVE_RFD_REG_OFFSET = 0x120,   /**< Rx FIFO Depth reg */
    REG_I2C_SLAVE_GPO_REG_OFFSET = 0x124,   /**< Output Register */
};


/** System register adresses in BAR2 */
enum BAR2_REGISTERS_e
{
    REG_PLASMA_CTRL     = 0x0010,       /**< Plasma Reset in bit 0 */
#define PLASMA_CTRL_RUN             (0)
#define PLASMA_CTRL_RESET           (1 << 0)

    REG_SYSTEM_INFO     = 0x0014,
#define SYSTEM_INFO_SW_RESET                      (1 << 0) /* POR always done */
#define SYSTEM_INFO_RESERVED1                     (1 << 1)
#define SYSTEM_INFO_DISABLE_STATE_MACHINE_LOGS    (1 << 2)
#define SYSTEM_INFO_EMULATE_DUAL_ADAPTER          (1 << 3)
#define SYSTEM_INFO_DISABLE_INIT_SEQUENCES        (1 << 4)
#define SYSTEM_INFO_DISABLE_EVENT_HANDLING        (1 << 5)
#define SYSTEM_INFO_I2C_BUS_SCAN_AT_STARTUP       (1 << 6)
#define SYSTEM_INFO_FACTORY_TEST_MODE             (1 << 7)
#define SYSTEM_INFO_INVALID_EEPROM_MASK           (0xFF << 8)
#define SYSTEM_INFO_INIT_SEQ_RUNNING_MASK         (0xFF << 16)
#define SYSTEM_INFO_INIT_SEQ_FAILED_MASK          (0xFF << 24)
#define SYSTEM_INFO_INVALID_EEPROM(chan)          ((0xFF & (1<<(chan))) << 8)
#define SYSTEM_INFO_INIT_SEQ_RUNNING(chan)        ((0xFF & (1<<(chan))) << 16)
#define SYSTEM_INFO_INIT_SEQ_FAILED(chan)         ((0xFF & (1<<(chan))) << 24)

    REG_DEVICE_DNA0     = 0x80,
    REG_DEVICE_DNA1     = 0x84,
    REG_DEVICE_DNA2     = 0x88,

    REG_REQ_FIFO_WR     = 0x0800,
    REG_REQ_FIFO_RD     = 0x0804,
    REG_RESP_FIFO_WR    = 0x0808,
    REG_RESP_FIFO_RD    = 0x080c,
    REG_MSG_FIFO_STAT_H = 0x0810,       /**< FIFO status for Host */
    REG_MSG_FIFO_STAT_M = 0x0814,       /**< FIFO status for Plasma */
#define MSG_FIFO_FILL_LEVEL(stat)   ((stat) & 0xffff)
#define MSG_FIFO_FREE_LEVEL(stat)   (((stat) >> 16) & 0xffff)
    REG_REQ_FIFO_CTRL   = 0x0818,
    REG_RESP_FIFO_CTRL  = 0x081c,
#define FIFO_CTRL_RESET     (1 << 0)    /**< Clear the FIFO */

    REG_EVT_FIFO_WR     = 0x0c08,
    REG_EVT_FIFO_RD     = 0x0c0c,
    REG_EVT_FIFO_STAT_H = 0x0c10,       /**< FIFO status for Host */
    REG_EVT_FIFO_STAT_M = 0x0c14,       /**< FIFO status for Plasma */
    REG_EVT_FIFO_CTRL   = 0x0c18,

    REG_IRQ_STATUS_HOST = 0x1008,
    REG_IRQ_MASK_HOST   = 0x100c,
#define IRQ_STAT_RESP_FIFO  (1 << 29)   /**< Event/Response FIFO not empty */
#define IRQ_STAT_REQ_FIFO   (1 << 28)   /**< Cmd.\ FIFO not empty; for Plasma */
#define IRQ_STAT_I2C_3      (1 << 27)   /**< I2C host #3 interrupt */
#define IRQ_STAT_I2C_2      (1 << 26)   /**< I2C host #2 interrupt */
#define IRQ_STAT_I2C_1      (1 << 25)   /**< I2C host #1 interrupt */
#define IRQ_STAT_I2C_0      (1 << 24)   /**< I2C host #0 interrupt */

    REG_ADDR_RAM_BASE   = 0x4000,   /**< 2KB FPGA RAM used for command args */

    /*
     * Special software-defined Plasma registers
     */

    /** vritual regs start here; smaller offsets are hardware */
    REG_SPECIAL_BASE_OFFSET = 0x100000,

    /** software timeout for a single I2C character transmission on specified
     * channel */
#define REG_I2C_CHARACTER_TIMEOUT(ch)   (REG_SPECIAL_BASE_OFFSET + 4 * (ch))
    REG_I2C_CHARACTER_TIMEOUT_CH0 = REG_SPECIAL_BASE_OFFSET + 0x00,
    REG_I2C_CHARACTER_TIMEOUT_CH1 = REG_SPECIAL_BASE_OFFSET + 0x04,
    REG_I2C_CHARACTER_TIMEOUT_CH2 = REG_SPECIAL_BASE_OFFSET + 0x08,
    REG_I2C_CHARACTER_TIMEOUT_CH3 = REG_SPECIAL_BASE_OFFSET + 0x0c,

    /** Command registers to enable/disable generation of START/STOP conditions
     *  on the I2C bus of a channel.
     *  This is used to break a single I2C transaction that is longer than what
     *  can be transfered with a single call to sxpf_i2c_xfer() into multiple
     *  smaller transfers.
     *  @see I2C_CONTROLLER_FLAGS_START_STOP,
     *       I2C_CONTROLLER_FLAGS_START_NO_STOP,
     *       I2C_CONTROLLER_FLAGS_NO_START_NO_STOP and
     *       I2C_CONTROLLER_FLAGS_STOP_NO_START
     *  Writing an other value than the above listed will result in normal
     *  operation with START and STOP sent for the following I2C transfers.
     */
#define REG_I2C_CONTROLLER_FLAGS(ch) (REG_SPECIAL_BASE_OFFSET + 0x10 + 4 * (ch))
    REG_I2C_CONTROLLER_FLAGS_CH0 = REG_SPECIAL_BASE_OFFSET + 0x10,
    REG_I2C_CONTROLLER_FLAGS_CH1 = REG_SPECIAL_BASE_OFFSET + 0x14,
    REG_I2C_CONTROLLER_FLAGS_CH2 = REG_SPECIAL_BASE_OFFSET + 0x18,
    REG_I2C_CONTROLLER_FLAGS_CH3 = REG_SPECIAL_BASE_OFFSET + 0x1c,

    /** normal operation with START & STOP in following I2C transfers */
#define I2C_CONTROLLER_FLAGS_START_STOP         (0)

    /** send START but no STOP (start of a long transfer) */
#define I2C_CONTROLLER_FLAGS_START_NO_STOP      (1)

    /** send neither START nor STOP (middle of a long transfer ) */
#define I2C_CONTROLLER_FLAGS_NO_START_NO_STOP   (2)

    /** send STOP but no START (end of a long transfer) */
#define I2C_CONTROLLER_FLAGS_STOP_NO_START      (3)

    /** Configure an I2C device address alias and optionally enable or disable
     *  a FuSa method for the given device.
     *  Use \ref I2C_TRANSLATION_CONFIG to obtain a valid value to write into
     *  these registers.
     *  @note: The grabber cards supports up to a maximum of 8 alias/FuSa
     *         defintions per channel.
     */
#define REG_CMD_I2C_TRANSLATION(ch)  (REG_SPECIAL_BASE_OFFSET + 0x20 + 4 * (ch))
    REG_CMD_I2C_TRANSLATION_CH0 = REG_SPECIAL_BASE_OFFSET + 0x20,
    REG_CMD_I2C_TRANSLATION_CH1 = REG_SPECIAL_BASE_OFFSET + 0x24,
    REG_CMD_I2C_TRANSLATION_CH2 = REG_SPECIAL_BASE_OFFSET + 0x28,
    REG_CMD_I2C_TRANSLATION_CH3 = REG_SPECIAL_BASE_OFFSET + 0x2c,

    /** @return A configuration word to write to a channel's I2C translation
     *          cmd register \ref REG_CMD_I2C_TRANSLATION(ch).
     * @param fusa_method   Functional safety method ID of the I2C device.
     *                      @see I2cFusaMethodId
     * @param alias_id      New alias to use for the device ID of the I2C device
     * @param dest_id       Actual hardware device ID of the I2C device.
     */
#define I2C_TRANSLATION_CONFIG(fusa_method, dest_id, alias_id) \
    ((fusa_method) << 24 | (alias_id) << 8 | (dest_id))
};


/** Valid method IDs for the \ref I2C_TRANSLATION_CONFIG() macro to enable or
 *  disable Functional Safety algorithms for a specific I2C device.
 */
enum I2cFusaMethodId
{
    I2C_FUSA_NONE     = 0,      /**< No FuSa method active for the device */
    I2C_FUSA_MAX9671x = 1,      /**< FuSa of the MAX9671x (de)serializers */
    I2C_FUSA_IMX490   = 2,      /**< FuSa of the SONY IMX490 image sensor */
    I2C_FUSA_S5K1H1   = 3,      /**< FuSa of the SAMSUNG S5K1H1 image sensor */
};


/** Valid modes a proFRAME card may be in */
typedef enum
{
    SXPF_IDLE       = 0,
    SXPF_CAPTURING  = CTRL_RUN | CTRL_MODE_RECORD,
    SXPF_REPLAYING  = CTRL_RUN | CTRL_MODE_PLAYBACK,
    SXPF_SHUTDOWN   = -1,

} sxpf_mode_t;


/** Layout of a scatter-gather descriptor for the PLDA ezDMA core */
typedef struct
{
    __u32   phys_addr_lo;   /**< bits [31:3] of address; bits [2:0] must be 0 */
    __u32   phys_addr_hi;   /**< bits [63:32] of address */
    __u32   chunk_size;     /**< number of bytes; must be a multiple of 8 for
                                 record and multiple of 256 for playback! */
    __u32   next_lo;        /**< low part of ptr to next sg_desc_plda;
                                 must be aligned to 32bit;
                                 must be 0x00000001 for the last chunk */
    __u32   next_hi;        /**< high part of ptr to next sg_desc_plda */

} sg_desc_plda;


/** Layout of a scatter-gather descriptor for the XDMA core from Xilinx */
typedef struct
{
    __u32   control;
    __u32   chunk_size;     /**< number of bytes; must be a multiple of 8 for
                                 record and multiple of 256 for playback! */
    __u64   src_addr;       /**< physical source addr; bits [2:0] must be 0 */
    __u64   dst_addr;       /**< physical dest addr; bits [2:0] must be 0 */

    __u64   next;           /**< physical addr of ptr to next sg_desc_xdma;
                                 must be aligned to multiple of 32 bytes */
} sg_desc_xdma;


#endif /* !defined SXPF_REGS_H_ */
