/**
 * @file sxpf_module.h
 *
 * @if (!DRIVER)
 * The driver's internal documentation is not included.
 * @else
 * The driver's private declarations.
 * @endif
 *
 * @cond DRIVER
 */
#ifndef SXPF_MODULE_H_
#define SXPF_MODULE_H_

#include <linux/cdev.h>
#include <linux/dmapool.h>
#include <linux/pci.h>
#include <linux/version.h>
#include "linux/scatterlist.h"
#include <linux/semaphore.h>

#include "sxpf.h"
#include "sxpf_regs.h"
#include "xvc_pcie_driver.h"

#ifdef ENABLE_NV_P2P
#ifdef NV_BUILD_DGPU
#define NV_PAGE_SHIFT       (16)
#else
#define NV_PAGE_SHIFT       (12)
#endif
#define NV_PAGE_SIZE        ((u64)1 << NV_PAGE_SHIFT)
#define NV_PAGE_OFFSET      (NV_PAGE_SIZE - 1)
#define NV_PAGE_MASK        (~NV_PAGE_OFFSET)
#endif

#define SXPF_MAX_DEVICES    (8)     /**< allow max. 8 cards */
#define SXPF_MAX_I2C_SLAVES (8)

/** Internal name of the device */
#define DEVICE_NAME         "sxpf"

#define SEND_VSYNC_EVENTS   (0)     /**< enable/disable VSYNC events */
#define POLL_INTERRUPT      (0)     /**< HW ISR when 0, timer ISR when 1 */

/* defines */
#define PCI_VENDOR_ID_PLDA      (0x1556) /* missing in Linux' PCI vendor list */
#define PCI_DEVICE_ID_PROFRAME  (0x5555)

/** Default DMA buffer size: 4Mbyte */
#define SXPF_DMA_DEF_SLOT_SIZE  (4 << 20)

#ifdef CONFIG_X86
#define ENABLE_XVC
#endif

/* Debug output level definitions.
*/
#define SXPF_ERROR          (0) /**< Errors */
#define SXPF_WARNING        (1) /**< Warnings */
#define SXPF_INFO           (2) /**< Informative message, printed by default */
#define SXPF_MESSAGE        (3) /**< Even more messages */
#define SXPF_DEBUG          (4) /**< Debug messages */
#define SXPF_DEBUG_EXTRA    (5) /**< Extra debug messages */

extern int sxpfDebugLevel;      /**< The current debug level 0-4 */
extern int sxpfDebugLevelIrq;   /**< The irq handler's current debug level */

/* Debug output macros */
#define CARD_PRINTF(debugLevel, fmt, args...)                               \
        do                                                                  \
        {                                                                   \
            if (sxpfDebugLevel >= (debugLevel))                             \
                printk("%s" DEVICE_NAME "%d: " fmt "\n",                    \
                       (debugLevel) == SXPF_ERROR ? KERN_ERR : KERN_NOTICE, \
                       card->nr, ## args);                                  \
        }                                                                   \
        while (false)

#define CARD_PRINTF_IRQ(debugLevel, fmt, args...)                           \
        do                                                                  \
        {                                                                   \
            if (sxpfDebugLevelIrq >= (debugLevel))                          \
                printk("%s" DEVICE_NAME "%d irq: " fmt "\n",                \
                       (debugLevel) == SXPF_ERROR ? KERN_ERR : KERN_NOTICE, \
                       card->nr, ## args);                                  \
        }                                                                   \
        while (false)


// forward declarations
typedef struct sxpf_pci_device_s    sxpf_card_t;
typedef struct sxpf_file_hdl_s      sxpf_file_hdl_t;


typedef struct buf_seg_s
{
    union
    {
        sg_desc_plda    plda;       /**< scatter list entry for Gen1 cards */
        sg_desc_xdma    xdma;       /**< scatter list entry for Gen2 cards */

    }                  *sg_entry;   /**< NULL if no SG used */
    dma_addr_t          sgd_bus_adr;/**< Physical PCIe bus address of SG desc */

    struct list_head    list;       /**< list of segments forming the buffer */
    dma_addr_t          bus_adr;    /**< Physical PCIe bus address */
    void*               kernel_adr; /**< kernel address */
    u32                 seg_size;   /**< buffer segment size */
} buffer_segment;


/** Physical memory buffer desccriptor */
struct  sxpf_buffer
{
    u32                 magic;      /**< magic number to check type */
#define SXPF_BUFFER_MAGIC       (0xB3287F9A)

    enum dma_data_direction     dma_dir;

    void (*dma_unmap)(struct sxpf_buffer *); /**< undo DMA mapping */

    dma_addr_t          bus_adr;    /**< Physical PCIe bus address of the buffer
                                         itself or its scatter list */
    void*               kernel_adr; /**< kernel address of the header segment */
    u32                 buf_size;   /**< buffer size */
    unsigned long       attr;       /**< Attrs to be used in dma_free_attrs */
    struct list_head    segments;   /**< segements, when scatter-gathering */

    sxpf_card_t        *card;       /**< owning card for work handler */
    sxpf_stream_type_t  stream_type;/**< buffer type: video or meta data */
    struct work_struct  dma_work;   /**< work to be done after DMA finished */
    atomic_t            hw_owned;   /**< if true, the FPGA owns the buffer */

    sxpf_file_hdl_t    *file_owner; /**< allocated by this user */
    struct list_head    owned_list; /**< member either in card's free list or in
                                         file's owned_buffers; may be an empty
                                         list if freed but still owned by DMA */

    u32                 rx_size;    /**< actually used number of bytes */

    struct semaphore    lock;       /**< protect access and manipulations */
    struct list_head    readers;    /**< reades that haven't released the frame,
                                         yet, by way of a buffer_state_t  */

    sxpf_buf_alloc_type_t alloc_kind;/**< Driver, none or user-space */
    struct scatterlist *sg_list;
    unsigned long       sg_len;

    /* management of user-space provided image DMA buffers */
    struct page       **pages;      /**< lockead user pages */
    long                nr_pages;   /**< number of locked pages */
    unsigned long long  user_header;
    unsigned long       data_offset;/**< data start within first payload page */

#ifdef ENABLE_NV_P2P
    u64                 nv_va;
    u64                 nv_offset;
    u64                 nv_len;
    int                 nv_dma_map_valid;

    struct nvidia_p2p_page_table    *nv_page_table;
    struct nvidia_p2p_dma_mapping   *nv_dma_mapping;
#endif

    u8                  hdr_cpy[64];/**< Copy of the first 64 bytes of the
                                         buffer after reception from FPGA */
};


/** Reference data for HW timestamp extrapolation based on system clock */
struct hw_ts_sync
{
    s64                 hw_time;    /**< HW timestamp */
    s64                 pc_time;    /**< CPU system time */
    u32                 pc_ns;      /**< nanosecond interval */
    u32                 hw_clocks;  /**< same interval in HW clocks */
};


// declaration of i2c slave device
typedef struct
{
    sxpf_file_hdl_t    *hClient;
    uint8_t             i2cChannel;
    uint32_t            i2cBaseAddr;
    uint8_t             i2cSlaveAddr;

} sxpfi2c_slave_device;


/** proFRAME PCIe card management structure */
struct sxpf_pci_device_s
{
    u32                 magic;      /**< magic number to check type */
#define SXPF_CARD_MAGIC         (0xCA2D81C5)

    u32                 type_id;    /**< hardware ID from register REG_NAME */
    int                 nr;         /**< card number */
    struct cdev         cdev;

    sxpf_card_props_t   props;      /**< card properties */

    u32                 feature_flags[8];

    /** Kernel PCI Device representation */
    struct pci_dev     *pci_device;
#if POLL_INTERRUPT
    struct timer_list   isr_timer;      /**< timer used to call ISR */
#endif

    /** bit mask of channels (in REG_CONTROL) that may be enabled */
    unsigned long       valid_channels;

    atomic_t            n_bufs_in_hw;   /**< number of buffers posted to FPGA */
    atomic_t            n_bufs_in_sw;   /**< no.\ of buffers recvd from FPGA */
    atomic_t            n_bufs_corrupt; /**< invalid buffers recvd from FPGA */

    /* DMA buffers for video info */
    struct dma_pool    *sge_pool;       /**< pool for scatter list entries */
    struct sxpf_buffer  dma_buffer[SXPF_MAX_DMA_BUFFERS];
    int                 ringBufferSize; /**< img buffers from driver AND user */
    int                 numDrvBuffers;  /**< img buffers allocated by driver */
    int                 numUserBuffers; /**< img buffers allocatable by user */
    int                 i2cRingSize;
    dma_addr_t          header_addr_bus;/**< Header DMA memory bus address */
    char               *header_addr_cpu;/**< Header DMA memory mapped for CPU */

    /* playback buffer allocation */
    struct semaphore    buf_alloc_lock;     /**< sync list update */
    wait_queue_head_t   play_buf_requesters;/**< wait queue of requesters */
    struct list_head    free_play_buffers;  /**< list of free buffers */

    /** Addresses and sizes of PCI BARs. Only even-numbered BARs 0,2, and 4
     *  are actual PCI BAR mappings. The others are used as "virtual" mappings
     *  consisting of a pointer-size-pair referencing existing memory.
     */
    void __iomem       *pci_base[6];
    u32                 pci_bar_size[6];

    /** Mask of enabled IRQs, which depends on channel activity status */
    u32                 irq_mask;
    spinlock_t          irq_mask_lock;  /**< sync access to REG_IRQ_MASK */

    atomic_t            io_disabled;    /**< 0=normal, 1=disabled by AER */
    sxpf_mode_t         mode;

    struct timer_list   hw_tstamp_timer;/**< extrapolate HW timestamps */
    int                 hw_ts_idx;      /**< index of most recent entry */
    s64                 hw_ts_highword; /**< primary HW ts high-word */
    struct hw_ts_sync   hw_ts_buf[4];   /**< pool of sync structs */
    struct hw_ts_sync  *hw_ts_ref;      /**< rcu pointer for readers */
#if !USE_GPL_SYMBOLS
    spinlock_t          hw_ts_lock;     /**< atomic HW ts access if non-GPLed */
#endif

#if USE_GPL_SYMBOLS
    struct workqueue_struct *event_queue;   /**< event work_queue */
#endif
    struct work_struct      event_work;     /**< element for event work queue */
    spinlock_t              event_lock;     /**< sync event_list access */
    struct list_head        event_list;     /**< event list filled by ISRs */

    /** Serialize channel activity changes by multiple users */
    struct semaphore    channel_lock;
    int                 video_channel_users[8];
    int                 i2c_channel_users[8];

    wait_queue_head_t   rd_queue;       /**< wait queue for readers */

    /* DMA command FIFO */
    spinlock_t          dma_fifo_lock;  /**< syncs concurrent cmd FIFO writes */

    u32                 control_reg;    /**< cached control reg content */
    spinlock_t          control_lock;   /**< syncs REG_CONTROL access */

    /* command FIFO lock and command argument RAM management */
    spinlock_t          cmd_fifo_lock;  /**< syncs concurrent cmd FIFO writes */
    spinlock_t          cmd_arg_lock;   /**< protect arguments slot free list */
    struct semaphore    cmd_slot_count; /**< argument slot allocation */
    int                 free_cmd_arg_slot;              /**< free list anchor */
    int                 cmd_arg_slots[SXPF_NUM_CMD_ARG_SLOTS]; /**< free list */
    atomic_t            cmd_status[SXPF_NUM_CMD_ARG_SLOTS]; /**< status flags */
#define CMD_STATUS_IDLE     (0xffffffff)
#define CMD_STATUS_PENDING  (0x00000000)
    wait_queue_head_t   fifo_wait_queue;/**< Waiting for a response FIFO IRQ */

    int                 user_seq_size;  /**< size of user sequence RAM */
    u32                 user_seq_offset;/**< offset in Plasma user seq.\ RAM */
    union
    {
        u8              b[512];
        u32             w[128];
    }                   user_seq_image; /**< User I2C sequence RAM image */

    /** Current VSYNC counters. The counter of each of the 4 channels takes up
     *  16 bits. Bit 15 (the uppermost, 16th, bit) of each counter is used as
     *  overflow bit, and hence is always kept cleared.
     *  This value is only modified by the interrupt handler.
     */
    // TODO support 8 channels
    u64                 current_vsyncs;

    /** frame counter of the most recently received image, one for each of
     *  four input channels.
     */
    u32                 frame_counters[8];

    struct semaphore    client_lock;    /**< sync access to clients list */
    struct list_head    clients;        /**< list of opened file handles */

    /// Array of i2c- slave structs to manage the 8 i2c slaves
    sxpfi2c_slave_device i2c_slaves[SXPF_MAX_I2C_SLAVES];
    struct semaphore    i2c_slave_lock; /**< sync I2C slave allocation */

#ifdef ENABLE_XVC
    // Xilinx Virtual Cable support
    spinlock_t          xvc_lock;   /**< sync config space access */
    struct xvc_algo_t   xvc_algo;   /**< XVC algo config */
    struct pcie_user_config xvc_config;
#endif
};


/** Wrapper for event list element to be delivered to reader */
typedef struct
{
    struct list_head    list;   /**< queue node */
    sxpf_event_t        event;  /**< event data to be delivererd to client */

} sxpf_unread_event_t;


/** in-flight status of a single buffer slot
 *
 * valid states are:
 *
 * Buffer state                                       | unread    | received
 * -------------------------------------------------- | :-------: | :-------:
 * hardware-owned (DMA in progress)                   | empty     | empty
 * event read by user; buffer not released, yet       | empty     | linked in
 * invalid (should never occur)                       | linked in | empty
 * received and announced to user (poll returns true) | linked in | linked in
 */
typedef struct
{
    sxpf_file_hdl_t    *client;     /**< client fd we are associated with */

    /** empty or element in file_hdl->unread_frames ring list */
    struct list_head    unread;

    /** empty or element of card->dma_buffer[idx].readers ring list */
    struct list_head    received;

} sxpf_buffer_state_t;


/** Structure used for storing per-user data about an opened card */
struct sxpf_file_hdl_s
{
    struct list_head    list;           /**< part of card->clients */

    sxpf_card_t        *card;

    struct semaphore    rlock; /**< serialize read access by multiple clients */
    struct semaphore    dlock; /**< sync list data addition and extraction */

    /** list of allocated buffers, locked by card->buf_alloc_lock */
    struct list_head    owned_buffers;

    u32                 channel_mask;   /**< Active channels for this handle */

    // TODO support 8 channels
    u64                 last_vsyncs;    /**< 4 15bit counters in one value */

    struct list_head    unread_events;  /**< unreadvents (e.g., errors) */

    /** Each read frame in this list will generate a SXPF_EVENT_FRAME_RECEIVED
     *  event instance
     */
    struct list_head    unread_frames;  /**< frame events not read by client */

    /** In-flight handling of individual buffer slots. These are here so that
     *  we don't need dynamic allocation in the ISR
     */
    sxpf_buffer_state_t buffer_state[SXPF_MAX_DMA_BUFFERS];

};


extern int          sxpf_num;
extern sxpf_card_t *g_cards[SXPF_MAX_DEVICES];


#endif /* !defined SXPF_MODULE_H_ */
/**
 * @endcond
 */
