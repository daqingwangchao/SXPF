/**
 * @file sxpf_module.c
 *
 * @if (!DRIVER)
 * The driver's internal documentation is not included.
 * @else
 * Driver's main implementation.
 * @endif
 *
 * @cond DRIVER
 */
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/version.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/aer.h>
#include <linux/pci_hotplug.h>
#include <linux/ctype.h>
#include <linux/dma-mapping.h>
#include <linux/mfd/core.h>
#include <linux/stringify.h>

#include "sxpf.h"
#include "sxpf_module.h"
#include "driver_acc.h"
#include "svnversion.h"
#include "sxpf_version.h"
#include "sxpf_xvc.h"

#ifdef ENABLE_NV_P2P
#include "nv-p2p.h"
#endif

#if USE_GPL_SYMBOLS
MODULE_LICENSE("GPL");
#else
MODULE_LICENSE("Solectrix");
#endif
MODULE_VERSION(__stringify(SXPF_SW_MAJOR)"."__stringify(SXPF_SW_MINOR)"."
               __stringify(SXPF_SW_ISSUE)".r"__stringify(SVNVERSION));

#define POLL_PERIOD_MS          (1)     /**< timer ISR poll period in ms */

/** Default number of DMA buffers allocated per card, if not changed with
 *  \c buffers command line argument when starting the Linux kernel module.
 */
#define SXPF_DEF_FRAME_SLOTS    (8)


#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
#define get_clock_realtime      ktime_get_real_ts64
#define get_clock_monotonic_raw ktime_get_raw_ts64
#else
#define get_clock_realtime      getnstimeofday64
#define get_clock_monotonic_raw getrawmonotonic64
#endif


enum
{
    SXPF_PENDING_EVENT_FRAME    = (1 << 0), /**< flag pending received frame */
    SXPF_PENDING_EVENT_VSYNC    = (1 << 1), /**< flag vertical blanking */
    SXPF_PENDING_EVENT_GENERIC  = (1 << 2), /**< flag generic event */
};


static void sxpf_enable_irqs(sxpf_card_t *card, u32 add_irqs);
static int free_owned_frame(sxpf_card_t *card, struct sxpf_buffer *buf);

extern int sxpf_init_cmd_fifo(sxpf_card_t *card);
extern void sxpf_stop_cmd_fifo(sxpf_card_t *card);
extern int sxpf_send_cmd(sxpf_card_t *card, u32 cmd,
                         u8 *args, unsigned num_args,
                         u8 *ret, unsigned num_ret, unsigned timeout_ms);
extern void sxpf_init_procfs(sxpf_card_t *card);
extern void sxpf_exit_procfs(sxpf_card_t *card);
extern void sxpf_isr_bar2(sxpf_card_t *card);
extern void sxpf_isr_channel_event(sxpf_card_t *card);

static void free_buffer(sxpf_card_t *card, struct sxpf_buffer *buf);
static void free_sg_segments(struct sxpf_buffer *buf);
static void free_driver_buffer(struct sxpf_buffer *buf);
static void* declare_cuda_buffer(struct sxpf_buffer *buf,
                                 sxpf_user_buffer_declare_t *req);
static void free_cuda_buffer(struct sxpf_buffer *buf);
static void* declare_heap_buffer(struct sxpf_buffer *buf,
                                 sxpf_user_buffer_declare_t *req);
static void free_heap_buffer(struct sxpf_buffer *buf);

static void disable_card(sxpf_card_t *card, const char *reason);

int ioctl_remove_i2c_slave(sxpf_file_hdl_t *fh, unsigned long arg);


/** Global card references to find them by index, e.g.\ in timer ISR */
sxpf_card_t  *g_cards[SXPF_MAX_DEVICES];

/** Debug level */
int sxpfDebugLevel     = SXPF_WARNING;
int sxpfDebugLevelIrq  = SXPF_ERROR;

static int sxpfFrameSize       = SXPF_DMA_DEF_SLOT_SIZE;
static int sxpfRingBufferSize  = SXPF_DEF_FRAME_SLOTS;
static u32 buf_sg_segment_size = 1 << 20;
static int sxpfNumUserBuffers  = 0;
static int sxpfNumI2cBuffers   = SXPF_MAX_I2C_MSG_SLOTS;
static bool sxpfPatchFirmware  = 0;
static bool sxpfSgEnable       = 0;    // no scatter-gathering by default
static int sxpfPayloadSize     = 0;    // default: don't change system setting
static int sxpfReadRequestSize = 512;
#if USE_GPL_SYMBOLS
static bool sxpfAerEnable      = false;
#endif
static bool sxpfTrigIntEnable  = false;
static int sxpfMaxBufDelay     = 20;   // wait at most 20us for buffer header
static bool sxpfBufMapCached   = true;
static bool sxpf_zero_rx_headers = true; // clear RX headers by default

/* Module params to be seen by the outside world */
module_param_named(debug, sxpfDebugLevel, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug level for sxpf driver.");

module_param_named(debug_irq, sxpfDebugLevelIrq, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug_irq, "Debug level for sxpf irq handler.");

module_param_named(buffers, sxpfRingBufferSize, int, S_IRUGO);
MODULE_PARM_DESC(buffers, "Number of pre-allocated image DMA buffers (0...31)");

module_param_named(bufgranularity, buf_sg_segment_size, int, S_IRUGO);
MODULE_PARM_DESC(bufgranularity, "Granularity of image DMA buffers (power of 2)");

module_param_named(user_buffers, sxpfNumUserBuffers, int, S_IRUGO);
MODULE_PARM_DESC(user_buffers,
                 "Number of user-provided image DMA buffers (0...31)");

module_param_named(i2c_buffers, sxpfNumI2cBuffers, int, S_IRUGO);
MODULE_PARM_DESC(i2c_buffers,
                 "Number of pre-allocated I2C DMA buffers (0...31)");

module_param_named(framesize, sxpfFrameSize, int, S_IRUGO);
MODULE_PARM_DESC(framesize, "Max. size of one frame (Bytes, multiple of 4K)");

module_param_named(fwpatch, sxpfPatchFirmware, bool, S_IRUGO);
MODULE_PARM_DESC(fwpatch, "Patch firmware using fw file sxpf.fw or sxpf_g2.fw");

module_param_named(sgenable, sxpfSgEnable, bool, S_IRUGO);
MODULE_PARM_DESC(sgenable, "Enable scatter-gathering for supported hardware");

module_param_named(mps, sxpfPayloadSize, int, S_IRUGO);
MODULE_PARM_DESC(mps, "Change PCIe Max. Payload Size (0=use system default)");

#if USE_GPL_SYMBOLS
module_param_named(aer, sxpfAerEnable, bool, S_IRUGO);
MODULE_PARM_DESC(aer, "Enable PCIe Advanced Error Reporting");
#endif

module_param_named(trigint, sxpfTrigIntEnable, bool, S_IRUGO);
MODULE_PARM_DESC(trigint, "Enable trigger timestamp interrupts (default=0)");

module_param_named(max_buf_delay, sxpfMaxBufDelay, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(max_buf_delay, "Max time in us of waiting for buffer data.");

module_param_named(map_cached, sxpfBufMapCached, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(map_cached, "Allow DMA buffers to be cached (default=1)");

#ifdef ENABLE_XVC
static bool sxpfXvcEnable = true;

module_param_named(xvc, sxpfXvcEnable, bool, S_IRUGO);
MODULE_PARM_DESC(xvc, "Enable Xilinx Virtual Cable debug sybsystem");
#endif

module_param_named(zero_rx_headers, sxpf_zero_rx_headers, bool,
                   S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(zero_rx_headers,
                 "Zero image headers before capturing (default=true)");

/** Specifies vendor and device ids of device this driver can manage */
static struct pci_device_id sxpf_pci_device_ids [] =
{
    { PCI_DEVICE(PCI_VENDOR_ID_PLDA, PCI_DEVICE_ID_PROFRAME) },
    { 0 }
};

MODULE_DEVICE_TABLE(pci, sxpf_pci_device_ids);


int sxpf_num; /**< Current number of initialized chards (in probe()) */

static struct class *sxpf_class;
static int sxpf_major;


void write_reg8_bar(sxpf_card_t *card, u32 bar, u32 adr, u32 dat)
{
    WARN_ON(card->magic != SXPF_CARD_MAGIC);

    CARD_PRINTF(SXPF_DEBUG, "reg8[%x:0x%08x] <- 0x%02x", bar, adr, dat);

    iowrite8(dat, card->pci_base[bar] + adr);
}


void write_reg16_bar(sxpf_card_t *card, u32 bar, u32 adr, u32 dat)
{
    WARN_ON(card->magic != SXPF_CARD_MAGIC);

    CARD_PRINTF(SXPF_DEBUG, "reg16[%x:0x%08x] <- 0x%04x", bar, adr, dat);

    iowrite16(dat, card->pci_base[bar] + adr);
}


// specialized funtion for BAR0 access
static void write_reg32_int(sxpf_card_t *card, u32 adr, u32 dat)
{
    WARN_ON(card->magic != SXPF_CARD_MAGIC);

    iowrite32(dat, card->pci_base[0] + adr);
}


// specialized funtion for BAR0 access
static void write_reg32(sxpf_card_t *card, u32 adr, u32 dat)
{
    CARD_PRINTF(SXPF_DEBUG, "reg32[0x%08x] <- 0x%08x", adr, dat);

    write_reg32_int(card, adr, dat);
}


void write_reg32_bar(sxpf_card_t *card, u32 bar, u32 adr, u32 dat)
{
    WARN_ON(card->magic != SXPF_CARD_MAGIC);

    CARD_PRINTF(SXPF_DEBUG, "reg32[%x:0x%08x] <- 0x%08x", bar, adr, dat);

    iowrite32(dat, card->pci_base[bar] + adr);
}


u32 read_reg8_bar(sxpf_card_t *card, u32 bar, u32 adr)
{
    u32 res;

    WARN_ON(card->magic != SXPF_CARD_MAGIC);

    res = ioread8(card->pci_base[bar] + adr);

    CARD_PRINTF(SXPF_DEBUG, "reg8[%x:0x%08x] -> 0x%02x", bar, adr, res);

    return res;
}


u32 read_reg16_bar(sxpf_card_t *card, u32 bar, u32 adr)
{
    u32 res;

    WARN_ON(card->magic != SXPF_CARD_MAGIC);

    res = ioread16(card->pci_base[bar] + adr);

    CARD_PRINTF(SXPF_DEBUG, "reg16[%x:0x%08x] -> 0x%04x", bar, adr, res);

    return res;
}


u32 read_reg32_bar(sxpf_card_t *card, u32 bar, u32 adr)
{
    u32 res;

    WARN_ON(card->magic != SXPF_CARD_MAGIC);

    res = ioread32(card->pci_base[bar] + adr);

    CARD_PRINTF(SXPF_DEBUG, "reg32[%x:0x%08x] -> 0x%08x", bar, adr, res);

    return res;
}


// specialized funtion for BAR0 access
static u32 read_reg32_int(sxpf_card_t *card, u32 adr)
{
    WARN_ON(card->magic != SXPF_CARD_MAGIC);

    return ioread32(card->pci_base[0] + adr);
}


// specialized funtion for BAR0 access
static u32 read_reg32(sxpf_card_t *card, u32 adr)
{
    u32 res = read_reg32_int(card, adr);

    CARD_PRINTF(SXPF_DEBUG, "reg32[0x%08x] -> 0x%08x", adr, res);

    return res;
}


static void set_control_reg(sxpf_card_t* card, u32 val)
{
    if (!atomic_read(&card->io_disabled))
    {
        write_reg32(card, REG_CONTROL, val);
    }
    card->control_reg = val;
}


static int has_feature(sxpf_card_t *card, u32 feature_num)
{
    return card->feature_flags[feature_num / 32] & (1 << (feature_num % 32));
}


static void force_feature(sxpf_card_t *card, u32 feature_num)
{
    card->feature_flags[feature_num / 32] |= (1u << (feature_num % 32));
}


static int is_record(sxpf_card_t* card)
{
    return card->props.capabilities & SXPF_CAP_VIDEO_RECORD;
}


static int is_replay(sxpf_card_t* card)
{
    return card->props.capabilities & SXPF_CAP_VIDEO_PLAYBACK;
}


static inline bool is_driver_buf_slot(sxpf_card_t *card, int slot)
{
    return slot < card->numDrvBuffers || slot >= card->ringBufferSize;
}


static inline bool is_user_buf_slot(sxpf_card_t *card, int slot)
{
    return slot >= card->numDrvBuffers && slot < card->ringBufferSize;
}


static inline bool is_driver_buffer(struct sxpf_buffer *buf)
{
    sxpf_card_t *card = buf->card;
    int          slot = buf - card->dma_buffer;

    return is_driver_buf_slot(card, slot);
}


static inline bool is_user_buffer(struct sxpf_buffer *buf)
{
    sxpf_card_t *card = buf->card;
    int          slot = buf - card->dma_buffer;

    return is_user_buf_slot(card, slot);
}


#define DUMP_BUF(debugLevel, buf, fmt, args...)                         \
    do                                                                  \
    {                                                                   \
        sxpf_card_t *card = (buf)->card;                                \
        u32         *hdr = buf->kernel_adr;                             \
        u32         i;                                                  \
        if (sxpfDebugLevel >= debugLevel)                               \
        {                                                               \
            printk(KERN_ERR "[" DEVICE_NAME "%d] buffer: " fmt "\n",    \
                   MINOR(card->cdev.dev), ## args);                     \
                for (i = 0; i < 16; i++)                                \
                    printk(KERN_ERR "[" DEVICE_NAME "%d] %02x: %08x\n", \
                           MINOR(card->cdev.dev), i * 4, hdr[i]);       \
        }                                                               \
    }                                                                   \
    while (false)


/** Pass the given buffer's address to the fpga for reception of more data or
 *  playback of user-provided data.
 *
 * @param card  The grabber card reference
 * @param buf   The buffer to provide to the FPGA
 * @param size  Number of bytes allowed in the following DMA operation
 *              (complete buffer for record, or user data size for replay)
 *
 * @return  0 on success, negative errno on error
 */
static long sxpf_hand_buffer_to_fpga(sxpf_card_t *card,
                                     struct sxpf_buffer *buf, u32 size)
{
    int             ret = 0;
    unsigned long   iflags;
    int             buf_no = (int)(buf - card->dma_buffer);

    if (card->mode == SXPF_SHUTDOWN)
    {
        CARD_PRINTF(SXPF_DEBUG, "shutdown - keeping buf #%d", buf_no);
        return 0;
    }

    if (card->mode == SXPF_CAPTURING && sxpf_zero_rx_headers)
        memset(buf->kernel_adr, 0, sizeof(sxpf_image_header_t));

    if (sxpfDebugLevel >= SXPF_DEBUG)
    {
        // the channel to send on is encoded in bits 30..31 of the size argument
        // bit 31 therefore determines the memory channel on the card for this
        // transfer
        u32     chan = size >> 30;
        u32     fill_level;
        fill_level = read_reg32(card, REG_VIDEO_LEVEL0 + ((size >> 29) & 8));

        DUMP_BUF(SXPF_DEBUG, buf, "tx buf #%d, size=0x%08x, chan=%u, lvl=%u",
                 buf_no, size & 0x3fffffff, chan, fill_level);
    }

    if (buf->alloc_kind != SXPF_ALLOC_DRIVER && buf->sg_list != NULL)
    {
        dma_sync_sg_for_device(&card->pci_device->dev, buf->sg_list,
                               buf->sg_len, buf->dma_dir);
    }

    /* acquire exclusive access to cmd FIFO */
    spin_lock_irqsave(&card->dma_fifo_lock, iflags);

#if 0 /* the FIFO can accept more commands (64) than we have buffers */
    // wait for command FIFO to have space for our command
    // note: using register access functions that do _not_ print debug messages
    while (0 == (read_reg32_int(card, REG_FIFO_STATUS)
                 & (buf->type == SXPF_DATA_TYPE_VIDEO ?
                    FIFO_STAT_VCMD_READY : FIFO_STAT_MCMD_READY)))
        udelay(10); // XXX problem, because we are holding a spinlock!
    // TODO implement timeout to prevent blocking
#endif

    atomic_inc(&card->n_bufs_in_hw);

    if (atomic_cmpxchg(&buf->hw_owned, 0, 1))
    {
        CARD_PRINTF(SXPF_ERROR,
                    "not DMA'ing buf #%d, because it's still owned by HW",
                    buf_no);

        atomic_dec(&card->n_bufs_in_hw);

        ret = -EINVAL; // FPGA already ownes the buffer
    }
    else
    {
        atomic_dec(&card->n_bufs_in_sw);

        // send command
        if (buf->stream_type == SXPF_DATA_TYPE_VIDEO)
        {
            write_reg32(card, REG_VCMD_ADDR_LO, buf->bus_adr);
            write_reg32(card, REG_VCMD_ADDR_HI, buf->bus_adr >> 32);
            write_reg32(card, REG_VCMD_SIZE, size);
        }
        else if (buf->stream_type == SXPF_DATA_TYPE_META)
        {
            write_reg32(card, REG_MCMD_ADDR_LO, buf->bus_adr);
            write_reg32(card, REG_MCMD_ADDR_HI, buf->bus_adr >> 32);
            write_reg32(card, REG_MCMD_SIZE, size);
        }
    }

    /* release command FIFO */
    spin_unlock_irqrestore(&card->dma_fifo_lock, iflags);

    return ret;
}


/** Handle parts of generic event work that need to happen in a thread context,
 *  because they might block.
 *
 * @param work  Event work struct of the SXPF instance, must be the work member
 *              of an event_work_t structure.
 */
static void sxpf_do_generic_event_work(struct work_struct *work)
{
    sxpf_card_t     *card = container_of(work, sxpf_card_t, event_work);
    sxpf_file_hdl_t *fh;
    unsigned long   iflags;

    spin_lock_irqsave(&card->event_lock, iflags);

    while (!list_empty(&card->event_list))
    {
        struct list_head    *l = card->event_list.next;
        sxpf_unread_event_t *w = container_of(l, sxpf_unread_event_t, list);

        list_del(l);

        // don't block ISRs too long, while we update clients (we may need to
        // wait for memory)
        spin_unlock_irqrestore(&card->event_lock, iflags);

        down(&card->client_lock);// prevent concurrent client list manipulations

        // enqueue a copy of the event in each connected client's unread_events
        // list
        list_for_each_entry(fh, &card->clients, list)
        {
            sxpf_unread_event_t *evt = kmalloc(sizeof(sxpf_unread_event_t),
                                               GFP_KERNEL);

            if (!evt)
            {
                CARD_PRINTF(SXPF_ERROR,
                            "couldn't alloc event struct for client!");
                break;
            }

            evt->event = w->event;

            down(&fh->dlock);

            list_add_tail(&evt->list, &fh->unread_events);

            up(&fh->dlock);
        }

        up(&card->client_lock);

        wake_up_interruptible(&card->rd_queue); // wake up readers

        // free memory that was allocated in the ISR
        kfree(w);

        // acquire event list access, again, before checking if more evenst are
        // enqueued
        spin_lock_irqsave(&card->event_lock, iflags);
    }

    spin_unlock_irqrestore(&card->event_lock, iflags);
}


static void reuse_capture_buffer(struct sxpf_buffer *buf)
{
    sxpf_card_t *card = buf->card;

    // unmap a user-allocated buffer has been discarded by the user
    if (is_user_buffer(buf) && buf->file_owner == NULL)
    {
        // call buffer's DMA unmap function
        free_buffer(card, buf);
    }
    else
    {
        sxpf_hand_buffer_to_fpga(card, buf, buf->buf_size);
    }
}

/** Handle parts of DMA completion work that need to happen in a thread context,
 *  because they might block.
 *
 * @param work  Event work struct of the SXPF instance, must be the dma_work
 *              member of a struct sxpf_buffer sturcture.
 */
static void sxpf_do_dma_event_work(struct work_struct *work)
{
    sxpf_card_t         *card;
    struct sxpf_buffer  *buf = container_of(work, struct sxpf_buffer, dma_work);
    ptrdiff_t           slot;
    int                 was_hw_owned;
    sxpf_file_hdl_t     *fh;
    int                 num_readers = 0;
    sxpf_image_header_t *header = buf->kernel_adr;
    int                 buf_delay = 0;
    u32                 payload_size;

    WARN_ON(buf->magic != SXPF_BUFFER_MAGIC);

    card = buf->card;

    down(&buf->lock);

    // reclaim buffer back from FPGA
    was_hw_owned = atomic_cmpxchg(&buf->hw_owned, 1, 0);

    WARN_ON(!was_hw_owned);

    if (is_record(card))
    {
        WARN_ON(!list_empty(&buf->readers));
    }

    if (buf->alloc_kind != SXPF_ALLOC_DRIVER && buf->sg_list != NULL)
    {
        dma_sync_sg_for_cpu(&card->pci_device->dev, buf->sg_list, buf->sg_len,
                            buf->dma_dir);
    }

    // in playback mode we may receive a buffer here, that no-one is interested
    // in anymore (it was freed, while the DMA was still in progress).
    if (is_replay(card) && buf->file_owner == NULL)
    {
        up(&buf->lock);

        DUMP_BUF(SXPF_DEBUG, buf, "rx buf #%d, size=0x%08x, cam=%d (freed)",
                 (int)(buf - card->dma_buffer), buf->rx_size, header->cam_id);
        return;
    }

    // on some cards the buffer may have been signaled as ready before the
    // header data has been transfered: wait for the buffer size to become != 0
    do
    {
        payload_size = READ_ONCE(*(u32*)buf->kernel_adr);

        if (payload_size > 0)
            break;  // header has been received

        udelay(1);  // wait one micro second
        buf_delay += 1;

    } while (buf_delay < sxpfMaxBufDelay);

    if (payload_size == 0)
    {
        if (card->mode == SXPF_SHUTDOWN)
        {
            up(&buf->lock);
            return;
        }

        CARD_PRINTF(SXPF_ERROR, "rx buf #%d: header contents not available",
                    (int)(buf - card->dma_buffer));

        // recycle buffer for further reception
        if (card->mode == SXPF_CAPTURING)
        {
            reuse_capture_buffer(buf);

            up(&buf->lock);
            return;
        }
    }
    else
    {
        // make a copy of the header for later checks
        memcpy(buf->hdr_cpy, buf->kernel_adr, sizeof(sxpf_image_header_t));

        DUMP_BUF(SXPF_DEBUG, buf, "rx buf #%d, size=0x%08x, cam=%d",
                 (int)(buf - card->dma_buffer), buf->rx_size, header->cam_id);
    }

    // enqueue event in event queues of all readers
    slot = buf - card->dma_buffer;

    down(&card->client_lock);   // prevent concurrent client list manipulations

    list_for_each_entry(fh, &card->clients, list)
    {
        int do_post;
        int is_meta = (buf->stream_type == SXPF_DATA_TYPE_META);

        /* during record mode all clients that have started the channel will
         * get the notification - except for user-allocated buffers, which will
         * only be delivered to the user who allocated them
         */
        do_post = is_record(card) &&
            (!is_user_buf_slot(card, slot) || buf->file_owner == fh) &&
            ((!is_meta &&
              ((SXPF_STREAM_VIDEO0 << header->cam_id) & fh->channel_mask)) ||
            (is_meta &&
             ((SXPF_STREAM_I2C0 << header->cam_id) & fh->channel_mask)));

        /* during playback only the client that has exclusive access to the
         * buffer will be notified
         */
        do_post |= is_replay(card) && !is_meta && (buf->file_owner == fh);

        if (do_post)
        {
            num_readers++;

            // add client to buffer's readers list
            list_add(&fh->buffer_state[slot].received, &buf->readers);

            // add buffer to this client's unread frames list
            down(&fh->dlock);

            list_add_tail(&fh->buffer_state[slot].unread, &fh->unread_frames);

            up(&fh->dlock);
        }
    }

    up(&card->client_lock);

    if (num_readers == 0)
    {
        // No readers are interested in this channel.
        // this shouldn't happen, but we return the buffer back to the FPGA in
        // case it happens anyway, so we don't stall the hardware
        if (card->mode == SXPF_CAPTURING)
        {
            reuse_capture_buffer(buf);
        }

        CARD_PRINTF(SXPF_DEBUG, "No readers for buf #%d notification!",
                    (int)(buf - card->dma_buffer));
    }
    else
    {
        wake_up_interruptible(&card->rd_queue); // wake up readers
    }

    up(&buf->lock);
}


/** Find the buffer handle belonging to the given physical bus address
 *
 * @param card  The frame grabber handle
 * @param type  The type of DMA that finished (video or meta data)
 *
 * @return  The buffer address if found, or NULL
 */
static struct sxpf_buffer *sxpf_find_buffer(sxpf_card_t *card,
                                            dma_addr_t bus_addr,
                                            sxpf_stream_type_t type)
{
    int     i, first = 0, last = 0;

    if (type == SXPF_DATA_TYPE_VIDEO)
    {
        last = card->ringBufferSize;
    }
    else if (type == SXPF_DATA_TYPE_META)
    {
        first = card->ringBufferSize;
        last = first + card->i2cRingSize;
    }

    // TODO speed up look-up
    for (i = first; i < last; i++)
    {
        struct sxpf_buffer  *buf = &card->dma_buffer[i];

        if (buf->alloc_kind != SXPF_BUF_UNUSED && bus_addr == buf->bus_adr)
            return buf;
    }

    // If we come here the FPGA returned a physical DMA address that didn't
    // originate from us (perhaps from an earlier driver instance?)
    return NULL;
}


/** Read DMA completion FIFO and return the buffer handle of the finished DMA
 *
 * @param card  The frame grabber handle
 * @param type  The type of DMA that finished (video or meta data)
 * @param pbuf  Pointer to the Buffer handle belonging to the finshed DMA.
 *              If this function writes a NULL pointer here, then the PCIe bus
 *              is broken. In this case the return value will be 0, too.
 *
 * @return  0 on success; non-0 on error
 *
 * @note    This function is called in interrupt context. (No sleeping allowed.)
 */
static int sxpf_handle_dma_completion(sxpf_card_t *card,
                                      sxpf_stream_type_t type,
                                      struct sxpf_buffer **pbuf)
{
    u32                 addr_lo, addr_hi, size;
    dma_addr_t          bus_addr;
    struct sxpf_buffer *buf;
    u32                 stat = read_reg32(card, REG_FIFO_STATUS);

    if (stat & 0xffff0000)
    {
        write_reg32(card, REG_FIFO_STATUS, stat); // write 1 to clear trig bits
        CARD_PRINTF_IRQ(SXPF_WARNING, "FIFO status triggered: 0x%08x", stat);
    }

    // read the completion FIFO
    if (type == SXPF_DATA_TYPE_VIDEO)
    {
        if (stat & FIFO_STAT_VFIN_EMPTY)
            return 3;

        addr_lo = read_reg32(card, REG_VFIN_ADDR_LO);
        addr_hi = read_reg32(card, REG_VFIN_ADDR_HI);
        size    = read_reg32(card, REG_VFIN_SIZE);
    }
    else if (type == SXPF_DATA_TYPE_META)
    {
        if (stat & FIFO_STAT_MFIN_EMPTY)
            return 3;

        addr_lo = read_reg32(card, REG_MFIN_ADDR_LO);
        addr_hi = read_reg32(card, REG_MFIN_ADDR_HI);
        size    = read_reg32(card, REG_MFIN_SIZE);
    }
    else
    {
        BUG();
    }

    bus_addr = ((u64)addr_hi << 32ull) | addr_lo;

    if (WARN(bus_addr == 0xffffffffffffffffull || bus_addr == 0, KERN_ERR
             "sxpf%d: got invalid buffer address from grabber - PCIe broken\n",
             card->nr))
    {
        buf = NULL; // PCIe broken
    }
    else
    {
        atomic_dec(&card->n_bufs_in_hw);

        // identify the buffer that was used in the DMA operation
        buf = sxpf_find_buffer(card, bus_addr, type);

        if (WARN(buf == NULL, KERN_ERR
                 "sxpf%d: got unknown buffer address 0x%016llx from grabber\n",
                 card->nr, bus_addr))
        {
            atomic_inc(&card->n_bufs_corrupt);
        }
        else
        {
            atomic_inc(&card->n_bufs_in_sw);

            // remember number of received bytes in record mode (not used during
            // playback)
            buf->rx_size = size;
        }
    }

    *pbuf = buf;

    return 0;
}


/** Enable FPGA stream logic with the channels passed in the channel_op param
 *
 * @param card          The grabber card reference
 * @param channel_op    Channels and mode to enable. @see REG_CONTROL
 *
 * @return 1 if the stream logic was previously enabled, 0 otherwise
 */
static int sxpf_pciestream_start(sxpf_card_t* card, unsigned long channel_op)
{
    u32             ctrl;
    unsigned long   iflags;

    spin_lock_irqsave(&card->control_lock, iflags);

    ctrl = card->control_reg;

    //write_reg32(card, REG_FIFO_CONTROL, FIFO_CTRL_NOTIFY_USER);

    // note: CTRL_MODE_PLAYBACK will be enabled only if requested in channel_op
    set_control_reg(card, ctrl | CTRL_RUN | channel_op);

    spin_unlock_irqrestore(&card->control_lock, iflags);

    return (ctrl & CTRL_RUN) ? 1 : 0;
}


/** Stop FPGA stream logic for the channels passed in the channel_op param
 *
 * @note
 *      On playback cards, this does _not_ disable the streaming logic, because
 *      by disabling the video frontend it would cause buffers to get stuck in
 *      the command queue and thus prevent proper cleanup.
 *
 * @param card          The grabber card reference
 * @param channel_op    Channels and mode to disable. @see REG_CONTROL
 */
static void sxpf_pciestream_stop(sxpf_card_t* card, unsigned long channel_op)
{
    u32             ctrl;
    unsigned long   iflags;

    spin_lock_irqsave(&card->control_lock, iflags);

    ctrl = card->control_reg;

    if (!(ctrl & CTRL_MODE_PLAYBACK))
    {
        ctrl &= ~channel_op;    // disable requested channels

        set_control_reg(card, ctrl);
    }

    spin_unlock_irqrestore(&card->control_lock, iflags);
}


/** Enable FPGA stream recording logic with the channels passed in the
 *  channel_mask param.
 *
 * The file handle will only receive images from channels it has requested via
 * this function. Other clients may request the same or different channels
 * without affecting this client.
 *
 * @param fh            The file client to start capturing for
 * @param channel_mask  Channels to enable. @see REG_CONTROL and sxpf_channel_t
 *
 * @return 0 on success
 * @return Negative errno on error
 */
static long ioctl_start_record(sxpf_file_hdl_t *fh, unsigned long channel_mask)
{
    sxpf_card_t     *card = fh->card;
    int             i;

    if (!is_record(card))
    {
        CARD_PRINTF(SXPF_ERROR, "Card doesn't have RECORD capability!");
        return -EINVAL;
    }

    if (channel_mask & ~card->valid_channels)
    {
        CARD_PRINTF(SXPF_INFO,
                    "Invalid channel options requested for recording: %lx",
                    channel_mask);

        channel_mask &= card->valid_channels;
    }

    if (down_interruptible(&card->channel_lock))
    {
        return -EINTR;
    }

    card->mode = SXPF_CAPTURING;

    if (!(channel_mask & ~fh->channel_mask))
    {
        up(&card->channel_lock);

        CARD_PRINTF(SXPF_INFO,
                    "All channels in mask 0x%lx are already RECORDing",
                    channel_mask);
        return 0;
    }

    // only activate channels that are not yet active for this user
    channel_mask &= ~fh->channel_mask;

    fh->channel_mask |= channel_mask;

    for (i = 0; i < card->props.num_channels; i++)
    {
        if (channel_mask & (SXPF_STREAM_VIDEO0 << i))
            ++card->video_channel_users[i];     // increment user-count

        if (channel_mask & (SXPF_STREAM_I2C0 << i))
            ++card->i2c_channel_users[i];       // increment user-count
    }

    //channel_mask |= CTRL_VCHAN_NBYPASS_ALL;

    if (sxpf_pciestream_start(card, channel_mask) == 0)
    {
        // provide grabber with DMA addresses to store received images and I2C
        // messages, if the streaming logic wasn't active before
        for (i = 0; i < card->ringBufferSize + card->i2cRingSize; i++)
        {
            struct sxpf_buffer  *buf = card->dma_buffer + i;

            // skip user-allocated buffers - they should be posted to the
            // hardware explicitly
            if (is_user_buf_slot(card, i))
                continue;

            down(&buf->lock);

            if (list_empty(&buf->readers))
            {
                if (!atomic_read(&buf->hw_owned))
                {
                    sxpf_hand_buffer_to_fpga(card, buf, buf->buf_size);
                }
            }

            up(&buf->lock);
        }
    }

    up(&card->channel_lock);

    CARD_PRINTF(SXPF_INFO, "RECORD started for channel mask 0x%lx",
                channel_mask);

    return 0;
}


/** Enable FPGA stream playback logic with the channels passed in the
 *  channel_mask param
 *
 * @param fh            The file client to start replaying for
 * @param channel_mask  Channels to enable. @see REG_CONTROL and sxpf_channel_t
 *
 * @return 0 on success
 * @return Negative errno on error
 */
static long ioctl_start_playback(sxpf_file_hdl_t *fh,
                                 unsigned long channel_mask)
{
    sxpf_card_t     *card = fh->card;
    int             i;

    if (!is_replay(card))
    {
        CARD_PRINTF(SXPF_ERROR, "Card doesn't have PLAYBACK capability!");
        return -EINVAL;
    }

    if (channel_mask & ~card->valid_channels)
    {
        CARD_PRINTF(SXPF_INFO,
                    "Invalid channel options requested for playback: %lx",
                    channel_mask);

        channel_mask &= card->valid_channels;
    }

    if (down_interruptible(&card->channel_lock))
    {
        return -EINTR;
    }

    card->mode = SXPF_REPLAYING;

    if (!(channel_mask & ~fh->channel_mask))
    {
        up(&card->channel_lock);

        CARD_PRINTF(SXPF_INFO,
                    "All channels in mask 0x%lx are already REPLAYing",
                    channel_mask);
        return 0;
    }

    // only activate channels that are not yet active for this user
    channel_mask &= ~fh->channel_mask;

    fh->channel_mask |= channel_mask;

    for (i = 0; i < card->props.num_channels; i++)
    {
        if (channel_mask & (SXPF_STREAM_VIDEO0 << i))
            ++card->video_channel_users[i];     // increment user-count

        // TODO support I2C replay?
    }

    sxpf_pciestream_start(card, channel_mask | CTRL_MODE_PLAYBACK);

    up(&card->channel_lock);

    CARD_PRINTF(SXPF_INFO, "PLAYBACK started for channel mask 0x%lx",
                channel_mask);

    return 0;
}


static inline int has_free_play_buffers_unsynced(sxpf_card_t *card)
{
    int ret = !list_empty(&card->free_play_buffers);

    if (ret)
    {
        struct sxpf_buffer *buf = list_first_entry(&card->free_play_buffers,
                                                   struct sxpf_buffer,
                                                   owned_list);
        ret = !atomic_read(&buf->hw_owned);
    }

    return ret;
}


static inline int has_free_play_buffers(sxpf_card_t *card)
{
    int             ret;

    down(&card->buf_alloc_lock);

    ret = has_free_play_buffers_unsynced(card);

    up(&card->buf_alloc_lock);

    return ret;
}


/** Allocate a buffer for exclusive use by the calling process during playback.
 *
 * The buffer is sent to the hardware for playback when ioctl_release_frame() is
 * called on it with data_size set unequal to 0 in the passed sxpf_frame_info_t
 * structure.
 *
 * Exclusive use must be relinquished by calling ioctl_release_frame() with
 * data_size set to 0 in the passed sxpf_frame_info_t structure.
 *
 * @param fh   The file client to stop streaming for
 * @param arg  Pointer to the variable where to store the allocated slot number.
 *
 * @return 0 on success
 * @return negative errno on error.
 */
static int ioctl_alloc_play_frame(sxpf_file_hdl_t *fh, unsigned long arg)
{
    sxpf_card_t         *card = fh->card;
    __u32               request;    // in: timeout, out: slot number
#if USE_GPL_SYMBOLS
    ktime_t             abs_end_time;
#else
    u64                 abs_end_time;   // jiffies
#endif
    int                 res;
    struct sxpf_buffer  *buf;

    // can only aquire exclusive buffer access on playback cards
    if (!is_replay(card))
        return -EINVAL;

    // read timeout argument from user
    if (copy_from_user(&request, (void __user *)arg, sizeof(request)))
        return -EIO;

#if USE_GPL_SYMBOLS
    abs_end_time = ktime_add_us(ktime_get(), request);
#else
    abs_end_time = get_jiffies_64() + usecs_to_jiffies(request);
#endif

    down(&card->buf_alloc_lock);

    while (!has_free_play_buffers_unsynced(card))
    {
        up(&card->buf_alloc_lock);

        if (request == (__u32)-1)
        {
            res = wait_event_interruptible(card->play_buf_requesters,
                                           has_free_play_buffers(card));
        }
        else
        {
#if USE_GPL_SYMBOLS
            ktime_t timeout = ktime_sub(abs_end_time, ktime_get());

            if (ktime_to_ns(timeout) <= 0)
            {
                res = -ETIME;
                goto err_timeout;
            }

            res = wait_event_interruptible_hrtimeout(card->play_buf_requesters,
                                                     has_free_play_buffers(card),
                                                     timeout);
#else
            u64 now = get_jiffies_64();

            if (time_after64(now, abs_end_time))
            {
                res = -ETIME;
                goto err_timeout;
            }

            res = wait_event_interruptible_timeout(card->play_buf_requesters,
                                                   has_free_play_buffers(card),
                                                   abs_end_time - now);
#endif
        }

        if (res)
        {
err_timeout:
            CARD_PRINTF(SXPF_WARNING, "alloc playback frame timed out");
            return res; // -ETIME, or -ERESTARTSYS
        }

        down(&card->buf_alloc_lock);
        // check condition again, as we may contend with another thread
    }

    // extract one element from the list of free playback buffers, while other
    // concurrent accesses are locked out, and let file handle take ownership of
    // the buffer
    buf = list_first_entry(&card->free_play_buffers, struct sxpf_buffer,
                           owned_list);
    list_move(&buf->owned_list, &fh->owned_buffers);

    up(&card->buf_alloc_lock);

    buf->file_owner = fh;
    request = buf - card->dma_buffer;

    CARD_PRINTF(SXPF_DEBUG, "allocated playback frame #%u", request);

    return copy_to_user((void __user *)arg, &request, sizeof(request));
}


/** Let the user provide a DMA buffer.
 *
 * @param fh   The file client to stop streaming for
 * @param arg  Pointer to the variable where to store the allocated slot number.
 *
 * @return 0 on success
 * @return negative errno on error.
 */
static int ioctl_init_user_buffer(sxpf_file_hdl_t *fh, unsigned long arg)
{
    sxpf_card_t                *card = fh->card;
    sxpf_user_buffer_declare_t  request;
    unsigned                    i;
    struct timespec64           before, after;
    unsigned long long          duration;
    unsigned long               n_pages = 0;
    int                         res = -EIO;

    // buffer allocation function, depending on allocation kind
    void* (*init_buffer)(struct sxpf_buffer *, sxpf_user_buffer_declare_t *);

    if (!has_feature(card, SXPF_FEATURE_GENERIC_SG))
    {
        CARD_PRINTF(SXPF_ERROR, "Card firmware doesn't have the generic scatter"
                    "/gather ability needed for user-space capture buffers.");
        return -EINVAL;
    }

    if (copy_from_user(&request, (void __user *)arg, sizeof(request)))
        return -EIO;

    // TODO lift size restriction when mmap has been sorted out with respect to
    // user-allocated buffers
    if (request.size != sxpfFrameSize || !request.header || !request.payload)
    {
        CARD_PRINTF(SXPF_ERROR, "%s: invalid user buffer properties",
                    __func__);
        return -EINVAL;
    }

    if (!IS_ALIGNED(request.payload, 64))
    {
        CARD_PRINTF(SXPF_ERROR, "%s: invalid user buffer alignment", __func__);
        return -EINVAL;
    }

    switch (request.type)
    {
    default:
        CARD_PRINTF(SXPF_ERROR, "%s: invalid alloc type requested (%u)",
                    __func__, request.type);
        return -EINVAL;

    case SXPF_ALLOC_HEAP:
    case SXPF_ALLOC_NV_DVP:
        init_buffer = declare_heap_buffer;
        break;

    case SXPF_ALLOC_NV_P2P:
#ifdef ENABLE_NV_P2P
        init_buffer = declare_cuda_buffer;
        break;
#else
        CARD_PRINTF(SXPF_ERROR, "%s: CUDA P2P support not compiled in",
                    __func__);
        return -EINVAL;
#endif
    }

    CARD_PRINTF(SXPF_DEBUG, "ENTER: %s", __func__);
    CARD_PRINTF(SXPF_DEBUG,
                "type=%d, header=0x%016llx, payload=0x%016llx, size=%u",
                request.type, request.header, request.payload, request.size);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
    ktime_get_raw_ts64(&before);
#else
    getrawmonotonic64(&before);
#endif

    down(&card->buf_alloc_lock);

    /* Allocate a DMA buffer (one page) for storing the image headers, which is
     * only mapped to kernel space. Headers for user-space-allocated buffers
     * will be copied between kernel und user-space, instead of being DMAed to
     * user-space directly.
     */
    if (!card->header_addr_cpu)
        card->header_addr_cpu =
            dma_alloc_attrs(&card->pci_device->dev, 4096,
                            &card->header_addr_bus, GFP_KERNEL, 0);

    if (!card->header_addr_cpu)
    {
        up(&card->buf_alloc_lock);

        CARD_PRINTF(SXPF_ERROR,
                    "couldn't allocate DMA memory for image headers");
        return -ENOMEM;
    }

    for (i = 0; i < card->numUserBuffers; i++)
    {
        unsigned            slot = card->numDrvBuffers + i;
        struct sxpf_buffer *buf = card->dma_buffer + slot;

        if (buf->file_owner == NULL && !atomic_read(&buf->hw_owned))
        {
            void *addr;

            down(&buf->lock);

            buf->magic = SXPF_BUFFER_MAGIC;
            buf->dma_dir = is_record(card) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
            buf->stream_type = SXPF_DATA_TYPE_VIDEO;
            INIT_LIST_HEAD(&buf->readers);
            atomic_set(&buf->hw_owned, 0);

            // init list of buffer segments
            INIT_LIST_HEAD(&buf->segments);
            buf->buf_size = 0;
            buf->attr = 0;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0))
            down_read(&current->mm->mmap_sem);
#else
            mmap_read_lock(current->mm);
#endif

            addr = init_buffer(buf, &request);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 8, 0))
            up_read(&current->mm->mmap_sem);
#else
            mmap_read_unlock(current->mm);
#endif

            if (!addr)
            {
                CARD_PRINTF(SXPF_ERROR, "failed mapping user buffer");  // -EIO
            }
            else
            {
                CARD_PRINTF(SXPF_DEBUG, "buf #%u provided by user @ 0x%016llx",
                            slot, (__u64)addr);

                request.slot = slot;        // return allocated slot to user

                buf->kernel_adr = addr;
                buf->file_owner = fh;
                buf->alloc_kind = request.type;

                list_move(&buf->owned_list, &fh->owned_buffers);

                CARD_PRINTF(SXPF_DEBUG, "User buffer #%d size: %d KB, addr: %p,"
                            " bus addr: %llx.", i, sxpfFrameSize / 1024,
                            buf->kernel_adr, (u64)(buf->bus_adr));
                n_pages = buf->nr_pages;
            }

            up(&buf->lock);
            break;
        }
    }

    up(&card->buf_alloc_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,18,0)
    ktime_get_raw_ts64(&after);
#else
    getrawmonotonic64(&after);
#endif

    if (i >= card->numUserBuffers)
    {
        res = -ENOMEM;

        CARD_PRINTF(SXPF_WARNING,
                    "no capture buffer available for user allocation");
    }
    else if (n_pages > 0)
    {
        res = copy_to_user((void __user *)arg, &request, sizeof(request));

        duration = after.tv_sec * NSEC_PER_SEC + after.tv_nsec
            - before.tv_sec * NSEC_PER_SEC - before.tv_nsec;

        CARD_PRINTF(SXPF_MESSAGE, "%s took %lluus (%lu pages)", __func__,
                    duration / 1000, n_pages);
    }

    CARD_PRINTF(SXPF_DEBUG, "LEAVE: %s", __func__);

    return res;
}


/** Disable FPGA stream logic for the channels passed in the channel_mask param
 *
 * @param fh            The file client to stop streaming for
 * @param channel_mask  Channels to disable. @see REG_CONTROL
 *
 * @return 0 on success
 * @return Negative errno on error
 */
static int sxpf_stream_stop(sxpf_file_hdl_t *fh, unsigned long channel_mask)
{
    sxpf_card_t     *card = fh->card;
    int             i, type;

    if (down_interruptible(&card->channel_lock))
    {
        return -EINTR;
    }

    channel_mask &= fh->channel_mask;

    if (channel_mask == 0)
    {
        // none of the given channel options were active for this file handle
        up(&card->channel_lock);

        CARD_PRINTF(SXPF_DEBUG, "None of the channels requested to stop in "
                    "mask %lx were active for this user",
                    channel_mask);

        return -EINVAL;
    }

    for (type = SXPF_DATA_TYPE_VIDEO; type <= SXPF_DATA_TYPE_META; type++)
    {
        int    *channel_users = (type == SXPF_DATA_TYPE_VIDEO)
            ? card->video_channel_users : card->i2c_channel_users;

        for (i = 0; i < card->props.num_channels; i++)
        {
            unsigned long   test_mask =
                (type == SXPF_DATA_TYPE_VIDEO)
                ? (SXPF_STREAM_VIDEO0 << i) : (SXPF_STREAM_I2C0 << i);

            // only if the tested channel i is
            //   a) requested to stop by user, and
            //   b) it was activated by this user
            if (channel_mask & test_mask & fh->channel_mask)
            {
                // disable channel for this client
                fh->channel_mask &= ~test_mask;

                // decrement user-count
                if (--channel_users[i])
                {
                    // don't stop channel, yet, if there are still other users
                    channel_mask &= ~test_mask;
                }
                else
                {
                    // disable scanline interrupts when stopping a video channel
                    if (type == SXPF_DATA_TYPE_VIDEO)
                        write_reg32(card, REG_SCANLINE_TRIG(i), -1u);
                }
            }
        }
    }

    sxpf_pciestream_stop(card, channel_mask);

    up(&card->channel_lock);

    CARD_PRINTF(SXPF_INFO, "Transmission stopped for channel mask 0x%lx",
                channel_mask);

    return 0;
}


/** Release a buffer from a file handle.
 *
 * If we are the last file releasing the slot in record mode, the buffer is
 * handed back to the FPGA for further reception.
 *
 * During playback the buffer is handed to the FPGA directly.
 *
 * @param fh    The file client to release the buffer slot from
 * @param slot  The buffer slot number/index
 *
 * @return 0 on success
 */
static int file_release_slot(sxpf_file_hdl_t *fh, int slot)
{
    sxpf_card_t         *card = fh->card;
    struct sxpf_buffer  *buf = card->dma_buffer + slot;
    sxpf_buffer_state_t *bs = fh->buffer_state + slot;
    u32                 transfer_size = buf->buf_size;
    sxpf_image_header_t *header = buf->kernel_adr;

    down(&buf->lock);

    if (card->mode == SXPF_REPLAYING)
    {
        // prevent DMA from over-running the buffer if user specified an
        // invalid playback size
        // note: the channel number to play on is encoded in bits 30..31, which
        // is why we mask them out in our size check.
        if ((buf->rx_size & 0x3fffffff) > buf->buf_size)
        {
            up(&buf->lock);

            return 1;
        }

        transfer_size = buf->rx_size;

        if (is_user_buf_slot(card, slot))
        {
            // user-provided buffer: copy header from user-space to CMA memory
            if (copy_from_user(header, (void __user *)buf->user_header,
                               sizeof(sxpf_image_header_t)))
            {
                up(&buf->lock);

                CARD_PRINTF(SXPF_ERROR, "Failed copying image header for user-"
                            "allocated replay buffer");
                return 1;
            }
        }

        // enter card_id and cam_id elements into the header so they can be
        // used for dispatch upon getting the buffer back from the hardware
        header->card_id = card->nr;

        // TODO FIXME for 8 channel support
        header->cam_id = transfer_size / (1 << 30); // top-most 2 bits = cam_id
    }
    else if (list_empty(&bs->received) && is_driver_buf_slot(card, slot))
    {
        up(&buf->lock);

        CARD_PRINTF(SXPF_ERROR,
                    "File tried to release a slot it hadn't received before!");
        return 1;
    }

    // unlink from list of buffer users
    list_del_init(&bs->received);

    // no more readers and still capturing?
    if (list_empty(&buf->readers) || card->mode == SXPF_REPLAYING)
    {
        // callee checks if buffer isn't already owned by HW...
        sxpf_hand_buffer_to_fpga(card, buf, transfer_size);
    }

    up(&buf->lock);

    return 0;
}


/** Release a frame slot that was previously filled with reveived video data
 *  and processed by user code.
 *
 * During playback mode this function is called by the user to send video data
 * to the FPGA.
 *
 * @param fh    The file handle
 * @param arg   The calling IOCTL's user argument
 *
 * @return  0 on success, negative on error
 */
static long ioctl_release_frame(sxpf_file_hdl_t *fh, unsigned long arg)
{
    sxpf_card_t         *card = fh->card;
    sxpf_frame_info_t   request;
    int                 slot;
    int                 ret = 0;

    if (copy_from_user(&request, (void __user *)arg, sizeof(request)))
        return -EIO;

    slot = request.slot;

    if (slot < 0 || slot >= fh->card->ringBufferSize + fh->card->i2cRingSize)
    {
        CARD_PRINTF(SXPF_ERROR, "Invalid frame to release: #%d", slot);

        ret = -EINVAL;
    }
    else
    {
        struct sxpf_buffer  *buf = card->dma_buffer + slot;

        if (request.target == SXPF_FREE_BUFFER)
        {
            if (buf->file_owner == fh)
            {
                // relinquish exclusive ownership of the buffer
                ret = free_owned_frame(card, buf);
            }
            else
            {
                ret = -EINVAL;  // buffer not owned by the requesting fh
            }

            CARD_PRINTF(SXPF_DEBUG, "request to free buf #%d %s", slot,
                        ret ? "failed" : "successful");
            return ret;
        }

        if (is_user_buf_slot(card, slot))
        {
            // make sure that the user-allocatable buffer is owned by the user
            if (buf->file_owner != fh)
            {
                CARD_PRINTF(SXPF_ERROR,
                            "frame #%d to be posted is not owned by caller",
                            slot);
                return -EINVAL;
            }

            if (atomic_read(&buf->hw_owned))
            {
                CARD_PRINTF(SXPF_ERROR, "File tried to release frame #%d, which"
                            " is still being accessed by DMA", slot);
                return 1;
            }
        }

        if (is_replay(card))
        {
            // on playback cards the buffer to be freed _must_ have been
            // previously allocated for exlusive access by the client...
            if (buf->file_owner != fh)
            {
                CARD_PRINTF(SXPF_ERROR,
                            "playback frame to free #%d is not owned by caller",
                            slot);
                return -EINVAL;
            }

            // ...and it must not currently be controlled by the FPGA when
            // sending new data
            if (request.data_size > 0 && atomic_read(&buf->hw_owned))
            {
                CARD_PRINTF(SXPF_ERROR,
                            "invalid playback frame to send #%d: DMA active!",
                            slot);
                return -EINVAL;
            }

            // check if data size fits in DMA buffer
            if ((request.data_size & 0x3fffffff) > card->props.buffer_size)
            {
                return -EINVAL;
            }
        }

        if (card->mode == SXPF_REPLAYING)
        {
            buf->rx_size = request.data_size;
        }

        if (file_release_slot(fh, slot))
            ret = -EINVAL;
    }

    CARD_PRINTF(SXPF_DEBUG, "release frame slot %d - %s", request.slot,
                ret ? "failed" : "success");

    return ret;
}


/** Deallocate a privately owned buffer from a file handle.
 *
 * This function takes care not to make a buffer available that is still in use
 * by the DMA engine.
 *
 * If a playback buffer is still controlled by the hardware we just clear its
 * owner pointer - it will then be released by the DMA ISR.
 *
 * If this is a user-space allocated record buffer, we wait here for the end of
 * the DMA, to prevent it overwriting memory that is already used otherwise.
 *
 * @param card  The card the buffer belongs to
 * @param buf   The buffer to free
 */
int free_owned_frame(sxpf_card_t *card, struct sxpf_buffer *buf)
{
    const char          *buf_state = "";
    int                  slot = (int)(buf - card->dma_buffer);
    sxpf_file_hdl_t     *fh = buf->file_owner;
    sxpf_buffer_state_t *bs = fh->buffer_state + slot;

    if (is_record(card))
    {
        if (!is_user_buf_slot(card, slot))
        {
            CARD_PRINTF(SXPF_ERROR, "%s: frame #%d not owned by user",
                        __func__, slot);
            return -EINVAL;
        }
        else
        {
            down(&card->buf_alloc_lock);

            down(&buf->lock);

            down(&fh->dlock);   // start changing pending data

            list_del_init(&bs->unread);   // needs fh->dlock
            list_del_init(&bs->received);

            list_del_init(&buf->owned_list);

            up(&fh->dlock);   // end changing pending data

            if (!atomic_read(&buf->hw_owned))
            {
                // call buffer's DMA unmap function
                free_buffer(card, buf);
            }

            list_del_init(&buf->owned_list);

            // release buffer back to free pool
            buf->file_owner = NULL;

            up(&buf->lock);

            up(&card->buf_alloc_lock);

            CARD_PRINTF(SXPF_DEBUG, "owned frame #%d freed", slot);
        }
    }
    else
    {
        down(&card->buf_alloc_lock);

        // make buffer available for new allocations
        if (atomic_read(&buf->hw_owned))
        {
            // put buffers that are still DMA'd at the end of the free list
            list_move_tail(&buf->owned_list, &card->free_play_buffers);
            buf_state = " (DMA pending)";
        }
        else
        {
            // buffers are not owned by the DMA engine anymore can be reused
            // immediately, so we put them at the beginning of the free list
            list_move(&buf->owned_list, &card->free_play_buffers);
        }

        // release buffer back to free pool
        buf->file_owner = NULL;

        // unblock clients waiting to allocate a playback buffer
        wake_up_interruptible(&card->play_buf_requesters);

        up(&card->buf_alloc_lock);

        CARD_PRINTF(SXPF_DEBUG, "playback frame #%d freed%s", slot, buf_state);
    }

    return 0;
}


static long ioctl_send_cmd_sync(sxpf_file_hdl_t *fh, unsigned long arg)
{
    sxpf_card_t         *card = fh->card;
    int                 ret = 0;
    sxpf_cmd_fifo_el_t  request;

    // TODO only copy argument data the user has requested
    if (copy_from_user(&request, (void __user *)arg, sizeof(request)))
        return -EIO;

    ret = sxpf_send_cmd(card, request.cmd_stat, request.args, request.num_args,
                        request.args, request.num_ret, request.timeout_ms);

    if (ret != SXPF_CMD_STATUS_RESPONSE_OK)
        return ret;

    return copy_to_user((void __user *)arg, &request, sizeof(request));
}


static long ioctl_read_register(sxpf_card_t* card, unsigned long arg)
{
    sxpf_rw_register_t  rw;

    if (copy_from_user(&rw, (void __user *)arg, sizeof(rw)))
        return -EIO;

    if (rw.bar > 5)
        return -EINVAL;
    if (rw.offs + rw.size > card->pci_bar_size[rw.bar])
        return -EINVAL;

    switch (rw.size)
    {
    case 1:
        rw.data = read_reg8_bar(card, rw.bar, rw.offs);
        break;

    case 2:
        if (rw.offs & 1)
            return -EINVAL;
        rw.data = read_reg16_bar(card, rw.bar, rw.offs);
        break;

    case 4:
        if (rw.offs & 3)
            return -EINVAL;
        rw.data = read_reg32_bar(card, rw.bar, rw.offs);
        break;

    default:
        return -EINVAL;
    }

    return copy_to_user((void __user *)arg, &rw, sizeof(rw));
}


static long ioctl_write_register(sxpf_card_t* card, unsigned long arg)
{
    sxpf_rw_register_t  rw;

    if (copy_from_user(&rw, (void __user *)arg, sizeof(rw)))
        return -EIO;

    if (rw.bar > 5)
        return -EINVAL;
    if (rw.offs + rw.size > card->pci_bar_size[rw.bar])
        return -EINVAL;

    switch (rw.size)
    {
    case 1:
        write_reg8_bar(card, rw.bar, rw.offs, rw.data);
        break;

    case 2:
        if (rw.offs & 1)
            return -EINVAL;
        write_reg16_bar(card, rw.bar, rw.offs, rw.data);
        break;

    case 4:
        if (rw.offs & 3)
            return -EINVAL;
        write_reg32_bar(card, rw.bar, rw.offs, rw.data);
        break;

    default:
        return -EINVAL;
    }

    return 0;
}


static u32 TRG_REG(u32 ch, u32 reg)
{
    return ch == 0 ? reg : reg - REG_TRIG_CONTROL + REG_TRIG_BASE_CH(ch);
}

static long ioctl_read_config(sxpf_card_t* card, unsigned long arg)
{
    sxpf_config_t   cfg;
    u32             ch;
    u32             max_channel = 0;

    if (copy_from_user(&cfg, (void __user *)arg, sizeof(cfg)))
        return -EIO;

    ch = cfg.channel;
    if (has_feature(card, SXPF_FEATURE_IND_TRIG))
    {
        max_channel = card->props.num_channels - 1;
    }
    if (ch > max_channel)
        return -EINVAL;

    cfg.trig_mode         = read_reg32(card, TRG_REG(ch, REG_TRIG_CONTROL));
    cfg.trig_period       = read_reg32(card, TRG_REG(ch, REG_TRIG_PERIOD));
    cfg.trig_length       = read_reg32(card, TRG_REG(ch, REG_TRIG_LENGTH));
    cfg.trig_delay        = read_reg32(card, TRG_REG(ch, REG_TRIG_DELAY));
    cfg.trig_ext_length   = read_reg32(card, TRG_REG(ch, REG_TRIG_EXT_LENGTH));
    cfg.trig_in_mult      = read_reg32(card, TRG_REG(ch, REG_TRIG_IN_MULT));
    cfg.trig_user_div     = read_reg32(card, TRG_REG(ch, REG_TRIG_USER_DIV));
    cfg.trig_out_div      = read_reg32(card, TRG_REG(ch, REG_TRIG_OUT_DIV));
    cfg.trig_source_ext   =
        (card->control_reg & CTRL_EXT_TRIG_SOURCE) != 0;
    cfg.pps_source_ext    =
        (card->control_reg & CTRL_UNIX_TS_SYNC_SEL) != 0;
    cfg.timestamp_mode    = read_reg32(card, TRG_REG(ch, REG_TIMESTAMP_CONTROL));
    cfg.timestamp_offset  = read_reg32(card, TRG_REG(ch, REG_TIMESTAMP_OFFSET));
    cfg.trig_secondary_in = read_reg32(card, TRG_REG(ch, REG_SECONDARY_INPUT));

    return copy_to_user((void __user *)arg, &cfg, sizeof(cfg));
}


static long ioctl_write_config(sxpf_card_t* card, unsigned long arg)
{
    sxpf_config_t   cfg;
    u32             reg_control;
    u32             ch;
    u32             max_channel = 0;
    unsigned long   iflags;

    if (copy_from_user(&cfg, (void __user *)arg, sizeof(cfg)))
        return -EIO;

    ch = cfg.channel;
    if (has_feature(card, SXPF_FEATURE_IND_TRIG))
    {
        max_channel = card->props.num_channels - 1;
    }
    if (ch > max_channel)
        return -EINVAL;

    write_reg32(card, TRG_REG(ch, REG_TRIG_PERIOD),       cfg.trig_period);
    write_reg32(card, TRG_REG(ch, REG_TRIG_LENGTH),       cfg.trig_length);
    write_reg32(card, TRG_REG(ch, REG_TRIG_DELAY),        cfg.trig_delay);
    write_reg32(card, TRG_REG(ch, REG_TRIG_EXT_LENGTH),   cfg.trig_ext_length);
    write_reg32(card, TRG_REG(ch, REG_TRIG_IN_MULT),      cfg.trig_in_mult);
    write_reg32(card, TRG_REG(ch, REG_TRIG_USER_DIV),     cfg.trig_user_div);
    write_reg32(card, TRG_REG(ch, REG_TRIG_OUT_DIV),      cfg.trig_out_div);
    write_reg32(card, TRG_REG(ch, REG_TRIG_CONTROL),      cfg.trig_mode);
    write_reg32(card, TRG_REG(ch, REG_TIMESTAMP_CONTROL), cfg.timestamp_mode);
    write_reg32(card, TRG_REG(ch, REG_TIMESTAMP_OFFSET),  cfg.timestamp_offset);
    write_reg32(card, TRG_REG(ch, REG_SECONDARY_INPUT),   cfg.trig_secondary_in);

    if (ch == 0)
    {
        spin_lock_irqsave(&card->control_lock, iflags);

        reg_control = card->control_reg &
                      ~(CTRL_EXT_TRIG_SOURCE | CTRL_UNIX_TS_SYNC_SEL);
        reg_control |= (cfg.trig_source_ext ? CTRL_EXT_TRIG_SOURCE : 0) |
                       (cfg.pps_source_ext ? CTRL_UNIX_TS_SYNC_SEL : 0);
        set_control_reg(card, reg_control);

        spin_unlock_irqrestore(&card->control_lock, iflags);
    }

    return 0;
}


static long ioctl_trigger_exposure(sxpf_card_t* card, unsigned long arg)
{
    sxpf_trigger_exposure_t manual_trigger;
    u32                     ch;
    u32                     max_channel = 0;

    if (copy_from_user(&manual_trigger, (void __user *)arg,
                       sizeof(manual_trigger)))
        return -EIO;

    ch = manual_trigger.channel;
    if (has_feature(card, SXPF_FEATURE_IND_TRIG))
    {
        max_channel = card->props.num_channels - 1;
    }
    if (ch > max_channel)
        return -EINVAL;

    write_reg32(card, TRG_REG(ch, REG_TRIG_LENGTH), manual_trigger.exposure);

    return 0;
}


s64 sxpf_timestamp(sxpf_card_t* card, int is_secondary)
{
    u32     hi, hi2, lo;
    u32     reg_ts_lo = is_secondary ? REG_TIMESTAMP_SEC_LO : REG_TIMESTAMP_LO;
    u32     reg_ts_hi = is_secondary ? REG_TIMESTAMP_SEC_HI : REG_TIMESTAMP_HI;

    hi = read_reg32(card, reg_ts_hi);

    do
    {
        hi2 = hi;
        lo = read_reg32(card, reg_ts_lo);
        hi = read_reg32(card, reg_ts_hi);

    } while (hi != hi2);    // ensure hi hasn't changed

    return ((s64)hi << 32) | lo;
}


static long ioctl_get_timestamp(sxpf_card_t* card, unsigned long arg,
                                int is_secondary)
{
    s64     time = sxpf_timestamp(card, is_secondary);

    return copy_to_user((void __user *)arg, &time, sizeof(time));
}


static void hw_ts_set_sync(sxpf_timestamp_sync_t *sync, s64 hw_ts,
                           struct timespec64 *before, struct timespec64 *after)
{
    sync->timestamp = hw_ts;

    sync->slack =
        (after->tv_sec - before->tv_sec) * NSEC_PER_SEC +
        after->tv_nsec - before->tv_nsec;

    sync->systemTime =
        before->tv_sec * NSEC_PER_SEC + before->tv_nsec + sync->slack / 2;
}


static int sxpf_sample_hw_ts(sxpf_card_t *card, sxpf_timestamp_sync_t *sync)
{
    struct timespec64       t[11];
    u32                     hw_ts_lowword[10];
    s64                     hw_ts_highword;
    int                     ff_counter = 0;
    int                     i;

    // read low word of hardware timestamp 10 times
    // use read_reg32_int() to prevent excessive debug logging
    get_clock_monotonic_raw(t + 0);
    hw_ts_lowword[0] = read_reg32_int(card, REG_TIMESTAMP_LO);
    get_clock_monotonic_raw(t + 1);
    hw_ts_lowword[1] = read_reg32_int(card, REG_TIMESTAMP_LO);
    get_clock_monotonic_raw(t + 2);
    hw_ts_lowword[2] = read_reg32_int(card, REG_TIMESTAMP_LO);
    get_clock_monotonic_raw(t + 3);
    hw_ts_lowword[3] = read_reg32_int(card, REG_TIMESTAMP_LO);
    get_clock_monotonic_raw(t + 4);
    hw_ts_lowword[4] = read_reg32_int(card, REG_TIMESTAMP_LO);
    get_clock_monotonic_raw(t + 5);
    hw_ts_lowword[5] = read_reg32_int(card, REG_TIMESTAMP_LO);
    get_clock_monotonic_raw(t + 6);
    hw_ts_lowword[6] = read_reg32_int(card, REG_TIMESTAMP_LO);
    get_clock_monotonic_raw(t + 7);
    hw_ts_lowword[7] = read_reg32_int(card, REG_TIMESTAMP_LO);
    get_clock_monotonic_raw(t + 8);
    hw_ts_lowword[8] = read_reg32_int(card, REG_TIMESTAMP_LO);
    get_clock_monotonic_raw(t + 9);
    hw_ts_lowword[9] = read_reg32_int(card, REG_TIMESTAMP_LO);
    get_clock_monotonic_raw(t + 10);

    sync->slack = 1 << 30;

    // find HW timestamp with tightest bound of enclosing CPU timestamps
    for (i = 0; i < 10; i++)
    {
        sxpf_timestamp_sync_t   temp_sync;
        s64                     hw_ts;

        // get cached high word of hardware timestamp
        hw_ts_highword = card->hw_ts_highword;

        // if low-word of hardware time overflowed...
        if (hw_ts_lowword[i] + hw_ts_highword < card->hw_ts_ref->hw_time)
        {
            // ...then high word needs to be incremented
            hw_ts_highword += 0x100000000LL;
        }

        // get the full-resolution hardware timestamp
        hw_ts = hw_ts_lowword[i] + hw_ts_highword;

        hw_ts_set_sync(&temp_sync, hw_ts, t + i, t + i + 1);

        if (temp_sync.slack < sync->slack)
           *sync = temp_sync;

        if (hw_ts_lowword[i] == 0xffffffff)
            ff_counter++;
        else
            ff_counter = 0;
    }

    if (ff_counter > 1)
    {
        // if more than 1 of the last polled values above returned ff we assume
        // that the PCIe bus is broken
        disable_card(card, "PCIe broken - timersync read 0xffffffff");
        return 1;
    }

    // update cached HW timestamp high word
    card->hw_ts_highword = hw_ts_highword;

    return 0;
}


static void sxpf_hw_timer_sync_init(sxpf_card_t *card)
{
    // set up initial sync point
    sxpf_timestamp_sync_t   local_sync;
    struct hw_ts_sync      *first_sync;
    s64                     hw_ts = sxpf_timestamp(card, 0);

    card->hw_ts_idx = 0;
    card->hw_ts_highword = hw_ts & 0xffffffff00000000;

    first_sync = card->hw_ts_buf + card->hw_ts_idx;
    card->hw_ts_ref = first_sync;
    card->hw_ts_ref->hw_time = hw_ts;   // needed in sxpf_sample_hw_ts()

    first_sync->hw_clocks = 0;
    first_sync->pc_ns = 0;

    if (sxpf_sample_hw_ts(card, &local_sync))
        return;

    first_sync->pc_time = local_sync.systemTime;
    first_sync->hw_time = local_sync.timestamp;

    rcu_assign_pointer(card->hw_ts_ref, first_sync);

    mod_timer(&card->hw_tstamp_timer, jiffies + msecs_to_jiffies(500));
}


static void sxpf_hw_timer_sync_update(sxpf_card_t *card)
{
    s64                     hw_ts_highword;
    struct hw_ts_sync      *next_sync;
    sxpf_timestamp_sync_t   local_sync;
    unsigned long           flags;

    if (atomic_read(&card->io_disabled))
        return;

    if (sxpf_sample_hw_ts(card, &local_sync))
        return;

    // fill in extrapolation parameters for next interval
    card->hw_ts_idx = (card->hw_ts_idx + 1) % 4;
    next_sync = card->hw_ts_buf + card->hw_ts_idx;

    next_sync->hw_time = local_sync.timestamp;
    next_sync->pc_time = local_sync.systemTime;
    next_sync->pc_ns = local_sync.systemTime - card->hw_ts_ref->pc_time;
    next_sync->hw_clocks = local_sync.timestamp - card->hw_ts_ref->hw_time;

    if (card->hw_ts_ref->pc_ns > 0)
    {
        s64 sim_ts = card->hw_ts_ref->hw_time +
            mul_u64_u32_div(next_sync->pc_ns, card->hw_ts_ref->hw_clocks,
                            card->hw_ts_ref->pc_ns);

        CARD_PRINTF(SXPF_DEBUG_EXTRA,
                    "HW timer sync update: slack=%lld, acc=%lld",
                    local_sync.slack, sim_ts - next_sync->hw_time);
    }
    else
    {
        CARD_PRINTF(SXPF_DEBUG_EXTRA, "HW timer sync setup: slack=%lld",
                    local_sync.slack);
    }

    // let next readers use new params
#if USE_GPL_SYMBOLS
    rcu_assign_pointer(card->hw_ts_ref, next_sync);
#else
    spin_lock_irqsave(&card->hw_ts_lock, flags);

    card->hw_ts_ref = next_sync;

    spin_unlock_irqrestore(&card->hw_ts_lock, flags);
#endif

    // restart timer for next sync
    mod_timer(&card->hw_tstamp_timer, jiffies + msecs_to_jiffies(500));
}


static long ioctl_get_timestamp_fast(sxpf_card_t* card, unsigned long arg,
                                     int is_secondary)
{
    struct hw_ts_sync  *sync;
    s64                 hw_ts;

#if USE_GPL_SYMBOLS
    rcu_read_lock();

    sync = rcu_dereference(card->hw_ts_ref);
#else
    unsigned long       flags;

    spin_lock_irqsave(&card->hw_ts_lock, flags);

    sync = card->hw_ts_ref;
#endif

    if (sync->pc_ns == 0 || is_secondary)
    {
        // read hardware timestamp registers if extrapolation params aren't
        // ready, yet, or if secondary timestamp is queried
        hw_ts = sxpf_timestamp(card, is_secondary);
    }
    else
    {
        // extrapolate primary hardware timestamp from stored sync point and
        // current system time
        struct timespec64   now;
        s64                 now_ns;

        get_clock_monotonic_raw(&now);

        // get elapsed number of nanoseconds since last sync update
        now_ns = now.tv_sec * NSEC_PER_SEC + now.tv_nsec - sync->pc_time;

        // extrapolate hardware timestamp
        hw_ts = sync->hw_time + mul_u64_u32_div(now_ns, sync->hw_clocks,
                                                sync->pc_ns);
    }

#if USE_GPL_SYMBOLS
    rcu_read_unlock();
#else
    spin_unlock_irqrestore(&card->hw_ts_lock, flags);
#endif

    return copy_to_user((void __user *)arg, &hw_ts, sizeof(hw_ts));
}


static long ioctl_timestamp_sync(sxpf_card_t* card, unsigned long arg)
{
    sxpf_timestamp_sync_t   req;
    struct timespec64       before, after;
    void                  (*get_ts_func)(struct timespec64 *ts);
    int                     is_secondary;
    s64                     hw_ts;

    if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
        return -EIO;

    switch (req.clockSelect & SXPF_SYSTEM_CLOCK_MASK)
    {
    default:
        return -EINVAL;

    case SXPF_CLOCK_REALTIME:
        get_ts_func = get_clock_realtime;
        break;

    case SXPF_CLOCK_MONOTONIC:
#if USE_GPL_SYMBOLS
        get_ts_func = ktime_get_ts64;
        break;
#else
        return -EINVAL;
#endif

    case SXPF_CLOCK_MONOTONIC_RAW:
        get_ts_func = get_clock_monotonic_raw;
        break;
    }

    is_secondary = req.clockSelect & SXPF_CLOCK_HW_SECONDARY;

    mb();

    get_ts_func(&before);
    hw_ts = sxpf_timestamp(card, is_secondary);
    get_ts_func(&after);

    mb();

    hw_ts_set_sync(&req, hw_ts, &before, &after);

    return copy_to_user((void __user *)arg, &req, sizeof(req));
}


static long ioctl_set_user_sequence(sxpf_card_t* card, unsigned long arg)
{
    sxpf_user_sequence_t    req;
    int                     ret, i;

    union
    {
        u8      b[512];
        u32     w[128];
    }                       ram_new;

    if (card->user_seq_size < 0)
    {
        // Plasma hasn't been queried, yet, if it supports execution of the
        // scanline-triggered user event, so we do that now by asking for the
        // location and size of the user sequence RAM
        u32 cmd_stat =
            SXPF_CMD_USER_EVENT_GET_LOCATION | SXPF_CMD_STATUS_REQUEST;
        u8  args[8];

        ret = sxpf_send_cmd(card, cmd_stat, args, 0, args, 8, 1000);

        if (ret == SXPF_CMD_STATUS_RESPONSE_UNKNOWN_CMD)
        {
            // no support
            card->user_seq_size = 0;
        }
        else
        {
            // support detected
            card->user_seq_offset = 0x10000 + // Plasma RAM start in BAR2
                (args[0] | (args[1] << 8) | (args[2] << 16) | (args[3] << 24));
            card->user_seq_size =
                args[4] | (args[5] << 8) | (args[6] << 16) | (args[7] << 24);

            CARD_PRINTF(SXPF_DEBUG,
                        "User sequence RAM at offset 0x%04x, size %d",
                        card->user_seq_offset, card->user_seq_size);
        }
    }

    if (card->user_seq_size < 512)             // we want at least 512 bytes...
    {
        CARD_PRINTF(SXPF_ERROR, "Card has no user event support.");
        return -EINVAL;
    }

    if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
        return -EIO;

    if (req.ram_offset + req.seq_size > 512 || // ...and we use at most as much
        req.seq_size > 128)                    // size limit for one sequence
    {
        CARD_PRINTF(SXPF_ERROR, "User sequence out of bounds: 0x%04x+0x%x",
                    req.ram_offset, req.seq_size);
        return -EINVAL;
    }

    // create a RAM image as it will look like after insertion of the new
    // sequence
    memcpy(ram_new.b, card->user_seq_image.b, sizeof(ram_new.b));
    memcpy(ram_new.b + req.ram_offset, req.sequence, req.seq_size);

    // write each word that differs between old and new RAM image to the user
    // RAM located in the Plasma code/data space
    for (i = 0; i < req.seq_size; i += sizeof(u32))
    {
        u32     offs = (req.ram_offset + i) / sizeof(u32);

        if (card->user_seq_image.w[offs] != ram_new.w[offs] ||
            sxpfDebugLevel >= SXPF_DEBUG)   // debug: show all sequence updates
        {
            write_reg32_bar(card, PLASMA_REGION,
                            card->user_seq_offset + offs * sizeof(u32),
                            ram_new.w[offs]);
            card->user_seq_image.w[offs] = ram_new.w[offs];
        }
    }

    return 0;
}


static long ioctl_enable_user_sequence(sxpf_card_t* card, unsigned long arg)
{
    sxpf_enable_user_sequence_t req;
    int                         ret;
    u8                          args[4];
    u32                         cmd_stat;
    const char                  *msg;

    if (card->user_seq_size < 512)
    {
        // user sequence support hasn't been queried from the Plasma, yet, or
        // no support is available
        return -EINVAL;
    }

    if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
        return -EIO;

    if (req.ram_offset >= card->user_seq_size - 4 || req.channel > 3)
        return -EINVAL;

    if (req.scanline > 0)
    {
        args[0] = req.ram_offset & 255;
        args[1] = ((req.ram_offset >> 8) & 15) | ((req.repeat ? 1 : 0) << 4);

        cmd_stat = SXPF_CMD_USER_EVENT_SET;
    }
    else
    {
        args[0] = 0;
        args[1] = 0;

        cmd_stat = SXPF_CMD_USER_EVENT_REMOVE;
    }

    args[2] = req.channel;
    args[3] = 3;    // event ID: "Active Line IRQ"

    ret = sxpf_send_cmd(card, cmd_stat | SXPF_CMD_STATUS_REQUEST,
                        args, 4, args, 0, 1000);

    msg = (ret == SXPF_CMD_STATUS_RESPONSE_OK) ? "Succeeded" : "Failed";

    if (req.scanline > 0)
    {
        CARD_PRINTF(SXPF_DEBUG,
                    "%s enabling user sequence for channel #%d: line %d",
                    msg, req.channel, req.scanline);
    }
    else
    {
        CARD_PRINTF(SXPF_DEBUG, "%s disabling user sequence for channel #%d",
                    msg, req.channel);
    }

    if (ret != SXPF_CMD_STATUS_RESPONSE_OK)
        return ret;

    write_reg32(card, REG_SCANLINE_TRIG(req.channel), req.scanline);

    return 0;
}


static int ioctl_get_buf_header(sxpf_card_t *card, unsigned long arg)
{
    sxpf_get_buf_hdr_t  req;
    int                 slot;
    int                 ret;

    if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
        return -EIO;

    slot = req.slot;

    if (slot < 0 || slot >= card->ringBufferSize + card->i2cRingSize)
    {
        CARD_PRINTF(SXPF_ERROR, "Invalid frame to get buf header for: #%d",
                    slot);

        ret = -EINVAL;
    }
    else
    {
        struct sxpf_buffer  *buf = card->dma_buffer + slot;

        down(&buf->lock);

        memcpy(req.header, buf->hdr_cpy, sizeof(sxpf_image_header_t));

        ret = copy_to_user((void __user *)arg, &req, sizeof(req));

        up(&buf->lock);
    }

    return ret;
}


int ioctl_init_i2c_slave(sxpf_file_hdl_t *fh, unsigned long arg)
{
    sxpf_card_t *card = fh->card;
    sxpf_i2c_slave_init_t request;
    __u8 idx = 0;
    uint32_t RegValue = 0;

    if (!has_feature(card, SXPF_FEATURE_I2C_SLAVES))
        return -EOPNOTSUPP;

    if (copy_from_user(&request, (void __user *)arg, sizeof(sxpf_i2c_slave_init_t)))
        return -EIO;

    // possible i2c channels are 0...7
    if (request.channel > 7)
        return -EINVAL;

    down(&card->i2c_slave_lock);

    // check which i2c core is not initialized and has no slave address
    while (card->i2c_slaves[idx].i2cSlaveAddr != 0 &&
                            idx < SXPF_MAX_I2C_SLAVES)
    {
        idx++;
    }

    switch (idx)
    {
    case 0: card->i2c_slaves[idx].i2cBaseAddr = REG_I2C_SLAVE_0_BASE; break;
    case 1: card->i2c_slaves[idx].i2cBaseAddr = REG_I2C_SLAVE_1_BASE; break;
    case 2: card->i2c_slaves[idx].i2cBaseAddr = REG_I2C_SLAVE_2_BASE; break;
    case 3: card->i2c_slaves[idx].i2cBaseAddr = REG_I2C_SLAVE_3_BASE; break;
    case 4: card->i2c_slaves[idx].i2cBaseAddr = REG_I2C_SLAVE_4_BASE; break;
    case 5: card->i2c_slaves[idx].i2cBaseAddr = REG_I2C_SLAVE_5_BASE; break;
    case 6: card->i2c_slaves[idx].i2cBaseAddr = REG_I2C_SLAVE_6_BASE; break;
    case 7: card->i2c_slaves[idx].i2cBaseAddr = REG_I2C_SLAVE_7_BASE; break;
    default:
        up(&card->i2c_slave_lock);
        return -ENODEV;
    }
    request.handle = idx;

    card->i2c_slaves[idx].hClient = fh;
    card->i2c_slaves[idx].i2cSlaveAddr = request.dev_id; // mark core as used

    up(&card->i2c_slave_lock);

    CARD_PRINTF(SXPF_DEBUG, "connect i2c slave %d", idx);

    // connect i2c slave to the given channel.
    write_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
        REG_I2C_SLAVE_GPO_REG_OFFSET, 0x00000001 << request.channel);

    // Set slave address to which this device should respond.
    write_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
        REG_I2C_SLAVE_ADR_REG_OFFSET, card->i2c_slaves[idx].i2cSlaveAddr);

    // Mask off all interrupts, each is enabled when needed.
    write_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
        REG_I2C_SLAVE_IIER_OFFSET, 0x00);

    // Clear all interrupts by reading and rewriting exact value back.
    RegValue = read_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
        REG_I2C_SLAVE_IISR_OFFSET);
    write_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
        REG_I2C_SLAVE_IISR_OFFSET, RegValue);

    // Enable the i2c slave device.
    write_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
        REG_I2C_SLAVE_CR_REG_OFFSET, 0x01);

    // Set Rx FIFO Occupancy depth to 7+1 ->
    //  because "sxpf_event_t.extra" provides place for 8 data bytes
    write_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
        REG_I2C_SLAVE_RFD_REG_OFFSET, 0x07);

    // flush Tx- Fifo
    RegValue = read_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
        REG_I2C_SLAVE_CR_REG_OFFSET);
    write_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
        REG_I2C_SLAVE_CR_REG_OFFSET, RegValue | 0x00000002);

    // enable i2c slave irq: NAS(6), AAS(5), RxFifo_Full(3),
    // TxFifo_Empty(2), TxCmplt(1) ==> 01101110
    write_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
        REG_I2C_SLAVE_IIER_OFFSET, 0x20);

    // enable the irq input signal of the Intc
    RegValue = read_reg32_bar(card, 1, 0x00009E08);
    write_reg32_bar(card, 1, 0x00009E08, RegValue | (1 << idx));
    // Set the Global Interrupt Enable
    write_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
        REG_I2C_SLAVE_DGIER_OFFSET, 0x80000000);

    CARD_PRINTF(SXPF_DEBUG, "i2c slave %d with adr %d initialised",
                request.handle, request.dev_id);

    return request.handle;
}


int ioctl_remove_i2c_slave(sxpf_file_hdl_t *fh, unsigned long arg)
{
    sxpf_card_t *card = fh->card;
    uint8_t idx = arg;
    uint32_t RegValue = 0;

    if (!has_feature(card, SXPF_FEATURE_I2C_SLAVES))
        return -EOPNOTSUPP;

    if (idx >= SXPF_MAX_I2C_SLAVES)
        return -EINVAL;

    if (card->i2c_slaves[idx].hClient != fh)
        return -EINVAL;

    down(&card->i2c_slave_lock);

    // Reset the Global Interrupt Enable
    write_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
                    REG_I2C_SLAVE_DGIER_OFFSET, 0x00000000);

    // disconnect i2c slave from Intc: acknowledge (IAR) and disable (CIE) irq
    write_reg32_bar(card, 1, 0x00009E14, (1 << idx));
    write_reg32_bar(card, 1, 0x00009E0C, (1 << idx));

    // disable i2c slave device: reset EN bit in CR register.
    // Whatever i2c transfer may be runnung, the i2c slave core
    // releases the sda/scl line in any case.
    write_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
                    REG_I2C_SLAVE_CR_REG_OFFSET, 0x00);

    // disconnect i2c slave from i2c bus
    write_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
                    REG_I2C_SLAVE_GPO_REG_OFFSET, 0x00000000);

    card->i2c_slaves[idx].i2cSlaveAddr = 0;
    card->i2c_slaves[idx].hClient = NULL;

    up(&card->i2c_slave_lock);

    CARD_PRINTF(SXPF_DEBUG, "i2c slave %d removed", idx);

    return 0;
}


int ioctl_i2c_slave_tx_ack(sxpf_card_t *card, unsigned long arg)
{
    sxpf_i2c_slave_tx_ack_t request;
    uint32_t i = 0;
    uint32_t i2c_slave_isr = 0;
    uint32_t reg_addr = 0;

    if (!has_feature(card, SXPF_FEATURE_I2C_SLAVES))
        return -EOPNOTSUPP;

    if (copy_from_user(&request, (void __user *)arg,
                                        sizeof(sxpf_i2c_slave_tx_ack_t)))
        return -EIO;

    for (i = 0; i < request.tx_size; i++)
    {
        write_reg32_bar(card, 1, card->i2c_slaves[request.handle].i2cBaseAddr +
            REG_I2C_SLAVE_DTR_REG_OFFSET, request.tx_data[i]);
    }

    // clear Tx_Fifo_empty irq
    reg_addr = card->i2c_slaves[request.handle].i2cBaseAddr +
                                    REG_I2C_SLAVE_IISR_OFFSET;
    i2c_slave_isr = read_reg32_bar(card, 1, reg_addr);
    write_reg32_bar(card, 1, card->i2c_slaves[request.handle].i2cBaseAddr +
        REG_I2C_SLAVE_IISR_OFFSET, i2c_slave_isr & 0x04);

    // enable NAS, Tx_Fifo_empty & Tx_Cmplt ==> 01000110
    write_reg32_bar(card, 1, card->i2c_slaves[request.handle].i2cBaseAddr +
        REG_I2C_SLAVE_IIER_OFFSET, 0x44);

    return i;
}


int sxpf_get_i2c_slave_rx_data( sxpf_card_t *card, uint8_t i2c_slave_nr,
                                uint8_t* buf, uint8_t count)
{
    uint8_t idx = i2c_slave_nr;
    uint8_t i = 0;

    if (buf == NULL)
        return -EINVAL;

    for ( i = 0; i < count; i++)
    {
        *buf = read_reg32_bar(card, 1, card->i2c_slaves[idx].i2cBaseAddr +
            REG_I2C_SLAVE_DRR_REG_OFFSET);
        buf++;
    }

    return 0;
}


/** free a DMA buffer that was allocated by this driver */
static void free_driver_buffer(struct sxpf_buffer *buf)
{
    sxpf_card_t     *card = buf->card;
    buffer_segment  *seg, *tmp;

    if (!card)
        return;

    // free all buffer memory segments
    free_sg_segments(buf);
}


static void free_buffer(sxpf_card_t *card, struct sxpf_buffer *buf)
{
    if (!buf->kernel_adr)
    {
        CARD_PRINTF(SXPF_ERROR, "frame #%d to be freed has no kernel_adr",
                    (int)(buf - card->dma_buffer));
        return;
    }

    if (!buf->dma_unmap)
    {
        CARD_PRINTF(SXPF_ERROR, "INTERNAL error: unknown allocation "
                    "method while trying to free DMA buffer #%d!",
                    (int)(buf - card->dma_buffer));
        return;
    }

    // possible functions that can be called via the dma_unmap function pointer:
    // - free_cuda_buffer
    // - free_driver_buffer
    // - free_heap_buffer
    buf->dma_unmap(buf);

    // reset buffer state
    atomic_dec(&card->n_bufs_in_sw);
    buf->alloc_kind = SXPF_BUF_UNUSED;
    buf->kernel_adr = NULL; // prevent double-free when unloading the driver
}


/**  Free DMA buffers
 */
static void sxpf_free_dma_buffers(sxpf_card_t *card)
{
    int i;
    int done_reset = 0;
    u32 ctrl = card->control_reg;

    WARN_ON(ctrl & CTRL_RUN);    // bug, if streaming is still active

    // release VIDEO DMA buffers
    for (i = 0; i < card->ringBufferSize + card->i2cRingSize; ++i)
    {
        struct sxpf_buffer  *buf = card->dma_buffer + i;

        if (buf->kernel_adr != NULL)
        {
            int was_hw_owned = atomic_cmpxchg(&buf->hw_owned, 0 , 1);
            // note: now the buffer is counted as owned by HW, in order for it
            //       not to be handed back to the HW, again, in another thread

            // must not free buffer that DMA might still be transferring to/from
            if (was_hw_owned && ! done_reset)
            {
                CARD_PRINTF(SXPF_ERROR,
                            "trying SOFT_RESET to force DMA to stop");

                // reset FPGA to forcibly stop any pending DMA transfers
                set_control_reg(card, ctrl | CTRL_SOFT_RESET);
                done_reset = 1;
                // Note: This has the (unwanted) side-effect of re-initializing
                //       the FPGA's memory controllers and adapter I2C busses!

                // wait to be sure all DMA activity has stopped
                mdelay(2);
                // (RAM and I2C init are allowed to take longer)
            }

            free_buffer(card, buf);
        }
    }

    if (card->sge_pool)
    {
        dma_pool_destroy(card->sge_pool);   // release SG descriptor pool
        card->sge_pool = NULL;
    }

    if (card->header_addr_cpu)
        dma_free_attrs(&card->pci_device->dev, 4096, card->header_addr_cpu,
                       card->header_addr_bus, 0);

    memset(&card->dma_buffer, 0, sizeof(card->dma_buffer)); // clear descriptors
}


/** Get pool of hardware scatter/gather descriptors of the passed card
 *
 * This function performs a lazy initialization of the pool. Hardware without
 * S/G support won't need this pool.
 */
static struct dma_pool *get_sgd_dma_pool(sxpf_card_t *card)
{
    if (!card->sge_pool)
    {
        struct pci_dev *pci_device = card->pci_device;
        char            name[32];

        sprintf(name, "sxpf%d_sgd_pool", card->nr);

#define SGD_SIZE    sizeof(*((struct buf_seg_s*)NULL)->sg_entry)

        card->sge_pool = dma_pool_create(name, &pci_device->dev,
                                         SGD_SIZE, SGD_SIZE, PAGE_SIZE);
        if (!card->sge_pool)
        {
            CARD_PRINTF(SXPF_ERROR,
                        "Failed to allocate SG descriptor pool");
            return NULL;
        }
    }

    return card->sge_pool;
}


/** For a S/G segment that has kernel address, bus address and size already set
 *  allocate and initialize the actual scatter/gather descriptor that is read
 *  by the FPGA.
 *
 *  If this is the first S/G segment added to the buffer, we also set the
 *  buffer's kernel address (for accessing the header) and bus address (for
 *  posting to the FPGA's command queue).
 */
static void add_hw_sg_descriptor(struct sxpf_buffer *buf, buffer_segment *seg)
{
    sxpf_card_t *card = buf->card;
    int          is_first_segment = (seg->list.prev == &buf->segments);

    if (has_feature(card, SXPF_FEATURE_SG_PLDA))
    {
        // set up scatter-gather list entry for PLDA ezDMA
        sg_desc_plda   *sge = &seg->sg_entry->plda;

        // fill in PLDA ezDMA scatter list descriptor
        sge->phys_addr_lo = (seg->bus_adr & 0xffffffff);
        sge->phys_addr_hi = seg->bus_adr >> 32;
        sge->chunk_size   = seg->seg_size;
        sge->next_lo = 1;   // as of now, this is the last segment
        sge->next_hi = 0;

        if (!is_first_segment)
        {
            buffer_segment *last_seg = list_prev_entry(seg, list);

            // if there is a preceding segment, link its next pointer
            // to the current segment
            last_seg->sg_entry->plda.next_lo = seg->sgd_bus_adr & 0xfffffffc;
            last_seg->sg_entry->plda.next_hi = seg->sgd_bus_adr >> 32;
        }
    }
    else if (has_feature(card, SXPF_FEATURE_SG_XDMA))
    {
        // set up scatter-gather list entry for Xilinx XDMA
        sg_desc_xdma   *sge = &seg->sg_entry->xdma;

        // fill in Xilinx XDMA scatter list descriptor
        if (buf->dma_dir == DMA_FROM_DEVICE)
        {
            // host address is destination
            sge->dst_addr = seg->bus_adr;

            // card adr 0-based is source
            sge->src_addr = buf->buf_size;
        }
        else
        {
            // host address is source
            sge->src_addr = seg->bus_adr;

            // card adr 0-based is destination
            sge->dst_addr = buf->buf_size;
        }
        sge->chunk_size = seg->seg_size;
        sge->next = 0; // as of now, this is the last segment
        sge->control = 0xad4b0000
            // mark segment for XDMA as last in chain
            | 1    // set STOP bit to stop fetching descriptors
            //  | 2    // COMPLETED bit: request interrupt when finished
            //  | 16   // EOP bit: End of packet for stream interface
            ;

        if (!is_first_segment)
        {
            buffer_segment *last_seg = list_prev_entry(seg, list);

            // if there is a preceding segment, link its next pointer
            // to the current segment...
            last_seg->sg_entry->xdma.next = seg->sgd_bus_adr;

            // ...and remove its "last segment" marker
            last_seg->sg_entry->xdma.control &= 0xffff0000;
        }
    }

    if (is_first_segment)
    {
        // if this is the first segment, we use the address of its
        // sg_entry as the buffer's bus address for DMA transfers
        buf->kernel_adr = seg->kernel_adr;
        buf->bus_adr    = seg->sgd_bus_adr;
    }
}


void free_sg_segments(struct sxpf_buffer *buf)
{
    sxpf_card_t    *card = buf->card;
    int             is_driver_allocated = is_driver_buffer(buf);
    buffer_segment *seg, *tmp;

    // free all buffer memory segments
    list_for_each_entry_safe(seg, tmp, &buf->segments, list)
    {
        if (is_driver_allocated)
            dma_free_attrs(&card->pci_device->dev, seg->seg_size,
                           seg->kernel_adr, seg->bus_adr, buf->attr);

        if (seg->sg_entry)
        {
            // return SG descriptor back to pool
            dma_pool_free(card->sge_pool, seg->sg_entry, seg->sgd_bus_adr);
        }

        list_del(&seg->list);
        kfree(seg);
    }
}


/** Append a newly allocated buffer_segment struct to the given buffer as a new
 *  element in the S/G descriptor chain making up the buffer.
 */
static buffer_segment *add_sg_segment(struct sxpf_buffer *buf, void *kernel_adr,
                                      dma_addr_t bus_adr, u32 size)
{
    sxpf_card_t        *card = buf->card;
    struct dma_pool    *sge_pool = get_sgd_dma_pool(card);
    buffer_segment     *seg;

    if (!sge_pool)
        return NULL;

    seg = kzalloc(sizeof(buffer_segment), GFP_KERNEL);

    if (!seg)
        return NULL;    // out of memory

    seg->sg_entry = dma_pool_alloc(sge_pool, GFP_KERNEL, &seg->sgd_bus_adr);

    if (!seg->sg_entry)
    {
        CARD_PRINTF(SXPF_ERROR, "failed to allocate S/G descriptor");
        kfree(seg);
        return NULL;
    }

    seg->kernel_adr = kernel_adr;
    seg->bus_adr = bus_adr;
    seg->seg_size = size;

    list_add_tail(&seg->list, &buf->segments);

    add_hw_sg_descriptor(buf, seg);

    buf->buf_size += size;

    return seg;
}


/** Allocate a buffer_segment struct, initialize it with the address and size
 *  of the contiguous DMA buffer and insert it as sole element into the segment
 *  list of the buffer.
 *
 * The buffer's kernel and bus address member are also set.
 */
static buffer_segment *add_full_segment(struct sxpf_buffer *buf,
                                        void *kernel_adr, dma_addr_t bus_adr,
                                        u32 size)
{
    sxpf_card_t        *card = buf->card;
    buffer_segment     *seg = kzalloc(sizeof(buffer_segment), GFP_KERNEL);

    if (!seg)
        return NULL;    // out of memory

    // no scatter-gather support
    seg->sg_entry = NULL;
    seg->kernel_adr = kernel_adr;
    seg->bus_adr = bus_adr;
    seg->seg_size = size;

    list_add_tail(&seg->list, &buf->segments);

    buf->kernel_adr = seg->kernel_adr;
    buf->bus_adr    = seg->bus_adr;
    buf->buf_size  += size;

    return seg;
}


/** Allocate a DMA buffer for the grabber card.
 *
 * @param card  The card for which to allocate the DMA buffer
 * @param buf   The DMA buffer structure to fill
 * @param type  Buffer type (video or meta data)
 * @param size  The size of the DMA buffer to allocate
 *
 * @return  The kernel address of the allocated buffer (NULL on error).
 */
static void* allocate_buffer(sxpf_card_t *card,
                             struct sxpf_buffer *buf,
                             sxpf_stream_type_t type, u32 size)
{
    // The PCI dev to allocate for (needed for correct DMA mask)
    struct pci_dev *pci_device = card->pci_device;
    u32             unallocated = size;
    size_t          segment_size = size;
    u32             is_rec = is_record(card);
    struct dma_pool*sge_pool = NULL;

    // SG is optional if available on PLDA; if XDMA has SG, it _must_ be used
    int            using_sg =
        (sxpfSgEnable && has_feature(card, SXPF_FEATURE_SG_PLDA))
        |                has_feature(card, SXPF_FEATURE_SG_XDMA);

    if (using_sg)
    {
        sge_pool = get_sgd_dma_pool(card);

        // ensure we have a pool of SG descriptors available
        if (!sge_pool)
            return NULL;

        // Restrict individual scatterlist segment size
        segment_size = buf_sg_segment_size;

#if USE_GPL_SYMBOLS && LINUX_VERSION_CODE >= KERNEL_VERSION(5,1,0)
        segment_size = min(dma_max_mapping_size(&pci_device->dev),
                           segment_size);
#endif
    }

    buf->magic = SXPF_BUFFER_MAGIC;
    buf->dma_dir = is_rec ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
    buf->stream_type = type;
    sema_init(&buf->lock, 1);
    INIT_LIST_HEAD(&buf->readers);
    atomic_set(&buf->hw_owned, 0);

    // init list of buffer segments
    INIT_LIST_HEAD(&buf->segments);
    buf->card = card;
    buf->buf_size = 0;

#if defined(CONFIG_ARM64) && defined(CONFIG_DMA_CMA)
    // force allocation to also be contiguous in RAM, as the promise that
    // it will be contiguous for the device doesn't seem to be kept on
    // Xavier AGX
    // note: linux-4.9.y as used on the Xavier AGX has only the initial
    //       implementation of DMA_ATTR_FORCE_CONTIGUOUS. There have been
    //       numerous fixes to this feature in later kernels, which makes it
    //       seem somewhat unsafe on this old kernel.
    buf->attr = DMA_ATTR_FORCE_CONTIGUOUS;
#else
    buf->attr = 0;
#endif

    while (unallocated > 0)
    {
        dma_addr_t      bus_adr;
        void*           kernel_adr;
        buffer_segment *seg;

        if (segment_size > unallocated)
        {
            segment_size = max(unallocated, (u32)PAGE_SIZE);
            unallocated = segment_size;
        }

        kernel_adr = dma_alloc_attrs(&pci_device->dev, segment_size,
                                     &bus_adr, GFP_KERNEL,
                                     buf->attr | DMA_ATTR_NO_WARN);
        if (kernel_adr == NULL)
        {
            // DMA memory segment allocation failed
            // try a smaller segment size, if scatter-gathering is supported by
            // the hardware
            u32 next_size = PAGE_ALIGN(rounddown_pow_of_two(segment_size - 1));

            if (!using_sg || segment_size <= next_size)
            {
                CARD_PRINTF(SXPF_WARNING,
                            "allocate_buffer: out of DMA memory.");
                break;
            }

            segment_size = next_size;
            continue;   // try again
        }

        if (using_sg)
            seg = add_sg_segment(buf, kernel_adr, bus_adr, segment_size);
        else
            seg = add_full_segment(buf, kernel_adr, bus_adr, segment_size);

        if (!seg)
        {
            dma_free_attrs(&pci_device->dev, segment_size, kernel_adr,
                           bus_adr, buf->attr | DMA_ATTR_NO_WARN);

            CARD_PRINTF(SXPF_WARNING, "allocate_buffer: out of memory.");
            break;
        }

        // segment allocation succeeded
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,0,0)
        // the kernel now guarantees that dma_alloc_coherent() zeroes memory
        memset(kernel_adr, 0, segment_size); // prevent data leak
#endif

        unallocated -= segment_size;
    }

    if (unallocated)
    {
        free_driver_buffer(buf);  // free any partial buffer segments
        return NULL;
    }

    atomic_inc(&card->n_bufs_in_sw);
    buf->dma_unmap = free_driver_buffer;

    return buf->kernel_adr;
}


#ifdef ENABLE_NV_P2P

#define CUDA_DEBUG  SXPF_DEBUG

static void free_nv_tables(struct sxpf_buffer *buf)
{
    sxpf_card_t        *card = buf->card;

    CARD_PRINTF(CUDA_DEBUG, "ENTER: %s", __func__);

    // TODO header page?

    if (buf->nv_page_table)
    {
        nvidia_p2p_free_page_table(buf->nv_page_table);
        buf->nv_page_table = NULL;
    }

    if (buf->nv_dma_mapping)
    {
        nvidia_p2p_free_dma_mapping(buf->nv_dma_mapping);
        buf->nv_dma_mapping = NULL;
    }

    CARD_PRINTF(CUDA_DEBUG, "LEAVE: %s", __func__);
}


static void nv_p2p_free_callback(void *data)
{
    struct sxpf_buffer *buf = data;
    sxpf_card_t        *card = buf->card;

    CARD_PRINTF(CUDA_DEBUG, "ENTER: %s", __func__);

    // TODO locking?
    down(&buf->lock);

    free_nv_tables(buf);

    up(&buf->lock);

    CARD_PRINTF(CUDA_DEBUG, "LEAVE: %s", __func__);
}


static void* declare_cuda_buffer(struct sxpf_buffer *buf,
                                 sxpf_user_buffer_declare_t *req)
{
    // indexed by variable of type enum nvidia_p2p_page_size_type
    static u32  nv_page_sizes[] = { 4<<10, 64<<10, 128<<10, 0, 0, 0, 0, 0, 0 };

    sxpf_card_t        *card = buf->card;
    long                slot = buf - card->dma_buffer;
    __u64               aligned_len;
    unsigned long const header_size = sizeof(sxpf_image_header_t);
    long                n_header_pages = 1;
    unsigned int        gup_flags;
    struct scatterlist *sg;
    unsigned long       i;
    unsigned long       pg_off;
    unsigned long       pg_len;
    buffer_segment     *seg;
    int                 ret;

    CARD_PRINTF(CUDA_DEBUG, "ENTER: %s", __func__);

    buf->dma_unmap = free_cuda_buffer;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
    gup_flags = FOLL_TOUCH | FOLL_WRITE;
#else
    gup_flags = FOLL_LONGTERM | FOLL_WRITE;
#endif

    if ((req->header & (PAGE_SIZE - 1)) > PAGE_SIZE - header_size)
        n_header_pages = 2;

    // allocate an array of page pointers for the image header
    buf->pages = kcalloc(n_header_pages, sizeof(void *), GFP_KERNEL);

    if (!buf->pages)
        return NULL;

    // pin page(s) containing the header of the user's image buffer
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
    n_header_pages = get_user_pages(req->header & PAGE_MASK, n_header_pages,
                                    gup_flags, buf->pages, NULL);
#else
    n_header_pages = pin_user_pages(req->header & PAGE_MASK, n_header_pages,
                                    gup_flags, buf->pages, NULL);
#endif

    if (n_header_pages <= 0)
    {
        CARD_PRINTF(SXPF_ERROR, "failed locking page(s) for user buffer header"
                    " @ 0x%016llx", req->header);
        goto err_out;
    }

    buf->nr_pages = n_header_pages;

    buf->user_header = req->header;
    buf->data_offset = req->payload & NV_PAGE_OFFSET;

    buf->nv_va     = req->payload & NV_PAGE_MASK;
    buf->nv_offset = req->payload & NV_PAGE_OFFSET;
    buf->nv_len    = req->size;

    buf->nv_page_table  = NULL;
    buf->nv_dma_mapping = NULL;

    aligned_len =
        (buf->nv_offset + buf->nv_len + NV_PAGE_SIZE - 1) & NV_PAGE_MASK;

    ret = nvidia_p2p_get_pages(
#ifdef NV_BUILD_DGPU
                               0, 0,
#endif
                               buf->nv_va, aligned_len, &buf->nv_page_table,
                               nv_p2p_free_callback, buf);
    if (ret < 0)
    {
        CARD_PRINTF(SXPF_ERROR, "nvidia_p2p_get_pages() failed with error %d",
                    ret);
        buf->nv_page_table  = NULL;
        goto err_out;
    }

#ifdef NV_BUILD_DGPU
    ret = nvidia_p2p_dma_map_pages(card->pci_device, buf->nv_page_table,
                                   &buf->nv_dma_mapping);
#else
    ret = nvidia_p2p_dma_map_pages(&card->pci_device->dev, buf->nv_page_table,
                                   &buf->nv_dma_mapping, buf->dma_dir);
#endif
    if (ret < 0)
    {
        CARD_PRINTF(SXPF_ERROR,
                    "nvidia_p2p_dma_map_pages() failed with error %d", ret);
        goto err_out;
    }

    // fill hardware descriptors
    buf->buf_size = 0;

    // header
    if (!add_sg_segment(buf, card->header_addr_cpu + 128 * slot,
                        card->header_addr_bus + 128 * slot,
                        sizeof(sxpf_image_header_t)))
    {
        CARD_PRINTF(SXPF_ERROR, "%s - error in add_sg_segment(header)",
                    __func__);

        goto err_out;
    }

    // payload
    for (i = 0; i < buf->nv_dma_mapping->entries; i++)
    {
#ifdef NV_BUILD_DGPU
        dma_addr_t bus_adr = buf->nv_dma_mapping->dma_addresses[i];
        u32 len = nv_page_sizes[buf->nv_dma_mapping->page_size_type];
        if (WARN(!len, "unsupported NVIDIA page size type"))
            goto err_out;

#else
        dma_addr_t bus_adr = buf->nv_dma_mapping->hw_address[i];
        u32 len = buf->nv_dma_mapping->hw_len[i];
#endif

        if (!add_sg_segment(buf, NULL, bus_adr, len))
        {
            CARD_PRINTF(SXPF_ERROR, "%s - error in add_sg_segment(payload)",
                        __func__);

            goto err_out;
        }
    }

    atomic_inc(&card->n_bufs_in_sw);

    CARD_PRINTF(CUDA_DEBUG, "LEAVE: %s", __func__);

    // return the kernel address of the image buffer's header segment
    return buf->kernel_adr;

err_out:
    CARD_PRINTF(CUDA_DEBUG, "LEAVE: %s (ERROR)", __func__);

    free_cuda_buffer(buf);
    return NULL;
}

static void free_cuda_buffer(struct sxpf_buffer *buf)
{
    sxpf_card_t        *card = buf->card;
    buffer_segment     *seg, *tmp;

    CARD_PRINTF(CUDA_DEBUG, "ENTER: %s", __func__);

    // free all buffer memory segments
    free_sg_segments(buf);

    if (buf->pages)
    {
        if (buf->pages[0])
        {
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
            put_page(buf->pages[0]);
#else
            unpin_user_page(buf->pages[0]);
#endif
        }

        kvfree(buf->pages);
        buf->pages = NULL;
        buf->nr_pages = 0;
    }

    if (buf->nv_page_table)
    {
        if (buf->nv_dma_mapping)
        {
#ifdef NV_BUILD_DGPU
            nvidia_p2p_dma_unmap_pages(card->pci_device, buf->nv_page_table,
                                       buf->nv_dma_mapping);
#else
            nvidia_p2p_dma_unmap_pages(buf->nv_dma_mapping);
#endif
        }

        if (buf->nv_page_table)
        {
            nvidia_p2p_put_pages(
#ifdef NV_BUILD_DGPU
                                 0, 0, buf->nv_va,
#endif
                                 buf->nv_page_table);
        }
    }
    buf->nv_va     = 0;
    buf->nv_offset = 0;
    buf->nv_len    = 0;

#ifdef NV_BUILD_DGPU
    free_nv_tables(buf);    // clean up buf->nv_page_table & buf->nv_dma_mapping
#else
    /*
     * nvidia_p2p_put_pages() calls nv_p2p_free_callback()
     */
#endif

    CARD_PRINTF(CUDA_DEBUG, "LEAVE: %s", __func__);
}

#endif

static void* declare_heap_buffer(struct sxpf_buffer *buf,
                                 sxpf_user_buffer_declare_t *req)
{
    sxpf_card_t        *card = buf->card;
    long                slot = buf - card->dma_buffer;
    __u64               first, last; // addresses of first and last payload byte
    long                n_pages;
    unsigned long const header_size = sizeof(sxpf_image_header_t);
    long                n_header_pages = 1;
    unsigned int        gup_flags;
    struct scatterlist *sg;
    unsigned long       i;
    unsigned long       pg_off;
    unsigned long       pg_len;
    unsigned long       len;
    buffer_segment     *seg;

    CARD_PRINTF(SXPF_DEBUG, "ENTER: %s", __func__);

    buf->dma_unmap = free_heap_buffer;

    first = (req->payload                   & PAGE_MASK) >> PAGE_SHIFT;
    last  = ((req->payload + req->size - 1) & PAGE_MASK) >> PAGE_SHIFT;
    n_pages = last - first + 1;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
    gup_flags = FOLL_LONGTERM | FOLL_WRITE;
#else
    gup_flags = FOLL_TOUCH | FOLL_WRITE; // TODO check replay vs. record?
#endif

    if ((req->header & (PAGE_SIZE - 1)) > PAGE_SIZE - header_size)
        n_header_pages = 2;

    // allocate an array of page pointers_pages for header and payload
    buf->pages = kcalloc(n_header_pages + n_pages, sizeof(void *), GFP_KERNEL);

    if (!buf->pages)
        return NULL;

    // pin page(s) containing the header of the user's image buffer
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,5,0)
    n_header_pages = pin_user_pages(req->header & PAGE_MASK, n_header_pages,
                                    gup_flags, buf->pages);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
    n_header_pages = pin_user_pages(req->header & PAGE_MASK, n_header_pages,
                                    gup_flags, buf->pages, NULL);
#else /* version < 5.6.0 */
    n_header_pages = get_user_pages(req->header & PAGE_MASK, n_header_pages,
                                    gup_flags, buf->pages, NULL);
#endif
    if (n_header_pages <= 0)
    {
        CARD_PRINTF(SXPF_ERROR, "failed locking page(s) for user buffer header"
                    " @ 0x%016llx", req->header);
        goto err_out;
    }

    // pin pages making up the payload of the user's image buffer
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,5,0)
    n_pages = pin_user_pages(req->payload & PAGE_MASK, n_pages, gup_flags,
                             buf->pages + n_header_pages);
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
    n_pages = pin_user_pages(req->payload & PAGE_MASK, n_pages, gup_flags,
                             buf->pages + n_header_pages, NULL);
#else /* version < 5.6.0 */
    n_pages = get_user_pages(req->payload & PAGE_MASK, n_pages, gup_flags,
                             buf->pages + n_header_pages, NULL);
#endif
    if (n_pages <= 0)
    {
        CARD_PRINTF(SXPF_ERROR, "failed locking pages for user buffer paylod, "
                    "res=%ld", buf->nr_pages);
        buf->nr_pages = n_header_pages; // only the header pages were locked
        goto err_out;
    }
    buf->nr_pages = n_header_pages + n_pages;

    buf->user_header = req->header;
    buf->data_offset = req->payload & (PAGE_SIZE - 1);

    // allocate the scatter list for the user buffer's payload area
    buf->sg_list = kcalloc(n_pages, sizeof(struct scatterlist), GFP_KERNEL);

    if (!buf->sg_list)
    {
        CARD_PRINTF(SXPF_ERROR, "error allocating scatterlist for user buffer");
        goto err_out;
    }

    // fill in scatter list entries for payload
    // (header's DMA bus address is already known)
    sg_init_table(buf->sg_list, n_pages);

    len = req->size;
    pg_off = buf->data_offset;

    CARD_PRINTF(SXPF_DEBUG, "sg_init_table done");

    for (i = 0; i < n_pages; i++)
    {
        pg_len = min(PAGE_SIZE - pg_off, len);

        if (pg_len == 0)
        {
            CARD_PRINTF(SXPF_ERROR, "too many pages for user buffer");
            break;
        }

        sg_set_page(buf->sg_list + i, buf->pages[i + n_header_pages], pg_len,
                    pg_off);

        pg_off = 0;
        len -= pg_len;
    }

    if (len > 0)
    {
        CARD_PRINTF(SXPF_ERROR, "not enough pages for user buffer");
    }

    CARD_PRINTF(SXPF_DEBUG, "sg_set_page() done");

    // map payload memory for device DMA
    buf->sg_len = dma_map_sg_attrs(&card->pci_device->dev, buf->sg_list, i,
                                   buf->dma_dir, 0);

    if (buf->sg_len <= 0)
    {
        CARD_PRINTF(SXPF_ERROR, "dma_map_sg() failed for user heap buffer");
        goto err_out;
    }

    // fill hardware descriptors
    buf->buf_size = 0;

    // header
    if (!add_sg_segment(buf, card->header_addr_cpu + 128 * slot,
                        card->header_addr_bus + 128 * slot,
                        sizeof(sxpf_image_header_t)))
    {
        CARD_PRINTF(SXPF_ERROR, "%s - error in add_sg_segment(header)",
                    __func__);

        goto err_out;
    }

    // payload
    for_each_sg(buf->sg_list, sg, i, buf->sg_len)
    {
        if (!add_sg_segment(buf, NULL, sg_dma_address(sg), sg_dma_len(sg)))
        {
            CARD_PRINTF(SXPF_ERROR, "%s - error in add_sg_segment(payload)",
                        __func__);

            goto err_out;
        }
    }

    atomic_inc(&card->n_bufs_in_sw);

    CARD_PRINTF(SXPF_DEBUG, "LEAVE: %s", __func__);

    // return the kernel address of the image buffer's header segment
    return buf->kernel_adr;

err_out:
    CARD_PRINTF(SXPF_DEBUG, "LEAVE: %s (ERROR)", __func__);

    free_heap_buffer(buf);
    return NULL;
}


static void free_heap_buffer(struct sxpf_buffer *buf)
{
    sxpf_card_t     *card = buf->card;
    buffer_segment  *seg, *tmp;

    if (!card)
        return;

    // unmap DMA access by device
    if (buf->sg_len > 0)
    {
        dma_unmap_sg_attrs(&card->pci_device->dev, buf->sg_list, buf->sg_len,
                           buf->dma_dir, 0);
        buf->sg_len = 0;
    }

    // free all buffer memory segments
    free_sg_segments(buf);

    if (buf->sg_list)
    {
        kvfree(buf->sg_list);
        buf->sg_list = NULL;
    }

    if (buf->pages)
    {
        unsigned long i;
        for (i = 0; i < buf->nr_pages; i++)
        {
            if (!buf->pages[i])
                break;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,6,0)
            put_page(buf->pages[i]);
#else
            unpin_user_page(buf->pages[i]);
#endif
        }
        kvfree(buf->pages);
        buf->pages = NULL;
        buf->nr_pages = 0;
    }
}


/** Allocate DMA buffers for a grabber card
 *
 * @param card  The card to initialize the DMA buffers for
 *
 * @return 0 on success, negative errno on error
 */
static int sxpf_init_dma_buffers(sxpf_card_t *card)
{
    int     res = 0;
    int     i;

    /* playback buffer allocation management structures */
    init_waitqueue_head(&card->play_buf_requesters);
    sema_init(&card->buf_alloc_lock, 1);
    INIT_LIST_HEAD(&card->free_play_buffers);

    /* DMA buffer for sending DMA addresses to FPGA */

    /* ring buffer for frames */
    memset(&card->dma_buffer, 0, sizeof(card->dma_buffer)); // clear descriptors

    /* video buffers that are allocated by the driver */
    for (i = 0; i < sxpfRingBufferSize; ++i)
    {
        struct sxpf_buffer  *buf = &card->dma_buffer[i];

        buf->card = card;   // might be needed for free_buffer func

        if (allocate_buffer(card, buf, SXPF_DATA_TYPE_VIDEO,
                            sxpfFrameSize) == NULL)
        {
            CARD_PRINTF(SXPF_WARNING,
                        "Could only allocate %d of %d video DMA buffers.", i,
                        sxpfRingBufferSize);

            break; /* for */
        }

        buf->alloc_kind = SXPF_ALLOC_DRIVER;

        INIT_WORK(&buf->dma_work, sxpf_do_dma_event_work);

        buf->file_owner = NULL;
        list_add_tail(&buf->owned_list, &card->free_play_buffers);

        CARD_PRINTF(SXPF_DEBUG,
                    "Video buffer %d size: %d KB, addr: %p, bus addr: %llx.", i,
                    sxpfFrameSize / 1024, buf->kernel_adr, (u64)(buf->bus_adr));
    }

    // in case we exited the loop above early, allow more buffers to be
    // provided by the user
    card->numDrvBuffers = i;
    if (has_feature(card, SXPF_FEATURE_SG_XDMA) && sxpfNumUserBuffers > 0)
    {
        card->numUserBuffers = sxpfNumUserBuffers + sxpfRingBufferSize - i;
    }
    else
    {
        card->numUserBuffers = 0;
    }

    /* video buffers that are provided by a user later on */
    for (; i < card->numDrvBuffers + card->numUserBuffers; ++i)
    {
        struct sxpf_buffer  *buf = &card->dma_buffer[i];

        buf->card = card;   // might be needed for free_buffer func

        buf->alloc_kind = SXPF_BUF_UNUSED;

        INIT_WORK(&buf->dma_work, sxpf_do_dma_event_work);

        buf->kernel_adr = NULL;
        buf->file_owner = NULL;

        buf->stream_type = SXPF_DATA_TYPE_VIDEO;
        sema_init(&buf->lock, 1);
        INIT_LIST_HEAD(&buf->readers);
        atomic_set(&buf->hw_owned, 0);

        // init list of buffer segments
        INIT_LIST_HEAD(&buf->segments);
        buf->buf_size = 0;

        INIT_LIST_HEAD(&buf->owned_list);
    }

    card->ringBufferSize = i;

    /* it's allowed to have all image DMA buffers come from user-space */
    if (i < SXPF_MIN_FRAME_SLOTS && card->numUserBuffers == 0)
    {
        CARD_PRINTF(SXPF_ERROR, "Video DMA buffer allocation failed");

        sxpf_free_dma_buffers(card);

        // disallow recording any video if no buffers are available
        card->valid_channels &= ~SXPF_STREAM_ALL_VIDEO;
        res = -ENOMEM;
    }

    if ((card->valid_channels & SXPF_STREAM_ALL_I2C) == 0)
    {
        // no I2C capture available
        card->i2cRingSize = 0;
        return res;
    }

    /* I2C buffers */
    for (i = 0; i < sxpfNumI2cBuffers; ++i)
    {
        struct sxpf_buffer *buf = &card->dma_buffer[card->ringBufferSize + i];
        u32 const           i2c_buf_size = PAGE_ALIGN(SXPF_I2C_FRAME_SIZE);

        buf->card = card;   // might be needed for free_buffer func

        if (allocate_buffer(card, buf, SXPF_DATA_TYPE_META, i2c_buf_size)
            == NULL)
        {
            CARD_PRINTF(SXPF_WARNING,
                        "Could only allocate %d of %d I2C DMA buffers.", i,
                        sxpfNumI2cBuffers);

            break; /* for */
        }

        buf->alloc_kind = SXPF_ALLOC_DRIVER;

        INIT_WORK(&buf->dma_work, sxpf_do_dma_event_work);

        buf->file_owner = NULL;

        CARD_PRINTF(SXPF_DEBUG,
                    "I2C buffer %d size: %d bytes, addr: %p, bus addr: %llx.",
                    i, i2c_buf_size, buf->kernel_adr, (u64)(buf->bus_adr));
    }

    card->i2cRingSize = i;

    if (card->i2cRingSize == 0)
    {
        if (sxpfNumI2cBuffers > 0)
            CARD_PRINTF(SXPF_ERROR, "I2C DMA buffer allocation failed");

        // disallow recording I2C messages if no buffers are available
        card->valid_channels &= ~SXPF_STREAM_ALL_I2C;
    }

    return res;
}


/** Write a memory dump to the kernel log, if the logging level is at least
 *  SXPF_INFO.
 *
 * @param p     Memory pointer
 * @param size  Number of bytes to dump
 */
void sxpf_dump(u8 *p, u32 size)
{
    int i, j;
    char buf[200];
    char *pbuf;
    const int BL_SIZE = 16;

    for (i = 0; i < size; i += BL_SIZE) /* last bytes lost */
    {
        pbuf = buf;
        for (j = 0; j < BL_SIZE; ++j, pbuf += 3)
        {
            sprintf(pbuf," %02x", *p++);
        }
        if (sxpfDebugLevel >= SXPF_INFO)            \
            printk(KERN_ERR "[" DEVICE_NAME "] %03x:  %s", i, buf);
    }
}


/** Dump the first 512 bytes of the given DMA buffer to the kernel log, if the
 *  logging level is at least SXPF_INFO.
 *
 * @param card  The card to dump the buffer for
 * @param idx   The index of the buffer to dump
 *              (0 <= idx < card->ringBufferSize + card->i2cRingSize)
 */
void sxpf_dump_buffer(sxpf_card_t *card, int idx)
{
    struct sxpf_buffer *buf = card->dma_buffer + idx;

    WARN_ON(card->magic != SXPF_CARD_MAGIC);

    if (buf->alloc_kind != SXPF_BUF_UNUSED)
        sxpf_dump(buf->kernel_adr, 512);
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
static int sxpf_ioctl(struct inode *inode, struct file *file,
                     unsigned int cmd, unsigned long arg)
#else
static long sxpf_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
    sxpf_file_hdl_t *fh = file->private_data;
    sxpf_card_t     *card = fh->card;
    long            ret = 0;

    WARN_ON(card->magic != SXPF_CARD_MAGIC);

    CARD_PRINTF(SXPF_DEBUG, "ioctl(0x%08x)", cmd);

    if (card->pci_device == NULL)
    {
        CARD_PRINTF(SXPF_ERROR, "device currently disabled");
        return -ENODEV;
    }

    if (atomic_read(&card->io_disabled))
        return -EIO;

    switch (cmd)
    {
    case IOCTL_SXPF_READ_CONFIG:
        ret = ioctl_read_config(card, arg);
        break;

    case IOCTL_SXPF_WRITE_CONFIG:
        ret = ioctl_write_config(card, arg);
        break;

    case IOCTL_SXPF_GET_CARD_PROPS:
        ret = copy_to_user((void __user *)arg, &card->props,
                           sizeof(card->props));
        break;

    case IOCTL_SXPF_RELEASE_FRAME:
        ret = ioctl_release_frame(fh, arg);
        break;

    case IOCTL_SXPF_START_RECORD:
        ret = ioctl_start_record(fh, arg);
        break;

    case IOCTL_SXPF_START_PLAYBACK:
        ret = ioctl_start_playback(fh, arg);
        break;

    case IOCTL_SXPF_ALLOC_PLAY_FRAME:
        ret = ioctl_alloc_play_frame(fh, arg);
        break;

    case IOCTL_SXPF_STOP:
        ret = sxpf_stream_stop(fh, arg);
        break;

    case IOCTL_SXPF_READ_REG:
        ret = ioctl_read_register(card, arg);
        break;

    case IOCTL_SXPF_WRITE_REG:
        ret = ioctl_write_register(card, arg);
        break;

    case IOCTL_SXPF_CMD_SYNC:
        ret = ioctl_send_cmd_sync(fh, arg);
        break;

    case IOCTL_SXPF_TRIG_EXPOSURE:
        ret = ioctl_trigger_exposure(card, arg);
        break;

    case IOCTL_SXPF_GET_TIMESTAMP:
        ret = ioctl_get_timestamp(card, arg, 0);
        break;

    case IOCTL_SXPF_GET_TIMESTAMP_ALT:
        ret = ioctl_get_timestamp_fast(card, arg, 0);
        break;

    case IOCTL_SXPF_GET_TIMESTAMP_SEC:
        ret = ioctl_get_timestamp(card, arg, 1);
        break;

    case IOCTL_SXPF_TIMESTAMP_SYNC:
        ret = ioctl_timestamp_sync(card, arg);
        break;

    case IOCTL_SXPF_SET_USR_SEQUENCE:
        ret = ioctl_set_user_sequence(card, arg);
        break;

    case IOCTL_SXPF_ENABLE_USER_SEQ:
        ret = ioctl_enable_user_sequence(card, arg);
        break;

    case IOCTL_SXPF_GET_BUF_HDR:
        ret = ioctl_get_buf_header(card, arg);
        break;

    case IOCTL_SXPF_INIT_USER_BUFFER:
        ret = ioctl_init_user_buffer(fh, arg);
        break;

    case IOCTL_SXPF_INIT_I2C_SLAVE:
        ret = ioctl_init_i2c_slave(fh, arg);
        break;

    case IOCTL_SXPF_REMOVE_I2C_SLAVE:
        ret = ioctl_remove_i2c_slave(fh, arg);
        break;

    case IOCTL_SXPF_I2C_SLAVE_TX_ACK:
        ret = ioctl_i2c_slave_tx_ack(card, arg);
        break;

    default:
        ret = -ENOIOCTLCMD;
        break;
    }

#ifdef ENABLE_XVC
    if (ret == -ENOIOCTLCMD && sxpfXvcEnable)
        ret= ioctl_xvc(card, cmd, arg); // try XVC ioctl
#endif

    if (ret == -ENOIOCTLCMD)
        CARD_PRINTF (SXPF_WARNING, "Got unknown ioctl: 0x%08x.", cmd);

    CARD_PRINTF(SXPF_DEBUG, "ioctl(0x%08x) -> %ld", cmd, ret);

    return ret;
}


/** Callback for the open method of the card device.
 *
 */
int sxpf_open_dev(struct inode *inode, struct file *file)
{
    struct cdev     *cdev = inode->i_cdev;
    sxpf_card_t     *card = container_of(cdev, sxpf_card_t, cdev);
    sxpf_file_hdl_t *fh = kzalloc(sizeof(sxpf_file_hdl_t), GFP_KERNEL);
    int             i;

    CARD_PRINTF(SXPF_DEBUG_EXTRA, "%s", __func__);

    if (atomic_read(&card->io_disabled))
    {
        CARD_PRINTF(SXPF_ERROR, "device currently disabled");
        return -EIO;
    }

    if (!fh)
        return -ENODEV;

    fh->card = card;
    sema_init(&fh->rlock, 1);
    sema_init(&fh->dlock, 1);
    INIT_LIST_HEAD(&fh->owned_buffers);
    INIT_LIST_HEAD(&fh->unread_events);
    INIT_LIST_HEAD(&fh->unread_frames);

    for (i = 0; i < card->ringBufferSize + card->i2cRingSize; i++)
    {
        sxpf_buffer_state_t *bs = fh->buffer_state + i;

        bs->client = fh;
        INIT_LIST_HEAD(&bs->unread);
        INIT_LIST_HEAD(&bs->received);
    }

    file->private_data = fh;

    down(&card->client_lock);

    list_add_tail(&fh->list, &card->clients);

    up(&card->client_lock);

    return 0;
}


/** Callback for the close method of the card device.
 *
 * @param inode Unused
 * @param file
 *
 * @return Zero
 *
 */
static int sxpf_release(struct inode *inode, struct file *file)
{
    sxpf_file_hdl_t *fh = file->private_data;
    sxpf_card_t     *card = fh->card;
    int             i;

    CARD_PRINTF(SXPF_DEBUG_EXTRA, "%s", __func__);

    sxpf_stream_stop(fh, CTRL_RUN | CTRL_MODE_PLAYBACK | CTRL_VCHAN_ENABLE_ALL |
                     CTRL_I2C_CHAN_ENABLE_ALL);

    // return file-exclusive buffers back to the card's free pool
    while (!list_empty(&fh->owned_buffers))
    {
        free_owned_frame(card, list_first_entry(&fh->owned_buffers,
                                                struct sxpf_buffer,
                                                owned_list));
    }

    // unlink file handle from list of connected clients
    down(&card->client_lock);

    list_del(&fh->list);

    up(&card->client_lock);

    // make sure no-one else is currently locking/unlocking frames
    down(&card->channel_lock);

    if (is_record(card))
    {
        // on record cards release all frames still held by this file handle
        for (i = 0; i < card->ringBufferSize + card->i2cRingSize; i++)
        {
            if (is_user_buf_slot(card, i))
                continue;   // buffer comes from userspace

            if (!list_empty(&fh->buffer_state[i].received))
                file_release_slot(fh, i);
        }
    }

    // check if i2c slave file handle is closed:
    for (i = 0; i < SXPF_MAX_I2C_SLAVES; i++)
    {
        if (card->i2c_slaves[i].hClient != NULL)
        {
            if (card->i2c_slaves[i].hClient == fh)
            {
                ioctl_remove_i2c_slave(fh, i);
            }
        }
    }

    // re-allow locking/unlocking operations for other users
    up(&card->channel_lock);

    kfree(fh);

    return 0;
}


static int sxpf_buffer_access_phys(struct vm_area_struct *vma,
                                   unsigned long addr,
                                   void *buf, int len, int write)
{
    struct sxpf_buffer  *dmabuf = vma->vm_private_data;
    u32                 offset = addr - vma->vm_start;
    u32                 uncopied = len;
    buffer_segment      *seg;

    // find the buffer segment that contains the requested offset
    seg = list_first_entry(&dmabuf->segments, buffer_segment, list);
    while (offset >= seg->seg_size)
    {
        offset -= seg->seg_size;
        seg = list_next_entry(seg, list);
    }

    // copy data
    while (uncopied > 0 && &seg->list != &dmabuf->segments)
    {
        void __iomem    *maddr = phys_to_virt(__pa(seg->kernel_adr));
        u32             size = min(seg->seg_size - offset, uncopied);

        if (write)
            memcpy_toio(maddr + offset, buf, size);
        else
            memcpy_fromio(buf, maddr + offset, size);

        buf = (char*)buf + size;    // addvance target buffer pointer
        uncopied -= size;           // update remaining length
        offset = 0;     // the following segments are accessed from their start

        seg = list_next_entry(seg, list);
    }

    return len - uncopied;
}


/* Allow ptrace access to memory-mapped buffers by providing an alternative
 * access method.
 * This enables gdb to display the contents of our DMA buffers.
 */
static struct vm_operations_struct  sxpf_vm_ops =
{
    .access = sxpf_buffer_access_phys,
};


/** Callback for the mmap method of the card device.
 */
int sxpf_mmap(struct file *file, struct vm_area_struct *vma)
{
    sxpf_file_hdl_t     *fh = file->private_data;
    sxpf_card_t         *card = fh->card;
    unsigned long       size = vma->vm_end - vma->vm_start;
    unsigned int        buffersize;
    int                 slot;
    unsigned long       start = vma->vm_start;
    struct sxpf_buffer  *buf;
    buffer_segment      *seg;
    u32 const           i2c_buf_size = PAGE_ALIGN(SXPF_I2C_FRAME_SIZE);

    slot = vma->vm_pgoff / (sxpfFrameSize >> PAGE_SHIFT);

    CARD_PRINTF(SXPF_DEBUG, "ENTER: %s(#%d)", __func__, slot);

    if (slot < card->numDrvBuffers)
    {
        /* driver-allocated video buffer */
        CARD_PRINTF(SXPF_DEBUG,
                    "%s: start 0x%lx, video buf #%d (offs=0x%lx, size=0x%lx)",
                    __func__, start, slot, vma->vm_pgoff, size);

        // we're mapping a driver-allocated buffer into user-space. check that
        // the user provided an address representing the start of a buffer.
        if (vma->vm_pgoff != (unsigned long)slot * (sxpfFrameSize >> PAGE_SHIFT))
        {
            CARD_PRINTF(SXPF_ERROR, "Can't mmap buf #%d, invalid offset", slot);
            return -EINVAL;
        }
    }
    else if (slot < card->ringBufferSize)
    {
        /* user-allocated video buffer */
        CARD_PRINTF(SXPF_ERROR, "Can't mmap buf #%d, not driver-allocated",
                    slot);
        return -EINVAL; // TODO should this be allowed?
    }
    else
    {
        /* I2C buffer */
        unsigned long   i2c_start_page =
            card->ringBufferSize * (sxpfFrameSize >> PAGE_SHIFT);

        int     i2c_slot =
            (vma->vm_pgoff - i2c_start_page) / (i2c_buf_size >> PAGE_SHIFT);

        CARD_PRINTF(SXPF_DEBUG,
                    "%s: start 0x%lx, I2C buf #%d (offs=0x%lx, size=0x%lx)",
                    __func__, start, i2c_slot, vma->vm_pgoff, size);

        if (i2c_slot >= card->i2cRingSize)
        {
            CARD_PRINTF(SXPF_WARNING, "Can't map slot %d, only %d available",
                       slot, card->ringBufferSize + card->i2cRingSize);
            return -ENXIO;
        }

        if (vma->vm_pgoff !=
            i2c_start_page + i2c_slot * (i2c_buf_size >> PAGE_SHIFT))
        {
            CARD_PRINTF(SXPF_ERROR, "Can't mmap buf #%d, invalid offset", slot);
            return -EINVAL;
        }

        slot = card->ringBufferSize + i2c_slot;
    }

    buf = card->dma_buffer + slot;

    /* Total size of buffer in pages (note: buffer sizes are page-aligned) */
    buffersize = buf->buf_size >> PAGE_SHIFT;

    if ((size >> PAGE_SHIFT) > buffersize)
    {
        CARD_PRINTF (SXPF_ERROR, "map size too big (%ld > %d)",
                    size >> PAGE_SHIFT, buffersize);
        return -ENOMEM;
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,3,0)
    // TODO check if this is really needed for older kernels and if we have to
    //      find a replacement for newer kernels
    vma->vm_flags |= VM_IO;
#endif

    if (!sxpfBufMapCached)
    {
#if defined(CONFIG_X86)
        // note: DMA memory on x86 may be cached, because of hardware snooping
#else
        vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#endif
    }

    // allow ptrace/gdb access to DMA buffers mapped into user space
    vma->vm_private_data = card->dma_buffer + slot;
    vma->vm_ops = &sxpf_vm_ops;

    // map buffer segments into virtually contiguous area
    list_for_each_entry(seg, &buf->segments, list)
    {
        unsigned long pfn;

        if (virt_addr_valid(seg->kernel_adr))
        {
            // x86_64
            // NVIDIA Orin (kernel 5.10.x) works like x86_64
            pfn = virt_to_phys(seg->kernel_adr) >> PAGE_SHIFT;
        }
        else
        {
            // IOMMU active
            pfn = vmalloc_to_pfn(seg->kernel_adr);
            // note: pfn = seg->bus_adr; would also work...
        }

        if (remap_pfn_range(vma, start, pfn, seg->seg_size, vma->vm_page_prot))
        {
            CARD_PRINTF(SXPF_ERROR, "remap returned something wrong");
            return -EAGAIN;
        }

        start += seg->seg_size;
    }

    CARD_PRINTF(SXPF_DEBUG, "LEAVE: %s(#%d) -> 0", __func__, slot);

    return 0;
}


/** Check if new events have been received since the last time the user
 *  read them from the file handle.
 *
 * @param fh    The card device file handle.
 *
 * @return  0, if nothing happend
 * @return  An OR combination of zero or more of the following flags:
 *          - SXPF_PENDING_EVENT_FRAME
 *          - SXPF_PENDING_EVENT_VSYNC
 *          - SXPF_PENDING_EVENT_GENERIC
 */
static int sxpf_pending_events(sxpf_file_hdl_t *fh)
{
    int         ret = 0;

#if SEND_VSYNC_EVENTS
    sxpf_card_t *card = fh->card;

    if (card->current_vsyncs != fh->last_vsyncs)
        ret |= SXPF_PENDING_EVENT_VSYNC;
#endif

    if (!list_empty(&fh->unread_events))
        ret |= SXPF_PENDING_EVENT_GENERIC;

    if (!list_empty(&fh->unread_frames))
        ret |= SXPF_PENDING_EVENT_FRAME;

    return ret;
}


/** Callback for the poll method of the card device.
 */
static unsigned int sxpf_poll(struct file *file, poll_table *wait)
{
    sxpf_file_hdl_t *fh = file->private_data;
    sxpf_card_t     *card = fh->card;
    unsigned int    mask = 0;

    poll_wait(file, &card->rd_queue, wait);

    if (sxpf_pending_events(fh))
    {
        mask |= POLLIN | POLLRDNORM;
    }

    return mask;
}


/** Callback for the read method of the card device.
 */
static ssize_t sxpf_read(struct file *file, char __user *user_buf, size_t count,
                        loff_t *ppos)
{
    sxpf_file_hdl_t     *fh = file->private_data;
    sxpf_card_t         *card = fh->card;
    sxpf_event_t        evt;
    size_t              result = -EAGAIN;
    size_t              len = 0;
    int                 event_mask = 0;

    CARD_PRINTF(SXPF_DEBUG, "ENTER: %s", __func__);

    if (card->pci_device == NULL)
    {
        CARD_PRINTF(SXPF_ERROR, "device currently disabled");
        return -ENODEV;
    }

    // we only deliver complete event structures...
    if (count < sizeof(sxpf_event_t))
        return -EINVAL;

    if (down_trylock(&fh->rlock))
    {
        // if someone else is blocked reading from this fd, return if we are
        // non-blocking
        if (file->f_flags & O_NONBLOCK)
            return -EAGAIN;

        // wait for the other one to release his lock
        if (down_interruptible(&fh->rlock))
            return -EINTR;
    }

    if (file->f_flags & O_NONBLOCK)
    {
        event_mask = sxpf_pending_events(fh);

        if (!event_mask)
        {
            up(&fh->rlock);
            return -EAGAIN;
        }
    }
    else
    {
        if (wait_event_interruptible(card->rd_queue,
                                     (event_mask = sxpf_pending_events(fh))
                                     ))
        {
            up(&fh->rlock);
            return -EINTR;
        }
    }

    down(&fh->dlock);   // start changing pending data

#if SEND_VSYNC_EVENTS
    if (event_mask & SXPF_PENDING_EVENT_VSYNC)
    {
        // find out, which channels had a VSYNC since last time we checked
        u64     vsync_diff = fh->last_vsyncs ^ card->current_vsyncs;
        u64     counter_mask;
        int     i;

        // deliver signal change event to user
        evt.type = SXPF_EVENT_VSYNC;
        evt.data = 0;
        evt.extra.unused = 0;
        counter_mask = 0x7fff;
        for (i = 0; i < 4; i++) // TODO handle channels 4..7
        {
            if (vsync_diff & counter_mask)
                evt.data |= 1 << i;
            counter_mask <<= 16;
        }

        // reset signal-changed flag for this open file
        fh->last_vsyncs = card->current_vsyncs;

        if (copy_to_user(user_buf + len, &evt, sizeof(sxpf_event_t)))
            goto out;

        len += sizeof(sxpf_event_t);
    }
#endif

    // notify client about generic events
    if (event_mask & SXPF_PENDING_EVENT_GENERIC)
    {
        while (!list_empty(&fh->unread_events) &&
               (len + sizeof(sxpf_event_t)) <= count)
        {
            struct list_head    *l = fh->unread_events.next;
            sxpf_unread_event_t *unread_event;

            unread_event = container_of(l, sxpf_unread_event_t, list);

            // deliver event to user
            evt = unread_event->event;

            list_del(l);
            kfree(unread_event);

            if (copy_to_user(user_buf + len, &evt, sizeof(sxpf_event_t)))
                goto out;

            len += sizeof(sxpf_event_t);
        }
    }

    // notify user about received frames
    while ((event_mask & SXPF_PENDING_EVENT_FRAME) &&
           (len + sizeof(sxpf_event_t)) <= count)
    {
        sxpf_buffer_state_t *bs;
        int                 buf_idx;
        u32                 info;
        struct sxpf_buffer *buf;
        bool                is_buf_owner = 1; // if buf is driver-allocated
        bool                hdr_fail = 0;

        // extract buffer_state element from unread_frames list and get
        // hold of DMA buffer itself
        bs = container_of(fh->unread_frames.next, sxpf_buffer_state_t, unread);
        buf_idx = bs - fh->buffer_state;
        buf = card->dma_buffer + buf_idx;

        list_del_init(&bs->unread); // guarded by dlock

        // deliver new frame info to user
        info = buf->rx_size & 0x00ffffff;// mask channel ID
        if (info != buf->rx_size)
            info = 1;   // prevent size 0 due to masking operation

        if (is_user_buf_slot(card, buf_idx) && card->mode == SXPF_CAPTURING)
        {
            // stop changing pending data (also, to prevent possible deadlock
            // in sxpf_do_dma_event_work)
            up(&fh->dlock);

            down(&buf->lock);

            is_buf_owner = (buf->file_owner == fh);

            if (is_buf_owner)
            {
                // user-provided buffer: copy header from CMA area to user space
                hdr_fail = copy_to_user((void __user *)buf->user_header,
                                        buf->kernel_adr,
                                        sizeof(sxpf_image_header_t));
            }

            up(&buf->lock);

            down(&fh->dlock);   // start changing pending data, again
        }

        if (buf->stream_type == SXPF_DATA_TYPE_VIDEO)
            evt.type = SXPF_EVENT_FRAME_RECEIVED;
        else
            evt.type = SXPF_EVENT_I2C_MSG_RECEIVED;

        evt.data = (buf_idx << 24) | info;
        evt.extra.unused = 0;

        if (!is_buf_owner)
        {
            CARD_PRINTF(SXPF_DEBUG, "not delivering image event for buf #%d "
                        "that was allocated by different user", buf_idx);
        }
        else if (hdr_fail)
        {
            CARD_PRINTF(SXPF_ERROR, "failed copying header from CMA memory to "
                        "userspace for buf #%d", buf_idx);
        }
        else
        {
            CARD_PRINTF(SXPF_DEBUG, "user read of buf #%d notification",
                        buf_idx);

            if (copy_to_user(user_buf + len, &evt, sizeof(sxpf_event_t)))
                goto out;

            len += sizeof(sxpf_event_t);
        }

        if (list_empty(&fh->unread_frames))
            event_mask &= ~SXPF_EVENT_FRAME_RECEIVED;
    }

    result = len;       // OK

out:
    /*
     * If we jumped here, skipping the asignment to result above, not all of
     * the user memory needed for our events could be written (e.g. due to the
     * user passing a pointer to the end of his valid memory range).
     * In this case we return -EFAULT.
     */
    up(&fh->dlock);     // done changing pending data

    up(&fh->rlock);

    CARD_PRINTF(SXPF_DEBUG, "LEAVE: %s -> %lu", __func__, result);

    return result;
}


/** File operations of the driver for registration to the kernel */
static struct file_operations fops =
{
    .owner          = THIS_MODULE,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
    .ioctl          = sxpf_ioctl,
#else
    .compat_ioctl   = sxpf_ioctl,
    .unlocked_ioctl = sxpf_ioctl,
#endif
    .open           = sxpf_open_dev,
    .release        = sxpf_release,
    .mmap           = sxpf_mmap,
    .read           = sxpf_read,
    .poll           = sxpf_poll,
};


/** Enable interrupt bits in a SMP-safe manner
 *
 * @param card      The grabber card to enable interrupts on
 * @param add_irqs  The mask of interrupts to enable
 */
static void sxpf_enable_irqs(sxpf_card_t *card, u32 add_irqs)
{
    unsigned long   flags;

    spin_lock_irqsave(&card->irq_mask_lock, flags);

    card->irq_mask |= add_irqs;

#if POLL_INTERRUPT
    write_reg32(card, REG_IRQ_MASK, 0);
#else
    write_reg32(card, REG_IRQ_MASK, card->irq_mask);
#endif

    spin_unlock_irqrestore(&card->irq_mask_lock, flags);
}


/** Disable interrupt bits in a SMP-safe manner
 *
 * @param card      The grabber card to disable interrupts on
 * @param add_irqs  The mask of interrupts to disable
 */
static void sxpf_disable_irqs(sxpf_card_t *card, u32 del_irqs)
{
    unsigned long   flags;

    spin_lock_irqsave(&card->irq_mask_lock, flags);

    card->irq_mask &= ~del_irqs;

#if POLL_INTERRUPT
    write_reg32(card, REG_IRQ_MASK, 0);
#else
    write_reg32(card, REG_IRQ_MASK, card->irq_mask);
#endif

    spin_unlock_irqrestore(&card->irq_mask_lock, flags);
}


/** Upon reception of an PCIe AER error notifcation or after an invalid value
 *  was read from a register this function is called to (temporarily) disable
 *  all PCIe accesses by the driver.
 *
 * @param card      The grabber card to disable
 * @param reason    Textual description of reason for shut-off
 */
static void disable_card(sxpf_card_t *card, const char *reason)
{
    if (atomic_cmpxchg(&card->io_disabled, 0, 1))
    {
        CARD_PRINTF(SXPF_ERROR, "Card already disabled; new reason: %s",
                    reason);
    }
    else
    {
        CARD_PRINTF(SXPF_ERROR, "Disabling card: %s", reason);

        disable_irq_nosync(card->pci_device->irq);
        pci_disable_device(card->pci_device);
    }
}


/** Read video DMA completion events and enqueue events in our event queue
 *  so they can be passed on to our readers.
 *
 * @param card  The frame grabber handle
 */
static void sxpf_isr_dma_done(sxpf_card_t *card, sxpf_stream_type_t type)
{
    struct sxpf_buffer  *buf = NULL;

    int res = sxpf_handle_dma_completion(card, type, &buf);

    if (res)
    {
        if (res == 3)
            CARD_PRINTF_IRQ(SXPF_MESSAGE, "Spurious DMA interrupt - ignored.");

        return;
    }

    if (!buf)
    {
        disable_card(card, "PCIe broken - no valid DMA buffer returned!");
        return;
    }

#if USE_GPL_SYMBOLS
    // delegate work to thread context on our own work queue
    queue_work(card->event_queue, &buf->dma_work);
#else
    // use system work queue to delegate work to thread context
    queue_work(system_wq, &buf->dma_work);
#endif
}


/** Send a generic event to the user.
 *
 * @param card  The frame grabber handle
 * @param type  The event type
 * @param data  Event-dependent data
 * @param extra Extra event-dependent data
 */
void enqueue_generic_event(sxpf_card_t *card, __u32 type, __u32 data,
                           __u64 extra)
{
    sxpf_unread_event_t *w = kmalloc(sizeof(sxpf_unread_event_t), GFP_ATOMIC);
    unsigned long       iflags;

    if (!w)
    {
        CARD_PRINTF_IRQ(SXPF_ERROR,
                        "Out of memory - Couldn't deliver event: %d", type);
        return;
    }

    w->event.type = type;
    w->event.data = data;
    w->event.extra.unused = extra;

    // enque event work element into event_list for sxpf_do_generic_event_work()
    spin_lock_irqsave(&card->event_lock, iflags);

    list_add_tail(&w->list, &card->event_list);

    spin_unlock_irqrestore(&card->event_lock, iflags);

#if USE_GPL_SYMBOLS
    // delegate work to thread context on our own work queue
    queue_work(card->event_queue, &card->event_work);
#else
    // use system work queue to delegate work to thread context
    queue_work(system_wq, &card->event_work);
#endif
}


/** Send a generic event to the user and pass the current hw timestamp as the
 *  extra parameter.
 *
 * @param card  The frame grabber handle
 * @param type  The event type
 * @param data  Event-dependent data
 */
static void enqueue_generic_ts_event(sxpf_card_t *card, __u32 type, __u32 data)
{
    u64 ts = sxpf_timestamp(card, 0);   // read primary timestamp

    return enqueue_generic_event(card, type, data, ts);
}


/** Read error register and clear the set bits.
 *
 * @param card  The frame grabber handle
 */
static void sxpf_isr_error_event(sxpf_card_t *card)
{
    u32     type = (card->mode == SXPF_CAPTURING) ?
        SXPF_EVENT_CAPTURE_ERROR : SXPF_EVENT_REPLAY_ERROR;
    u32     errors = read_reg32(card, REG_ERROR);

    write_reg32(card, REG_ERROR, errors);   // acknowledge/clear error bits

    enqueue_generic_ts_event(card, type, errors);
}


/** Send a test event to the user.
 *
 * @param card  The frame grabber handle
 */
static void sxpf_isr_test_event(sxpf_card_t *card)
{
    enqueue_generic_event(card, SXPF_EVENT_SW_TEST, 0, 0);
}


/** Read timestamp latch register and post a trigger event for each set bit in
 *  the trigger_channels bit mask.
 *
 * @param card              The frame grabber handle
 * @param trigger_channels  Bit mask of channels that were triggered
 */
static void sxpf_isr_trigger_event(sxpf_card_t *card, u32 trigger_channels)
{
    static int reg_lo[5] =
    {
        REG_TSTAMP_TRIG0_LO, REG_TSTAMP_TRIG1_LO, REG_TSTAMP_TRIG2_LO, 0,
        REG_TSTAMP_TRIG3_LO
    };

    while (trigger_channels != 0)
    {
        // extract lowest-set bit from bit mask of triggered channels
        u32     chan_mask = trigger_channels & -(int)trigger_channels;
        u32     x = chan_mask / 2;  // 1->0, 2->1, 4->2, 8->4

        u32     tstamp_lo = read_reg32(card, reg_lo[x]);
        u32     tstamp_hi = read_reg32(card, reg_lo[x] + 4);

        enqueue_generic_event(card, SXPF_EVENT_TRIGGER, chan_mask,
                              ((u64)tstamp_hi << 32) | tstamp_lo);

        // remove handled bit from bit mask of triggered channels
        trigger_channels ^= chan_mask;
    }
}


/** Enqueue i2c slave interrupt in our and concurrent processes' event queues.
 *
 * 1. Determine from IntC status the participant i2c slave.
 * 2. Get the reason of the irq from relevant i2c slave irq status reg,
 *    get the required info from the slaves isr, sr ier
 * 3. Handle the irq and encode the special properties of the event
 *    into the u32 data word:
 *    - slave nr 0..7 to:           0xsl______;
 *    - slave event according to:   0x__ev____;
 *      "i2c_slave_event_e"
 *    - rx fifo occupacy:           0x____rx__;
 *    - tx fifo occupacy:           0x______tx;
 *
 * @param card              The frame grabber handle
 */
static void sxpf_isr_i2cslave_event(sxpf_card_t* card)
{
    uint32_t   IntcStatus = 0;
    uint8_t    bit_mask = 0, bitIdx = 0;

    uint32_t   i2c_slave_isr = 0;
    uint32_t   i2c_slave_ier = 0;
    uint32_t   i2c_slave_sr = 0;
    uint32_t   i2c_slave_irq_status = 0;

    static uint8_t          direction_rx_tx = I2C_DIR_UNSPEC;
    enum i2c_slave_event_e  i2c_slave_event = I2C_EVT_NOTHING;
    uint32_t                i2c_slave_regval = 0;
    uint32_t                i2c_slave_regadr = 0;
    uint32_t                enqueue_data;
    __u64                   enqueue_extra;

    // read the Intc ISR
    IntcStatus = read_reg32_bar(card, 1, 0x00009E00);

    // acknowledge/clear ALL pending Intc irq bits for our I2C slaves
    write_reg32_bar(card, 1, 0x00009E0C, IntcStatus & 0xff);

    // check all slaves for an active irq
    for (bitIdx = 0; IntcStatus && bitIdx < SXPF_MAX_I2C_SLAVES; bitIdx++)
    {
        // 1.
        // check if the slave Idx is responsible for the irq
        bit_mask = 1 << bitIdx;
        if (!(IntcStatus & bit_mask))
            continue;

        IntcStatus &= ~bit_mask;

        // bitIdx corresponds to the relevant slave no 0...7
        enqueue_data = bitIdx << 24;

        // 2.
        // get the reason of the irq from relevant i2c slave irq status reg.
        i2c_slave_regadr = card->i2c_slaves[bitIdx].i2cBaseAddr +
            REG_I2C_SLAVE_IISR_OFFSET;
        i2c_slave_isr = read_reg32_bar(card, 1, i2c_slave_regadr);

        // get the i2c slave irq enable register
        i2c_slave_regadr = card->i2c_slaves[bitIdx].i2cBaseAddr +
            REG_I2C_SLAVE_IIER_OFFSET;
        i2c_slave_ier = read_reg32_bar(card, 1, i2c_slave_regadr);

        // interested irqs are only enabled ones:
        i2c_slave_irq_status = i2c_slave_isr & i2c_slave_ier;
        CARD_PRINTF_IRQ(SXPF_DEBUG, "i2c slave%d isr=%x, irq_stat=%x",
                        bitIdx, i2c_slave_isr, i2c_slave_irq_status);

        // if the irq currently not expected => ackn. and return
        if (i2c_slave_irq_status == 0)
        {
            write_reg32_bar(card, 1, card->i2c_slaves[bitIdx].i2cBaseAddr +
                            REG_I2C_SLAVE_IISR_OFFSET, i2c_slave_isr);
            return;
        }

        // get the i2c slave status register
        i2c_slave_regadr = card->i2c_slaves[bitIdx].i2cBaseAddr +
            REG_I2C_SLAVE_SR_REG_OFFSET;
        i2c_slave_sr = read_reg32_bar(card, 1, i2c_slave_regadr);

        // 3.
        if (i2c_slave_irq_status & 0x40)   //NAS: stop condition
        {
            //disable NAS IRQ
            i2c_slave_regadr = card->i2c_slaves[bitIdx].i2cBaseAddr +
                REG_I2C_SLAVE_IIER_OFFSET;
            write_reg32_bar(card, 1, i2c_slave_regadr, i2c_slave_ier & ~0x40);

            // "not AAS" irq occurred ==> clear AAS
            i2c_slave_regadr = card->i2c_slaves[bitIdx].i2cBaseAddr +
                REG_I2C_SLAVE_IISR_OFFSET;
            write_reg32_bar(card, 1, i2c_slave_regadr, i2c_slave_isr & 0x20);

            // if Tx_Cmplt irq => flush tx_Fifo, send i2c_evt_stop_tx
            if (direction_rx_tx == I2C_DIR_TX)
            {
                // Tx_Cmplt" irq occurred ==> clear all Tx IRQs
                i2c_slave_regadr = card->i2c_slaves[bitIdx].i2cBaseAddr +
                    REG_I2C_SLAVE_IISR_OFFSET;
                write_reg32_bar(card, 1, i2c_slave_regadr, i2c_slave_isr & 0x86);

                // if tx fifo not empty, read the occupacy register
                if ((i2c_slave_sr & 0x80) == 0x00)
                {
                    i2c_slave_regadr = card->i2c_slaves[bitIdx].i2cBaseAddr +
                        REG_I2C_SLAVE_TFO_REG_OFFSET;
                    i2c_slave_regval = read_reg32_bar(card, 1, i2c_slave_regadr);
                    enqueue_data |= (i2c_slave_regval + 1);
                }

                // flush Tx- Fifo
                i2c_slave_regadr = card->i2c_slaves[bitIdx].i2cBaseAddr +
                    REG_I2C_SLAVE_CR_REG_OFFSET;
                i2c_slave_regval = read_reg32_bar(card, 1, i2c_slave_regadr);
                write_reg32_bar(card, 1, i2c_slave_regadr,
                                i2c_slave_regval | 0x00000002);

                i2c_slave_event = I2C_EVT_STOP_TX;
            }
            // rx transfer finished ==> get number of last rx data
            else if (direction_rx_tx == I2C_DIR_RX)
            {
                // Rx_Cmplt" irq occurred ==> clear all Rx IRQs
                i2c_slave_regadr = card->i2c_slaves[bitIdx].i2cBaseAddr +
                    REG_I2C_SLAVE_IISR_OFFSET;
                write_reg32_bar(card, 1, i2c_slave_regadr, i2c_slave_isr & 0x08);

                // if rx fifo not empty, read the occupacy register
                if ((i2c_slave_sr & 0x40) == 0x00)
                {
                    i2c_slave_regadr = card->i2c_slaves[bitIdx].i2cBaseAddr +
                        REG_I2C_SLAVE_RFO_REG_OFFSET;
                    i2c_slave_regval = read_reg32_bar(card, 1, i2c_slave_regadr);
                    enqueue_data |= (i2c_slave_regval + 1) << 8;

                    // read rx data in "enqueue_extra"
                    sxpf_get_i2c_slave_rx_data(card, bitIdx,
                                               (uint8_t*)&(enqueue_extra),
                                               i2c_slave_regval + 1);
                }

                i2c_slave_event = I2C_EVT_STOP_RX;
            }
            direction_rx_tx = I2C_DIR_UNSPEC;

            //enable AAS
            write_reg32_bar(card, 1, card->i2c_slaves[bitIdx].i2cBaseAddr +
                            REG_I2C_SLAVE_IIER_OFFSET, 0x20);
        }
        else if (i2c_slave_irq_status & 0x20)   // AAS: start condition
        {
            // clear NAS- and TxCmplt- IRQ
            write_reg32_bar(card, 1, card->i2c_slaves[bitIdx].i2cBaseAddr +
                            REG_I2C_SLAVE_IISR_OFFSET, i2c_slave_isr & 0x42);

            enqueue_extra = sxpf_timestamp(card, 0);
            // read SRW (bit3): 0-> slave rx, 1-> slave tx
            if (i2c_slave_sr & 0x08)
            {
                direction_rx_tx = I2C_DIR_TX;
                i2c_slave_event = I2C_EVT_START_TX;

                // enable Tx- Fifo
                i2c_slave_regadr = card->i2c_slaves[bitIdx].i2cBaseAddr +
                    REG_I2C_SLAVE_CR_REG_OFFSET;
                i2c_slave_regval = read_reg32_bar(card, 1, i2c_slave_regadr);
                write_reg32_bar(card, 1, i2c_slave_regadr,
                                i2c_slave_regval & ~0x00000002);

                // enable NAS, Tx_Fifo_empty ==> 01000100
                write_reg32_bar(card, 1, card->i2c_slaves[bitIdx].i2cBaseAddr +
                                REG_I2C_SLAVE_IIER_OFFSET, 0x44);
            }
            else
            {
                direction_rx_tx = I2C_DIR_RX;
                i2c_slave_event = I2C_EVT_START_RX;
                // enable Rx_Fifo_full & NAS ==> 01001000
                write_reg32_bar(card, 1, card->i2c_slaves[bitIdx].i2cBaseAddr +
                                REG_I2C_SLAVE_IIER_OFFSET, 0x48);
            }
        }
        else if (i2c_slave_irq_status & 0x04)   // AAS & TxFifo_empty
        {
            //clear pending Tx_Fifo_empty IRQ
            write_reg32_bar(card, 1, card->i2c_slaves[bitIdx].i2cBaseAddr +
                            REG_I2C_SLAVE_IISR_OFFSET, (i2c_slave_isr & 0x04));

            i2c_slave_event = I2C_EVT_TX_EMPTY;

            // disable all IRQs
            write_reg32_bar(card, 1, card->i2c_slaves[bitIdx].i2cBaseAddr +
                            REG_I2C_SLAVE_IIER_OFFSET, 0x00);

            //note: Appl. has to fill the Tx_Fifo via "DEV_SXPF_I2C_SLAVE_TX_ACK"
            //       AND reset the Tx_Fifo_empty IRQs
            //       AND enable required IIC- IRQs
        }
        else if (i2c_slave_irq_status & 0x08)   // AAS & Rx_Fifo_full
        {
            i2c_slave_event = I2C_EVT_RX_DATA;

            i2c_slave_regadr = card->i2c_slaves[bitIdx].i2cBaseAddr +
                REG_I2C_SLAVE_RFO_REG_OFFSET;
            i2c_slave_regval = read_reg32_bar(card, 1, i2c_slave_regadr);
            enqueue_data |= (i2c_slave_regval + 1) << 8;

            // rx data in "enqueue_extra"
            sxpf_get_i2c_slave_rx_data(card, bitIdx,
                                       (uint8_t*)&(enqueue_extra), i2c_slave_regval + 1);

            //clear pending Rx_Fifo_full IRQ
            write_reg32_bar(card, 1, card->i2c_slaves[bitIdx].i2cBaseAddr +
                            REG_I2C_SLAVE_IISR_OFFSET, (i2c_slave_isr & 0x08));
        }

        enqueue_data |= i2c_slave_event << 16;
        enqueue_generic_event(card, SXPF_EVENT_I2C_SLAVE,
                              enqueue_data, enqueue_extra);
    }
}


/** Interrupt handler */
static irqreturn_t sxpf_irq_handler (int irq, void *dev_id)
{
    sxpf_card_t     *card = dev_id;
    u32             isr;
    u32             status;
    irqreturn_t     ret = IRQ_NONE;
    unsigned long   iflags;
    int             loop_count = 0;

    if (atomic_read(&card->io_disabled))
        return IRQ_NONE;

    // Prevent concurrent changes to the IRQ mask (by channel activation or
    // deactivation) from keeping the IRQ status active while we think all
    // interrupt events have been handled.
    spin_lock_irqsave(&card->irq_mask_lock, iflags);

    // read interrupt status register
    isr = read_reg32_int(card, REG_IRQ_STATUS);

    // There are PCIe bridge drivers that deassert the IRQ line unconditionally,
    // even if the originating device's IRQ hasn't been deasserted, yet.
    // By looping we make sure that our handler deasserts our interrupt request
    // before returning, instead of relying on being reinvoked for all unhandled
    // events.
    while (isr)
    {
        if (isr == 0xffffffff)
        {
            enqueue_generic_event(card, SXPF_EVENT_IO_STATE, 4, 0);
            disable_card(card, "PCIe broken: ISR read as 0xffffffff");
            break;
        }

        // only check for enabled interrupt sources
        status = isr & card->irq_mask;

        if (!status)
            break;  // don't return IRQ_HANDLED below, if no events are pending

        // TODO remove when testing done...
        CARD_PRINTF_IRQ(SXPF_DEBUG, "IRQ%d: status = 0x%08x, isr = 0x%08x (%d)",
                        irq, status, isr, ++loop_count);

        // clear all write-1-to-clear flags
        write_reg32_int(card, REG_IRQ_STATUS, status);

        // check response FIFO status for video
        if (status & IRQ_STAT_VIDEO_DMA)
        {
            sxpf_isr_dma_done(card, SXPF_DATA_TYPE_VIDEO);
        }

        // check response FIFO status for I2C
        if (status & IRQ_STAT_META_DMA)
        {
            sxpf_isr_dma_done(card, SXPF_DATA_TYPE_META);
        }

        // check error interrupt
        if (status & IRQ_STAT_ERROR)
        {
            sxpf_isr_error_event(card);
        }

        // forward BAR2 interrupts to BAR2 handler
        if (status & IRQ_STAT_BAR2)
        {
            sxpf_isr_bar2(card);
        }

        // check for forwarded events from camera adapters or video path
        if (status & IRQ_STAT_EVENT)
        {
            sxpf_isr_channel_event(card);
        }

        // check test interrupt
        if (status & IRQ_STAT_TEST)
        {
            sxpf_isr_test_event(card);
        }

        // check trigger interrupt
        if (status & IRQ_STAT_TRIGGER)
        {
            sxpf_isr_trigger_event(card, (status & IRQ_STAT_TRIGGER) >> 8);
        }

        // check i2c slave interrupt
        if (status & IRQ_STAT_I2CSLAVE)
        {
            sxpf_isr_i2cslave_event(card);
        }

        ret = IRQ_HANDLED;

        // check for enabled interrupt sources again
        isr = read_reg32_int(card, REG_IRQ_STATUS);
    }

    spin_unlock_irqrestore(&card->irq_mask_lock, iflags);

    return ret;
}


#if POLL_INTERRUPT
static void sxpf_timer_isr_core(sxpf_card_t *card)
{
    if (card)
    {
        sxpf_irq_handler(-1, card);

        // restart
        mod_timer(&card->isr_timer, jiffies + msecs_to_jiffies(POLL_PERIOD_MS));
    }
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
/** Poll the interrupt handler from a timer (older kernels)
 *
 * @param idx   Index into the global card reference array
 */
static void sxpf_timer_isr(unsigned long idx)
{
    sxpf_card_t *card = g_cards[idx];

    sxpf_timer_isr_core(card);
}
#else
/** Poll the interrupt handler from a timer (newer kernels)
 *
 * @param idx   Index into the global card reference array
 */
static void sxpf_timer_isr(struct timer_list *timer)
{
    sxpf_card_t *card = from_timer(card, timer, isr_timer);

    sxpf_timer_isr_core(card);
}
#endif /* LINUX_VERSION_CODE < 4.14 */
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
/** Poll the interrupt handler from a timer (older kernels)
 *
 * @param idx   Index into the global card reference array
 */
static void tmr_hw_timer_sync_update(unsigned long idx)
{
    sxpf_card_t *card = g_cards[idx];

    sxpf_hw_timer_sync_update(card);
}
#else
static void tmr_hw_timer_sync_update(struct timer_list *timer)
{
    sxpf_card_t *card = from_timer(card, timer, hw_tstamp_timer);

    sxpf_hw_timer_sync_update(card);
}
#endif /* LINUX_VERSION_CODE < 4.14 */


/** Upload firmware and start it.
 *
 * The Plasma is held in reset. Then the firmware - if available - is uploaded.
 * Then the application is started by releasing the Plasma CPU from reset.
 *
 * If no firmware is uploaded, the application in the FPGA Flash is (re)started.
 *
 * @param card      The card to start.
 * @param ver_major Major fimrware version number (2, 3, 8, 9, ...)
 *
 * @return 0 on success; non-0, if  application couldn't be started.
 */
static int sxpf_upload_firmware(sxpf_card_t *card, u32 ver_major)
{
    int                     ret = 0;
    const struct firmware   *plasma_fw;
    const char              *fw_name = SXPF_FIRMWARE_NAME;  // default name

    if (!sxpfPatchFirmware)
        return 0;

    if (ver_major == 3 || ver_major == 9)
        fw_name = SXPF_FIRMWARE_NAME_G2;

    if (ver_major == 4)
        fw_name = SXPF_FIRMWARE_NAME_G3;

    ret = request_firmware(&plasma_fw, fw_name, &card->pci_device->dev);

    if (ret)
    {
        /* not an error - use firmware that is already present in the FPGA */
        ret = 0;

        CARD_PRINTF(SXPF_ERROR,
                    "FW patch file %s not found, using stock firmware.",
                    SXPF_FIRMWARE_NAME);
    }
    else
    {
        int     i;
        int     debugLevel = sxpfDebugLevel;
        u32     *fw_data = (u32*)plasma_fw->data;

        // reset Plasma
        write_reg32_bar(card, PLASMA_REGION, REG_PLASMA_CTRL,
                        PLASMA_CTRL_RESET);

        CARD_PRINTF(SXPF_INFO, "writing device firmware");
        sxpfDebugLevel = 0;

        // write firmware data
        for (i = 0; i < plasma_fw->size; i += 4)
        {
            write_reg32_bar(card, PLASMA_REGION, 0x10000 + i, *fw_data++);
        }

        sxpfDebugLevel = debugLevel;

        release_firmware(plasma_fw);

        // release Plasma from reset
        write_reg32_bar(card, PLASMA_REGION, REG_PLASMA_CTRL, PLASMA_CTRL_RUN);

        msleep(1000);   // wait for Plasma to finish init sequence

        CARD_PRINTF(SXPF_ERROR, "Plasma firmware uploaded");
    }

    return 0;
}


/** @return The Max Payload Size on the path from the device to the root
 *          complex hosting it.
 *
 * @param dev   The PCIe card to check.
 */
static int pcie_get_path_mps(struct pci_dev *dev)
{
    int res = 4096; // upper MPS limit according to PCIe spec.
    int mps;

    while (dev->bus->self)
    {
        dev = dev->bus->self;

        mps = pcie_get_mps(dev);

        if (mps < res)
            res = mps;
    }

    return res;
}


static void sxpf_fixup_mps(sxpf_card_t *card, int desired_mps)
{
    struct pci_dev *dev = card->pci_device;
    int             dev_mps, bus_mps, set_mps;

    dev_mps = pcie_get_mps(dev);
    bus_mps = pcie_get_path_mps(dev);

    // don't exceed the root complex's limit, in case the dolphin stack has
    // modified it
    if (desired_mps <= 0)
        set_mps = min(dev_mps, bus_mps);
    else
        set_mps = min(desired_mps, bus_mps);

    CARD_PRINTF(SXPF_WARNING, "MPS (BUS: %d , DEV: %d, SET: %d)",
                bus_mps, dev_mps, set_mps);

    if (dev_mps != set_mps)
    {
        CARD_PRINTF(SXPF_WARNING, "MPS mismatch (BUS: %d , DEV: %d)"
                    " -> adjusting to %d", bus_mps, dev_mps, set_mps);

        if (pcie_set_mps(dev, set_mps) == 0)
        {
            CARD_PRINTF(SXPF_INFO, "Set PCIe Max. Payload Size to %d bytes",
                        set_mps);
        }
        else
        {
            CARD_PRINTF(SXPF_ERROR, "Couldn't set Max Payload Size");
        }
    }
}


static void sxpf_shutdown_card(sxpf_card_t *card)
{
    int i;

    card->mode = SXPF_SHUTDOWN;

    sxpf_pciestream_stop(card, ~0);

    // wait until FPGA has returned all buffers back to us
    for (i = 0; atomic_read(&card->n_bufs_in_hw) > 0; i++)
    {
        if (i >= 1000)
        {
            CARD_PRINTF(SXPF_ERROR, "FPGA took too long to return DMA buffers");
            break;  // don't prevent system shutdwon
        }
        msleep(1);
    }

    sxpf_disable_irqs(card, ~0);

    // reset core to ensure that all DMA transfers are really stopped
    set_control_reg(card, CTRL_SOFT_RESET);

    // there is no register bit that shows DMA activity, so we wait a while
    // before proceeding to release the DMA memory
    mdelay(50);
}


static void sxpf_cleanup_cdev(sxpf_card_t *card)
{
    unsigned int    minor = MINOR(card->cdev.dev);

    sxpf_shutdown_card(card);

#if USE_GPL_SYMBOLS
    device_destroy(sxpf_class, MKDEV(sxpf_major, minor));
#endif

    CARD_PRINTF(SXPF_WARNING, "destroying sxpf device 0x%08x", card->cdev.dev);

    cdev_del(&card->cdev);
}


static int sxpf_initialize_cdev(sxpf_card_t *card)
{
    int     result = 0;
    int     minor = card->nr;

    init_waitqueue_head(&card->rd_queue);
    sema_init(&card->channel_lock, 1);

    sxpf_init_dma_buffers(card);
    // note: We allow the creation of a character device for this instance,
    //       even if no DMA buffers are available. Thereby we can still
    //       perform a firmware update that may add scatter/gather ability
    //       to enable DMA operation with the current kernel.

    cdev_init(&card->cdev, &fops);
    card->cdev.owner = THIS_MODULE;
    result = cdev_add(&card->cdev, MKDEV(sxpf_major, minor), 1);
    if (result)
    {
        CARD_PRINTF(SXPF_ERROR, "chardev registration failed");
        goto err_cdev_add;
    }

    CARD_PRINTF(SXPF_WARNING, "created sxpf device 0x%08x", card->cdev.dev);

#if USE_GPL_SYMBOLS
    if (IS_ERR(device_create(sxpf_class, &card->pci_device->dev,
                             MKDEV(sxpf_major, minor), NULL,
                             "sxpf%u", minor)))
    {
        CARD_PRINTF(SXPF_ERROR, "can't create device");
        result = -EEXIST;
        goto err_dev_create;
    }
#endif

    sxpf_pciestream_stop(card, CTRL_RUN | CTRL_MODE_PLAYBACK |
                         CTRL_VCHAN_ENABLE_ALL | CTRL_I2C_CHAN_ENABLE_ALL);

    sxpf_enable_irqs(card, IRQ_MASK_DEFAULT &
                     ~(sxpfTrigIntEnable ? 0: IRQ_STAT_TRIGGER));
    return 0;

err_dev_create:

    cdev_del(&card->cdev);
err_cdev_add:

    sxpf_free_dma_buffers(card);

    return result;
}


static int mk_number_list(char *number_list, u32 mask, u32 base_bit,
                          int num_ports)
{
    int i, len = 0;
    for (i = 0; i < num_ports; i++)
    {
        if (mask & (base_bit << i))
            len += sprintf(number_list + len, "%s%d", len ? ", " : "", i);
    }
    return len;
}


static int sxpf_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    static const char *type_names[4] =
    {
        "INVALID", "Record", "Playback", "Record & Playback"
    };

    sxpf_card_t *card = kzalloc(sizeof(sxpf_card_t), GFP_KERNEL);
    int         result = 0;
    int         minor;
    u32         fw_version, fw_date, ver_major, ctrl, info;
    int         i;
    char        number_list[32] = { 0 };

    if (!card)
        return -ENOMEM;

    card->nr = sxpf_num;
    CARD_PRINTF(SXPF_INFO, "%s", __func__);

    card->magic = SXPF_CARD_MAGIC;

    if (sxpf_num >= SXPF_MAX_DEVICES)
    {
        CARD_PRINTF(SXPF_INFO, "Too many cards inserted");
        result = -ENODEV;
        goto err_free;
    }
    minor = sxpf_num;

    // reset the i2c slaves
    memset(card->i2c_slaves, 0,
           SXPF_MAX_I2C_SLAVES * sizeof(sxpfi2c_slave_device));

    /* init command fifo lock and argument pool */
    spin_lock_init(&card->dma_fifo_lock);
    spin_lock_init(&card->control_lock);

    // TODO scan sockets for available adapters
    card->valid_channels = CTRL_VCHAN_ENABLE_ALL4;

    card->irq_mask = 0;
    spin_lock_init(&card->irq_mask_lock);

    card->pci_device = dev;

    sxpf_fixup_mps(card, sxpfPayloadSize);

    if (sxpfReadRequestSize != 512)
    {
        if (pcie_set_readrq(dev, sxpfReadRequestSize))
        {
            CARD_PRINTF(SXPF_WARNING, "Couldn't set Max Read Request Size");
        }
        else
        {
            CARD_PRINTF(SXPF_INFO,
                        "Set PCIe Max. Read Request Size to %d bytes",
                        sxpfReadRequestSize);
        }
    }

    /* init isr event work queue */
#if USE_GPL_SYMBOLS
    card->event_queue = create_workqueue("sxpf-work");
#endif
    spin_lock_init(&card->event_lock);
    INIT_LIST_HEAD(&card->event_list);
    INIT_WORK(&card->event_work, sxpf_do_generic_event_work);

    sema_init(&card->client_lock, 1);
    INIT_LIST_HEAD(&card->clients);

    sema_init(&card->i2c_slave_lock, 1);

    result = pci_enable_device(dev);
    if (result)
    {
        CARD_PRINTF(SXPF_ERROR, "Enable pci device failed");
        goto err_free;
    }

    // enable DMA bus master on PCIe device
    pci_set_master(dev);
    pci_try_set_mwi(dev);   // enable Memory-Write-Invalidate transactions

    // announce that we support 64 bit addresses in our DMA implementation
    if (!dma_set_mask_and_coherent(&dev->dev, DMA_BIT_MASK(64)))
    {
        CARD_PRINTF(SXPF_INFO, "setup for 64 bit DMA");
    }
    else if (!dma_set_mask_and_coherent(&dev->dev, DMA_BIT_MASK(32)))
    {
        CARD_PRINTF(SXPF_INFO, "setup for 32 bit DMA");
    }
    else
    {
        CARD_PRINTF(SXPF_ERROR, "Set DMA mask failed");
    }

    // dma_cap_set(DMA_MEMCPY, );
    // dma_cap_set(DMA_SLAVE, );

    result = pci_request_regions(dev, "sxpf");
    if (result < 0)
    {
        CARD_PRINTF (SXPF_ERROR, "reserve regions failed");
        goto err_dis;
    }

#if USE_GPL_SYMBOLS
    /* AER (Advanced Error Reporting) hooks */
    if (sxpfAerEnable)
        pci_enable_pcie_error_reporting(dev);
#endif

    result = pci_save_state(dev);
    if (result)
    {
        CARD_PRINTF(SXPF_ERROR, "PCI save state failed");
        goto err_release_regions;
    }

    result = -ENOMEM;
    for (i = 0; i < 6; i++)
    {
        if (i & 1)
        {
            card->pci_bar_size[i] = 0;
            continue;
        }

        if (i == 2 && pci_resource_len (dev, i) == 0)
        {
            // skip BAR2 because DMA engine is controlled by FW
            CARD_PRINTF(SXPF_INFO, "BAR %d: not used by host", i);
            continue;
        }

        if (i == 4 && pci_resource_len (dev, i) == 0)
        {
            // skip BAR4 if no Flash is connected by firmware
            CARD_PRINTF(SXPF_INFO, "BAR %d: not used by firmware", i);
            continue;
        }

        // map complete BAR
        card->pci_base[i] = pci_iomap(dev, i, 0);

        if (card->pci_base[i] == NULL)
        {
            CARD_PRINTF (SXPF_ERROR, "pci_iomap of BAR %d failed", i);
            goto err_release_regions;
        }
        card->pci_bar_size[i] = (u32)pci_resource_len (dev, i);

        CARD_PRINTF(SXPF_INFO, "BAR %d: length = %u KB", i,
                    card->pci_bar_size[i] / 1024);
    }

    card->type_id = read_reg32(card, REG_NAME);

    switch (card->type_id)
    {
    case CARD_TYPE_SXPCIEX4:
        // first-generation cards have the Plasma-related registers in BAR 2
        card->pci_base[PLASMA_REGION] = card->pci_base[2];
        card->pci_bar_size[PLASMA_REGION] = card->pci_bar_size[2];
        break;

    case CARD_TYPE_SXPCIEX4_G2:
    case CARD_TYPE_SXPCIEX8_G3:
        // second-generation cards have the Plasma-related registers in BAR 0,
        // appended to the regular BAR0 registers
        card->pci_base[PLASMA_REGION] = (char*)card->pci_base[0] + (256 << 10);
        card->pci_bar_size[PLASMA_REGION] = 256 << 10;
        break;

    default:
        CARD_PRINTF(SXPF_ERROR, "Error: Unexpected device ID: %08x",
                    card->type_id);
        result = -ENODEV;
        goto err_unmap;
    }

    // read feature flag registers
    for (i = 0; i < 8; i++)
    {
        card->feature_flags[i] = read_reg32(card, REG_FEATURE(i));
    }

    // Gen3 cards use a feature flag set by the firmware to signal availability
    // of the "generic" scatter/gather ability, in contrast to an earlier
    // restricted S/G implementation.
    // Gen2 cards with the scatter/gather-enabled XDMA core automatically have
    // this ability.
    if (card->type_id == CARD_TYPE_SXPCIEX4_G2 &&
        has_feature(card, SXPF_FEATURE_SG_XDMA))
    {
        force_feature(card, SXPF_FEATURE_GENERIC_SG);
    }

    // Reset Video receiver and pipeline
    //set_control_reg(card, CTRL_SOFT_RESET);
    //set_control_reg(card, 0);
    //mdelay(100);    // allow reset to complete

    // enable I2C hub ports
    ctrl = CTRL_I2C_2_ENABLE | CTRL_I2C_3_ENABLE;
    if (sxpfSgEnable && has_feature(card, SXPF_FEATURE_SG_PLDA))
        ctrl |= CTRL_PLDA_SG_ENABLE;
    set_control_reg(card, ctrl);

    // clear previous interrupt requests
    write_reg32(card, REG_ERROR_ENABLE, ~0);  // enable all error sources
    write_reg32(card, REG_ERROR_CRITICAL, 0); // no error is critical
    write_reg32(card, REG_ERROR_SINGLE, 0);   // no error is signalled only once
    write_reg32(card, REG_ERROR, ~0);         // acknowledge all pending errors
    write_reg32(card, REG_IRQ_STATUS, IRQ_MASK_DEFAULT);

    // store card ID in FPGA so video header gets the ID filled in
    write_reg32(card, REG_SYSTEM_ID, card->nr);

    // let us receive DMA completion messages
    write_reg32(card, REG_FIFO_CONTROL, FIFO_CTRL_NOTIFY_USER);

    // disable trace buffer DMA
    write_reg32(card, REG_TRACE_CONTROL, TRACE_CTRL_DISABLE);

#if 0 /* re-enable if/when frame grabber is extended to support MSI */
    /* Set up a single MSI interrupt */
    if (pci_enable_msi(dev))
    {
        CARD_PRINTF(SXPF_ERROR, "Failed to enable MSI.");
    }
#endif

    g_cards[sxpf_num] = card;

#if POLL_INTERRUPT
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
    setup_timer(&card->isr_timer, sxpf_timer_isr, card->nr);
#else
    timer_setup(&card->isr_timer, sxpf_timer_isr, 0);
#endif /* LINUX_VERSION_CODE < 4.14 */

    mod_timer(&card->isr_timer, jiffies + msecs_to_jiffies(1000));
#else
    result = request_irq(dev->irq, sxpf_irq_handler, IRQF_SHARED,
                         DEVICE_NAME, card);
    if (result)
    {
        CARD_PRINTF(SXPF_ERROR, "Cannot get IRQ %d.", dev->irq);
        goto err_unmap;
    }
    CARD_PRINTF(SXPF_INFO, "Requested IRQ %d successfully.", dev->irq);
#endif

    // setup card properties
    fw_version = read_reg32(card, REG_VERSION);
    fw_date = read_reg32(card, REG_DATE);
    ver_major = (fw_version >> 24) & 255;

    card->props.buffer_size = sxpfFrameSize;
    card->props.fw_version = fw_version;
    card->props.fw_date = fw_date;
    card->props.capabilities =
        (ver_major ==  2 ? SXPF_CAP_VIDEO_RECORD : 0) |   // Gen1 record
        (ver_major ==  3 ? SXPF_CAP_VIDEO_RECORD : 0) |   // Gen2 record
        (ver_major ==  4 ? SXPF_CAP_VIDEO_RECORD : 0) |   // Gen3 record
        (ver_major ==  8 ? SXPF_CAP_VIDEO_PLAYBACK : 0) | // Gen1 playback
        (ver_major ==  9 ? SXPF_CAP_VIDEO_PLAYBACK : 0) | // Gen2 playback
        (ver_major == 10 ? SXPF_CAP_VIDEO_PLAYBACK : 0);  // Gen3 playback

    if (ver_major == 4) // Gen3 RECORD
    {
        card->props.num_channels = 8;
        card->valid_channels = CTRL_VCHAN_ENABLE_ALL;    // 8 channels
    }
    else
    {
        card->props.num_channels = 4;
    }

    if (is_record(card) && has_feature(card, SXPF_FEATURE_I2C_RECORD))
    {
        // allow capturing I2C messages
        if (ver_major == 4) // Gen3 RECORD
            card->valid_channels |= CTRL_I2C_CHAN_ENABLE_ALL;   // 8 channels
        else
            card->valid_channels |= CTRL_I2C_CHAN_ENABLE_ALL4;  // 4 channels
    }

    result = sxpf_initialize_cdev(card);    // needs valid_channels to be set
    if (result)
        goto err_init_cdev;

    // now sxpfRingBufferSize & i2cRingSize are initialized
    card->props.num_buffers = card->numDrvBuffers + card->numUserBuffers;
    card->props.num_user_buffers = card->numUserBuffers;
    card->props.i2c_buffer_size = PAGE_ALIGN(SXPF_I2C_FRAME_SIZE);
    card->props.num_i2c_buffers = card->i2cRingSize;
    card->props.sw_caps = 0;

    {
        enum pci_bus_speed      speed;
        enum pcie_link_width    width;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,17,0)
        if (pcie_get_minimum_link(dev, &speed, &width))
#else
        if (!pcie_bandwidth_available(dev, 0, &speed, &width))
#endif
        {
            CARD_PRINTF(SXPF_ERROR, "Couldn't get minimum PCIe link stats!");
        }
        else
        {
            if (width > 0 && width < 0xff &&
                speed >= 0x14 && speed < 0xff)
            {
                card->props.speed_index = width << (speed - 0x14);
                card->props.pcie_gen    = speed - 0x14 + 1;
                card->props.pcie_width  = width;
            }
        }
    }

    CARD_PRINTF(SXPF_WARNING,
                "%s FPGA version %d.%d.%d, built on %d-%02d-%02d",
                type_names[card->props.capabilities &
                           (SXPF_CAP_VIDEO_RECORD | SXPF_CAP_VIDEO_PLAYBACK)],
                ver_major, (fw_version >> 16) & 255,
                fw_version & 0xffff, fw_date & 0xffff,
                (fw_date >> 16) & 255, (fw_date >> 24) & 255);
    CARD_PRINTF(SXPF_WARNING, "driver: %u image buffers, user: %u image buffers"
                ", %u i2c buffers", card->numDrvBuffers, card->numUserBuffers,
                card->i2cRingSize);


    pci_set_drvdata(dev, card);

    result = sxpf_upload_firmware(card, ver_major);
    if (result)
    {
        //CARD_PRINTF(SXPF_ERROR, "Error bringing up Stream Controller!");
        //goto err_init_cdev;
    }
    else
    {
        // Plasma firmware upload was successful -> setup command FIFO
        sxpf_init_cmd_fifo(card);
    }

    // wait for Plasma to finish automatic INIT sequences
    info = read_reg32_bar(card, PLASMA_REGION, REG_SYSTEM_INFO);

    if ((info & SYSTEM_INFO_INIT_SEQ_RUNNING_MASK) != 0)
    {
        CARD_PRINTF(SXPF_INFO, "waiting for start-up sequences to finish");

        // allow 20sec for all init sequences at start-up
        for (i = 0; i < 20 * 1000; i += 100)
        {
            msleep(100);    // wait 100ms before checking again

            info = read_reg32_bar(card, PLASMA_REGION, 0x0014);

            if ((info & SYSTEM_INFO_INIT_SEQ_RUNNING_MASK) == 0)
                break;      // sequences are finished
        }
    }

    if (has_feature(card, SXPF_FEATURE_I2C_SLAVES))
    {
        //initialise and configure the InterruptController INTC_DEVICE_ID
        // #define XPAR_INTC_0_BASEADDR 0x40801E00U @ bar 1 offset 0x00009E00
        // #define XIN_MER_OFFSET 28
        // #define XIN_IER_OFFSET  8
        // #define XIN_IAR_OFFSET 12
        write_reg32_bar(card, 1, 0x00009E00 + 28, 0x00000000);
        write_reg32_bar(card, 1, 0x00009E00 + 8, 0x00000000);
        write_reg32_bar(card, 1, 0x00009E00 + 12, 0xffffffff);
        // MasterEnableRegister: ME bit 0, HIE bit 1
        // HIE is write once, 0 for testing purpose
        write_reg32_bar(card, 1, 0x00009E00 + 28, 0x00000003);
    }


    if (mk_number_list(number_list, info, SYSTEM_INFO_INIT_SEQ_RUNNING(0),
                       card->props.num_channels))
        CARD_PRINTF(SXPF_ERROR,
                    "start-up sequences took too long on channels: %s",
                    number_list);

    if (mk_number_list(number_list, info, SYSTEM_INFO_INIT_SEQ_FAILED(0),
                       card->props.num_channels))
        CARD_PRINTF(SXPF_ERROR,
                    "start-up sequences failed on channels: %s", number_list);

    if (!number_list[0])
        CARD_PRINTF(SXPF_INFO, "start-up sequences finished successfully");

    sxpf_init_procfs(card);

    sxpf_num++;

    // start loop that keeps the simulated HW timestamp in sync with the real
    // HW timestamp
#if !USE_GPL_SYMBOLS
    spin_lock_init(&card->hw_ts_lock);  // can't use RCU in this case
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
    setup_timer(&card->hw_tstamp_timer, tmr_hw_timer_sync_update, card->nr);
#else
    timer_setup(&card->hw_tstamp_timer, tmr_hw_timer_sync_update, 0);
#endif /* LINUX_VERSION_CODE < 4.14 */

    sxpf_hw_timer_sync_init(card);

#ifdef ENABLE_XVC
    if (sxpfXvcEnable)
        xil_probe_xvc(card);
#endif

    return 0;

err_init_cdev:
    sxpf_disable_irqs(card, ~0); // disable all IRQs
#if POLL_INTERRUPT
    del_timer_sync(&card->isr_timer);
    mdelay(POLL_PERIOD_MS * 2);
#else
    free_irq(dev->irq, card);
#endif

err_unmap:
    for (i = 0; i < 6; i += 2)
    {
        if (card->pci_bar_size[i] > 0)
        {
            pci_iounmap(dev,  card->pci_base[i]);
            card->pci_base[i] = NULL;
            card->pci_bar_size[i] = 0;
        }
    }

err_release_regions:
    pci_release_regions(dev);

err_dis:
    pci_disable_device(dev);

err_free:
    kfree(card);

    g_cards[sxpf_num] = 0;

    return result;
}


/**  Driver release function
 *
 * Function is called by the kernel whenever the driver module is unloaded.
 * Releases the resources that are not needed anymore.
 *
 * @param dev  Device to released
 */
static void sxpf_remove(struct pci_dev *dev)
{
    sxpf_card_t *card = pci_get_drvdata(dev);
    int         i;

    CARD_PRINTF(SXPF_DEBUG, "%s", __func__);

    sxpf_exit_procfs(card);
    sxpf_stop_cmd_fifo(card);
    set_control_reg(card, 0);   // stop all DMA activity, return buffers
    sxpf_cleanup_cdev(card);    // shutdown_card() + remove cdev

    /* mask all IRQs */
    sxpf_disable_irqs(card, ~0);

#if POLL_INTERRUPT
    del_timer_sync(&card->isr_timer);
    mdelay(POLL_PERIOD_MS * 2);
#else
    /* remove irq handler */
    free_irq(dev->irq, card);
#endif

    del_timer_sync(&card->hw_tstamp_timer);

#if USE_GPL_SYMBOLS
    flush_workqueue(card->event_queue);
    destroy_workqueue(card->event_queue);
#endif

    sxpf_free_dma_buffers(card);

    /* unmap BARs */
    for (i = 0; i < 6; i += 2)
    {
        if (card->pci_base[i] && card->pci_bar_size[i] > 0)
        {
            pci_iounmap(dev,  card->pci_base[i]);
            card->pci_base[i] = NULL;
            card->pci_bar_size[i] = 0;
        }
    }

    pci_release_regions(dev);
    pci_set_drvdata(dev, NULL);
    pci_disable_device (dev);

    kfree(card);
}


/**  Driver shutdown function
 *
 * Function is called by the kernel when the system shuts down or re-boots.
 * Releases the resources that are not needed anymore.
 *
 * @param dev  Device to released
 */
void sxpf_shutdown(struct pci_dev *dev)
{
    sxpf_card_t *card = pci_get_drvdata(dev);
    int         i;

    CARD_PRINTF(SXPF_INFO, "%s", __func__);

    /* mask all Plasma IRQs */
    sxpf_stop_cmd_fifo(card);

    /* stop DMA activity, return buffers, disable IRQs, soft reset */
    sxpf_shutdown_card(card);

#if 0 /* this crashes in pci_iounmap() - WHY???
         since we're shutting down, we just keep the mappings - no-one should
         care about the leak */
    /* unmap BARs */
    for (i = 0; i < 6; i++)
    {
        if (card->pci_base[i] && card->pci_bar_size[i] > 0)
        {
            pci_iounmap(dev,  card->pci_base[i]);
            card->pci_base[i] = NULL;
            card->pci_bar_size[i] = 0;
        }
    }
#endif

    pci_release_regions(dev);
    pci_set_drvdata(dev, NULL);
    pci_disable_device (dev);
}


/* ------------------ PCI Error Recovery infrastructure  -------------- */
/**
 * Called when PCI error is detected.
 * @param dev   Pointer to PCI device
 * @param state The current pci connection state
 */
static pci_ers_result_t sxpf_io_error_detected(struct pci_dev *dev,
                                               pci_channel_state_t state)
{
    sxpf_card_t        *card = pci_get_drvdata(dev);
    pci_ers_result_t    result = PCI_ERS_RESULT_RECOVERED;

    CARD_PRINTF(SXPF_ERROR, "PCIe error occured (state %d)", state);

    switch (state)
    {
    case pci_channel_io_normal:
        CARD_PRINTF(SXPF_ERROR, "Normal PCI slot state - ignoring");
        break;

    case pci_channel_io_frozen:
#if 1
        CARD_PRINTF(SXPF_ERROR, "PCI slot frozen - requesting reset");

        write_reg32(card, REG_CONTROL, 0);
#if ! POLL_INTERRUPT
        write_reg32(card, REG_IRQ_MASK, 0); /* try to disable interrupts */
#endif
        // TODO: remove - this only works on Gen2 and tramples over camera
        //                config on Gen3
        //write_reg32(card, 0xbc, 1);         /* force FPGA reload from flash */

        disable_card(card, "PCIe error notification received");
        result = PCI_ERS_RESULT_NEED_RESET; /* Request a slot reset. */
        break;
#else
        CARD_PRINTF(SXPF_ERROR, "PCI slot frozen - disabling");
        if (0)  // skip printf in next case...
        /* fall through */
#endif

    case pci_channel_io_perm_failure:
        CARD_PRINTF(SXPF_ERROR, "Permanent PCI slot failure - disabling");
        atomic_set(&card->io_disabled, 1);
        result = PCI_ERS_RESULT_DISCONNECT;
        break;

    default:
        CARD_PRINTF(SXPF_ERROR, "Invalid PCI slot state - ignoring");
    }

    enqueue_generic_event(card, SXPF_EVENT_IO_STATE, state, 0);

    return result;
}


static pci_ers_result_t sxpf_io_mmio_enabled(struct pci_dev *dev)
{
    sxpf_card_t *card = pci_get_drvdata(dev);

    CARD_PRINTF(SXPF_ERROR, "PCIe MMIO enabled");
    pci_restore_state(dev);
    pci_set_master(dev);

    write_reg32(card, REG_CONTROL, card->control_reg);

#if ! POLL_INTERRUPT
    write_reg32(card, REG_IRQ_MASK, card->irq_mask); /* reenable interrupts */
#endif

    return PCI_ERS_RESULT_RECOVERED;
}


static pci_ers_result_t sxpf_io_slot_reset(struct pci_dev *dev)
{
    sxpf_card_t        *card = pci_get_drvdata(dev);
    pci_ers_result_t    result;
    int                 err;

    CARD_PRINTF(SXPF_ERROR, "PCIe slot reset");

    err = pci_enable_device(dev);
    if (err)
    {
        CARD_PRINTF(SXPF_ERROR, "Cannot re-enable PCI device after reset.");
        result = PCI_ERS_RESULT_DISCONNECT;
    }
    else
    {
        dev->state_saved = true;
        pci_restore_state(dev);
        pci_set_master(dev);

        //sxpf_reset(card);

        result = PCI_ERS_RESULT_RECOVERED;
    }

    return result;
}


/** Called when capturing/replaying can start again.
 *
 * @param dev   Pointer to PCI device
 *
 * This callback is called when the error recovery driver tells us that
 * its OK to resume normal operation.
 */
static void sxpf_io_resume(struct pci_dev *dev)
{
    sxpf_card_t        *card = pci_get_drvdata(dev);

    CARD_PRINTF(SXPF_ERROR, "PCIe resume");
    atomic_set(&card->io_disabled, 0);

    write_reg32(card, REG_CONTROL, card->control_reg);

#if ! POLL_INTERRUPT
    /* reenable interrupts */
    write_reg32(card, REG_IRQ_MASK, card->irq_mask);
#endif

    enqueue_generic_event(card, SXPF_EVENT_IO_STATE, pci_channel_io_normal, 0);
}


/** PCIe Advanced Error Reporting callbacks */
static const struct pci_error_handlers sxpf_err_handler =
{
    .error_detected = sxpf_io_error_detected,
    .mmio_enabled = sxpf_io_mmio_enabled,
    .slot_reset = sxpf_io_slot_reset,
    .resume = sxpf_io_resume,
};


/** Representation of the driver in the kernel */
static struct pci_driver pci_driver =
{
    .name       = DEVICE_NAME,
    .id_table   = sxpf_pci_device_ids,
    .probe      = sxpf_probe,
    .remove     = sxpf_remove,
//  .suspend    = suspend,
//  .resume     = resume
/* no shutdown in older kernel versions */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,9)
    .shutdown   = sxpf_shutdown
#endif
//    .err_handler = &sxpf_err_handler
};



/**
 */
static int __init sxpf_module_init (void)
{
    int retval;
    dev_t dev;

    printk(KERN_NOTICE "sxpf: Driver revision r%d (v%d.%d.%d) initializing.\n",
           SVNVERSION, SXPF_SW_MAJOR, SXPF_SW_MINOR, SXPF_SW_ISSUE);

    sxpf_num = 0;

    if (sxpfRingBufferSize < 0)
        sxpfRingBufferSize = 0;

    if (sxpfNumUserBuffers < 0)
        sxpfNumUserBuffers = 0;

    if (sxpfNumUserBuffers > SXPF_MAX_FRAME_SLOTS)
        sxpfNumUserBuffers = SXPF_MAX_FRAME_SLOTS;

    if (sxpfRingBufferSize < SXPF_MIN_FRAME_SLOTS && sxpfNumUserBuffers == 0)
        sxpfRingBufferSize = SXPF_MIN_FRAME_SLOTS;

    if (sxpfRingBufferSize + sxpfNumUserBuffers > SXPF_MAX_FRAME_SLOTS)
        sxpfRingBufferSize = SXPF_MAX_FRAME_SLOTS - sxpfNumUserBuffers;

    if (sxpfNumI2cBuffers < 0)
        sxpfNumI2cBuffers = 0;

    if (sxpfNumI2cBuffers > SXPF_MAX_I2C_MSG_SLOTS)
        sxpfNumI2cBuffers = SXPF_MAX_I2C_MSG_SLOTS;

    if (sxpfFrameSize > 0x04000000)
        sxpfFrameSize = 0x04000000;

    // ensure the image buffer segment size is a power of 2
    buf_sg_segment_size = roundup_pow_of_two(buf_sg_segment_size);

    // ensure the image buffer size is an integer multiple of the segment size
    sxpfFrameSize = ALIGN(sxpfFrameSize, buf_sg_segment_size);

    if (sxpfPayloadSize > 0)
    {
        // get value equal to the lowest-set bit in sxpfPayloadSize
        int t = sxpfPayloadSize & -sxpfPayloadSize;

        if (t != sxpfPayloadSize ||     /* not a power of 2 */
            sxpfPayloadSize < 128 || sxpfPayloadSize > 4096)
        {
            printk(KERN_ERR "sxpf: Invalid MPS of %d requested. Ignoring.\n",
                   sxpfPayloadSize);
            sxpfPayloadSize = 0;
        }
        else
        {
            printk(KERN_WARNING "sxpf: Changing MPS value may lead to system "
                   "instability.\n");
        }
    }
    else if (sxpfPayloadSize < 0)
        sxpfPayloadSize = 0;

    sxpfFrameSize = PAGE_ALIGN(sxpfFrameSize);

#if USE_GPL_SYMBOLS
    if (sxpfAerEnable)
        pci_driver.err_handler = &sxpf_err_handler;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,4,0)
    sxpf_class = class_create(DEVICE_NAME);
#else
    sxpf_class = class_create(THIS_MODULE, DEVICE_NAME);
#endif
    if (IS_ERR(sxpf_class))
    {
        retval = PTR_ERR(sxpf_class);
        goto err;
    }
#endif

    retval = alloc_chrdev_region(&dev, 0, SXPF_MAX_DEVICES, DEVICE_NAME);
    if (retval)
    {
        printk(KERN_ERR "sxpf: Unable to register character device.\n");
        goto err_class;
    }
    sxpf_major = MAJOR(dev);
    if (sxpfDebugLevel >= SXPF_INFO)
        printk(KERN_NOTICE "sxpf: Character device registered with major %d.\n",
               sxpf_major);

#ifdef ENABLE_XVC
    if (sxpfXvcEnable)
    {
        if (xil_xvc_init())
            printk(KERN_ERR "sxpf: Failed initializing XVC support. "
                   "Xilinx Virtual Cable not available.\n");
    }
#endif

    retval = pci_register_driver (&pci_driver);
    if (retval)
    {
        printk(KERN_ERR "sxpf: Unable to register pci driver (%d)\n", retval);
        goto err_unchr;
    }

    return 0;

err_unchr:
    unregister_chrdev_region(dev, SXPF_MAX_DEVICES);
err_class:
#if USE_GPL_SYMBOLS
    class_destroy(sxpf_class);
#endif
err:
    return retval;
}


/**
 */
static void __exit sxpf_module_exit (void)
{
    pci_unregister_driver (&pci_driver);

#ifdef ENABLE_XVC
    if (sxpfXvcEnable)
        xil_xvc_exit();
#endif

    unregister_chrdev_region(MKDEV(sxpf_major,0), SXPF_MAX_DEVICES);

#if USE_GPL_SYMBOLS
    class_destroy(sxpf_class);
#endif

    printk(KERN_NOTICE "sxpf: module removed\n");
}

module_init (sxpf_module_init);
module_exit (sxpf_module_exit);
/**
 * @endcond
 */
