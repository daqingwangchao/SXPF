/**
 * @file sxpf_cmd_fifo.c
 *
 * @if (!DRIVER)
 * The driver's internal documentation is not included.
 * @else
 * The driver's command queue implementation.
 * @endif
 *
 * @cond DRIVER
 */
#include <linux/sched.h>
#include <linux/delay.h>

#include "sxpf.h"
#include "sxpf_module.h"


#define POLL_FOR_FREE_CMD_FIFO      (0)


extern void write_reg8_bar(sxpf_card_t *card, u32 bar, u32 adr, u32 dat);
extern void write_reg16_bar(sxpf_card_t *card, u32 bar, u32 adr, u32 dat);
extern void write_reg32_bar(sxpf_card_t *card, u32 bar, u32 adr, u32 dat);
extern u32 read_reg8_bar(sxpf_card_t *card, u32 bar, u32 adr);
extern u32 read_reg16_bar(sxpf_card_t *card, u32 bar, u32 adr);
extern u32 read_reg32_bar(sxpf_card_t *card, u32 bar, u32 adr);
extern s64 sxpf_timestamp(sxpf_card_t* card);
extern void enqueue_generic_event(sxpf_card_t *card, __u32 type, __u32 data,
                                  __u64 extra);

static u32 read_reg32(sxpf_card_t *card, u32 adr)
{
    return read_reg32_bar(card, PLASMA_REGION, adr);
}

static void write_reg32(sxpf_card_t *card, u32 adr, u32 dat)
{
    write_reg32_bar(card, PLASMA_REGION, adr, dat);
}


/** Enqueue a command value in the Request FIFO.
 *
 * The function waits until a free entry is available in the Request FIFO, and
 * returns right after the command has been written to the FIFO.
 *
 * @param card  The grabber card reference
 * @param cmd   the command value to write
 */
static void sxpf_enqueue_cmd(sxpf_card_t *card, u32 cmd)
{
#if POLL_FOR_FREE_CMD_FIFO
    unsigned long   iflags;

    /* acquire exclusive access to cmd FIFO */
    spin_lock_irqsave(&card->cmd_fifo_lock, iflags);

    // wait for command FIFO to have space for our command
    while (MSG_FIFO_FREE_LEVEL(read_reg32(card, REG_MSG_FIFO_STAT_H)) == 0)
        udelay(10);
    // TODO implement timeout to prevent blocking
#endif

    // send command
    write_reg32(card, REG_REQ_FIFO_WR, cmd);

#if POLL_FOR_FREE_CMD_FIFO
    /* release command FIFO */
    spin_unlock_irqrestore(&card->cmd_fifo_lock, iflags);
#endif
}


static inline void set_cmd_status(sxpf_card_t *card, int arg_slot,
                                  u32 status)
{
    atomic_set(&card->cmd_status[arg_slot], status);
}


static inline u32 get_cmd_status(sxpf_card_t *card, int arg_slot)
{
    return atomic_read(&card->cmd_status[arg_slot]);
}


/** Print info message about sent commands and received responses/indications.
 */
static void sxpf_dbg_fifo(sxpf_card_t *card, const char *what, u32 cmd_resp,
                         u8 *args, u32 num_args)
{
    if (sxpfDebugLevel >= SXPF_MESSAGE)
    {
        int     i, len = 0;
        int     max_msg_sz = SXPF_NUM_ARGS_PER_CMD * 4 + 8;
        char   *arg_msg = kmalloc(max_msg_sz, GFP_KERNEL);

        if (arg_msg == NULL)
        {
            CARD_PRINTF(SXPF_MESSAGE, "%s 0x%08x (error: couldn't allocate "
                        "memory to dump buffer contents)", what, cmd_resp);
            return;
        }

        arg_msg[0] = 0;
        for (i = 0; i < num_args && i < SXPF_NUM_ARGS_PER_CMD; i++)
        {
            if (i)
            {
                arg_msg[len++] = ',';
                arg_msg[len++] = ' ';
            }
            len += snprintf(arg_msg + len, max_msg_sz - len, "%02x", args[i]);

            WARN_ON(len >= SXPF_NUM_ARGS_PER_CMD * 4);
        }

        if (i < num_args)
            len += snprintf(arg_msg + len, max_msg_sz - len, ",...");

        CARD_PRINTF(SXPF_MESSAGE, "%s 0x%08x <%s>", what, cmd_resp, arg_msg);

        kfree(arg_msg);
    }
}



/** Send a user command to the Plasma in the FPGA. Optional Arguments are
 *  passed in the parameter RAM area.
 *
 * Only, if arguments are sent and/or a response is expected, the function
 * waits for the Plasma to to acknowledge the command with a response.
 * Otherwise the function returns, as soon as it has posted the command value.
 *
 * In synchronous case, the cmd value must have its lower 12 bits cleared (they
 * are used to transmit a reference to the argument/response RAM location).
 *
 * @param card      Card handle.
 * @param cmd       32bit Command code
 * @param args      Pointer to array of 32bit argument values
 * @param num_args  Number of arguments (at most \ref SXPF_NUM_ARGS_PER_CMD)
 * @param ret       Pointer to array where to return the command's response data
 * @param num_ret   Number of expected 32bit response values
 * @param timeout_ms    Number of milli seconds to wait for a response from the
 *                  Plasma before declaring a timeout error.
 *
 * @return SXPF_CMD_STATUS_RESPONSE_OK if successful
 * @return Upon error one of the following codes is returned:
 *          - SXPF_CMD_STATUS_RESPONSE_ERROR
 *          - SXPF_CMD_STATUS_RESPONSE_UNKNOWN_CMD
 *          - SXPF_CMD_STATUS_RESPONSE_INVALID_ARG
 *          - -EIO on Plasma timeout
 *          - -ERESTARTSYS if interrupted while waiting for free argument slot
 */
int sxpf_send_cmd(sxpf_card_t *card, u32 cmd, u8 *args, unsigned num_args,
                 u8 *ret, unsigned num_ret, unsigned timeout_ms)
{
    int             arg_slot;
    unsigned long   iflags;
    int             i;
    unsigned long   arg_base = 0;
    int             result = SXPF_CMD_STATUS_RESPONSE_OK;

    if (num_args > SXPF_NUM_BYTES_PER_CMD || (num_args > 0 && args == NULL) ||
        num_ret > SXPF_NUM_BYTES_PER_CMD || (num_ret > 0 && ret == NULL))
    {
        return -EINVAL;
    }

    // allocate a command slot; block until one is free or interrupted by user
    if (down_interruptible(&card->cmd_slot_count))
        return -ERESTARTSYS;

    // allocate space for arguments and/or return values in parameter RAM
    spin_lock_irqsave(&card->cmd_arg_lock, iflags);

    arg_slot = card->free_cmd_arg_slot;
    if (arg_slot != -1)
    {
        card->free_cmd_arg_slot = card->cmd_arg_slots[arg_slot];
    }

    spin_unlock_irqrestore(&card->cmd_arg_lock, iflags);

    if (arg_slot == -1)
    {
        // internal error: there must be a free slot if the semaphore was
        // taken successfully...
        up(&card->cmd_slot_count);
        return -EIO;
    }

    set_cmd_status(card, arg_slot, CMD_STATUS_PENDING);

    arg_base = REG_ADDR_RAM_BASE + arg_slot * SXPF_NUM_BYTES_PER_CMD;

    // insert argument/ret-value location into command code
    cmd = (cmd & ~0x00000fff) | (arg_slot * SXPF_NUM_BYTES_PER_CMD);

    if (num_args > 0)
    {
        // write arguments to parameter RAM in units of 32 bits (instead of 8)
        for (i = 0; i < num_args; i += 4)
        {
            u32     arg32;

            // TODO review for big-endian host!
            arg32 = args[i] + (args[i + 1] << 8) + (args[i + 2] << 16)
                + (args[i + 3] << 24);

            write_reg32(card, arg_base + i, arg32);
        }
    }

    /*
     * asynchronously enqueue the command in the request FIFO
     */
    sxpf_enqueue_cmd(card, cmd);

    sxpf_dbg_fifo(card, "cmd <-", cmd, args, num_args);

    // wait for response from FPGA
    // note: _not_ interruptible, because we can't restart the command!
    if (!wait_event_timeout(card->fifo_wait_queue,
                            (get_cmd_status(card, arg_slot) != 0),
                            HZ * timeout_ms / 1000))
    {
        CARD_PRINTF(SXPF_ERROR, "Plasma command timed out on arg slot %d!",
                    arg_slot);

        result = -EIO;  // Stream Controller hangs...            

        // note: the argument RAM slot is _not_ freed, otherwise a future
        //       command _might_ receive the delayed answer and misbehave.
        //       This is a potential resource leak, but in this situation
        //       there is something awry with the Plasma, anyway, from
        //       which we can't recover, except by a host/driver restart.
    }
    else
    {
        u32     response = get_cmd_status(card, arg_slot);

        // retrieve response data
        for (i = 0; i < num_ret; i += 4)
        {
            u32     r32 = read_reg32(card, arg_base + i);

            ret[i + 0] = (r32 >>  0) & 0xff;
            ret[i + 1] = (r32 >>  8) & 0xff;
            ret[i + 2] = (r32 >> 16) & 0xff;
            ret[i + 3] = (r32 >> 24) & 0xff;
        }

        /* free space for command arguments in parameter RAM */
        spin_lock_irqsave(&card->cmd_arg_lock, iflags);

        card->cmd_arg_slots[arg_slot] = card->free_cmd_arg_slot;
        card->free_cmd_arg_slot = arg_slot;

        spin_unlock_irqrestore(&card->cmd_arg_lock, iflags);

        sxpf_dbg_fifo(card, "response ->", response, ret, num_ret);

        // return value is the status code contained in the response value
        result = response & SXPF_CMD_STATUS_MASK;
    }

    up(&card->cmd_slot_count);

    return result;
}


/** Interrupt handler for events generated by BAR2 modules
 *
 * @param card  The grabber card instance
 */
void sxpf_isr_bar2(sxpf_card_t *card)
{
    u32     num_responses =
        MSG_FIFO_FILL_LEVEL(read_reg32(card, REG_MSG_FIFO_STAT_H));

    while (num_responses--)
    {
        u32 response = read_reg32(card, REG_RESP_FIFO_RD);
        u32 arg_slot;

        // check for asynchronous indication from Plasma
        if ((response & SXPF_CMD_STATUS_MASK) == SXPF_CMD_STATUS_REQUEST)
        {
            CARD_PRINTF_IRQ(SXPF_MESSAGE, "indication -> 0x%08x",
                            response);

            // handle indications from Plasma that were not triggered by
            // a command from us
            switch (response & SXPF_CMD_MASK)
            {
            case SXPF_CMD_NOP:
                // Plasma sent a ping: TODO answer with pong
                break;
            }

            continue;
        }

        // TODO check if Plasma uses byte or u32 offset
        arg_slot = (response & 0x00000fff) / SXPF_NUM_BYTES_PER_CMD;

        // acknowledge command with expected response
        if (arg_slot < SXPF_NUM_CMD_ARG_SLOTS)
        {
            u32 status = get_cmd_status(card, arg_slot);

            if (status != 0)
            {
                CARD_PRINTF_IRQ(SXPF_ERROR,
                                "Unexpected Response recvd.: 0x%08x (no"
                                " cmd pending on arg slot %d: 0x%08x)",
                                response, arg_slot, status);
            }
            else
            {
                CARD_PRINTF_IRQ(SXPF_DEBUG, "Awaited Response: 0x%08x"
                                " (arg slot %d status: 0x%08x)",
                                response, arg_slot, status);

                // store response status - let waiting process
                // continue after wake_up
                set_cmd_status(card, arg_slot, response);
            }
        }
        else
        {
            CARD_PRINTF_IRQ(SXPF_ERROR,
                            "Invalid Response ignored: 0x%08x",
                            response);
        }
    }

    wake_up(&(card->fifo_wait_queue));
}


/** Forward a channel event to the user.
 *
 * @param card  The frame grabber handle
 */
void sxpf_isr_channel_event(sxpf_card_t *card)
{
    s64     time = sxpf_timestamp(card);
    u32     num_events =
        MSG_FIFO_FILL_LEVEL(read_reg32(card, REG_EVT_FIFO_STAT_H));

    while (num_events--)
    {
        u32 evt = read_reg32(card, REG_EVT_FIFO_RD);

        enqueue_generic_event(card, SXPF_EVENT_CHANNEL_INFO, evt, time);

        CARD_PRINTF_IRQ(SXPF_DEBUG, "Channel event #%d on channel #%d (0x%08x)",
                        (evt >> 8) & 255, (evt >> 20) & 3, evt);
    }
}


/** Initialize command FIFO processing
 *
 * @param card  The grabber card instance
 *
 * @return 0 on success
 */
int sxpf_init_cmd_fifo(sxpf_card_t *card)
{
    int     i;

    sema_init(&card->cmd_slot_count, SXPF_NUM_CMD_ARG_SLOTS);
    /* init command fifo lock and argument pool */
    spin_lock_init(&card->cmd_fifo_lock);
    spin_lock_init(&card->cmd_arg_lock);
    for (i = 0; i < SXPF_NUM_CMD_ARG_SLOTS - 1; ++i)
    {
        card->cmd_arg_slots[i] = i + 1;
        set_cmd_status(card, i, CMD_STATUS_IDLE);
    }
    set_cmd_status(card, i, CMD_STATUS_IDLE);
    card->cmd_arg_slots[i] = -1;    // end of free-list
    card->free_cmd_arg_slot = 0;
    init_waitqueue_head(&card->fifo_wait_queue);

    card->user_seq_size = -1;       // not queried, yet

    write_reg32(card, REG_RESP_FIFO_CTRL, FIFO_CTRL_RESET);
    write_reg32(card, REG_EVT_FIFO_CTRL, FIFO_CTRL_RESET);
    write_reg32(card, REG_IRQ_MASK_HOST, IRQ_STAT_RESP_FIFO);

    return 0;
}


void sxpf_stop_cmd_fifo(sxpf_card_t *card)
{
    write_reg32(card, REG_IRQ_MASK_HOST, 0);
}
/**
 * @endcond
 */
