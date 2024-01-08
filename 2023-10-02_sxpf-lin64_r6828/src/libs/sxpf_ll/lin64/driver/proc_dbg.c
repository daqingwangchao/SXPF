/**
 * @file proc_dbg.c
 *
 * @if (!DRIVER)
 * The driver's internal documentation is not included.
 * @else
 * Provide debug information via a virtual file /proc/sxpfX
 * @endif
 *
 * @cond DRIVER
 */
#include "sxpf_module.h"

#include <linux/proc_fs.h>
#include <linux/seq_file.h>


#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
#define HAVE_PROC_OPS
#endif


static int sxpf_proc_show(struct seq_file *seq, void *offset)
{
	sxpf_card_t     *card = seq->private;
    sxpf_file_hdl_t *fh;
    int              i = 0;

    seq_printf(seq, "buffers=%d\n", card->props.num_buffers);
    seq_printf(seq, "i2c_buffers=%d\n", card->props.num_i2c_buffers);
    seq_printf(seq, "n_bufs_in_hw=%d\n", atomic_read(&card->n_bufs_in_hw));
    seq_printf(seq, "n_bufs_in_sw=%d\n", atomic_read(&card->n_bufs_in_sw));
    seq_printf(seq, "n_bufs_corrupt=%d\n", atomic_read(&card->n_bufs_corrupt));

    down(&card->client_lock);   // prevent concurrent client list manipulations

    // print table header
    seq_printf(seq, "BUF-#\tSOURCE\tFREE\tALLOC\tHW\tUNREAD");
    list_for_each_entry(fh, &card->clients, list)
    {
        seq_printf(seq, "\t%d.User", ++i);
    }
    seq_printf(seq, "\n");

    for (i = 0; i < card->ringBufferSize; i++)
    {
        static char             cross[2] = "-X";
        struct sxpf_buffer     *buf = card->dma_buffer + i;
        struct sxpf_buffer     *b;
        sxpf_buffer_state_t    *bs;
        unsigned long           iflags;

        int hw_owned = atomic_read(&buf->hw_owned);
        int allocated = !!buf->file_owner;
        int is_free = 0;
        int num_readers = 0;
        const char *kind;

        switch (buf->alloc_kind)
        {
        case SXPF_BUF_UNUSED:   kind = "-";     break;
        case SXPF_ALLOC_HEAP:   kind = "Heap";  break;
        case SXPF_ALLOC_NV_DVP: kind = "NVDVP"; break;
        case SXPF_ALLOC_DRIVER: kind = "sxpf";  break;
#ifdef ENABLE_NV_P2P
        case SXPF_ALLOC_NV_P2P: kind = "NVRDMA";break;
#endif
        default:                kind = "INVAL!";
        }

        if (card->props.capabilities & SXPF_CAP_VIDEO_PLAYBACK)
        {
            down(&card->buf_alloc_lock);

            list_for_each_entry(b, &card->free_play_buffers, owned_list)
            {
                if (b == buf)
                {
                    is_free = 1;
                    break;
                }
            }

            up(&card->buf_alloc_lock);
        }

        down(&buf->lock);

        list_for_each_entry(bs, &buf->readers, unread)
        {
            num_readers++;
        }

        up(&buf->lock);

        seq_printf(seq, "%d\t%s\t%c\t%c\t%c\t%d", i, kind,
                   cross[is_free], cross[allocated], cross[hw_owned],
                   num_readers);

        list_for_each_entry(fh, &card->clients, list)
        {
            static const char *state[4] =   // see sxpf_buffer_state_t
            {
                "-", "read", "INVAL", "pend"
            };
            int st = 0;
            bs = fh->buffer_state + i;

            if (!list_empty(&bs->received))
                st |= 1;
            if (!list_empty(&bs->unread))
                st |= 2;

            seq_printf(seq, "\t%s", state[st]);
        }
        seq_printf(seq, "\n");
    }

    up(&card->client_lock);

    return 0;
}


static int sxpf_open_fs(struct inode *inode, struct file *file)
{
    int idx;

    int ret = sscanf(file->f_path.dentry->d_name.name, DEVICE_NAME "%d", &idx);

    if (ret != 1 || idx < 0 || idx > sxpf_num)
        return -EINVAL;

    return single_open(file, sxpf_proc_show, g_cards[idx]);
}


#ifdef HAVE_PROC_OPS

static const struct proc_ops sxpf_proc_ops =
{
    .proc_open    = sxpf_open_fs,
    .proc_read    = seq_read,
    .proc_lseek   = seq_lseek,
    .proc_release = single_release,
};

#else

static const struct file_operations sxpf_proc_ops =
{
  .owner   = THIS_MODULE,
  .open    = sxpf_open_fs,
  .read    = seq_read,
  .llseek  = seq_lseek,
  .release = single_release,
};

#endif


static void get_proc_name(sxpf_card_t *card, char *buf)
{
    sprintf(buf, DEVICE_NAME "%d", card->nr);
}


void sxpf_init_procfs(sxpf_card_t *card)
{
    char    name[32];

    get_proc_name(card, name);

    /* Register the proc entry */
    proc_create(name, 0, NULL, &sxpf_proc_ops);
}

void sxpf_exit_procfs(sxpf_card_t *card)
{
    char    name[32];

    get_proc_name(card, name);

    remove_proc_entry(name, NULL);
}
/** @endcond */
