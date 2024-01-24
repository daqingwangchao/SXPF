#include "sxpf.h"
#include "sxpf_regs.h"
#include "driver_acc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>

#define NELEMENTS(ar)   (sizeof(ar) / sizeof((ar)[0]))
#define MAX_FILE_SIZE   (10*1024*1024)

int fill_frame(char* file, sxpf_image_header_t* img_hdr)
{
    FILE*     fp;
    uint8_t*  buffer;
    int       size;
    size_t    ret;

    fp = fopen(file, "rb");
    if (fp == 0)
    {
        printf("File not found\n");
        exit(-1);
    }
    fseek(fp, 0, SEEK_END);
    size = ftell(fp);
    printf("size(Byte): %d\n", size);
    
    rewind(fp);

    if (size < 0)
    {
        fprintf(stderr, "Error: Couldn't get file size for '%s'.\n", file);
        fclose(fp);
        return 1;
    }

    buffer = (uint8_t*)malloc(sizeof(uint8_t)*size);
    ret = fread(buffer, 1, size, fp);
    printf("buffer: %d, size of buffer: %ld\n", *(buffer+1), sizeof(buffer));
    printf("ret: %ld, size: %d\n", sizeof(uint32_t), size);
    if (ret != (size_t)size)
    {
        printf("Read failed\n");
        exit(-1);
    }
    fclose(fp);

    memcpy(img_hdr, buffer, size);

    free(buffer);

    return size;
}

#define IOCTL_SXPF_MAGIC             'p' 
#define IOCTL_CHANNEL_MAPP         _IOWR(IOCTL_SXPF_MAGIC, 30, uint32_t *)


int main(int argc, char* argv[])
{
    int                  card;
    int                  channel;

    sxpf_hdl             fg;
    sxpf_image_header_t* img_hdr[2];
    HWAITSXPF            devfd;
    sxpf_buf_t           frame_slot[2];
    sxpf_buf_t           frame_slot_returned;
    int                  size;

    sxpf_event_t         events[20];
    ssize_t              len;

    if (argc == 1)
    {
        printf("The %s sample applications shows how easy it is to replay a frame to the grabber card.\n"
               "Usage: %s <card> <channel> <raw_file>\n", basename(argv[0]), basename(argv[0]));
        return 0;
    }

    card = ((int)strtol(argv[1], NULL, 10));
    channel = ((int)strtol(argv[2], NULL, 10));

    

    fg = sxpf_open(card);

    

    if (fg == NULL)
    {
        fprintf(stderr, "failed to open card\n");
        return 1;
    }

    //ioctl(fg->dev,IOCTL_CHANNEL_MAPP,&channel);

    if (sxpf_start_playback(fg, SXPF_STREAM_ALL))
    {
        fprintf(stderr, "failed to start playback\n");
        sxpf_close(fg);
        return 1;
    }

    sxpf_alloc_playback_frame(fg, &(frame_slot[0]), 1000);// return the slot number, so what is the slot?

    printf("frame_slot[0]: %d\n", frame_slot[0]);
    img_hdr[0] = sxpf_get_frame_ptr(fg, frame_slot[0]);// hardware allocates frame buffer and extracts the hearder pointer to img_hdr[0]
    size = fill_frame(argv[3], img_hdr[0]);k
    printf("address of img_hr[0]: %p\n", img_hdr[0]);
    printf("frame_size: %d, sizeof(xx): %ld\n", img_hdr[0]->frame_size,sizeof(img_hdr[0]->frame_size));
    printf("card: %d, channel: %d\n", img_hdr[0]->card_id, img_hdr[0]->cam_id);
    img_hdr[0]->ts_start_hi = -1;
    img_hdr[0]->ts_start_lo = -1;
    img_hdr[0]->ts_end_hi = -1;
    img_hdr[0]->ts_end_lo = -1;
    img_hdr[0]->cam_id = channel;
    img_hdr[0]->card_id = card;

    sxpf_alloc_playback_frame(fg, &(frame_slot[1]), 1000);

    printf("frame_slot[0]: %d\n", frame_slot[1]);

    img_hdr[1] = sxpf_get_frame_ptr(fg, frame_slot[1]);

    printf("address of img_hr[1]: %p\n", img_hdr[1]);

    size = fill_frame(argv[3], img_hdr[1]); 
    img_hdr[1]->ts_start_hi = -1;
    img_hdr[1]->ts_start_lo = -1;
    img_hdr[1]->ts_end_hi = -1;
    img_hdr[1]->ts_end_lo = -1;
    img_hdr[1]->cam_id = channel;
    img_hdr[1]->card_id = card;

    // send both frames to hardware
    sxpf_release_frame(fg, frame_slot[0], (channel << 30) | size);
    sxpf_release_frame(fg, frame_slot[1], (channel << 30) | size);
    long chan_encode = (channel << 30) | size;

    printf("Debug: the encoded channel value:%ld, and decode: %ld, size: %d\n", chan_encode, chan_encode>>30, (1u << 30) - 1);

    sxpf_get_device_fd(fg, &devfd);

    // new wait until the frame buffers come back to send new frames
    
    for (ssize_t i=0; i<5; i+=len)
    {
        len = (sxpf_wait_events(1, &devfd, 50 /* timeout in ms */) > 0) ? 1 : 0;// the return value 0 indicates that timeout

        // if select didn't time out, read event(s) into a buffer
        if (len > 0)
        {
            len = sxpf_read_event(fg, events, NELEMENTS(events));
        }
        int j = 0;
        printf("len = %ld\n",len);

        for (ssize_t n = 0; n < len; n++)
        {
            
            switch (events[n].type)
            {
                case SXPF_EVENT_FRAME_RECEIVED:
                    frame_slot_returned = events[n].data / (1 << 24);
                    printf("received frame slot %d back from hardware. sending next frame.", frame_slot_returned);
                    printf("event type %d, data: 0x%08x \n", events[n].type, events[n].data);
                    // this does not necessarily mean the frame is already send out by the hardware.
                    // the frame was only copied to the frame buffer in the hardware.
                    // in this sample we do not modify the frame data. only send again.
                    sxpf_release_frame(fg, frame_slot_returned, (channel << 30) | size);
                    break;
                default: // log unexpected events
                    printf("event type %d: data = 0x%08x\n",
                       events[n].type,
                       events[n].data);
                    break;
            }
        }
    }

    sxpf_stop(fg, SXPF_STREAM_ALL);
    sxpf_close(fg);

    return 0;
}
