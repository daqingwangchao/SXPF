#include "sxpf.h"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#ifndef WIN32
#include <sys/select.h>
#endif

#define NELEMENTS(ar)   (sizeof(ar) / sizeof((ar)[0]))

int main(int argc, char* argv[])
{
    sxpf_hdl            fg;
    HWAITSXPF           devfd;
    long long           last_time = 0;
    sxpf_card_props_t   props;
    int                 card = 0;
    uint32_t            channel = SXPF_STREAM_ALL;

    if (argc >= 2)
        card = atol(argv[1]);

    if (argc >= 3)
    {
        channel = atol(argv[2]);
        channel = SXPF_STREAM_VIDEO0 << channel | SXPF_STREAM_I2C0 << channel; // this might be a another way to code the channel number
    }

    fg = sxpf_open(card);

    sxpf_get_device_fd(fg, &devfd);
    sxpf_get_timestamp(fg, &last_time); // HW time at start
    sxpf_get_card_properties(fg, &props);

    if (sxpf_start_record(fg, channel))
    {
        fprintf(stderr, "failed to start stream\n");
        sxpf_close(fg);
        return 1;
    }

    while (1)
    {
        sxpf_event_t            events[20];
        int                     len;

        sxpf_image_header_t    *img_hdr;
        uint8_t                *img_ptr;
        sxpf_meta_header_t     *i2c_hdr;
        uint8_t                *i2c_ptr;
        int                     frame_slot;
        long long int           rx_time = 0;
        long long int           ts_start;
        long long int           latency;

        // wait for events from frame grabber
        len = sxpf_wait_events(1, &devfd, 50 /* ms */);

        // if select didn't time out, read event(s) into a buffer
        if (len > 0)
        {
            sxpf_get_timestamp(fg, &rx_time);   // current HW time

            len = sxpf_read_event(fg, events, NELEMENTS(events));
        }

        // loop over all received events
        for (int n = 0; n < len; n++)
        {
            switch (events[n].type)
            {
            case SXPF_EVENT_FRAME_RECEIVED:
                frame_slot = events[n].data / (1 << 24);
                img_hdr    = sxpf_get_frame_ptr(fg, frame_slot);
                img_ptr    = (uint8_t*)img_hdr + img_hdr->payload_offset;

                ts_start =
                    img_hdr->ts_start_hi * (1ull<<32) + img_hdr->ts_start_lo;

                latency = rx_time - ts_start;

                printf("%+7.3fms - image from port #%d: %dx%d/%d, "
                       "ts = %llu (%+.2fms), first payload byte %d\n",
                       (rx_time - last_time) / 4e4, img_hdr->cam_id,
                       img_hdr->columns, img_hdr->rows, img_hdr->bpp,
                       ts_start, latency / 40000., img_ptr[0]);

                if (img_hdr->frame_size > props.buffer_size)
                {
                    printf("image frame exceeds frame buffer size\n");
                }

                // this must be done for each received buffer to ensure the
                // hardware can continue to send images
                sxpf_release_frame(fg, frame_slot, 0);
                break;

            case SXPF_EVENT_I2C_MSG_RECEIVED:
                frame_slot = events[n].data / (1 << 24);
                i2c_hdr    = sxpf_get_frame_ptr(fg, frame_slot);
                i2c_ptr    = (uint8_t*)i2c_hdr + i2c_hdr->payload_offset;

                ts_start =
                    i2c_hdr->ts_start_hi * (1ull<<32) + i2c_hdr->ts_start_lo;

                latency = rx_time - ts_start;

                printf("%+7.3fms - I2C message from port #%d: ts = %llu (%+.2fms):",
                       (rx_time - last_time) / 4e4, i2c_hdr->cam_id,
                       ts_start, latency / 40000.);

                // dump I2C message
                for (unsigned offs = 0; offs < i2c_hdr->payload_size; offs += 2)
                {
                    // we receive 2 bytes per message byte
                    printf(" %x|%02x", i2c_ptr[offs + 1], i2c_ptr[offs + 0]);
                }
                printf("\n");

                // this must be done for each received buffer to ensure the
                // hardware can continue to send images
                sxpf_release_frame(fg, frame_slot, 0);
                break;

            case SXPF_EVENT_CAPTURE_ERROR:
                printf("\ncapture error: 0x%08x\n", events->data);
                continue;

            case SXPF_EVENT_TRIGGER:
                printf("\nTrigger Interrupt received (timestamp "
                       "= 0x%016llx)!\n", events[n].extra.timestamp);
                continue;

            default:                    // log unexpected events
                printf("%+7.3fms(%d/%d) - event type %d: data = 0x%08x\n",
                       (rx_time - last_time) / 4e4, n + 1, len, events[n].type,
                       events[n].data);
                break;
            }

            last_time = rx_time;
        }
    }

    sxpf_stop(fg, SXPF_STREAM_ALL);
    sxpf_close(fg);
}
