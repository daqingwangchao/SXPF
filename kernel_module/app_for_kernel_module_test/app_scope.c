#define _CRT_SECURE_NO_WARNINGS
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <libgen.h>
#include <string.h>
#include <assert.h>
#include <signal.h>
#include "./sxpf_ll/sxpf_instance.h"
#include "./sxpf_ll/lin64/driver_acc.h"


#include "sxpf.h"
#include <time.h>
 
void delay(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;
 
    // Storing start time
    clock_t start_time = clock();
 
    // looping till required time is not achieved
    while (clock() < start_time + milli_seconds)
        ;
}

volatile int done = 0;

int usage(char *progname, int exitcode);

void sigint_handler(int signo)
{
    (void)signo;
    done = 1;
}

int usage(char *progname, int exitcode)
{
    printf("The sxpfscope sample application shows if frame data is received by\n"
           "the grabber card based on the frame valid and line valid signals\n"
           "\n"
           "Usage: %s <device number> <channel> [characters per line]\n"
           "\t <channel> = 0 ... 7\n"
           "\t hit Ctrl-C to close application\n"
           "Output:\n"
           "\t '_' = FrameValid = LOW  and LineValid = LOW"
           "  (inter frame gap)\n"
           "\t '-' = FrameValid = HIGH and LineValid = LOW"
           "  (inter line gap)\n"
           "\t 'x' = FrameValid = HIGH and LineValid = HIGH"
           " (valid data)\n"
           "\t 'o' = FrameValid = LOW  and LineValid = HIGH"
           " (invalid lines received maybe frame sync pulse received "
           "instead of frame valid signal)\n",
           basename(progname));

    exit(exitcode);
}


int main(int argc, char* argv[])
{
    int                 ret = 0;
    int                 device;
    sxpf_hdl            pf;
    char                *endp;
    uint32_t            channel;
    unsigned long       chars_per_line = -1;
    unsigned long       line_len = 0;
    uint32_t            reg;
    int                 temp;

    if (argc < 3)
        exit(usage(argv[0], 1));

    // allow clean shut-down when CTRL-C is pressed
    signal(SIGINT, sigint_handler);

    device = strtol(argv[1], &endp, 0);
    if (endp == argv[1] || *endp)
    {
        fprintf(stderr, "Invalid device number: %s\n", argv[1]);
        usage(argv[0], 2);
    }

    channel = strtoul(argv[2], &endp, 0);




    if ((endp == argv[2] || *endp) || (channel > 7))
    {
        fprintf(stderr, "Invalid channel number: %s\n", argv[2]);
        usage(argv[0], 2);
    }

    if (argc > 3)
    {
        unsigned long cpl = strtoul(argv[3], &endp, 0);
        if (endp == argv[3] || *endp)
        {
            fprintf(stderr, "Invalid number of chars per line: %s\n", argv[3]);
            usage(argv[0], 2);
        }
        chars_per_line = cpl;
    }

    pf = sxpf_open(device);

    /*************this is modify by Zhao ************************/

    temp = DEV_IOCTL(pf->dev, IOCTL_SXPF_CHAMA_WR, (int32_t*)&channel);
    if(!temp) perror("write to sxpf1");
    
    //temp = DEV_IOCTL(pf->dev, IOCTL_SXPF_CHAMA_R, (int32_t*)&channel);
    //if(!temp) perror("read from sxpf1");
    
    printf("channel modified: %d\n", channel);


    /*************this is modify by Zhao ************************/

    if (!pf)
    {
        fprintf(stderr, "sxpf_open(%d) failed: %s", device, strerror(errno));
        return 1;
    }


    if (channel > 3)
    {
        reg = 0x830;//I think this is related to the channel position, channel 0~3 is on the register 0x82c and 4~7 is on the 0x830
    }
    else
    {
        reg = 0x82c;
    }

    while ( !done )//stop by typying "ctrl + c"
    {
        uint32_t    data;

        ret = sxpf_read_register(pf, 0, reg, &data);//if success then 0

        if (ret)
        {
            perror("read register");
        }
        else
        {
            static const char code[4] =
            {
                '_',    // 0 = VSync + HSync low  (inter-frame gap)
                '-',    // 1 = VSync high         (inter-line gap)
                'o',    // 2 = HSync high         (invalid)
                'x'     // 3 = VSync + HSync high (active data)
            };

            data = data >> ((channel % 4) * 8);
            putchar(code[(data >> 1) & 3]);

            if (++line_len >= chars_per_line)
            {
                printf("\n");
                line_len = 0;
            }
        }

        delay(2);
    }

    goto end;

end:
    sxpf_close(pf);

    return ret;
}
