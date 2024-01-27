#define _GNU_SOURCE
#define _CRT_SECURE_NO_WARNINGS
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <libgen.h>
#include <string.h>
#include <assert.h>

#include "sxpf.h"

// ----------------------------------------------------------------------------
// - private configuration
// ----------------------------------------------------------------------------

#define CONFIG_LOW_LEVEL_REGISTER_ACCESS    0

// ----------------------------------------------------------------------------
// - private defines
// ----------------------------------------------------------------------------

#define MAX_BUFFER_SIZE         (1 << 20)

#define LOG_ERR(args, ...)      \
    fprintf(stderr, "\nERROR! " args "\n\n", ##__VA_ARGS__)

#define MAX_IGNORE_PATTERN_SIZE (8)

struct ini_pattern
{
    unsigned            len;
    uint8_t             pattern[MAX_IGNORE_PATTERN_SIZE];
    struct ini_pattern *next;
};


struct ini_pattern *ignored_xfers = NULL, **last_ignored_xfer = &ignored_xfers;


// ----------------------------------------------------------------------------
// - private variables
// ----------------------------------------------------------------------------

static sxpf_hdl g_sxpf = NULL;

// ----------------------------------------------------------------------------
// - private functions
// ----------------------------------------------------------------------------

static uint8_t* readFileInBuffer(const char * filename,
                                 uint32_t* fileSizeInBytes);
static int writeBufferToFile(const char * filename,
                             char* buffer, uint32_t bufferSize);


static int maybe_ignore_i2c_result(int result, sxpf_hdl pf, int port, __u8 dev_id,
                                   const __u8 *wbuf, unsigned int wbytes,
                                   __u8 *rbuf, unsigned int rbytes)
{
    (void)pf; (void)port; (void)rbuf; (void)rbytes;

    if (!result)
        return result;

#define ignore_err() do { printf("--> failure ignored"); return 0; } while (0)

    for (struct ini_pattern *p = ignored_xfers; p; p = p->next)
    {
        if (p->len == 0)
            ignore_err();   // catch-all: ignore all errors

        if (p->pattern[0] != dev_id)
            continue;       // dev_id doesn't match

        if (p->len == 1)
            ignore_err();   // dev_id matches, no pattern data -> ignore error

        if (p->len - 1 > wbytes)
            continue;       // pattern longer than transfer, no match

        if (!memcmp(p->pattern + 1, wbuf, p->len - 1))
            ignore_err();   // transfer matches pattern -> ignore error
    }

    return result;
}


static void dumpUsageAndExit(char *progname, int exitcode)
{
    char* bnp = basename(progname);

    printf("The sxpf-init-sequence sample applications shows how to write INI files\n"
           "to initialize the camera adapters and connected cameras\n"
           "\n"
           "Usage:\n");
    printf("  program eeprom:\n");
    printf("       %s <device> -port <n> -ini <input.ini> --write\n", bnp);
    printf("       %s <device> -port <n> -bin <input.bin> --write\n", bnp);
    printf("  execute directly (don't touch eeprom):\n");
    printf("       %s <device> -port <n> -ini <input.ini> --execute <event-id> [-continue [ignore-spec]]\n", bnp);
    printf("       %s <device> -port <n> -bin <input.bin> --execute <event-id> [-continue [ignore-spec]]\n", bnp);
    printf("  generate binary setup file:\n");
    printf("       %s -ini <input.ini> -bin <output.bin>\n", bnp);
    printf("  dump setup sequence:\n");
    printf("       %s -ini <input.ini> --dump\n", bnp);
    printf("       %s -bin <input.bin> --dump\n", bnp);
    printf("       %s <device> -port <n> --dump\n", bnp);
    printf("  decode and write setup-sequence to new ini-file:\n");
    printf("       %s -ini <input.ini> -out <output.ini>\n", bnp);
    printf("       %s -bin <input.bin> -out <output.ini>\n", bnp);
    printf("\n");
    printf("  ignore-spec is an optional comma-separated sequence of colon-separated byte sequences, e.g.\n"
           "       0x34,0x80:0x01   (ignore errors for device 0x34, and errors for register 1 of device 0x80)\n"
           "       If no ignore-spec is given to -continue, then all I2C errors are ignored.\n");
    printf("\n");

    exit(exitcode);
}


void add_pattern(uint8_t const *data, unsigned len)
{
    struct ini_pattern *p = calloc(1, sizeof(struct ini_pattern));

    if (!p)
    {
        LOG_ERR("out of memory\n");
        exit(1);
    }

    if ((p->len = len))
        memcpy(p->pattern, data, len);

    *last_ignored_xfer = p;
   last_ignored_xfer = &p->next;
}


void set_ignore_patterns(char *prog, char const *arg)
{
    uint8_t     data[MAX_IGNORE_PATTERN_SIZE];
    unsigned    len = 0;
    char       *endp;

    sxpf_set_i2c_result_filter(maybe_ignore_i2c_result);

    while (arg)
    {
        unsigned long v = strtoul(arg, &endp, 0);

        if (endp == arg || v > 255)
            goto syntax_error;  // no value or value to big to for a byte

        data[len++] = (uint8_t)v;

        switch (*endp)
        {
        case 0:     // end of last pattern
            add_pattern(data, len);
            return;

        case ',':   // end of pattern, more patterns follow
            add_pattern(data, len);
            len = 0;
            // fall through

        case ':':   // byte separator within pattern
            arg = endp + 1;
            if (len < MAX_IGNORE_PATTERN_SIZE)
                break;
            // more that 8 data bytes are a syntax error
            // fall through

        default:
syntax_error:
            LOG_ERR("Invalid -continue argument");
            dumpUsageAndExit(prog, 1);
        }
    }

    printf("Note: ignoring all I2C errors.\n");
    add_pattern(NULL, 0);
}


// ----------------------------------------------------------------------------
// - main entry function
// ----------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    #define MAX_TMP_SIZE (1*1024*1024)

    int              i, ret = EXIT_SUCCESS, doDump = 0, doWrite = 0, doExecute = 0;
    char            *executeEventIds = NULL;
    const char      *iniFileName = NULL;
    const char      *binFileName = NULL;
    const char      *outFileName = NULL;
    const char      *codeFileName = NULL;
    int              devNumber = -1;
    uint8_t         *data = NULL;
    char            *tmp = NULL;
    uint32_t         dataSize = 0;
    int              tmpSize = 0;
    uint32_t         port = 0;
    sxpf_ini_props_t iniProps;

    printf("\n");

    for (i = 1; i < argc; i++) {

        char* arg = argv[i];

        if (0 == strcmp("--dump", arg)) {

            doDump = 1;

        } else if (0 == strcmp("--write", arg)) {

            doWrite = 1;

        } else if (0 == strcmp("--execute", arg)) {

            doExecute = 1;

            if (++i >= argc) {
                printf("WANRING! missing event-id-number, use '0x0' by default\n\n");
            } else {
                executeEventIds = argv[i];
            }

        } else if (0 == strcmp("-port", arg)) {

            if (++i >= argc) {
                LOG_ERR("missing port-number");
                dumpUsageAndExit(argv[0], 1);
            }

            port = atoi(argv[i]);

        } else if (0 == strcmp("-ini", arg)) {

            if (++i >= argc) {
                LOG_ERR("missing ini-filename");
                dumpUsageAndExit(argv[0], 1);
            }

            iniFileName = argv[i];

        } else if (0 == strcmp("-bin", arg)) {

            if (++i >= argc) {
                LOG_ERR("missing bin-filename");
                dumpUsageAndExit(argv[0], 1);
            }

            binFileName = argv[i];

        }
        else if (0 == strcmp(arg, "-continue"))
        {
            if (i + 1 == argc || argv[i + 1][0] == '-')
                set_ignore_patterns(argv[0], NULL);
            else
                set_ignore_patterns(argv[0], argv[++i]);
        }
        else if (0 == strcmp(arg, "-out")) {

            if (++i >= argc) {
                LOG_ERR("missing out-filename");
                dumpUsageAndExit(argv[0], 1);
            }

            outFileName = argv[i];

        } else if (0 == strcmp(arg, "-code")) { // inofficial argument!

            if (++i >= argc) {
                LOG_ERR("missing code-filename");
                dumpUsageAndExit(argv[0], 1);

            }

            codeFileName = argv[i];

        } else if (0 == strncmp("/dev/sxpf", arg, 9)) {

            if (10 != strlen(arg)) {
                LOG_ERR("Invalid device name number");
                dumpUsageAndExit(argv[0], 1);
            }

            devNumber = arg[9] - '0';

        } else {
            LOG_ERR("unknown argument: '%s'", arg);
            dumpUsageAndExit(argv[0], 0);
        }
    }

    if (!iniFileName && devNumber<0 && !binFileName && !outFileName) {
        dumpUsageAndExit(argv[0], 0);
    }

    printf("parsed arguments:\n");
    if (iniFileName) printf(" ini  = %s\n", iniFileName);
    if (devNumber>=0) printf(" dev  = %d, port %d\n", devNumber, port);
    if (binFileName) printf(" bin  = %s\n", binFileName);
    if (outFileName) printf(" out  = %s\n", outFileName);
    if (codeFileName) printf(" code = %s\n", outFileName);
    if (doExecute) printf("\nexecute event-ids: %s\n", executeEventIds);
    printf("\n");

    if (devNumber>=0) {
        g_sxpf = sxpf_open(devNumber); //filediscriptor 
        if (NULL == g_sxpf) {
            LOG_ERR("open device %d failed", devNumber);
            ret = 1;
            goto exitApp;
        }
    }

    if (iniFileName) {

        data = sxpf_load_init_sequence(iniFileName, &dataSize);// from here get the dataSize

    } else if (binFileName) {

        data = readFileInBuffer(binFileName, &dataSize); // for the BinaryFile

    } else if (g_sxpf && !doWrite) {

        if (sxpf_eeprom_getSize(g_sxpf, port, &dataSize)) {
            LOG_ERR("can't read eeprom size");
            ret = 5;
            goto exitApp;
        }
        printf("size of eeprom: %d bytes\n", dataSize);

        data = malloc(dataSize);
        if (data) {

            printf("read %d bytes from eeprom\n", dataSize);

            if (sxpf_eeprom_readBytes(g_sxpf, port, 0x0, data, dataSize)) {
                LOG_ERR("Can't read bytes from EEPROM");
                ret = 5;
                goto exitApp;
            }
        }

        printf("\n");

    }

    if (data) {

        printf("decode binary data   : ");
        fflush(stdout);

        tmp = malloc(MAX_TMP_SIZE);
        if (!tmp) {
            LOG_ERR("Out of memory during allocating memory for temporary "
                    "decoding data!");
            ret = 5;
            goto exitApp;
        }
        if ((tmpSize = sxpf_decode_init_sequence(data, dataSize,
                                                 tmp, MAX_TMP_SIZE)) < 0) {
            ret = 5;
            goto exitApp;
        }

        printf("ok\n");

        if (doDump && !iniFileName) {
            printf("%s", tmp);
        }
    }

    printf("\n");

    if (doWrite) {

        if (!g_sxpf) {
            LOG_ERR("No device given! can't write data to EEPROM!");
            ret = 6;
            goto exitApp;
        }

        if (sxpf_analyze_init_sequence(data, dataSize, &iniProps) < 0 ||
            iniProps.eeprom_compatible != 1) {
            LOG_ERR("INI is not compatible for EEPROM!");
            ret = 6;
            goto exitApp;
        }

        if (data) {

            uint32_t eepromSize;
            if (sxpf_eeprom_getSize(g_sxpf, port, &eepromSize)) {
                LOG_ERR("can't read eeprom size");
                ret = 5;
                goto exitApp;
            }
            printf("size of eeprom: %d bytes\n", eepromSize);

            if (dataSize<=eepromSize) {

                printf("Write %d bytes to EEPROM...\n", dataSize);

                if (sxpf_eeprom_writeBytes(g_sxpf, port, 0x0, data, dataSize)) {
                    LOG_ERR("Can't write bytes to EEPROM");
                    ret = 5;
                    goto exitApp;
                }

            } else {

                LOG_ERR("Size of binary data (%d bytes) not fit in EEPROM (size=%d)!", dataSize, eepromSize);
                ret = 6;
                goto exitApp;
            }

        } else {
            LOG_ERR("No input init sequence given! can't write data to EEPROM!");
            ret = 6;
            goto exitApp;
        }

    }

    if (doExecute)
    {
        char delim[] = ",";
        char *ptr = strtok(executeEventIds, delim); //This function returns a pointer to the first token found in the string. A null pointer is returned if there are no tokens left to retrieve.
        while(ptr != NULL)
        {
            ret = sxpf_send_init_sequence(g_sxpf, data, dataSize, port, atoi(ptr));
            if (ret != 0)
            {
                goto exitApp;
            }
            printf("\n");
            ptr = strtok(NULL, delim);
        }
    }

    if ((iniFileName || (g_sxpf && !doWrite)) && binFileName && data) {

        if (sxpf_analyze_init_sequence(data, dataSize, &iniProps) < 0 ||
            iniProps.eeprom_compatible != 1 ||
            iniProps.spi_compatible != 1) {
            LOG_ERR("INI is not compatible for EEPROM or SPI!");
            ret = 6;
            goto exitApp;
        }

        printf("Write binary data to file '%s'\n", binFileName);

        if (writeBufferToFile(binFileName, (char*)data, dataSize) < 0) {
            ret = 4;
            goto exitApp;
        }
    }

    if (outFileName && tmp) {

        printf("write decoded binary data to file '%s'\n", outFileName);

        if (writeBufferToFile(outFileName, tmp, tmpSize) < 0) {
            ret = 5;
            goto exitApp;
        }
    }

    if (codeFileName && data) {

        printf("write c-code binary blob to file '%s'\n", codeFileName);

        uint32_t i;
        FILE * out;

        // open out for write
        out = fopen(codeFileName, "w");
        if (!out) {
            fprintf(stderr, "Can't open out '%s' for writing (%s)!\n",
                    codeFileName, strerror(errno));
            goto exitApp;
        }

        // write c-code snippet
        fprintf(out, "/**\n");
        fprintf(out, " * binary init sequence blob\n");
        fprintf(out, " */\n");
        fprintf(out, "static const uint8_t initsequence[%d] = {", dataSize);
        for (i = 0; i < dataSize; i++) {
            if (0 == i % 8) { fprintf(out, "\n         "); }
            fprintf(out, "0x%02x, ", data[i]);
        }
        fprintf(out, "\n    };\n");

        // close out
        int status = fclose(out);
        if (status != 0) {
            fprintf(stderr, "Closing of out '%s' failed (%s)!\n", codeFileName,
                    strerror(errno));
            return -3;
        }
    }

    exitApp:

    if (data) { free(data); }
    if (tmp) { free(tmp); }

    if (g_sxpf) { sxpf_close(g_sxpf); }

    return ret;
}


uint8_t* readFileInBuffer(const char * filename, uint32_t* fileSizeInBytes)
{
    off_t rd;
    struct stat sb;
    uint8_t *buffer;

    // open file for read
    FILE * file = fopen(filename, "rb");
    if (!file) {
        //LOG_ERR("Can't open file '%s' for reading (%s)!", filename,
        //        strerror(errno));
        return NULL;
    }

    // get filesize
    if (stat(filename, &sb) != 0) {
        LOG_ERR("Can't determine size for file '%s' (%s)!", filename,
                strerror(errno));
        return NULL;
    }
    *fileSizeInBytes = sb.st_size;

    // protect buffer for overflow
    if (sb.st_size > MAX_BUFFER_SIZE) {
        LOG_ERR("Size of file '%s' exceeded the buffer-size of %d " "bytes. "
                "(file-size=%d)!", filename, MAX_BUFFER_SIZE, (int )sb.st_size);
        return NULL;
    }

    buffer = (uint8_t*)malloc(sb.st_size + 1);
    if (!buffer) {
        LOG_ERR("Buffer allocation of %d bytes failed!", (int )(sb.st_size + 1));
        return NULL;
    }

    // read content into buffer
    rd = (off_t)fread(buffer, sizeof(uint8_t), sb.st_size, file);
    if (rd != sb.st_size) {
        LOG_ERR("Read content of file '%s' failed! Expected %d bytes," " got %d "
                "bytes.", filename, (int )sb.st_size, (int )rd);
        free(buffer);
        return NULL;
    }

    // close file
    int status = fclose(file);
    if (status != 0) {
        LOG_ERR("Closing of file '%s' failed (%s)!", filename, strerror(errno));
        free(buffer);
        return NULL;
    }

    // mark end of content with '\0'
    buffer[sb.st_size] = '\0';

    // all done
    printf("%d bytes read\n", (int) sb.st_size);

    // all done
    return buffer;
}


int writeBufferToFile(const char * filename, char* buffer, uint32_t bufferSize)
{
    size_t wr;
    FILE * file;

    // open file for write
    file = fopen(filename, "wb");
    if (!file) {
        fprintf(stderr, "Can't open file '%s' for writing (%s)!\n", filename,
                strerror(errno));
        return -1;
    }

    // write buffer-content to file
    wr = fwrite(buffer, sizeof(uint8_t), bufferSize, file);
    if (wr != bufferSize) {
        fprintf(stderr, "Write content to file '%s' failed! Expected %d bytes,"
                " wrote %d bytes.\n", filename, (int) bufferSize, (int) wr);
        return -2;
    }

    // close file
    int status = fclose(file);
    if (status != 0) {
        fprintf(stderr, "Closing of file '%s' failed (%s)!\n", filename,
                strerror(errno));
        return -3;
    }

    return 0;
}
